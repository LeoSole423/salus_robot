#!/usr/bin/env python3
"""Advisory person detection; deliberately independent of ROS and motion control."""
import argparse
from collections import deque
from dataclasses import dataclass
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import threading
import time
import uuid

MODEL_SHA256 = 'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d'
WIDTH, HEIGHT = 640, 360


@dataclass
class Confirmation:
    hits: int = 0
    last_sample: float = -math.inf
    last_alert: float = -math.inf

    def update(self, now, positive, *, fresh=True, required=2, max_gap=6., cooldown=30.):
        """Only consecutive fresh observations confirm; cooldown survives disconnects."""
        if not fresh or not positive or now - self.last_sample > max_gap:
            self.hits = 0
        self.last_sample = now
        if fresh and positive:
            self.hits += 1
        if self.hits >= required and now - self.last_alert >= cooldown:
            self.last_alert = now
            return True
        return False


def validate_config(config):
    from urllib.parse import urlsplit
    if not 0.5 <= config.get('fps_per_camera', .5) <= 2:
        raise ValueError('fps_per_camera must be 0.5..2')
    import ipaddress
    bind = ipaddress.ip_address(config.get('bind', '127.0.0.1'))
    if not (bind.is_loopback or bind in ipaddress.ip_network('100.64.0.0/10')):
        raise ValueError('bind must be loopback or Tailscale')
    if type(config.get('port', 8791)) is not int or not 1 <= config.get('port', 8791) <= 65535:
        raise ValueError('invalid metadata port')
    cameras = config['cameras']
    if len(cameras) != 3 or {c['id'] for c in cameras} != {'cam3', 'cam4', 'cam5'}:
        raise ValueError('exactly cam3, cam4, cam5 required')
    for camera in cameras:
        uri = urlsplit(camera['url'])
        if uri.scheme != 'rtsp' or uri.hostname != '127.0.0.1' or uri.username or uri.password:
            raise ValueError('only credential-free loopback RTSP sources allowed')
        if uri.path != '/' + camera['id'] or uri.query or uri.fragment or any(c.isspace() for c in camera['url']):
            raise ValueError('only saving paths allowed')
    if not 0.2 <= config.get('confidence', .55) <= .95:
        raise ValueError('confidence must be 0.2..0.95')
    return config


def decoder_command(config, camera):
    return [config['ffmpeg'], '-nostdin', '-hide_banner', '-loglevel', 'quiet',
            '-threads', '1', '-rtsp_transport', 'tcp', '-allowed_media_types', 'video',
            '-fflags', 'nobuffer', '-analyzeduration', '500000', '-probesize', '524288',
            '-i', camera['url'], '-an', '-sn', '-dn', '-filter_threads', '1',
            '-vf', f"fps={config.get('fps_per_camera', .5)},scale={WIDTH}:{HEIGHT}",
            '-pix_fmt', 'bgr24', '-f', 'rawvideo', 'pipe:1']


class Reader:
    """Continuously drain the pipe; overwrite one slot, never queue video."""
    def __init__(self, config, camera, stop):
        self.config, self.camera, self.stop = config, camera, stop
        self.lock = threading.Lock()
        self.latest = None
        self.process = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        env = dict(os.environ)
        if self.config.get('ffmpeg_library_path'):
            env['LD_LIBRARY_PATH'] = self.config['ffmpeg_library_path']
        size = WIDTH * HEIGHT * 3
        while not self.stop.is_set():
            try:
                self.process = subprocess.Popen(decoder_command(self.config, self.camera),
                                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                                stdin=subprocess.DEVNULL, env=env)
                data = bytearray()
                while not self.stop.is_set():
                    if not select.select([self.process.stdout], [], [], 6)[0]:
                        break
                    chunk = os.read(self.process.stdout.fileno(), min(65536, size-len(data)))
                    if not chunk:
                        break
                    data.extend(chunk)
                    if len(data) == size:
                        with self.lock:
                            self.latest = (time.monotonic(), bytes(data))
                        data.clear()
            except OSError:
                pass  # No URLs/credentials in journal.
            finally:
                if self.process:
                    if self.process.poll() is None:
                        self.process.kill()
                    self.process.wait()
                    if self.process.stdout:
                        self.process.stdout.close()
                with self.lock:
                    self.latest = None
            self.stop.wait(3)

    def take(self):
        with self.lock:
            value, self.latest = self.latest, None
            return value


class Detector:
    def __init__(self, model, threshold, *, backend="ncnn_vulkan", param=None, weights=None):
        import cv2
        import numpy as np
        self.cv, self.np, self.threshold = cv2, np, threshold
        if hashlib.sha256(Path(model).read_bytes()).hexdigest() != MODEL_SHA256:
            raise ValueError('model checksum mismatch')
        cv2.setNumThreads(1)
        self.runner = None
        if backend == 'ncnn_vulkan':
            from ncnn_backend import NcnnRunner
            self.runner = NcnnRunner(param, weights)
        elif backend == 'opencv_cpu':
            self.net = cv2.dnn.readNetFromONNX(model)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        else:
            raise ValueError('Unsupported detector backend')
        grids, strides = [], []
        for stride in (8, 16, 32):
            y, x = np.meshgrid(np.arange(416//stride), np.arange(416//stride), indexing='ij')
            grids.append(np.stack([x, y], axis=-1).reshape(-1, 2))
            strides.append(np.full((x.size, 1), stride))
        self.grid, self.strides = np.concatenate(grids), np.concatenate(strides)

    def detect(self, data):
        cv, np = self.cv, self.np
        image = np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
        ratio = 416 / WIDTH
        canvas = np.full((416, 416, 3), 114, dtype=np.uint8)
        canvas[:int(HEIGHT*ratio), :416] = cv.resize(image, (416, int(HEIGHT*ratio)))
        # Official YOLOX preprocessing: BGR, 0..255, top-left letterbox, CHW.
        blob = canvas.transpose(2, 0, 1)[None].astype(np.float32)
        if self.runner:
            pred = self.runner.forward(blob)[0]
        else:
            self.net.setInput(blob)
            pred = self.net.forward()[0]
        scores = pred[:, 4] * pred[:, 5]  # COCO class 0: person, not VOC class 15.
        chosen = np.flatnonzero(np.isfinite(scores) & (scores >= self.threshold))
        boxes, confidences = [], []
        for index in chosen:
            p = pred[index]
            center = (p[:2] + self.grid[index]) * self.strides[index] / ratio
            wh = np.exp(np.clip(p[2:4], -12, 12)) * self.strides[index] / ratio
            x1, y1 = np.maximum(center - wh/2, 0)
            x2, y2 = np.minimum(center + wh/2, [WIDTH, HEIGHT])
            if not np.all(np.isfinite([x1,y1,x2,y2])) or x2 <= x1 or y2 <= y1:
                continue
            boxes.append([float(x1), float(y1), float(x2-x1), float(y2-y1)])
            confidences.append(float(scores[index]))
        indices = cv.dnn.NMSBoxes(boxes, confidences, self.threshold, .45)
        return [{'confidence': round(confidences[int(i)], 4),
                 'bbox': [round(v/d, 4) for v,d in zip(boxes[int(i)], [WIDTH, HEIGHT, WIDTH, HEIGHT])]}
                for i in np.asarray(indices).reshape(-1)]


def under_pressure():
    available = next(int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()
                     if line.startswith('MemAvailable:'))
    if available < 700 * 1024:
        return True
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            if int(path.read_text()) >= 82000:
                return True
        except (OSError, ValueError):
            continue
    return False


MAX_SNAPSHOT_BYTES = 256 * 1024


def annotate_snapshot(data, detections):
    """Encode only the triggering frame, never take another camera sample."""
    import cv2
    import numpy as np
    image = np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
    for detection in detections:
        x, y, w, h = detection['bbox']
        left, top = max(0, min(WIDTH-1, round(x*WIDTH))), max(0, min(HEIGHT-1, round(y*HEIGHT)))
        right, bottom = max(left, min(WIDTH-1, round((x+w)*WIDTH))), max(top, min(HEIGHT-1, round((y+h)*HEIGHT)))
        cv2.rectangle(image, (left, top), (right, bottom), (0, 220, 255), 2)
        caption = 'Person %.0f%%' % (detection['confidence']*100)
        cv2.rectangle(image, (left, max(0, top-22)), (min(WIDTH-1,left+130), max(22,top)), (0,0,0), -1)
        cv2.putText(image, caption, (left+3, max(16, top-6)), cv2.FONT_HERSHEY_SIMPLEX, .45, (0,220,255), 1, cv2.LINE_AA)
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok or jpeg.nbytes > MAX_SNAPSHOT_BYTES:
        return None
    return jpeg.tobytes()


class State:
    def __init__(self, config):
        self.lock = threading.Lock()
        self.boot_id = str(uuid.uuid4())
        self.started = time.time()
        self.alerts = deque(maxlen=50)
        self.images = {}
        self.cameras = {c['id']: {'id': c['id'], 'label': c['label'], 'status': 'starting',
                                'sample_ms': 0, 'detections': [], 'inference_ms': 0}
                        for c in config['cameras']}

    def append_alert(self, alert, jpeg):
        """Caller owns lock. Image lifetime is exactly the bounded alert lifetime."""
        if len(self.alerts) == self.alerts.maxlen:
            self.images.pop(self.alerts[0]['id'], None)
        available = bool(jpeg) and len(jpeg) <= MAX_SNAPSHOT_BYTES
        self.alerts.append(dict(alert, image_available=available))
        if available:
            self.images[alert['id']] = jpeg

    def snapshot(self):
        with self.lock:
            return {'schema': 1, 'boot_id': self.boot_id, 'server_ms': round(time.time()*1000),
                    'started_ms': round(self.started*1000),
                    'cameras': [dict(c, status='unavailable', detections=[])
                                if c['status'] == 'watching' and time.time()*1000-c['sample_ms'] > 5000
                                else dict(c) for c in self.cameras.values()],
                    'alerts': list(self.alerts)}


def start_http(config, state):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            origin = self.headers.get('Origin')
            if origin and origin not in config.get('allowed_origins', []):
                self.send_error(403)
                return
            if self.path == '/v1/state':
                body = json.dumps(state.snapshot(), allow_nan=False).encode()
                content_type = 'application/json'
            elif self.path.startswith('/v1/alerts/') and self.path.endswith('.jpg'):
                identifier = self.path[len('/v1/alerts/'):-4]
                try:
                    if str(uuid.UUID(identifier)) != identifier:
                        raise ValueError('Invalid alert ID')
                except ValueError:
                    self.send_error(404)
                    return
                with state.lock:
                    body = state.images.get(identifier)
                if body is None:
                    self.send_error(404)
                    return
                content_type = 'image/jpeg'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            if origin:
                self.send_header('Access-Control-Allow-Origin', origin)
                self.send_header('Vary', 'Origin')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Type', content_type)
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass
    server = ThreadingHTTPServer((config.get('bind', '127.0.0.1'), config.get('port', 8791)), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def run(config):
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    state = State(config)
    detector = Detector(config['model'], config.get('confidence', .55),
                        backend=config.get('backend', 'ncnn_vulkan'),
                        param=config.get('param'), weights=config.get('weights'))
    if config.get('capture', 'gstreamer') == 'gstreamer':
        from gstreamer_reader import HardwareReader
        reader_class = HardwareReader
    elif config['capture'] == 'ffmpeg':
        reader_class = Reader
    else:
        raise ValueError('Unsupported capture backend')
    readers = [reader_class(config, camera, stop) for camera in config['cameras']]
    policies = {c['id']: Confirmation() for c in config['cameras']}
    server = start_http(config, state)
    for reader in readers:
        reader.thread.start()
    try:
        while not stop.wait(.1):
            paused = under_pressure()
            for reader in readers:
                camera_id = reader.camera['id']
                value = reader.take()
                now = time.monotonic()
                with state.lock:
                    previous = state.cameras[camera_id]
                    expired = time.time()*1000 - previous['sample_ms'] > 5000
                    if paused or expired:
                        previous.update(status='resource_paused' if paused else 'unavailable', detections=[])
                if paused or value is None or now - value[0] > 2:
                    if paused or expired or (value is not None and now-value[0] > 2):
                        policies[camera_id].update(now, False, fresh=False)
                    continue
                detections = detector.detect(value[1])
                duration = time.monotonic() - now
                fresh = time.monotonic() - value[0] <= 2
                alert = policies[camera_id].update(time.monotonic(), bool(detections), fresh=fresh)
                jpeg = None
                if alert:
                    try:
                        jpeg = annotate_snapshot(value[1], detections)
                    except Exception:
                        pass  # Preserve the detection even if evidence encoding fails.
                with state.lock:
                    state.cameras[camera_id].update(status='watching' if fresh else 'unavailable',
                        sample_ms=round((time.time() - (time.monotonic()-value[0]))*1000),
                        detections=detections if fresh else [], inference_ms=round(duration*1000, 1))
                    if alert:
                        state.append_alert({'id': str(uuid.uuid4()), 'camera_id': camera_id,
                            'label': reader.camera['label'], 'timestamp_ms': round(time.time()*1000),
                            'count': len(detections), 'confidence': max(d['confidence'] for d in detections)}, jpeg)
    finally:
        stop.set()
        for reader in readers:
            if getattr(reader, 'process', None) and reader.process.poll() is None:
                reader.process.kill()
            reader.thread.join(timeout=8)
        server.shutdown()
        server.server_close()
        if detector.runner:
            detector.runner.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    run(validate_config(json.loads(Path(args.config).read_text())))
