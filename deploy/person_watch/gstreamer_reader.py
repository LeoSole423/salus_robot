"""Hardware H264 decode, bounded appsink and one latest BGR frame per camera."""
import threading
import time


class HardwareReader:
    def __init__(self, config, camera, stop):
        self.config, self.camera, self.stop = config, camera, stop
        self.lock = threading.Lock()
        self.latest = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        Gst.init(None)
        interval = min(30, max(1, round(15/self.config.get('fps_per_camera', .5))))
        # Source URLs have already been restricted to fixed loopback paths.
        description = (
            f'rtspsrc location={self.camera["url"]} protocols=tcp latency=150 drop-on-latency=true '
            '! application/x-rtp,media=video,encoding-name=H264 '
            '! rtph264depay ! h264parse '
            f'! nvv4l2decoder drop-frame-interval={interval} num-extra-surfaces=1 '
            '! nvvidconv ! video/x-raw,format=BGRx,width=640,height=360 '
            '! videoconvert n-threads=1 ! video/x-raw,format=BGR '
            '! appsink name=frames max-buffers=1 drop=true sync=false enable-last-sample=false'
        )
        while not self.stop.is_set():
            pipeline = None
            try:
                pipeline = Gst.parse_launch(description)
                sink, bus = pipeline.get_by_name('frames'), pipeline.get_bus()
                if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError('Hardware decoder unavailable')
                last_sample = time.monotonic()
                while not self.stop.is_set():
                    if bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS):
                        break
                    sample = sink.emit('try-pull-sample', Gst.SECOND)
                    if sample is None:
                        if time.monotonic()-last_sample > 8:
                            break
                        continue
                    buffer = sample.get_buffer()
                    ok, mapped = buffer.map(Gst.MapFlags.READ)
                    if ok:
                        try:
                            data = bytes(mapped.data)
                            if len(data) == 640*360*3:
                                last_sample = time.monotonic()
                                with self.lock:
                                    self.latest = (last_sample, data)
                        finally:
                            buffer.unmap(mapped)
            except Exception:
                pass  # Bounded retry; public status reports unavailable, never negative.
            finally:
                if pipeline:
                    pipeline.set_state(Gst.State.NULL)
                with self.lock:
                    self.latest = None
            self.stop.wait(3)

    def take(self):
        with self.lock:
            value, self.latest = self.latest, None
            return value
