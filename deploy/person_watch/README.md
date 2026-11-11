# Advisory person watch

Independent host service for person alerts in Cockpit. Owns no ROS topic, TF,
actuator, navigation parameter or operator lease. Runs even with Cockpit closed;
only metadata is polled by clients. This is an advisory detector, not a certified
safety sensor or an automatic stop mechanism.

## Architecture

Local MediaMTX saving paths cam3/cam4/cam5 → GStreamer RTSP/TCP H264 hardware
decoder → one latest 640×360 BGR slot per camera → sequential YOLOX Nano
416×416 NCNN Vulkan inference → confirmation/cooldown → read-only HTTP state.
No video is encoded, saved or uploaded by this service. Existing recording and
live quality selection are independent. Monitoring holds all three saving paths
open, including the mobile camera; this adds local camera LAN traffic.

The host must have a working NVIDIA device/driver, NCNN Vulkan Python
bindings, GStreamer Python bindings and NVIDIA decode plugins.
CPU inference and software decode are explicit diagnostic backends only; a GPU
failure never silently switches a deployed GPU configuration to CPU.

Default sampling: 0.5 FPS per camera (1.5 total); two consecutive positive
samples, confidence ≥0.55 and 30 s cooldown per camera. Approximate confirmation
latency is 2–4 seconds plus transport/processing, not guaranteed response time.
No sample or a negative resets confirmation. A gap over 6 s resets it too.
Disconnects do not erase cooldown. No observations are queued. Small, distant,
partly hidden and poorly lit people can be missed; accept with actual patrol
scenes before relying on coverage.

## Model and environment

Official release:
https://github.com/Megvii-BaseDetection/YOLOX/releases/tag/0.1.1rc0

Download `yolox_nano.onnx` from that release. Required SHA-256:
`c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d`.
YOLOX project license: Apache-2.0. Dependencies are not vendored.
The implementation verifies the ONNX digest at startup. Model preprocessing
follows the official ONNX example: BGR float32 0..255, top-left letterbox with
114, CHW; COCO class 0; decode stride 8/16/32; NMS IoU 0.45.

Create an isolated Python 3 environment with `--system-site-packages` so host
GStreamer bindings remain accessible, then install `requirements.txt`. No CUDA
SDK, TensorRT, PyTorch or DeepStream is needed on the robot. NCNN 1.0.20261020
wheel is about 5 MB; converted weights are about 3.5 MB.

Convert the pinned ONNX once on a development machine using an isolated tool
environment (`onnx==1.19.1`, `numpy==2.2.6`, `pnnx==20261020`). Install the pnnx
wheel with `--no-deps` and invoke its bundled native `pnnx/pnnx` executable;
its Python wrapper otherwise pulls PyTorch, which is unnecessary for ONNX.

```bash
python3 convert_model.py --source yolox_nano.onnx --output salus_folded.onnx --pnnx /path/to/native/pnnx
```

The pinned model starts with Focus slices unsupported by this converter.
`convert_model.py` folds those slices plus the first 3×3 convolution into an
equivalent 6×6 stride-2 convolution. It checks the exact source checksum/graph.
The resulting NCNN param and weights checksums are pinned in `ncnn_backend.py`;
regeneration with different tools must be reviewed and fixture-tested before
updating these checksums. Do not copy generated Python wrappers to the robot.
Only the original ONNX (provenance check) and the two converted files are needed.
GPU inference uses FP32 arithmetic/storage and FP32 weights; explicit
contiguous CHW buffers are required by the NCNN Python binding. No TensorRT
engine is used. The supplied service caps
aggregate CPU (including children) at half one core, uses Nice 15/CPUWeight 10,
and MemoryHigh 512 MiB / MemoryMax 768 MiB. GPU work has no hard utilization
quota; sampling limits work, and telemetry must verify interference. Memory
under 700 MiB available or any thermal zone ≥82°C pauses inference. Hardware
decoding remains active; no movement or navigation cancellation is requested.

## Configuration

Copy `config.example.json` and adapt paths, private bind IP and allowed origins.
`ffmpeg` and `ffmpeg_library_path` are required only for software decode trials.

| Key | Type / default | Range / meaning |
| --- | --- | --- |
| backend | string / ncnn_vulkan | ncnn_vulkan or explicit opencv_cpu diagnostic |
| capture | string / gstreamer | gstreamer or explicit ffmpeg diagnostic |
| model | path / required | exact pinned ONNX file |
| param / weights | paths / required for NCNN | pinned converted param/bin files |
| fps_per_camera | number / 0.5 FPS | 0.5–2; hardware rounds decoder interval assuming 15 FPS input |
| confidence | number / 0.55 | 0.2–0.95; probability score threshold |
| bind | string / 127.0.0.1 | loopback or private Tailscale IP, never public wildcard |
| port | integer / 8791 | read-only metadata listener |
| allowed_origins | strings / empty | exact browser origins, no wildcard |
| cameras | three objects / required | fixed cam3,cam4,cam5; label and credential-free loopback saving RTSP URL |

Install the service template with the actual user/paths and enable/start just
`salus-person-watch.service`. It has no dependency that restarts ROS. Stop or
remove this service to disable monitoring; do not stop MediaMTX or ROS.
No ROS bringup changes or new ROS public interfaces are introduced.

## HTTP / Cockpit contract

`GET /v1/state`, schema 1, `Cache-Control: no-store`. Producer: person-watch;
consumer: Cockpit person-watch package module; every 2 seconds, 3 s HTTP timeout,
one in-flight request. Service input URLs never appear in output.

- `boot_id`, `server_ms`, `started_ms`: detector instance identity / epoch ms.
- `cameras`: id, label, status (`starting`, `watching`, `unavailable`,
  `resource_paused`), `sample_ms` epoch ms, normalized xywh `bbox` detections and
  confidence, latest `inference_ms`.
- `alerts`: bounded 50-entry process-memory history; unique id, camera_id,
  label, timestamp_ms, count, confidence. Restart clears history/cooldowns.

HTTP is intentionally read-only and carries no images, secrets or commands.
Network access is restricted by binding to Tailscale; CORS is an additional
browser-origin restriction, not authentication. Tailnet members with reachability
can read this metadata. For HTTPS Cockpit deployment configure a same-origin
HTTPS reverse proxy and set `person_watch_url`; browsers can block an HTTP
endpoint from an HTTPS page. Never disable browser security to work around this.
Simulation/other-host connections do not query the configured real detector.
Stale frames >5 s and network failures show unavailable rather than zero people.

## Verification / recovery

Run `python3 -m unittest tools/test_person_watch.py`, build/tests and the web smoke.
Verify the pinned model on a positive/negative fixture and live frames. Compare
live CPU/RAM/GPU and navigation freshness with service stopped/running. Confirm
hardware decoding works for each camera, loss/reconnect, alerts and cooldown.
Record limitations separately from completed checks. CPU-only benchmark is not
proof of GPU execution or physical person detection.

Rollback: stop/disable only this service and remove its Cockpit URL/module if
needed. Keep model and isolated Python environment for
review; package removal must be reviewed separately. GPU boot repair and its
backup are a separate maintenance action, documented in deployment evidence.
