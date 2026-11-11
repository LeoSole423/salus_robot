# Person watch — 2026-11-11

Operator requests lightweight monitoring of all three operational cameras and
person alerts in Cockpit, preserving navigation. This is a new advisory feature;
it does not replace obstacle detection, brake or publish motion commands.

Observed host: Orin Nano 8 GB, L4T R39.2, Python 3.12, about 5 GiB available;
CPU already busy with ROS. TensorRT/DeepStream not installed; NVIDIA decoder
inspection reports CUDA status 100. Do not upgrade GPU drivers to enable alerts.
Front/mobile ONVIF snapshots unavailable; rear snapshot available. Reuse local
MediaMTX saving streams, never camera credentials in UI or source control.

Evaluate official YOLOX Nano ONNX (Apache-2.0 project), single-thread OpenCV DNN
CPU, sequential inference at low rate, dedicated low-priority systemd service.
Bound decode processes, latest-frame slots and age checks prevent backlog.
Confirm consecutive positives and cooldown per camera. Distinguish unavailable,
paused for resource pressure, fresh negative and confirmed detection. Expose
read-only metadata on private Tailscale interface for Cockpit polling. No images,
recognition of identity, external notifications, ROS services or navigation restarts.

Hardware acceptance must report observed inference latency, CPU/RAM, live
camera reception, service continuity and detection limitations. Positive field
validation with the operator remains distinct from synthetic decoder tests.

Final choice after the operator prioritised GPU: repair the incorrect boot FDT
under an explicitly authorised maintenance/reboot window; use NCNN Vulkan and
hardware GStreamer decoding. TensorRT download was interrupted before package
installation because its 2.3 GB builder/runtime package is unnecessary here.
The 5 MB official NCNN wheel recognises the Orin GPU. Model conversion and
positive/negative fixtures validate the GPU path; field acceptance stays pending.
