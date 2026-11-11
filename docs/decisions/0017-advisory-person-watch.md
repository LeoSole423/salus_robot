# ADR 0017 — Independent advisory person watch

Date: 2026-11-11. Status: accepted for implementation, field acceptance pending.

The operator requests lightweight person alerts from three operational cameras,
visible in Cockpit, while preserving navigation CPU capacity. Existing UI vision
parsers are presentation scaffolding and do not establish an operational detector.

Use an independent host service consuming local saving RTSP paths. Hardware
H264 decoding and NCNN Vulkan run on Jetson; one latest frame slot, sequential
low-frequency inference, two-hit confirmation and per-camera cooldown bound
work and noise. Expose read-only private-network metadata directly to a modular
Cockpit contribution. Do not restart or expand the deployed ROS composition to
add this optional feature. Shared ROS contracts remain unchanged.

Orin Nano lacks DLA; do not promise independent inference hardware or hard GPU
quotas. CPU/memory are constrained by systemd. Backpressure drops observations,
never queues old images. Failure is visible as unavailable, not absence of
people. No commands to navigation, safety, PTZ or recording are emitted.

Rejected initially: continuous full-FPS detection, video re-encoding, general
language/vision models, complete DeepStream deployment and automatic CPU fallback.
They add unnecessary runtime/dependency cost for low-rate advisory alerts.
CPU backend remains an explicit diagnostic tool, not the deployment default.

Consequences: approximate 2–4 s confirmation at default cadence; small/distant
people may be missed. Monitoring keeps saving sources open. Detector history is
bounded and volatile. HTTPS publication needs a same-origin reverse proxy.
GPU availability must be repaired and verified separately before enabling this
service; no driver/kernel upgrade is implied by installation.
