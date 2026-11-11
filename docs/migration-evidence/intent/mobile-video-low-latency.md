# Mobile video-only TCP relay — 2026-11-11

Operator requests the same optimisation already applied to the fixed front
camera, now for mobile/PTZ. Current MediaMTX cam4/cam4_normal still ingest RTSP
sources directly; saving is ready with H264/G711/Generic. Five-minute passive
sample contained no loss/decode warnings; no current loss/latency reduction is
claimed. Keep camera encoders, PTZ, other paths and navigation unchanged.

Extend the existing launcher allowlist to mobile paths. Back up private YAML,
source JSON and helper, preserve original mobile source URLs, then hot-reload
only the two mobile routes as video-only RTSP/TCP stream-copy publishers.
Validate both quality modes, unchanged service PIDs and detector recovery.
No ROS restart, movement, re-encoding, new camera settings or source secrets in
versioned documentation. Rollback is selective restoration of mobile entries.
