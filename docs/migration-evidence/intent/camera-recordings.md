# Camera video recordings — 2026-10-24

New operator request, not a port of rosbag or legacy `start_recording` (waypoint
capture). MediaMTX v1.19.1 was observed active on Jetson with recording, playback
and administrative API disabled. Streams cam3/cam4/cam5 map to principal/mobile/
reverse. These facts do not prove recording or download availability.

Owner: salus_web operator tooling, MediaMTX owns video. No video bytes in ROS or
WebSocket. The optional gateway `recordings_config` parameter is a string file
path (default empty = disabled); config contains loopback API/playback URLs,
recording root and atomic intent state file, download bind address/port (1–65535)
and public HTTP URL reachable only on the authorized operator network.

WebSocket operations: recordings_status and recordings_list are read-only;
recordings_start and recordings_stop require the existing exclusive control
lease. Mutations toggle only the persistent manual reason. List takes camera
(all/main/mobile/rear) and timezone-aware start/end spanning at most 25 hours.
Responses include camera/start/duration and signed download URL valid ten minutes;
clips are at most 600 seconds and the client refreshes signatures on download.
The loopback MediaMTX API is never exposed to browser clients.

Gateway polls canonical GetRouteMissionState and GetPatrolMissionState every
second with a four-second timeout. Any active route/patrol requires recording;
pauses/blocks and HOME remain part of that mission. Losing authority never
turns off an existing mission capture. Terminal state starts 30-second tail.
A new mission cancels that tail. Manual capture survives mission end and process
restart. Tail restart conservatively allows another 30 seconds pending authority.
No velocity, goal button or client connection is a recording trigger.

Video-only rec_cam3/rec_cam4/rec_cam5 paths use fMP4, 1s parts, 10m minimum segments, 7d deletion; initial
record=false and sourceOnDemand=true. FFmpeg copies only the first video track from each live RTSP stream into its
recording publisher path, without recoding. Audio and metadata are discarded.
Reconciliation starts/stops these publishers and recording flags independently;
the original live paths retain on-demand behavior. `setpriv --pdeathsig TERM`
prevents orphan capture workers if the gateway dies. FFmpeg is an image dependency;
optional ffmpeg/ffmpeg_library_path overrides support a persistent distro bundle.
Newest segment of every camera is protected from space cleanup, as are recently
modified files and symlinks. Older closed segments can be deleted below 20 GiB
free. Below that margin capture is suspended; seven days is a maximum, not a
promise. Recording status requires a ready source and recent nonempty file.

Deployment must preserve the Jetson branch: 0f1ff69 contains camera features
absent from local/main cd58958. Integrate as a bounded patch, never reset that
checkout. Save private MediaMTX YAML without publishing it. Recording config must
be mounted into the ROS runtime and root writable/visible on the same filesystem
as MediaMTX. API/playback bind loopback; signed download listener binds only the
operator network. No real mission or motion test without operator request.

Validation pending: MediaMTX actual download/container playback, audio track
absence, controlled retention fixtures, restart recovery, integrated physical
mission transitions. Do not claim hardware parity until evidence is appended.

## Physical bench evidence (2026-10-24)

All three canonical mission services were read with active=false before tests.
Three short manual captures produced video-only H264 (no audio or metadata).
Signed MP4 downloads were parsed by ffprobe: principal 257894 bytes, mobile
211809 bytes, reverse 442687 bytes. Unsigned HTTP download returned 403.
The live WebSocket rejected manual start while locked; after taking the lease,
start recorded all three cameras, then stop and lock restoration succeeded.
Gateway replacement did not restart controllers or the parent real launch.
Mission automation was tested in policy fixtures, NOT with a moving robot.
Seven-day retention is configured; elapsed seven-day behavior is not observed.

The Jetson uses a recording-specific branch based on its deployed camera-feature
branch, not a reset to main. Its startup wrapper forwards recordings_config on
next boot. Since its cached ROS image predates FFmpeg, an Ubuntu Jammy ARM64
FFmpeg 4.4.2 package/dependency bundle was extracted to persistent state and is
referenced explicitly by config. No package binaries are committed to Git.
Original private media YAML and startup wrapper backups are retained on Jetson.

The video-only publisher paths are rec_cam3/rec_cam4/rec_cam5; original camera
paths are unchanged. The MediaMTX merge snippet is in
`src/salus_web/config/recordings-mediamtx.example.yaml`. Its host recording root
and the coordinator's container root must resolve to the same directory.
`ffmpeg` is an optional executable path (default ffmpeg on PATH);
`ffmpeg_library_path` optionally supplies its LD_LIBRARY_PATH. `rtsp` is the local
MediaMTX RTSP base URL, default rtsp://127.0.0.1:8554. Never use camera credentials
in these URLs: MediaMTX retains the private camera sources.

Rollback: disconnect Cockpit; stop only the replacement gateway, restore the
private MediaMTX YAML and startup wrapper backups, and relaunch only the gateway
with its saved web-params.yaml and without recordings_config. Preserve recordings
and intent state. No robot-wide restart is required for rollback. Before any
rollback ensure no mission is active; do not cancel a mission implicitly.

Full robot reboot was not exercised. On future image replacement the standard
Dockerfile installs FFmpeg and util-linux; the current deployment explicitly
uses the persistent Jammy bundle so it does not depend on container-layer changes.
The optional feature remains disabled in default simulation/real launches unless
recordings_config is supplied.
