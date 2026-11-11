# Cockpit startup recovery — 2026-11-11

After the authorised GPU maintenance reboot the real ROS service and navigation
were active, but no process listened on 8766. Docker startup logs show
web_bridge exiting with EADDRNOTAVAIL from Recordings.start: its download proxy
bound to the private Tailscale IP before that address existed. Launch did not
respawn the node. A standalone bridge launched with the same persistent route,
recording and sensor parameters recovered the public Cockpit connection without
restarting navigation. This is a confirmed optional-service startup dependency,
not evidence of an HTTPS/WebSocket browser failure.

Keep gateway startup independent of that private interface. Retry only
EADDRNOTAVAIL at the existing two-second recording-worker cadence; preserve the
exact configured bind, signed URLs, lease and control lock. Other synchronous
bind failures still raise. Stop cancels retry and closes any recovered server.
No wildcard fallback, network security changes, ROS contracts or motion changes.
Characterisation: injected unavailable interface, eventual recovery, shutdown,
address-in-use rejection, and existing unsigned-download rejection tests.
