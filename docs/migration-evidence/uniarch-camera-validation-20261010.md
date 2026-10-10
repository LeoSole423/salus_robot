# Uniarch camera deployment and validation — 2026-10-10

## Current deployment

Uniarch Uho-P2S-M33F34, MAC 88:26:3f:a6:5e:b7. Source 1 is the fixed
panoramic lens; source 2 is the movable lens. IPv4 is now static
192.168.1.250/24, gateway 192.168.1.1. Router DHCP pool was verified as
192.168.1.2–192.168.1.244. Camera LAPI IPv4 IPGetType=0 and GET confirmed
this configuration. Previous DHCP addresses were .148 and .149. Persistence
across a camera power cycle remains untested.

Both secondary streams use H.264, 640×360, 15 FPS, configured 256 kbit/s,
GOP 30. Main streams use 1280×720, 15 FPS, 512 kbit/s, GOP 30.
ONVIF ForcePersistence and subsequent GET/ffprobe verified the encoders.
Effective network consumption and CBR/VBR mode remain unverified. RTSP still
includes G.711 audio despite removing the ONVIF audio encoder association.

Host-owned MediaMTX relays source 1 /media1/video2 as cam3 and source 2
/media2/video2 as cam4. Both use sourceOnDemand and close after 10 seconds
without readers. Cockpit connects through the existing Jetson Tailscale host
on port 8889 at /cam3/whep and /cam4/whep. Credentials and private relay
configuration remain external to Git.

Cockpit presents the panorama prominently, with mobile and future rear views
below. Mobile video opens on request; rear video will open automatically while
the Camera workspace is mounted once its URL is configured. No rear camera is
configured yet. Browser verification observed both videos at 640×360,
readyState=4 and advancing timestamps.

## PTZ and presets

Backend source commit 0b7e667 selects onvif_uniarch, channel 2, profile
media_profile2_1. Deployment uses a private credential file and a separate
Uniarch preset JSON file, preserving Hikvision calibration. The effective
Jetson startup wrapper was backed up before modification. These deployment
files are host-owned and are not part of this repository.

Bounded PanTilt targets run at normalized speed 0.15. ROS degree-shaped
coordinates are adapter coordinates, not measured physical angles. Zoom is
unsupported. Cockpit provides stepped arrows and editable Front/Right/Rear/Left
views with confirmation when saving; existing operator authority checks remain.
Continuous joystick control is not integrated.

Stationary hardware test: initial pan 223.949997, tilt 0; saved HOME; relative
pan +3 produced 226.929993; HOME restored 223.949997. A direct ONVIF read
confirmed final pan 223.94999556, tilt 0. Only lens 2 moved. HOME remains at
the original test orientation. Directional presets require calibration to the
installed robot orientation; defaults do not establish physical directions.
The robot remained STARTUP_LOCKED, with driving disabled.

## Validation and limits

Humble build: 16 packages. Focused hardware tests: 176; bringup tests: 171,
zero failures. Web/Cockpit smoke passed. Broad tests excluded external
rslidar_msg as prescribed by tools/test.sh; an earlier run exposed its existing
lint failures. New adapter lint failures were corrected and rechecked.
The preflight path validator also encounters pre-existing untracked .agents
content; that user content was left untouched.
Cockpit build passed; full frontend suite: 196 passed, 1 skipped.
No complete driving, navigation parity, joystick response or zoom validation
is claimed. Evidence images and raw diagnostics remain local operator artifacts
outside Git.

## Audio investigation

Operator-approved siren was converted to 5 seconds, mono signed little-endian
16-bit PCM at 8000 Hz, amplitude 0.25. LAPI POST multipart field file at
/LAPI/V1.0/System/AudioFile installed ID 1; existing ID 0 was preserved.
PUT {} at /LAPI/V1.0/Channels/0/System/AudioFile/1/Play succeeded and the
operator confirmed hearing it. No automatic alarm or live talkback integration
was implemented. Selected source: [Police Operation Siren](https://pixabay.com/sound-effects/film-special-effects-police-operation-siren-144229/).
