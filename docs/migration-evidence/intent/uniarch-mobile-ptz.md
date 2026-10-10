# Uniarch mobile lens PTZ

Operator requests Cockpit movement controls for source 2 and user-facing saved
views Front/Right/Rear/Left. Keep source 1 fixed panorama and source 2 secondary.

Add hardware-owned ONVIF Uniarch adapter, explicit profile media_profile2_1,
HTTP Digest external credentials, bounded normalized absolute PanTilt commands
at speed 0.15. Existing selected-axis relative ROS commands resolve to targets;
no angle-entry UI or velocity joystick contract is introduced. Normalized camera
positions map into the existing degree-shaped ROS contract (x=(pan/180)-1,
y=(tilt/45)-1); these are adapter coordinates, not calibrated physical angles.
Zoom is unsupported and rejected rather than reported as executed.

Make four directional presets editable with atomic persistence. Preserve HOME
and old zoom-save behavior; use separate Uniarch preset file on deployment to
avoid reinterpreting Hikvision calibrations. Real launch selects camera_backend,
default remains isapi. Video transport remains external MediaMTX.

Hardware evidence and remaining axis/orientation calibration will be recorded
after deployment. Presets must be set from the installed robot orientation;
default coordinates are not calibration. No robot driving is part of this task.
