"""Timer-driven camera auxiliary policy; caller supplies time and serializes I/O."""

from .camera_backend import CameraBackendError
from .camera_domain import PtzPose


class CameraFeaturesController:
    LEG_TIMEOUT_S = 120.0
    LIGHT_DURATION_S = 60
    LIGHT_RENEW_S = 40.0
    PAN_TOLERANCE_DEG = 0.8

    def __init__(self, backend, limits):
        self.backend = backend
        self.supported = all(callable(getattr(backend, name, None))
                             for name in ("stop", "set_light"))
        self.left = min(limits.pan_max_deg, 349.99999524)
        self.right = limits.pan_min_deg
        self.phase = "idle"
        self.error = ""
        self.lights = {"fixed": -1, "mobile": -1}
        self.renew_at = {}
        self.deadline = 0.0
        self.target = None

    @property
    def running(self):
        return self.phase in ("moving_left", "sweeping_right", "sweeping_left")

    def _require_supported(self):
        if not self.supported:
            raise CameraBackendError("camera auxiliary controls unsupported")

    def _write_target(self):
        writer = getattr(self.backend, "write_sweep_absolute", None)
        if not callable(writer):
            writer = self.backend.write_absolute
        writer(self.target)

    def start(self, now):
        self._require_supported()
        if self.running:
            raise CameraBackendError("camera sweep already running")
        pose = self.backend.read_state()
        self.target = PtzPose(self.left, pose.tilt_deg, pose.zoom_level)
        self._write_target()
        self.phase, self.deadline, self.error = "moving_left", now + self.LEG_TIMEOUT_S, ""

    def stop(self):
        self._require_supported()
        # Cancel policy first even if network Stop fails. Never resume old targets.
        self.phase = "idle"
        try:
            self.backend.stop()
            self.error = ""
        except CameraBackendError:
            self.phase, self.error = "error", "camera stop failed"
            raise

    def set_light(self, lens, enabled, now):
        self._require_supported()
        if lens not in self.lights or not isinstance(enabled, bool):
            raise ValueError("invalid camera light target")
        self.renew_at.pop(lens, None)
        try:
            self.backend.set_light(lens, enabled, self.LIGHT_DURATION_S)
            self.lights[lens] = int(enabled)
            if enabled:
                self.renew_at[lens] = now + self.LIGHT_RENEW_S
            self.error = ""
        except CameraBackendError:
            self.lights[lens] = -1
            self.error = "camera light command failed"
            raise

    def tick(self, now):
        if self.running:
            try:
                if now >= self.deadline:
                    raise CameraBackendError("camera sweep timed out")
                pose = self.backend.read_state()
                if abs(pose.pan_deg - self.target.pan_deg) <= self.PAN_TOLERANCE_DEG:
                    if self.phase in ("moving_left", "sweeping_left"):
                        self.target = PtzPose(self.right, self.target.tilt_deg,
                                              self.target.zoom_level)
                        self._write_target()
                        self.phase, self.deadline = "sweeping_right", now + self.LEG_TIMEOUT_S
                    else:
                        self.target = PtzPose(self.left, self.target.tilt_deg,
                                              self.target.zoom_level)
                        self._write_target()
                        self.phase, self.deadline = "sweeping_left", now + self.LEG_TIMEOUT_S
            except CameraBackendError as error:
                try:
                    self.stop()
                except CameraBackendError:
                    pass
                self.phase, self.error = "error", str(error)
        for lens, due in list(self.renew_at.items()):
            if now >= due:
                try:
                    self.set_light(lens, True, now)
                except CameraBackendError:
                    # No blind retries: mark unknown and leave finite camera timeout.
                    pass

    def shutdown(self):
        if not self.supported:
            return
        if self.running:
            try:
                self.stop()
            except CameraBackendError:
                pass
        for lens in list(self.renew_at):
            try:
                self.set_light(lens, False, 0.0)
            except CameraBackendError:
                pass
