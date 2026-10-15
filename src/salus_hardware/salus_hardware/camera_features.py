"""Timer-driven camera auxiliary policy; caller supplies time and serializes I/O."""

from .camera_backend import CameraBackendError
from .camera_domain import PtzPose


class CameraFeaturesController:
    LEG_TIMEOUT_S = 120.0
    LIGHT_DURATION_S = 60
    LIGHT_RENEW_S = 40.0
    PAN_TOLERANCE_DEG = 0.8
    MAX_STATUS_FAILURES = 3
    MAX_RECOVERY_ATTEMPTS = 3
    RECOVERY_DELAY_S = 5.0

    def __init__(self, backend, limits):
        self.backend = backend
        self.limits = limits
        self.supported = all(callable(getattr(backend, name, None))
                             for name in ("stop", "set_light"))
        self.left = min(limits.pan_max_deg, 349.99999524)
        self.right = limits.pan_min_deg
        self.phase = "idle"
        self.sweep_error = ""
        self.light_errors = {}
        self.status_failures = 0
        self.lights = {"fixed": -1, "mobile": -1}
        self.renew_at = {}
        self.deadline = 0.0
        self.target = None
        self.recovery_attempts = 0
        self.recovery_at = None
        self.resume_phase = None

    @property
    def error(self):
        return "; ".join(filter(None, [self.sweep_error, *self.light_errors.values()]))

    @property
    def running(self):
        return self.phase in (
            "moving_left", "sweeping_right", "sweeping_left", "recovering", "centering",
        )

    def _require_supported(self):
        if not self.supported:
            raise CameraBackendError("camera auxiliary controls unsupported")

    def _write_target(self):
        writer = getattr(self.backend, "write_sweep_absolute", None)
        if not callable(writer):
            writer = self.backend.write_absolute
        writer(self.target)

    def start(self, now, center=None):
        self._require_supported()
        if self.running:
            raise CameraBackendError("camera sweep already running")
        pose = self.backend.read_state()
        center = pose if center is None else center
        self.left = center.pan_deg + 120.0
        self.right = center.pan_deg - 120.0
        if self.right < self.limits.pan_min_deg or self.left > min(
                self.limits.pan_max_deg, 349.99999524):
            raise CameraBackendError("front preset cannot fit a 120-degree sweep on each side")
        self.target = center
        self._write_target()
        self.phase, self.deadline = "centering", now + self.LEG_TIMEOUT_S
        self.sweep_error = ""
        self.status_failures = 0
        self.recovery_attempts = 0
        self.recovery_at = None

    def stop(self):
        self._require_supported()
        # Cancel policy first even if network Stop fails. Never resume old targets.
        self.phase = "idle"
        self.recovery_at = None
        try:
            self.backend.stop()
            self.sweep_error = ""
        except CameraBackendError:
            self.phase, self.sweep_error = "error", "camera stop failed"
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
            self.light_errors.pop(lens, None)
        except CameraBackendError:
            self.lights[lens] = -1
            self.light_errors[lens] = f"camera {lens} light command failed"
            raise

    def _recover_after_failure(self, now, error):
        if self.phase != "recovering":
            self.resume_phase = self.phase
        self.sweep_error = str(error)
        try:
            self.backend.stop()
        except CameraBackendError:
            self.phase = "error"
            self.recovery_at = None
            self.sweep_error += "; camera stop failed; automatic recovery cancelled"
            return
        if self.recovery_attempts >= self.MAX_RECOVERY_ATTEMPTS:
            self.phase = "error"
            self.recovery_at = None
            self.sweep_error += "; automatic recovery exhausted"
        else:
            self.phase = "recovering"
            self.recovery_at = now + self.RECOVERY_DELAY_S

    def tick(self, now):
        if self.phase == "recovering":
            if now >= self.recovery_at:
                self.recovery_attempts += 1
                try:
                    # Read current position before retrying the interrupted target.
                    self.backend.read_state()
                    self._write_target()
                    self.phase = self.resume_phase
                    self.deadline = now + self.LEG_TIMEOUT_S
                    self.status_failures = 0
                    self.sweep_error = ""
                except CameraBackendError as error:
                    self._recover_after_failure(now, error)
        elif self.running:
            try:
                if now >= self.deadline:
                    raise CameraBackendError("camera sweep timed out")
                try:
                    pose = self.backend.read_state()
                except CameraBackendError:
                    self.status_failures += 1
                    if self.status_failures >= self.MAX_STATUS_FAILURES:
                        raise
                    pose = None
                else:
                    self.status_failures = 0
                if (pose is not None
                        and abs(pose.pan_deg - self.target.pan_deg) <= self.PAN_TOLERANCE_DEG):
                    if self.phase == "centering":
                        self.target = PtzPose(self.left, self.target.tilt_deg,
                                              self.target.zoom_level)
                        self._write_target()
                        self.phase, self.deadline = "moving_left", now + self.LEG_TIMEOUT_S
                    elif self.phase in ("moving_left", "sweeping_left"):
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
                self._recover_after_failure(now, error)
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
