"""Manual command bounds/gate; pure logic, no ROS or automatic route generation."""
import math

ZERO = (0.0,) * 6
LINEAR_LIMIT = 0.10
ANGULAR_LIMIT = 0.10
INPUT_TIMEOUT = 0.35
CLOCK_TIMEOUT = 0.50
ARM_WINDOW = 120.0


def bounded(values):
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('manual input must contain six finite values')
    # Only differential-drive X/yaw are supported, regardless of GUI settings.
    return (max(-LINEAR_LIMIT, min(LINEAR_LIMIT, values[0])), 0.0, 0.0,
            0.0, 0.0, max(-ANGULAR_LIMIT, min(ANGULAR_LIMIT, values[5])))


class ManualGate:
    def __init__(self):
        self.clock_ns = None
        self.clock_wall = None
        self.input_wall = None
        self.command = ZERO
        self.armed = False
        self.armed_wall = None
        self.fault = None

    def fail(self, reason):
        self.fault = reason
        self.disarm()

    def clock(self, stamp_ns, now):
        if stamp_ns < 0 or (self.clock_ns is not None and stamp_ns < self.clock_ns):
            self.fail('simulation clock rewind/invalid stamp')
            return
        if self.clock_ns is None or stamp_ns > self.clock_ns:
            self.clock_ns, self.clock_wall = stamp_ns, now

    def receive(self, values, now):
        try:
            self.command = bounded(values)
            self.input_wall = now
        except ValueError:
            self.fail('nonfinite/malformed manual input')

    def clock_fresh(self, now):
        return self.clock_wall is not None and 0 <= now - self.clock_wall <= CLOCK_TIMEOUT

    def arm(self, now):
        if self.armed or self.fault or not self.clock_fresh(now) or self.command != ZERO:
            return False
        # Forget preview input; movement requires a new operator event after arming.
        self.command, self.input_wall = ZERO, None
        self.armed, self.armed_wall = True, now
        return True

    def disarm(self):
        self.armed, self.armed_wall = False, None
        self.command, self.input_wall = ZERO, None

    def sample(self, now):
        if self.armed and not self.clock_fresh(now):
            self.fail('simulation clock stalled')
        if self.armed and now - self.armed_wall >= ARM_WINDOW:
            self.fail('manual authorization window expired')
        fresh = (self.input_wall is not None and 0 <= now - self.input_wall <= INPUT_TIMEOUT
                 and self.clock_fresh(now) and not self.fault)
        preview = self.command if fresh else ZERO
        return preview, preview if self.armed else ZERO
