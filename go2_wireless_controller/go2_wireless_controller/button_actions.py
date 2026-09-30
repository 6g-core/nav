"""Recognize exclusive wireless-controller button gestures."""

START = 0x0004
UP = 0x1000
DOWN = 0x4000
LEFT = 0x8000
RIGHT = 0x2000

DOUBLE_ACTIONS = {
    START: "grab",
    UP: "forward",
    DOWN: "return",
    LEFT: "retract",
    RIGHT: "local_workflow",
}
LONG_ACTIONS = {START: "testclient_workflow"}


class ButtonGestures:
    """Require release between clicks and reject stale or combined input."""

    def __init__(self, double_window=0.5, long_duration=2.0, stale_after=0.5):
        self.double_window = double_window
        self.long_duration = long_duration
        self.stale_after = stale_after
        self.last_sample = None
        self.last_keys = None
        self.reset()

    def reset(self):
        self.armed = False
        self.held = 0
        self.pressed_at = None
        self.first_key = 0
        self.released_at = None

    def feed(self, keys, now):
        if self.last_sample is not None and now - self.last_sample > self.stale_after:
            was_neutral = self.last_keys == 0
            self.reset()
            self.armed = was_neutral
        self.last_sample = now
        self.last_keys = keys

        if keys != 0 and keys not in DOUBLE_ACTIONS:
            self.reset()
            return None
        if not self.armed:
            if keys == 0:
                self.armed = True
            return None
        if keys == 0:
            if self.held:
                duration = now - self.pressed_at
                if duration < self.long_duration:
                    self.first_key = self.held
                    self.released_at = now
                else:
                    action = LONG_ACTIONS.get(self.held)
                    self.reset()
                    return action
                self.held = 0
                self.pressed_at = None
            if self.released_at is not None and now - self.released_at > self.double_window:
                self.first_key = 0
            return None
        if self.held and keys != self.held:
            self.reset()
            return None
        if not self.held:
            if (self.first_key == keys and self.released_at is not None
                    and now - self.released_at <= self.double_window):
                self.reset()
                return DOUBLE_ACTIONS[keys]
            self.first_key = 0
            self.held = keys
            self.pressed_at = now
        return None

    def observe_busy(self, keys, now):
        """Observe input while busy without queuing a later action."""
        self.reset()
        self.last_sample = now
        self.last_keys = keys
        self.armed = keys == 0
