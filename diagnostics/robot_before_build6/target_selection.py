"""Choose one image detection to follow without depending on ROS."""

import math


def normalize_target(value):
    value = ' '.join(str(value).strip().lower().replace('_', ' ').split())
    return {'phone': 'cell phone', 'mobilephone': 'cell phone',
            'mobile phone': 'cell phone', 'cellphone': 'cell phone',
            'điện thoại': 'cell phone', 'ghế': 'chair', 'chai': 'bottle',
            'người': 'person'}.get(value, value)


def approach_command(speed, turn, preferred_side, check):
    """Try slower arcs/diagonals before falling back to escape rotation.

    A candidate is accepted only by the same swept-footprint guard used by
    the final motor heartbeat. Side ordering is latched by the caller.
    """
    side = 1 if preferred_side >= 0 else -1
    turn = max(-12.0, min(12.0, turn))
    for vx in (min(speed, .08), .04):
        for vy in (0.0, side * .04, side * .08, -side * .04, -side * .08):
            for wz in (turn, 0.0):
                if check(vx, vy, wz):
                    return vx, vy, wz
    return None


class TargetSteering(object):
    """Filter box-centre jitter and add hysteresis around straight ahead."""

    def __init__(self, alpha=0.35, enter_px=24.0, exit_px=12.0,
                 gain_deg_per_px=0.045, max_turn_deg=18.0):
        self.alpha = alpha
        self.enter_px = enter_px
        self.exit_px = exit_px
        self.gain = gain_deg_per_px
        self.max_turn = max_turn_deg
        self.filtered_error = None
        self.active = False

    def reset(self):
        self.filtered_error = None
        self.active = False

    def update(self, error_px):
        error_px = float(error_px)
        if self.filtered_error is None:
            self.filtered_error = error_px
        else:
            self.filtered_error += self.alpha * (error_px - self.filtered_error)
        magnitude = abs(self.filtered_error)
        if not self.active:
            if magnitude < self.enter_px:
                return 0.0
            self.active = True
        elif magnitude <= self.exit_px:
            self.active = False
            return 0.0
        turn = -self.filtered_error * self.gain
        return max(-self.max_turn, min(self.max_turn, turn))


def choose_target(detections, image_size, previous_center=None, target_class=None):
    if not detections:
        return None
    width, height = image_size
    if previous_center is not None:
        # A detection far from the last box is a different object. Treat this
        # frame as a miss instead of jumping to it while the robot is moving.
        max_jump = .24 * max(width, height)
        detections = [det for det in detections
                      if math.hypot(det['box'][0] + det['box'][2] * .5 - previous_center[0],
                                    det['box'][1] + det['box'][3] * .5 - previous_center[1]) <= max_jump]
        if not detections:
            return None
    # Small portable objects low in the image are usually the ones a ground
    # robot can approach. Confidence still matters; continuity breaks ties.
    low_object = target_class in ('bottle', 'cup', 'cell phone', 'remote')

    def score(det):
        x, y, bw, bh = det['box']
        center = (x + bw * .5, y + bh * .5)
        result = float(det['conf'])
        if low_object:
            result += .55 * max(0., min(1., (y + bh) / float(height)))
        if previous_center is not None:
            result -= .18 * math.hypot(center[0] - previous_center[0],
                                       center[1] - previous_center[1]) / max(width, height)
        return result

    return max(detections, key=score)


class TargetAvoidance(object):
    """Keep one escape direction through brief detection/safety fluctuations."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase = 'IDLE'
        self.direction = 0
        self.start_pose = None
        self.started = 0.0
        self.turn_yaw = 0.0
        self.clear_count = 0
        self.attempts = 0
        self.retry_after = 0.0

    def _blocked(self, now):
        self.phase = 'BLOCKED'
        # Recheck periodically instead of freezing in BLOCKED forever. The
        # next attempt uses the opposite side and still passes every command
        # through FootprintSafety before any motion is issued.
        self.retry_after = now + min(4.0, 1.0 + 0.75 * max(0, self.attempts - 1))

    def command(self, forward_clear, pose, preferred_side, check, now, align_turn=None):
        """Return an override (vx, vy, turn, label), or None for forward."""
        if self.phase == 'IDLE' and forward_clear:
            return None
        if self.phase == 'BLOCKED':
            if forward_clear:
                self.clear_count += 1
                if self.clear_count >= 3:
                    self.reset()
                    return None
            else:
                self.clear_count = 0
            if now < self.retry_after:
                return 0.0, 0.0, 0.0, 'CHUA CO LOI DI AN TOAN TOI MUC TIEU'
            preferred_side = -self.direction if self.direction else -preferred_side
            self.phase = 'IDLE'
        x, y, yaw = pose
        if self.phase == 'IDLE':
            self.start_pose = (x, y)
            self.started = now
            self.direction = 1 if preferred_side >= 0 else -1
            self.attempts += 1
            self.clear_count = 0
            if not check(0.0, self.direction * .08, 0.0):
                if check(0.0, -self.direction * .08, 0.0):
                    self.direction *= -1
                else:
                    self.phase = 'TURN'
                    self.turn_yaw = yaw
                    turn = self.direction * 18.0 if align_turn is None else align_turn
                    if abs(turn) < .5 or not check(0.0, 0.0, turn):
                        self._blocked(now)
                        return 0.0, 0.0, 0.0, 'CHUA CO LOI DI AN TOAN TOI MUC TIEU'
            if self.phase == 'IDLE':
                self.phase = 'STRAFE'
        self.clear_count = self.clear_count + 1 if forward_clear else 0
        if self.clear_count >= 3:
            self.reset()
            return None
        if self.phase == 'STRAFE':
            moved = math.hypot(x - self.start_pose[0], y - self.start_pose[1])
            side = self.direction * .08
            if moved < .30 and now - self.started < 4.0 and check(0.0, side, 0.0):
                return 0.0, side, 0.0, 'NE NGANG MOT HUONG'
            self.phase = 'TURN'
            self.turn_yaw = yaw
        if self.phase == 'TURN':
            turned = abs((yaw - self.turn_yaw + math.pi) % (2 * math.pi) - math.pi)
            # While a target is visible, align toward it instead of rotating
            # blindly in the strafe direction and pushing it out of view.
            turn = self.direction * 18.0 if align_turn is None else align_turn
            if abs(turn) < .5:
                self._blocked(now)
                return 0.0, 0.0, 0.0, 'MUC TIEU O GIUA; CHO LOI DI AN TOAN'
            if turned < math.radians(45) and now - self.started < 8.0 and check(0.0, 0.0, turn):
                return 0.0, 0.0, turn, 'XOAY TIM LOI DI CUNG HUONG'
            self._blocked(now)
        return 0.0, 0.0, 0.0, 'CHUA CO LOI DI AN TOAN TOI MUC TIEU'
