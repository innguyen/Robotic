"""Choose one image detection to follow without depending on ROS."""

import math


def normalize_target(value):
    value = ' '.join(str(value).strip().lower().replace('_', ' ').split())
    return {'phone': 'cell phone', 'mobilephone': 'cell phone',
            'mobile phone': 'cell phone', 'cellphone': 'cell phone',
            'điện thoại': 'cell phone', 'ghế': 'chair', 'chai': 'bottle',
            'người': 'person'}.get(value, value)


TARGET_WIDTH_M = {
    'person': .42, 'chair': .48, 'couch': 1.5, 'dining table': .9,
    'suitcase': .45, 'cell phone': .075, 'bottle': .07, 'cup': .09,
    'remote': .05, 'laptop': .33, 'tv': .75, 'book': .16,
}
TARGET_STANDOFF_CM = {
    'person': 80.0, 'cell phone': 45.0, 'bottle': 45.0,
    'cup': 45.0, 'remote': 40.0, 'chair': 75.0, 'suitcase': 70.0,
}


def estimate_target_distance(box_width_px, image_width_px, target):
    ratio = max(float(box_width_px) / max(1.0, float(image_width_px)), .02)
    width = TARGET_WIDTH_M.get(normalize_target(target), .35)
    return max(.25, min(5.0, .866 * width / ratio))


def target_standoff_cm(target):
    return TARGET_STANDOFF_CM.get(normalize_target(target), 65.0)


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


def tracking_command(avoidance, speed, turn, pose, preferred_side, check, now):
    """Finish a latched side step before accepting another diagonal command."""
    direct = (speed, 0.0, turn)
    forward = direct if check(*direct) else None
    if forward is None:
        for vx in (min(speed, .08), .04):
            for wz in (max(-12.0, min(12.0, turn)), 0.0):
                if check(vx, 0.0, wz):
                    forward = (vx, 0.0, wz)
                    break
            if forward is not None:
                break
    override = avoidance.command(forward is not None, pose, preferred_side,
                                 check, now, align_turn=turn)
    if override is not None:
        return override
    if forward is not None:
        return forward + ('BAM LAI MUC TIEU',)
    return 0.0, 0.0, 0.0, 'CHUA CO LOI DI AN TOAN TOI MUC TIEU'


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


class TargetConfirmation(object):
    """Require two nearby detections before interrupting exploration."""

    def __init__(self, max_gap=0.7, max_jump_fraction=0.24):
        self.max_gap = max_gap
        self.max_jump_fraction = max_jump_fraction
        self.reset()

    def reset(self):
        self.center = None
        self.last_time = 0.0
        self.count = 0

    def update(self, detection, image_size, now):
        if detection is None:
            if now - self.last_time > self.max_gap:
                self.reset()
            return False
        x, y, width, height = detection['box']
        center = (x + width * .5, y + height * .5)
        nearby = (self.center is not None and now - self.last_time <= self.max_gap
                  and math.hypot(center[0] - self.center[0],
                                 center[1] - self.center[1])
                  <= self.max_jump_fraction * max(image_size))
        self.count = self.count + 1 if nearby else 1
        self.center = center
        self.last_time = now
        return self.count >= 2


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

    def __init__(self, step_m=.05):
        self.step_m = step_m
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
        self.step_pose = None
        self.steps = 0
        self.best_moved = 0.0
        self.last_progress = 0.0

    def _side_command(self, direction, check, max_speed=.08):
        for speed in (.08, .04):
            if speed <= max_speed and check(0.0, direction * speed, 0.0):
                return 0.0, direction * speed, 0.0, 'NE NGANG TUNG BUOC 5CM'
        return None

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
        # Stop side motion on the first clear observation. Confirm before
        # moving forward, so confirmation does not add an unnecessary step.
        self.clear_count = self.clear_count + 1 if forward_clear else 0
        if self.clear_count >= 2:
            self.reset()
            return None
        if forward_clear:
            return 0.0, 0.0, 0.0, 'LOI TOI MUC TIEU DA THONG; XAC NHAN'
        if self.phase == 'BLOCKED':
            preferred = self.direction or preferred_side
            # Fresh side evidence can unblock a centred target immediately.
            if self._side_command(preferred, check) is not None:
                preferred_side = preferred
            elif self._side_command(-preferred, check) is not None:
                preferred_side = -preferred
            elif now < self.retry_after:
                return 0.0, 0.0, 0.0, 'CHUA CO LOI DI AN TOAN TOI MUC TIEU'
            self.phase = 'IDLE'
        x, y, yaw = pose
        if self.phase == 'IDLE':
            self.start_pose = (x, y)
            self.step_pose = (x, y)
            self.started = now
            self.last_progress = now
            self.best_moved = 0.0
            self.direction = 1 if preferred_side >= 0 else -1
            self.attempts += 1
            self.clear_count = 0
            if self._side_command(self.direction, check) is None:
                if self._side_command(-self.direction, check) is not None:
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
        if self.phase == 'STRAFE':
            moved = math.hypot(x - self.start_pose[0], y - self.start_pose[1])
            if moved > self.best_moved + .01:
                self.best_moved = moved
                self.last_progress = now
            step = math.hypot(x - self.step_pose[0], y - self.step_pose[1])
            if step >= self.step_m:
                self.step_pose = (x, y)
                self.steps += 1
                return 0.0, 0.0, 0.0, 'DA LACH 5CM; KIEM TRA LAI TARGET'
            side_command = self._side_command(
                self.direction, check, .04 if self.step_m - step <= .025 else .08)
            if (moved < .60 and now - self.started < 8.0
                    and now - self.last_progress < 2.5 and side_command is not None):
                return side_command
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
