#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import Control


STOP_LABELS  = {'red_light', 'stop'}
SLOW_LABELS  = {'yellow_light', 'slow_down', 'speed_limit_20'}
LEFT_LABELS  = {'turn_left'}
# The track has no right-turn sign. 'no_right_turn' means "do not turn right"
# and must never arm a right turn.
RIGHT_LABELS = set()


class TurnBehaviorNode(Node):

    def __init__(self):

        super().__init__('turn_behavior_node')

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter('sign_timeout',       1.2)
        self.declare_parameter('lane_timeout',       0.3)
        self.declare_parameter('approach_duration', 15.0)   # timeout tiếp cận dự phòng
        self.declare_parameter('approach_min_duration', 0.6)
        self.declare_parameter('intersection_confirm_count', 4)
        self.declare_parameter('recovery_duration',  0.5)   # giây đi thẳng sau khi rẽ
        self.declare_parameter('approach_speed',     0.18)  # tốc độ tiếp cận giao lộ
        self.declare_parameter('turn_speed',         0.22)
        self.declare_parameter('min_speed',          0.0)
        self.declare_parameter('slow_speed',         0.03)  # tốc độ tối thiểu khi tiếp cận giao lộ
        self.declare_parameter('sign_slow_speed',    0.18)  # tốc độ khi gặp biển 20 km/h
        self.declare_parameter('turn_cooldown',      2.0)
        self.declare_parameter('stop_duration',      5.0)
        self.declare_parameter('slow_duration',      5.0)

        # --- Tham số rẽ ---
        self.declare_parameter('sign_confirm_count', 10)    # số frame liên tiếp cần xác nhận biển
        self.declare_parameter('turn_duration',      3.0)   # giây tối đa cho phase rẽ
        self.declare_parameter('turn_min_duration',  0.7)
        self.declare_parameter('lane_reacquire_count', 5)
        self.declare_parameter('left_turn_angle',   -28.0)  # góc lái trái (âm = trái)
        self.declare_parameter('right_turn_angle',   28.0)  # góc lái phải

        # =========================================================
        # LOAD PARAMETERS
        # =========================================================

        self.sign_timeout      = float(self.get_parameter('sign_timeout').value)
        self.lane_timeout      = float(self.get_parameter('lane_timeout').value)
        self.approach_duration = float(self.get_parameter('approach_duration').value)
        self.approach_min_duration = float(
            self.get_parameter('approach_min_duration').value)
        self.intersection_confirm_needed = int(
            self.get_parameter('intersection_confirm_count').value)
        self.recovery_duration = float(self.get_parameter('recovery_duration').value)
        self.approach_speed    = float(self.get_parameter('approach_speed').value)
        self.turn_speed        = float(self.get_parameter('turn_speed').value)
        self.min_speed         = float(self.get_parameter('min_speed').value)
        self.slow_speed        = float(self.get_parameter('slow_speed').value)
        self.sign_slow_speed   = float(self.get_parameter('sign_slow_speed').value)
        self.turn_cooldown     = float(self.get_parameter('turn_cooldown').value)
        self.stop_duration     = float(self.get_parameter('stop_duration').value)
        self.slow_duration     = float(self.get_parameter('slow_duration').value)

        self.sign_confirm_needed = int(self.get_parameter('sign_confirm_count').value)
        self.turn_duration       = float(self.get_parameter('turn_duration').value)
        self.turn_min_duration   = float(
            self.get_parameter('turn_min_duration').value)
        self.lane_reacquire_needed = int(
            self.get_parameter('lane_reacquire_count').value)
        self.left_turn_angle     = float(self.get_parameter('left_turn_angle').value)
        self.right_turn_angle    = float(self.get_parameter('right_turn_angle').value)

        # =========================================================
        # LANE STATE
        # =========================================================

        self.lane_speed    = 0.0
        self.lane_steering = 0.0
        self.lane_mode     = 'NO LANE'
        self.intersection_count = 0
        self.lane_reacquire_count = 0
        self.last_lane_time = None

        # =========================================================
        # LABEL STATE
        # =========================================================

        self.current_label    = 'none'
        self.current_distance = float('inf')
        self.execute_ready    = False
        self.last_label_time  = self.now_sec()

        # =========================================================
        # SIGN ACTION
        # =========================================================

        self.sign_action       = None
        self.sign_action_start = None
        self.sign_action_label = None
        self.consumed_label = None

        # =========================================================
        # TURN SIGN DEBOUNCE
        # Biển rẽ phải được thấy liên tiếp sign_confirm_needed frame
        # mới kích hoạt pending_turn — tránh rẽ sớm do phát hiện nhầm.
        # =========================================================

        self.turn_sign_count = 0
        self.turn_sign_label = None

        # =========================================================
        # TURN STATE
        # =========================================================

        self.pending_turn       = None
        self.pending_since      = 0.0

        self.active_turn             = None
        self.active_turn_start       = 0.0
        self.active_turn_phase       = None  # 'turning' | 'recovering'
        self.active_turn_phase_start = 0.0
        self.turn_saw_lane_loss      = False

        self.cooldown_until = 0.0

        # =========================================================
        # SUBSCRIBERS
        # =========================================================

        self.create_subscription(
            String,
            '/perception/detected_label',
            self.label_callback,
            10
        )

        self.create_subscription(
            Control,
            '/control/lane_suggest',
            self.lane_callback,
            10
        )

        # =========================================================
        # PUBLISHER
        # =========================================================

        self.cmd_pub = self.create_publisher(
            Control,
            '/control/auto',
            10
        )
        self.diagnostic_pub = self.create_publisher(
            String, '/diagnostics/decision', 10)

        # =========================================================
        # TIMER
        # =========================================================

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'TurnBehaviorNode READY  '
            f'confirm={self.sign_confirm_needed}fr  '
            f'approach={self.approach_duration}s  '
            f'turn={self.turn_duration}s'
        )

    # =============================================================
    # TIME
    # =============================================================

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # =============================================================
    # LABEL CALLBACK
    # =============================================================

    def label_callback(self, msg: String):

        raw = msg.data.strip()
        now = self.now_sec()

        if ':' in raw:
            parts = raw.split(':', 1)
            label = parts[0].strip().lower()
            try:
                distance = float(parts[1])
            except ValueError:
                distance = float('inf')
        else:
            label    = raw.lower()
            distance = 0.0

        if not label:
            return

        if label == 'none':
            return

        # A green light explicitly releases a red-light stop.
        if label == 'green_light' and self.sign_action_label == 'red_light':
            self.sign_action = None
            self.sign_action_start = None
            self.sign_action_label = None

        self.current_label    = label
        self.current_distance = distance
        self.last_label_time  = now
        self.execute_ready    = (distance == 0.0)

        # =========================================================
        # STOP / SLOW ACTION
        # =========================================================

        if (self.execute_ready and self.sign_action is None and
                label != self.consumed_label):
            if label in STOP_LABELS:
                self.sign_action       = 'stopping'
                self.sign_action_start = now
                self.sign_action_label = label
                self.consumed_label = label
            elif label in SLOW_LABELS:
                self.sign_action       = 'slowing'
                self.sign_action_start = now
                self.sign_action_label = label
                self.consumed_label = label

        # =========================================================
        # TURN ARM (debounce: phải thấy liên tiếp N frame)
        # =========================================================

        if not self.execute_ready:
            self.turn_sign_count = 0
            self.turn_sign_label = None
            return

        if now < self.cooldown_until:
            return

        if self.active_turn is not None or self.pending_turn is not None:
            self.turn_sign_count = 0
            self.turn_sign_label = None
            return

        if label in LEFT_LABELS or label in RIGHT_LABELS:
            if label == self.turn_sign_label:
                self.turn_sign_count += 1
            else:
                self.turn_sign_count = 1
                self.turn_sign_label = label

            if self.turn_sign_count >= self.sign_confirm_needed:
                direction            = 'left' if label in LEFT_LABELS else 'right'
                self.pending_turn    = direction
                self.pending_since   = now
                self.turn_sign_count = 0
                self.turn_sign_label = None
                self.get_logger().info(
                    f'Turn armed {direction.upper()} '
                    f'(confirmed {self.sign_confirm_needed} frames) — '
                    f'approaching for {self.approach_duration}s'
                )
        else:
            self.turn_sign_count = 0
            self.turn_sign_label = None

    # =============================================================
    # LANE CALLBACK
    # =============================================================

    def lane_callback(self, msg: Control):
        self.last_lane_time = self.now_sec()
        self.lane_speed    = float(msg.speed)
        self.lane_steering = float(msg.steering)
        self.lane_mode     = msg.message.strip().upper() or 'UNKNOWN'

        if self.pending_turn is not None and self.lane_mode == 'NO LANE':
            self.intersection_count += 1
        else:
            self.intersection_count = 0

        if self.active_turn is not None and self.lane_mode != 'NO LANE':
            self.lane_reacquire_count += 1
        else:
            self.lane_reacquire_count = 0
            if self.active_turn is not None and self.lane_mode == 'NO LANE':
                self.turn_saw_lane_loss = True

    # =============================================================
    # BUILD CMD
    # =============================================================

    def build_cmd(self, speed, steering, message):
        cmd          = Control()
        cmd.speed    = float(max(0.0, min(1.0, speed)))
        cmd.steering = float(steering)
        cmd.message  = message
        return cmd

    # =============================================================
    # NORMAL LANE CMD
    # =============================================================

    def normal_lane_cmd(self):

        now      = self.now_sec()
        steering = self.lane_steering

        if self.sign_action == 'stopping':
            elapsed = now - self.sign_action_start
            if self.sign_action_label == 'red_light' or elapsed < self.stop_duration:
                return self.build_cmd(0.0, steering, 'STOP')
            self.sign_action       = None
            self.sign_action_start = None
            self.sign_action_label = None

        elif self.sign_action == 'slowing':
            elapsed = now - self.sign_action_start
            if elapsed < self.slow_duration:
                # Never exceed the lane follower's speed: it is 0 when the
                # lane is lost, and a speed sign must not override that stop.
                speed = min(self.lane_speed, self.sign_slow_speed)
                return self.build_cmd(speed, steering, 'SLOW')
            self.sign_action       = None
            self.sign_action_start = None
            self.sign_action_label = None

        speed = max(self.min_speed, self.lane_speed)
        return self.build_cmd(speed, steering, 'LANE_ONLY')

    # =============================================================
    # ACTIVE TURN CMD
    # Bám làn tự nhiên khi rẽ — không dùng góc cố định.
    # Giai đoạn 1 (turning):   theo lane_suggest trong turn_duration giây (tối đa).
    #                           Nếu không thấy làn → góc nhẹ mặc định.
    # Giai đoạn 2 (recovering): đi thẳng 0° trong recovery_duration giây.
    # =============================================================

    def active_turn_cmd(self):

        now = self.now_sec()

        if self.active_turn_phase == 'turning':
            elapsed = now - self.active_turn_phase_start
            lane_reacquired = (
                elapsed >= self.turn_min_duration and
                self.turn_saw_lane_loss and
                self.lane_reacquire_count >= self.lane_reacquire_needed
            )
            if elapsed < self.turn_duration and not lane_reacquired:
                # Khi da vao giao lo, ra lenh re ro rang. Khong dung lai
                # lane_suggest cu vi tam lan cu van con trong anh va se keo xe
                # di thang giong hanh vi truoc day.
                steer = (
                    self.left_turn_angle if self.active_turn == 'left'
                    else self.right_turn_angle
                )
                spd = self.turn_speed
                return self.build_cmd(spd, steer, f'TURN_{self.active_turn.upper()}')

            reason = 'lane reacquired' if lane_reacquired else 'turn timeout'
            self.get_logger().info(f'Finish turning phase: {reason}')
            # Chuyển sang recovery
            self.active_turn_phase       = 'recovering'
            self.active_turn_phase_start = now

        # Phase: recovering
        elapsed = now - self.active_turn_phase_start
        if elapsed < self.recovery_duration:
            return self.build_cmd(self.approach_speed, 0.0, 'RECOVER')

        # Xong — quay lại bám làn
        self.get_logger().info(f'Turn complete: {self.active_turn}')
        self.active_turn       = None
        self.active_turn_phase = None
        self.turn_saw_lane_loss = False
        self.cooldown_until    = now + self.turn_cooldown
        return self.normal_lane_cmd()

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        now = self.now_sec()

        # Lane perception is safety-critical. Never continue a previous turn or
        # lane command after its producer has stopped publishing.
        if self.last_lane_time is None or now - self.last_lane_time > self.lane_timeout:
            self.cmd_pub.publish(self.build_cmd(0.0, 0.0, 'LANE_TIMEOUT'))
            diagnostic = String()
            diagnostic.data = 'ERROR:LANE_TIMEOUT'
            self.diagnostic_pub.publish(diagnostic)
            return

        if now - self.last_label_time > self.sign_timeout:
            self.current_label = 'none'
            self.current_distance = float('inf')
            self.execute_ready = False
            self.turn_sign_count = 0
            self.turn_sign_label = None
            self.consumed_label = None

        diagnostic = String()
        diagnostic.data = 'OK'
        self.diagnostic_pub.publish(diagnostic)

        # =========================================================
        # ACTIVE TURN
        # =========================================================

        if self.active_turn is not None:
            self.cmd_pub.publish(self.active_turn_cmd())
            return

        # =========================================================
        # PENDING TURN: tiếp tục đi bình thường 2 giây trước khi rẽ
        # =========================================================

        if self.pending_turn is not None:
            approach_elapsed = now - self.pending_since
            intersection_ready = (
                approach_elapsed >= self.approach_min_duration and
                self.intersection_count >= self.intersection_confirm_needed
            )
            approach_timeout = approach_elapsed >= self.approach_duration

            if not intersection_ready and not approach_timeout:
                # Cham lai trong luc den ngã ba, van bam theo lan hien tai.
                speed = min(max(self.lane_speed, self.slow_speed), self.approach_speed)
                self.cmd_pub.publish(
                    self.build_cmd(
                        speed,
                        self.lane_steering,
                        f'APPROACH_{self.pending_turn.upper()}'
                    )
                )
                return

            # =====================================================
            # START TURN
            # =====================================================

            self.active_turn             = self.pending_turn
            self.active_turn_start       = now
            self.active_turn_phase       = 'turning'
            self.active_turn_phase_start = now
            self.pending_turn            = None
            self.intersection_count      = 0
            self.lane_reacquire_count    = 0
            self.turn_saw_lane_loss      = self.lane_mode == 'NO LANE'

            reason = 'intersection detected' if intersection_ready else 'approach timeout'
            self.get_logger().info(
                f'Start turn {self.active_turn.upper()}: {reason}')
            self.cmd_pub.publish(self.active_turn_cmd())
            return

        # =========================================================
        # NORMAL
        # =========================================================

        self.cmd_pub.publish(self.normal_lane_cmd())


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)
    node = TurnBehaviorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
