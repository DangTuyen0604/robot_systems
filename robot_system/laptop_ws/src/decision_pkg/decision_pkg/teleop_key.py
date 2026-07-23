#!/usr/bin/env python3
"""
Teleop Mux Node
  AUTO  : forward /control/auto → /control/cmd
  MANUAL: keyboard              → /control/cmd
  TAB   : chuyển chế độ
"""

import sys
import tty
import termios
import select
import threading

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import Control


SPEED_STEP = 0.05
STEER_STEP = 5.0
MAX_SPEED  = 0.50
MAX_STEER  = 45.0

BANNER = """
╔══════════════════════════════════╗
║        TELEOP MUX  (Xe RC)       ║
╠══════════════════════════════════╣
║  TAB     : AUTO <-> MANUAL       ║
║  W / up  : Tang toc              ║
║  S / down: Giam toc              ║
║  A / left: Re trai               ║
║  D / right: Re phai              ║
║  SPACE   : Dung han              ║
║  Q / ESC : Thoat                 ║
╚══════════════════════════════════╝
"""


class TeleopMuxNode(Node):

    def __init__(self):
        super().__init__('teleop_mux_node')

        self.cmd_pub = self.create_publisher(Control, '/control/cmd', 10)
        self.create_subscription(Control, '/control/auto', self._auto_cb, 10)

        self.mode      = 'AUTO'
        self.auto_cmd  = Control()
        self.speed     = 0.0
        self.steering  = 0.0
        self._running  = True

        self.create_timer(0.05, self._publish)

        t = threading.Thread(target=self._key_loop, daemon=True)
        t.start()

        print(BANNER)
        self._print_state()

    # ----------------------------------------------------------------
    def _auto_cb(self, msg: Control):
        self.auto_cmd = msg

    # ----------------------------------------------------------------
    def _publish(self):
        if not self._running:
            return
        if self.mode == 'AUTO':
            self.cmd_pub.publish(self.auto_cmd)
        else:
            cmd          = Control()
            cmd.speed    = float(self.speed)
            cmd.steering = float(self.steering)
            cmd.message  = 'MANUAL'
            self.cmd_pub.publish(cmd)

    # ----------------------------------------------------------------
    def _key_loop(self):
        """Đọc bàn phím — raw mode bật 1 lần, giữ suốt vòng lặp."""
        fd       = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        tty.setraw(fd)                     # raw mode: bật 1 lần duy nhất
        try:
            while self._running and rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)

                # Arrow keys: ESC [ A/B/C/D
                if ch == '\x1b':
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r2:
                        ch2 = sys.stdin.read(1)
                        if ch2 == '[':
                            r3, _, _ = select.select([sys.stdin], [], [], 0.05)
                            if r3:
                                ch = '\x1b[' + sys.stdin.read(1)
                            else:
                                ch = '\x1b'
                        else:
                            ch = '\x1b'
                    else:
                        ch = '\x1b'   # lone ESC → thoát

                self._handle_key(ch)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

    # ----------------------------------------------------------------
    def _handle_key(self, key: str):
        # --- Chuyển chế độ ---
        if key == '\t':
            if self.mode == 'AUTO':
                self.mode     = 'MANUAL'
                self.speed    = 0.0
                self.steering = 0.0
            else:
                self.mode = 'AUTO'
            self._print_state()
            return

        # --- Thoát ---
        if key in ('q', 'Q', '\x1b'):
            self._running = False
            stop = Control()
            self.cmd_pub.publish(stop)
            rclpy.shutdown()
            return

        # --- Phím điều khiển (chỉ MANUAL) ---
        if self.mode != 'MANUAL':
            return

        if key in ('w', 'W', '\x1b[A'):
            self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
        elif key in ('s', 'S', '\x1b[B'):
            self.speed = max(self.speed - SPEED_STEP, 0.0)
        elif key in ('a', 'A', '\x1b[D'):
            self.steering = max(self.steering - STEER_STEP, -MAX_STEER)
        elif key in ('d', 'D', '\x1b[C'):
            self.steering = min(self.steering + STEER_STEP, MAX_STEER)
        elif key == ' ':
            self.speed    = 0.0
            self.steering = 0.0

        self._print_state()

    # ----------------------------------------------------------------
    def _print_state(self):
        if self.mode == 'AUTO':
            spd = getattr(self.auto_cmd, 'speed',    0.0)
            st  = getattr(self.auto_cmd, 'steering', 0.0)
            print(f'\r  [AUTO  ]  spd={spd:.2f}  steer={st:+.1f}deg    ',
                  end='', flush=True)
        else:
            bar = int(self.speed / MAX_SPEED * 20)
            print(f'\r  [MANUAL]  spd=[{"#"*bar:<20}]{self.speed:.2f}  steer={self.steering:+.0f}deg  ',
                  end='', flush=True)


# ----------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TeleopMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
