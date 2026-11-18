import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from std_srvs.srv import Trigger
import serial

from drill_arm_interfaces.action import Home, MoveY, DrillToDepth


class DrillArmBridge(Node):

    def __init__(self):
        super().__init__('drill_arm_bridge')

        # ── Serial port ──────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.serial_lock = threading.Lock()

        # Shared state updated by the reader thread
        self.status = {}
        self.status_event = threading.Event()

        # ── Topics ───────────────────────────────────────────────────────────
        # Publishes the raw STATUS line so the CLI and any other node can read it
        self.status_pub = self.create_publisher(String, '/drill_arm/status', 10)

        # Raw command topic — anything published here is sent straight to Teensy
        self.create_subscription(String, '/drill_arm/command', self._cmd_cb, 10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, '/drill_arm/stop',        self._stop_cb)
        self.create_service(Trigger, '/drill_arm/estop',       self._estop_cb)
        self.create_service(Trigger, '/drill_arm/clear_fault', self._clear_fault_cb)
        self.create_service(Trigger, '/drill_arm/tare',        self._tare_cb)
        # BOOT = home -> tare -> baseline calibration, all sequenced on the Teensy.
        # Fire-and-forget: watch HOMING= / CALIBRATING= on /drill_arm/status for progress.
        self.create_service(Trigger, '/drill_arm/boot',        self._boot_cb)
        # SET_HOME forces the current position to be Y=0 without moving.
        self.create_service(Trigger, '/drill_arm/set_home',    self._set_home_cb)

        # ── Action servers ───────────────────────────────────────────────────
        self._home_as  = ActionServer(self, Home,         '/drill_arm/home',          self._home_execute)
        self._move_as  = ActionServer(self, MoveY,        '/drill_arm/move_y',        self._move_execute)
        self._drill_as = ActionServer(self, DrillToDepth, '/drill_arm/drill_to_depth',self._drill_execute)

        # ── Background serial reader thread ───────────────────────────────────
        self._running = True
        threading.Thread(target=self._serial_reader, daemon=True).start()

        # Poll STATUS every second so the topic stays live
        self.create_timer(1.0, lambda: self._send('STATUS'))

        self.get_logger().info(f'Drill arm bridge started on {port}')

    # ── Serial helpers ───────────────────────────────────────────────────────

    def _send(self, cmd: str):
        with self.serial_lock:
            self.ser.write((cmd + '\n').encode())

    def _serial_reader(self):
        while self._running:
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                if raw.startswith('STATUS'):
                    self.status = self._parse_status(raw)
                    msg = String()
                    msg.data = raw
                    self.status_pub.publish(msg)
                    self.status_event.set()
                elif raw.startswith('ERR'):
                    self.get_logger().warn(f'Teensy: {raw}')
                elif raw.startswith('READY') or raw.startswith('INFO'):
                    # INFO lines: AUTO_TARE_AFTER_HOME, BOOT_TARE_DONE, BASELINE_CAL_* progress
                    self.get_logger().info(f'Teensy: {raw}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')

    @staticmethod
    def _parse_status(line: str) -> dict:
        result = {}
        for m in re.finditer(r'(\w+)=([\-\d.]+)', line):
            try:
                result[m.group(1)] = float(m.group(2))
            except ValueError:
                result[m.group(1)] = m.group(2)
        return result

    def _wait_for_status(self, timeout=3.0):
        self.status_event.clear()
        self._send('STATUS')
        self.status_event.wait(timeout=timeout)

    # ── Topic callback ───────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        self._send(msg.data)

    # ── Service callbacks ────────────────────────────────────────────────────

    def _stop_cb(self, _, response):
        self._send('STOP')
        response.success = True
        response.message = 'STOP sent'
        return response

    def _estop_cb(self, _, response):
        self._send('ESTOP')
        response.success = True
        response.message = 'ESTOP sent'
        return response

    def _clear_fault_cb(self, _, response):
        self._send('CLEAR_FAULT')
        response.success = True
        response.message = 'CLEAR_FAULT sent'
        return response

    def _tare_cb(self, _, response):
        self._send('TARE')
        response.success = True
        response.message = 'TARE sent'
        return response

    def _boot_cb(self, _, response):
        self._send('BOOT')
        response.success = True
        response.message = 'BOOT sequence started (home -> tare -> baseline calibration)'
        return response

    def _set_home_cb(self, _, response):
        self._send('SET_HOME')
        response.success = True
        response.message = 'SET_HOME sent — current position is now Y=0'
        return response

    # ── Action: HOME ─────────────────────────────────────────────────────────

    def _home_execute(self, goal_handle):
        self._send('HOME')
        feedback = Home.Feedback()

        while rclpy.ok():
            self._wait_for_status()
            s = self.status

            feedback.current_mm = float(s.get('Y', 0))
            if s.get('CALIBRATING', 0):
                feedback.phase = 'calibrating_baseline'
            elif s.get('HOMING', 0):
                feedback.phase = 'homing'
            else:
                feedback.phase = 'approach'
            goal_handle.publish_feedback(feedback)

            if s.get('FAULT_ACTIVE', 0) == 1:
                goal_handle.abort()
                result = Home.Result()
                result.success = False
                result.message = f"Fault {int(s.get('FAULT', 0))}"
                return result

            if s.get('HOMED', 0) == 1 and s.get('HOMING', 0) == 0 and s.get('CALIBRATING', 0) == 0:
                break

        goal_handle.succeed()
        result = Home.Result()
        result.success = True
        result.message = 'Homed — Y=0'
        return result

    # ── Action: MOVE Y ───────────────────────────────────────────────────────

    def _move_execute(self, goal_handle):
        target = goal_handle.request.target_mm
        speed  = goal_handle.request.speed_mm_s
        self._send(f'MOVE Y={target:.1f} SPEED={speed:.1f}')

        feedback = MoveY.Feedback()

        while rclpy.ok():
            self._wait_for_status()
            s = self.status

            feedback.current_mm = float(s.get('Y', 0))
            feedback.moving     = bool(s.get('MOVING', 0))
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                self._send('STOP')
                goal_handle.canceled()
                result = MoveY.Result()
                result.success  = False
                result.message  = 'Cancelled'
                result.final_mm = feedback.current_mm
                return result

            if s.get('FAULT_ACTIVE', 0) == 1:
                goal_handle.abort()
                result = MoveY.Result()
                result.success  = False
                result.message  = f"Fault {int(s.get('FAULT', 0))}"
                result.final_mm = feedback.current_mm
                return result

            if s.get('MOVING', 0) == 0 and s.get('HOMING', 0) == 0:
                break

        goal_handle.succeed()
        result = MoveY.Result()
        result.success  = True
        result.message  = 'Move complete'
        result.final_mm = float(self.status.get('Y', 0))
        return result

    # ── Action: DRILL TO DEPTH ───────────────────────────────────────────────

    def _drill_execute(self, goal_handle):
        depth = goal_handle.request.depth_mm
        speed = goal_handle.request.speed_mm_s
        pct   = goal_handle.request.drill_percent

        self._send(f'DRILL PERCENT={pct}')
        time.sleep(0.5)
        self._send(f'MOVE Y={depth:.1f} SPEED={speed:.1f}')

        feedback = DrillToDepth.Feedback()
        feedback.drill_percent = pct

        while rclpy.ok():
            self._wait_for_status()
            s = self.status

            feedback.current_mm = float(s.get('Y', 0))
            feedback.force_g    = float(s.get('FORCE_G', 0))
            feedback.moving     = bool(s.get('MOVING', 0))
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                self._send('STOP')
                self._send('DRILL PERCENT=0')
                goal_handle.canceled()
                result = DrillToDepth.Result()
                result.success      = False
                result.message      = 'Cancelled'
                result.final_mm     = feedback.current_mm
                result.final_force_g = feedback.force_g
                return result

            fault = int(s.get('FAULT', 0))

            # Ground contact = success
            if fault == 9:
                self._send('DRILL PERCENT=0')
                goal_handle.succeed()
                result = DrillToDepth.Result()
                result.success      = True
                result.message      = 'Ground contact reached'
                result.final_mm     = feedback.current_mm
                result.final_force_g = feedback.force_g
                return result

            if s.get('FAULT_ACTIVE', 0) == 1:
                self._send('DRILL PERCENT=0')
                goal_handle.abort()
                result = DrillToDepth.Result()
                result.success      = False
                result.message      = f'Fault {fault}'
                result.final_mm     = feedback.current_mm
                result.final_force_g = feedback.force_g
                return result

            if s.get('MOVING', 0) == 0:
                self._send('DRILL PERCENT=0')
                break

        goal_handle.succeed()
        result = DrillToDepth.Result()
        result.success      = True
        result.message      = 'Drill complete'
        result.final_mm     = float(self.status.get('Y', 0))
        result.final_force_g = float(self.status.get('FORCE_G', 0))
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DrillArmBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
