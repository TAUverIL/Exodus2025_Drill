import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drill_arm_interfaces.action import Home, MoveY, DrillToDepth


class DrillArmCLI(Node):

    def __init__(self):
        super().__init__('drill_arm_cli')

        # ── Service clients ──────────────────────────────────────────────────
        self.stop_cli        = self.create_client(Trigger, '/drill_arm/stop')
        self.estop_cli       = self.create_client(Trigger, '/drill_arm/estop')
        self.clear_fault_cli = self.create_client(Trigger, '/drill_arm/clear_fault')
        self.tare_cli        = self.create_client(Trigger, '/drill_arm/tare')
        self.boot_cli        = self.create_client(Trigger, '/drill_arm/boot')
        self.set_home_cli    = self.create_client(Trigger, '/drill_arm/set_home')

        # ── Action clients ───────────────────────────────────────────────────
        self.home_ac  = ActionClient(self, Home,         '/drill_arm/home')
        self.move_ac  = ActionClient(self, MoveY,        '/drill_arm/move_y')
        self.drill_ac = ActionClient(self, DrillToDepth, '/drill_arm/drill_to_depth')

        # ── Status subscriber ────────────────────────────────────────────────
        self.last_status = 'No status yet'
        self.create_subscription(String, '/drill_arm/status', self._status_cb, 10)

    def _status_cb(self, msg: String):
        self.last_status = msg.data

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _wait(self, future, timeout=10.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                print('  Timed out waiting for response')
                return False
            time.sleep(0.05)
        return True

    def _call_service(self, client, label):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f'  {label}: service not available')
            return
        future = client.call_async(Trigger.Request())
        if self._wait(future, timeout=5.0) and future.result():
            print(f'  {label}: {future.result().message}')

    def _ask_float(self, prompt, lo=None, hi=None):
        while True:
            try:
                val = float(input(f'  {prompt}: '))
                if lo is not None and val < lo:
                    print(f'  Must be >= {lo}'); continue
                if hi is not None and val > hi:
                    print(f'  Must be <= {hi}'); continue
                return val
            except ValueError:
                print('  Enter a number')

    def _ask_int(self, prompt, lo=None, hi=None):
        while True:
            try:
                val = int(input(f'  {prompt}: '))
                if lo is not None and val < lo:
                    print(f'  Must be >= {lo}'); continue
                if hi is not None and val > hi:
                    print(f'  Must be <= {hi}'); continue
                return val
            except ValueError:
                print('  Enter a whole number')

    # ── CLI main loop ────────────────────────────────────────────────────────

    def run(self):
        print('\n╔══════════════════════════════╗')
        print('║   Drill Arm CLI              ║')
        print('╚══════════════════════════════╝')
        print('Commands: boot | home | sethome | move | drill | stop | estop | tare | clear | status | quit\n')

        while rclpy.ok():
            try:
                cmd = input('> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\nExiting.')
                break

            if not cmd:
                continue

            # ── STATUS ───────────────────────────────────────────────────────
            elif cmd == 'status':
                print(f'  {self.last_status}')

            # ── STOP ─────────────────────────────────────────────────────────
            elif cmd == 'stop':
                self._call_service(self.stop_cli, 'STOP')

            # ── ESTOP ────────────────────────────────────────────────────────
            elif cmd == 'estop':
                self._call_service(self.estop_cli, 'ESTOP')

            # ── TARE ─────────────────────────────────────────────────────────
            elif cmd == 'tare':
                self._call_service(self.tare_cli, 'TARE')

            # ── BOOT (home + tare + baseline calibration) ────────────────────
            elif cmd == 'boot':
                print('  Starting BOOT sequence: home -> tare -> baseline calibration.')
                print('  Watch `status` — HOMING=1 then CALIBRATING=1 until done (~2 min).')
                self._call_service(self.boot_cli, 'BOOT')

            # ── SET HOME (force current position = Y0) ───────────────────────
            elif cmd == 'sethome':
                confirm = input('  Force CURRENT position to be Y=0? (y/n): ').strip().lower()
                if confirm == 'y':
                    self._call_service(self.set_home_cli, 'SET_HOME')
                else:
                    print('  Cancelled.')

            # ── CLEAR FAULT ──────────────────────────────────────────────────
            elif cmd == 'clear':
                self._call_service(self.clear_fault_cli, 'CLEAR_FAULT')

            # ── HOME ─────────────────────────────────────────────────────────
            elif cmd == 'home':
                print('  Homing...')
                self.home_ac.wait_for_server()

                def home_feedback(fb):
                    print(f'    phase={fb.feedback.phase}  Y={fb.feedback.current_mm:.1f}mm')

                future = self.home_ac.send_goal_async(Home.Goal(), feedback_callback=home_feedback)
                self._wait(future)
                gh = future.result()
                result_future = gh.get_result_async()
                self._wait(result_future, timeout=120.0)
                r = result_future.result().result
                print(f'  {"Done — homed at Y=0." if r.success else "Failed: " + r.message}')

            # ── MOVE ─────────────────────────────────────────────────────────
            elif cmd == 'move':
                target = self._ask_float('Target position (mm)', lo=0, hi=480)
                speed  = self._ask_float('Speed (mm/s)', lo=0.1, hi=25.0)

                print(f'  Moving to {target}mm at {speed}mm/s...')
                self.move_ac.wait_for_server()

                goal = MoveY.Goal()
                goal.target_mm  = target
                goal.speed_mm_s = speed

                def move_feedback(fb):
                    print(f'    Y={fb.feedback.current_mm:.1f}mm  moving={fb.feedback.moving}')

                future = self.move_ac.send_goal_async(goal, feedback_callback=move_feedback)
                self._wait(future)
                gh = future.result()
                result_future = gh.get_result_async()
                self._wait(result_future, timeout=300.0)
                r = result_future.result().result
                print(f'  {"Done. Final Y=" + str(round(r.final_mm, 1)) + "mm" if r.success else "Failed: " + r.message}')

            # ── DRILL ────────────────────────────────────────────────────────
            elif cmd == 'drill':
                depth = self._ask_float('Depth to drill to (mm)', lo=0, hi=480)
                speed = self._ask_float('Descent speed (mm/s)', lo=0.1, hi=25.0)
                pct   = self._ask_int('Drill motor percent (0-100)', lo=0, hi=100)

                print(f'  Drilling to {depth}mm at {speed}mm/s, drill={pct}%...')
                print('  Press Ctrl-C to cancel.')
                self.drill_ac.wait_for_server()

                goal = DrillToDepth.Goal()
                goal.depth_mm      = depth
                goal.speed_mm_s    = speed
                goal.drill_percent = pct

                def drill_feedback(fb):
                    f = fb.feedback
                    print(f'    Y={f.current_mm:.1f}mm  Force={f.force_g:.0f}g  drill={f.drill_percent}%')

                future = self.drill_ac.send_goal_async(goal, feedback_callback=drill_feedback)
                self._wait(future)
                gh = future.result()
                try:
                    result_future = gh.get_result_async()
                    self._wait(result_future, timeout=600.0)
                    r = result_future.result().result
                    if r.success:
                        print(f'  Done. Final Y={r.final_mm:.1f}mm  Force={r.final_force_g:.0f}g')
                    else:
                        print(f'  Failed: {r.message}')
                except KeyboardInterrupt:
                    print('\n  Cancelling...')
                    gh.cancel_goal_async()

            # ── QUIT ─────────────────────────────────────────────────────────
            elif cmd == 'quit':
                print('Bye.')
                break

            else:
                print('  Unknown command. Try: boot | home | sethome | move | drill | stop | estop | tare | clear | status | quit')


def main(args=None):
    rclpy.init(args=args)
    node = DrillArmCLI()

    # Spin in a background thread so ROS callbacks (status subscriber, action
    # feedback) are processed while the main thread is blocked on input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.run()
    node.destroy_node()
    rclpy.shutdown()
