
"""
Drill Arm Control GUI — runs on Mac, talks directly to the Teensy over USB serial.
No Jetson / ROS needed. Same commands as the firmware's serial protocol.

Install dependencies once:
    pip3 install pyserial matplotlib

Run:
    python3 drill_arm_gui.py
"""

import collections
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200
LOOP_SLEEP_S = 0.15      # pause between request/response cycles
HISTORY_LEN = 300        # points kept on the graphs

# Must match the FAULT_* codes in drill_arm_teensy_usb_serial.ino
FAULT_MESSAGES = {
    0: "None",
    1: "Top limit hit while moving up unexpectedly",
    2: "Move requested before homing",
    3: "Comms timeout — no command received in time",
    4: "Bad command — unrecognized or malformed",
    5: "ESTOP triggered",
    6: "Force limit exceeded (load cell)",
    7: "Homing timeout — limit switch never found",
    8: "ToF clearance — too close to ground while descending",
    9: "Ground contact — drill reached the surface",
}


def fault_text(code):
    try:
        code = int(code)
    except (TypeError, ValueError):
        return str(code)
    return f"{code} - {FAULT_MESSAGES.get(code, 'Unknown fault')}"


def bool_text(value):
    return "Yes" if value else "No"


def drill_text(percent):
    try:
        percent = int(percent)
    except (TypeError, ValueError):
        return str(percent)
    if percent == 0:
        return "0 - Stopped"
    direction = "forward" if percent > 0 else "reverse"
    return f"{percent} - Running {direction}"


def parse_status(line):
    """STATUS Y=1.234 TARGET=5.000 ... -> {'Y': 1.234, 'TARGET': 5.0, ...}"""
    result = {}
    for token in line.split():
        if "=" in token:
            key, val = token.split("=", 1)
            try:
                result[key] = float(val)
            except ValueError:
                result[key] = val
    return result


class SerialWorker(threading.Thread):
    """Owns the serial port exclusively. Sends one command per loop — either
    something queued by the GUI, or a STATUS poll if nothing is queued."""

    def __init__(self, port, cmd_queue, on_status, on_log):
        super().__init__(daemon=True)
        self.port = port
        self.cmd_queue = cmd_queue
        self.on_status = on_status
        self.on_log = on_log
        self._running = True
        self.ser = None

    def stop(self):
        self._running = False

    def run(self):
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=1.0)
        except Exception as e:
            self.on_log(f"ERR could not open {self.port}: {e}")
            return

        time.sleep(2.0)  # Teensy resets on USB-serial open; wait for boot
        self.ser.reset_input_buffer()
        self.on_log(f"Connected to {self.port}")

        while self._running:
            try:
                cmd = self.cmd_queue.get_nowait()
            except queue.Empty:
                cmd = "STATUS"

            try:
                self.ser.write((cmd + "\n").encode())
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:
                self.on_log(f"ERR serial: {e}")
                break

            if line.startswith("STATUS"):
                self.on_status(parse_status(line))
            elif line:
                self.on_log(line)

            time.sleep(LOOP_SLEEP_S)

        if self.ser:
            self.ser.close()
        self.on_log("Disconnected")


class DrillArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drill Arm Control")
        self.geometry("1150x720")

        self.cmd_queue = queue.Queue()
        self.worker = None

        self.latest_status = {}
        self.status_lock = threading.Lock()

        self.t0 = time.time()
        self.hist_t = collections.deque(maxlen=HISTORY_LEN)
        self.hist_y = collections.deque(maxlen=HISTORY_LEN)
        self.hist_force = collections.deque(maxlen=HISTORY_LEN)
        self.hist_drill = collections.deque(maxlen=HISTORY_LEN)

        self._build_ui()
        self.after(150, self._poll_ui)

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=32)
        self.port_combo.pack(side="left", padx=4)
        self._refresh_ports()

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=4)

        self.conn_status_lbl = ttk.Label(top, text="Disconnected", foreground="red")
        self.conn_status_lbl.pack(side="left", padx=12)

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=8, pady=6)

        # Scrollable left panel
        left_outer = ttk.Frame(body)
        left_outer.pack(side="left", fill="y", padx=(0, 8))
        left_canvas = tk.Canvas(left_outer, width=280, highlightthickness=0)
        left_scroll = ttk.Scrollbar(left_outer, orient="vertical", command=left_canvas.yview)
        left_canvas.configure(yscrollcommand=left_scroll.set)
        left_scroll.pack(side="right", fill="y")
        left_canvas.pack(side="left", fill="both", expand=True)
        left = ttk.Frame(left_canvas)
        left_canvas_window = left_canvas.create_window((0, 0), window=left, anchor="nw")

        def _on_left_configure(event):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))

        def _on_canvas_resize(event):
            left_canvas.itemconfig(left_canvas_window, width=event.width)

        left.bind("<Configure>", _on_left_configure)
        left_canvas.bind("<Configure>", _on_canvas_resize)

        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        left_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)

        self._build_controls(left)
        self._build_graphs(right)

        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.pack(fill="x", padx=8, pady=(0, 8))
        self.log_text = tk.Text(log_frame, height=6, state="disabled")
        self.log_text.pack(fill="x")

    def _build_controls(self, parent):
        status_frame = ttk.LabelFrame(parent, text="Status")
        status_frame.pack(fill="x", pady=4)

        self.status_vars = {}
        for key in ["Y", "TARGET", "SPEED", "DRILL", "FORCE_G", "TOF_MM",
                    "HOMED", "MOVING", "HOMING", "FAULT", "FAULT_ACTIVE", "CALIBRATING"]:
            row = ttk.Frame(status_frame)
            row.pack(fill="x")
            ttk.Label(row, text=key + ":", width=12).pack(side="left", anchor="n")
            var = tk.StringVar(value="--")
            self.status_vars[key] = var
            ttk.Label(row, textvariable=var, wraplength=220, justify="left").pack(side="left", anchor="n")

        self.fault_banner = tk.Label(parent, text="", fg="white", bg="red", wraplength=260, justify="left")
        self.fault_banner.pack(fill="x", pady=4)

        basic_frame = ttk.LabelFrame(parent, text="Commands")
        basic_frame.pack(fill="x", pady=4)

        ttk.Button(basic_frame, text="BOOT (home + tare + calibrate)",
                   command=lambda: self._send("BOOT")).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="HOME", command=lambda: self._send("HOME")).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="SET HOME (force here = 0)",
                   command=self._confirm_set_home).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="STOP", command=lambda: self._send("STOP")).pack(fill="x", pady=2)
        tk.Button(basic_frame, text="ESTOP", bg="red", fg="white",
                  command=lambda: self._send("ESTOP")).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="CLEAR FAULT", command=lambda: self._send("CLEAR_FAULT")).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="TARE LOAD CELL", command=lambda: self._send("TARE")).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="🎵 IMPERIAL MARCH", command=lambda: self._send("TUNE")).pack(fill="x", pady=2)
        ttk.Button(basic_frame, text="🎵 HATIKVAH", command=lambda: self._send("TUNE HATIKVAH")).pack(fill="x", pady=2)

        move_frame = ttk.LabelFrame(parent, text="Move")
        move_frame.pack(fill="x", pady=4)
        ttk.Label(move_frame, text="Target (mm):").pack(anchor="w")
        self.move_target_var = tk.StringVar(value="0")
        ttk.Entry(move_frame, textvariable=self.move_target_var).pack(fill="x")
        ttk.Label(move_frame, text="Speed (mm/s):").pack(anchor="w")
        self.move_speed_var = tk.StringVar(value="1.0")
        ttk.Entry(move_frame, textvariable=self.move_speed_var).pack(fill="x")
        ttk.Button(move_frame, text="MOVE", command=self._do_move).pack(fill="x", pady=4)

        jog_y_row = ttk.Frame(move_frame)
        jog_y_row.pack(fill="x", pady=2)
        ttk.Button(jog_y_row, text="-50", command=lambda: self._jog_y(-50)).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(jog_y_row, text="+50", command=lambda: self._jog_y(50)).pack(side="left", expand=True, fill="x", padx=(2, 0))

        drill_frame = ttk.LabelFrame(parent, text="Drill")
        drill_frame.pack(fill="x", pady=4)
        ttk.Label(drill_frame, text="Percent (-100..100):").pack(anchor="w")
        self.drill_pct_var = tk.StringVar(value="0")
        ttk.Entry(drill_frame, textvariable=self.drill_pct_var).pack(fill="x")
        ttk.Button(drill_frame, text="SET DRILL", command=self._do_drill).pack(fill="x", pady=2)
        ttk.Button(drill_frame, text="DRILL STOP (0%)",
                   command=lambda: self._send("DRILL PERCENT=0")).pack(fill="x", pady=2)

        jog_drill_row = ttk.Frame(drill_frame)
        jog_drill_row.pack(fill="x", pady=2)
        ttk.Button(jog_drill_row, text="-10%", command=lambda: self._jog_drill(-10)).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(jog_drill_row, text="+10%", command=lambda: self._jog_drill(10)).pack(side="left", expand=True, fill="x", padx=(2, 0))

    def _build_graphs(self, parent):
        fig = Figure(figsize=(6, 6))
        self.ax_y = fig.add_subplot(311)
        self.ax_force = fig.add_subplot(312)
        self.ax_drill = fig.add_subplot(313)

        self.ax_y.set_ylabel("Y (mm)")
        self.ax_force.set_ylabel("Force (g)")
        self.ax_drill.set_ylabel("Drill %")
        self.ax_drill.set_xlabel("Time (s)")

        (self.line_y,) = self.ax_y.plot([], [], color="tab:blue")
        (self.line_force,) = self.ax_force.plot([], [], color="tab:red")
        (self.line_drill,) = self.ax_drill.plot([], [], color="tab:green")

        fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ── Connection ─────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            for p in ports:
                if "usbmodem" in p:
                    self.port_var.set(p)
                    break
            else:
                self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.worker is None:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Select a port first")
                return
            self.worker = SerialWorker(port, self.cmd_queue, self._on_status, self._on_log)
            self.worker.start()
            self.connect_btn.config(text="Disconnect")
            self.conn_status_lbl.config(text=f"Connected: {port}", foreground="green")
        else:
            self.worker.stop()
            self.worker = None
            self.connect_btn.config(text="Connect")
            self.conn_status_lbl.config(text="Disconnected", foreground="red")

    # ── Commands ─────────────────────────────────────────────────────────

    def _send(self, cmd):
        if self.worker is None:
            messagebox.showwarning("Not connected", "Connect to the Teensy first")
            return
        self.cmd_queue.put(cmd)
        self._append_log(f"> {cmd}")

    def _confirm_set_home(self):
        if messagebox.askyesno(
            "Set Home",
            "This forces the CURRENT position to be Y=0 (home).\n"
            "Only use this if the arm is physically at the correct home position.\n\n"
            "Continue?",
        ):
            self._send("SET_HOME")

    def _do_move(self):
        try:
            target = float(self.move_target_var.get())
            speed = float(self.move_speed_var.get())
        except ValueError:
            messagebox.showerror("Error", "Target and speed must be numbers")
            return
        self._send(f"MOVE Y={target:.2f} SPEED={speed:.2f}")

    def _do_drill(self):
        try:
            pct = int(self.drill_pct_var.get())
        except ValueError:
            messagebox.showerror("Error", "Percent must be an integer")
            return
        self._send(f"DRILL PERCENT={pct}")

    def _jog_y(self, delta):
        with self.status_lock:
            current_y = self.latest_status.get("Y")
        if current_y is None:
            try:
                current_y = float(self.move_target_var.get())
            except ValueError:
                current_y = 0.0
        new_target = current_y + delta
        self.move_target_var.set(f"{new_target:.2f}")
        self._do_move()

    def _jog_drill(self, delta):
        try:
            current = int(self.drill_pct_var.get())
        except ValueError:
            current = 0
        new_pct = max(-100, min(100, current + delta))
        self.drill_pct_var.set(str(new_pct))
        self._do_drill()

    # ── Worker callbacks (called from the worker thread) ──────────────────

    def _on_status(self, status_dict):
        with self.status_lock:
            self.latest_status = status_dict

    def _on_log(self, line):
        self.after(0, self._append_log, line)

    def _append_log(self, line):
        self.log_text.config(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ── Periodic UI refresh (main thread) ──────────────────────────────────

    def _poll_ui(self):
        with self.status_lock:
            status = dict(self.latest_status)

        if status:
            for key, var in self.status_vars.items():
                if key in status:
                    if key == "FAULT":
                        var.set(fault_text(status[key]))
                    elif key == "DRILL":
                        var.set(drill_text(status[key]))
                    elif key in ("HOMED", "MOVING", "HOMING", "FAULT_ACTIVE"):
                        var.set(bool_text(status[key]))
                    else:
                        var.set(str(status[key]))

            if status.get("FAULT_ACTIVE", 0):
                self.fault_banner.config(text=f"FAULT ACTIVE — {fault_text(status.get('FAULT', -1))}")
            else:
                self.fault_banner.config(text="")

            t = time.time() - self.t0
            self.hist_t.append(t)
            self.hist_y.append(status.get("Y", 0))
            self.hist_force.append(status.get("FORCE_G", 0))
            self.hist_drill.append(status.get("DRILL", 0))

            self.line_y.set_data(self.hist_t, self.hist_y)
            self.line_force.set_data(self.hist_t, self.hist_force)
            self.line_drill.set_data(self.hist_t, self.hist_drill)

            for ax, ydata in (
                (self.ax_y, self.hist_y),
                (self.ax_force, self.hist_force),
                (self.ax_drill, self.hist_drill),
            ):
                ax.relim()
                ax.autoscale_view()
                if self.hist_t:
                    ax.set_xlim(max(0, self.hist_t[-1] - 30), self.hist_t[-1] + 1)

            self.canvas.draw_idle()

        self.after(150, self._poll_ui)

    def on_close(self):
        if self.worker is not None:
            self.worker.stop()
        self.destroy()


def main():
    app = DrillArmGUI()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()


if __name__ == "__main__":
    main()
