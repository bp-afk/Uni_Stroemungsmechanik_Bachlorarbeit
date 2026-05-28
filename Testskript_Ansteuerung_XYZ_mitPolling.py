import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, time, os
import serial
import queue
import pandas as pd
import numpy as np

# DAQ import wie am Anfang!
from mcculw import ul
from mcculw.enums import ULRange

ARDUINO_PORT = 'COM6'           
BAUD = 115200
DAQ_BOARD = 0
DAQ_CH = 0

def voltage_to_airflow(voltage):
    return (3.6834 * voltage ** 3
            - 17.79 * voltage ** 2
            + 29.777 * voltage
            - 17.358)

def daq_voltage():
    adc_value = ul.a_in(DAQ_BOARD, DAQ_CH, ULRange.BIP5VOLTS)
    # Mittelstellung: 12 Bit => Wertebereich -2047.5 ... +2047.5
    return (adc_value - 2047.5) * (5/2047.5)

class UsbDatalogger:
    BURST_SAMPLES = 8
    BURST_DELAY_S = 0.01

    def __init__(self, board_num=0, channel=0, ul_range=ULRange.BIP5VOLTS):
        self.board_num = board_num
        self.channel = channel
        self.ul_range = ul_range

    def read_raw(self):
        return ul.a_in(self.board_num, self.channel, self.ul_range)

    def raw_to_voltage(self, adc_value, n_bits=12, v_range=5):
        max_adc = (2 ** n_bits) - 1
        half_scale = max_adc / 2
        return (adc_value - half_scale) * (v_range / half_scale)

    def read_once(self):
        raw = self.read_raw()
        voltage = self.raw_to_voltage(raw)
        return raw, voltage

    def read_burst(self, samples=BURST_SAMPLES, delay_s=BURST_DELAY_S):
        vals = []
        last_raw = None
        last_voltage = None
        for _ in range(max(1, samples)):
            raw, voltage = self.read_once()
            last_raw = raw
            last_voltage = voltage
            vals.append(voltage)
            if delay_s > 0:
                time.sleep(delay_s)
        return last_raw, last_voltage, float(np.mean(vals))

    def close(self):
        pass

class MotorController:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lock = threading.Lock()
        self.queue = queue.Queue()
        self.stop_event = threading.Event()
        threading.Thread(target=self.reader_thread, daemon=True).start()

    def send(self, cmd):
        with self.lock:
            if not self.ser.is_open:
                return
            self.ser.write((cmd + '\n').encode())
            time.sleep(0.01)

    def reader_thread(self):
        while not self.stop_event.is_set():
            try:
                msg = self.ser.readline().decode(errors="ignore").strip()
                if msg.startswith("POS"):
                    self.queue.put(msg)
            except Exception:
                if self.stop_event.is_set():
                    break

    def get_pos(self):
        last = None
        while not self.queue.empty():
            last = self.queue.get_nowait()
        if last:
            parts = last.split()
            return int(parts[1]), int(parts[2])
        return None, None

    def close(self):
        self.stop_event.set()
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

class ToggleButton(ttk.Button):
    def __init__(self, master, text_on, text_off, command=None, initial_state=False, *args, **kwargs):
        super().__init__(master, text=text_on, *args, **kwargs)
        self.state = initial_state  # True = gehalten/ENABLE, False = stromlos/IDLE
        self.text_on = text_on
        self.text_off = text_off
        self.command = command
        self.configure(command=self.toggle)
        self.update_text()

    def toggle(self):
        self.state = not self.state
        self.update_text()
        if self.command:
            self.command(self.state)

    def set_state(self, enabled):
        self.state = enabled
        self.update_text()
        if self.command:
            self.command(self.state)

    def update_text(self):
        self.configure(text=self.text_on if self.state else self.text_off)

    def set_disabled(self, mode=True):
        self.state = True if mode else self.state
        self.configure(state=("disabled" if mode else "normal"))
        self.update_text()

class MeasurementGUI(tk.Tk):
    LOGGER_IDLE_SLEEP_S = 0.02
    LOGGER_JOIN_TIMEOUT_S = 1.0

    def __init__(self):
        super().__init__()
        self.title("Strömungsmessung XY-Datalogger")

        # Parameter
        self.x_max = tk.DoubleVar(value=750)
        self.y_max = tk.DoubleVar(value=750)
        self.z_pos = tk.DoubleVar(value=375)
        self.step_size = tk.DoubleVar(value=10)
        self.speed = tk.IntVar(value=600)
        self.accel = tk.IntVar(value=200)
        self.pulses_per_rev = tk.IntVar(value=200)
        self.mm_per_rev = tk.DoubleVar(value=5)
        self.start_x = tk.DoubleVar(value=375)
        self.start_y = tk.DoubleVar(value=375)

        self.ablaufart = tk.StringVar(value="KREUZ")
        self.messmodus = tk.StringVar(value="PUNKTE")
        self.save_dir = tk.StringVar(value=os.getcwd())

        self.mot = MotorController(ARDUINO_PORT, BAUD)
        self.datalogger = UsbDatalogger(board_num=DAQ_BOARD, channel=DAQ_CH, ul_range=ULRange.BIP5VOLTS)
        self.data = []
        self.data_lock = threading.Lock()
        self.automatik_stop = threading.Event()
        self.measure_thread = None
        self.logger_thread = None
        self.logger_stop = threading.Event()
        self.log_interval_s = 0.1
        self.move_in_progress_evt = threading.Event()
        self.target_lock = threading.Lock()
        self.current_target = (self.start_x.get(), self.start_y.get(), self.z_pos.get())

        self.moving_x = False
        self.moving_y = False
        self.automatik_running = False

        self.build_gui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def build_gui(self):
        left = ttk.Frame(self); left.pack(side=tk.LEFT, fill="y")
        right = ttk.Frame(self); right.pack(side=tk.RIGHT, fill="both", expand=True)
        # Parameterfeld per grid
        param = ttk.LabelFrame(left, text="Parameter Eingabe")
        param.pack(padx=5, pady=5, fill="x")
        row = 0
        self._make_entry(param, "X-Strecke [mm]:", self.x_max, row); row += 1
        self._make_entry(param, "Y-Strecke [mm]:", self.y_max, row); row += 1
        self._make_entry(param, "Z-lvl (Hand):", self.z_pos, row); row += 1
        self._make_entry(param, "Schrittweite [mm]:", self.step_size, row); row += 1
        self._make_entry(param, "Speed [steps/s]:", self.speed, row); row += 1
        self._make_entry(param, "Accel [steps/s²]:", self.accel, row); row += 1
        self._make_entry(param, "Pulse/rev:", self.pulses_per_rev, row); row += 1
        self._make_entry(param, "mm/rev:", self.mm_per_rev, row); row += 1
        self._make_entry(param, "Start X [mm]:", self.start_x, row); row += 1
        self._make_entry(param, "Start Y [mm]:", self.start_y, row); row += 1
        ttk.Button(param, text="Wähle Speicherordner", command=self.choose_dir).grid(row=row, column=0, columnspan=2, sticky="ew", pady=2)

        # Ablauf- und Messmodus
        auswahl = ttk.LabelFrame(left, text="Ablaufart / Messmodus")
        auswahl.pack(padx=5, pady=5, fill="x")
        ttk.Label(auswahl, text="Abfolge:").pack(anchor="w")
        for name in [("Kreuz", "KREUZ"), ("Schlange", "SCHLANGE")]:
            ttk.Radiobutton(auswahl, text=name[0], variable=self.ablaufart, value=name[1]).pack(anchor="w")
        ttk.Label(auswahl, text="Messart:").pack(anchor="w")
        for name in [("Immer an Punkten stoppen", "PUNKTE"), ("Kontinuierlich während Bewegung", "KONTI")]:
            ttk.Radiobutton(auswahl, text=name[0], variable=self.messmodus, value=name[1]).pack(anchor="w")

        # Motoren-Buttons
        motorlf = ttk.LabelFrame(left, text="Motoren (stromlos/halten)")
        motorlf.pack(padx=5,pady=5,fill="x")
        self.tog_x = ToggleButton(motorlf, "X wird gehalten", "X ist stromlos", command=self.update_mot_x, initial_state=False)
        self.tog_x.pack(fill="x",pady=1)
        self.tog_y = ToggleButton(motorlf, "Y wird gehalten", "Y ist stromlos", command=self.update_mot_y, initial_state=False)
        self.tog_y.pack(fill="x",pady=1)

        # Manuelle Pfeilsteuerung
        handlf = ttk.LabelFrame(left, text="Manuelle Bewegung")
        handlf.pack(padx=5,pady=5,fill="x")
        self.create_arrow_pad(handlf)

        # Automatik
        auto = ttk.LabelFrame(left, text="Automatische Steuerung")
        auto.pack(padx=5,pady=5,fill="x")
        ttk.Button(auto, text="Start Automatik", command=self.start_automatik).pack(fill="x",pady=2)
        ttk.Button(auto, text="Stop Automatik", command=self.abort_automatik).pack(fill="x")
        self.status = tk.StringVar()
        ttk.Label(auto, textvariable=self.status).pack(fill="x",pady=2)

        # Rechts: Anzeige & Export
        self.listbox = tk.Listbox(right, width=60, font=("Consolas",10))
        self.listbox.pack(fill="both", expand=True, padx=8, pady=8)
        ttk.Button(right, text="Exportiere CSV", command=self.export_csv).pack(fill="x", pady=3)

    def _make_entry(self, frame, label, var, row):
        ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=var, width=8).grid(row=row, column=1, sticky="w")

    def choose_dir(self):
        newdir = filedialog.askdirectory()
        if newdir: self.save_dir.set(newdir)

    def _ui_call(self, fn, *args, **kwargs):
        if threading.current_thread() is threading.main_thread():
            fn(*args, **kwargs)
        else:
            self.after(0, lambda fn=fn, args=args, kwargs=kwargs: fn(*args, **kwargs))

    def _set_status(self, text):
        self._ui_call(self.status.set, text)

    def _set_toggle_enabled(self, enabled):
        self._ui_call(self.tog_x.set_disabled, not enabled)
        self._ui_call(self.tog_y.set_disabled, not enabled)

    def apply_motion_settings(self):
        spd = int(self.speed.get())
        acc = int(self.accel.get())
        for axis in ("X", "Y"):
            self.mot.send(f"SPEED {axis} {spd}")
            self.mot.send(f"ACCEL {axis} {acc}")

    def _set_target(self, x, y, z):
        with self.target_lock:
            self.current_target = (x, y, z)

    # Update Motorstatus mit ToggleButton
    def update_mot_x(self, enabled):
        self.mot.send("ENABLE X" if enabled else "IDLE X")

    def update_mot_y(self, enabled):
        self.mot.send("ENABLE Y" if enabled else "IDLE Y")

    def create_arrow_pad(self, root):
        f = ttk.Frame(root); f.pack()
        up = ttk.Button(f, text="↑", width=5)
        dn = ttk.Button(f, text="↓", width=5)
        lt = ttk.Button(f, text="←", width=5)
        rt = ttk.Button(f, text="→", width=5)
        up.grid(row=0,column=1); dn.grid(row=2,column=1); lt.grid(row=1,column=0); rt.grid(row=1,column=2)
        up.bind('<ButtonPress-1>', lambda e:self.start_move('y+', up))
        up.bind('<ButtonRelease-1>', lambda e:self.end_move('y'))
        dn.bind('<ButtonPress-1>', lambda e:self.start_move('y-', dn))
        dn.bind('<ButtonRelease-1>', lambda e:self.end_move('y'))
        lt.bind('<ButtonPress-1>', lambda e:self.start_move('x-', lt))
        lt.bind('<ButtonRelease-1>', lambda e:self.end_move('x'))
        rt.bind('<ButtonPress-1>', lambda e:self.start_move('x+', rt))
        rt.bind('<ButtonRelease-1>', lambda e:self.end_move('x'))

    def start_move(self, direction, btn):
        axis = direction[0]
        sign = 1 if direction[1] == '+' else -1
        if axis == "x" and not self.moving_x:
            self.moving_x = True
            threading.Thread(target=self._move_hold, args=("x",sign), daemon=True).start()
        elif axis == "y" and not self.moving_y:
            self.moving_y = True
            threading.Thread(target=self._move_hold, args=("y",sign), daemon=True).start()

    def end_move(self, axis):
        if axis == "x":
            self.moving_x = False
            # Nur stromlos, wenn Button auf stromlos steht
            if not self.tog_x.state:
                self.mot.send("IDLE X")
            else:
                self.mot.send("ENABLE X")
        elif axis == "y":
            self.moving_y = False
            if not self.tog_y.state:
                self.mot.send("IDLE Y")
            else:
                self.mot.send("ENABLE Y")

    def _move_hold(self, axis, sign):
        self.apply_motion_settings()
        step = int(sign * float(self.step_size.get()) * float(self.pulses_per_rev.get()) / float(self.mm_per_rev.get()))
        while (self.moving_x if axis=="x" else self.moving_y):
            # Immer nur schicken, wenn auf "halten"
            if (axis == "x" and self.tog_x.state) or (axis == "y" and self.tog_y.state):
                self.mot.send(f"ENABLE {axis.upper()}")
            move_cmd = f"MOVE {axis.upper()} {step}"
            self.mot.send(move_cmd)
            time.sleep(0.04)

    def start_automatik(self):
        if self.measure_thread is not None and self.measure_thread.is_alive():
            messagebox.showerror("Fehler","Messlauf läuft bereits!")
            return
        # Während Automatik Toggle-Buttons "halten" + blockiert
        self.tog_x.set_state(True)
        self.tog_x.set_disabled(True)
        self.tog_y.set_state(True)
        self.tog_y.set_disabled(True)
        self.data.clear()
        self.listbox.delete(0, tk.END)
        self.automatik_stop.clear()
        self.logger_stop.clear()
        self.automatik_running = True
        self.apply_motion_settings()
        self.measure_thread = threading.Thread(target=self.automatik, daemon=True)
        self.measure_thread.start()

    def abort_automatik(self):
        self.automatik_stop.set()
        self.logger_stop.set()
        self._set_status("Automatik abgebrochen!")
        self.automatik_running = False
        self._set_toggle_enabled(True)

    def goto_pos(self, x, y, timeout_s=60, poll_s=0.05):
        s_x = int(x * float(self.pulses_per_rev.get()) / float(self.mm_per_rev.get()))
        s_y = int(y * float(self.pulses_per_rev.get()) / float(self.mm_per_rev.get()))
        self._set_target(x, y, self.z_pos.get())
        self.mot.send(f"ENABLE X"); self.mot.send(f"ENABLE Y")
        self.mot.send(f"GOTO X {s_x}"); self.mot.send(f"GOTO Y {s_y}")
        self.move_in_progress_evt.set()
        t_end = time.time() + timeout_s
        act_x, act_y = None, None
        while time.time() < t_end:
            self.mot.send("POS?")
            act_x, act_y = self.mot.get_pos()
            if act_x is not None and abs(act_x-s_x)<3 and abs(act_y-s_y)<3:
                self.move_in_progress_evt.clear()
                return True
            if self.automatik_stop.is_set(): 
                self.move_in_progress_evt.clear()
                return False
            time.sleep(poll_s)
        self.move_in_progress_evt.clear()
        self._set_status(
            f"Warnung: Zielposition ({x:.1f}, {y:.1f}) nicht bestätigt "
            f"(POS={act_x},{act_y}, Timeout)."
        )
        return False

    def _start_continuous_logger(self):
        if self.logger_thread is not None and self.logger_thread.is_alive():
            return
        self.logger_stop.clear()
        self.logger_thread = threading.Thread(target=self._continuous_logger_worker, daemon=True)
        self.logger_thread.start()

    def _stop_continuous_logger(self):
        self.logger_stop.set()
        if self.logger_thread is not None and self.logger_thread.is_alive():
            self.logger_thread.join(timeout=self.LOGGER_JOIN_TIMEOUT_S)

    def _continuous_logger_worker(self):
        while not self.logger_stop.is_set() and not self.automatik_stop.is_set():
            if not self.move_in_progress_evt.is_set():
                time.sleep(self.LOGGER_IDLE_SLEEP_S)
                continue
            with self.target_lock:
                x, y, z = self.current_target
            self._add_measurement_row(x, y, z, burst=False)
            time.sleep(self.log_interval_s)

    def automatik(self):
        self._set_status("Automatik aktiv.")
        ablauf = self.ablaufart.get()
        modus = self.messmodus.get()
        stepx = float(self.step_size.get())
        stepy = float(self.step_size.get())
        xmin, xmax = 10, self.x_max.get()
        ymin, ymax = 10, self.y_max.get()
        z = self.z_pos.get()
        try:
            if modus == "KONTI":
                self._start_continuous_logger()
            self.goto_pos(self.start_x.get(), self.start_y.get())
            time.sleep(0.2)
            if ablauf=="KREUZ":
                ym = (ymin + ymax)/2
                xlist = np.arange(xmin,xmax+0.01,stepx)
                for x in xlist:
                    self.goto_pos(x, ym)
                    if modus == "PUNKTE":
                        self._messpunkt(x,ym,z)
                    if self.automatik_stop.is_set():
                        return
                xm = (xmin + xmax)/2
                ylist = np.arange(ymin, ymax+0.01, stepy)
                for y in ylist:
                    self.goto_pos(xm, y)
                    if modus == "PUNKTE":
                        self._messpunkt(xm,y,z)
                    if self.automatik_stop.is_set():
                        return
            elif ablauf=="SCHLANGE":
                ylist = np.arange(ymin, ymax+0.01, stepy)
                xlist = np.arange(xmin, xmax+0.01, stepx)
                reverse = False
                for y in ylist:
                    xrow = xlist if not reverse else xlist[::-1]
                    for x in xrow:
                        self.goto_pos(x, y)
                        if modus == "PUNKTE":
                            self._messpunkt(x,y,z)
                        if self.automatik_stop.is_set():
                            return
                    reverse = not reverse
            if not self.automatik_stop.is_set():
                self._set_status("Messlauf abgeschlossen.")
        finally:
            self.move_in_progress_evt.clear()
            self._stop_continuous_logger()
            self._set_toggle_enabled(True)
            self.automatik_running = False

    def _messpunkt(self, x, y, z):
        self._add_measurement_row(x, y, z, burst=True)

    def _add_measurement_row(self, x, y, z, burst):
        try:
            voltage = daq_voltage()
            airflow = voltage_to_airflow(voltage)
            if burst:
                dl_raw, dl_voltage, dl_mean = self.datalogger.read_burst()
            else:
                dl_raw, dl_voltage = self.datalogger.read_once()
                dl_mean = dl_voltage
            row = {
                'x':x, 'y':y, 'z':z,
                'voltage':voltage, 'airflow':airflow,
                'dl_raw':dl_raw, 'dl_voltage':dl_voltage, 'dl_mean':dl_mean
            }
            with self.data_lock:
                self.data.append(row)
            self._display_row(row)
            return row
        except Exception as exc:
            self._set_status(f"Datalogger/DAQ Fehler: {exc}")
            return None

    def _display_row(self, row):
        def _insert():
            self.listbox.insert(
                tk.END,
                f"x={row['x']:.1f} y={row['y']:.1f} z={row['z']:.1f} "
                f"U={row['voltage']:+.4f}V w={row['airflow']:+.2f}m/s "
                f"DL={row['dl_voltage']:+.4f}V"
            )
            self.listbox.yview_moveto(1)
        self._ui_call(_insert)

    def export_csv(self):
        with self.data_lock:
            rows = list(self.data)
        if not rows:
            messagebox.showinfo("Export", "Keine Daten vorhanden!")
            return
        df = pd.DataFrame(rows)
        dest = filedialog.asksaveasfilename(initialdir=self.save_dir.get(),defaultextension=".csv", filetypes=[("CSV","*.csv")])
        if dest:
            df.to_csv(dest, index=False)
            messagebox.showinfo("Export", f"Exportiert nach {dest}")

    def on_close(self):
        self.automatik_stop.set()
        self.logger_stop.set()
        self._stop_continuous_logger()
        self.mot.close()
        self.datalogger.close()
        self.destroy()

if __name__ == "__main__":
    MeasurementGUI().mainloop()