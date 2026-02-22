import re
import os
import sys
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.widgets import Cursor, Button

# ===== ANALISIS PID =====
SETTLING_BAND = 0.1       # ± °C
METROLOGY_TIME = 60       # segundos estabilidad
RISE_LOW = 0.1
RISE_HIGH = 0.9

# ===== LIVE STREAM AVANZADO =====
WINDOW_SECONDS = 600      # mostrar últimos 10 min (None = todo)
AUTO_SCROLL = True        # seguir datos nuevos automáticamente
STREAM_INTERVAL = 500     # ms

# ============================================================
# CONFIG
# ============================================================

PID_REGEX = re.compile(
    r"(\d{2}:\d{2}:\d{2}[:.]\d{3})\s*->.*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

REFRESH_INTERVAL = 1000  # ms live streaming


# ============================================================
# PARSER
# ============================================================

def parse_log(file_path):
    timestamps, temps, outputs, setpoints = [], [], [], []

    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            if m := PID_REGEX.search(line):
                ts = datetime.strptime(m.group(1).replace('.', ':'), "%H:%M:%S:%f")
                timestamps.append(ts)
                outputs.append(float(m.group(2)))
                temps.append(float(m.group(3)))
                setpoints.append(float(m.group(4)))

    if not temps:
        return (
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([])
        )

    start = timestamps[0]
    elapsed = np.array([(t - start).total_seconds() for t in timestamps])

    return elapsed, np.array(temps), np.array(outputs), np.array(setpoints)

class IncrementalLogReader:
    def __init__(self, file_path, regex, read_existing=True):
        self.file_path = file_path
        self.regex = regex
        self.file = open(file_path, "r", encoding="utf-8")
        if not read_existing:
            self.file.seek(0, os.SEEK_END)

        self.start_time = None
        self.times = []
        self.temps = []
        self.outputs = []
        self.setpoints = []

    def read_new_data(self):
        new_data = False

        while True:
            line = self.file.readline()
            if not line:
                break

            if m := self.regex.search(line):
                ts = datetime.strptime(m.group(1).replace('.', ':'), "%H:%M:%S:%f")

                if self.start_time is None:
                    self.start_time = ts

                elapsed = (ts - self.start_time).total_seconds()

                self.times.append(elapsed)
                self.outputs.append(float(m.group(2)))
                self.temps.append(float(m.group(3)))
                self.setpoints.append(float(m.group(4)))

                new_data = True

        if not new_data:
            return None

        return (
            np.array(self.times),
            np.array(self.temps),
            np.array(self.outputs),
            np.array(self.setpoints)
        )

def detect_setpoint_segments(time, temps, setpoints):
    changes = np.where(np.diff(setpoints) != 0)[0] + 1
    idx = np.concatenate(([0], changes, [len(time)]))

    segments = []
    for i in range(len(idx)-1):
        s, e = idx[i], idx[i+1]
        segments.append({
            "time": time[s:e],
            "temps": temps[s:e],
            "setpoint": setpoints[s],
            "start": s,
            "end": e
        })
    return segments


def compute_rise_time(time, temps, sp):
    target_low = sp * RISE_LOW
    target_high = sp * RISE_HIGH

    try:
        t1 = time[np.where(temps >= target_low)[0][0]]
        t2 = time[np.where(temps >= target_high)[0][0]]
        return t2 - t1
    except:
        return None


def compute_overshoot(temps, sp):
    return max(0, np.max(temps) - sp)


def compute_settling_time(time, temps, sp):
    band = SETTLING_BAND
    inside = np.abs(temps - sp) <= band

    for i in range(len(time)):
        if np.all(inside[i:]):
            return time[i]
    return None


def compute_rms_error(temps, sp):
    return np.sqrt(np.mean((temps - sp)**2))


def detect_metrology_stability(time, temps, sp):
    band = SETTLING_BAND
    inside = np.abs(temps - sp) <= band

    for i in range(len(time)):
        t0 = time[i]
        mask = (time >= t0) & (time <= t0 + METROLOGY_TIME)
        if np.sum(mask) == 0:
            continue
        if np.all(inside[mask]):
            return t0, t0 + METROLOGY_TIME
    return None


def analyze_segments(time, temps, setpoints):
    segments = detect_setpoint_segments(time, temps, setpoints)

    results = []
    for seg in segments:
        t = seg["time"]
        y = seg["temps"]
        sp = seg["setpoint"]

        if len(t) < 5:
            continue

        results.append({
            "setpoint": sp,
            "start_idx": seg["start"],
            "end_idx": seg["end"],
            "rise_time": compute_rise_time(t, y, sp),
            "overshoot": compute_overshoot(y, sp),
            "settling_time": compute_settling_time(t, y, sp),
            "rms": compute_rms_error(y, sp),
            "metrology": detect_metrology_stability(t, y, sp)
        })

    return results

# ============================================================
# TOOLTIP INTELIGENTE (tu lógica restaurada)
# ============================================================

class HoverTooltip:

    def __init__(self, ax, x_data, y_data, fig):
        self.ax = ax
        self.x_data = x_data
        self.y_data = y_data
        self.fig = fig
        self.active = False

        self.annot = ax.annotate(
            "", xy=(0,0), xytext=(15,15), textcoords="offset points",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="black"),
            fontsize=9, family='monospace', weight='bold', visible=False
        )

        self.highlight, = ax.plot([], [], 'ro', markersize=8,
                                  markeredgecolor='white', visible=False)

        self.cid_move = fig.canvas.mpl_connect("motion_notify_event", self.on_move)
        self.cid_click = fig.canvas.mpl_connect("button_press_event", self.on_click)

    def update_data(self, x, y):
        self.x_data = x
        self.y_data = y

    def on_move(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return

        self.active = True
        idx = np.argmin(np.abs(self.x_data - event.xdata))
        x = self.x_data[idx]
        y = self.y_data[idx]

        self.highlight.set_data([x], [y])
        self.highlight.set_visible(True)

        self.annot.xy = (x, y)
        self.annot.set_text(f"Time: {int(x//60)}:{int(x%60):02d}\nVal: {y:.2f}")
        self.annot.set_visible(True)

        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if self.active:
            self.active = False
            self.highlight.set_visible(False)
            self.annot.set_visible(False)
            self.fig.canvas.draw_idle()


# ============================================================
# ANALYZER
# ============================================================

class PIDAnalyzer:

    def __init__(self, log_file, live=True):
        self.log_file = log_file
        self.live_mode = live
        self.last_size = 0

        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 5))

        self.setup_ui()

        # crear reader incremental
        self.reader = IncrementalLogReader(self.log_file, PID_REGEX, read_existing=True)

        # buffers iniciales
        self.time = np.array([])
        self.temps = np.array([])
        self.outputs = np.array([])
        self.setpoints = np.array([])
        
        # ---------- crear lineas una sola vez ----------
        self.temp_line, = self.ax1.plot([], [], linewidth=2)
        self.output_line, = self.ax2.plot([], [], linewidth=2, color="green")

        self.sp_line = None
        self.sp_band = None
        
        self.load_data()
        self.draw()

        if self.live_mode:
            self.start_live_stream()

    # ---------- UI ----------

    def setup_ui(self):
        Cursor(self.ax1, useblit=True, color='gray', linewidth=0.5, linestyle=':')
        Cursor(self.ax2, useblit=True, color='gray', linewidth=0.5, linestyle=':')

        ax_btn = self.fig.add_axes([0.01, 0.93, 0.05, 0.05])
        self.btn_reload = Button(ax_btn, "⟳")
        self.btn_reload.on_clicked(self.reload)

    # ---------- DATA ----------

    def load_data(self):
        self.time, self.temps, self.outputs, self.setpoints = parse_log(self.log_file)
        self.last_size = os.path.getsize(self.log_file)

    # ---------- DRAW ----------

    def draw(self):

        # ---------- SIN DATOS ----------
        if len(self.time) == 0:
            self.ax1.set_title("Esperando datos del controlador...")
            self.fig.canvas.draw_idle()
            return

        # ---------- actualizar datos (NO clear) ----------
        self.temp_line.set_data(self.time, self.temps)
        self.output_line.set_data(self.time, self.outputs)

        # ---------- SETPOINT ----------
        sp = self.setpoints[0] if len(self.setpoints) > 0 else None

        if sp is not None:
            if self.sp_line is None:
                self.sp_line = self.ax1.axhline(sp, linestyle="--", color="red")
                self.sp_band = self.ax1.fill_between(self.time, sp-0.1, sp+0.1, alpha=0.1)
            else:
                self.sp_line.set_ydata([sp, sp])

                # recrear banda (matplotlib no permite update fácil)
                if self.sp_band:
                    self.sp_band.remove()
                self.sp_band = self.ax1.fill_between(self.time, sp-0.1, sp+0.1, alpha=0.1)

        # ---------- autoscale inteligente ----------
        self.ax1.relim()
        self.ax1.autoscale_view()

        self.ax2.relim()
        self.ax2.autoscale_view()

        # ---------- tooltips ----------
        if not hasattr(self, "tooltip1"):
            self.tooltip1 = HoverTooltip(self.ax1, self.time, self.temps, self.fig)
            self.tooltip2 = HoverTooltip(self.ax2, self.time, self.outputs, self.fig)
        else:
            self.tooltip1.update_data(self.time, self.temps)
            self.tooltip2.update_data(self.time, self.outputs)

        # ---------- refresh rapido ----------
        self.fig.canvas.draw_idle()

    # ---------- ACTIONS ----------

    def reload(self, event=None):
        print("Reiniciando stream...")

        self.reader.file.close()
        self.reader = IncrementalLogReader(self.log_file, PID_REGEX, read_existing=True)

        self.time = np.array([])
        self.temps = np.array([])
        self.outputs = np.array([])
        self.setpoints = np.array([])

        self.draw()

    # ---------- LIVE STREAM ----------

    def start_live_stream(self):
        self.timer = self.fig.canvas.new_timer(interval=STREAM_INTERVAL)
        self.timer.add_callback(self.update_stream)
        self.timer.start()

    def update_stream(self):
        data = self.reader.read_new_data()

        if data is None:
            return

        t, temps, outputs, setpoints = data

        self.time = t
        self.temps = temps
        self.outputs = outputs
        self.setpoints = setpoints

        if len(self.time) == 0:
            return

        # ventana deslizante
        if WINDOW_SECONDS and len(self.time) > 0:
            tmax = self.time[-1]
            mask = self.time >= (tmax - WINDOW_SECONDS)

            self.time = self.time[mask]
            self.temps = self.temps[mask]
            self.outputs = self.outputs[mask]
            self.setpoints = self.setpoints[mask]

        self.draw()

        # auto scroll eje X
        if AUTO_SCROLL and len(self.time) >= 2:
            for ax in (self.ax1, self.ax2):
                ax.set_xlim(self.time[0], self.time[-1])
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def check_updates(self):
        try:
            size = os.path.getsize(self.log_file)
            if size != self.last_size:
                self.reload()
        except:
            pass


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    log_file = sys.argv[1] if len(sys.argv) > 1 else "log_pid.txt"

    analyzer = PIDAnalyzer(log_file, live=True)

    # plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    plt.show()