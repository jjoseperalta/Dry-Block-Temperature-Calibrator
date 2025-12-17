import re
from statistics import mean, stdev

# ---------------- CONFIG ----------------
LOG_FILE = "log_pid.txt"
PID_PERIOD = 1.0        # segundos
STABILITY_TIME = 60.0   # segundos
FINE_BAND = 0.1
RISE_BAND = 0.5
# ----------------------------------------

pid_line = re.compile(
    r"Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

temps = []
setpoints = []
outputs = []

with open(LOG_FILE, "r", encoding="utf-8") as f:
    for line in f:
        m = pid_line.search(line)
        if m:
            outputs.append(float(m.group(1)))
            temps.append(float(m.group(2)))
            setpoints.append(float(m.group(3)))

samples = len(temps)
times = [i * PID_PERIOD for i in range(samples)]

print("\n===== PID TIME ANALYSIS REPORT =====\n")
print(f"Samples analyzed: {samples}")
print(f"Total time: {times[-1]:.1f} s\n")

# ---------------- BASIC STATS ----------------
error = [sp - t for sp, t in zip(setpoints, temps)]
abs_error = [abs(e) for e in error]

print("Temperature:")
print(f"  Min: {min(temps):.2f} °C")
print(f"  Max: {max(temps):.2f} °C")
print(f"  StdDev: {stdev(temps):.3f} °C\n")

# ---------------- RISE TIME ----------------
rise_time = None
for i, e in enumerate(abs_error):
    if e <= RISE_BAND:
        rise_time = times[i]
        break

print(f"Rise Time (±{RISE_BAND} °C): {rise_time:.1f} s" if rise_time else "Rise Time: not reached")

# ---------------- TIME TO SETPOINT ----------------
t_setpoint = None
for i, e in enumerate(abs_error):
    if e <= FINE_BAND:
        t_setpoint = times[i]
        break

print(f"Time to Setpoint (±{FINE_BAND} °C): {t_setpoint:.1f} s" if t_setpoint else "Time to Setpoint: not reached")

# ---------------- OVERSHOOT ----------------
overshoot = max(temps[i] - setpoints[i] for i in range(samples))
overshoot_time = times[
    max(range(samples), key=lambda i: temps[i] - setpoints[i])
]

print(f"Overshoot: {overshoot:.2f} °C at {overshoot_time:.1f} s")

# ---------------- UNDERSHOOT ----------------
undershoot = min(temps[i] - setpoints[i] for i in range(samples))
undershoot_time = times[
    min(range(samples), key=lambda i: temps[i] - setpoints[i])
]

print(f"Undershoot: {undershoot:.2f} °C at {undershoot_time:.1f} s")

# ---------------- SETTLING TIME ----------------
stable_samples = int(STABILITY_TIME / PID_PERIOD)
settling_time = None

for i in range(samples - stable_samples):
    window = abs_error[i:i + stable_samples]
    if all(e <= FINE_BAND for e in window):
        settling_time = times[i]
        break

print(f"Settling Time (±{FINE_BAND} °C for {STABILITY_TIME}s): "
      f"{settling_time:.1f} s" if settling_time else
      "Settling Time: not reached")

print("\n===== END TIME REPORT =====\n")
