import re
import sys
from datetime import datetime
from statistics import mean, stdev

# --------------------------------------------------
# Regex patterns
# --------------------------------------------------
PID_LINE = re.compile(
    r'(?P<time>\d{2}:\d{2}:\d{2}:\d{3}).*Error:\s*(?P<error>-?\d+\.?\d*)\s*\|\s*Output:\s*(?P<output>-?\d+\.?\d*)\s*\|\s*Temp:\s*(?P<temp>-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(?P<sp>-?\d+\.?\d*)'
)

HEAT_LINE = re.compile(
    r'Heater set to\s*(?P<pwm>-?\d+\.?\d*)% power'
)

COOL_LINE = re.compile(
    r'Cooling set to\s*(?P<pwm>-?\d+\.?\d*)% power'
)

HOLD_LINE = re.compile(
    r'Holding:\s*Temp=(?P<temp>-?\d+\.?\d*),\s*Setpoint=(?P<sp>-?\d+\.?\d*)\s*\|\s*HeatPWM=(?P<pwm>-?\d+\.?\d*)'
)

# --------------------------------------------------
# Helpers
# --------------------------------------------------
def parse_time(t):
    return datetime.strptime(t, "%H:%M:%S:%f")

# --------------------------------------------------
# Load log
# --------------------------------------------------
if len(sys.argv) < 2:
    print("Uso: python analyze_pid.py log.txt")
    sys.exit(1)

logfile = sys.argv[1]

times = []
temps = []
errors = []
outputs = []
heating = []
cooling = []
hold_pwm = []
in_hold = []
setpoints = []
current_heat = 0.0
current_cool = 0.0
current_hold_pwm = None
hold_active = False

t0 = None

with open(logfile, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        m = PID_LINE.search(line)
        if m:
            t = parse_time(m.group('time'))
            if t0 is None:
                t0 = t
            times.append((t - t0).total_seconds())
            temps.append(float(m.group('temp')))
            errors.append(float(m.group('error')))
            outputs.append(float(m.group('output')))
            heating.append(current_heat)
            cooling.append(current_cool)
            hold_pwm.append(current_hold_pwm)
            in_hold.append(hold_active)
            setpoints.append(float(m.group('sp')))
            continue

        m = HEAT_LINE.search(line)
        if m:
            current_heat = float(m.group('pwm'))
            current_cool = 0.0
            hold_active = False
            current_hold_pwm = None
            continue

        m = COOL_LINE.search(line)
        if m:
            current_cool = float(m.group('pwm'))
            current_heat = 0.0
            hold_active = False
            current_hold_pwm = None
            continue

        m = HOLD_LINE.search(line)
        if m:
            hold_active = True
            current_hold_pwm = float(m.group('pwm'))
            current_heat = current_hold_pwm
            current_cool = 0.0
            continue

# --------------------------------------------------
# Basic checks
# --------------------------------------------------
if not temps:
    print("No se encontraron datos PID válidos.")
    sys.exit(1)

setpoint = mean(setpoints) if setpoints else temps[0]  # dummy, SP viene en línea
dt = mean([times[i+1] - times[i] for i in range(len(times)-1)])

# --------------------------------------------------
# Metrics
# --------------------------------------------------
max_temp = max(temps)
min_temp = min(temps)
overshoot = max_temp - setpoint
undershoot = setpoint - min_temp

# Stability (±0.1 °C)
stable_idx = [i for i, t in enumerate(temps) if abs(t - setpoint) <= 0.1]

stable_time = len(stable_idx) * dt

# HOLD metrics
hold_temps = [temps[i] for i in range(len(temps)) if in_hold[i]]
hold_pwms = [hold_pwm[i] for i in range(len(hold_pwm)) if hold_pwm[i] is not None]

# --------------------------------------------------
# Report
# --------------------------------------------------
print("\n▶️ Datos generales")
print(f"  Muestras: {len(temps)}")
print(f"  Duración total: {times[-1]:.1f} s")

print("\n📉 Amortiguación")
print(f"  Overshoot: {overshoot:.3f} °C")
print(f"  Undershoot: {undershoot:.3f} °C")

print("\n🟢 HOLD")
if hold_temps:
    print(f"  Tiempo en HOLD: {len(hold_temps) * dt:.1f} s")
    print(f"  Temp media HOLD: {mean(hold_temps):.3f} °C")
    if len(hold_temps) > 2:
        print(f"  σ HOLD: {stdev(hold_temps):.4f}")
    if hold_pwms:
        print(f"  HeatPWM medio HOLD: {mean(hold_pwms):.2f} %")
else:
    print("  HOLD no detectado")

print("\n⚡ Actuación")
print(f"  PID Output Max: {max(outputs):.2f}")
print(f"  Heating PWM Max: {max(heating):.2f}")
print(f"  Cooling PWM Max: {max(cooling):.2f}")

print("\n✅ Análisis completado\n")
