import re
import sys
from statistics import mean, stdev
from datetime import datetime

# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================
RISE_TOL = 0.5            # ±0.5 °C
SETPOINT_TOL = 0.1        # ±0.1 °C
SETTLING_TOL = 0.1        # ±0.1 °C
SETTLING_TIME_SEC = 10    # settling "clásico"
SOLID_STABILITY_SEC = 120  # estabilidad metrológica

# ============================================================
# ARCHIVO DE LOG
# ============================================================
if len(sys.argv) < 2:
    print("⚠️  Usando log por defecto: log_pid.txt")
    LOG_FILE = "log_pid.txt"
else:
    LOG_FILE = sys.argv[1]

# ============================================================
# REGEX
# ============================================================
pid_line = re.compile(
    r"(\d{2}:\d{2}:\d{2}:\d{3})\s*->\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)
heat_line = re.compile(r"Heater set to\s*(\d+\.?\d*)%")
cool_line = re.compile(r"Cooling set to\s*(\d+\.?\d*)%")

# ============================================================
# CARGA DE DATOS
# ============================================================
timestamps, temps, setpoints, outputs = [], [], [], []
heat_pwms, cool_pwms = [], []

def format_hms(seconds):
    if seconds is None:
        return "N/A"
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    return f"{h:02}:{m:02}:{s:02}"

with open(LOG_FILE, "r", encoding="utf-8") as f:
    for line in f:
        m = pid_line.search(line)
        if m:
            timestamps.append(datetime.strptime(m.group(1), "%H:%M:%S:%f"))
            outputs.append(float(m.group(2)))
            temps.append(float(m.group(3)))
            setpoints.append(float(m.group(4)))

        m = heat_line.search(line)
        if m:
            heat_pwms.append(float(m.group(1)))

        m = cool_line.search(line)
        if m:
            cool_pwms.append(float(m.group(1)))

if not temps:
    print("❌ No se encontraron datos válidos")
    sys.exit(1)

# ============================================================
# TIEMPO RELATIVO
# ============================================================
t0 = timestamps[0]
time_sec = [(t - t0).total_seconds() for t in timestamps]

# ============================================================
# ERRORES
# ============================================================
error = [sp - t for sp, t in zip(setpoints, temps)]
abs_error = [abs(e) for e in error]
SP = setpoints[0]

# ============================================================
# MÉTRICAS TEMPORALES
# ============================================================
def first_time_within(tol):
    for i, e in enumerate(abs_error):
        if e <= tol:
            return time_sec[i]
    return None

rise_time = first_time_within(RISE_TOL)
setpoint_time = first_time_within(SETPOINT_TOL)

# Settling clásico (continua)
settling_time = None
for i in range(len(abs_error)):
    start = time_sec[i]
    ok = True
    for j in range(i, len(abs_error)):
        if abs_error[j] > SETTLING_TOL:
            ok = False
            break
        if time_sec[j] - start >= SETTLING_TIME_SEC:
            settling_time = start
            break
    if settling_time is not None:
        break

# ============================================================
# OVERSHOOT / UNDERSHOOT (post rise-time)
# ============================================================
start_idx = 0
if rise_time is not None:
    start_idx = next(i for i, t in enumerate(time_sec) if t >= rise_time)

max_temp = max(temps[start_idx:])
min_temp = min(temps[start_idx:])
max_temp_time = time_sec[start_idx + temps[start_idx:].index(max_temp)]
min_temp_time = time_sec[start_idx + temps[start_idx:].index(min_temp)]

overshoot = max(0.0, max_temp - SP)
undershoot = max(0.0, SP - min_temp)

# ============================================================
# ESTABILIDAD GLOBAL (% muestras)
# ============================================================
stable_samples = sum(1 for e in abs_error if e <= SETTLING_TOL)
stability_ratio = 100.0 * stable_samples / len(abs_error)

# ============================================================
# ESTABILIDAD SÓLIDA (METROLÓGICA)
# ============================================================
solid_periods = []
start_idx = None

for i in range(len(temps)):
    if start_idx is None:
        start_idx = i
        ref = temps[i]
        continue

    if abs(temps[i] - ref) > SETTLING_TOL:
        duration = time_sec[i-1] - time_sec[start_idx]
        if duration >= SOLID_STABILITY_SEC:
            block = temps[start_idx:i]
            solid_periods.append({
                "duration": duration,
                "avg": mean(block),
                "std": stdev(block) if len(block) > 1 else 0.0
            })
        start_idx = None

# ============================================================
# VELOCIDAD TÉRMICA
# ============================================================
rates = []
for i in range(1, len(temps)):
    dt = time_sec[i] - time_sec[i-1]
    if dt > 0:
        rates.append((temps[i] - temps[i-1]) / dt)

initial_temp = temps[0]
delta_t = SP - initial_temp
# ============================================================
# REPORTE
# ============================================================
print("\n" + "="*60)
print("📊 REPORTE DE ANÁLISIS PID – v1.0")
print("="*60)

print(f"\n▶️ Datos generales")
print(f"  Muestras: {len(temps)}")
print(f"  Duración total: {time_sec[-1]:.1f} s ({format_hms(time_sec[-1])})")
print(f"  Temp inicial: {initial_temp:.2f} °C")
print(f"  Setpoint: {SP:.2f} °C")
print(f"  ΔT inicial: {delta_t:+.2f} °C")

print(f"\n📈 Respuesta temporal")
print(f"  Rise Time (±{RISE_TOL} °C): {rise_time:.2f} s ({format_hms(rise_time)})" if rise_time else "  Rise Time: N/A")
print(f"  Time to Setpoint (±{SETPOINT_TOL} °C): {setpoint_time:.2f} s ({format_hms(setpoint_time)})" if setpoint_time else "  Time to Setpoint: N/A")
print(f"  Settling Time (±{SETTLING_TOL} °C, {SETTLING_TIME_SEC}s): {settling_time:.2f} s ({format_hms(settling_time)})" if settling_time else "  Settling Time: N/A")

print(f"\n📉 Amortiguación")
print(f"  Overshoot: {overshoot:.3f} °C @ {max_temp_time:.1f} s ({format_hms(max_temp_time)})")
print(f"  Undershoot: {undershoot:.3f} °C @ {min_temp_time:.1f} s ({format_hms(min_temp_time)})")

print(f"\n🟢 Estabilidad")
print(f"  % muestras dentro ±{SETTLING_TOL} °C: {stability_ratio:.1f} %")

if solid_periods:
    print(f"  Estabilidad sólida ≥ {SOLID_STABILITY_SEC}s:")
    for p in solid_periods:
        print(f"    - Duración: {p['duration']:.1f}s ({format_hms(p['duration'])}) | Avg: {p['avg']:.3f} °C | σ: {p['std']:.4f}")
else:
    print(f"  Estabilidad sólida: NO alcanzada")

print(f"\n🚀 Dinámica térmica")
print(f"  Máx subida: {max(rates):.4f} °C/s")
print(f"  Máx bajada: {min(rates):.4f} °C/s")

print(f"\n⚡ Actuación")
print(f"  PID Output Min / Max: {min(outputs):.2f} / {max(outputs):.2f}")
if heat_pwms:
    print(f"  Heating PWM Mean / Max: {mean(heat_pwms):.2f} / {max(heat_pwms):.2f}")
if cool_pwms:
    print(f"  Cooling PWM Mean / Max: {mean(cool_pwms):.2f} / {max(cool_pwms):.2f}")

print("\n" + "="*60)
