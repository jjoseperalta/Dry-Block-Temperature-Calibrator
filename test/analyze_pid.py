import re
import sys
from statistics import mean, stdev
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================
RISE_TOL = 0.5              # ±0.5 °C
SETPOINT_TOL = 0.1          # ±0.1 °C
SETTLING_TOL = 0.1          # ±0.1 °C
SETTLING_TIME_SEC = 10      # settling clásico
SOLID_STABILITY_SEC = 60    # estabilidad metrológica
MAX_PHYSICAL_STEP = 1.0     # °C por muestra (filtro glitches)

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
    r"(\d{2}:\d{2}:\d{2}:\d{3})\s*->\s*State:\s*(\w+)\s*\|\s*Error:\s*(-?\d+\.?\d*)\s*\|\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)
heat_line = re.compile(r"Heater set to\s*(\d+\.?\d*)%")
cool_line = re.compile(r"Cooling set to\s*(\d+\.?\d*)%")

# ============================================================
# CARGA DE DATOS
# ============================================================
timestamps, temps, setpoints, outputs = [], [], [], []
heat_pwms, cool_pwms = [], []

with open(LOG_FILE, "r", encoding="utf-8") as f:
    for line in f:
        m = pid_line.search(line)
        if m:
            timestamps.append(datetime.strptime(m.group(1), "%H:%M:%S:%f"))
            state = m.group(2)
            error = float(m.group(3))
            outputs.append(float(m.group(4)))
            temps.append(float(m.group(5)))
            setpoints.append(float(m.group(6)))

        m = heat_line.search(line)
        if m:
            heat_pwms.append(float(m.group(1)))

        m = cool_line.search(line)
        if m:
            cool_pwms.append(float(m.group(1)))

if not temps:
    print("❌ No se encontraron datos válidos")
    sys.exit(1)

# Extraer segundos con decimales
start = timestamps[0]
seconds = [(ts - start).total_seconds() for ts in timestamps]

elapsed_seconds = np.array(seconds).astype(int)
outputs = np.array(outputs)
temps = np.array(temps)

# print("len elapsed_seconds:", len(elapsed_seconds))
# print("len outputs:", len(outputs))
# print("len temps:", len(temps))

np.set_printoptions(threshold=np.inf)
np.set_printoptions(precision=2, suppress=True)
# print(outputs)
# print(np.array2string(elapsed_seconds, separator=", "))
# print(np.array2string(outputs, separator=", "))
# print(np.array2string(temps, separator=", "))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

# Gráfica 1: Temperatura 25°C
ax1.plot(elapsed_seconds, temps, 'b-', linewidth=2, label='Temperatura')
ax1.axhline(y=setpoints[0], color='r', linestyle='--', linewidth=1.5, label=f'Setpoint {setpoints[0]}°C')
ax1.fill_between(elapsed_seconds, setpoints[0] - 0.1, setpoints[0] + 0.1, alpha=0.1, color='green')
ax1.set_title(f'Setpoint {setpoints[0]}°C - Respuesta de Temperatura')
ax1.set_xlabel('Tiempo (s)')
ax1.set_ylabel('Temperatura (°C)')
ax1.grid(True, alpha=0.3)
ax1.legend()
ax1.set_ylim(np.min(temps) - 0.1, np.max(temps) + 0.1)
# ax1.xaxis.set_major_locator(ticker.MultipleLocator(2))
# ax1.yaxis.set_major_locator(ticker.MultipleLocator(0.25))

# Gráfica 2: Salida 25°C
ax2.plot(elapsed_seconds, outputs, 'g-', linewidth=2)
ax2.fill_between(elapsed_seconds, 0, outputs, alpha=0.3, color='green')
ax2.set_title(f'Setpoint {setpoints[0]}°C - Salida del Controlador')
ax2.set_xlabel('Tiempo (s)')
ax2.set_ylabel('Potencia (%)')
ax2.grid(True, alpha=0.3)
ax2.set_ylim(np.min(outputs) - 0.1, np.max(outputs) + 0.1)
# ax2.yaxis.set_major_locator(ticker.MultipleLocator(2.5))

plt.tight_layout()
plt.show()

sys.exit()
# ============================================================
# TIEMPO RELATIVO
# ============================================================
t0 = timestamps[0]
time_sec = [(t - t0).total_seconds() for t in timestamps]

# ============================================================
# FILTRO DE GLITCHES NO FÍSICOS
# ============================================================
f_temps = [temps[0]]
f_time = [time_sec[0]]

for i in range(1, len(temps)):
    if abs(temps[i] - temps[i - 1]) <= MAX_PHYSICAL_STEP:
        f_temps.append(temps[i])
        f_time.append(time_sec[i])

temps = f_temps
time_sec = f_time
SP = setpoints[0]

# ============================================================
# ERRORES
# ============================================================
error = [SP - t for t in temps]
abs_error = [abs(e) for e in error]

# ============================================================
# FUNCIONES AUXILIARES
# ============================================================
def format_hms(seconds):
    if seconds is None:
        return "N/A"
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    return f"{h:02}:{m:02}:{s:02}"

def first_time_within(tol):
    for i, e in enumerate(abs_error):
        if e <= tol:
            return time_sec[i]
    return None

def calc_drift(temp_block, time_block):
    if len(temp_block) < 2:
        return 0.0
    dt = time_block[-1] - time_block[0]
    if dt <= 0:
        return 0.0
    return (temp_block[-1] - temp_block[0]) / dt * 60.0  # °C/min

def stability_grade(std, drift):
    if std < 0.03 and abs(drift) < 0.02:
        return "A (Excelente)"
    elif std < 0.05 and abs(drift) < 0.05:
        return "B (Buena)"
    else:
        return "C (Insuficiente)"

# ============================================================
# MÉTRICAS TEMPORALES
# ============================================================
rise_time = first_time_within(RISE_TOL)
setpoint_time = first_time_within(SETPOINT_TOL)

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
# OVERSHOOT / UNDERSHOOT
# ============================================================
start_idx = 0
if rise_time is not None:
    start_idx = next(i for i, t in enumerate(time_sec) if t >= rise_time)

max_temp = max(temps[start_idx:])
min_temp = min(temps[start_idx:])
overshoot = max(0.0, max_temp - SP)
undershoot = max(0.0, SP - min_temp)

# ============================================================
# ESTABILIDAD SÓLIDA
# ============================================================
solid_periods = []
start_idx = None

for i in range(len(temps)):
    if start_idx is None:
        start_idx = i
        ref = temps[i]
        continue

    if abs(temps[i] - ref) > SETTLING_TOL:
        duration = time_sec[i - 1] - time_sec[start_idx]
        if duration >= SOLID_STABILITY_SEC:
            block = temps[start_idx:i]
            drift = calc_drift(block, time_sec[start_idx:i])
            errors_block = [SP - t for t in block]
            solid_periods.append({
                "duration": duration,
                "avg": mean(block),
                "std": stdev(block) if len(block) > 1 else 0.0,
                "drift": drift,
                "err_mean": mean(errors_block),
                "err_max": max(abs(e) for e in errors_block),
                "grade": stability_grade(
                    stdev(block) if len(block) > 1 else 0.0,
                    drift
                )
            })
        start_idx = None

total_stable_time = sum(p["duration"] for p in solid_periods)

# ============================================================
# DETECCIÓN DE MESETAS TÉRMICAS (NO SOLO SETPOINT)
# ============================================================
plateaus = []
start_idx = None

for i in range(len(temps)):
    if start_idx is None:
        start_idx = i
        ref = temps[i]
        continue

    if abs(temps[i] - ref) > SETTLING_TOL:
        duration = time_sec[i - 1] - time_sec[start_idx]
        if duration >= SOLID_STABILITY_SEC:
            block = temps[start_idx:i]
            drift = calc_drift(block, time_sec[start_idx:i])
            plateaus.append({
                "duration": duration,
                "avg": mean(block),
                "std": stdev(block) if len(block) > 1 else 0.0,
                "drift": drift,
                "distance_to_sp": abs(mean(block) - SP)
            })
        start_idx = None

# ============================================================
# TEMPERATURA DOMINANTE (HISTOGRAMA TEMPORAL)
# ============================================================
BIN_WIDTH = 0.02  # resolución térmica (0.01–0.02 recomendado)

bins = {}

for i in range(1, len(temps)):
    dt = time_sec[i] - time_sec[i - 1]
    if dt <= 0:
        continue

    bin_center = round(temps[i] / BIN_WIDTH) * BIN_WIDTH
    bins[bin_center] = bins.get(bin_center, 0.0) + dt

# Ordenar por tiempo acumulado
dominant_bins = sorted(bins.items(), key=lambda x: x[1], reverse=True)

# ============================================================
# VELOCIDAD TÉRMICA
# ============================================================
rates = []
for i in range(1, len(temps)):
    dt = time_sec[i] - time_sec[i - 1]
    if dt > 0:
        rates.append((temps[i] - temps[i - 1]) / dt)

# ============================================================
# REPORTE
# ============================================================
print("\n" + "=" * 60)
print("📊 REPORTE METROLÓGICO - DRY BLOCK CALIBRATOR")
print("=" * 60)

print("\n▶️ Datos generales")
print(f"  Muestras: {len(temps)}")
print(f"  Duración total: {time_sec[-1]:.1f} s ({format_hms(time_sec[-1])})")
print(f"  Temp inicial: {temps[0]:.2f} °C")
print(f"  Setpoint: {SP:.2f} °C")

print("\n📈 Respuesta temporal")
print(f"  Rise Time (±{RISE_TOL} °C): {rise_time:.2f} s ({format_hms(rise_time)})" if rise_time else "  Rise Time: N/A")
print(f"  Time to Setpoint (±{SETPOINT_TOL} °C): {setpoint_time:.2f} s ({format_hms(setpoint_time)})" if setpoint_time else "  Time to Setpoint: N/A")
print(f"  Settling Time (±{SETTLING_TOL} °C): {settling_time:.2f} s ({format_hms(settling_time)})" if settling_time else "  Settling Time: N/A")

print("\n📉 Amortiguación")
print(f"  Overshoot: {overshoot:.3f} °C")
print(f"  Undershoot: {undershoot:.3f} °C")

print("\n🟢 Estabilidad sólida")
if solid_periods:
    for p in solid_periods:
        print(f"  Duración: {p['duration']:.1f}s ({format_hms(p['duration'])})")
        print(f"    Avg: {p['avg']:.3f} °C | σ: {p['std']:.4f}")
        print(f"    Drift: {p['drift']:+.4f} °C/min")
        print(f"    Error medio: {p['err_mean']:+.4f} °C | Error máx: {p['err_max']:.4f} °C")
        print(f"    Calidad: {p['grade']}")
else:
    print("  NO alcanzada")

    print(f"\n🟣 Mesetas térmicas detectadas (≥ {SOLID_STABILITY_SEC} s):")

if plateaus:
    plateaus.sort(key=lambda p: p["duration"], reverse=True)

    for i, p in enumerate(plateaus, 1):
        print(f"  #{i}:")
        print(f"    Temp media: {p['avg']:.3f} °C")
        print(f"    Duración: {p['duration']:.1f} s ({format_hms(p['duration'])})")
        print(f"    σ: {p['std']:.4f}")
        print(f"    Drift: {p['drift']:+.4f} °C/min")
        print(f"    |ΔSP|: {p['distance_to_sp']:.3f} °C")
else:
    print(f"  No se detectaron mesetas ≥ {SOLID_STABILITY_SEC} s")

print("\n🟠 Temperaturas dominantes (por tiempo acumulado):")

for temp, t in dominant_bins[:5]:
    print(f"  {temp:.2f} °C → {t:.1f} s ({format_hms(t)})")

if dominant_bins:
    dom_temp, dom_time = dominant_bins[0]
    offset = dom_temp - SP
    print(f"\n📐 Offset térmico dominante: {offset:+.3f} °C")

print(f"\n⏱ Tiempo total estable: {total_stable_time:.1f} s")

print("\n🚀 Dinámica térmica")
if rates:
    print(f"  Máx subida: {max(rates):.4f} °C/s")
    print(f"  Máx bajada: {min(rates):.4f} °C/s")

print("\n⚡ Actuación")
print(f"  PID Output Min / Max: {min(outputs):.2f} / {max(outputs):.2f}")
if heat_pwms:
    print(f"  Heating PWM Mean / Max: {mean(heat_pwms):.2f} / {max(heat_pwms):.2f}")
if cool_pwms:
    print(f"  Cooling PWM Mean / Max: {mean(cool_pwms):.2f} / {max(cool_pwms):.2f}")

print("\n" + "=" * 60)
