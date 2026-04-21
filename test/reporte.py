import sys
import re
from datetime import datetime

# -----------------------------
# Configuración
# -----------------------------
TOLERANCE = 0.5   # ± grados
HOLD_TIME = 120    # segundos dentro de banda
MIN_DEVIATION = 1.5  # °C (ajústalo)

# -----------------------------
# Parseo de línea
# -----------------------------
def parse_line(line):
    match = re.search(
        r'(\d{2}:\d{2}:\d{2}):\d+\s+->.*Temp:\s*([\d\.]+)\s*\|\s*Setpoint:\s*([\d\.]+)',
        line
    )
    if not match:
        return None

    time_str = match.group(1)
    temp = float(match.group(2))
    sp = float(match.group(3))

    t = datetime.strptime(time_str, "%H:%M:%S")

    return t, temp, sp


# -----------------------------
# Formato de tiempo
# -----------------------------
def format_time(delta):
    total_sec = int(delta.total_seconds())
    m = total_sec // 60
    s = total_sec % 60
    return f"{m}:{s:02d} min"


# -----------------------------
# MAIN
# -----------------------------
def main(file_path):

    data = []

    with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parsed = parse_line(line)
            if parsed:
                data.append(parsed)

    if not data:
        print("No se encontraron datos válidos.")
        return

    # -----------------------------
    # Preparar datos
    # -----------------------------
    t0 = data[0][0]

    times = [(t - t0) for t, _, _ in data]
    temps = [temp for _, temp, _ in data]
    setpoints = [sp for _, _, sp in data]

    setpoint = setpoints[0]
    temp_ini = temps[0]

    # -----------------------------
    # Detectar modo
    # -----------------------------
    mode = "Calentamiento" if temp_ini < setpoint else "Enfriamiento"

    # -----------------------------
    # Overshoot / Undershoot (lógica correcta)
    # -----------------------------
    overshoot = None
    undershoot = None
    time_peak = None
    time_valley = None

    if mode == "Calentamiento":

        # Buscar si cruza el setpoint
        crossed = [i for i, t in enumerate(temps) if t > setpoint]

        if crossed:
            # Overshoot = máximo después de cruzar
            start = crossed[0]
            max_temp = max(temps[start:])
            idx_peak = temps.index(max_temp, start)
            time_peak = times[idx_peak]
            overshoot = max_temp

            # Buscar undershoot: bajar por debajo del setpoint después del pico
            post_peak = temps[idx_peak:]
            below = [i for i, t in enumerate(post_peak) if t < (setpoint - MIN_DEVIATION)]

            if below:
                idx_valley = idx_peak + below[0]
                min_temp = min(temps[idx_valley:])
                idx_valley = temps.index(min_temp, idx_valley)
                time_valley = times[idx_valley]
                undershoot = min_temp
            else:
                undershoot = None
                idx_valley = None
                time_valley = None

        # si no cruza → no hay overshoot

    else:  # Enfriamiento

        crossed = [i for i, t in enumerate(temps) if t < setpoint]

        if crossed:
            # Overshoot = mínimo después de cruzar
            start = crossed[0]
            min_temp = min(temps[start:])
            idx_peak = temps.index(min_temp, start)
            time_peak = times[idx_peak]
            overshoot = min_temp

            # Buscar undershoot: subir por encima del setpoint después del valle
            post_peak = temps[idx_peak:]
            above = [i for i, t in enumerate(post_peak) if t > (setpoint + MIN_DEVIATION)]

            if above:
                idx_valley = idx_peak + above[0]
                max_temp = max(temps[idx_valley:])
                idx_valley = temps.index(max_temp, idx_valley)
                time_valley = times[idx_valley]
                undershoot = max_temp
            else:
                undershoot = None
                idx_valley = None
                time_valley = None

    # -----------------------------
    # HOLDING
    # -----------------------------
    holding_time = None
    holding_temp = None

    # Punto de inicio correcto
    start_idx = 0
    if undershoot is not None and idx_valley is not None:
        start_idx = idx_valley
    elif overshoot is not None:
        start_idx = idx_peak

    for i in range(start_idx, len(temps)):
        ok = True

        for j in range(i, min(i + HOLD_TIME, len(temps))):
            if abs(temps[j] - setpoint) > TOLERANCE:
                ok = False
                break

        if ok:
            holding_time = times[i]
            holding_temp = temps[i]
            break

    # -----------------------------
    # FINAL
    # -----------------------------
    temp_final = temps[-1]
    time_final = times[-1]

    # -----------------------------
    # REPORTE
    # -----------------------------
    print()
    print("File: ", file_path)
    # print()
    print(f"Temp inicial: {temp_ini:.2f} °C to Setpoint: {setpoint:.2f} °C")
    print(f"Modo: {mode}")
    # print()

    if overshoot is not None:
        delta_over = overshoot - setpoint
        print(f"Overshoot: {overshoot:.2f} °C ({delta_over:+.2f}) en {format_time(time_peak)}")

        if undershoot is not None:
            delta_under = undershoot - setpoint
            print(f"Undershoot: {undershoot:.2f} °C ({delta_under:+.2f}) en {format_time(time_valley)}")
        else:
            print("Undershoot: no presente")

    else:
        print("Overshoot: no presente (no cruza el setpoint)")
        print("Undershoot: no aplica")
        
    # print()

    if holding_time:
        print(f"Holding: {holding_temp:.2f} °C en {format_time(holding_time)} (±{TOLERANCE} °C)")
    else:
        print("Holding: no detectado")

    # print()
    print(f"Temp final: {temp_final:.2f} °C en {format_time(time_final)}")
    print()


# -----------------------------
# ENTRY POINT
# -----------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python script.py archivo.txt")
    else:
        main(sys.argv[1])