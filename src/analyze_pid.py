import re
from statistics import mean, stdev

log_file = "log_pid.txt"

# Regex
pid_line = re.compile(
    r"Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

heat_line = re.compile(r"Heater set to\s*(\d+\.?\d*)%")
cool_line = re.compile(r"Cooling set to\s*(\d+\.?\d*)%")

outputs = []
temps = []
setpoints = []
heat_pwms = []
cool_pwms = []

with open(log_file, "r", encoding="utf-8") as f:
    for line in f:
        m = pid_line.search(line)
        if m:
            outputs.append(float(m.group(1)))
            temps.append(float(m.group(2)))
            setpoints.append(float(m.group(3)))

        m = heat_line.search(line)
        if m:
            heat_pwms.append(float(m.group(1)))

        m = cool_line.search(line)
        if m:
            cool_pwms.append(float(m.group(1)))

# ---------- REPORTE ----------
print("\n===== PID ANALYSIS REPORT =====\n")

print(f"Samples analyzed: {len(temps)}")

if temps:
    error = [sp - t for sp, t in zip(setpoints, temps)]
    abs_error = [abs(e) for e in error]

    print(f"\nTemperature:")
    print(f"  Min: {min(temps):.2f} °C")
    print(f"  Max: {max(temps):.2f} °C")
    print(f"  Mean: {mean(temps):.2f} °C")
    print(f"  StdDev: {stdev(temps):.3f} °C")

    print(f"\nError:")
    print(f"  Mean abs error: {mean(abs_error):.3f} °C")
    print(f"  Max abs error: {max(abs_error):.3f} °C")

if outputs:
    print(f"\nPID Output:")
    print(f"  Min: {min(outputs):.2f}")
    print(f"  Max: {max(outputs):.2f}")
    print(f"  Mean: {mean(outputs):.2f}")

if heat_pwms:
    print(f"\nHeating PWM:")
    print(f"  Mean: {mean(heat_pwms):.2f} %")
    print(f"  Max: {max(heat_pwms):.2f} %")

if cool_pwms:
    print(f"\nCooling PWM:")
    print(f"  Mean: {mean(cool_pwms):.2f} %")
    print(f"  Max: {max(cool_pwms):.2f} %")

# Stability check (±0.1 °C)
stable = [
    abs(sp - t) < 0.1
    for sp, t in zip(setpoints, temps)
]

if stable:
    stable_ratio = sum(stable) / len(stable) * 100
    print(f"\nStability:")
    print(f"  Within ±0.1 °C: {stable_ratio:.1f} % of samples")

print("\n===== END REPORT =====\n")
