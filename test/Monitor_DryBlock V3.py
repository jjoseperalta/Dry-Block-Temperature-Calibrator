import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import os
import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.ticker as ticker
from matplotlib.widgets import Cursor

# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================
RISE_TOL = 0.5
SETPOINT_TOL = 0.1
SETTLING_TOL = 0.1
SETTLING_TIME_SEC = 10
SOLID_STABILITY_SEC = 60
MAX_PHYSICAL_STEP = 1.0
REFRESH_INTERVAL = 1.0

FIG_SIZE = (12, 4)
SENSOR_ERROR_VALUE = -999.0

PID_LINE = re.compile(
    r"(\d{2}:\d{2}:\d{2}[:.]\d{3})\s*->\s*State:\s*(\w+)\s*\|\s*Error:\s*(-?\d+\.?\d*)\s*\|\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)(?:\s*\|\s*Test:\s*(-?\d+\.?\d*))?(?:\s*\|\s*Diff:\s*(-?\d+\.?\d*))?\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

def segundos_a_formato(x, pos=None):
    minutos = int(x) // 60
    segundos = int(x) % 60
    return f"{minutos}:{segundos:02d}"

def es_temperatura_valida(temp):
    if temp is None or np.isnan(temp):
        return False
    if abs(temp - SENSOR_ERROR_VALUE) < 0.01:
        return False
    if temp < -50 or temp > 500:
        return False
    return True

def cargar_datos_desde_texto(texto):
    timestamps, temps, setpoints = [], [], []
    tests, diffs = [], []
    tiene_test, tiene_diff = False, False
    
    lines = texto.split('\n')
    for line in lines:
        if m := PID_LINE.search(line):
            time_str = m.group(1).replace('.', ':')
            try:
                temp_val = float(m.group(5))
                test_val = float(m.group(6)) if m.group(6) else None
                setpoint_val = float(m.group(8))
                
                if not es_temperatura_valida(temp_val):
                    continue
                    
                timestamps.append(datetime.strptime(time_str, "%H:%M:%S:%f"))
                temps.append(temp_val)
                setpoints.append(setpoint_val)
                
                if test_val is not None and es_temperatura_valida(test_val):
                    tests.append(test_val)
                    tiene_test = True
                else:
                    tests.append(None)
                
                diff_val = m.group(7)
                if diff_val is not None:
                    diffs.append(float(diff_val))
                    tiene_diff = True
                else:
                    diffs.append(None)
            except (ValueError, TypeError):
                continue
    
    if not temps:
        return None
    
    start = timestamps[0]
    elapsed_seconds = np.array([(ts - start).total_seconds() for ts in timestamps])
    return {
        'elapsed_seconds': elapsed_seconds,
        'temps': np.array(temps),
        'setpoints': np.array(setpoints),
        'tests': np.array(tests, dtype=float),
        'diffs': np.array(diffs, dtype=float),
        'tiene_test': tiene_test,
        'tiene_diff': tiene_diff,
        'num_muestras': len(temps)
    }

# ============================================================
# TOOLTIP INTERACTIVO
# ============================================================
class TooltipManager:
    def __init__(self, ax, datos_temp, x_data, fig, datos_tests=None, app=None):
        self.ax = ax
        self.datos_temp = datos_temp
        self.datos_tests = datos_tests
        self.x_data = x_data
        self.fig = fig
        self.app = app
        self.tooltip_activo = False
        
        self.annot = self._crear_anotacion(ax, 'blue')
        self.highlight, = ax.plot([], [], 'ro', markersize=8,
                                 markerfacecolor='red', markeredgecolor='white',
                                 markeredgewidth=1.5, alpha=1.0, visible=False)
        self.vline = ax.axvline(x=0, color='red', linestyle=':', linewidth=1, alpha=0.7, visible=False)
        
        self.cid_move = fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.cid_click = fig.canvas.mpl_connect('button_press_event', self.on_click)
    
    def _crear_anotacion(self, ax, color):
        return ax.annotate("", xy=(0,0), xytext=(10,10), textcoords="offset points",
                          bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                                   edgecolor=color, linewidth=1.5, alpha=0.95),
                          arrowprops=dict(arrowstyle="->", color=color, lw=0.8, alpha=0.8),
                          family='monospace', weight='bold', visible=False, annotation_clip=False, zorder=99)
    
    def limpiar_visibles(self):
        if hasattr(self, 'highlight') and self.highlight: self.highlight.set_visible(False)
        if hasattr(self, 'vline') and self.vline: self.vline.set_visible(False)
        if hasattr(self, 'annot') and self.annot: self.annot.set_visible(False)
        self.tooltip_activo = False

    def actualizar_datos(self, datos_temp, x_data, datos_tests=None):
        self.datos_temp = datos_temp
        self.x_data = x_data
        self.datos_tests = datos_tests
        self.limpiar_visibles()
        
    def on_move(self, event):
        if len(self.x_data) == 0: return
        if self.app and self.app.real_time_plot.get():
            self.limpiar_visibles()
            return
        if event.inaxes == self.ax and event.xdata is not None:
            self._actualizar_desde_tiempo(event.xdata)
    
    def _calcular_offset_inteligente(self, ax, tiempo, valor, es_ultimo_punto=False):
        xlim, ylim = ax.get_xlim(), ax.get_ylim()
        x_rel = (tiempo - xlim[0]) / (xlim[1] - xlim[0]) if (xlim[1] - xlim[0]) != 0 else 0.5
        y_rel = (valor - ylim[0]) / (ylim[1] - ylim[0]) if (ylim[1] - ylim[0]) != 0 else 0.5
        offset_x = -80 if (x_rel > 0.9 or es_ultimo_punto) else 10
        offset_y = -40 if y_rel > 0.9 else 10
        return (offset_x, offset_y)
    
    def _actualizar_desde_tiempo(self, tiempo):
        self.tooltip_activo = True
        idx = np.argmin(np.abs(self.x_data - tiempo))
        tiempo_real = self.x_data[idx]
        es_ultimo_punto = idx == len(self.x_data) - 1
        valor_temp = self.datos_temp[idx] if idx < len(self.datos_temp) else None
        valor_test = self.datos_tests[idx] if self.datos_tests is not None and idx < len(self.datos_tests) else None
        self._actualizar_tooltip(tiempo_real, valor_temp, valor_test, es_ultimo_punto)
    
    def _actualizar_tooltip(self, tiempo, valor_temp, valor_test, es_ultimo_punto=False):
        self.vline.set_xdata([tiempo])
        self.vline.set_visible(True)
        
        if valor_temp is not None and es_temperatura_valida(valor_temp):
            self.highlight.set_data([tiempo], [valor_temp])
            self.highlight.set_visible(True)
            offset = self._calcular_offset_inteligente(self.ax, tiempo, valor_temp, es_ultimo_punto)
            
            tooltip_text = f'Time: {segundos_a_formato(tiempo)}\nTemp: {valor_temp:.2f}°C'
            if valor_test is not None and es_temperatura_valida(valor_test):
                tooltip_text += f'\nTest: {valor_test:.2f}°C\nDiff: {(valor_temp - valor_test):+.2f}°C'
            
            self.annot.xy = (tiempo, valor_temp)
            self.annot.set_position(offset)
            self.annot.set_text(tooltip_text)
            self.annot.set_visible(True)
        self.fig.canvas.draw_idle()
    
    def on_click(self, event):
        if self.tooltip_activo:
            self.limpiar_visibles()
            self.fig.canvas.draw_idle()
    
    def disconnect(self):
        self.limpiar_visibles()
        try:
            self.fig.canvas.mpl_disconnect(self.cid_move)
            self.fig.canvas.mpl_disconnect(self.cid_click)
        except:
            pass

# ============================================================
# CLASE PRINCIPAL UNIFICADA
# ============================================================
class SerialMonitorWithPlot:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Monitor with Real-time Plot")
        self.root.geometry("970x850") 
        self.root.minsize(830, 600)
        
        self.serial_connection = None
        self.is_connected = False
        self.read_thread = None
        self.command_history = []
        self.history_index = 0
        self.autoscroll = tk.BooleanVar(value=True)
        self.show_timestamp = tk.BooleanVar(value=True)
        self.save_log = tk.BooleanVar(value=False)
        self.log_file = None
        self.log_file_path = None
        
        self.real_time_plot = tk.BooleanVar(value=False)
        self.plot_data = None
        self.figure = None
        self.ax = None
        self.canvas = None
        self.toolbar = None
        self.tooltip_manager = None
        self.cursor = None
        
        self.setup_styles()
        self.create_widgets()
        self.setup_matplotlib_figure()
        self.refresh_ports()
        self.setup_keyboard_events()
        self.crear_menus_contextuales()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.plot_update_timer = None
        self.start_plot_updates()
        
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        self.bg_color, self.fg_color, self.accent_color = "#2b2b2b", "#ffffff", "#007acc"
        self.output_bg, self.input_bg, self.plot_bg = "#1e1e1e", "#252526", "#2b2b2b"
        self.root.configure(bg=self.bg_color)
        
    def create_widgets(self):
        self.main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        self.main_paned.pack(fill=tk.BOTH, expand=True)
        
        top_frame = tk.Frame(self.main_paned, bg=self.bg_color)
        self.main_paned.add(top_frame, weight=1)
        
        bottom_frame = tk.Frame(self.main_paned, bg=self.bg_color)
        self.main_paned.add(bottom_frame, weight=2) # Gráfica con mayor espacio por defecto
        
        # TODOS LOS CONTROLES SUPERIORES EN UNA SOLA FILA UNIFICADA
        controls_frame = tk.Frame(top_frame, bg=self.bg_color, padx=10, pady=8)
        controls_frame.pack(fill=tk.X, side=tk.TOP)
        
        # Grupo Port
        tk.Label(controls_frame, text="Port:", bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(controls_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        # Grupo Baud
        tk.Label(controls_frame, text="Baud:", bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT, padx=(0, 5))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(controls_frame, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200", "230400", "460800"], width=8, state="readonly")
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # Botón Conectar
        self.connect_btn = tk.Button(controls_frame, text="Connect", command=self.toggle_connection, bg=self.accent_color, fg="white", padx=12, pady=3, relief=tk.FLAT, cursor="hand2")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 8))
        
        # Botón Refrescar
        self.refresh_btn = tk.Button(controls_frame, text="⟳", command=self.refresh_ports, bg=self.bg_color, fg=self.fg_color, padx=6, pady=2, relief=tk.FLAT, cursor="hand2", font=("Arial", 11))
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 20))
        
        # Separador visual sutil entre grupos
        lbl_sep = tk.Label(controls_frame, text="|", bg=self.bg_color, fg="#555555")
        lbl_sep.pack(side=tk.LEFT, padx=(0, 20))
        
        # Botón Clear Output (Mudado a la primera fila)
        self.clear_btn = tk.Button(controls_frame, text="Clear Output", command=self.clear_output, bg=self.accent_color, fg=self.fg_color, padx=10, pady=3, relief=tk.FLAT, cursor="hand2")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # Checkboxes (Mudados a la primera fila)
        self.autoscroll_cb = tk.Checkbutton(controls_frame, text="Autoscroll", variable=self.autoscroll, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.autoscroll_cb.pack(side=tk.LEFT, padx=(0, 12))
        
        self.timestamp_cb = tk.Checkbutton(controls_frame, text="Timestamp", variable=self.show_timestamp, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.timestamp_cb.pack(side=tk.LEFT, padx=(0, 12))
        
        self.save_log_cb = tk.Checkbutton(controls_frame, text="Save Log", variable=self.save_log, command=self.toggle_save_log, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.save_log_cb.pack(side=tk.LEFT)

        # ENTRADA DE COMANDOS (Fijada abajo de forma segura)
        cmd_frame = tk.Frame(top_frame, bg=self.bg_color, padx=10, pady=8)
        cmd_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.input_entry = tk.Entry(cmd_frame, bg=self.input_bg, fg=self.fg_color, font=("Consolas", 10), insertbackground=self.fg_color, relief=tk.FLAT, borderwidth=1, highlightthickness=1, highlightcolor=self.accent_color)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        self.send_btn = tk.Button(cmd_frame, text="Send", command=self.send_command, bg=self.accent_color, fg="white", padx=15, pady=3, relief=tk.FLAT, cursor="hand2")
        self.send_btn.pack(side=tk.RIGHT)
        
        self.input_entry.config(state='disabled')
        self.send_btn.config(state='disabled')
        
        # ÁREA DE SALIDA DE TEXTO (Altura optimizada a 12 líneas para laptops)
        self.output_area = scrolledtext.ScrolledText(top_frame, wrap=tk.WORD, bg=self.output_bg, fg=self.fg_color, font=("Consolas", 9), insertbackground=self.fg_color, relief=tk.FLAT, borderwidth=0, height=12)
        self.output_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=2)
        
        def bloquear_escritura(event):
            if event.state & 0x0004 or event.keysym in ("Up", "Down", "Left", "Right", "Prior", "Next", "Home", "End"): return None
            return "break"
        self.output_area.bind("<Key>", bloquear_escritura)
        self.output_area.tag_config("timestamp", foreground="#6a9955")
        self.output_area.tag_config("error", foreground="#f48771")
        
        # CONTROLES DE LA GRÁFICA (BOTTOM FRAME)
        plot_controls_frame = tk.Frame(bottom_frame, bg=self.bg_color, padx=10, pady=5)
        plot_controls_frame.pack(fill=tk.X, side=tk.TOP)
        
        self.open_log_btn = tk.Button(plot_controls_frame, text="📂 Open Log File", command=self.open_log_file, bg=self.accent_color, fg="white", padx=10, pady=4, relief=tk.FLAT, cursor="hand2")
        self.open_log_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.realtime_plot_cb = tk.Checkbutton(plot_controls_frame, text="Real-time Plot", variable=self.real_time_plot, command=self.toggle_real_time_plot, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.realtime_plot_cb.pack(side=tk.LEFT, padx=(0, 10))
        
        self.clear_plot_btn = tk.Button(plot_controls_frame, text="Clear Plot", command=self.clear_plot, bg=self.accent_color, fg=self.fg_color, padx=10, pady=4, relief=tk.FLAT, cursor="hand2")
        self.clear_plot_btn.pack(side=tk.LEFT)
        
        self.plot_frame = tk.Frame(bottom_frame, bg=self.plot_bg)
        self.plot_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 5))
        
    def setup_matplotlib_figure(self):
        self.figure = Figure(figsize=FIG_SIZE, dpi=100, facecolor=self.plot_bg)
        self.figure.subplots_adjust(left=0.06, right=0.98, top=0.92, bottom=0.15)
        self.ax = self.figure.add_subplot(111)
        self.clear_plot()
        
        self.canvas = FigureCanvasTkAgg(self.figure, self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame)
        self.toolbar.update()
        
    def toggle_real_time_plot(self):
        if self.real_time_plot.get():
            self.clear_plot()
            self.update_plot_from_textarea()
    
    def update_plot_from_textarea(self):
        if not self.real_time_plot.get(): return
        text_content = self.output_area.get(1.0, tk.END)
        datos = cargar_datos_desde_texto(text_content)
        if datos is not None and datos['num_muestras'] > 0:
            self.plot_data = datos
            self._draw_plot(datos)
    
    def _draw_plot(self, datos):
        self.ax.clear()
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        for spine in self.ax.spines.values(): spine.set_color('white')
        self.ax.grid(True, alpha=0.3, color='gray')
        
        mask_temp_valid = [es_temperatura_valida(t) for t in datos['temps']]
        if np.any(mask_temp_valid):
            self.ax.plot(datos['elapsed_seconds'][mask_temp_valid], datos['temps'][mask_temp_valid], 'b-', linewidth=1.5, label='Temperatura Sensor')
        
        if datos['tiene_test']:
            mask_test_valid = [es_temperatura_valida(t) for t in datos['tests']]
            if np.any(mask_test_valid):
                self.ax.plot(datos['elapsed_seconds'][mask_test_valid], datos['tests'][mask_test_valid], 'm-', linewidth=1.5, label='Temperatura Test')
        
        if len(datos['setpoints']) > 0:
            setpoint_inicial = datos['setpoints'][-1]
            self.ax.axhline(y=setpoint_inicial, color='r', linestyle='--', linewidth=1.5, label=f'Setpoint {setpoint_inicial}°C')
            self.ax.fill_between(datos['elapsed_seconds'], setpoint_inicial - 0.1, setpoint_inicial + 0.1, alpha=0.1, color='green')
            self.ax.set_title(f'Setpoint {setpoint_inicial}°C - Gráfica de Temperaturas', fontweight='bold', pad=8)
        else:
            self.ax.set_title(f'Gráfica de Temperaturas - {datos["num_muestras"]} muestras', fontweight='bold', pad=8)
            
        self.ax.set_xlabel('Tiempo (m:s)')
        self.ax.set_ylabel('Temperatura (°C)')
        self.ax.legend(loc='upper left', facecolor='#2b2b2b', edgecolor='white', labelcolor='white')
        
        valid_temps = datos['temps'][mask_temp_valid]
        if len(valid_temps) > 0:
            all_temps = [valid_temps]
            if datos['tiene_test']:
                valid_tests = datos['tests'][[es_temperatura_valida(t) for t in datos['tests']]]
                if len(valid_tests) > 0: all_temps.append(valid_tests)
            combined = np.concatenate(all_temps)
            self.ax.set_ylim(np.min(combined) - 0.5, np.max(combined) + 0.5)
        
        if len(datos['elapsed_seconds']) > 0:
            x_min, x_max = np.min(datos['elapsed_seconds']), np.max(datos['elapsed_seconds'])
            x_range = max(x_max - x_min, 1)
            self.ax.set_xlim(x_min - 0.01*x_range, x_max + 0.01*x_range)
        
        self.ax.xaxis.set_major_locator(ticker.MultipleLocator(250))
        self.ax.xaxis.set_major_formatter(ticker.FuncFormatter(segundos_a_formato))
        
        if self.tooltip_manager:
            self.tooltip_manager.disconnect()
            self.tooltip_manager = None
            
        tests_for_tooltip = datos['tests'] if datos['tiene_test'] else None
        self.tooltip_manager = TooltipManager(self.ax, datos['temps'], datos['elapsed_seconds'], self.figure, tests_for_tooltip, app=self)
        self.cursor = Cursor(self.ax, useblit=False, color='gray', linewidth=0.5, linestyle=':')
        self.canvas.draw()
    
    def clear_plot(self):
        self.ax.clear()
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        for spine in self.ax.spines.values(): spine.set_color('white')
        self.ax.grid(True, alpha=0.3, color='gray')
        self.ax.set_title('Gráfica de Temperatura', color='white', fontweight='bold')
        self.ax.set_xlabel('Tiempo (m:s)')
        self.ax.set_ylabel('Temperatura (°C)')
        
        if self.tooltip_manager:
            self.tooltip_manager.disconnect()
            self.tooltip_manager = None
            
        self.cursor = Cursor(self.ax, useblit=False, color='gray', linewidth=0.5, linestyle=':')
        if self.canvas: self.canvas.draw()
        self.plot_data = None
    
    def open_log_file(self):
        file_path = filedialog.askopenfilename(title="Seleccionar archivo de log", filetypes=[("Text files", "*.txt"), ("Log files", "*.log"), ("All files", "*.*")])
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f: content = f.read()
                if self.real_time_plot.get(): self.real_time_plot.set(False)
                
                datos = cargar_datos_desde_texto(content)
                if datos and datos['num_muestras'] > 0:
                    self.plot_data = datos
                    self._draw_plot(datos)
                    self.append_output(f"Loaded plot from: {os.path.basename(file_path)} ({datos['num_muestras']} samples)\n")
                    
                    mask_valid = [es_temperatura_valida(t) for t in datos['temps']]
                    valid_count = np.sum(mask_valid)
                    if valid_count < datos['num_muestras']:
                        self.append_output(f"Warning: {datos['num_muestras'] - valid_count} invalid temperature values were ignored\n", is_error=True)
                else:
                    self.append_output(f"Error: No valid data found in {file_path}\n", is_error=True)
            except Exception as e:
                self.append_output(f"Error loading file: {str(e)}\n", is_error=True)
    
    def start_plot_updates(self):
        def update():
            if self.real_time_plot.get(): self.update_plot_from_textarea()
            self.plot_update_timer = self.root.after(1000, update)
        self.plot_update_timer = self.root.after(1000, update)
    
    def toggle_save_log(self):
        if self.save_log.get():
            self.show_timestamp.set(True)
            if self.is_connected and self.port_var.get(): self.create_log_file()
        else:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.append_output(f"Log saved to: {self.log_file_path}\n")
                self.log_file_path = None
    
    def create_log_file(self):
        if not self.port_var.get(): return
        port_name = self.port_var.get().replace('/', '_').replace('\\', '_')
        timestamp = datetime.now().strftime("%Y_%m_%d.%H.%M.%S.%f")[:-3]
        filename = f"{port_name}_{timestamp}.txt"
        current_dir = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else os.path.dirname(os.path.abspath(__file__))
        self.log_file_path = os.path.join(current_dir, filename)
        try:
            self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
            self.append_output(f"Logging enabled. Saving to: {self.log_file_path}\n")
            self.log_file.write(f"Serial Monitor Log\nPort: {self.port_var.get()}\nBaud Rate: {self.baud_var.get()}\nStarted: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n" + "-" * 80 + "\n")
            self.log_file.flush()
        except Exception as e:
            self.append_output(f"Error creating log file: {str(e)}\n", is_error=True)
            self.save_log.set(False)
    
    def write_to_log(self, text):
        if self.save_log.get() and self.log_file:
            try: self.log_file.write(text); self.log_file.flush()
            except Exception as e: self.append_output(f"Error writing to log: {str(e)}\n", is_error=True)
    
    def setup_keyboard_events(self):
        self.input_entry.bind('<Return>', lambda e: self.send_command())
        self.input_entry.bind('<Up>', self.navigate_history_up)
        self.input_entry.bind('<Down>', self.navigate_history_down)
    
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list and not self.port_var.get(): self.port_combo.set(port_list[0])
    
    def toggle_connection(self):
        if not self.is_connected: self.connect()
        else: self.disconnect()
    
    def connect(self):
        port, baud = self.port_var.get(), int(self.baud_var.get())
        if not port: self.append_output("Error: No port selected\n", is_error=True); return
        try:
            self.serial_connection = serial.Serial(port, baud, timeout=0.1)
            self.is_connected = True
            self.connect_btn.config(text="Disconnect", bg="#d32f2f")
            self.port_combo.config(state='disabled')
            self.baud_combo.config(state='disabled')
            self.refresh_btn.config(state='disabled')
            self.input_entry.config(state='normal')
            self.send_btn.config(state='normal')
            self.append_output(f"Connected to {port} at {baud} baud\n")
            if self.save_log.get(): self.create_log_file()
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
        except Exception as e: self.append_output(f"Connection error: {str(e)}\n", is_error=True)
    
    def disconnect(self):
        if self.serial_connection and self.serial_connection.is_open: self.serial_connection.close()
        self.is_connected = False
        if self.log_file:
            self.log_file.write("\n" + "-" * 80 + "\n" + f"Log ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.close(); self.log_file = None
            if self.log_file_path: self.append_output(f"Log saved to: {self.log_file_path}\n"); self.log_file_path = None
        self.connect_btn.config(text="Connect", bg=self.accent_color)
        self.port_combo.config(state='readonly')
        self.baud_combo.config(state='readonly')
        self.refresh_btn.config(state='normal')
        self.input_entry.config(state='disabled')
        self.send_btn.config(state='disabled')
        self.append_output("Disconnected from serial port\n")
    
    def read_serial(self):
        while self.is_connected and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting:
                    data = self.serial_connection.readline()
                    if data:
                        decoded_data = data.decode('utf-8', errors='ignore')
                        self.root.after(0, self.append_output, decoded_data)
                time.sleep(0.01)
            except Exception as e:
                if self.is_connected: self.root.after(0, self.append_output, f"Read error: {str(e)}\n", True)
                break
    
    def send_command(self):
        if not self.is_connected: self.append_output("Not connected to serial port\n", is_error=True); return
        command = self.input_entry.get().strip()
        if not command: return
        if not self.command_history or self.command_history[-1] != command: self.command_history.append(command)
        self.history_index = len(self.command_history)
        timestamp = datetime.now().strftime("%H:%M:%S:%f")[:-3] + " -> " if self.show_timestamp.get() else ""
        self.append_output(f"\n{timestamp}→ {command}\n")
        try: self.serial_connection.write((command + "\r\n").encode())
        except Exception as e: self.append_output(f"Send error: {str(e)}\n", is_error=True)
        self.input_entry.delete(0, tk.END)
    
    def navigate_history_up(self, event):
        if self.command_history and self.history_index > 0:
            self.history_index -= 1
            self.input_entry.delete(0, tk.END)
            self.input_entry.insert(0, self.command_history[self.history_index])
        return "break"
    
    def navigate_history_down(self, event):
        if self.command_history and self.history_index < len(self.command_history) - 1:
            self.history_index += 1
            self.input_entry.delete(0, tk.END)
            self.input_entry.insert(0, self.command_history[self.history_index])
        elif self.history_index == len(self.command_history) - 1:
            self.history_index = len(self.command_history)
            self.input_entry.delete(0, tk.END)
        return "break"
    
    def append_output(self, text, is_error=False):
        self.output_area.config(state='normal')
        timestamped_text = ""
        if self.show_timestamp.get() and not text.startswith("\n") and not text.startswith("→"):
            timestamp = datetime.now().strftime("%H:%M:%S:%f")[:-3] + " -> "
            timestamped_text = timestamp
            self.output_area.insert(tk.END, timestamp, "timestamp")
        if is_error: self.output_area.insert(tk.END, text, "error")
        else: self.output_area.insert(tk.END, text)
        if self.save_log.get(): self.write_to_log(timestamped_text + text)
        if self.autoscroll.get(): self.output_area.see(tk.END)
        self.output_area.config(state='disabled')
    
    def clear_output(self):
        self.output_area.config(state='normal')
        self.output_area.delete(1.0, tk.END)
        self.output_area.config(state='disabled')
        if self.real_time_plot.get(): self.clear_plot()
    
    def on_closing(self):
        if self.plot_update_timer: self.root.after_cancel(self.plot_update_timer)
        if self.tooltip_manager: self.tooltip_manager.disconnect()
        self.disconnect()
        self.root.destroy()

    def crear_menus_contextuales(self):
        self.menu_output = tk.Menu(self.root, tearoff=0, bg="#333333", fg="white", activebackground="#007acc")
        self.menu_output.add_command(label="Copiar (Ctrl+C)", command=lambda: self.output_area.event_generate("<<Copy>>"))
        self.menu_output.add_command(label="Seleccionar todo", command=lambda: self.output_area.tag_add("sel", "1.0", "end"))
        self.menu_output.add_separator()
        self.menu_output.add_command(label="Limpiar pantalla", command=self.clear_output)
        
        self.menu_input = tk.Menu(self.root, tearoff=0, bg="#333333", fg="white", activebackground="#007acc")
        self.menu_input.add_command(label="Cortar (Ctrl+X)", command=lambda: self.input_entry.event_generate("<<Cut>>"))
        self.menu_input.add_command(label="Copiar (Ctrl+C)", command=lambda: self.input_entry.event_generate("<<Copy>>"))
        self.menu_input.add_command(label="Pegar (Ctrl+V)", command=lambda: self.input_entry.event_generate("<<Paste>>"))
        self.menu_input.add_separator()
        self.menu_input.add_command(label="Seleccionar todo", command=lambda: self.input_entry.select_range(0, tk.END))

        def mostrar_menu_output(event):
            self.menu_output.tk_popup(event.x_root, event.y_root)
            return "break"
        def mostrar_menu_input(event):
            if self.input_entry.cget('state') == 'normal': self.menu_input.tk_popup(event.x_root, event.y_root)
            return "break"

        self.output_area.bind("<Button-3>", mostrar_menu_output)
        self.input_entry.bind("<Button-3>", mostrar_menu_input)

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialMonitorWithPlot(root)
    root.mainloop()