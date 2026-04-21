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

# Intentar importar mpl_fontkit (opcional)
try:
    import mpl_fontkit as fk
except ImportError:
    fk = None

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

# Configuración de tamaño de figura
FIG_SIZE = (12, 5)

# ============================================================
# REGEX COMPILADAS
# ============================================================
PID_LINE = re.compile(
    r"(\d{2}:\d{2}:\d{2}[:.]\d{3})\s*->\s*State:\s*(\w+)\s*\|\s*Error:\s*(-?\d+\.?\d*)\s*\|\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)(?:\s*\|\s*Test:\s*(-?\d+\.?\d*))?(?:\s*\|\s*Diff:\s*(-?\d+\.?\d*))?\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

# ============================================================
# FUNCIONES DE UTILIDAD
# ============================================================
def segundos_a_formato(x, pos=None):
    """Convierte segundos a formato MM:SS"""
    minutos = int(x) // 60
    segundos = int(x) % 60
    return f"{minutos}:{segundos:02d}"

def cargar_datos_desde_texto(texto):
    """Carga y procesa los datos desde el contenido del textarea"""
    timestamps, temps, setpoints = [], [], []
    tests, diffs = [], []
    
    tiene_test = False
    tiene_diff = False
    
    lines = texto.split('\n')
    for line in lines:
        if m := PID_LINE.search(line):
            time_str = m.group(1).replace('.', ':')
            try:
                timestamps.append(datetime.strptime(time_str, "%H:%M:%S:%f"))
                temps.append(float(m.group(5)))
                setpoints.append(float(m.group(8)))
                
                test_val = m.group(6)
                diff_val = m.group(7)
                
                if test_val is not None:
                    tests.append(float(test_val))
                    tiene_test = True
                else:
                    tests.append(None)
                
                if diff_val is not None:
                    diffs.append(float(diff_val))
                    tiene_diff = True
                else:
                    diffs.append(None)
            except:
                continue
    
    if not temps:
        return None
    
    start = timestamps[0]
    elapsed_seconds = np.array([(ts - start).total_seconds() for ts in timestamps])
    
    tests_array = np.array(tests, dtype=float)
    diffs_array = np.array(diffs, dtype=float)
    
    return {
        'elapsed_seconds': elapsed_seconds,
        'temps': np.array(temps),
        'setpoints': np.array(setpoints),
        'tests': tests_array,
        'diffs': diffs_array,
        'tiene_test': tiene_test,
        'tiene_diff': tiene_diff,
        'num_muestras': len(temps)
    }

# ============================================================
# CLASE PRINCIPAL UNIFICADA
# ============================================================
class SerialMonitorWithPlot:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Monitor with Real-time Plot")
        self.root.geometry("1400x800")
        self.root.minsize(1200, 600)
        
        # Variables seriales
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
        
        # Variables de plot
        self.real_time_plot = tk.BooleanVar(value=False)
        self.plot_data = None
        self.figure = None
        self.ax = None
        self.canvas = None
        self.toolbar = None
        self.tooltip_manager = None
        self.cursor = None
        
        # Configurar estilo
        self.setup_styles()
        
        # Crear interfaz
        self.create_widgets()
        
        # Configurar la figura de matplotlib
        self.setup_matplotlib_figure()
        
        # Actualizar puertos disponibles
        self.refresh_ports()
        
        # Configurar eventos de teclado
        self.setup_keyboard_events()
        
        # Configurar cierre de aplicación
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Timer para actualización en tiempo real
        self.plot_update_timer = None
        self.start_plot_updates()
        
    def setup_styles(self):
        """Configurar estilos para una apariencia minimalista"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configurar colores
        self.bg_color = "#2b2b2b"
        self.fg_color = "#ffffff"
        self.accent_color = "#007acc"
        self.output_bg = "#1e1e1e"
        self.input_bg = "#252526"
        self.plot_bg = "#2b2b2b"
        
        self.root.configure(bg=self.bg_color)
        
    def create_widgets(self):
        # Frame principal con PanedWindow para redimensionar
        self.main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        self.main_paned.pack(fill=tk.BOTH, expand=True)
        
        # Frame izquierdo para controles y output
        left_frame = tk.Frame(self.main_paned, bg=self.bg_color)
        self.main_paned.add(left_frame, weight=2)
        
        # Frame derecho para la gráfica
        right_frame = tk.Frame(self.main_paned, bg=self.bg_color)
        self.main_paned.add(right_frame, weight=1)
        
        # ==================== LEFT FRAME (Serial Monitor) ====================
        # Frame superior para controles
        top_frame = tk.Frame(left_frame, bg=self.bg_color, padx=10, pady=10)
        top_frame.pack(fill=tk.X)
        
        # Puerto
        tk.Label(top_frame, text="Port:", bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        # Baud rate
        tk.Label(top_frame, text="Baud:", bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT, padx=(0, 5))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(top_frame, textvariable=self.baud_var, 
                                       values=["9600", "19200", "38400", "57600", "115200", "230400", "460800"],
                                       width=8, state="readonly")
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        # Botón Connect/Disconnect
        self.connect_btn = tk.Button(top_frame, text="Connect", command=self.toggle_connection,
                                     bg=self.accent_color, fg="white", padx=15, pady=5,
                                     relief=tk.FLAT, cursor="hand2")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Botón Refresh Ports
        self.refresh_btn = tk.Button(top_frame, text="⟳", command=self.refresh_ports,
                                     bg=self.bg_color, fg=self.fg_color, padx=8, pady=5,
                                     relief=tk.FLAT, cursor="hand2", font=("Arial", 12))
        self.refresh_btn.pack(side=tk.LEFT)
        
        # Frame para controles de output
        output_controls_frame = tk.Frame(left_frame, bg=self.bg_color, padx=10, pady=5)
        output_controls_frame.pack(fill=tk.X)
        
        # Botón Clear Output
        self.clear_btn = tk.Button(output_controls_frame, text="Clear Output", command=self.clear_output,
                                   bg=self.accent_color, fg=self.fg_color, padx=8, pady=3,
                                   relief=tk.FLAT, cursor="hand2")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Checkbutton Autoscroll
        self.autoscroll_cb = tk.Checkbutton(output_controls_frame, text="Autoscroll", 
                                            variable=self.autoscroll,
                                            bg=self.bg_color, fg=self.fg_color,
                                            selectcolor=self.bg_color,
                                            activebackground=self.bg_color,
                                            relief=tk.FLAT, cursor="hand2")
        self.autoscroll_cb.pack(side=tk.LEFT, padx=(0, 10))
        
        # Checkbutton Timestamp
        self.timestamp_cb = tk.Checkbutton(output_controls_frame, text="Timestamp", 
                                           variable=self.show_timestamp,
                                           bg=self.bg_color, fg=self.fg_color,
                                           selectcolor=self.bg_color,
                                           activebackground=self.bg_color,
                                           relief=tk.FLAT, cursor="hand2")
        self.timestamp_cb.pack(side=tk.LEFT, padx=(0, 10))
        
        # Checkbutton Save Log
        self.save_log_cb = tk.Checkbutton(output_controls_frame, text="Save Log", 
                                          variable=self.save_log,
                                          command=self.toggle_save_log,
                                          bg=self.bg_color, fg=self.fg_color,
                                          selectcolor=self.bg_color,
                                          activebackground=self.bg_color,
                                          relief=tk.FLAT, cursor="hand2")
        self.save_log_cb.pack(side=tk.LEFT)
        
        # Área de salida (multilínea)
        self.output_area = scrolledtext.ScrolledText(left_frame, wrap=tk.WORD, 
                                                      bg=self.output_bg, fg=self.fg_color,
                                                      font=("Consolas", 9), insertbackground=self.fg_color,
                                                      relief=tk.FLAT, borderwidth=0)
        self.output_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        
        # Configurar tags para colores
        self.output_area.tag_config("timestamp", foreground="#6a9955")
        self.output_area.tag_config("error", foreground="#f48771")
        
        # Frame inferior para entrada de comandos
        bottom_frame = tk.Frame(left_frame, bg=self.bg_color, padx=10, pady=10)
        bottom_frame.pack(fill=tk.X)
        
        # Campo de entrada de comandos
        self.input_entry = tk.Entry(bottom_frame, bg=self.input_bg, fg=self.fg_color,
                                    font=("Consolas", 9), insertbackground=self.fg_color,
                                    relief=tk.FLAT, borderwidth=1, highlightthickness=1,
                                    highlightcolor=self.accent_color)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        # Botón Send
        self.send_btn = tk.Button(bottom_frame, text="Send", command=self.send_command,
                                  bg=self.accent_color, fg="white", padx=15, pady=5,
                                  relief=tk.FLAT, cursor="hand2")
        self.send_btn.pack(side=tk.RIGHT)
        
        # Estado inicial de los controles
        self.input_entry.config(state='disabled')
        self.send_btn.config(state='disabled')
        
        # ==================== RIGHT FRAME (Plot Controls) ====================
        plot_controls_frame = tk.Frame(right_frame, bg=self.bg_color, padx=10, pady=10)
        plot_controls_frame.pack(fill=tk.X)
        
        # Botón Open Log File
        self.open_log_btn = tk.Button(plot_controls_frame, text="📂 Open Log File", command=self.open_log_file,
                                      bg=self.accent_color, fg="white", padx=10, pady=5,
                                      relief=tk.FLAT, cursor="hand2")
        self.open_log_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Checkbutton Real-time Plot
        self.realtime_plot_cb = tk.Checkbutton(plot_controls_frame, text="Real-time Plot", 
                                               variable=self.real_time_plot,
                                               command=self.toggle_real_time_plot,
                                               bg=self.bg_color, fg=self.fg_color,
                                               selectcolor=self.bg_color,
                                               activebackground=self.bg_color,
                                               relief=tk.FLAT, cursor="hand2")
        self.realtime_plot_cb.pack(side=tk.LEFT, padx=(0, 10))
        
        # Botón Clear Plot
        self.clear_plot_btn = tk.Button(plot_controls_frame, text="Clear Plot", command=self.clear_plot,
                                        bg=self.accent_color, fg=self.fg_color, padx=10, pady=5,
                                        relief=tk.FLAT, cursor="hand2")
        self.clear_plot_btn.pack(side=tk.LEFT)
        
        # Frame para la figura de matplotlib
        self.plot_frame = tk.Frame(right_frame, bg=self.plot_bg)
        self.plot_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        
    def setup_matplotlib_figure(self):
        """Configurar la figura de matplotlib"""
        self.figure = Figure(figsize=FIG_SIZE, dpi=100, facecolor=self.plot_bg)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        
        for spine in self.ax.spines.values():
            spine.set_color('white')
        
        self.ax.grid(True, alpha=0.3, color='gray')
        self.ax.set_xlabel('Tiempo (m:s)', color='white')
        self.ax.set_ylabel('Temperatura (°C)', color='white')
        self.ax.set_title('Gráfica de Temperatura', color='white', fontweight='bold')
        
        self.canvas = FigureCanvasTkAgg(self.figure, self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Agregar toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame)
        self.toolbar.update()
        
    def toggle_real_time_plot(self):
        """Habilitar/deshabilitar gráfica en tiempo real"""
        if self.real_time_plot.get():
            # Limpiar gráfica actual
            self.clear_plot()
            # Actualizar inmediatamente
            self.update_plot_from_textarea()
        else:
            # Limpiar gráfica
            self.clear_plot()
    
    def update_plot_from_textarea(self):
        """Actualizar la gráfica con los datos del textarea"""
        if not self.real_time_plot.get():
            return
            
        text_content = self.output_area.get(1.0, tk.END)
        datos = cargar_datos_desde_texto(text_content)
        
        if datos is None or datos['num_muestras'] == 0:
            return
            
        self.plot_data = datos
        self._draw_plot(datos)
    
    def _draw_plot(self, datos):
        """Dibujar la gráfica con los datos proporcionados"""
        self.ax.clear()
        
        # Configurar colores del eje
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        for spine in self.ax.spines.values():
            spine.set_color('white')
        self.ax.grid(True, alpha=0.3, color='gray')
        
        # Graficar temperatura principal
        self.ax.plot(datos['elapsed_seconds'], datos['temps'], 'b-', linewidth=1.5, label='Temperatura Sensor')
        
        # Graficar temperatura test si existe
        if datos['tiene_test']:
            mask_test = ~np.isnan(datos['tests'])
            if np.any(mask_test):
                self.ax.plot(datos['elapsed_seconds'][mask_test], datos['tests'][mask_test], 
                           'm-', linewidth=1.5, label='Temperatura Test')
        
        # Setpoint
        if len(datos['setpoints']) > 0:
            setpoint_inicial = datos['setpoints'][-1]
            self.ax.axhline(y=setpoint_inicial, color='r', linestyle='--', linewidth=1.5, 
                           label=f'Setpoint {setpoint_inicial}°C')
            self.ax.fill_between(datos['elapsed_seconds'], setpoint_inicial - 0.1, setpoint_inicial + 0.1, 
                                alpha=0.1, color='green')
        
        self.ax.set_title(f'Setpoint {setpoint_inicial}°C - Gráfica de Temperaturas - {datos["num_muestras"]} muestras', fontweight='bold', pad=8)
        self.ax.set_xlabel('Tiempo (m:s)')
        self.ax.set_ylabel('Temperatura (°C)')
        
        self.ax.legend(loc='upper left', facecolor='#2b2b2b', edgecolor='white', labelcolor='white')
        
        # Ajustar límites
        if len(datos['temps']) > 0:
            all_temps = [datos['temps']]
            if datos['tiene_test']:
                valid_tests = datos['tests'][~np.isnan(datos['tests'])]
                if len(valid_tests) > 0:
                    all_temps.append(valid_tests)
            combined = np.concatenate(all_temps) if all_temps else datos['temps']
            y_min = np.min(combined) - 0.5
            y_max = np.max(combined) + 0.5
            self.ax.set_ylim(y_min, y_max)
        
        if len(datos['elapsed_seconds']) > 0:
            x_min = np.min(datos['elapsed_seconds'])
            x_max = np.max(datos['elapsed_seconds'])
            x_range = x_max - x_min
            self.ax.set_xlim(x_min - 0.01*x_range, x_max + 0.01*x_range)
        
        self.ax.xaxis.set_major_locator(ticker.MultipleLocator(250))
        self.ax.xaxis.set_major_formatter(ticker.FuncFormatter(segundos_a_formato))
        
        self.canvas.draw()
    
    def clear_plot(self):
        """Limpiar la gráfica"""
        self.ax.clear()
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.title.set_color('white')
        for spine in self.ax.spines.values():
            spine.set_color('white')
        self.ax.grid(True, alpha=0.3, color='gray')
        self.ax.set_title('Gráfica de Temperatura', color='white', fontweight='bold')
        self.ax.set_xlabel('Tiempo (m:s)', color='white')
        self.ax.set_ylabel('Temperatura (°C)', color='white')
        self.canvas.draw()
        self.plot_data = None
    
    def open_log_file(self):
        """Abrir un archivo de log existente y graficarlo"""
        file_path = filedialog.askopenfilename(
            title="Seleccionar archivo de log",
            filetypes=[("Text files", "*.txt"), ("Log files", "*.log"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # Deshabilitar tiempo real si estaba activo
                was_realtime = self.real_time_plot.get()
                if was_realtime:
                    self.real_time_plot.set(False)
                
                # Cargar y graficar datos
                datos = cargar_datos_desde_texto(content)
                if datos and datos['num_muestras'] > 0:
                    self.plot_data = datos
                    self._draw_plot(datos)
                    self.append_output(f"Loaded plot from: {os.path.basename(file_path)} ({datos['num_muestras']} samples)\n")
                else:
                    self.append_output(f"Error: No valid data found in {file_path}\n", is_error=True)
                    
            except Exception as e:
                self.append_output(f"Error loading file: {str(e)}\n", is_error=True)
    
    def start_plot_updates(self):
        """Iniciar actualizaciones periódicas de la gráfica"""
        def update():
            if self.real_time_plot.get():
                self.update_plot_from_textarea()
            self.plot_update_timer = self.root.after(1000, update)  # Actualizar cada segundo
        
        self.plot_update_timer = self.root.after(1000, update)
    
    # ==================== MÉTODOS DEL MONITOR SERIAL ====================
    
    def toggle_save_log(self):
        """Habilitar/deshabilitar guardado de log"""
        if self.save_log.get():
            self.show_timestamp.set(True)
            if self.is_connected and self.port_var.get():
                self.create_log_file()
        else:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.append_output(f"Log saved to: {self.log_file_path}\n")
                self.log_file_path = None
    
    def create_log_file(self):
        """Crear archivo de log con formato: PORT_YYYY_MM_DD.HH.MM.SS.ms.txt"""
        if not self.port_var.get():
            return
            
        port_name = self.port_var.get().replace('/', '_').replace('\\', '_')
        timestamp = datetime.now().strftime("%Y_%m_%d.%H.%M.%S.%f")[:-3]
        filename = f"{port_name}_{timestamp}.txt"
        
        if getattr(sys, 'frozen', False):
            current_dir = os.path.dirname(sys.executable)
        else:
            current_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.log_file_path = os.path.join(current_dir, filename)
        
        try:
            self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
            self.append_output(f"Logging enabled. Saving to: {self.log_file_path}\n")
            self.log_file.write(f"Serial Monitor Log\n")
            self.log_file.write(f"Port: {self.port_var.get()}\n")
            self.log_file.write(f"Baud Rate: {self.baud_var.get()}\n")
            self.log_file.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.write("-" * 80 + "\n")
            self.log_file.flush()
        except Exception as e:
            self.append_output(f"Error creating log file: {str(e)}\n", is_error=True)
            self.save_log.set(False)
    
    def write_to_log(self, text):
        """Escribir texto en el archivo de log"""
        if self.save_log.get() and self.log_file:
            try:
                self.log_file.write(text)
                self.log_file.flush()
            except Exception as e:
                self.append_output(f"Error writing to log: {str(e)}\n", is_error=True)
    
    def setup_keyboard_events(self):
        """Configurar eventos de teclado"""
        self.input_entry.bind('<Return>', lambda e: self.send_command())
        self.input_entry.bind('<Up>', self.navigate_history_up)
        self.input_entry.bind('<Down>', self.navigate_history_down)
    
    def refresh_ports(self):
        """Actualizar lista de puertos disponibles"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list and not self.port_var.get():
            self.port_combo.set(port_list[0])
    
    def toggle_connection(self):
        """Conectar o desconectar del puerto serial"""
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """Conectar al puerto serial"""
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        
        if not port:
            self.append_output("Error: No port selected\n", is_error=True)
            return
        
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
            
            if self.save_log.get():
                self.create_log_file()
            
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
        except Exception as e:
            self.append_output(f"Connection error: {str(e)}\n", is_error=True)
    
    def disconnect(self):
        """Desconectar del puerto serial"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        
        self.is_connected = False
        
        if self.log_file:
            self.log_file.write("\n" + "-" * 80 + "\n")
            self.log_file.write(f"Log ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.close()
            self.log_file = None
            if self.log_file_path:
                self.append_output(f"Log saved to: {self.log_file_path}\n")
                self.log_file_path = None
        
        self.connect_btn.config(text="Connect", bg=self.accent_color)
        self.port_combo.config(state='readonly')
        self.baud_combo.config(state='readonly')
        self.refresh_btn.config(state='normal')
        self.input_entry.config(state='disabled')
        self.send_btn.config(state='disabled')
        
        self.append_output("Disconnected from serial port\n")
    
    def read_serial(self):
        """Hilo para leer datos del puerto serial"""
        while self.is_connected and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting:
                    data = self.serial_connection.readline()
                    if data:
                        decoded_data = data.decode('utf-8', errors='ignore')
                        self.root.after(0, self.append_output, decoded_data)
                time.sleep(0.01)
            except Exception as e:
                if self.is_connected:
                    self.root.after(0, self.append_output, f"Read error: {str(e)}\n", True)
                break
    
    def send_command(self):
        """Enviar comando al ESP32"""
        if not self.is_connected:
            self.append_output("Not connected to serial port\n", is_error=True)
            return
        
        command = self.input_entry.get().strip()
        if not command:
            return
        
        if not self.command_history or self.command_history[-1] != command:
            self.command_history.append(command)
        self.history_index = len(self.command_history)
        
        timestamp = datetime.now().strftime("%H:%M:%S:%f")[:-3] + " -> " if self.show_timestamp.get() else ""
        self.append_output(f"\n{timestamp}→ {command}\n")
        
        try:
            self.serial_connection.write((command + "\r\n").encode())
        except Exception as e:
            self.append_output(f"Send error: {str(e)}\n", is_error=True)
        
        self.input_entry.delete(0, tk.END)
    
    def navigate_history_up(self, event):
        """Navegar hacia arriba en el historial"""
        if self.command_history and self.history_index > 0:
            self.history_index -= 1
            self.input_entry.delete(0, tk.END)
            self.input_entry.insert(0, self.command_history[self.history_index])
        return "break"
    
    def navigate_history_down(self, event):
        """Navegar hacia abajo en el historial"""
        if self.command_history and self.history_index < len(self.command_history) - 1:
            self.history_index += 1
            self.input_entry.delete(0, tk.END)
            self.input_entry.insert(0, self.command_history[self.history_index])
        elif self.history_index == len(self.command_history) - 1:
            self.history_index = len(self.command_history)
            self.input_entry.delete(0, tk.END)
        return "break"
    
    def append_output(self, text, is_error=False):
        """Agregar texto al área de salida"""
        timestamped_text = ""
        
        if self.show_timestamp.get() and not text.startswith("\n") and not text.startswith("→"):
            timestamp = datetime.now().strftime("%H:%M:%S:%f")[:-3] + " -> "
            timestamped_text = timestamp
            self.output_area.insert(tk.END, timestamp, "timestamp")
        
        if is_error:
            self.output_area.insert(tk.END, text, "error")
        else:
            self.output_area.insert(tk.END, text)
        
        if self.save_log.get():
            log_text = timestamped_text + text
            self.write_to_log(log_text)
        
        if self.autoscroll.get():
            self.output_area.see(tk.END)
    
    def clear_output(self):
        """Limpiar el área de salida"""
        self.output_area.delete(1.0, tk.END)
        if self.real_time_plot.get():
            self.clear_plot()
    
    def on_closing(self):
        """Manejar cierre de la aplicación"""
        if self.plot_update_timer:
            self.root.after_cancel(self.plot_update_timer)
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialMonitorWithPlot(root)
    root.mainloop()