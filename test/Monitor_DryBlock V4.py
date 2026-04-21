import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
from click import style
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
# RISE_TOL = 0.5
SETPOINT_TOL = 0.1
# SETTLING_TOL = 0.1
# SETTLING_TIME_SEC = 10
# SOLID_STABILITY_SEC = 60
# MAX_PHYSICAL_STEP = 1.0
# REFRESH_INTERVAL = 1.0

FIG_SIZE = (12, 4)
SENSOR_ERROR_VALUE = -999.0

PID_LINE = re.compile(
    r"(\d{2}:\d{2}:\d{2}[:.]\d{3})\s*->\s*State:\s*(\w+)\s*\|\s*Error:\s*(-?\d+\.?\d*)\s*\|\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)(?:\s*\|\s*Test:\s*(-?\d+\.?\d*))?(?:\s*\|\s*Diff:\s*(-?\d+\.?\d*))?\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

TOLERANCE_LINE = re.compile(r'\[TOLERANCE\]:\s*([0-9.]+)')
FAN_PWM_LINE = re.compile(r"Fan PWM set to\s+([\d\.]+)%")
COOLING_PWM_LINE = re.compile(r"Cooling set to\s+([\d\.]+)%")
HEATER_PWM_LINE = re.compile(r"Heater set to\s+([\d\.]+)%")

# Temas predefinidos
THEMES = {
    "Dark": {
        "bg": "#2b2b2b",
        "fg": "white",
        "accent": "#007acc",
        "output_bg": "#1e1e1e",
        "input_bg": "#3c3c3c",
        "plot_bg": "#2b2b2b",
        "plot_face": "#1e1e1e",
        "grid_color": "#555555",
        "tick_color": "white",
        "spine_color": "gray",
        "combo_bg": "#3c3c3c",
        "combo_fg": "white"
    },
    "Light": {
        "bg": "#f0f0f0",
        "fg": "black",
        "accent": "#0078d4",
        "output_bg": "white",
        "input_bg": "white",
        "plot_bg": "#f8f8f8",
        "plot_face": "white",
        "grid_color": "#cccccc",
        "tick_color": "black",
        "spine_color": "black",
        "combo_bg": "white",
        "combo_fg": "black"
    }
}

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
        
        if m := TOLERANCE_LINE.search(line):
            try:
                global SETPOINT_TOL
                SETPOINT_TOL = float(m.group(1))
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
# SIDEBAR COLAPSABLE DE COMANDOS
# ============================================================
class CollapsibleSidebar:
    """Panel lateral colapsable que muestra información de comandos"""
    def __init__(self, parent, bg_color, fg_color, accent_color):
        self.parent = parent
        self.bg_color = bg_color
        self.fg_color = fg_color
        self.accent_color = accent_color
        self.expanded = True
        self.sidebar_width = 380
        
        # Contenedor principal
        self.sidebar_container = tk.Frame(parent, bg=bg_color)
        
        # Panel izquierdo (contenido)
        self.left_panel = tk.Frame(self.sidebar_container, bg=bg_color, width=self.sidebar_width)
        self.left_panel.pack_propagate(False)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y)
        
        # Barra divisora con botón toggle
        self.divider_bar = tk.Frame(self.sidebar_container, bg="#555555", width=12)
        self.divider_bar.pack_propagate(False)
        self.divider_bar.pack(side=tk.LEFT, fill=tk.Y)
        
        # Botón toggle
        self.toggle_btn = tk.Button(self.divider_bar, text="◀",
                                    font=("Arial", 7, "bold"),
                                    bd=0, bg="#555555", fg="white",
                                    activebackground="#777777",
                                    relief=tk.FLAT, command=self.toggle)
        self.toggle_btn.pack(side=tk.TOP, fill=tk.X, pady=10)
        
        # Inicializar contenido
        self.init_content()
        
    def init_content(self):
        """Inicializa el contenido del sidebar con los comandos"""
        # Frame interno con scroll
        self.content_frame = tk.Frame(self.left_panel, bg=self.bg_color)
        self.content_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        # Título
        title = tk.Label(self.content_frame, text="📋 Comandos Disponibles", 
                        bg=self.bg_color, fg=self.accent_color,
                        font=("Arial", 11, "bold"))
        title.pack(anchor="w", pady=(0, 10))
        
        # Separador
        separator = tk.Frame(self.content_frame, bg=self.accent_color, height=2)
        separator.pack(fill=tk.X, pady=(0, 10))
        
        # Frame con scroll para los comandos
        canvas_frame = tk.Frame(self.content_frame, bg=self.bg_color)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(canvas_frame, bg=self.bg_color, highlightthickness=0)
        scrollbar = tk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas, bg=self.bg_color)
        
        self.scrollable_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=scrollbar.set)
        
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Comandos agrupados
        self.add_command_section("📊 CONTROL PRINCIPAL")
        self.add_command("$HEAT", "Iniciar calentamiento (control PID)")
        self.add_command("$COOL", "Iniciar enfriamiento (control PID)")
        self.add_command("$STOP", "Detener calentamiento/enfriamiento/calibración")
        self.add_command("$RUN", "Iniciar ciclo de calibración")
        
        self.add_command_section("⚙️ CONFIGURACIÓN PID")
        self.add_command("$SET KP <value>", "Establecer Kp del PID")
        self.add_command("$SET TI <value>", "Establecer Ti del PID (segundos)")
        self.add_command("$SET TD <value>", "Establecer Td del PID (segundos)")
        self.add_command("$SET PERIOD <value>", "Establecer período del ciclo PID (segundos)")
        self.add_command("$SET STABLE <value>", "Establecer tiempo de estabilidad (segundos)")
        self.add_command("$SET WAITPROCESS <value>", "Establecer tiempo máximo de proceso (segundos)")
        
        self.add_command_section("🌡️ TEMPERATURA")
        self.add_command("$TEMP", "Mostrar temperaturas y setpoint actual")
        self.add_command("$SET SETPOINT <value>[unit]", "Establecer temperatura objetivo")
        self.add_command("$SET SCALE <C|F>", "Establecer escala (Celsius/Fahrenheit)")
        
        self.add_command_section("🔧 CALIBRACIÓN")
        self.add_command("$SET MASTER_CORR <1|2|3> <value>[unit]", "Offset corrección sensor maestro")
        self.add_command("$SET TEST_CORR <1|2|3> <value>[unit]", "Offset corrección sensor test")
        self.add_command("$SET CAL_POINT <1|2|3> <value>[unit]", "Establecer punto de calibración")
        self.add_command("$SET TOLERANCE <value>[unit]", "Establecer tolerancia de calibración")
        
        self.add_command_section("⚠️ ALARMAS Y SEGURIDAD")
        self.add_command("$SET HIGH <value>[unit]", "Establecer límite alto de alarma")
        self.add_command("$SET LOW <value>[unit]", "Establecer límite bajo de alarma")
        self.add_command("$SET DANGER <value>[unit]", "Establecer límite de temperatura peligrosa")
        self.add_command("$SET SAFE <value>[unit]", "Establecer límite de temperatura segura")
        
        self.add_command_section("🛠️ SENSORES")
        self.add_command("$SET SENSOR <100|1000>", "Establecer tipo de sensor (PT100/PT1000)")
        self.add_command("$SET WIRES <2|3|4>", "Establecer modo de cableado del sensor")
        
        self.add_command_section("💾 SISTEMA")
        self.add_command("$DEFAULT", "Restablecer configuraciones por defecto")
        self.add_command("$SAVE", "Guardar configuraciones en LittleFS")
        self.add_command("$SHOW", "Mostrar configuraciones y temperaturas actuales")
        self.add_command("$SET TIME <YYYYMMDDHHMM>", "Establecer fecha/hora RTC")
        self.add_command("$TIME", "Mostrar fecha/hora RTC actual")
        self.add_command("$RESET_CONFIG", "Restablecer configuraciones guardadas (LittleFS)")
        self.add_command("$HELP", "Mostrar mensaje de ayuda")
        
        # Vincular scroll del mouse
        self.canvas.bind("<Enter>", self._bind_mousewheel)
        self.canvas.bind("<Leave>", self._unbind_mousewheel)
    
    def _bind_mousewheel(self, event):
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
    
    def _unbind_mousewheel(self, event):
        self.canvas.unbind_all("<MouseWheel>")
    
    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
    
    def add_command_section(self, title):
        """Agrega una sección con título"""
        section_label = tk.Label(self.scrollable_frame, text=title, 
                                 bg=self.bg_color, fg=self.accent_color,
                                 font=("Arial", 9, "bold"))
        section_label.pack(anchor="w", pady=(8, 3))
    
    def add_command(self, command, description):
        """Agrega un comando con su descripción"""
        cmd_frame = tk.Frame(self.scrollable_frame, bg=self.bg_color)
        cmd_frame.pack(fill=tk.X, pady=2)
        
        # Comando (monospace, color destacado)
        cmd_label = tk.Label(cmd_frame, text=command, 
                            bg=self.bg_color, fg="#98c379" if self.bg_color == "#2b2b2b" else "#0078d4",
                            font=("Consolas", 8, "bold"))
        cmd_label.pack(side=tk.LEFT, anchor="w")
        
        # Descripción
        desc_label = tk.Label(cmd_frame, text=description,
                             bg=self.bg_color, fg=self.fg_color,
                             font=("Arial", 8))
        desc_label.pack(side=tk.LEFT, padx=(8, 0), anchor="w")
    
    def pack(self, **kwargs):
        """Empaqueta el sidebar"""
        self.sidebar_container.pack(**kwargs)
    
    def toggle(self):
        """Alterna la visibilidad del panel lateral"""
        if self.expanded:
            self.left_panel.pack_forget()
            self.toggle_btn.configure(text="▶")
            self.expanded = False
        else:
            self.left_panel.pack(side=tk.LEFT, fill=tk.Y, before=self.divider_bar)
            self.toggle_btn.configure(text="◀")
            self.expanded = True
    
    def update_theme(self, bg_color, fg_color, accent_color):
        """Actualiza los colores del sidebar cuando cambia el tema"""
        self.bg_color = bg_color
        self.fg_color = fg_color
        self.accent_color = accent_color
        
        self.sidebar_container.configure(bg=bg_color)
        self.left_panel.configure(bg=bg_color)
        self.content_frame.configure(bg=bg_color)
        self.canvas.configure(bg=bg_color)
        self.scrollable_frame.configure(bg=bg_color)
        
        # Actualizar colores de todos los labels recursivamente
        for widget in self.scrollable_frame.winfo_children():
            if isinstance(widget, tk.Label):
                if widget.cget("font") == ("Consolas", 8, "bold"):
                    # Es un comando
                    widget.configure(bg=bg_color, fg="#98c379" if bg_color == "#2b2b2b" else "#0078d4")
                elif widget.cget("font") == ("Arial", 9, "bold"):
                    # Es una sección
                    widget.configure(bg=bg_color, fg=accent_color)
                else:
                    # Es descripción
                    widget.configure(bg=bg_color, fg=fg_color)
            elif isinstance(widget, tk.Frame):
                widget.configure(bg=bg_color)
                for child in widget.winfo_children():
                    if isinstance(child, tk.Label):
                        child.configure(bg=bg_color)

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
        
        # Variables para detección de actividad serial
        self.last_data_time = time.time()
        self.no_data_counter = 0
        self.is_receiving_data = False
        
        self.current_theme = "Light"

        style = ttk.Style()
        style.theme_use('clam')

        theme = THEMES["Light"]
        self.bg_color = theme["bg"]
        self.fg_color = theme["fg"]
        self.accent_color = theme["accent"]
        self.output_bg = theme["output_bg"]
        self.input_bg = theme["input_bg"]
        self.plot_bg = theme["plot_bg"]

        style.configure("Custom.TCombobox", 
                   fieldbackground=theme["combo_bg"],
                   background=theme["combo_bg"],
                   foreground=theme["combo_fg"])
        style.configure("CustomBaud.TCombobox", 
                   fieldbackground=theme["combo_bg"],
                   background=theme["combo_bg"],
                   foreground=theme["combo_fg"])

        self.sidebar = CollapsibleSidebar(root, self.bg_color, self.fg_color, self.accent_color)
        self.sidebar.pack(side=tk.LEFT, fill=tk.Y)
        
        self.main_container = tk.Frame(root, bg=self.bg_color)
        self.main_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.create_widgets()
        self.setup_matplotlib_figure()
        self.refresh_ports()
        self.setup_keyboard_events()
        self.crear_menus_contextuales()
        self.setup_styles()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.plot_update_timer = None
        self.start_plot_updates()
        
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        self.apply_theme(self.current_theme)
        
    def apply_theme(self, theme_name):
        """Aplica el tema seleccionado a toda la interfaz"""
        theme = THEMES[theme_name]
        
        # Actualizar variables de color
        self.bg_color = theme["bg"]
        self.fg_color = theme["fg"]
        self.accent_color = theme["accent"]
        self.output_bg = theme["output_bg"]
        self.input_bg = theme["input_bg"]
        self.plot_bg = theme["plot_bg"]
        
        # 1. Actualizar ventana principal
        self.root.configure(bg=self.bg_color)
        
        # 2. Actualizar PanedWindow usando style
        style = ttk.Style()
        style.configure("TPanedwindow", background=self.bg_color)
        self.main_paned.configure(style="TPanedwindow")

        if hasattr(self, 'sidebar'):
            self.sidebar.update_theme(self.bg_color, self.fg_color, self.accent_color)
        
        # 3. Actualizar frames (son tk.Frame, no ttk)
        for frame_name in ['top_frame', 'bottom_frame', 'plot_frame']:
            if hasattr(self, frame_name):
                frame = getattr(self, frame_name)
                if frame:
                    frame.configure(bg=self.bg_color)
        
        # 4. Actualizar frames de controles
        for name in ['controls_frame', 'left_controls', 'right_controls', 'cmd_frame', 'plot_controls_frame']:
            if hasattr(self, name):
                widget = getattr(self, name)
                if widget and hasattr(widget, 'configure'):
                    widget.configure(bg=self.bg_color)
        
        # 5. Configurar estilos ttk
        style.configure(".", background=self.bg_color, foreground=self.fg_color)
        style.configure("TFrame", background=self.bg_color)
        style.configure("TLabel", background=self.bg_color, foreground=self.fg_color)
        
        # Estilos específicos para Combobox
        style.configure("Custom.TCombobox", 
                    fieldbackground=theme["combo_bg"],
                    background=theme["combo_bg"],
                    foreground=theme["combo_fg"],
                    selectbackground=self.accent_color,
                    selectforeground="white")
        
        style.map("Custom.TCombobox",
                fieldbackground=[("readonly", theme["combo_bg"])],
                background=[("readonly", theme["combo_bg"])],
                foreground=[("readonly", theme["combo_fg"])])
        
        style.configure("CustomBaud.TCombobox", 
                    fieldbackground=theme["combo_bg"],
                    background=theme["combo_bg"],
                    foreground=theme["combo_fg"],
                    selectbackground=self.accent_color,
                    selectforeground="white")
        
        style.map("CustomBaud.TCombobox",
                fieldbackground=[("readonly", theme["combo_bg"])],
                background=[("readonly", theme["combo_bg"])],
                foreground=[("readonly", theme["combo_fg"])])
        
        # 6. Actualizar widgets tkinter
        self._update_widget_colors()
        
        # 7. Actualizar área de texto
        if hasattr(self, 'output_area'):
            self.output_area.configure(bg=self.output_bg, fg=self.fg_color)
            timestamp_color = "#6a9955" if theme_name == "Dark" else "#2d6a4f"
            error_color = "#f48771" if theme_name == "Dark" else "#d32f2f"
            self.output_area.tag_config("timestamp", foreground=timestamp_color)
            self.output_area.tag_config("error", foreground=error_color)
            self.output_area.tag_config("ramping", foreground="#ff6b6b")
            self.output_area.tag_config("approaching", foreground="#ffa500")
            self.output_area.tag_config("holding", foreground="#ffd700")
        
        # 8. Actualizar Combobox
        if hasattr(self, 'port_combo') and self.port_combo:
            self.port_combo.configure(style="Custom.TCombobox")
        
        if hasattr(self, 'baud_combo') and self.baud_combo:
            self.baud_combo.configure(style="CustomBaud.TCombobox")
        
        # 9. Actualizar gráfica
        if self.figure and self.ax:
            self._update_plot_theme(theme)
            if self.plot_data:
                self._draw_plot(self.plot_data)
            else:
                self.clear_plot()

    def _recursive_update_bg(self, widget, color):
        """Actualiza recursivamente el fondo de todos los widgets hijos"""
        try:
            if hasattr(widget, 'configure'):
                # Verificar si el widget acepta la opción 'bg'
                if 'bg' in widget.keys():
                    widget.configure(bg=color)
            # Recursivamente actualizar hijos
            for child in widget.winfo_children():
                self._recursive_update_bg(child, color)
        except:
            pass

    def _update_widget_colors(self):
        """Actualiza los colores de los widgets existentes"""
        # Botones
        for widget in [self.connect_btn, self.refresh_btn, self.clear_btn, 
                    self.send_btn, self.open_log_btn, self.clear_plot_btn, self.theme_btn]:
            if widget and hasattr(widget, 'configure'):
                if widget == self.connect_btn:
                    widget.configure(bg=self.accent_color if not self.is_connected else "#d32f2f", 
                                fg="white")
                elif widget == self.theme_btn:
                    widget.configure(bg=self.bg_color, fg=self.fg_color)
                elif widget in [self.send_btn, self.open_log_btn, self.clear_btn, self.clear_plot_btn]:
                    widget.configure(bg=self.accent_color, fg="white")
                else:  # refresh_btn
                    widget.configure(bg=self.bg_color, fg=self.fg_color)
        
        # Checkbuttons
        for cb in [self.autoscroll_cb, self.timestamp_cb, self.save_log_cb, self.realtime_plot_cb]:
            if cb and hasattr(cb, 'configure'):
                cb.configure(bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color)
        
        # Labels
        for label in [self.lbl_port, self.lbl_baud, self.status_text, self.status_label, self.lbl_sep]:
            if label and hasattr(label, 'configure'):
                label.configure(bg=self.bg_color)
                if label == self.lbl_port:
                    label.configure(fg=self.fg_color)
                elif label == self.lbl_baud:
                    label.configure(fg=self.fg_color)
                elif label == self.status_text:
                    label.configure(fg=self.fg_color)
                elif label == self.status_label:
                    label.configure(fg="#00aa00" if self.is_receiving_data else "#555555")
                elif label == self.lbl_sep:
                    label.configure(fg=self.fg_color)
        
        # Entry
        if hasattr(self, 'input_entry') and self.input_entry:
            self.input_entry.configure(bg=self.input_bg, fg=self.fg_color,
                                    insertbackground=self.fg_color,
                                    highlightcolor=self.accent_color)
    
    def _update_plot_theme(self, theme):
        """Actualiza los colores de la gráfica según el tema"""
        if not self.figure or not self.ax:
            return
        
        self.figure.patch.set_facecolor(theme["plot_bg"])
        self.ax.set_facecolor(theme["plot_face"])
        self.ax.tick_params(colors=theme["tick_color"])
        self.ax.xaxis.label.set_color(theme["tick_color"])
        self.ax.yaxis.label.set_color(theme["tick_color"])
        self.ax.title.set_color(theme["tick_color"])
        
        for spine in self.ax.spines.values():
            spine.set_color(theme["spine_color"])
        
        self.ax.grid(True, alpha=0.3, color=theme["grid_color"])
        
        # Actualizar leyenda si existe
        legend = self.ax.get_legend()
        if legend:
            legend.get_frame().set_facecolor(theme["plot_bg"])
            legend.get_frame().set_edgecolor(theme["spine_color"])
            for text in legend.get_texts():
                text.set_color(theme["tick_color"])
    
    def change_theme(self):
        """Cambia entre tema oscuro y claro"""
        self.current_theme = "Light" if self.current_theme == "Dark" else "Dark"
        self.apply_theme(self.current_theme)
        
        # Actualizar texto del botón
        if hasattr(self, 'theme_btn'):
            self.theme_btn.config(text=f"🌙       Dark" if self.current_theme == "Light" else "☀️ Light")
    
    def create_widgets(self):
        self.main_paned = ttk.PanedWindow(self.main_container, orient=tk.VERTICAL)
        self.main_paned.pack(fill=tk.BOTH, expand=True)
        
        self.top_frame = tk.Frame(self.main_paned, bg=self.bg_color)
        self.main_paned.add(self.top_frame, weight=1)
        
        self.bottom_frame = tk.Frame(self.main_paned, bg=self.bg_color)
        self.main_paned.add(self.bottom_frame, weight=2)

        # Guardar referencias a los frames
        self.top_frame = self.top_frame
        self.bottom_frame = self.bottom_frame
        
        # CONTROLES SUPERIORES - Ahora con botón de tema en la derecha
        self.controls_frame = tk.Frame(self.top_frame, bg=self.bg_color, padx=10, pady=8)
        self.controls_frame.pack(fill=tk.X, side=tk.TOP)
        
        # Frame izquierdo para controles principales
        self.left_controls = tk.Frame(self.controls_frame, bg=self.bg_color)
        self.left_controls.pack(side=tk.LEFT)
        
        # Grupo Port
        self.lbl_port = tk.Label(self.left_controls, text="Port:", bg=self.bg_color, fg=self.fg_color)
        self.lbl_port.pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.left_controls, textvariable=self.port_var, width=12, state="readonly", style="Custom.TCombobox")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        # Grupo Baud
        self.lbl_baud = tk.Label(self.left_controls, text="Baud:", bg=self.bg_color, fg=self.fg_color)
        self.lbl_baud.pack(side=tk.LEFT, padx=(0, 5))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(self.left_controls, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200", "230400", "460800"], width=8, state="readonly", style="Custom.TCombobox")
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # Botón Conectar
        self.connect_btn = tk.Button(self.left_controls, text="Connect", command=self.toggle_connection, bg=self.accent_color, fg="white", padx=12, pady=3, relief=tk.FLAT, cursor="hand2")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 8))
        
        # Botón Refrescar
        self.refresh_btn = tk.Button(self.left_controls, text="⟳", command=self.refresh_ports, bg=self.bg_color, fg=self.fg_color, padx=6, pady=2, relief=tk.FLAT, cursor="hand2", font=("Arial", 11))
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 20))
        
        # Separador visual
        self.lbl_sep = tk.Label(self.left_controls, text="|", bg=self.bg_color, fg=self.fg_color)
        self.lbl_sep.pack(side=tk.LEFT, padx=(0, 20))
        
        # Botón Clear Output
        self.clear_btn = tk.Button(self.left_controls, text="Clear Output", command=self.clear_output, bg=self.accent_color, fg=self.fg_color, padx=10, pady=3, relief=tk.FLAT, cursor="hand2")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # Checkboxes
        self.autoscroll_cb = tk.Checkbutton(self.left_controls, text="Autoscroll", variable=self.autoscroll, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.autoscroll_cb.pack(side=tk.LEFT, padx=(0, 12))
        
        self.timestamp_cb = tk.Checkbutton(self.left_controls, text="Timestamp", variable=self.show_timestamp, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.timestamp_cb.pack(side=tk.LEFT, padx=(0, 12))
        
        self.save_log_cb = tk.Checkbutton(self.left_controls, text="Save Log", variable=self.save_log, command=self.toggle_save_log, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.save_log_cb.pack(side=tk.LEFT)
        
        # ===== BOTÓN DE TEMA (esquina superior derecha) =====
        self.right_controls = tk.Frame(self.controls_frame, bg=self.bg_color)
        self.right_controls.pack(side=tk.RIGHT)
        
        self.theme_btn = tk.Button(self.right_controls, text="🌙       Dark", command=self.change_theme,
                                   bg=self.bg_color, fg=self.fg_color, padx=10, pady=3,
                                   relief=tk.FLAT, cursor="hand2", font=("Arial", 9))
        self.theme_btn.pack(side=tk.RIGHT)
        
        # ENTRADA DE COMANDOS
        self.cmd_frame = tk.Frame(self.top_frame, bg=self.bg_color, padx=10, pady=8)
        self.cmd_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.input_entry = tk.Entry(self.cmd_frame, bg=self.input_bg, fg=self.fg_color, font=("Consolas", 10), insertbackground=self.fg_color, relief=tk.FLAT, borderwidth=1, highlightthickness=1, highlightcolor=self.accent_color)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        self.send_btn = tk.Button(self.cmd_frame, text="Send", command=self.send_command, bg=self.accent_color, fg="white", padx=15, pady=3, relief=tk.FLAT, cursor="hand2")
        self.send_btn.pack(side=tk.RIGHT)
        
        self.input_entry.config(state='disabled')
        self.send_btn.config(state='disabled')
        
        # ÁREA DE SALIDA DE TEXTO
        self.output_area = scrolledtext.ScrolledText(self.top_frame, wrap=tk.WORD, bg=self.output_bg, fg=self.fg_color, font=("Consolas", 9), insertbackground=self.fg_color, relief=tk.FLAT, borderwidth=0, height=12)
        self.output_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=2)
        
        def bloquear_escritura(event):
            if event.state & 0x0004 or event.keysym in ("Up", "Down", "Left", "Right", "Prior", "Next", "Home", "End"):
                return None
            return "break"
        self.output_area.bind("<Key>", bloquear_escritura)
        self.output_area.tag_config("timestamp", foreground="#6a9955")
        self.output_area.tag_config("error", foreground="#f48771")
        
        # CONTROLES DE LA GRÁFICA
        self.plot_controls_frame = tk.Frame(self.bottom_frame, bg=self.bg_color, padx=10, pady=5)
        self.plot_controls_frame.pack(fill=tk.X, side=tk.TOP)
        
        self.open_log_btn = tk.Button(self.plot_controls_frame, text="📂 Open Log File", command=self.open_log_file, bg=self.accent_color, fg="white", padx=10, pady=4, relief=tk.FLAT, cursor="hand2")
        self.open_log_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.realtime_plot_cb = tk.Checkbutton(self.plot_controls_frame, text="Real-time Plot", variable=self.real_time_plot, command=self.toggle_real_time_plot, bg=self.bg_color, fg=self.fg_color, selectcolor=self.bg_color, relief=tk.FLAT, cursor="hand2")
        self.realtime_plot_cb.pack(side=tk.LEFT, padx=(0, 10))
        
        self.clear_plot_btn = tk.Button(self.plot_controls_frame, text="Clear Plot", command=self.clear_plot, bg=self.accent_color, fg=self.fg_color, padx=10, pady=4, relief=tk.FLAT, cursor="hand2")
        self.clear_plot_btn.pack(side=tk.LEFT)
        
        # Indicador de estado de recepción de datos
        self.status_label = tk.Label(self.plot_controls_frame, text="●", bg=self.bg_color, fg="#00aa00", font=("Arial", 12))
        self.status_label.pack(side=tk.RIGHT, padx=5)
        self.status_text = tk.Label(self.plot_controls_frame, text="Esperando datos...", bg=self.bg_color, fg=self.fg_color, font=("Arial", 8))
        self.status_text.pack(side=tk.RIGHT, padx=(0, 10))
        
        self.plot_frame = tk.Frame(self.bottom_frame, bg=self.plot_bg)
        self.plot_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 5))

        # Barra de estado
        self.status_bar = tk.Frame(self.main_container, bg=self.bg_color, height=28)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
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
        """Actualiza la gráfica solo si hay datos nuevos (no si no hay recepción)"""
        if not self.real_time_plot.get():
            return
        
        # Si no hay recepción de datos activa, no actualizar la gráfica
        if not self.is_receiving_data:
            return
            
        text_content = self.output_area.get(1.0, tk.END)
        datos = cargar_datos_desde_texto(text_content)
        if datos is not None and datos['num_muestras'] > 0:
            # Verificar si hay datos nuevos respecto a la gráfica actual
            if self.plot_data is None or datos['num_muestras'] != self.plot_data.get('num_muestras', 0):
                self.plot_data = datos
                self._draw_plot(datos)
    
    def _draw_plot(self, datos):
        if not self.figure or not self.ax:
            return
            
        self.ax.clear()
        
        # Aplicar tema actual a la gráfica
        theme = THEMES[self.current_theme]
        self.ax.set_facecolor(theme["plot_face"])
        self.ax.tick_params(colors=theme["tick_color"])
        self.ax.xaxis.label.set_color(theme["tick_color"])
        self.ax.yaxis.label.set_color(theme["tick_color"])
        self.ax.title.set_color(theme["tick_color"])
        for spine in self.ax.spines.values():
            spine.set_color(theme["spine_color"])
        self.ax.grid(True, alpha=0.3, color=theme["grid_color"])
        
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
            self.ax.fill_between(datos['elapsed_seconds'], setpoint_inicial - SETPOINT_TOL, setpoint_inicial + SETPOINT_TOL, alpha=0.1, color='green')
            self.ax.set_title(f'Setpoint {setpoint_inicial}°C - Gráfica de Temperaturas', fontweight='bold', pad=8)
        else:
            self.ax.set_title(f'Gráfica de Temperaturas - {datos["num_muestras"]} muestras', fontweight='bold', pad=8)
            
        self.ax.set_xlabel('Tiempo (m:s)')
        self.ax.set_ylabel('Temperatura (°C)')
        self.ax.legend(loc='upper left', facecolor=theme["plot_bg"], edgecolor=theme["spine_color"], labelcolor=theme["tick_color"])
        
        valid_temps = datos['temps'][mask_temp_valid]
        if len(valid_temps) > 0:
            all_temps = [valid_temps]
            if datos['tiene_test']:
                valid_tests = datos['tests'][[es_temperatura_valida(t) for t in datos['tests']]]
                if len(valid_tests) > 0:
                    all_temps.append(valid_tests)
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
        theme = THEMES[self.current_theme]
        self.ax.set_facecolor(theme["plot_face"])
        self.ax.tick_params(colors=theme["tick_color"])
        self.ax.xaxis.label.set_color(theme["tick_color"])
        self.ax.yaxis.label.set_color(theme["tick_color"])
        self.ax.title.set_color(theme["tick_color"])
        for spine in self.ax.spines.values():
            spine.set_color(theme["spine_color"])
        self.ax.grid(True, alpha=0.3, color=theme["grid_color"])
        self.ax.set_title('Gráfica de Temperatura', color=theme["tick_color"], fontweight='bold')
        self.ax.set_xlabel('Tiempo (m:s)')
        self.ax.set_ylabel('Temperatura (°C)')
        
        if self.tooltip_manager:
            self.tooltip_manager.disconnect()
            self.tooltip_manager = None
            
        self.cursor = Cursor(self.ax, useblit=False, color='gray', linewidth=0.5, linestyle=':')
        if self.canvas:
            self.canvas.draw()
        self.plot_data = None
    
    def open_log_file(self):
        file_path = filedialog.askopenfilename(title="Seleccionar archivo de log", filetypes=[("Text files", "*.txt"), ("Log files", "*.log"), ("All files", "*.*")])
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                if self.real_time_plot.get():
                    self.real_time_plot.set(False)
                
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
            if self.real_time_plot.get():
                self.update_plot_from_textarea()
            
            # Actualizar indicador de estado de recepción
            if self.is_connected:
                time_since_data = time.time() - self.last_data_time
                if time_since_data > 5:  # 5 segundos sin datos
                    self.is_receiving_data = False
                    self.status_label.config(fg="#aa0000")
                    self.status_text.config(text="⚠️ Sin datos")
                else:
                    self.is_receiving_data = True
                    self.status_label.config(fg="#00aa00")
                    self.status_text.config(text="📡 Recibiendo")
            else:
                self.is_receiving_data = False
                self.status_label.config(fg="#888888")
                self.status_text.config(text="Desconectado")
            
            self.plot_update_timer = self.root.after(1000, update)
        self.plot_update_timer = self.root.after(1000, update)
    
    def toggle_save_log(self):
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
        if not self.port_var.get():
            return
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
            try:
                self.log_file.write(text)
                self.log_file.flush()
            except Exception as e:
                self.append_output(f"Error writing to log: {str(e)}\n", is_error=True)
    
    def setup_keyboard_events(self):
        self.input_entry.bind('<Return>', lambda e: self.send_command())
        self.input_entry.bind('<Up>', self.navigate_history_up)
        self.input_entry.bind('<Down>', self.navigate_history_down)
    
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list and not self.port_var.get():
            self.port_combo.set(port_list[0])
    
    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        port, baud = self.port_var.get(), int(self.baud_var.get())
        if not port:
            self.append_output("Error: No port selected\n", is_error=True)
            return
        try:
            self.serial_connection = serial.Serial(port, baud, timeout=0.1)
            self.is_connected = True
            self.last_data_time = time.time()  # Resetear tiempo de última recepción
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
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.is_connected = False
        if self.log_file:
            self.log_file.write("\n" + "-" * 80 + "\n" + f"Log ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
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
        while self.is_connected and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting:
                    data = self.serial_connection.readline()
                    if data:
                        decoded_data = data.decode('utf-8', errors='ignore')
                        self.root.after(0, self.append_output, decoded_data)
                        self.last_data_time = time.time()  # Actualizar timestamp de última recepción
                time.sleep(0.01)
            except Exception as e:
                if self.is_connected:
                    self.root.after(0, self.append_output, f"Read error: {str(e)}\n", True)
                break
    
    def send_command(self):
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
        if is_error:
            self.output_area.insert(tk.END, text, "error")
        else:
            self.output_area.insert(tk.END, text)
        if self.save_log.get():
            self.write_to_log(timestamped_text + text)
        if self.autoscroll.get():
            self.output_area.see(tk.END)
        self.output_area.config(state='disabled')
    
    def clear_output(self):
        self.output_area.config(state='normal')
        self.output_area.delete(1.0, tk.END)
        self.output_area.config(state='disabled')
        if self.real_time_plot.get():
            self.clear_plot()
    
    def on_closing(self):
        if self.plot_update_timer:
            self.root.after_cancel(self.plot_update_timer)
        if self.tooltip_manager:
            self.tooltip_manager.disconnect()
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
            if self.input_entry.cget('state') == 'normal':
                self.menu_input.tk_popup(event.x_root, event.y_root)
            return "break"

        self.output_area.bind("<Button-3>", mostrar_menu_output)
        self.input_entry.bind("<Button-3>", mostrar_menu_input)


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialMonitorWithPlot(root)
    root.mainloop()