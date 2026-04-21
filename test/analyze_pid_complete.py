import re
import sys
import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.widgets import Cursor
import mpl_fontkit as fk

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

# Configuración de tamaño de figura y espaciado
FIG_SIZE = (14, 7)
LEFT_MARGIN = 0.08
RIGHT_MARGIN = 0.95
BOTTOM_MARGIN = 0.08
TOP_MARGIN = 0.95

# ============================================================
# ARCHIVO DE LOG
# ============================================================
LOG_FILE = sys.argv[1] if len(sys.argv) > 1 else "log_pid.txt"
print(f"⚠️  Usando log: {LOG_FILE}")
print(f"🔄 Recarga automática cada {REFRESH_INTERVAL} segundo(s)")

# ============================================================
# REGEX COMPILADAS
# ============================================================
PID_LINE = re.compile(
    r"(\d{2}:\d{2}:\d{2}[:.]\d{3})\s*->\s*State:\s*(\w+)\s*\|\s*Error:\s*(-?\d+\.?\d*)\s*\|\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)(?:\s*\|\s*Test:\s*(-?\d+\.?\d*))?(?:\s*\|\s*Diff:\s*(-?\d+\.?\d*))?\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)

# ============================================================
# CARGA DE DATOS
# ============================================================
def cargar_datos(archivo):
    """Carga y procesa los datos del archivo de log"""
    timestamps, temps, setpoints = [], [], []
    tests, diffs = [], []
    
    tiene_test = False
    tiene_diff = False
    
    try:
        with open(archivo, "r", encoding="utf-8") as f:
            for line in f:
                if m := PID_LINE.search(line):
                    time_str = m.group(1).replace('.', ':')
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
                        
    except FileNotFoundError:
        print(f"❌ Archivo no encontrado: {archivo}")
        return None
    except Exception as e:
        print(f"❌ Error leyendo archivo: {e}")
        return None
    
    if not temps:
        print("⚠️  No hay datos en el archivo aún")
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

# Cargar datos iniciales
datos = cargar_datos(LOG_FILE)
if datos is None:
    print("❌ No se pudieron cargar los datos iniciales")
    sys.exit(1)

# ============================================================
# FUNCIONES DE UTILIDAD
# ============================================================
def segundos_a_formato(x, pos=None):
    """Convierte segundos a formato MM:SS"""
    minutos = int(x) // 60
    segundos = int(x) % 60
    return f"{minutos}:{segundos:02d}"

def crear_grafica_temperatura(ax, datos):
    """Configura la gráfica de temperatura con leyenda fija"""
    ax.clear()
    
    # Graficar temperatura principal
    ax.plot(datos['elapsed_seconds'], datos['temps'], 'b-', linewidth=1.5, label='Temperatura Sensor')
    
    # Graficar temperatura test si existe
    if datos['tiene_test']:
        mask_test = ~np.isnan(datos['tests'])
        if np.any(mask_test):
            ax.plot(datos['elapsed_seconds'][mask_test], datos['tests'][mask_test], 
                   'm-', linewidth=1.5, label='Temperatura Test')
    
    # Setpoint
    setpoint_inicial = datos['setpoints'][-1]
    ax.axhline(y=setpoint_inicial, color='r', linestyle='--', linewidth=1.5, 
               label=f'Setpoint {setpoint_inicial}°C')
    ax.fill_between(datos['elapsed_seconds'], setpoint_inicial - 0.1, setpoint_inicial + 0.1, 
                    alpha=0.1, color='green')
    
    ax.set_title(f'Setpoint {setpoint_inicial}°C - Temperaturas', fontweight='bold', pad=8)
    ax.set_xlabel('Tiempo (m:s)', labelpad=4)
    ax.set_ylabel('Temperatura (°C)', labelpad=4)
    ax.grid(True, alpha=0.3)
    
    # Leyenda fija en esquina superior izquierda
    ax.legend(loc='upper left')
    
    # Ajustar límites
    if len(datos['temps']) > 0:
        all_temps = [datos['temps']]
        if datos['tiene_test']:
            valid_tests = datos['tests'][~np.isnan(datos['tests'])]
            if len(valid_tests) > 0:
                all_temps.append(valid_tests)
        combined = np.concatenate(all_temps) if all_temps else datos['temps']
        y_min = np.min(combined) - 0.2
        y_max = np.max(combined) + 0.2
        ax.set_ylim(y_min, y_max)
    
    if len(datos['elapsed_seconds']) > 0:
        x_min = np.min(datos['elapsed_seconds'])
        x_max = np.max(datos['elapsed_seconds'])
        x_range = x_max - x_min
        ax.set_xlim(x_min - 0.01*x_range, x_max + 0.01*x_range)
    
    ax.xaxis.set_major_locator(ticker.MultipleLocator(250))
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(segundos_a_formato))
    ax.tick_params(axis='both', which='major', labelsize=8, pad=2)

# ============================================================
# TOOLTIP INTERACTIVO
# ============================================================
class TooltipManager:
    """Gestiona los tooltips con posicionamiento mejorado"""
    
    def __init__(self, ax, datos_temp, x_data, fig, datos_tests=None):
        self.ax = ax
        self.datos_temp = datos_temp
        self.datos_tests = datos_tests
        self.x_data = x_data
        self.fig = fig
        self.tooltip_activo = False
        self.ultimo_tiempo = None
        
        # Crear anotación
        self.annot = self._crear_anotacion(ax, 'blue')
        
        # Punto destacado
        self.highlight, = ax.plot([], [], 'ro', markersize=8,
                                 markerfacecolor='red', markeredgecolor='white',
                                 markeredgewidth=1.5, alpha=1.0, visible=False)
        
        # Línea vertical
        self.vline = ax.axvline(x=0, color='red', linestyle=':', linewidth=1, alpha=0.7, visible=False)
        
        # Conectar eventos
        self.cid_move = fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.cid_click = fig.canvas.mpl_connect('button_press_event', self.on_click)
    
    def _crear_anotacion(self, ax, color):
        """Crea una anotación para la gráfica"""
        return ax.annotate("", xy=(0,0), xytext=(10,10), textcoords="offset points",
                          bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                                   edgecolor=color, linewidth=1.5, alpha=0.95),
                          arrowprops=dict(arrowstyle="->", color=color, lw=0.8, alpha=0.8),
                          family='monospace', weight='bold',
                          visible=False, annotation_clip=False,
                          zorder=10)
    
    def actualizar_datos(self, datos_temp, x_data, datos_tests=None):
        """Actualiza los datos de los tooltips"""
        self.datos_temp = datos_temp
        self.x_data = x_data
        self.datos_tests = datos_tests
    
    def on_move(self, event):
        if event.inaxes == self.ax and event.xdata is not None:
            self._actualizar_desde_tiempo(event.xdata)
    
    def _calcular_offset_inteligente(self, ax, tiempo, valor, es_ultimo_punto=False):
        """Calcula el offset óptimo para el tooltip considerando los bordes"""
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        x_rel = (tiempo - xlim[0]) / (xlim[1] - xlim[0])
        y_rel = (valor - ylim[0]) / (ylim[1] - ylim[0])
        
        offset_x = 10
        offset_y = 10
        
        if x_rel > 0.9 or es_ultimo_punto:
            offset_x = -80
        elif x_rel < 0.1:
            offset_x = 10
            
        if y_rel > 0.9:
            offset_y = -40
        elif y_rel < 0.1:
            offset_y = 10
            
        return (offset_x, offset_y)
    
    def _actualizar_desde_tiempo(self, tiempo):
        """Actualiza el tooltip basado en el tiempo"""
        self.tooltip_activo = True
        self.ultimo_tiempo = tiempo
        
        idx = np.argmin(np.abs(self.x_data - tiempo))
        tiempo_real = self.x_data[idx]
        
        es_ultimo_punto = idx == len(self.x_data) - 1
        
        valor_temp = self.datos_temp[idx] if idx < len(self.datos_temp) else None
        valor_test = self.datos_tests[idx] if self.datos_tests is not None and idx < len(self.datos_tests) else None
        
        self._actualizar_tooltip(tiempo_real, valor_temp, valor_test, es_ultimo_punto)
    
    def _actualizar_tooltip(self, tiempo, valor_temp, valor_test, es_ultimo_punto=False):
        """Actualiza la visualización del tooltip"""
        self.vline.set_xdata([tiempo])
        self.vline.set_visible(True)
        
        if valor_temp is not None:
            self.highlight.set_data([tiempo], [valor_temp])
            self.highlight.set_visible(True)
            
            offset = self._calcular_offset_inteligente(
                self.ax, tiempo, valor_temp, es_ultimo_punto
            )
            
            # Construir texto del tooltip
            tooltip_text = f'Time: {segundos_a_formato(tiempo)}\nTemp: {valor_temp:.2f}°C'
            if valor_test is not None and not np.isnan(valor_test):
                tooltip_text += f'\nTest: {valor_test:.2f}°C'
                diff = valor_temp - valor_test
                tooltip_text += f'\nDiff: {diff:+.2f}°C'
            
            self.annot.xy = (tiempo, valor_temp)
            self.annot.set_position(offset)
            self.annot.set_text(tooltip_text)
            self.annot.set_visible(True)
        
        self.fig.canvas.draw_idle()
    
    def on_click(self, event):
        """Oculta los tooltips al hacer clic"""
        if self.tooltip_activo:
            self.tooltip_activo = False
            self.highlight.set_visible(False)
            self.vline.set_visible(False)
            self.annot.set_visible(False)
            self.fig.canvas.draw_idle()
    
    def disconnect(self):
        """Desconecta los eventos"""
        self.fig.canvas.mpl_disconnect(self.cid_move)
        self.fig.canvas.mpl_disconnect(self.cid_click)

# ============================================================
# RECARGA AUTOMÁTICA DE DATOS
# ============================================================
class AutoRefresher:
    """Gestiona la recarga automática de datos"""
    
    def __init__(self, fig, ax, tooltip_manager, refresh_interval=1.0):
        self.fig = fig
        self.ax = ax
        self.tooltip_manager = tooltip_manager
        self.refresh_interval = refresh_interval
        self.ultimas_muestras = 0
        self.ultimo_tamanio = 0
        self.activo = True
        self.root = None
        
        try:
            self.root = fig.canvas.manager.window
        except:
            print("⚠️  No se pudo obtener la ventana Tkinter")
    
    def iniciar(self):
        """Inicia el timer de recarga automática"""
        self._programar_recarga()
    
    def detener(self):
        """Detiene la recarga automática"""
        self.activo = False
    
    def _programar_recarga(self):
        """Programa la próxima recarga"""
        if self.activo and self.root:
            try:
                self.root.after(int(self.refresh_interval * 1000), self._verificar_y_recargar)
            except:
                pass
    
    def _verificar_y_recargar(self):
        """Verifica si hay cambios y recarga si es necesario"""
        if not self.activo:
            return
        
        try:
            if os.path.exists(LOG_FILE):
                tamanio_actual = os.path.getsize(LOG_FILE)
                if tamanio_actual != self.ultimo_tamanio:
                    self._recargar_datos()
                    self.ultimo_tamanio = tamanio_actual
        except Exception as e:
            print(f"❌ Error verificando cambios: {e}")
        
        self._programar_recarga()
    
    def _recargar_datos(self):
        """Recarga los datos y actualiza las gráficas"""
        try:
            nuevos_datos = cargar_datos(LOG_FILE)
            if nuevos_datos is None:
                return
            
            if nuevos_datos['num_muestras'] > self.ultimas_muestras:
                self._actualizar_graficas(nuevos_datos)
                self.ultimas_muestras = nuevos_datos['num_muestras']
            
        except Exception as e:
            print(f"❌ Error recargando datos: {e}")
    
    def _actualizar_graficas(self, nuevos_datos):
        """Actualiza las gráficas con nuevos datos"""
        crear_grafica_temperatura(self.ax, nuevos_datos)
        
        Cursor(self.ax, useblit=True, color='gray', linewidth=0.5, linestyle=':')
        
        self.tooltip_manager.actualizar_datos(
            nuevos_datos['temps'],
            nuevos_datos['elapsed_seconds'],
            nuevos_datos['tests'] if nuevos_datos['tiene_test'] else None
        )
        
        base_filename = os.path.splitext(os.path.basename(LOG_FILE))[0]
        self.fig.canvas.manager.set_window_title(
            f"{base_filename} - {nuevos_datos['num_muestras']} muestras"
        )
        
        self.fig.canvas.draw_idle()

# ============================================================
# CONFIGURACIÓN DE LA FIGURA
# ============================================================
fig = plt.figure(figsize=FIG_SIZE)
ax = fig.add_subplot(111)

# Crear gráfica
crear_grafica_temperatura(ax, datos)

# Agregar cursor
Cursor(ax, useblit=True, color='gray', linewidth=0.5, linestyle=':')

# Crear tooltip manager
tooltip_manager = TooltipManager(
    ax,
    datos['temps'],
    datos['elapsed_seconds'],
    fig,
    datos['tests'] if datos['tiene_test'] else None
)

# Configurar título de la ventana
base_filename = os.path.splitext(os.path.basename(LOG_FILE))[0]
fig.canvas.manager.set_window_title(f"{base_filename}")

# Intentar configurar fuente
try:
    fk.set_font("Font Awesome 6 Free")
except:
    pass

# ============================================================
# INICIAR RECARGA AUTOMÁTICA
# ============================================================
refresher = AutoRefresher(fig, ax, tooltip_manager, REFRESH_INTERVAL)
refresher.ultimas_muestras = datos['num_muestras']
if os.path.exists(LOG_FILE):
    refresher.ultimo_tamanio = os.path.getsize(LOG_FILE)
refresher.iniciar()

# ============================================================
# MOSTRAR GRÁFICA
# ============================================================
plt.subplots_adjust(left=LEFT_MARGIN, right=RIGHT_MARGIN, bottom=BOTTOM_MARGIN, top=TOP_MARGIN)
plt.rcParams['font.family'] = 'sans-serif'

def on_close(event):
    """Manejador para cuando se cierra la ventana"""
    refresher.detener()
    tooltip_manager.disconnect()

fig.canvas.mpl_connect('close_event', on_close)
plt.show()

refresher.detener()
sys.exit()