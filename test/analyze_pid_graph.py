import re
import sys
import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.widgets import Cursor, Button
import mpl_fontkit as fk
import tkinter as tk

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

# Configuración de tamaño de figura y espaciado - REDUCIDO
FIG_SIZE = (16, 5.5)  # Reducido ligeramente
LEFT_MARGIN = 0.04     # Reducido
RIGHT_MARGIN = 0.98    # Aumentado (menos margen derecho)
BOTTOM_MARGIN = 0.08   # Reducido
TOP_MARGIN = 0.95      # Aumentado (menos margen superior)
WIDTH_PADDING = 0.15   # Espacio entre gráficas

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
    r"(\d{2}:\d{2}:\d{2}[:.]\d{3})\s*->\s*State:\s*(\w+)\s*\|\s*Error:\s*(-?\d+\.?\d*)\s*\|\s*Output:\s*(-?\d+\.?\d*)\s*\|\s*Temp:\s*(-?\d+\.?\d*)\s*\|\s*Setpoint:\s*(-?\d+\.?\d*)"
)
HEAT_LINE = re.compile(r"Heater set to\s*(\d+\.?\d*)%")
COOL_LINE = re.compile(r"Cooling set to\s*(\d+\.?\d*)%")

# ============================================================
# CARGA DE DATOS
# ============================================================
def cargar_datos(archivo):
    """Carga y procesa los datos del archivo de log"""
    timestamps, temps, setpoints, outputs = [], [], [], []
    
    try:
        with open(archivo, "r", encoding="utf-8") as f:
            for line in f:
                if m := PID_LINE.search(line):
                    time_str = m.group(1).replace('.', ':')
                    timestamps.append(datetime.strptime(time_str, "%H:%M:%S:%f"))
                    outputs.append(float(m.group(4)))
                    temps.append(float(m.group(5)))
                    setpoints.append(float(m.group(6)))
    except FileNotFoundError:
        print(f"❌ Archivo no encontrado: {archivo}")
        return None
    except Exception as e:
        print(f"❌ Error leyendo archivo: {e}")
        return None
    
    if not temps:
        print("⚠️  No hay datos en el archivo aún")
        return None
    
    # Calcular segundos desde el inicio
    start = timestamps[0]
    elapsed_seconds = np.array([(ts - start).total_seconds() for ts in timestamps])
    
    return {
        'elapsed_seconds': elapsed_seconds,
        'temps': np.array(temps),
        'setpoints': np.array(setpoints),
        'outputs': np.array(outputs),
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
    """Configura la gráfica de temperatura"""
    ax.clear()
    ax.plot(datos['elapsed_seconds'], datos['temps'], 'b-', linewidth=2, label='Temperatura')
    setpoint_inicial = datos['setpoints'][0]
    ax.axhline(y=setpoint_inicial, color='r', linestyle='--', linewidth=1.5, 
               label=f'Setpoint {setpoint_inicial}°C')
    ax.fill_between(datos['elapsed_seconds'], setpoint_inicial - 0.1, setpoint_inicial + 0.1, 
                    alpha=0.1, color='green')
    ax.set_title(f'Setpoint {setpoint_inicial}°C - Temperatura', fontweight='bold', pad=8)
    ax.set_xlabel('Tiempo (m:s)', labelpad=4)
    ax.set_ylabel('Temperatura (°C)', labelpad=4)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right')
    if len(datos['temps']) > 0:
        y_min = np.min(datos['temps']) - 0.2
        y_max = np.max(datos['temps']) + 0.2
        ax.set_ylim(y_min, y_max)
    if len(datos['elapsed_seconds']) > 0:
        x_min = np.min(datos['elapsed_seconds'])
        x_max = np.max(datos['elapsed_seconds'])
        # Añadir un pequeño margen del 1% en los bordes
        x_range = x_max - x_min
        ax.set_xlim(x_min - 0.01*x_range, x_max + 0.01*x_range)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(250))
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(segundos_a_formato))
    ax.tick_params(axis='both', which='major', labelsize=8, pad=2)

def crear_grafica_salida(ax, datos):
    """Configura la gráfica de salida del PID"""
    ax.clear()
    ax.plot(datos['elapsed_seconds'], datos['outputs'], 'g-', linewidth=2, label='Salida PID')
    ax.fill_between(datos['elapsed_seconds'], 0, datos['outputs'], alpha=0.3, color='green')
    ax.set_title(f'Setpoint {datos["setpoints"][0]}°C - Salida del Controlador', fontweight='bold', pad=8)
    ax.set_xlabel('Tiempo (m:s)', labelpad=4)
    ax.set_ylabel('Potencia (%)', labelpad=4)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right')
    if len(datos['outputs']) > 0:
        y_min = np.min(datos['outputs']) - 1
        y_max = np.max(datos['outputs']) + 1
        ax.set_ylim(y_min, y_max)
    if len(datos['elapsed_seconds']) > 0:
        x_min = np.min(datos['elapsed_seconds'])
        x_max = np.max(datos['elapsed_seconds'])
        # Añadir un pequeño margen del 1% en los bordes
        x_range = x_max - x_min
        ax.set_xlim(x_min - 0.01*x_range, x_max + 0.01*x_range)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(250))
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(segundos_a_formato))
    ax.tick_params(axis='both', which='major', labelsize=8, pad=2)

# ============================================================
# TOOLTIP INTERACTIVO SINCRONIZADO MEJORADO
# ============================================================
class TooltipManagerSincronizado:
    """Gestiona los tooltips sincronizados con posicionamiento mejorado"""
    
    def __init__(self, ax_temp, ax_salida, datos_temp, datos_salida, x_data, fig):
        self.ax_temp = ax_temp
        self.ax_salida = ax_salida
        self.datos_temp = datos_temp
        self.datos_salida = datos_salida
        self.x_data = x_data
        self.fig = fig
        self.tooltip_activo = False
        self.ultimo_tiempo = None
        
        # Crear anotaciones para ambas gráficas
        self.annot_temp = self._crear_anotacion(ax_temp, 'blue')
        self.annot_salida = self._crear_anotacion(ax_salida, 'green')
        
        # Puntos destacados
        self.highlight_temp, = ax_temp.plot([], [], 'ro', markersize=8,
                                           markerfacecolor='red', markeredgecolor='white',
                                           markeredgewidth=1.5, alpha=1.0, visible=False)
        self.highlight_salida, = ax_salida.plot([], [], 'ro', markersize=8,
                                              markerfacecolor='red', markeredgecolor='white',
                                              markeredgewidth=1.5, alpha=1.0, visible=False)
        
        # Línea vertical sincronizada
        self.vline_temp = ax_temp.axvline(x=0, color='red', linestyle=':', linewidth=1, alpha=0.7, visible=False)
        self.vline_salida = ax_salida.axvline(x=0, color='red', linestyle=':', linewidth=1, alpha=0.7, visible=False)
        
        # Conectar eventos para ambas gráficas
        self.cid_move_temp = fig.canvas.mpl_connect('motion_notify_event', self.on_move_temp)
        self.cid_move_salida = fig.canvas.mpl_connect('motion_notify_event', self.on_move_salida)
        self.cid_click = fig.canvas.mpl_connect('button_press_event', self.on_click)
    
    def _crear_anotacion(self, ax, color):
        """Crea una anotación para una gráfica"""
        return ax.annotate("", xy=(0,0), xytext=(10,10), textcoords="offset points",
                          bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                                   edgecolor=color, linewidth=1.5, alpha=0.95),
                          arrowprops=dict(arrowstyle="->", color=color, lw=0.8, alpha=0.8),
                          family='monospace', weight='bold',
                          visible=False, annotation_clip=False,
                          zorder=10)
    
    def actualizar_datos(self, datos_temp, datos_salida, x_data):
        """Actualiza los datos de los tooltips"""
        self.datos_temp = datos_temp
        self.datos_salida = datos_salida
        self.x_data = x_data
    
    def on_move_temp(self, event):
        if event.inaxes == self.ax_temp and event.xdata is not None:
            self._actualizar_desde_tiempo(event.xdata, 'temp')
    
    def on_move_salida(self, event):
        if event.inaxes == self.ax_salida and event.xdata is not None:
            self._actualizar_desde_tiempo(event.xdata, 'salida')
    
    def _calcular_offset_inteligente(self, ax, tiempo, valor, es_ultimo_punto=False):
        """Calcula el offset óptimo para el tooltip considerando los bordes"""
        # Obtener los límites del eje en coordenadas de datos
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        # Convertir a coordenadas de pantalla
        bbox = ax.get_window_extent()
        
        # Determinar la posición relativa del punto
        x_rel = (tiempo - xlim[0]) / (xlim[1] - xlim[0])
        y_rel = (valor - ylim[0]) / (ylim[1] - ylim[0])
        
        # Offset base
        offset_x = 10
        offset_y = 10
        
        # Ajustar según la posición
        if x_rel > 0.9 or es_ultimo_punto:  # Punto cerca del borde derecho
            offset_x = -80  # Mostrar a la izquierda
        elif x_rel < 0.1:  # Punto cerca del borde izquierdo
            offset_x = 10   # Mostrar a la derecha
            
        if y_rel > 0.9:  # Punto cerca del borde superior
            offset_y = -40  # Mostrar abajo
        elif y_rel < 0.1:  # Punto cerca del borde inferior
            offset_y = 10   # Mostrar arriba
            
        # Verificaciones adicionales para asegurar que no se salga
        if offset_x > 0 and tiempo + offset_x/100 * (xlim[1] - xlim[0]) > xlim[1]:
            offset_x = -80
        if offset_x < 0 and tiempo + offset_x/100 * (xlim[1] - xlim[0]) < xlim[0]:
            offset_x = 10
            
        return (offset_x, offset_y)
    
    def _actualizar_desde_tiempo(self, tiempo, origen):
        """Actualiza ambos tooltips basado en el tiempo"""
        self.tooltip_activo = True
        self.ultimo_tiempo = tiempo
        
        # Encontrar el índice más cercano en el tiempo
        idx = np.argmin(np.abs(self.x_data - tiempo))
        tiempo_real = self.x_data[idx]
        
        # Verificar si es el último punto
        es_ultimo_punto = idx == len(self.x_data) - 1
        
        # Obtener valores para ambas gráficas
        valor_temp = self.datos_temp[idx] if idx < len(self.datos_temp) else None
        valor_salida = self.datos_salida[idx] if idx < len(self.datos_salida) else None
        
        # Actualizar visualización
        self._actualizar_tooltips(tiempo_real, valor_temp, valor_salida, es_ultimo_punto)
    
    def _actualizar_tooltips(self, tiempo, valor_temp, valor_salida, es_ultimo_punto=False):
        """Actualiza la visualización de ambos tooltips"""
        # Actualizar líneas verticales
        self.vline_temp.set_xdata([tiempo])
        self.vline_salida.set_xdata([tiempo])
        self.vline_temp.set_visible(True)
        self.vline_salida.set_visible(True)
        
        # Actualizar puntos destacados
        if valor_temp is not None:
            self.highlight_temp.set_data([tiempo], [valor_temp])
            self.highlight_temp.set_visible(True)
            
            # Calcular offset para temperatura
            offset_temp = self._calcular_offset_inteligente(
                self.ax_temp, tiempo, valor_temp, es_ultimo_punto
            )
            
            # Actualizar anotación de temperatura
            self.annot_temp.xy = (tiempo, valor_temp)
            self.annot_temp.set_position(offset_temp)
            self.annot_temp.set_text(f'Time: {segundos_a_formato(tiempo)}\nTemp: {valor_temp:.2f}°C')
            self.annot_temp.set_visible(True)
        
        if valor_salida is not None:
            self.highlight_salida.set_data([tiempo], [valor_salida])
            self.highlight_salida.set_visible(True)
            
            # Calcular offset para salida
            offset_salida = self._calcular_offset_inteligente(
                self.ax_salida, tiempo, valor_salida, es_ultimo_punto
            )
            
            # Actualizar anotación de salida
            self.annot_salida.xy = (tiempo, valor_salida)
            self.annot_salida.set_position(offset_salida)
            self.annot_salida.set_text(f'Time: {segundos_a_formato(tiempo)}\nOutput: {valor_salida:.1f}%')
            self.annot_salida.set_visible(True)
        
        self.fig.canvas.draw_idle()
    
    def on_click(self, event):
        """Oculta los tooltips al hacer clic"""
        if self.tooltip_activo:
            self.tooltip_activo = False
            self.highlight_temp.set_visible(False)
            self.highlight_salida.set_visible(False)
            self.vline_temp.set_visible(False)
            self.vline_salida.set_visible(False)
            self.annot_temp.set_visible(False)
            self.annot_salida.set_visible(False)
            self.fig.canvas.draw_idle()
    
    def disconnect(self):
        """Desconecta los eventos"""
        self.fig.canvas.mpl_disconnect(self.cid_move_temp)
        self.fig.canvas.mpl_disconnect(self.cid_move_salida)
        self.fig.canvas.mpl_disconnect(self.cid_click)

# ============================================================
# RECARGA AUTOMÁTICA DE DATOS
# ============================================================
class AutoRefresher:
    """Gestiona la recarga automática de datos"""
    
    def __init__(self, fig, ax1, ax2, tooltip_manager, refresh_interval=1.0):
        self.fig = fig
        self.ax1 = ax1
        self.ax2 = ax2
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
                muestras_nuevas = nuevos_datos['num_muestras'] - self.ultimas_muestras
                print(f"📊 Nuevos datos: {nuevos_datos['num_muestras']} muestras (+{muestras_nuevas})")
                
                self._actualizar_graficas(nuevos_datos)
                self.ultimas_muestras = nuevos_datos['num_muestras']
            
        except Exception as e:
            print(f"❌ Error recargando datos: {e}")
    
    def _actualizar_graficas(self, nuevos_datos):
        """Actualiza las gráficas con nuevos datos"""
        crear_grafica_temperatura(self.ax1, nuevos_datos)
        crear_grafica_salida(self.ax2, nuevos_datos)
        
        Cursor(self.ax1, useblit=True, color='gray', linewidth=0.5, linestyle=':')
        Cursor(self.ax2, useblit=True, color='gray', linewidth=0.5, linestyle=':')
        
        self.tooltip_manager.actualizar_datos(
            nuevos_datos['temps'],
            nuevos_datos['outputs'],
            nuevos_datos['elapsed_seconds']
        )
        
        base_filename = os.path.splitext(os.path.basename(LOG_FILE))[0]
        self.fig.canvas.manager.set_window_title(
            f"{base_filename} - {nuevos_datos['num_muestras']} muestras"
        )
        
        self.fig.canvas.draw_idle()

# ============================================================
# CONFIGURACIÓN DE LA FIGURA
# ============================================================
# Crear figura con tamaño ajustado
fig = plt.figure(figsize=FIG_SIZE)

# Crear subplots con más espacio entre ellos
gs = fig.add_gridspec(1, 2, hspace=0, wspace=WIDTH_PADDING)
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1])

# Crear gráficas iniciales
crear_grafica_temperatura(ax1, datos)
crear_grafica_salida(ax2, datos)

# Añadir cursores
Cursor(ax1, useblit=True, color='gray', linewidth=0.5, linestyle=':')
Cursor(ax2, useblit=True, color='gray', linewidth=0.5, linestyle=':')

# Crear tooltip sincronizado mejorado
tooltip_manager = TooltipManagerSincronizado(
    ax1, ax2,
    datos['temps'],
    datos['outputs'],
    datos['elapsed_seconds'],
    fig
)

# Configurar título de ventana
base_filename = os.path.splitext(os.path.basename(LOG_FILE))[0]
# fig.canvas.manager.set_window_title(f"{base_filename} - {datos['num_muestras']} muestras")
fig.canvas.manager.set_window_title(f"{base_filename}")

# ============================================================
# BOTÓN DE RECARGA MANUAL
# ============================================================
def recarga_manual(event=None):
    """Función para recarga manual con el botón"""
    try:
        nuevos_datos = cargar_datos(LOG_FILE)
        if nuevos_datos:
            crear_grafica_temperatura(ax1, nuevos_datos)
            crear_grafica_salida(ax2, nuevos_datos)
            tooltip_manager.actualizar_datos(
                nuevos_datos['temps'],
                nuevos_datos['outputs'],
                nuevos_datos['elapsed_seconds']
            )
            fig.canvas.draw_idle()
            print(f"✅ Recarga manual completada: {nuevos_datos['num_muestras']} muestras")
    except Exception as e:
        print(f"❌ Error en recarga manual: {e}")

# Intentar cargar Font Awesome, continuar si falla
try:
    fk.set_font("Font Awesome 6 Free")
except:
    pass

# Botón más pequeño y mejor posicionado
# btn_ax = fig.add_axes([0.002, 0.97, 0.025, 0.025])
# btn = Button(btn_ax, '↻', color='lightgray', hovercolor='gray')
# btn.label.set_fontsize(8)
# btn.on_clicked(recarga_manual)

# ============================================================
# INICIAR RECARGA AUTOMÁTICA
# ============================================================
refresher = AutoRefresher(fig, ax1, ax2, tooltip_manager, REFRESH_INTERVAL)
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