import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import os
import sys

class SerialMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Monitor")
        self.root.geometry("900x700")
        self.root.minsize(700, 500)
        
        # Variables
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
        
        # Configurar estilo
        self.setup_styles()
        
        # Crear interfaz
        self.create_widgets()
        
        # Actualizar puertos disponibles
        self.refresh_ports()
        
        # Configurar eventos de teclado
        self.setup_keyboard_events()
        
        # Configurar cierre de aplicación
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
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
        
        self.root.configure(bg=self.bg_color)
        
    def create_widgets(self):
        # Frame superior para controles
        top_frame = tk.Frame(self.root, bg=self.bg_color, padx=10, pady=10)
        top_frame.pack(fill=tk.X)
        
        # Puerto
        tk.Label(top_frame, text="Port:", bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # Baud rate
        tk.Label(top_frame, text="Baud Rate:", bg=self.bg_color, fg=self.fg_color).pack(side=tk.LEFT, padx=(0, 5))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(top_frame, textvariable=self.baud_var, 
                                       values=["9600", "19200", "38400", "57600", "115200", "230400", "460800"],
                                       width=10, state="readonly")
        self.baud_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # Botón Connect/Disconnect
        self.connect_btn = tk.Button(top_frame, text="Connect", command=self.toggle_connection,
                                     bg=self.accent_color, fg="white", padx=20, pady=5,
                                     relief=tk.FLAT, cursor="hand2")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # Botón Refresh Ports
        self.refresh_btn = tk.Button(top_frame, text="⟳", command=self.refresh_ports,
                                     bg=self.bg_color, fg=self.fg_color, padx=10, pady=5,
                                     relief=tk.FLAT, cursor="hand2", font=("Arial", 12))
        self.refresh_btn.pack(side=tk.LEFT)
        
        # Frame para controles de output
        output_controls_frame = tk.Frame(self.root, bg=self.bg_color, padx=10, pady=5)
        output_controls_frame.pack(fill=tk.X)
        
        # Botón Clear Output
        self.clear_btn = tk.Button(output_controls_frame, text="Clear Output", command=self.clear_output,
                                   bg=self.bg_color, fg=self.fg_color, padx=10, pady=3,
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
        self.output_area = scrolledtext.ScrolledText(self.root, wrap=tk.WORD, 
                                                      bg=self.output_bg, fg=self.fg_color,
                                                      font=("Consolas", 10), insertbackground=self.fg_color,
                                                      relief=tk.FLAT, borderwidth=0)
        self.output_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        
        # Configurar tags para colores
        self.output_area.tag_config("timestamp", foreground="#6a9955")
        self.output_area.tag_config("error", foreground="#f48771")
        
        # Frame inferior para entrada de comandos
        bottom_frame = tk.Frame(self.root, bg=self.bg_color, padx=10, pady=10)
        bottom_frame.pack(fill=tk.X)
        
        # Campo de entrada de comandos
        self.input_entry = tk.Entry(bottom_frame, bg=self.input_bg, fg=self.fg_color,
                                    font=("Consolas", 10), insertbackground=self.fg_color,
                                    relief=tk.FLAT, borderwidth=1, highlightthickness=1,
                                    highlightcolor=self.accent_color)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        # Botón Send
        self.send_btn = tk.Button(bottom_frame, text="Send", command=self.send_command,
                                  bg=self.accent_color, fg="white", padx=20, pady=5,
                                  relief=tk.FLAT, cursor="hand2")
        self.send_btn.pack(side=tk.RIGHT)
        
        # Estado inicial de los controles
        self.input_entry.config(state='disabled')
        self.send_btn.config(state='disabled')
        
    def toggle_save_log(self):
        """Habilitar/deshabilitar guardado de log"""
        if self.save_log.get():
            # Habilitar timestamp automáticamente
            self.show_timestamp.set(True)
            # Crear archivo de log si está conectado
            if self.is_connected and self.port_var.get():
                self.create_log_file()
        else:
            # Cerrar archivo de log si está abierto
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
        
        # Obtener el directorio donde está el script/ejecutable
        if getattr(sys, 'frozen', False):
            # Si está ejecutando como .exe (compilado con PyInstaller)
            current_dir = os.path.dirname(sys.executable)
        else:
            # Si está ejecutando como script .py
            current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Guardar el archivo en el mismo directorio
        self.log_file_path = os.path.join(current_dir, filename)
        
        try:
            self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
            self.append_output(f"Logging enabled. Saving to: {self.log_file_path}\n")
            # Escribir encabezado en el log
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
        # Enter para enviar comando
        self.input_entry.bind('<Return>', lambda e: self.send_command())
        
        # Up/Down para navegar historial
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
            
            # Actualizar UI
            self.connect_btn.config(text="Disconnect", bg="#d32f2f")
            self.port_combo.config(state='disabled')
            self.baud_combo.config(state='disabled')
            self.refresh_btn.config(state='disabled')
            self.input_entry.config(state='normal')
            self.send_btn.config(state='normal')
            
            self.append_output(f"Connected to {port} at {baud} baud\n")
            
            # Crear archivo de log si está habilitado
            if self.save_log.get():
                self.create_log_file()
            
            # Iniciar hilo de lectura
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
        except Exception as e:
            self.append_output(f"Connection error: {str(e)}\n", is_error=True)
            
    def disconnect(self):
        """Desconectar del puerto serial"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        self.is_connected = False
        
        # Cerrar archivo de log
        if self.log_file:
            self.log_file.write("\n" + "-" * 80 + "\n")
            self.log_file.write(f"Log ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.close()
            self.log_file = None
            if self.log_file_path:
                self.append_output(f"Log saved to: {self.log_file_path}\n")
                self.log_file_path = None
        
        # Actualizar UI
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
            
        # Agregar al historial
        if not self.command_history or self.command_history[-1] != command:
            self.command_history.append(command)
        self.history_index = len(self.command_history)
        
        # Mostrar comando en output con timestamp si está habilitado
        timestamp = datetime.now().strftime("%H:%M:%S:%f")[:-3] + " -> " if self.show_timestamp.get() else ""
        self.append_output(f"\n{timestamp}→ {command}\n")
        
        # Enviar comando
        try:
            self.serial_connection.write((command + "\r\n").encode())
        except Exception as e:
            self.append_output(f"Send error: {str(e)}\n", is_error=True)
            
        # Limpiar entrada
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
        
        # Agregar timestamp si está habilitado (para líneas que no son comandos)
        if self.show_timestamp.get() and not text.startswith("\n") and not text.startswith("→"):
            timestamp = datetime.now().strftime("%H:%M:%S:%f")[:-3] + " -> "
            timestamped_text = timestamp
            self.output_area.insert(tk.END, timestamp, "timestamp")
            
        # Insertar texto
        if is_error:
            self.output_area.insert(tk.END, text, "error")
        else:
            self.output_area.insert(tk.END, text)
            
        # Escribir en log (sin formato, solo texto plano)
        if self.save_log.get():
            log_text = timestamped_text + text
            self.write_to_log(log_text)
            
        # Autoscroll si está habilitado
        if self.autoscroll.get():
            self.output_area.see(tk.END)
            
    def clear_output(self):
        """Limpiar el área de salida"""
        self.output_area.delete(1.0, tk.END)
        
    def on_closing(self):
        """Manejar cierre de la aplicación"""
        self.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialMonitor(root)
    root.mainloop()