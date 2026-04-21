import tkinter as tk

root = tk.Tk()
root.geometry("400x300")

# 1. Contenedor principal para el Canvas y las Scrollbars
contenedor = tk.Frame(root)
contenedor.pack(fill="both", expand=True)

# 2. Crear el Canvas
lienzo = tk.Canvas(contenedor)
lienzo.pack(side="left", fill="both", expand=True)

# 3. Crear las barras de desplazamiento (vertical y horizontal)
scroll_v = tk.Scrollbar(contenedor, orient="vertical", command=lienzo.yview)
scroll_v.pack(side="right", fill="y")

scroll_h = tk.Scrollbar(contenedor, orient="horizontal", command=lienzo.xview)
scroll_h.pack(side="bottom", fill="x")

# Vincular el Canvas a las barras de desplazamiento
lienzo.configure(yscrollcommand=scroll_v.set, xscrollcommand=scroll_h.set)

# 4. Crear el Frame que contendrá tus widgets (se colocará dentro del Canvas)
marco_desplazable = tk.Frame(lienzo)

# 5. Colocar el Frame dentro de una ventana en el Canvas
lienzo.create_window((0, 0), window=marco_desplazable, anchor="nw")

# 6. Actualizar la región de desplazamiento cuando el tamaño del Frame cambie
def actualizar_scroll(evento):
    lienzo.configure(scrollregion=lienzo.bbox("all"))

marco_desplazable.bind("<Configure>", actualizar_scroll)

# ==========================================
# EJEMPLO DE USO: Añadiendo elementos grandes
# ==========================================
for i in range(20):
    for j in range(15):
        # Texto largo para forzar el scroll horizontal y vertical
        texto = f"Fila {i}, Col {j}"
        lbl = tk.Label(marco_desplazable, text=texto, padx=10, pady=10, borderwidth=1, relief="solid")
        lbl.grid(row=i, column=j)

root.mainloop()
