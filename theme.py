"""
theme.py - Identidad visual MOLTECH / Molgroup para el HMI Spirax.

Tokens de color tomados del manual de marca Molgroup (paleta Pantone),
tipografia con fallback (Aeonik -> Segoe UI -> Arial) y helpers para
aplicar el tema a widgets tk/ttk de forma consistente.

Uso:
    import theme as T
    T.init(root)              # una sola vez, antes de construir la UI
    T.aplicar_estilos_ttk()   # estilos de Notebook / Scrollbar
    T.crear_header(root, ...) # header de marca
    T.pulir(root)             # pasada final: aplana botones, bordes, etc.
"""

import tkinter as tk
from tkinter import ttk, font as tkfont

# ======================================================================
#  Paleta - Manual de marca Molgroup
# ======================================================================

# Azules de marca
NAVY_900 = '#001638'   # fondo profundo (header / banners / consolas)
NAVY_800 = '#002459'   # PANTONE 2935 C - superficie principal
NAVY_700 = '#003384'   # PANTONE 2383 C - paneles / tabs inactivas
AZUL     = '#004BFF'   # PANTONE 3005 C - accion primaria / seleccion
AZUL_HOV = '#0042C6'   # PANTONE 300 C  - hover de accion primaria
CELESTE  = '#008AFF'   # PANTONE 2925 C - informativo / presencia
CELESTE2 = '#64BDFF'   # PANTONE 2905 C - acentos / eyebrows
CELESTE3 = '#B3E2FF'   # PANTONE 7457 C - texto secundario

# Neutros de marca (adaptados sobre navy)
FG       = '#F3F3F3'   # texto principal (COOL GRAY 1 C)
FG_SOFT  = '#C7D6EF'   # texto suave
FG_MUTED = '#8FA9D0'   # texto atenuado
FG_DIM   = '#54719E'   # deshabilitado

# Derivados de superficie
BG       = NAVY_800
BG_DEEP  = NAVY_900
PANEL    = NAVY_700
PANEL_DIM = '#01265C'  # trough deshabilitado / celdas vacias
BORDE    = '#0A3A7E'

# Botones neutros
BTN_BG    = '#0A3578'
BTN_HOVER = '#10439A'

# Semanticos (estados de maquina: se mantienen universales)
OK        = '#13A35B'
OK_TXT    = '#5CE49B'
ERR       = '#E0453F'
ERR_TXT   = '#FF7B72'
WARN      = '#FFC75A'
WARN_TXT  = '#FFC75A'
WARN_TXT2 = '#FFAB5E'
ABAJO     = '#E08A00'
AMARILLO  = '#FFE27A'
NEUTRO    = '#5E76A1'
INFO_TXT  = CELESTE2

# Matriz de mesa
CELDA_VACIA        = PANEL_DIM
CELDA_VACIA_BORDE  = '#08316E'
CELDA_OCUPADA      = CELESTE
CELDA_OCUPADA_BORDE = '#3AA4FF'
CELDA_ELEGIDA      = WARN

# ======================================================================
#  Tipografia
# ======================================================================

FAMILIA = 'Arial'        # se resuelve en init()
FAMILIA_COND = 'Arial'   # condensada para wordmarks / eyebrows

F_H1 = F_H2 = F_H3 = F_BOLD_LG = F_LG = F_BOLD = F_BASE = None
F_SM_BOLD = F_SM = F_XS = F_XS_IT = F_TAG = F_EYEBROW = F_WORDMARK = None
F_MONO = None


def _primera_disponible(candidatas, familias):
    bajas = {f.lower(): f for f in familias}
    for c in candidatas:
        if c.lower() in bajas:
            return bajas[c.lower()]
    return 'Arial'


def init(root):
    """Resuelve familias tipograficas y arma la escala. Llamar 1 vez."""
    global FAMILIA, FAMILIA_COND
    global F_H1, F_H2, F_H3, F_BOLD_LG, F_LG, F_BOLD, F_BASE
    global F_SM_BOLD, F_SM, F_XS, F_XS_IT, F_TAG, F_EYEBROW, F_WORDMARK
    global F_MONO

    familias = tkfont.families(root)
    # Aeonik es la tipografia de marca; Segoe UI es el fallback natural
    # en los equipos Windows de planta.
    FAMILIA = _primera_disponible(
        ['Aeonik', 'Segoe UI', 'Helvetica Neue', 'Arial'], familias)
    FAMILIA_COND = _primera_disponible(
        ['Roboto Condensed', 'Bahnschrift Condensed', 'Bahnschrift',
         'Arial Narrow', FAMILIA], familias)

    F_H1      = (FAMILIA, 18, 'bold')
    F_H2      = (FAMILIA, 16, 'bold')
    F_H3      = (FAMILIA, 13, 'bold')
    F_BOLD_LG = (FAMILIA, 12, 'bold')
    F_LG      = (FAMILIA, 12)
    F_BOLD    = (FAMILIA, 11, 'bold')
    F_BASE    = (FAMILIA, 11)
    F_SM_BOLD = (FAMILIA, 10, 'bold')
    F_SM      = (FAMILIA, 10)
    F_XS      = (FAMILIA, 9)
    F_XS_IT   = (FAMILIA, 9, 'italic')
    F_TAG     = (FAMILIA, 8, 'bold')
    F_EYEBROW = (FAMILIA_COND, 10, 'bold')
    F_WORDMARK = (FAMILIA_COND, 20, 'bold')
    F_MONO    = ('Consolas', 10)


# ======================================================================
#  Estilos ttk
# ======================================================================

def aplicar_estilos_ttk():
    style = ttk.Style()
    try:
        style.theme_use('clam')
    except Exception:
        pass

    style.configure('TNotebook', background=BG, borderwidth=0,
                    tabmargins=[12, 8, 12, 0])
    style.configure('TNotebook.Tab',
                    background=PANEL, foreground=CELESTE3,
                    padding=[26, 10], borderwidth=0,
                    font=(FAMILIA_COND, 11, 'bold'))
    style.map('TNotebook.Tab',
              background=[('selected', AZUL), ('active', AZUL_HOV)],
              foreground=[('selected', '#FFFFFF'), ('active', '#FFFFFF')])

    style.configure('Vertical.TScrollbar',
                    background=PANEL, troughcolor=BG_DEEP,
                    bordercolor=BG, arrowcolor=CELESTE3,
                    relief='flat')
    style.map('Vertical.TScrollbar',
              background=[('active', AZUL)])
    style.configure('Horizontal.TScrollbar',
                    background=PANEL, troughcolor=BG_DEEP,
                    bordercolor=BG, arrowcolor=CELESTE3,
                    relief='flat')


# ======================================================================
#  Header de marca
# ======================================================================

def _dibujar_iso(canvas, x, y, lado, color=AZUL, hueco=None):
    """Dibuja el iso Molgroup: cuadrado con esquina recortada y hueco
    hexagonal interior."""
    if hueco is None:
        hueco = canvas['bg']
    r = lado * 0.32          # recorte de esquina
    # Cuadrado con esquina superior-derecha recortada
    pts = [x, y,
           x + lado - r, y,
           x + lado, y + r,
           x + lado, y + lado,
           x, y + lado]
    canvas.create_polygon(pts, fill=color, outline=color)
    # Hueco interior (hexagono achatado, como el iso del manual)
    cx, cy = x + lado * 0.5, y + lado * 0.52
    h = lado * 0.20
    c = h * 0.55
    pts_h = [cx - h + c, cy - h,
             cx + h, cy - h,
             cx + h, cy + h - c,
             cx + h - c, cy + h,
             cx - h, cy + h,
             cx - h, cy - h + c]
    canvas.create_polygon(pts_h, fill=hueco, outline=hueco)


def crear_header(root, titulo='SPIRAX', subtitulo='Celda de carga robotizada',
                 empresa='MOLTECH'):
    """Header de marca: iso + wordmark MOLTECH | SPIRAX + tag Molgroup."""
    alto = 64
    head = tk.Frame(root, bg=BG_DEEP, height=alto)
    head.pack(fill='x', side='top')
    head.pack_propagate(False)

    # filete azul inferior
    tk.Frame(root, bg=AZUL, height=3).pack(fill='x', side='top')

    cv = tk.Canvas(head, bg=BG_DEEP, width=46, height=alto,
                   highlightthickness=0, bd=0)
    cv.pack(side='left', padx=(18, 12))
    _dibujar_iso(cv, 6, (alto - 34) // 2, 34)

    f_txt = tk.Frame(head, bg=BG_DEEP)
    f_txt.pack(side='left', fill='y')

    fila = tk.Frame(f_txt, bg=BG_DEEP)
    fila.pack(anchor='w', pady=(10, 0))
    tk.Label(fila, text=empresa, font=F_WORDMARK,
             bg=BG_DEEP, fg='#FFFFFF').pack(side='left')
    tk.Label(fila, text='|', font=(FAMILIA, 16),
             bg=BG_DEEP, fg=FG_DIM).pack(side='left', padx=10)
    tk.Label(fila, text=titulo, font=(FAMILIA_COND, 20),
             bg=BG_DEEP, fg=CELESTE2).pack(side='left')

    tk.Label(f_txt, text=subtitulo.upper(),
             font=(FAMILIA_COND, 9, 'bold'),
             bg=BG_DEEP, fg=FG_MUTED).pack(anchor='w')

    # Tag "A Molgroup member" a la derecha
    f_tag = tk.Frame(head, bg=BG_DEEP)
    f_tag.pack(side='right', padx=20)
    cv2_ = tk.Canvas(f_tag, bg=BG_DEEP, width=18, height=18,
                     highlightthickness=0, bd=0)
    cv2_.pack(side='left', padx=(0, 7))
    _dibujar_iso(cv2_, 1, 1, 16, color=CELESTE2)
    tk.Label(f_tag, text='A MOLGROUP MEMBER',
             font=(FAMILIA_COND, 9, 'bold'),
             bg=BG_DEEP, fg=CELESTE3).pack(side='left')

    return head


# ======================================================================
#  Pasada final de pulido
# ======================================================================

def pulir(widget):
    """Recorre el arbol de widgets aplicando el acabado del tema:
    botones planos, sin halos de foco, LabelFrames como eyebrows."""
    cls = widget.winfo_class()
    try:
        if cls == 'Button':
            bg_actual = str(widget.cget('bg'))
            fg_actual = str(widget.cget('fg'))
            widget.configure(bd=0, relief='flat', cursor='hand2',
                             highlightthickness=0,
                             activeforeground='#FFFFFF',
                             disabledforeground=FG_DIM)
            intencionales = {AZUL, AZUL_HOV, CELESTE, OK, ERR, WARN,
                             ABAJO, NEUTRO, BTN_BG, BTN_HOVER, AMARILLO,
                             WARN_TXT2, PANEL}
            if bg_actual not in intencionales:
                widget.configure(bg=BTN_BG, activebackground=BTN_HOVER)
                if fg_actual in ('black', '#000000', 'SystemButtonText'):
                    widget.configure(fg=FG)
            elif bg_actual == BTN_BG:
                widget.configure(activebackground=BTN_HOVER)
            try:
                if int(str(widget.cget('padx'))) < 10:
                    widget.configure(padx=10)
                if int(str(widget.cget('pady'))) < 5:
                    widget.configure(pady=5)
            except (ValueError, tk.TclError):
                pass
        elif cls == 'Scrollbar':
            widget.configure(bg=PANEL, troughcolor=BG_DEEP,
                             activebackground=AZUL, relief='flat',
                             bd=0, highlightthickness=0,
                             elementborderwidth=0)
        elif cls == 'Labelframe':
            widget.configure(bd=0, font=F_EYEBROW, fg=CELESTE2,
                             highlightthickness=0)
        elif cls in ('Checkbutton', 'Radiobutton'):
            widget.configure(bd=0, highlightthickness=0,
                             activebackground=widget.cget('bg'),
                             activeforeground=FG)
        elif cls == 'Scale':
            widget.configure(bd=0, highlightthickness=0,
                             sliderrelief='flat', sliderlength=22)
        elif cls in ('Entry', 'Spinbox'):
            widget.configure(relief='flat', bd=0,
                             highlightthickness=1,
                             highlightbackground=BORDE,
                             highlightcolor=AZUL,
                             bg=BG_DEEP, fg=FG,
                             insertbackground=CELESTE2,
                             disabledbackground=PANEL_DIM)
        elif cls == 'Listbox':
            widget.configure(relief='flat', bd=0,
                             highlightthickness=1,
                             highlightbackground=BORDE,
                             selectbackground=AZUL,
                             selectforeground='#FFFFFF')
        elif cls == 'Text':
            widget.configure(relief='flat', bd=0,
                             highlightthickness=1,
                             highlightbackground=BORDE,
                             insertbackground=CELESTE2)
        elif cls == 'Canvas':
            widget.configure(highlightthickness=0)
    except tk.TclError:
        pass

    for hijo in widget.winfo_children():
        pulir(hijo)
