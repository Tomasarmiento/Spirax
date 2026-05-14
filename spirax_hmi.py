"""
spirax_hmi.py - HMI Spirax (tkinter) + Servidor EKI integrado

Multi-modelo: cada Tipo (1-6) tiene su carpeta modelos/tipoN/ con su
referencia.png y su config.json.

Pestania OPERAR:
    - Selector de Tipo (1-6) - tilde verde si tiene referencia, X rojo si no
    - Modo Manual / Automatico (camara)
    - Botones de orientacion (modo Manual)
    - Panel de la ultima deteccion
    - Estado del robot

Pestania CALIBRAR:
    - Banner con el Tipo activo
    - Selector de Tipo (1-6) - mismo estilo, indica cual esta calibrado
    - Preview en vivo de la camara
    - Capturar referencia para el tipo activo (modelos/tipoN/referencia.png)
    - Ajustar recorte (4 lineas) para el tipo activo
    - Sliders de exposicion, ganancia y threshold (persistidos en
      modelos/tipoN/config.json)

Pestania LOG:
    - Eventos del servidor EKI y de la vision
"""

import os
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime

import cv2
from PIL import Image, ImageTk

from spirax_vision import (
    DetectorOrientacion,
    TIPOS_VALIDOS,
    tiene_referencia,
    path_referencia,
    path_config,
    modo_recorte,
)

# --- Configuracion red ---
HOST = '172.31.1.100'
PORT = 54600
BUFFER_SIZE = 1024

# Tamano del panel de imagen
PANEL_W = 400
PANEL_H = 300


# ======================================================================
#  Estado compartido entre threads
# ======================================================================

class EstadoCompartido:
    def __init__(self):
        self.tipo = 1
        self.orientacion = "ARRIBA"
        self.modo_auto = False
        self.lock = threading.Lock()
        self.consultas = 0
        self.ultima_consulta = None
        self.robot_conectado = False

        self.ultima_deteccion = None
        self.ultimo_ratio = None
        self.ultimo_panel = None
        self.ultimo_error = None

    def get_estado(self):
        with self.lock:
            return {
                "tipo": self.tipo,
                "orientacion": self.orientacion,
                "modo_auto": self.modo_auto,
            }

    def set_tipo(self, tipo):
        with self.lock:
            self.tipo = tipo

    def set_orientacion(self, orientacion):
        with self.lock:
            self.orientacion = orientacion

    def set_modo_auto(self, valor):
        with self.lock:
            self.modo_auto = bool(valor)

    def registrar_consulta(self):
        with self.lock:
            self.consultas += 1
            self.ultima_consulta = datetime.now()

    def set_deteccion(self, orientacion, ratio, panel, error=None):
        with self.lock:
            self.ultima_deteccion = orientacion
            self.ultimo_ratio = ratio
            self.ultimo_panel = panel
            self.ultimo_error = error


estado = EstadoCompartido()
detector = DetectorOrientacion(tipo_inicial=1)


# ======================================================================
#  Servidor TCP
# ======================================================================

def manejar_robot(conn, addr, log_callback):
    log_callback(f">>> Robot conectado desde {addr[0]}:{addr[1]}")
    estado.robot_conectado = True

    try:
        while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                log_callback("<<< Robot cerro la conexion")
                break

            mensaje = data.decode('utf-8').strip()
            log_callback(f"<-- Recibido: {mensaje}")

            estado.registrar_consulta()
            est = estado.get_estado()
            tipo = est["tipo"]

            if est["modo_auto"]:
                orientacion = consultar_vision(tipo, log_callback)
            else:
                orientacion = est["orientacion"]

            respuesta = (
                f"<Response>"
                f"<Tipo>{tipo}</Tipo>"
                f"<Orientacion>{orientacion}</Orientacion>"
                f"</Response>"
            )

            conn.sendall(respuesta.encode('utf-8'))
            log_callback(f"--> Enviado:  {respuesta}")

    except ConnectionResetError:
        log_callback("!!! Conexion perdida con el robot")
    except Exception as e:
        log_callback(f"!!! Error: {e}")
    finally:
        conn.close()
        estado.robot_conectado = False


def consultar_vision(tipo, log_callback):
    """Asegura que el detector use el tipo dado y analiza."""
    if not detector.esta_activo():
        log_callback("!!! Modo Auto pero detector inactivo -> VACIO")
        estado.set_deteccion("VACIO", None, None, error="Detector inactivo")
        return "VACIO"

    # Apuntar al tipo correcto antes de analizar (carga referencia + config)
    try:
        if detector.tipo_activo() != tipo:
            detector.usa_tipo(tipo)
            log_callback(f"[VISION] Cambio a Tipo {tipo}")
    except Exception as e:
        log_callback(f"!!! Error cambiando a Tipo {tipo}: {e}")
        estado.set_deteccion("VACIO", None, None, error=str(e))
        return "VACIO"

    # Si no hay referencia para ese tipo, mandamos VACIO y avisamos
    if not detector.tiene_referencia_activa():
        log_callback(f"!!! Tipo {tipo} no tiene referencia calibrada -> VACIO")
        estado.set_deteccion("VACIO", None, None,
                             error=f"Tipo {tipo} sin referencia")
        return "VACIO"

    try:
        res = detector.analizar()
        orient = res["orientacion"]
        ratio = res["ratio"]
        panel = res["panel"]

        estado.set_deteccion(orient, ratio, panel, error=None)

        try:
            cv2.imwrite("ultima_deteccion.png", panel)
        except Exception:
            pass

        ratio_str = f"{ratio:.3f}" if ratio is not None else "---"
        log_callback(f"[VISION] Tipo {tipo}: {orient} (ratio {ratio_str})")

        if orient not in ("ARRIBA", "ABAJO", "VACIO"):
            log_callback(f"[VISION] {orient} -> mando VACIO al robot")
            return "VACIO"
        return orient

    except Exception as e:
        log_callback(f"!!! Error en vision: {e}")
        estado.set_deteccion("VACIO", None, None, error=str(e))
        return "VACIO"


def correr_servidor(log_callback):
    log_callback(f"=== Servidor EKI iniciado en {HOST}:{PORT} ===")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as servidor:
        servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        servidor.bind((HOST, PORT))
        servidor.listen(1)

        log_callback("Esperando conexion del robot KUKA...")

        while True:
            try:
                conn, addr = servidor.accept()
                manejar_robot(conn, addr, log_callback)
                log_callback("Esperando proxima conexion del robot...")
            except Exception as e:
                log_callback(f"!!! Error en servidor: {e}")


# ======================================================================
#  HMI
# ======================================================================

class HMISpirax:
    def __init__(self, root):
        self.root = root
        self.root.title("Spirax HMI")
        self.root.geometry("1500x820")
        self.root.configure(bg='#2b2b2b')

        # Refs a imagenes
        self._img_operar = None
        self._img_calibrar = None
        self._img_referencia = None

        # Preview
        self._preview_activo = False
        self._preview_thread = None
        self._ultimo_frame_preview = None
        self._frame_lock = threading.Lock()

        # Botones de seleccion de tipo en cada pestana
        self.btns_tipo_operar = []
        self.btns_tipo_calibrar = []
        self.btns_orient = []

        # Tipo activo en CALIBRAR (independiente del de Operar)
        self._tipo_calibrar = 1

        self._construir_ui()
        self._iniciar_servidor()
        self._actualizar_estado_loop()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    #  UI raiz
    # ------------------------------------------------------------------
    def _construir_ui(self):
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('TNotebook', background='#2b2b2b', borderwidth=0)
        style.configure('TNotebook.Tab',
                        background='#3a3a3a', foreground='white',
                        padding=[20, 8], font=('Arial', 11, 'bold'))
        style.map('TNotebook.Tab',
                  background=[('selected', '#4a90e2')],
                  foreground=[('selected', 'white')])

        titulo = tk.Label(self.root, text="SPIRAX HMI",
                          font=("Arial", 18, "bold"),
                          bg='#2b2b2b', fg='white')
        titulo.pack(pady=(8, 4))

        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill='both', expand=True, padx=10, pady=5)

        self.tab_operar = tk.Frame(self.nb, bg='#2b2b2b')
        self.tab_calibrar = tk.Frame(self.nb, bg='#2b2b2b')
        self.tab_log = tk.Frame(self.nb, bg='#2b2b2b')

        self.nb.add(self.tab_operar, text='  OPERAR  ')
        self.nb.add(self.tab_calibrar, text='  CALIBRAR  ')
        self.nb.add(self.tab_log, text='  LOG  ')

        self.nb.bind('<<NotebookTabChanged>>', self._on_tab_changed)

        self._construir_tab_operar()
        self._construir_tab_calibrar()
        self._construir_tab_log()

        self._seleccionar_tipo_operar(1)
        self._seleccionar_orientacion("ARRIBA")
        self._cambiar_modo(False)
        self._seleccionar_tipo_calibrar(1)

    # ==================================================================
    #  Helper: estado visual de un boton de tipo
    # ==================================================================
    def _refrescar_boton_tipo(self, btn, tipo, seleccionado):
        """Pinta el boton de tipo segun:
        - Si es el seleccionado: azul.
        - Sino: blanco con tilde verde (calibrado) o cruz roja (sin ref).
        """
        if tiene_referencia(tipo):
            label = f"Tipo {tipo}\n  OK"
        else:
            label = f"Tipo {tipo}\n  --"

        if seleccionado:
            btn.configure(text=label, bg='#4a90e2', fg='white',
                          relief='sunken', activebackground='#4a90e2')
        else:
            color_fg = '#2d8f3a' if tiene_referencia(tipo) else '#cc4444'
            btn.configure(text=label, bg='SystemButtonFace', fg=color_fg,
                          relief='raised', activebackground='#dddddd')

    # ------------------------------------------------------------------
    #  OPERAR
    # ------------------------------------------------------------------
    def _construir_tab_operar(self):
        cont = tk.Frame(self.tab_operar, bg='#2b2b2b')
        cont.pack(fill='both', expand=True)

        col_izq = tk.Frame(cont, bg='#2b2b2b')
        col_izq.pack(side='left', fill='both', expand=True, padx=(15, 8), pady=10)

        col_der = tk.Frame(cont, bg='#2b2b2b')
        col_der.pack(side='right', fill='y', padx=(8, 15), pady=10)

        # Tipo
        f_tipo = tk.LabelFrame(col_izq, text=" TIPO DE PIEZA (Receta) ",
                               font=("Arial", 12, "bold"),
                               bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_tipo.pack(fill='x', pady=5)

        c_tipos = tk.Frame(f_tipo, bg='#2b2b2b')
        c_tipos.pack()
        for i in range(1, 7):
            btn = tk.Button(c_tipos, text=f"Tipo {i}",
                            font=("Arial", 12, "bold"),
                            width=8, height=3,
                            command=lambda t=i: self._seleccionar_tipo_operar(t))
            btn.grid(row=0, column=i - 1, padx=5, pady=5)
            self.btns_tipo_operar.append(btn)

        # Modo
        f_modo = tk.LabelFrame(col_izq, text=" MODO ORIENTACION ",
                               font=("Arial", 12, "bold"),
                               bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_modo.pack(fill='x', pady=10)

        c_modo = tk.Frame(f_modo, bg='#2b2b2b')
        c_modo.pack(fill='x')

        self.btn_manual = tk.Button(c_modo, text="MANUAL",
                                    font=("Arial", 13, "bold"),
                                    width=14, height=2,
                                    command=lambda: self._cambiar_modo(False))
        self.btn_manual.grid(row=0, column=0, padx=5, pady=5)

        self.btn_auto = tk.Button(c_modo, text="AUTOMATICO (camara)",
                                  font=("Arial", 13, "bold"),
                                  width=22, height=2,
                                  command=lambda: self._cambiar_modo(True))
        self.btn_auto.grid(row=0, column=1, padx=5, pady=5)

        self.lbl_camara = tk.Label(c_modo, text="Camara: apagada",
                                   font=("Arial", 10),
                                   bg='#2b2b2b', fg='#ff6b6b')
        self.lbl_camara.grid(row=0, column=2, padx=15, sticky='w')

        # Orientacion
        f_orient = tk.LabelFrame(col_izq, text=" ORIENTACION ",
                                 font=("Arial", 12, "bold"),
                                 bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_orient.pack(fill='x', pady=5)

        c_orient = tk.Frame(f_orient, bg='#2b2b2b')
        c_orient.pack()
        for i, op in enumerate(("ARRIBA", "ABAJO", "VACIO")):
            btn = tk.Button(c_orient, text=op,
                            font=("Arial", 14, "bold"),
                            width=12, height=2,
                            command=lambda o=op: self._seleccionar_orientacion(o))
            btn.grid(row=0, column=i, padx=8, pady=5)
            self.btns_orient.append(btn)

        # Estado
        f_estado = tk.LabelFrame(col_izq, text=" ESTADO ",
                                 font=("Arial", 11, "bold"),
                                 bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_estado.pack(fill='x', pady=5)

        self.lbl_seleccion = tk.Label(f_estado,
                                      text="Tipo: 1   |   Orientacion: ARRIBA",
                                      font=("Arial", 12),
                                      bg='#2b2b2b', fg='#7fffd4')
        self.lbl_seleccion.pack(anchor='w')

        self.lbl_robot = tk.Label(f_estado, text="Robot: desconectado",
                                  font=("Arial", 11),
                                  bg='#2b2b2b', fg='#ff6b6b')
        self.lbl_robot.pack(anchor='w', pady=(5, 0))

        self.lbl_consultas = tk.Label(f_estado, text="Consultas recibidas: 0",
                                      font=("Arial", 11),
                                      bg='#2b2b2b', fg='white')
        self.lbl_consultas.pack(anchor='w', pady=(5, 0))

        # Panel imagen
        f_img = tk.LabelFrame(col_der, text=" ULTIMA DETECCION ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=5, pady=5)
        f_img.pack(fill='y', expand=False)

        frame_canvas_op = tk.Frame(f_img, bg='#1a1a1a',
                                   width=PANEL_W, height=PANEL_H)
        frame_canvas_op.pack(padx=5, pady=5)
        frame_canvas_op.pack_propagate(False)

        self.canvas_operar = tk.Label(frame_canvas_op,
                                      bg='#1a1a1a', fg='#888888',
                                      text="(sin detecciones aun)",
                                      font=("Arial", 11))
        self.canvas_operar.pack(fill='both', expand=True)

        self.lbl_deteccion = tk.Label(f_img, text="Resultado: ---",
                                      font=("Arial", 11, "bold"),
                                      bg='#2b2b2b', fg='white')
        self.lbl_deteccion.pack(anchor='w', padx=5, pady=(5, 0))

        self.lbl_ratio = tk.Label(f_img, text="Ratio: ---",
                                  font=("Arial", 10),
                                  bg='#2b2b2b', fg='#cfcfcf')
        self.lbl_ratio.pack(anchor='w', padx=5, pady=(0, 5))

        self.btn_test = tk.Button(f_img, text="Capturar ahora (test)",
                                  font=("Arial", 10),
                                  command=self._capturar_test,
                                  state='disabled')
        self.btn_test.pack(fill='x', padx=5, pady=5)

    # ------------------------------------------------------------------
    #  CALIBRAR
    # ------------------------------------------------------------------
    def _construir_tab_calibrar(self):
        cont = tk.Frame(self.tab_calibrar, bg='#2b2b2b')
        cont.pack(fill='both', expand=True, padx=15, pady=10)

        # Banner: tipo activo
        f_banner = tk.Frame(cont, bg='#1a1a1a', height=60)
        f_banner.pack(fill='x', pady=(0, 10))
        f_banner.pack_propagate(False)

        self.lbl_banner_calibrar = tk.Label(
            f_banner, text="CALIBRANDO: TIPO 1",
            font=("Arial", 18, "bold"),
            bg='#1a1a1a', fg='#7fffd4')
        self.lbl_banner_calibrar.pack(side='left', padx=20, pady=10)

        self.lbl_estado_calibrar = tk.Label(
            f_banner, text="",
            font=("Arial", 11),
            bg='#1a1a1a', fg='#cfcfcf')
        self.lbl_estado_calibrar.pack(side='right', padx=20)

        # Selector de tipo
        f_sel = tk.LabelFrame(cont, text=" TIPO A CALIBRAR ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=10, pady=8)
        f_sel.pack(fill='x', pady=(0, 10))

        c_tipos_cal = tk.Frame(f_sel, bg='#2b2b2b')
        c_tipos_cal.pack()
        for i in range(1, 7):
            btn = tk.Button(c_tipos_cal, text=f"Tipo {i}",
                            font=("Arial", 11, "bold"),
                            width=8, height=3,
                            command=lambda t=i: self._seleccionar_tipo_calibrar(t))
            btn.grid(row=0, column=i - 1, padx=5, pady=3)
            self.btns_tipo_calibrar.append(btn)

        # Cuerpo: preview + controles
        cuerpo = tk.Frame(cont, bg='#2b2b2b')
        cuerpo.pack(fill='both', expand=True)
        cuerpo.columnconfigure(0, weight=1)
        cuerpo.columnconfigure(1, weight=0, minsize=280)
        cuerpo.rowconfigure(0, weight=1)

        col_izq = tk.Frame(cuerpo, bg='#2b2b2b')
        col_izq.grid(row=0, column=0, sticky='nsew', padx=(0, 10))

        col_der = tk.Frame(cuerpo, bg='#2b2b2b', width=280)
        col_der.grid(row=0, column=1, sticky='ns')
        col_der.grid_propagate(False)

        # Preview en vivo + referencia (lado a lado)
        f_visual = tk.Frame(col_izq, bg='#2b2b2b')
        f_visual.pack(fill='both', expand=True)
        f_visual.columnconfigure(0, weight=1, uniform='cols')
        f_visual.columnconfigure(1, weight=1, uniform='cols')
        f_visual.rowconfigure(0, weight=1)

        # --- Preview en vivo ---
        f_prev = tk.LabelFrame(f_visual, text=" PREVIEW EN VIVO ",
                               font=("Arial", 11, "bold"),
                               bg='#2b2b2b', fg='white', padx=5, pady=5)
        f_prev.grid(row=0, column=0, sticky='nsew', padx=(0, 5))

        frame_canvas = tk.Frame(f_prev, bg='#1a1a1a',
                                width=PANEL_W, height=PANEL_H)
        frame_canvas.pack(padx=5, pady=5)
        frame_canvas.pack_propagate(False)

        self.canvas_calibrar = tk.Label(
            frame_canvas,
            bg='#1a1a1a', fg='#888888',
            text="(preview apagado)\n\nApreta INICIAR PREVIEW",
            font=("Arial", 11))
        self.canvas_calibrar.pack(fill='both', expand=True)

        c_prev_btns = tk.Frame(f_prev, bg='#2b2b2b')
        c_prev_btns.pack(fill='x', padx=5, pady=5)

        self.btn_preview_start = tk.Button(c_prev_btns,
                                           text="INICIAR PREVIEW",
                                           font=("Arial", 11, "bold"),
                                           bg='#4a90e2', fg='white',
                                           command=self._toggle_preview)
        self.btn_preview_start.pack(side='left', padx=2)

        self.lbl_fps = tk.Label(c_prev_btns, text="",
                                font=("Arial", 9),
                                bg='#2b2b2b', fg='#cfcfcf')
        self.lbl_fps.pack(side='left', padx=10)

        # --- Referencia guardada del tipo activo ---
        f_ref = tk.LabelFrame(f_visual, text=" REFERENCIA GUARDADA ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=5, pady=5)
        f_ref.grid(row=0, column=1, sticky='nsew', padx=(5, 0))

        frame_canvas_ref = tk.Frame(f_ref, bg='#1a1a1a',
                                    width=PANEL_W, height=PANEL_H)
        frame_canvas_ref.pack(padx=5, pady=5)
        frame_canvas_ref.pack_propagate(False)

        self.canvas_referencia = tk.Label(
            frame_canvas_ref,
            bg='#1a1a1a', fg='#888888',
            text="(sin referencia)",
            font=("Arial", 11))
        self.canvas_referencia.pack(fill='both', expand=True)

        self.lbl_ref_info = tk.Label(f_ref, text="",
                                     font=("Arial", 9),
                                     bg='#2b2b2b', fg='#cfcfcf')
        self.lbl_ref_info.pack(anchor='w', padx=5, pady=(0, 5))

        # Acciones
        f_acc = tk.LabelFrame(col_der, text=" ACCIONES ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_acc.pack(fill='x', pady=5)

        tk.Button(f_acc, text="Capturar referencia",
                  font=("Arial", 11, "bold"),
                  bg='#2d8f3a', fg='white',
                  width=22, height=2,
                  command=self._capturar_referencia).pack(pady=4)

        tk.Button(f_acc, text="Ajustar recorte (4 lineas)",
                  font=("Arial", 11),
                  width=22, height=2,
                  command=self._ajustar_recorte).pack(pady=4)

        # Estado archivos
        f_est = tk.LabelFrame(col_der, text=" ESTADO ARCHIVOS ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_est.pack(fill='x', pady=5)

        self.lbl_ref = tk.Label(f_est, text="referencia.png: ?",
                                font=("Consolas", 10),
                                bg='#2b2b2b', fg='white',
                                justify='left', anchor='w', wraplength=240)
        self.lbl_ref.pack(anchor='w')

        self.lbl_cfg = tk.Label(f_est, text="config.json: ?",
                                font=("Consolas", 10),
                                bg='#2b2b2b', fg='white',
                                justify='left', anchor='w', wraplength=240)
        self.lbl_cfg.pack(anchor='w', pady=(3, 0))

        # Sliders
        f_sld = tk.LabelFrame(col_der, text=" PARAMETROS ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_sld.pack(fill='x', pady=5)

        tk.Label(f_sld, text="Exposicion:",
                 font=("Arial", 10),
                 bg='#2b2b2b', fg='white').pack(anchor='w')
        self.scl_exp = tk.Scale(f_sld, from_=1, to=200,
                                orient='horizontal', length=220,
                                bg='#2b2b2b', fg='white',
                                troughcolor='#1a1a1a',
                                highlightthickness=0,
                                command=self._on_exposure_change)
        self.scl_exp.pack(fill='x')
        self.scl_exp.bind('<ButtonRelease-1>', self._persistir_sliders)

        tk.Label(f_sld, text="Ganancia:",
                 font=("Arial", 10),
                 bg='#2b2b2b', fg='white').pack(anchor='w', pady=(5, 0))
        self.scl_gain = tk.Scale(f_sld, from_=0, to=128,
                                 orient='horizontal', length=220,
                                 bg='#2b2b2b', fg='white',
                                 troughcolor='#1a1a1a',
                                 highlightthickness=0,
                                 command=self._on_gain_change)
        self.scl_gain.pack(fill='x')
        self.scl_gain.bind('<ButtonRelease-1>', self._persistir_sliders)

        tk.Label(f_sld, text="Threshold BG Subtract:",
                 font=("Arial", 10),
                 bg='#2b2b2b', fg='white').pack(anchor='w', pady=(5, 0))
        self.scl_thr = tk.Scale(f_sld, from_=5, to=120,
                                orient='horizontal', length=220,
                                bg='#2b2b2b', fg='white',
                                troughcolor='#1a1a1a',
                                highlightthickness=0,
                                command=self._on_thr_change)
        self.scl_thr.pack(fill='x')
        self.scl_thr.bind('<ButtonRelease-1>', self._persistir_sliders)

    # ------------------------------------------------------------------
    #  LOG
    # ------------------------------------------------------------------
    def _construir_tab_log(self):
        cont = tk.Frame(self.tab_log, bg='#2b2b2b')
        cont.pack(fill='both', expand=True, padx=15, pady=10)

        self.txt_log = tk.Text(cont, bg='#1a1a1a', fg='#cfcfcf',
                               font=("Consolas", 10), wrap='word')
        self.txt_log.pack(fill='both', expand=True)

        c_btns = tk.Frame(cont, bg='#2b2b2b')
        c_btns.pack(fill='x', pady=(5, 0))
        tk.Button(c_btns, text="Limpiar log",
                  command=lambda: self.txt_log.delete('1.0', 'end')).pack(side='left')

    # ==================================================================
    #  Handlers OPERAR
    # ==================================================================
    def _seleccionar_tipo_operar(self, tipo):
        estado.set_tipo(tipo)
        # Refrescar estado visual de los 6 botones
        for i, btn in enumerate(self.btns_tipo_operar):
            self._refrescar_boton_tipo(btn, i + 1, seleccionado=(i + 1 == tipo))
        self._actualizar_label_seleccion()

    def _seleccionar_orientacion(self, orient):
        estado.set_orientacion(orient)
        colores = {"ARRIBA": "#2d8f3a", "ABAJO": "#cc7a00", "VACIO": "#777777"}
        for btn in self.btns_orient:
            if btn['text'] == orient:
                btn.configure(bg=colores[orient], fg='white', relief='sunken')
            else:
                btn.configure(bg='SystemButtonFace', fg='black', relief='raised')
        self._actualizar_label_seleccion()

    def _cambiar_modo(self, auto):
        if auto:
            if not detector.esta_activo():
                # No requerimos referencia para arrancar la camara: el
                # detector la cargara segun el tipo activo en cada consulta.
                # Pero avisamos si el tipo seleccionado actualmente no la tiene.
                est = estado.get_estado()
                if not tiene_referencia(est["tipo"]):
                    if not messagebox.askyesno(
                        "Atencion",
                        f"El Tipo {est['tipo']} no tiene referencia calibrada.\n"
                        f"Si el robot pide una deteccion ahora, se va a "
                        f"responder VACIO.\n\n"
                        f"Continuar igual?"):
                        return

                self._agregar_log("[VISION] Iniciando camara...")
                self.root.update_idletasks()
                try:
                    detector.start(requiere_referencia=False)
                    self._agregar_log("[VISION] Camara lista.")
                except Exception as e:
                    self._agregar_log(f"!!! Error iniciando camara: {e}")
                    messagebox.showerror("Camara",
                                         f"No se pudo iniciar la camara:\n\n{e}")
                    return

            estado.set_modo_auto(True)
            self.btn_auto.configure(bg='#2d8f3a', fg='white', relief='sunken')
            self.btn_manual.configure(bg='SystemButtonFace', fg='black', relief='raised')
            self.btn_test.configure(state='disabled')
            for btn in self.btns_orient:
                btn.configure(state='disabled',
                              bg='#3a3a3a', fg='#777777',
                              relief='flat',
                              disabledforeground='#777777')
        else:
            estado.set_modo_auto(False)
            self.btn_manual.configure(bg='#4a90e2', fg='white', relief='sunken')
            self.btn_auto.configure(bg='SystemButtonFace', fg='black', relief='raised')
            self.btn_test.configure(state='normal')
            for btn in self.btns_orient:
                btn.configure(state='normal')
            est = estado.get_estado()
            self._seleccionar_orientacion(est["orientacion"])

        self._actualizar_label_seleccion()

    def _capturar_test(self):
        est = estado.get_estado()
        tipo = est["tipo"]

        def trabajo():
            camara_iniciada_aqui = False
            try:
                if not detector.esta_activo():
                    self._log_thread_safe("[TEST] Iniciando camara...")
                    detector.start(requiere_referencia=False)
                    camara_iniciada_aqui = True
                    self._log_thread_safe("[TEST] Camara lista.")

                self._log_thread_safe(f"[TEST] Capturando con Tipo {tipo}...")
                consultar_vision(tipo, self._log_thread_safe)
            except Exception as e:
                self._log_thread_safe(f"!!! Error en test: {e}")
            finally:
                if camara_iniciada_aqui and detector.esta_activo():
                    try:
                        detector.stop()
                        self._log_thread_safe("[TEST] Camara apagada.")
                    except Exception as e:
                        self._log_thread_safe(f"!!! Error apagando camara: {e}")

        threading.Thread(target=trabajo, daemon=True).start()

    def _actualizar_label_seleccion(self):
        est = estado.get_estado()
        orient_txt = "(automatica - camara)" if est["modo_auto"] else est["orientacion"]
        cal = "OK" if tiene_referencia(est["tipo"]) else "SIN CALIBRAR"
        self.lbl_seleccion.configure(
            text=f"Tipo: {est['tipo']} ({cal})   |   Orientacion: {orient_txt}"
        )

    # ==================================================================
    #  Handlers CALIBRAR
    # ==================================================================
    def _seleccionar_tipo_calibrar(self, tipo):
        """Cambia el tipo activo en la pestana Calibrar.
        Recarga sliders, recorte, banner y estado de archivos.
        """
        # Si hay preview corriendo lo paramos para reiniciar la camara con
        # los params del nuevo tipo (exposicion/ganancia)
        venia_preview = self._preview_activo
        if venia_preview:
            self._detener_preview()

        self._tipo_calibrar = tipo

        # Hacer que el detector use esa config (recarga referencia + params)
        try:
            detector.usa_tipo(tipo)
        except Exception as e:
            self._agregar_log(f"!!! Error cargando Tipo {tipo}: {e}")

        # Refrescar botones
        for i, btn in enumerate(self.btns_tipo_calibrar):
            self._refrescar_boton_tipo(btn, i + 1, seleccionado=(i + 1 == tipo))

        # Refrescar banner
        if tiene_referencia(tipo):
            self.lbl_banner_calibrar.configure(
                text=f"CALIBRANDO: TIPO {tipo}", fg='#7fffd4')
            self.lbl_estado_calibrar.configure(
                text="(referencia OK)", fg='#7fff7f')
        else:
            self.lbl_banner_calibrar.configure(
                text=f"CALIBRANDO: TIPO {tipo}", fg='#ffaa55')
            self.lbl_estado_calibrar.configure(
                text="(falta capturar referencia)", fg='#ff6b6b')

        # Refrescar sliders con los params del tipo
        try:
            self.scl_exp.set(detector.exposure)
            self.scl_gain.set(detector.gain)
            self.scl_thr.set(detector.bg_diff_thresh)
        except Exception:
            pass

        self._refrescar_estado_archivos()
        self._mostrar_referencia_calibrar()
        self._agregar_log(f"[CALIBRAR] Tipo activo: {tipo}")

        # Si veniamos con preview activo, reanudarlo con los nuevos params
        if venia_preview:
            self._iniciar_preview()

    def _toggle_preview(self):
        if self._preview_activo:
            self._detener_preview()
        else:
            self._iniciar_preview()

    def _iniciar_preview(self):
        if not detector.esta_activo():
            try:
                detector.start(requiere_referencia=False)
                self._agregar_log("[CALIBRAR] Camara iniciada para preview.")
            except Exception as e:
                self._agregar_log(f"!!! Error iniciando camara: {e}")
                messagebox.showerror("Camara", f"No se pudo iniciar:\n\n{e}")
                return

        self._preview_activo = True
        self.btn_preview_start.configure(text="DETENER PREVIEW", bg='#cc4444')

        self._preview_thread = threading.Thread(
            target=self._loop_preview, daemon=True)
        self._preview_thread.start()

    def _detener_preview(self):
        self._preview_activo = False
        self.btn_preview_start.configure(text="INICIAR PREVIEW", bg='#4a90e2')
        self.lbl_fps.configure(text="")

    def _loop_preview(self):
        ultimo_t = time.time()
        contador = 0

        while self._preview_activo and detector.esta_activo():
            try:
                frame = detector.capturar_frame()
                with self._frame_lock:
                    self._ultimo_frame_preview = frame.copy()

                self.root.after(0, self._dibujar_preview, frame)

                contador += 1
                ahora = time.time()
                if ahora - ultimo_t >= 1.0:
                    fps = contador / (ahora - ultimo_t)
                    self.root.after(0,
                                    lambda f=fps:
                                    self.lbl_fps.configure(text=f"{f:.1f} fps"))
                    contador = 0
                    ultimo_t = ahora

                time.sleep(0.05)

            except Exception as e:
                self.root.after(0, self._agregar_log,
                                f"!!! Error en preview: {e}")
                self._preview_activo = False
                self.root.after(0, self._detener_preview)
                break

    def _dibujar_preview(self, frame):
        try:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            img.thumbnail((PANEL_W, PANEL_H), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_calibrar.configure(image=photo, text='')
            self._img_calibrar = photo
        except Exception:
            pass

    def _mostrar_referencia_calibrar(self):
        """Carga y muestra la referencia.png del tipo activo en Calibrar
        con el recorte configurado superpuesto (4 lineas amarillas +
        sombreado fuera del area util). Si no hay referencia, muestra
        placeholder.
        """
        tipo = self._tipo_calibrar
        ref_path = path_referencia(tipo)

        if not os.path.exists(ref_path):
            self.canvas_referencia.configure(
                image='',
                text=f"(Tipo {tipo} sin referencia)\n\nCapturala con el boton\n'Capturar referencia'",
                fg='#ff6b6b')
            self._img_referencia = None
            self.lbl_ref_info.configure(text="")
            return

        try:
            from spirax_vision import cargar_config
            import numpy as np

            ref = cv2.imread(ref_path)
            if ref is None:
                raise RuntimeError("No se pudo leer la imagen")

            cfg = cargar_config(tipo)
            h, w = ref.shape[:2]
            y_top = int(h * cfg.get("corte_y_top_pct", 0.0))
            y_bot = int(h * cfg.get("corte_y_pct", 1.0))
            x_izq = int(w * cfg.get("corte_x_izq_pct", 0.0))
            x_der = int(w * cfg.get("corte_x_der_pct", 1.0))

            # Sombrear zonas fuera del recorte (igual que modo_recorte)
            debug = ref.copy()
            overlay = debug.copy()
            cv2.rectangle(overlay, (0, 0), (w, y_top), (0, 0, 0), -1)
            cv2.rectangle(overlay, (0, y_bot), (w, h), (0, 0, 0), -1)
            cv2.rectangle(overlay, (0, y_top), (x_izq, y_bot), (0, 0, 0), -1)
            cv2.rectangle(overlay, (x_der, y_top), (w, y_bot), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, debug, 0.5, 0, debug)

            # Lineas amarillas del recorte
            cv2.line(debug, (0, y_top), (w, y_top), (0, 255, 255), 2)
            cv2.line(debug, (0, y_bot), (w, y_bot), (0, 255, 255), 2)
            cv2.line(debug, (x_izq, 0), (x_izq, h), (0, 255, 255), 2)
            cv2.line(debug, (x_der, 0), (x_der, h), (0, 255, 255), 2)

            rgb = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            img.thumbnail((PANEL_W, PANEL_H), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_referencia.configure(image=photo, text='')
            self._img_referencia = photo

            ts = datetime.fromtimestamp(os.path.getmtime(ref_path))
            self.lbl_ref_info.configure(
                text=(f"Tipo {tipo} - {ts.strftime('%Y-%m-%d %H:%M')}  |  "
                      f"recorte: Y {y_top}-{y_bot}  X {x_izq}-{x_der}"))
        except Exception as e:
            self.canvas_referencia.configure(
                image='',
                text=f"(error cargando)\n{e}",
                fg='#ff6b6b')
            self._img_referencia = None
            self.lbl_ref_info.configure(text="")

    def _capturar_referencia(self):
        tipo = self._tipo_calibrar

        if not detector.esta_activo():
            try:
                detector.start(requiere_referencia=False)
                self._agregar_log("[CALIBRAR] Camara iniciada.")
            except Exception as e:
                messagebox.showerror("Camara", f"No se pudo iniciar:\n\n{e}")
                return

        respuesta = messagebox.askyesno(
            "Capturar referencia",
            f"Capturar referencia para TIPO {tipo}.\n\n"
            f"Asegurate de que NO hay pieza en el setup.\n\n"
            f"Se va a sacar una foto del fondo vacio para usar como "
            f"referencia de Background Subtraction del Tipo {tipo}.\n\n"
            f"Continuar?"
        )
        if not respuesta:
            return

        # Si hay preview activo usamos el ultimo frame del preview
        with self._frame_lock:
            frame = (self._ultimo_frame_preview.copy()
                     if self._ultimo_frame_preview is not None else None)

        if frame is None:
            try:
                frame = detector.capturar_frame()
            except Exception as e:
                messagebox.showerror("Captura", f"No se pudo capturar:\n\n{e}")
                return

        try:
            ref_path = path_referencia(tipo)
            cv2.imwrite(ref_path, frame)
            detector.recargar_referencia()
            self._agregar_log(f"[CALIBRAR] Referencia Tipo {tipo} guardada en {ref_path}")
            messagebox.showinfo("Referencia",
                                f"Guardada como {ref_path}.\n\n"
                                f"Ya queda cargada en el detector.")
            # Refrescar estado visual de todos los botones de tipo
            self._refrescar_estado_archivos()
            for i, btn in enumerate(self.btns_tipo_calibrar):
                self._refrescar_boton_tipo(btn, i + 1,
                                           seleccionado=(i + 1 == tipo))
            for i, btn in enumerate(self.btns_tipo_operar):
                est = estado.get_estado()
                self._refrescar_boton_tipo(btn, i + 1,
                                           seleccionado=(i + 1 == est["tipo"]))
            # Banner
            self.lbl_banner_calibrar.configure(
                text=f"CALIBRANDO: TIPO {tipo}", fg='#7fffd4')
            self.lbl_estado_calibrar.configure(
                text="(referencia OK)", fg='#7fff7f')
            # Mostrar la imagen recien capturada en el panel de la derecha
            self._mostrar_referencia_calibrar()
        except Exception as e:
            self._agregar_log(f"!!! Error guardando referencia: {e}")
            messagebox.showerror("Error", str(e))

    def _ajustar_recorte(self):
        tipo = self._tipo_calibrar
        ref_path = path_referencia(tipo)

        if os.path.exists(ref_path):
            imagen = cv2.imread(ref_path)
            self._agregar_log(f"[CALIBRAR] Ajustando recorte sobre referencia Tipo {tipo}")
        else:
            if not detector.esta_activo():
                try:
                    detector.start(requiere_referencia=False)
                except Exception as e:
                    messagebox.showerror("Camara", f"No se pudo iniciar:\n\n{e}")
                    return
            try:
                imagen = detector.capturar_frame()
                self._agregar_log(f"[CALIBRAR] Ajustando recorte sobre frame en vivo (Tipo {tipo})")
            except Exception as e:
                messagebox.showerror("Captura", f"No se pudo capturar:\n\n{e}")
                return

        venia_preview = self._preview_activo
        if venia_preview:
            self._detener_preview()

        try:
            ok = modo_recorte(imagen, tipo)
            if ok:
                # Recargar referencia con el nuevo recorte
                if os.path.exists(ref_path):
                    detector.recargar_referencia()
                self._agregar_log(f"[CALIBRAR] Recorte Tipo {tipo} guardado.")
                self._refrescar_estado_archivos()
                # Refrescar la imagen de referencia con las lineas del nuevo recorte
                self._mostrar_referencia_calibrar()
            else:
                self._agregar_log("[CALIBRAR] Recorte cancelado.")
        except Exception as e:
            self._agregar_log(f"!!! Error en ajustar_recorte: {e}")
            messagebox.showerror("Error", str(e))

    def _on_exposure_change(self, valor):
        try:
            detector.set_exposure(int(valor), persist=False)
        except Exception as e:
            self._agregar_log(f"!!! exposure: {e}")

    def _on_gain_change(self, valor):
        try:
            detector.set_gain(int(valor), persist=False)
        except Exception as e:
            self._agregar_log(f"!!! gain: {e}")

    def _on_thr_change(self, valor):
        try:
            detector.set_threshold(int(valor), persist=False)
        except Exception as e:
            self._agregar_log(f"!!! threshold: {e}")

    def _persistir_sliders(self, _evt=None):
        """Llamado al soltar cualquier slider: guarda los 4 valores
        actuales en config.json del TIPO ACTIVO en Calibrar.
        """
        try:
            detector.set_exposure(detector.exposure, persist=True)
            self._agregar_log(
                f"[CONFIG] Tipo {self._tipo_calibrar}: "
                f"exp={detector.exposure} gain={detector.gain} "
                f"thr={detector.bg_diff_thresh}"
            )
        except Exception as e:
            self._agregar_log(f"!!! persistiendo: {e}")

    def _refrescar_estado_archivos(self):
        tipo = self._tipo_calibrar
        ref_path = path_referencia(tipo)
        cfg_path = path_config(tipo)

        if os.path.exists(ref_path):
            ts = datetime.fromtimestamp(os.path.getmtime(ref_path))
            self.lbl_ref.configure(
                text=f"referencia OK\n  {ts.strftime('%Y-%m-%d %H:%M')}",
                fg='#7fff7f')
        else:
            self.lbl_ref.configure(text="referencia FALTA",
                                   fg='#ff6b6b')
        if os.path.exists(cfg_path):
            self.lbl_cfg.configure(text="config OK",
                                   fg='#7fff7f')
        else:
            self.lbl_cfg.configure(text="config FALTA (default)",
                                   fg='#ffaa55')

    # ==================================================================
    #  Tab change
    # ==================================================================
    def _on_tab_changed(self, event):
        try:
            tab = self.nb.tab(self.nb.select(), 'text').strip()
        except Exception:
            return
        if tab != 'CALIBRAR' and self._preview_activo:
            self._detener_preview()
            self._agregar_log("[CALIBRAR] Preview detenido (cambio de pestania).")
        if tab == 'CALIBRAR':
            self._refrescar_estado_archivos()

    # ==================================================================
    #  Log
    # ==================================================================
    def _agregar_log(self, mensaje):
        ahora = datetime.now().strftime('%H:%M:%S')
        self.txt_log.insert('end', f"[{ahora}] {mensaje}\n")
        self.txt_log.see('end')

    def _log_thread_safe(self, mensaje):
        self.root.after(0, self._agregar_log, mensaje)

    # ==================================================================
    #  Server
    # ==================================================================
    def _iniciar_servidor(self):
        thread = threading.Thread(target=correr_servidor,
                                  args=(self._log_thread_safe,),
                                  daemon=True)
        thread.start()

    # ==================================================================
    #  Loop refresco
    # ==================================================================
    def _actualizar_estado_loop(self):
        if estado.robot_conectado:
            self.lbl_robot.configure(text="Robot: CONECTADO", fg='#7fff7f')
        else:
            self.lbl_robot.configure(text="Robot: desconectado", fg='#ff6b6b')

        self.lbl_consultas.configure(text=f"Consultas recibidas: {estado.consultas}")

        if detector.esta_activo():
            self.lbl_camara.configure(text="Camara: ACTIVA", fg='#7fff7f')
        else:
            self.lbl_camara.configure(text="Camara: apagada", fg='#ff6b6b')

        with estado.lock:
            panel = estado.ultimo_panel
            orient = estado.ultima_deteccion
            ratio = estado.ultimo_ratio
            error = estado.ultimo_error

        if panel is not None:
            self._mostrar_panel_operar(panel)
        elif error:
            self.canvas_operar.configure(text=f"Error:\n{error}",
                                         image='', fg='#ff6b6b')
            self._img_operar = None

        if orient is not None:
            colores = {"ARRIBA": "#7fff7f", "ABAJO": "#ffa500",
                       "VACIO": "#cccccc", "DUDOSO": "#ffff7f",
                       "N/A": "#ff6b6b"}
            self.lbl_deteccion.configure(
                text=f"Resultado: {orient}",
                fg=colores.get(orient, 'white'))
            self.lbl_ratio.configure(
                text=f"Ratio: {ratio:.3f}" if ratio is not None else "Ratio: ---")

        self.root.after(500, self._actualizar_estado_loop)

    def _mostrar_panel_operar(self, panel_bgr):
        try:
            rgb = cv2.cvtColor(panel_bgr, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            img.thumbnail((PANEL_W, PANEL_H), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_operar.configure(image=photo, text='')
            self._img_operar = photo
        except Exception as e:
            self._agregar_log(f"!!! Error mostrando panel: {e}")

    # ==================================================================
    #  Cierre
    # ==================================================================
    def _on_close(self):
        self._preview_activo = False
        try:
            if detector.esta_activo():
                detector.stop()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    HMISpirax(root)
    root.mainloop()


if __name__ == '__main__':
    main()