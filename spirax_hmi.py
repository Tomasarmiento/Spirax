"""
spirax_hmi.py - HMI Spirax con soporte para Cinta + Mesa

Cambios vs version anterior:
- Pestania CALIBRAR ahora tiene toggle [Cinta] [Mesa] (calibra una u otra
  para el tipo activo)
- Nueva pestania MESA en OPERAR con visualizacion de la matriz 10x8
- Servidor TCP maneja <Estacion>1</Estacion> (cinta) y <Estacion>3</Estacion> (mesa)
- Response unificado con campos Tipo, Orientacion, Fila, Columna
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
    ESTACIONES_VALIDAS,
    tiene_referencia,
    path_referencia,
    path_config,
    modo_recorte,
)
from spirax_mesa import (
    GRILLA_FILAS, GRILLA_COLS,
    matriz_vacia, matriz_con_celda,
)

# --- Configuracion red ---
HOST = '172.31.1.100'
PORT = 54600
BUFFER_SIZE = 1024
RECV_TIMEOUT = 60.0

PANEL_W = 400
PANEL_H = 300

# Tamano del visualizador de la matriz mesa (en pixels)
MATRIZ_W = 360
MATRIZ_H = 240


# ======================================================================
#  Estado compartido entre threads
# ======================================================================

class EstadoCompartido:
    def __init__(self):
        # Configuracion cinta
        self.tipo = 1
        self.orientacion_manual = "ARRIBA"
        self.modo_auto_cinta = False

        # Configuracion mesa
        self.modo_auto_mesa = False
        self.mesa_manual_fila = 1
        self.mesa_manual_columna = 1
        # Cache de la ultima deteccion de mesa (para el panel)
        self.ultima_matriz_mesa = None
        self.ultima_fila_mesa = 0
        self.ultima_columna_mesa = 0

        self.lock = threading.Lock()
        self.consultas = 0
        self.ultima_consulta = None
        self.robot_conectado = False

        self.ultima_deteccion_cinta = None
        self.ultimo_ratio_cinta = None
        self.ultimo_panel_cinta = None
        self.ultimo_panel_mesa = None
        self.ultimo_error = None

        self.shutdown = False

    def get_estado(self):
        with self.lock:
            return {
                "tipo": self.tipo,
                "orientacion_manual": self.orientacion_manual,
                "modo_auto_cinta": self.modo_auto_cinta,
                "modo_auto_mesa": self.modo_auto_mesa,
                "mesa_manual_fila": self.mesa_manual_fila,
                "mesa_manual_columna": self.mesa_manual_columna,
            }

    def set_tipo(self, tipo):
        with self.lock:
            self.tipo = tipo

    def set_orientacion_manual(self, orientacion):
        with self.lock:
            self.orientacion_manual = orientacion

    def set_modo_auto_cinta(self, valor):
        with self.lock:
            self.modo_auto_cinta = bool(valor)

    def set_modo_auto_mesa(self, valor):
        with self.lock:
            self.modo_auto_mesa = bool(valor)

    def set_mesa_manual(self, fila, columna):
        with self.lock:
            self.mesa_manual_fila = fila
            self.mesa_manual_columna = columna

    def registrar_consulta(self):
        with self.lock:
            self.consultas += 1
            self.ultima_consulta = datetime.now()

    def set_deteccion_cinta(self, orientacion, ratio, panel, error=None):
        with self.lock:
            self.ultima_deteccion_cinta = orientacion
            self.ultimo_ratio_cinta = ratio
            self.ultimo_panel_cinta = panel
            self.ultimo_error = error

    def set_deteccion_mesa(self, matriz, fila, columna, panel, error=None):
        with self.lock:
            self.ultima_matriz_mesa = matriz
            self.ultima_fila_mesa = fila
            self.ultima_columna_mesa = columna
            self.ultimo_panel_mesa = panel
            self.ultimo_error = error


estado = EstadoCompartido()
detector = DetectorOrientacion(tipo_inicial=1, estacion_inicial="cinta")


# ======================================================================
#  Servidor TCP - keepalive + reconexion
# ======================================================================

def configurar_keepalive(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    if hasattr(socket, 'SIO_KEEPALIVE_VALS'):
        try:
            sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 10000, 3000))
        except (OSError, AttributeError, TypeError):
            pass
    if hasattr(socket, 'TCP_KEEPIDLE'):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
        except OSError:
            pass
    if hasattr(socket, 'TCP_KEEPINTVL'):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
        except OSError:
            pass
    if hasattr(socket, 'TCP_KEEPCNT'):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        except OSError:
            pass


def _extraer_estacion(mensaje):
    """Saca el numero de estacion del XML <Estacion>N</Estacion>.
    Devuelve 1 si no encuentra (default = cinta para compatibilidad).
    """
    import re
    m = re.search(r"<Estacion>\s*(\d+)\s*</Estacion>", mensaje)
    if m:
        try:
            return int(m.group(1))
        except ValueError:
            return 1
    return 1


def manejar_robot(conn, addr, log_callback):
    log_callback(f">>> Robot conectado desde {addr[0]}:{addr[1]}")
    estado.robot_conectado = True

    configurar_keepalive(conn)
    conn.settimeout(RECV_TIMEOUT)

    try:
        while not estado.shutdown:
            try:
                data = conn.recv(BUFFER_SIZE)
            except socket.timeout:
                try:
                    conn.sendall(b'')
                    continue
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    log_callback(f"<<< Robot no responde al poll: {e}")
                    break
            except (ConnectionResetError, ConnectionAbortedError) as e:
                log_callback(f"<<< Conexion perdida con el robot: {e}")
                break

            if not data:
                log_callback("<<< Robot cerro la conexion (FIN)")
                break

            mensaje = data.decode('utf-8', errors='replace').strip()
            log_callback(f"<-- Recibido: {mensaje}")

            estado.registrar_consulta()
            est = estado.get_estado()
            tipo = est["tipo"]

            num_estacion = _extraer_estacion(mensaje)

            # Defaults de la respuesta
            orientacion = "VACIO"
            fila = 0
            columna = 0

            if num_estacion == 1:
                # Cinta: devolver Tipo + Orientacion
                if est["modo_auto_cinta"]:
                    orientacion = consultar_vision_cinta(tipo, log_callback)
                else:
                    orientacion = est["orientacion_manual"]

            elif num_estacion == 3:
                # Mesa: devolver Tipo + Fila + Columna
                if est["modo_auto_mesa"]:
                    fila, columna = consultar_vision_mesa(tipo, log_callback)
                else:
                    fila = est["mesa_manual_fila"]
                    columna = est["mesa_manual_columna"]
                    # Marcar el panel para que se vea en HMI
                    estado.set_deteccion_mesa(
                        matriz_con_celda(fila, columna),
                        fila, columna, None,
                    )

                if fila == 0 or columna == 0:
                    orientacion = "VACIO"
                else:
                    orientacion = "OK"

            else:
                log_callback(f"!!! Estacion desconocida: {num_estacion}")

            respuesta = (
                f"<Response>"
                f"<Tipo>{tipo}</Tipo>"
                f"<Orientacion>{orientacion}</Orientacion>"
                f"<Fila>{fila}</Fila>"
                f"<Columna>{columna}</Columna>"
                f"</Response>"
            )

            try:
                conn.sendall(respuesta.encode('utf-8'))
                log_callback(f"--> Enviado:  {respuesta}")
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                log_callback(f"!!! Error enviando respuesta: {e}")
                break

    except Exception as e:
        log_callback(f"!!! Error inesperado: {e}")
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            conn.close()
        except OSError:
            pass
        estado.robot_conectado = False


def consultar_vision_cinta(tipo, log_callback):
    """Devuelve ARRIBA/ABAJO/VACIO segun la camara, en estacion cinta."""
    if not detector.esta_activo():
        log_callback("!!! Detector inactivo -> VACIO")
        estado.set_deteccion_cinta("VACIO", None, None, error="Detector inactivo")
        return "VACIO"

    try:
        if (detector.tipo_activo() != tipo
                or detector.estacion_activa() != "cinta"):
            detector.usa_tipo(tipo, "cinta")
            log_callback(f"[VISION] Cambio a Tipo {tipo} (cinta)")
    except Exception as e:
        log_callback(f"!!! Error cambiando a Tipo {tipo} (cinta): {e}")
        estado.set_deteccion_cinta("VACIO", None, None, error=str(e))
        return "VACIO"

    if not detector.tiene_referencia_activa():
        log_callback(f"!!! Tipo {tipo} (cinta) sin referencia -> VACIO")
        estado.set_deteccion_cinta("VACIO", None, None,
                                    error=f"Tipo {tipo} cinta sin referencia")
        return "VACIO"

    try:
        res = detector.analizar()
        orient = res["orientacion"]
        ratio = res["ratio"]
        panel = res["panel"]

        estado.set_deteccion_cinta(orient, ratio, panel, error=None)

        ratio_str = f"{ratio:.3f}" if ratio is not None else "---"
        log_callback(f"[VISION] Cinta Tipo {tipo}: {orient} (ratio {ratio_str})")

        if orient not in ("ARRIBA", "ABAJO", "VACIO"):
            log_callback(f"[VISION] {orient} -> mando VACIO")
            return "VACIO"
        return orient

    except Exception as e:
        log_callback(f"!!! Error en vision cinta: {e}")
        estado.set_deteccion_cinta("VACIO", None, None, error=str(e))
        return "VACIO"


def consultar_vision_mesa(tipo, log_callback):
    """Devuelve (fila, columna) 1-indexed de la primera pieza en la mesa.
    (0, 0) si mesa vacia."""
    if not detector.esta_activo():
        log_callback("!!! Detector inactivo -> mesa vacia")
        estado.set_deteccion_mesa(None, 0, 0, None, error="Detector inactivo")
        return (0, 0)

    try:
        if (detector.tipo_activo() != tipo
                or detector.estacion_activa() != "mesa"):
            detector.usa_tipo(tipo, "mesa")
            log_callback(f"[VISION] Cambio a Tipo {tipo} (mesa)")
    except Exception as e:
        log_callback(f"!!! Error cambiando a Tipo {tipo} (mesa): {e}")
        estado.set_deteccion_mesa(None, 0, 0, None, error=str(e))
        return (0, 0)

    if not detector.tiene_referencia_activa():
        log_callback(f"!!! Tipo {tipo} (mesa) sin referencia -> 0,0")
        estado.set_deteccion_mesa(None, 0, 0, None,
                                   error=f"Tipo {tipo} mesa sin referencia")
        return (0, 0)

    try:
        res = detector.analizar()
        matriz = res.get("matriz")
        fila = res.get("fila", 0)
        columna = res.get("columna", 0)
        panel = res.get("panel")

        estado.set_deteccion_mesa(matriz, fila, columna, panel, error=None)

        n_piezas = 0
        if matriz is not None:
            n_piezas = sum(sum(1 for c in f if c) for f in matriz)

        log_callback(f"[VISION] Mesa Tipo {tipo}: {n_piezas} piezas, "
                     f"elegida=({fila},{columna})")
        return (fila, columna)

    except Exception as e:
        log_callback(f"!!! Error en vision mesa: {e}")
        estado.set_deteccion_mesa(None, 0, 0, None, error=str(e))
        return (0, 0)


def correr_servidor(log_callback):
    log_callback(f"=== Servidor EKI iniciado en {HOST}:{PORT} ===")

    while not estado.shutdown:
        servidor = None
        try:
            servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except (AttributeError, OSError):
                pass

            servidor.settimeout(1.0)
            servidor.bind((HOST, PORT))
            servidor.listen(1)

            log_callback("Esperando conexion del robot KUKA...")

            while not estado.shutdown:
                try:
                    conn, addr = servidor.accept()
                except socket.timeout:
                    continue
                except OSError as e:
                    log_callback(f"!!! accept() fallo: {e}")
                    break

                manejar_robot(conn, addr, log_callback)
                if not estado.shutdown:
                    log_callback("Esperando proxima conexion del robot...")

        except OSError as e:
            log_callback(f"!!! Socket roto, reabriendo en 2s: {e}")
            time.sleep(2)
        except Exception as e:
            log_callback(f"!!! Error fatal: {e}")
            time.sleep(2)
        finally:
            if servidor is not None:
                try:
                    servidor.close()
                except OSError:
                    pass

    log_callback("=== Servidor EKI detenido ===")


# ======================================================================
#  HMI
# ======================================================================

class HMISpirax:
    def __init__(self, root):
        self.root = root
        self.root.title("Spirax HMI")
        self.root.geometry("1600x900")
        self.root.configure(bg='#2b2b2b')

        # Refs a imagenes (evitan garbage collection)
        self._img_cinta = None
        self._img_calibrar = None
        self._img_referencia = None
        self._img_matriz = None

        # Preview
        self._preview_activo = False
        self._preview_thread = None
        self._ultimo_frame_preview = None
        self._frame_lock = threading.Lock()

        # Botones (los referencio para poder pintarlos segun estado)
        self.btns_tipo_operar = []
        self.btns_tipo_calibrar = []
        self.btns_orient = []

        # Estado de calibrar
        self._tipo_calibrar = 1
        self._estacion_calibrar = "cinta"

        self._construir_ui()
        self._iniciar_servidor()
        self._actualizar_estado_loop()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

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
        self._cambiar_modo_cinta(False)
        self._cambiar_modo_mesa(False)
        self._seleccionar_tipo_calibrar(1)

    # ==================================================================
    #  Boton de tipo (colores segun calibracion)
    # ==================================================================
    def _refrescar_boton_tipo(self, btn, tipo, seleccionado,
                               estacion="cinta"):
        if tiene_referencia(tipo, estacion):
            label = f"Tipo {tipo}\n  OK"
        else:
            label = f"Tipo {tipo}\n  --"

        if seleccionado:
            btn.configure(text=label, bg='#4a90e2', fg='white',
                          relief='sunken', activebackground='#4a90e2')
        else:
            color_fg = '#2d8f3a' if tiene_referencia(tipo, estacion) else '#cc4444'
            btn.configure(text=label, bg='SystemButtonFace', fg=color_fg,
                          relief='raised', activebackground='#dddddd')

    # ------------------------------------------------------------------
    #  OPERAR
    # ------------------------------------------------------------------
    def _construir_tab_operar(self):
        cont = tk.Frame(self.tab_operar, bg='#2b2b2b')
        cont.pack(fill='both', expand=True)

        col_izq = tk.Frame(cont, bg='#2b2b2b')
        col_izq.pack(side='left', fill='both', expand=True,
                     padx=(15, 8), pady=10)

        col_der = tk.Frame(cont, bg='#2b2b2b')
        col_der.pack(side='right', fill='y', padx=(8, 15), pady=10)

        # Tipo (afecta tanto cinta como mesa)
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

        # =========== Seccion CINTA ===========
        f_cinta = tk.LabelFrame(col_izq, text=" CINTA (Estacion 1) ",
                                 font=("Arial", 12, "bold"),
                                 bg='#2b2b2b', fg='#7fffd4',
                                 padx=10, pady=10)
        f_cinta.pack(fill='x', pady=8)

        c_modo_c = tk.Frame(f_cinta, bg='#2b2b2b')
        c_modo_c.pack(fill='x')

        self.btn_manual_cinta = tk.Button(c_modo_c, text="MANUAL",
                                           font=("Arial", 12, "bold"),
                                           width=14, height=2,
                                           command=lambda: self._cambiar_modo_cinta(False))
        self.btn_manual_cinta.grid(row=0, column=0, padx=5, pady=5)

        self.btn_auto_cinta = tk.Button(c_modo_c, text="AUTOMATICO (camara)",
                                         font=("Arial", 12, "bold"),
                                         width=22, height=2,
                                         command=lambda: self._cambiar_modo_cinta(True))
        self.btn_auto_cinta.grid(row=0, column=1, padx=5, pady=5)

        # Orientaciones manuales
        c_orient = tk.Frame(f_cinta, bg='#2b2b2b')
        c_orient.pack(pady=(8, 0))
        for i, op in enumerate(("ARRIBA", "ABAJO", "VACIO")):
            btn = tk.Button(c_orient, text=op,
                            font=("Arial", 13, "bold"),
                            width=12, height=2,
                            command=lambda o=op: self._seleccionar_orientacion(o))
            btn.grid(row=0, column=i, padx=8, pady=5)
            self.btns_orient.append(btn)

        # =========== Seccion MESA ===========
        f_mesa = tk.LabelFrame(col_izq, text=" MESA (Estacion 3) ",
                                font=("Arial", 12, "bold"),
                                bg='#2b2b2b', fg='#ffd47f',
                                padx=10, pady=10)
        f_mesa.pack(fill='x', pady=8)

        c_modo_m = tk.Frame(f_mesa, bg='#2b2b2b')
        c_modo_m.pack(fill='x')

        self.btn_manual_mesa = tk.Button(c_modo_m, text="MANUAL",
                                          font=("Arial", 12, "bold"),
                                          width=14, height=2,
                                          command=lambda: self._cambiar_modo_mesa(False))
        self.btn_manual_mesa.grid(row=0, column=0, padx=5, pady=5)

        self.btn_auto_mesa = tk.Button(c_modo_m, text="AUTOMATICO (camara)",
                                        font=("Arial", 12, "bold"),
                                        width=22, height=2,
                                        command=lambda: self._cambiar_modo_mesa(True))
        self.btn_auto_mesa.grid(row=0, column=1, padx=5, pady=5)

        # Spinboxes para fila/columna manual
        c_man = tk.Frame(f_mesa, bg='#2b2b2b')
        c_man.pack(pady=(8, 0))
        tk.Label(c_man, text="Fila manual:", bg='#2b2b2b', fg='white',
                 font=("Arial", 10)).grid(row=0, column=0, padx=5)
        self.spn_fila = tk.Spinbox(c_man, from_=1, to=GRILLA_FILAS,
                                    width=5, font=("Arial", 12),
                                    command=self._cambio_mesa_manual)
        self.spn_fila.grid(row=0, column=1, padx=5)
        tk.Label(c_man, text="Columna manual:", bg='#2b2b2b', fg='white',
                 font=("Arial", 10)).grid(row=0, column=2, padx=15)
        self.spn_col = tk.Spinbox(c_man, from_=1, to=GRILLA_COLS,
                                   width=5, font=("Arial", 12),
                                   command=self._cambio_mesa_manual)
        self.spn_col.grid(row=0, column=3, padx=5)

        # Estado y conexion
        f_estado = tk.LabelFrame(col_izq, text=" ESTADO ",
                                 font=("Arial", 11, "bold"),
                                 bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_estado.pack(fill='x', pady=5)

        self.lbl_seleccion = tk.Label(f_estado,
                                      text="Tipo: 1",
                                      font=("Arial", 12),
                                      bg='#2b2b2b', fg='#7fffd4')
        self.lbl_seleccion.pack(anchor='w')

        self.lbl_camara = tk.Label(f_estado, text="Camara: apagada",
                                    font=("Arial", 10),
                                    bg='#2b2b2b', fg='#ff6b6b')
        self.lbl_camara.pack(anchor='w', pady=(3, 0))

        self.lbl_robot = tk.Label(f_estado, text="Robot: desconectado",
                                  font=("Arial", 11),
                                  bg='#2b2b2b', fg='#ff6b6b')
        self.lbl_robot.pack(anchor='w', pady=(5, 0))

        self.lbl_consultas = tk.Label(f_estado,
                                       text="Consultas recibidas: 0",
                                       font=("Arial", 11),
                                       bg='#2b2b2b', fg='white')
        self.lbl_consultas.pack(anchor='w', pady=(5, 0))

        # =========== Columna derecha: paneles ===========

        # Panel cinta
        f_img_cinta = tk.LabelFrame(col_der, text=" ULTIMA DETECCION CINTA ",
                                     font=("Arial", 11, "bold"),
                                     bg='#2b2b2b', fg='#7fffd4',
                                     padx=5, pady=5)
        f_img_cinta.pack(fill='x', pady=(0, 10))

        frame_canvas_op = tk.Frame(f_img_cinta, bg='#1a1a1a',
                                    width=PANEL_W, height=PANEL_H)
        frame_canvas_op.pack(padx=5, pady=5)
        frame_canvas_op.pack_propagate(False)

        self.canvas_cinta = tk.Label(frame_canvas_op,
                                      bg='#1a1a1a', fg='#888888',
                                      text="(sin detecciones aun)",
                                      font=("Arial", 11))
        self.canvas_cinta.pack(fill='both', expand=True)

        self.lbl_deteccion_cinta = tk.Label(f_img_cinta,
                                             text="Resultado: ---",
                                             font=("Arial", 11, "bold"),
                                             bg='#2b2b2b', fg='white')
        self.lbl_deteccion_cinta.pack(anchor='w', padx=5, pady=(5, 0))

        self.lbl_ratio_cinta = tk.Label(f_img_cinta, text="Ratio: ---",
                                         font=("Arial", 10),
                                         bg='#2b2b2b', fg='#cfcfcf')
        self.lbl_ratio_cinta.pack(anchor='w', padx=5, pady=(0, 5))

        # Panel mesa con visualizacion de matriz
        f_img_mesa = tk.LabelFrame(col_der, text=" MATRIZ MESA ",
                                    font=("Arial", 11, "bold"),
                                    bg='#2b2b2b', fg='#ffd47f',
                                    padx=5, pady=5)
        f_img_mesa.pack(fill='x', pady=(0, 5))

        frame_canvas_m = tk.Frame(f_img_mesa, bg='#1a1a1a',
                                   width=MATRIZ_W, height=MATRIZ_H)
        frame_canvas_m.pack(padx=5, pady=5)
        frame_canvas_m.pack_propagate(False)

        # Matriz dibujada con canvas (no imagen): mejor para ver en vivo
        self.canvas_matriz = tk.Canvas(frame_canvas_m,
                                        bg='#1a1a1a', highlightthickness=0)
        self.canvas_matriz.pack(fill='both', expand=True)

        self.lbl_deteccion_mesa = tk.Label(f_img_mesa,
                                            text="Elegida: ---",
                                            font=("Arial", 11, "bold"),
                                            bg='#2b2b2b', fg='white')
        self.lbl_deteccion_mesa.pack(anchor='w', padx=5, pady=(5, 0))

        self.lbl_piezas_mesa = tk.Label(f_img_mesa, text="Piezas: ---",
                                         font=("Arial", 10),
                                         bg='#2b2b2b', fg='#cfcfcf')
        self.lbl_piezas_mesa.pack(anchor='w', padx=5, pady=(0, 5))

        # Inicializar matriz vacia
        self._dibujar_matriz(matriz_vacia(), 0, 0)

    def _dibujar_matriz(self, matriz, fila_elegida, columna_elegida):
        """Dibuja la matriz 10x8 en el canvas. matriz puede ser None.
        fila_elegida, columna_elegida son 1-indexed."""
        canvas = self.canvas_matriz
        canvas.delete("all")

        # Esperar a que el canvas tenga tamanio (primer dibujado)
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w <= 1 or h <= 1:
            # Usar valores por defecto
            w = MATRIZ_W
            h = MATRIZ_H

        cell_w = w / GRILLA_COLS
        cell_h = h / GRILLA_FILAS

        for fi in range(GRILLA_FILAS):
            for ci in range(GRILLA_COLS):
                x1 = ci * cell_w
                y1 = fi * cell_h
                x2 = (ci + 1) * cell_w
                y2 = (fi + 1) * cell_h

                ocupada = (matriz is not None and matriz[fi][ci])
                elegida = (fi + 1 == fila_elegida and ci + 1 == columna_elegida)

                if ocupada and elegida:
                    fill = '#FAC775'  # amarillo
                    outline = '#FAC775'
                elif ocupada:
                    fill = '#1D9E75'  # verde
                    outline = '#0F6E56'
                else:
                    fill = '#3a3a3a'
                    outline = '#2a2a2a'

                canvas.create_rectangle(x1, y1, x2, y2,
                                         fill=fill, outline=outline,
                                         width=1)

                # Texto opcional dentro de la celda
                if ocupada:
                    canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2,
                                        text="★" if elegida else "●",
                                        fill="white",
                                        font=("Arial", 10, "bold"))

    # ------------------------------------------------------------------
    #  CALIBRAR
    # ------------------------------------------------------------------
    def _construir_tab_calibrar(self):
        cont = tk.Frame(self.tab_calibrar, bg='#2b2b2b')
        cont.pack(fill='both', expand=True, padx=15, pady=10)

        # Banner
        f_banner = tk.Frame(cont, bg='#1a1a1a', height=60)
        f_banner.pack(fill='x', pady=(0, 10))
        f_banner.pack_propagate(False)

        self.lbl_banner_calibrar = tk.Label(
            f_banner, text="CALIBRANDO: TIPO 1 (CINTA)",
            font=("Arial", 18, "bold"),
            bg='#1a1a1a', fg='#7fffd4')
        self.lbl_banner_calibrar.pack(side='left', padx=20, pady=10)

        self.lbl_estado_calibrar = tk.Label(
            f_banner, text="",
            font=("Arial", 11),
            bg='#1a1a1a', fg='#cfcfcf')
        self.lbl_estado_calibrar.pack(side='right', padx=20)

        # Toggle Cinta / Mesa
        f_toggle = tk.LabelFrame(cont, text=" ESTACION ",
                                  font=("Arial", 11, "bold"),
                                  bg='#2b2b2b', fg='white',
                                  padx=10, pady=8)
        f_toggle.pack(fill='x', pady=(0, 8))

        c_tog = tk.Frame(f_toggle, bg='#2b2b2b')
        c_tog.pack()
        self.btn_estac_cinta = tk.Button(c_tog, text="CINTA",
                                          font=("Arial", 12, "bold"),
                                          width=14, height=2,
                                          command=lambda: self._set_estacion_calibrar("cinta"))
        self.btn_estac_cinta.grid(row=0, column=0, padx=5)
        self.btn_estac_mesa = tk.Button(c_tog, text="MESA",
                                         font=("Arial", 12, "bold"),
                                         width=14, height=2,
                                         command=lambda: self._set_estacion_calibrar("mesa"))
        self.btn_estac_mesa.grid(row=0, column=1, padx=5)

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

        # Cuerpo
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

        # Preview + referencia
        f_visual = tk.Frame(col_izq, bg='#2b2b2b')
        f_visual.pack(fill='both', expand=True)
        f_visual.columnconfigure(0, weight=1, uniform='cols')
        f_visual.columnconfigure(1, weight=1, uniform='cols')
        f_visual.rowconfigure(0, weight=1)

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
            text="(preview apagado)",
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

        self.lbl_ref = tk.Label(f_est, text="referencia: ?",
                                font=("Consolas", 10),
                                bg='#2b2b2b', fg='white',
                                justify='left', anchor='w', wraplength=240)
        self.lbl_ref.pack(anchor='w')

        self.lbl_cfg = tk.Label(f_est, text="config: ?",
                                font=("Consolas", 10),
                                bg='#2b2b2b', fg='white',
                                justify='left', anchor='w', wraplength=240)
        self.lbl_cfg.pack(anchor='w', pady=(3, 0))

        # Sliders
        f_sld = tk.LabelFrame(col_der, text=" PARAMETROS ",
                              font=("Arial", 11, "bold"),
                              bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_sld.pack(fill='x', pady=5)

        tk.Label(f_sld, text="Exposicion:", font=("Arial", 10),
                 bg='#2b2b2b', fg='white').pack(anchor='w')
        self.scl_exp = tk.Scale(f_sld, from_=1, to=200,
                                orient='horizontal', length=220,
                                bg='#2b2b2b', fg='white',
                                troughcolor='#1a1a1a',
                                highlightthickness=0,
                                command=self._on_exposure_change)
        self.scl_exp.pack(fill='x')
        self.scl_exp.bind('<ButtonRelease-1>', self._persistir_sliders)

        tk.Label(f_sld, text="Ganancia:", font=("Arial", 10),
                 bg='#2b2b2b', fg='white').pack(anchor='w', pady=(5, 0))
        self.scl_gain = tk.Scale(f_sld, from_=0, to=128,
                                 orient='horizontal', length=220,
                                 bg='#2b2b2b', fg='white',
                                 troughcolor='#1a1a1a',
                                 highlightthickness=0,
                                 command=self._on_gain_change)
        self.scl_gain.pack(fill='x')
        self.scl_gain.bind('<ButtonRelease-1>', self._persistir_sliders)

        tk.Label(f_sld, text="Threshold BG Subtract:", font=("Arial", 10),
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
        for i, btn in enumerate(self.btns_tipo_operar):
            self._refrescar_boton_tipo(btn, i + 1,
                                         seleccionado=(i + 1 == tipo),
                                         estacion="cinta")
        self._actualizar_label_seleccion()

    def _seleccionar_orientacion(self, orient):
        estado.set_orientacion_manual(orient)
        colores = {"ARRIBA": "#2d8f3a", "ABAJO": "#cc7a00",
                   "VACIO": "#777777"}
        for btn in self.btns_orient:
            if btn['text'] == orient:
                btn.configure(bg=colores[orient], fg='white',
                              relief='sunken')
            else:
                btn.configure(bg='SystemButtonFace', fg='black',
                              relief='raised')
        self._actualizar_label_seleccion()

    def _cambiar_modo_cinta(self, auto):
        if auto:
            if not self._asegurar_camara():
                return
            estado.set_modo_auto_cinta(True)
            self.btn_auto_cinta.configure(bg='#2d8f3a', fg='white',
                                           relief='sunken')
            self.btn_manual_cinta.configure(bg='SystemButtonFace', fg='black',
                                             relief='raised')
            for btn in self.btns_orient:
                btn.configure(state='disabled',
                              bg='#3a3a3a', fg='#777777',
                              relief='flat',
                              disabledforeground='#777777')
        else:
            estado.set_modo_auto_cinta(False)
            self.btn_manual_cinta.configure(bg='#4a90e2', fg='white',
                                             relief='sunken')
            self.btn_auto_cinta.configure(bg='SystemButtonFace', fg='black',
                                           relief='raised')
            for btn in self.btns_orient:
                btn.configure(state='normal')
            est = estado.get_estado()
            self._seleccionar_orientacion(est["orientacion_manual"])

        self._actualizar_label_seleccion()

    def _cambiar_modo_mesa(self, auto):
        if auto:
            if not self._asegurar_camara():
                return
            estado.set_modo_auto_mesa(True)
            self.btn_auto_mesa.configure(bg='#2d8f3a', fg='white',
                                          relief='sunken')
            self.btn_manual_mesa.configure(bg='SystemButtonFace', fg='black',
                                            relief='raised')
            self.spn_fila.configure(state='disabled')
            self.spn_col.configure(state='disabled')
        else:
            estado.set_modo_auto_mesa(False)
            self.btn_manual_mesa.configure(bg='#4a90e2', fg='white',
                                            relief='sunken')
            self.btn_auto_mesa.configure(bg='SystemButtonFace', fg='black',
                                          relief='raised')
            self.spn_fila.configure(state='normal')
            self.spn_col.configure(state='normal')

        self._actualizar_label_seleccion()

    def _asegurar_camara(self):
        """Si la camara no esta activa, intenta encenderla. True si quedo
        encendida."""
        if detector.esta_activo():
            return True

        est = estado.get_estado()
        self._agregar_log("[VISION] Iniciando camara...")
        self.root.update_idletasks()
        try:
            detector.start(requiere_referencia=False)
            self._agregar_log("[VISION] Camara lista.")
            return True
        except Exception as e:
            self._agregar_log(f"!!! Error iniciando camara: {e}")
            messagebox.showerror("Camara",
                                 f"No se pudo iniciar la camara:\n\n{e}")
            return False

    def _cambio_mesa_manual(self):
        try:
            fila = int(self.spn_fila.get())
            col = int(self.spn_col.get())
        except ValueError:
            return
        estado.set_mesa_manual(fila, col)

    def _actualizar_label_seleccion(self):
        est = estado.get_estado()
        cinta_cal = "OK" if tiene_referencia(est["tipo"], "cinta") else "sin cal"
        mesa_cal = "OK" if tiene_referencia(est["tipo"], "mesa") else "sin cal"

        cinta_modo = "AUTO" if est["modo_auto_cinta"] else f"manual:{est['orientacion_manual']}"
        mesa_modo = "AUTO" if est["modo_auto_mesa"] else f"manual:({est['mesa_manual_fila']},{est['mesa_manual_columna']})"

        self.lbl_seleccion.configure(
            text=f"Tipo: {est['tipo']}   |   "
                 f"Cinta ({cinta_cal}, {cinta_modo})   |   "
                 f"Mesa ({mesa_cal}, {mesa_modo})"
        )

    # ==================================================================
    #  Handlers CALIBRAR
    # ==================================================================
    def _set_estacion_calibrar(self, estacion):
        venia_preview = self._preview_activo
        if venia_preview:
            self._detener_preview()

        self._estacion_calibrar = estacion

        # Refrescar boton activo
        if estacion == "cinta":
            self.btn_estac_cinta.configure(bg='#4a90e2', fg='white',
                                            relief='sunken')
            self.btn_estac_mesa.configure(bg='SystemButtonFace', fg='black',
                                           relief='raised')
        else:
            self.btn_estac_mesa.configure(bg='#ffd47f', fg='black',
                                           relief='sunken')
            self.btn_estac_cinta.configure(bg='SystemButtonFace', fg='black',
                                            relief='raised')

        # Recargar tipo en la nueva estacion
        self._seleccionar_tipo_calibrar(self._tipo_calibrar)

        if venia_preview:
            self._iniciar_preview()

    def _seleccionar_tipo_calibrar(self, tipo):
        venia_preview = self._preview_activo
        if venia_preview:
            self._detener_preview()

        self._tipo_calibrar = tipo
        est = self._estacion_calibrar

        try:
            detector.usa_tipo(tipo, est)
        except Exception as e:
            self._agregar_log(f"!!! Error cargando Tipo {tipo} ({est}): {e}")

        for i, btn in enumerate(self.btns_tipo_calibrar):
            self._refrescar_boton_tipo(btn, i + 1,
                                         seleccionado=(i + 1 == tipo),
                                         estacion=est)

        # Banner
        if tiene_referencia(tipo, est):
            self.lbl_banner_calibrar.configure(
                text=f"CALIBRANDO: TIPO {tipo} ({est.upper()})",
                fg='#7fffd4' if est == "cinta" else '#ffd47f')
            self.lbl_estado_calibrar.configure(text="(referencia OK)",
                                                 fg='#7fff7f')
        else:
            self.lbl_banner_calibrar.configure(
                text=f"CALIBRANDO: TIPO {tipo} ({est.upper()})",
                fg='#ffaa55')
            self.lbl_estado_calibrar.configure(
                text="(falta capturar referencia)", fg='#ff6b6b')

        try:
            self.scl_exp.set(detector.exposure)
            self.scl_gain.set(detector.gain)
            self.scl_thr.set(detector.bg_diff_thresh)
        except Exception:
            pass

        self._refrescar_estado_archivos()
        self._mostrar_referencia_calibrar()
        self._agregar_log(f"[CALIBRAR] Tipo {tipo} ({est}) activo")

        if venia_preview:
            self._iniciar_preview()

    def _toggle_preview(self):
        if self._preview_activo:
            self._detener_preview()
        else:
            self._iniciar_preview()

    def _iniciar_preview(self):
        if not self._asegurar_camara():
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
        tipo = self._tipo_calibrar
        est = self._estacion_calibrar
        ref_path = path_referencia(tipo, est)

        if not os.path.exists(ref_path):
            self.canvas_referencia.configure(
                image='',
                text=f"(Tipo {tipo} {est} sin referencia)",
                fg='#ff6b6b')
            self._img_referencia = None
            self.lbl_ref_info.configure(text="")
            return

        try:
            from spirax_vision import cargar_config

            ref = cv2.imread(ref_path)
            if ref is None:
                raise RuntimeError("No se pudo leer la imagen")

            cfg = cargar_config(tipo, est)
            h, w = ref.shape[:2]
            y_top = int(h * cfg.get("corte_y_top_pct", 0.0))
            y_bot = int(h * cfg.get("corte_y_pct", 1.0))
            x_izq = int(w * cfg.get("corte_x_izq_pct", 0.0))
            x_der = int(w * cfg.get("corte_x_der_pct", 1.0))

            debug = ref.copy()
            overlay = debug.copy()
            cv2.rectangle(overlay, (0, 0), (w, y_top), (0, 0, 0), -1)
            cv2.rectangle(overlay, (0, y_bot), (w, h), (0, 0, 0), -1)
            cv2.rectangle(overlay, (0, y_top), (x_izq, y_bot), (0, 0, 0), -1)
            cv2.rectangle(overlay, (x_der, y_top), (w, y_bot), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, debug, 0.5, 0, debug)

            cv2.line(debug, (0, y_top), (w, y_top), (0, 255, 255), 2)
            cv2.line(debug, (0, y_bot), (w, y_bot), (0, 255, 255), 2)
            cv2.line(debug, (x_izq, 0), (x_izq, h), (0, 255, 255), 2)
            cv2.line(debug, (x_der, 0), (x_der, h), (0, 255, 255), 2)

            # Si es mesa, dibujar tambien la grilla 10x8 sobre el area recortada
            if est == "mesa":
                roi_h = y_bot - y_top
                roi_w = x_der - x_izq
                if roi_h > 0 and roi_w > 0:
                    for fi in range(1, GRILLA_FILAS):
                        y = y_top + int(round(fi * roi_h / GRILLA_FILAS))
                        cv2.line(debug, (x_izq, y), (x_der, y),
                                 (50, 200, 255), 1)
                    for ci in range(1, GRILLA_COLS):
                        x = x_izq + int(round(ci * roi_w / GRILLA_COLS))
                        cv2.line(debug, (x, y_top), (x, y_bot),
                                 (50, 200, 255), 1)

            rgb = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            img.thumbnail((PANEL_W, PANEL_H), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_referencia.configure(image=photo, text='')
            self._img_referencia = photo

            ts = datetime.fromtimestamp(os.path.getmtime(ref_path))
            extra = f"  |  grilla {GRILLA_COLS}x{GRILLA_FILAS}" if est == "mesa" else ""
            self.lbl_ref_info.configure(
                text=(f"Tipo {tipo} ({est}) - {ts.strftime('%Y-%m-%d %H:%M')}"
                      f"  |  Y {y_top}-{y_bot}  X {x_izq}-{x_der}{extra}"))
        except Exception as e:
            self.canvas_referencia.configure(
                image='',
                text=f"(error cargando)\n{e}",
                fg='#ff6b6b')
            self._img_referencia = None
            self.lbl_ref_info.configure(text="")

    def _capturar_referencia(self):
        tipo = self._tipo_calibrar
        est = self._estacion_calibrar

        if not self._asegurar_camara():
            return

        respuesta = messagebox.askyesno(
            "Capturar referencia",
            f"Capturar referencia para TIPO {tipo} ({est.upper()}).\n\n"
            f"Asegurate de que NO hay piezas en el setup.\n\n"
            f"Continuar?"
        )
        if not respuesta:
            return

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
            ref_path = path_referencia(tipo, est)
            cv2.imwrite(ref_path, frame)
            detector.recargar_referencia()
            self._agregar_log(f"[CALIBRAR] Referencia Tipo {tipo} ({est}) guardada")
            messagebox.showinfo("Referencia", f"Guardada como {ref_path}")
            self._refrescar_estado_archivos()

            for i, btn in enumerate(self.btns_tipo_calibrar):
                self._refrescar_boton_tipo(btn, i + 1,
                                             seleccionado=(i + 1 == tipo),
                                             estacion=est)
            # Tambien refrescar operar (cinta)
            for i, btn in enumerate(self.btns_tipo_operar):
                e2 = estado.get_estado()
                self._refrescar_boton_tipo(btn, i + 1,
                                             seleccionado=(i + 1 == e2["tipo"]),
                                             estacion="cinta")

            self.lbl_banner_calibrar.configure(
                text=f"CALIBRANDO: TIPO {tipo} ({est.upper()})",
                fg='#7fffd4' if est == "cinta" else '#ffd47f')
            self.lbl_estado_calibrar.configure(text="(referencia OK)",
                                                 fg='#7fff7f')
            self._mostrar_referencia_calibrar()
        except Exception as e:
            self._agregar_log(f"!!! Error guardando referencia: {e}")
            messagebox.showerror("Error", str(e))

    def _ajustar_recorte(self):
        tipo = self._tipo_calibrar
        est = self._estacion_calibrar
        ref_path = path_referencia(tipo, est)

        if os.path.exists(ref_path):
            imagen = cv2.imread(ref_path)
            self._agregar_log(f"[CALIBRAR] Ajustando recorte Tipo {tipo} ({est})")
        else:
            if not self._asegurar_camara():
                return
            try:
                imagen = detector.capturar_frame()
            except Exception as e:
                messagebox.showerror("Captura", f"No se pudo capturar:\n\n{e}")
                return

        venia_preview = self._preview_activo
        if venia_preview:
            self._detener_preview()

        try:
            ok = modo_recorte(imagen, tipo, est)
            if ok:
                if os.path.exists(ref_path):
                    detector.recargar_referencia()
                self._agregar_log(f"[CALIBRAR] Recorte Tipo {tipo} ({est}) guardado")
                self._refrescar_estado_archivos()
                self._mostrar_referencia_calibrar()
            else:
                self._agregar_log("[CALIBRAR] Recorte cancelado")
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
        try:
            detector.set_exposure(detector.exposure, persist=True)
            self._agregar_log(
                f"[CONFIG] Tipo {self._tipo_calibrar} ({self._estacion_calibrar}): "
                f"exp={detector.exposure} gain={detector.gain} "
                f"thr={detector.bg_diff_thresh}"
            )
        except Exception as e:
            self._agregar_log(f"!!! persistiendo: {e}")

    def _refrescar_estado_archivos(self):
        tipo = self._tipo_calibrar
        est = self._estacion_calibrar
        ref_path = path_referencia(tipo, est)
        cfg_path = path_config(tipo, est)

        if os.path.exists(ref_path):
            ts = datetime.fromtimestamp(os.path.getmtime(ref_path))
            self.lbl_ref.configure(
                text=f"referencia OK\n  {ts.strftime('%Y-%m-%d %H:%M')}",
                fg='#7fff7f')
        else:
            self.lbl_ref.configure(text="referencia FALTA", fg='#ff6b6b')
        if os.path.exists(cfg_path):
            self.lbl_cfg.configure(text="config OK", fg='#7fff7f')
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
        if tab == 'CALIBRAR':
            # Refrescar el toggle de estacion activa
            self._set_estacion_calibrar(self._estacion_calibrar)

    # ==================================================================
    #  Log
    # ==================================================================
    def _agregar_log(self, mensaje):
        ahora = datetime.now().strftime('%H:%M:%S')
        self.txt_log.insert('end', f"[{ahora}] {mensaje}\n")
        self.txt_log.see('end')

    def _log_thread_safe(self, mensaje):
        try:
            self.root.after(0, self._agregar_log, mensaje)
        except RuntimeError:
            pass

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

        # Refrescar paneles
        with estado.lock:
            panel_c = estado.ultimo_panel_cinta
            orient = estado.ultima_deteccion_cinta
            ratio = estado.ultimo_ratio_cinta
            matriz = estado.ultima_matriz_mesa
            fila_m = estado.ultima_fila_mesa
            col_m = estado.ultima_columna_mesa
            error = estado.ultimo_error

        if panel_c is not None:
            self._mostrar_panel_cinta(panel_c)
        elif error and orient is None:
            self.canvas_cinta.configure(text=f"Error:\n{error}",
                                         image='', fg='#ff6b6b')
            self._img_cinta = None

        if orient is not None:
            colores = {"ARRIBA": "#7fff7f", "ABAJO": "#ffa500",
                       "VACIO": "#cccccc", "DUDOSO": "#ffff7f",
                       "N/A": "#ff6b6b"}
            self.lbl_deteccion_cinta.configure(
                text=f"Resultado: {orient}",
                fg=colores.get(orient, 'white'))
            self.lbl_ratio_cinta.configure(
                text=f"Ratio: {ratio:.3f}" if ratio is not None else "Ratio: ---")

        # Refrescar matriz
        self._dibujar_matriz(matriz if matriz is not None else matriz_vacia(),
                              fila_m, col_m)
        if matriz is not None:
            n_piezas = sum(sum(1 for c in f if c) for f in matriz)
            self.lbl_piezas_mesa.configure(text=f"Piezas: {n_piezas}")
        else:
            self.lbl_piezas_mesa.configure(text="Piezas: ---")

        if fila_m > 0 and col_m > 0:
            self.lbl_deteccion_mesa.configure(
                text=f"Elegida: ({fila_m}, {col_m})", fg='#FAC775')
        else:
            self.lbl_deteccion_mesa.configure(text="Elegida: ---",
                                                fg='#cccccc')

        self.root.after(500, self._actualizar_estado_loop)

    def _mostrar_panel_cinta(self, panel_bgr):
        try:
            rgb = cv2.cvtColor(panel_bgr, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            img.thumbnail((PANEL_W, PANEL_H), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_cinta.configure(image=photo, text='')
            self._img_cinta = photo
        except Exception as e:
            self._agregar_log(f"!!! Error mostrando panel cinta: {e}")

    # ==================================================================
    #  Cierre
    # ==================================================================
    def _on_close(self):
        estado.shutdown = True
        self._preview_activo = False
        try:
            if detector.esta_activo():
                detector.stop()
        except Exception:
            pass
        time.sleep(0.2)
        self.root.destroy()


def main():
    root = tk.Tk()
    HMISpirax(root)
    root.mainloop()


if __name__ == '__main__':
    main()
