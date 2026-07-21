"""
hmi.py - HMI Spirax con soporte para Cinta + Mesa

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

from vision import (
    DetectorOrientacion,
    TIPOS_VALIDOS,
    ESTACIONES_VALIDAS,
    tiene_referencia,
    path_referencia,
    path_config,
    modo_recorte,
    cargar_rotacion,
    guardar_rotacion,
    cargar_flip_h,
    guardar_flip_h,
    cargar_espejo_col_salida,
    guardar_espejo_col_salida,
)
from mesa import (
    GRILLA_FILAS, GRILLA_COLS,
    matriz_vacia, matriz_con_celda,
)
from torno import TornoFanuc, FANUC_DISPONIBLE

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
        self.orientacion_manual = None  # None = manual sin seleccion
        self.modo_auto_cinta = False

        # Configuracion mesa
        self.modo_auto_mesa = False
        self.mesa_manual_fila = 0       # 0 = manual sin seleccion
        self.mesa_manual_columna = 0
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

        # Flags para pedir popups desde el server thread al HMI thread
        self.pedir_popup_cinta = False
        self.pedir_popup_mesa = False

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

# --- Detectores de VISION ---------------------------------------------
# Un detector por estacion, cada uno casado con su camara fisica (serial
# definido en vision.SERIAL_POR_ESTACION). Ambos se usan en modo AUTOMATICO
# y pueden estar abiertos a la vez (cada uno abre su propia camara).
detector_cinta = DetectorOrientacion(tipo_inicial=1, estacion_inicial="cinta")
detector_mesa = DetectorOrientacion(tipo_inicial=1, estacion_inicial="mesa")

# 'detector' es el detector ACTIVO para la pestania Calibrar/preview.
# Apunta a detector_cinta o detector_mesa segun la estacion que se este
# calibrando. Como Automatico y Calibrar NO se usan a la vez, reusar el
# mismo objeto no genera conflicto de camara.
detector = detector_cinta

torno = TornoFanuc()  # se inicia desde la HMI con start()


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
                    # Auto: siempre captura con camara
                    orientacion = consultar_vision_cinta(tipo, log_callback)
                else:
                    # Manual: si hay orientacion seteada, usar esa;
                    # si no, pedir al usuario que setee algo (popup)
                    if est["orientacion_manual"]:
                        orientacion = est["orientacion_manual"]
                    else:
                        log_callback("!!! Cinta MANUAL sin seleccion -> popup")
                        estado.pedir_popup_cinta = True
                        orientacion = "VACIO"

                # ENCOLAR AL TORNO segun el resultado de la cinta:
                #   VACIO  -> KUKA carga pieza nueva -> torno hara op10
                #   ARRIBA -> KUKA da vuelta pieza   -> torno hara op20
                #   ABAJO  -> error, no encolar
                if orientacion == "VACIO":
                    torno.encolar(tipo, 10)
                elif orientacion == "ARRIBA":
                    torno.encolar(tipo, 20)
                # ABAJO no encola (es error)

            elif num_estacion == 3:
                # Mesa: devolver Tipo + Fila + Columna
                if est["modo_auto_mesa"]:
                    # Auto: siempre captura con camara
                    fila, columna = consultar_vision_mesa(tipo, log_callback)
                else:
                    # Manual: si hay posicion seteada (>0), usarla;
                    # si no, pedir popup
                    if est["mesa_manual_fila"] > 0 and est["mesa_manual_columna"] > 0:
                        fila = est["mesa_manual_fila"]
                        columna = est["mesa_manual_columna"]
                        # Marcar el panel
                        estado.set_deteccion_mesa(
                            matriz_con_celda(fila, columna),
                            fila, columna, None,
                        )
                    else:
                        log_callback("!!! Mesa MANUAL sin seleccion -> popup")
                        estado.pedir_popup_mesa = True
                        fila = 0
                        columna = 0

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
    detector = detector_cinta  # detector casado con la camara de la cinta
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
    detector = detector_mesa  # detector casado con la camara de la mesa
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
        self._img_bg_mesa = None

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
        self.tab_torno = tk.Frame(self.nb, bg='#2b2b2b')
        self.tab_log = tk.Frame(self.nb, bg='#2b2b2b')

        self.nb.add(self.tab_operar, text='  OPERAR  ')
        self.nb.add(self.tab_calibrar, text='  CALIBRAR  ')
        self.nb.add(self.tab_torno, text='  TORNO  ')
        self.nb.add(self.tab_log, text='  LOG  ')

        self.nb.bind('<<NotebookTabChanged>>', self._on_tab_changed)

        self._construir_tab_operar()
        self._construir_tab_calibrar()
        self._construir_tab_torno()
        self._construir_tab_log()

        self._seleccionar_tipo_operar(1)
        # No setear orientacion: arranca en MANUAL sin seleccion
        self._refrescar_botones_orientacion()
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

        # Descripcion modo manual
        tk.Label(f_cinta,
                 text="Modo MANUAL: forzar una orientacion (tildá Activar y elegí)",
                 font=("Arial", 9, "italic"),
                 bg='#2b2b2b', fg='#a0a0a0').pack(anchor='w', pady=(8, 2))

        # Checkbox de activacion + orientaciones manuales
        c_orient_row = tk.Frame(f_cinta, bg='#2b2b2b')
        c_orient_row.pack(pady=(0, 0))

        self.var_cinta_manual_activo = tk.BooleanVar(value=False)
        self.chk_cinta_manual = tk.Checkbutton(
            c_orient_row, text="Activar",
            variable=self.var_cinta_manual_activo,
            command=self._toggle_cinta_manual,
            font=("Arial", 10),
            bg='#2b2b2b', fg='white',
            selectcolor='#1a1a1a',
            activebackground='#2b2b2b',
            activeforeground='white')
        self.chk_cinta_manual.grid(row=0, column=0, padx=(0, 10))

        c_orient = tk.Frame(c_orient_row, bg='#2b2b2b')
        c_orient.grid(row=0, column=1)
        for i, op in enumerate(("ARRIBA", "ABAJO", "VACIO")):
            btn = tk.Button(c_orient, text=op,
                            font=("Arial", 13, "bold"),
                            width=12, height=2,
                            command=lambda o=op: self._toggle_orientacion(o))
            btn.grid(row=0, column=i, padx=8, pady=5)
            self.btns_orient.append(btn)

        # Descripcion test
        tk.Label(f_cinta,
                 text="CAPTURAR AHORA: tomar foto y analizar (sin afectar al robot)",
                 font=("Arial", 9, "italic"),
                 bg='#2b2b2b', fg='#a0a0a0').pack(anchor='w', pady=(8, 2))

        # Boton de test instantaneo de cinta
        self.btn_capturar_test_cinta = tk.Button(
            f_cinta, text="CAPTURAR AHORA (test)",
            font=("Arial", 11, "bold"),
            bg='#4a90e2', fg='white',
            height=2,
            command=self._capturar_test_cinta)
        self.btn_capturar_test_cinta.pack(fill='x', pady=(0, 0))

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

        # Descripcion modo manual mesa
        tk.Label(f_mesa,
                 text="Modo MANUAL: forzar una posicion (activar tilde para usar fila/columna fija)",
                 font=("Arial", 9, "italic"),
                 bg='#2b2b2b', fg='#a0a0a0').pack(anchor='w', pady=(8, 2))

        # Spinboxes para fila/columna manual con checkbox para activar
        c_man = tk.Frame(f_mesa, bg='#2b2b2b')
        c_man.pack(pady=(0, 0))

        self.var_mesa_manual_activo = tk.BooleanVar(value=False)
        self.chk_mesa_manual = tk.Checkbutton(
            c_man, text="Activar",
            variable=self.var_mesa_manual_activo,
            command=self._toggle_mesa_manual,
            font=("Arial", 10),
            bg='#2b2b2b', fg='white',
            selectcolor='#1a1a1a',
            activebackground='#2b2b2b',
            activeforeground='white')
        self.chk_mesa_manual.grid(row=0, column=0, padx=5)

        tk.Label(c_man, text="Fila:", bg='#2b2b2b', fg='white',
                 font=("Arial", 10)).grid(row=0, column=1, padx=5)
        self.spn_fila = tk.Spinbox(c_man, from_=1, to=GRILLA_FILAS,
                                    width=5, font=("Arial", 12),
                                    command=self._cambio_mesa_manual)
        self.spn_fila.grid(row=0, column=2, padx=5)
        tk.Label(c_man, text="Columna:", bg='#2b2b2b', fg='white',
                 font=("Arial", 10)).grid(row=0, column=3, padx=15)
        self.spn_col = tk.Spinbox(c_man, from_=1, to=GRILLA_COLS,
                                   width=5, font=("Arial", 12),
                                   command=self._cambio_mesa_manual)
        self.spn_col.grid(row=0, column=4, padx=5)

        # Descripcion test mesa
        tk.Label(f_mesa,
                 text="CAPTURAR AHORA: tomar foto y analizar matriz (sin afectar al robot)",
                 font=("Arial", 9, "italic"),
                 bg='#2b2b2b', fg='#a0a0a0').pack(anchor='w', pady=(8, 2))

        # Boton de test instantaneo de mesa
        self.btn_capturar_test_mesa = tk.Button(
            f_mesa, text="CAPTURAR AHORA (test)",
            font=("Arial", 11, "bold"),
            bg='#ffd47f', fg='black',
            height=2,
            command=self._capturar_test_mesa)
        self.btn_capturar_test_mesa.pack(fill='x', pady=(0, 0))

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

        self.lbl_cam_cinta = tk.Label(f_estado, text="Camara cinta: apagada",
                                    font=("Arial", 10),
                                    bg='#2b2b2b', fg='#ff6b6b')
        self.lbl_cam_cinta.pack(anchor='w', pady=(3, 0))

        self.lbl_cam_mesa = tk.Label(f_estado, text="Camara mesa: apagada",
                                    font=("Arial", 10),
                                    bg='#2b2b2b', fg='#ff6b6b')
        self.lbl_cam_mesa.pack(anchor='w')

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

        # Boton ver en grande para cinta
        tk.Button(f_img_cinta, text="Ver en grande (zoom)",
                  font=("Arial", 9),
                  command=self._ver_en_grande_cinta).pack(
            fill='x', padx=5, pady=(0, 5))

        # Panel mesa: alterna entre vista MATRIZ y vista BG SUBTRACTION
        f_img_mesa = tk.LabelFrame(col_der, text=" MESA ",
                                    font=("Arial", 11, "bold"),
                                    bg='#2b2b2b', fg='#ffd47f',
                                    padx=5, pady=5)
        f_img_mesa.pack(fill='x', pady=(0, 5))

        # Toggle Matriz / BG Subtraction
        c_vista = tk.Frame(f_img_mesa, bg='#2b2b2b')
        c_vista.pack(fill='x', padx=5, pady=(0, 4))

        self.var_vista_mesa = tk.StringVar(value="matriz")

        tk.Label(c_vista, text="Vista:", font=("Arial", 9),
                 bg='#2b2b2b', fg='#cfcfcf').pack(side='left', padx=(0, 8))

        self.rb_vista_matriz = tk.Radiobutton(
            c_vista, text="Matriz",
            variable=self.var_vista_mesa, value="matriz",
            command=self._cambiar_vista_mesa,
            font=("Arial", 9),
            bg='#2b2b2b', fg='white',
            selectcolor='#1a1a1a',
            activebackground='#2b2b2b',
            activeforeground='white')
        self.rb_vista_matriz.pack(side='left')

        self.rb_vista_bg = tk.Radiobutton(
            c_vista, text="BG Subtract",
            variable=self.var_vista_mesa, value="bg",
            command=self._cambiar_vista_mesa,
            font=("Arial", 9),
            bg='#2b2b2b', fg='white',
            selectcolor='#1a1a1a',
            activebackground='#2b2b2b',
            activeforeground='white')
        self.rb_vista_bg.pack(side='left', padx=(8, 0))

        # Toggle mostrar numeritos (solo aplica a vista matriz)
        self.var_mostrar_numeros = tk.BooleanVar(value=False)
        self.chk_numeros = tk.Checkbutton(
            f_img_mesa,
            text="Mostrar numeros de posicion (fila, columna)",
            variable=self.var_mostrar_numeros,
            command=lambda: self._dibujar_matriz_actual(),
            font=("Arial", 9),
            bg='#2b2b2b', fg='#cfcfcf',
            selectcolor='#1a1a1a',
            activebackground='#2b2b2b',
            activeforeground='white')
        self.chk_numeros.pack(anchor='w', padx=5, pady=(0, 4))

        # Contenedor con tamaño fijo donde se alternan los dos visuales
        frame_canvas_m = tk.Frame(f_img_mesa, bg='#1a1a1a',
                                   width=MATRIZ_W, height=MATRIZ_H)
        frame_canvas_m.pack(padx=5, pady=5)
        frame_canvas_m.pack_propagate(False)

        # Vista 1: Canvas con matriz de cuadraditos
        self.canvas_matriz = tk.Canvas(frame_canvas_m,
                                        bg='#1a1a1a', highlightthickness=0)
        # Vista 2: Label con imagen de bg subtraction
        self.canvas_bg_mesa = tk.Label(frame_canvas_m, bg='#1a1a1a',
                                        fg='#888888',
                                        text="(sin capturas aun)",
                                        font=("Arial", 10))
        # Empezamos mostrando la matriz
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

        # Boton ver en grande para mesa (vista BG)
        tk.Button(f_img_mesa, text="Ver BG Subtract en grande (zoom)",
                  font=("Arial", 9),
                  command=self._ver_en_grande_mesa).pack(
            fill='x', padx=5, pady=(0, 5))

        # Inicializar matriz vacia
        self._dibujar_matriz(matriz_vacia(), 0, 0)

    def _cambiar_vista_mesa(self):
        """Alterna entre la vista de matriz y la vista de BG subtraction."""
        vista = self.var_vista_mesa.get()
        if vista == "matriz":
            self.canvas_bg_mesa.pack_forget()
            self.canvas_matriz.pack(fill='both', expand=True)
            self.chk_numeros.configure(state='normal')
            self._dibujar_matriz_actual()
        else:
            self.canvas_matriz.pack_forget()
            self.canvas_bg_mesa.pack(fill='both', expand=True)
            self.chk_numeros.configure(state='disabled')
            self._refrescar_panel_bg_mesa()

    def _refrescar_panel_bg_mesa(self):
        """Muestra la imagen de bg subtraction de la ultima captura mesa.
        Estira la imagen al tamaño del contenedor para no dejar bordes negros."""
        with estado.lock:
            panel = estado.ultimo_panel_mesa

        if panel is None:
            self.canvas_bg_mesa.configure(
                image='',
                text="(todavia no se capturo nada en mesa)",
                fg='#888888')
            self._img_bg_mesa = None
            return

        try:
            rgb = cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            # Tamano real del contenedor (puede que la primera vez sea 1x1)
            w_dest = self.canvas_bg_mesa.winfo_width()
            h_dest = self.canvas_bg_mesa.winfo_height()
            if w_dest <= 1 or h_dest <= 1:
                w_dest = MATRIZ_W
                h_dest = MATRIZ_H
            # Resize FORZANDO al tamaño exacto (sin preservar aspect ratio)
            # Asi llena el contenedor completo y se eliminan los bordes negros
            img = img.resize((w_dest, h_dest), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_bg_mesa.configure(image=photo, text='')
            self._img_bg_mesa = photo
        except Exception as e:
            self._agregar_log(f"!!! Error mostrando bg mesa: {e}")

    def _dibujar_matriz_actual(self):
        """Redibuja la matriz con los ultimos datos guardados.
        Util al cambiar el toggle de mostrar numeros."""
        with estado.lock:
            matriz = estado.ultima_matriz_mesa
            fila_m = estado.ultima_fila_mesa
            col_m = estado.ultima_columna_mesa
        self._dibujar_matriz(matriz if matriz is not None else matriz_vacia(),
                              fila_m, col_m)

    def _dibujar_matriz(self, matriz, fila_elegida, columna_elegida):
        """Dibuja la matriz 10x8 en el canvas. matriz puede ser None.
        fila_elegida, columna_elegida son 1-indexed."""
        canvas = self.canvas_matriz
        canvas.delete("all")

        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w <= 1 or h <= 1:
            w = MATRIZ_W
            h = MATRIZ_H

        cell_w = w / GRILLA_COLS
        cell_h = h / GRILLA_FILAS

        # Saber si mostrar numeritos
        mostrar_nums = False
        try:
            mostrar_nums = self.var_mostrar_numeros.get()
        except AttributeError:
            pass  # todavia no se construyo el var

        # Si el espejo de columna esta activo, damos vuelta el orden VISUAL
        # de las columnas para que la matriz se lea como la ve el robot
        # (col 1 a la izquierda, col 10 a la derecha). ci_canvas es la
        # posicion visual (0..9), ci_data es el indice en la matriz raw.
        espejar = cargar_espejo_col_salida("mesa")

        for fi in range(GRILLA_FILAS):
            for ci_canvas in range(GRILLA_COLS):
                ci_data = (GRILLA_COLS - 1 - ci_canvas) if espejar else ci_canvas

                x1 = ci_canvas * cell_w
                y1 = fi * cell_h
                x2 = (ci_canvas + 1) * cell_w
                y2 = (fi + 1) * cell_h

                ocupada = (matriz is not None and matriz[fi][ci_data])
                # Tanto fila_elegida como columna_elegida vienen en coordenadas
                # de salida (ya espejadas si corresponde), asi que comparamos
                # contra ci_canvas+1 (que tambien es coord de salida).
                elegida = (fi + 1 == fila_elegida
                           and ci_canvas + 1 == columna_elegida)

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

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                if mostrar_nums:
                    # Numeritos siempre visibles
                    if elegida and ocupada:
                        text_color = "#000000"
                        text_label = f"\u2605\n{fi+1},{ci_canvas+1}"
                    elif ocupada:
                        text_color = "#ffffff"
                        text_label = f"\u25cf\n{fi+1},{ci_canvas+1}"
                    else:
                        text_color = "#666666"
                        text_label = f"{fi+1},{ci_canvas+1}"
                    canvas.create_text(cx, cy,
                                        text=text_label,
                                        fill=text_color,
                                        font=("Arial", 8, "bold"),
                                        justify="center")
                else:
                    # Solo iconos en las ocupadas
                    if ocupada:
                        canvas.create_text(cx, cy,
                                            text="\u2605" if elegida else "\u25cf",
                                            fill="white" if not elegida else "black",
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

        # col_der: columna derecha CON SCROLL. Crecio mucho con los
        # botones de orientacion + sliders + estado y deja de entrar en
        # pantallas chicas. Envolvemos en Canvas + Scrollbar.
        col_der_outer = tk.Frame(cuerpo, bg='#2b2b2b')
        col_der_outer.grid(row=0, column=1, sticky='ns')

        col_der_canvas = tk.Canvas(col_der_outer, bg='#2b2b2b',
                                    highlightthickness=0, width=280)
        col_der_canvas.pack(side='left', fill='y', expand=False)

        col_der_scroll = ttk.Scrollbar(col_der_outer, orient='vertical',
                                        command=col_der_canvas.yview)
        col_der_scroll.pack(side='right', fill='y')
        col_der_canvas.configure(yscrollcommand=col_der_scroll.set)

        # Frame interior donde van TODOS los widgets que ya tenia col_der.
        # Mantenemos el nombre 'col_der' asi el resto del codigo no cambia.
        col_der = tk.Frame(col_der_canvas, bg='#2b2b2b')
        col_der_window = col_der_canvas.create_window(
            (0, 0), window=col_der, anchor='nw', width=280)

        # Actualizar scrollregion cada vez que el contenido cambia de tamano
        def _on_col_der_configure(event):
            col_der_canvas.configure(scrollregion=col_der_canvas.bbox('all'))
        col_der.bind('<Configure>', _on_col_der_configure)

        # Rueda del mouse: solo bindea cuando el cursor esta sobre el canvas
        # asi no roba el wheel a otras vistas (zoom de imagen, etc).
        def _bind_wheel_calib(_):
            col_der_canvas.bind_all('<MouseWheel>',
                lambda ev: col_der_canvas.yview_scroll(
                    int(-1 * (ev.delta / 120)), 'units'))
            # Linux usa Button-4/5 en lugar de MouseWheel
            col_der_canvas.bind_all('<Button-4>',
                lambda ev: col_der_canvas.yview_scroll(-1, 'units'))
            col_der_canvas.bind_all('<Button-5>',
                lambda ev: col_der_canvas.yview_scroll(1, 'units'))

        def _unbind_wheel_calib(_):
            col_der_canvas.unbind_all('<MouseWheel>')
            col_der_canvas.unbind_all('<Button-4>')
            col_der_canvas.unbind_all('<Button-5>')

        col_der_canvas.bind('<Enter>', _bind_wheel_calib)
        col_der_canvas.bind('<Leave>', _unbind_wheel_calib)

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

        # Rotacion global de camara (no es per-tipo, es del montaje fisico).
        # Cicla 0 -> 90 -> 180 -> 270 -> 0. Se aplica en vision.py apenas
        # captura el frame, antes del ROI: todo el pipeline ve la imagen rotada.
        # IMPORTANTE: despues de rotar hay que recapturar referencias y
        # reajustar recortes de cada tipo/estacion.
        self.btn_rotacion = tk.Button(
            f_acc, text="Rotacion: 0\u00b0",
            font=("Arial", 11),
            width=22, height=2,
            command=self._ciclar_rotacion)
        self.btn_rotacion.pack(pady=4)

        # Flip horizontal (espejo). Se aplica DESPUES de la rotacion en
        # el mismo punto (vision._capturar_frame_raw). Combinado con la
        # rotacion cubre las 8 orientaciones posibles del montaje de camara.
        # Mismo caveat que rotacion: hay que recapturar refs y recortes.
        self.btn_flip_h = tk.Button(
            f_acc, text="Flip H: OFF",
            font=("Arial", 11),
            width=22, height=2,
            command=self._alternar_flip_h)
        self.btn_flip_h.pack(pady=4)

        # Espejo de columna en la SALIDA al robot. NO toca la imagen ni
        # la deteccion: la pantalla muestra la celda detectada como
        # siempre, pero el valor que viaja al KUKA por EKI sale espejado
        # (col_robot = 11 - col_vision). Util cuando la convencion del
        # robot (col 1 a la izq fisicamente) esta espejada respecto a
        # lo que la camara ve, sin necesidad de rehacer recortes/refs.
        self.btn_espejo_col = tk.Button(
            f_acc, text="Espejo Col \u2192 Robot: OFF",
            font=("Arial", 11),
            width=22, height=2,
            command=self._alternar_espejo_col_salida)
        self.btn_espejo_col.pack(pady=4)

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

        # Slider de umbral de ocupacion (solo aplica a MESA)
        # Se guarda en config como 0.0-1.0 pero mostramos 0-100 (%)
        self.lbl_umbral_titulo = tk.Label(
            f_sld, text="Umbral ocupacion (%) [solo mesa]:",
            font=("Arial", 10),
            bg='#2b2b2b', fg='white')
        self.lbl_umbral_titulo.pack(anchor='w', pady=(5, 0))
        self.scl_umbral = tk.Scale(f_sld, from_=1, to=100,
                                    orient='horizontal', length=220,
                                    bg='#2b2b2b', fg='white',
                                    troughcolor='#1a1a1a',
                                    highlightthickness=0,
                                    command=self._on_umbral_change)
        self.scl_umbral.pack(fill='x')
        self.scl_umbral.bind('<ButtonRelease-1>', self._persistir_sliders)

        # Slider de margen interno de celda (solo aplica a MESA).
        # Se guarda en config como 0.0-0.4 pero mostramos 0-40 (%).
        # 0 = celda entera, 25 = solo el 50% central se analiza.
        self.lbl_margen_titulo = tk.Label(
            f_sld, text="Margen interno celda (%) [solo mesa]:",
            font=("Arial", 10),
            bg='#2b2b2b', fg='white')
        self.lbl_margen_titulo.pack(anchor='w', pady=(5, 0))
        self.scl_margen = tk.Scale(f_sld, from_=0, to=40,
                                    orient='horizontal', length=220,
                                    bg='#2b2b2b', fg='white',
                                    troughcolor='#1a1a1a',
                                    highlightthickness=0,
                                    command=self._on_margen_change)
        self.scl_margen.pack(fill='x')
        self.scl_margen.bind('<ButtonRelease-1>', self._persistir_sliders)

        # Inicializar texto del boton de rotacion con el valor persistido.
        self._actualizar_label_rotacion()
        self._actualizar_label_flip_h()
        self._actualizar_label_espejo_col_salida()

    # ------------------------------------------------------------------
    #  TORNO
    # ------------------------------------------------------------------
    def _construir_tab_torno(self):
        cont = tk.Frame(self.tab_torno, bg='#2b2b2b')
        cont.pack(fill='both', expand=True, padx=15, pady=10)

        # Header
        f_header = tk.Frame(cont, bg='#1a1a1a')
        f_header.pack(fill='x', pady=(0, 10))

        tk.Label(f_header, text="TORNO FANUC",
                 font=("Arial", 16, "bold"),
                 bg='#1a1a1a', fg='#7fffd4').pack(side='left', padx=15, pady=10)

        self.lbl_torno_estado = tk.Label(
            f_header, text="Estado: ---",
            font=("Arial", 12, "bold"),
            bg='#1a1a1a', fg='#ff6b6b')
        self.lbl_torno_estado.pack(side='right', padx=15)

        # Columnas
        cols = tk.Frame(cont, bg='#2b2b2b')
        cols.pack(fill='both', expand=True)
        cols.columnconfigure(0, weight=1, uniform='c')
        cols.columnconfigure(1, weight=1, uniform='c')
        cols.rowconfigure(0, weight=1)

        col_izq = tk.Frame(cols, bg='#2b2b2b')
        col_izq.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        col_der = tk.Frame(cols, bg='#2b2b2b')
        col_der.grid(row=0, column=1, sticky='nsew', padx=(8, 0))

        # ========== Columna izquierda: COLA ==========
        f_cola = tk.LabelFrame(col_izq, text=" COLA DE PIEZAS ",
                                font=("Arial", 11, "bold"),
                                bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_cola.pack(fill='both', expand=True)

        tk.Label(f_cola,
                 text="Primero = proximo que va a mecanizar el torno",
                 font=("Arial", 9, "italic"),
                 bg='#2b2b2b', fg='#a0a0a0').pack(anchor='w', pady=(0, 4))

        # Listbox
        f_lb = tk.Frame(f_cola, bg='#2b2b2b')
        f_lb.pack(fill='both', expand=True)

        sb = tk.Scrollbar(f_lb)
        sb.pack(side='right', fill='y')

        self.lst_cola = tk.Listbox(f_lb, bg='#1a1a1a', fg='#cfcfcf',
                                    font=("Consolas", 10),
                                    yscrollcommand=sb.set,
                                    selectmode='single',
                                    highlightthickness=0)
        self.lst_cola.pack(side='left', fill='both', expand=True)
        sb.config(command=self.lst_cola.yview)

        # Controles cola
        c_ctrl = tk.Frame(f_cola, bg='#2b2b2b')
        c_ctrl.pack(fill='x', pady=(8, 0))

        tk.Button(c_ctrl, text="Encolar manual",
                  font=("Arial", 9),
                  command=self._encolar_manual_torno).pack(side='left', padx=2)
        tk.Button(c_ctrl, text="Quitar seleccionado",
                  font=("Arial", 9),
                  command=self._quitar_seleccion_torno).pack(side='left', padx=2)
        tk.Button(c_ctrl, text="Limpiar cola",
                  font=("Arial", 9),
                  bg='#cc4444', fg='white',
                  command=self._limpiar_cola_torno).pack(side='left', padx=2)

        # Stats
        f_stats = tk.LabelFrame(col_izq, text=" ESTADISTICAS ",
                                 font=("Arial", 11, "bold"),
                                 bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_stats.pack(fill='x', pady=(10, 0))

        self.lbl_cola_count = tk.Label(f_stats,
                                        text="En cola: 0",
                                        font=("Arial", 11),
                                        bg='#2b2b2b', fg='white')
        self.lbl_cola_count.pack(anchor='w')

        self.lbl_torno_terminadas = tk.Label(f_stats,
                                              text="Piezas terminadas: 0",
                                              font=("Arial", 11),
                                              bg='#2b2b2b', fg='white')
        self.lbl_torno_terminadas.pack(anchor='w', pady=(3, 0))

        self.lbl_torno_ultimo = tk.Label(f_stats,
                                          text="Ultimo evento: ---",
                                          font=("Arial", 9),
                                          bg='#2b2b2b', fg='#a0a0a0',
                                          wraplength=400, justify='left')
        self.lbl_torno_ultimo.pack(anchor='w', pady=(5, 0))

        # ========== Columna derecha: CONEXION + INSPECCION ==========
        f_conn = tk.LabelFrame(col_der, text=" CONEXION ",
                                font=("Arial", 11, "bold"),
                                bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_conn.pack(fill='x')

        tk.Label(f_conn,
                 text=f"Host: {torno.config['torno_host']}:{torno.config['torno_port']}",
                 font=("Consolas", 10),
                 bg='#2b2b2b', fg='#cfcfcf').pack(anchor='w')

        tk.Label(f_conn,
                 text=f"Macros: #{torno.config['macro_tipo']}=tipo  "
                      f"#{torno.config['macro_op']}=op  "
                      f"#{torno.config['macro_fin']}=fin",
                 font=("Consolas", 9),
                 bg='#2b2b2b', fg='#a0a0a0').pack(anchor='w', pady=(2, 8))

        c_btns_conn = tk.Frame(f_conn, bg='#2b2b2b')
        c_btns_conn.pack(fill='x')

        tk.Button(c_btns_conn, text="Reconectar",
                  font=("Arial", 9),
                  command=self._reconectar_torno).pack(side='left', padx=2)
        tk.Button(c_btns_conn, text="Refrescar status",
                  font=("Arial", 9),
                  command=self._refrescar_torno_status).pack(side='left', padx=2)

        # Status detallado
        f_status = tk.LabelFrame(col_der, text=" STATUS DEL TORNO ",
                                  font=("Arial", 11, "bold"),
                                  bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_status.pack(fill='x', pady=(10, 0))

        self.txt_torno_status = tk.Text(
            f_status, height=10, bg='#1a1a1a', fg='#cfcfcf',
            font=("Consolas", 9), wrap='word',
            state='disabled', highlightthickness=0)
        self.txt_torno_status.pack(fill='x')

        # Inspector de macros
        f_insp = tk.LabelFrame(col_der, text=" INSPECTOR DE MACROS ",
                                font=("Arial", 11, "bold"),
                                bg='#2b2b2b', fg='white', padx=10, pady=10)
        f_insp.pack(fill='x', pady=(10, 0))

        c_insp = tk.Frame(f_insp, bg='#2b2b2b')
        c_insp.pack(fill='x')

        tk.Label(c_insp, text="Macro #:", font=("Arial", 10),
                 bg='#2b2b2b', fg='white').grid(row=0, column=0, padx=2)
        self.ent_insp_num = tk.Entry(c_insp, width=8, font=("Consolas", 10))
        self.ent_insp_num.grid(row=0, column=1, padx=2)
        self.ent_insp_num.insert(0, "500")

        tk.Button(c_insp, text="Leer",
                  command=self._leer_macro_torno,
                  font=("Arial", 9)).grid(row=0, column=2, padx=2)

        tk.Label(c_insp, text="Valor:", font=("Arial", 10),
                 bg='#2b2b2b', fg='white').grid(row=0, column=3, padx=2)
        self.ent_insp_val = tk.Entry(c_insp, width=10, font=("Consolas", 10))
        self.ent_insp_val.grid(row=0, column=4, padx=2)

        tk.Button(c_insp, text="Escribir",
                  command=self._escribir_macro_torno,
                  font=("Arial", 9),
                  bg='#cc7a00', fg='white').grid(row=0, column=5, padx=2)

        self.lbl_insp_resultado = tk.Label(
            f_insp, text="(sin lectura)",
            font=("Consolas", 10),
            bg='#2b2b2b', fg='#cfcfcf')
        self.lbl_insp_resultado.pack(anchor='w', pady=(8, 0))

    def _refrescar_cola_torno(self):
        """Refresca el listbox de la cola del torno."""
        items = torno.cola.snapshot()
        seleccion = self.lst_cola.curselection()
        idx_sel = seleccion[0] if seleccion else None

        self.lst_cola.delete(0, 'end')
        for i, item in enumerate(items):
            marca = "►" if i == 0 else " "
            linea = (f"{marca} #{i + 1:2}  tipo={item['tipo']}  "
                     f"op={item['op']:2}  ({item.get('ts', '?')})")
            self.lst_cola.insert('end', linea)
            if i == 0:
                self.lst_cola.itemconfig(0, fg='#FAC775')

        if idx_sel is not None and idx_sel < len(items):
            self.lst_cola.selection_set(idx_sel)

        self.lbl_cola_count.configure(text=f"En cola: {len(items)}")

    def _encolar_manual_torno(self):
        from tkinter import simpledialog
        tipo_s = simpledialog.askstring("Encolar pieza",
                                         "Tipo (1-6):", parent=self.root)
        if not tipo_s:
            return
        op_s = simpledialog.askstring("Encolar pieza",
                                       "Operacion (10 o 20):", parent=self.root)
        if not op_s:
            return
        try:
            tipo = int(tipo_s)
            op = int(op_s)
            torno.encolar(tipo, op)
            self._refrescar_cola_torno()
        except ValueError:
            messagebox.showerror("Error", "Valores invalidos")

    def _quitar_seleccion_torno(self):
        seleccion = self.lst_cola.curselection()
        if not seleccion:
            messagebox.showinfo("Quitar",
                                 "Seleccioná un item de la cola primero.")
            return
        idx = seleccion[0]
        if messagebox.askyesno("Confirmar",
                                f"¿Quitar item #{idx + 1} de la cola?"):
            torno.quitar_indice(idx)
            self._refrescar_cola_torno()

    def _limpiar_cola_torno(self):
        n = len(torno.cola)
        if n == 0:
            return
        if messagebox.askyesno("Confirmar",
                                f"¿Limpiar toda la cola? ({n} items)"):
            torno.limpiar_cola()
            self._refrescar_cola_torno()

    def _reconectar_torno(self):
        self._agregar_log("[TORNO] Forzando reconexion...")
        torno.reconectar()

    def _refrescar_torno_status(self):
        st = torno.status()
        self.txt_torno_status.configure(state='normal')
        self.txt_torno_status.delete('1.0', 'end')
        if st is None:
            self.txt_torno_status.insert('end',
                                          "(torno no conectado)\n")
        else:
            txt = (f"Modo:         {st.modo}\n"
                   f"Ejecucion:    {st.ejecucion}\n"
                   f"Emergencia:   {st.emergencia}\n"
                   f"Alarma:       {st.alarma}\n"
                   f"Tipo:         {st.tipo}\n"
                   f"Spindle RPM:  {st.spindle_rpm:.1f}\n"
                   f"Spindle load: {st.spindle_load_pct}%\n"
                   f"Posicion:     {st.posicion_ejes}\n"
                   f"Programa:     {st.programa_actual}\n"
                   f"Linea:        {st.linea}\n"
                   f"Part count:   {st.part_count}\n"
                   f"Mensaje op:   {st.mensaje_operador}\n"
                   f"Alarma mask:  {st.alarma_bitmask}\n")
            self.txt_torno_status.insert('end', txt)
        self.txt_torno_status.configure(state='disabled')

    def _leer_macro_torno(self):
        try:
            num = int(self.ent_insp_num.get())
            v = torno.read_macro(num)
            self.lbl_insp_resultado.configure(
                text=f"#{num} = {v}", fg='#7fff7f')
            self.ent_insp_val.delete(0, 'end')
            self.ent_insp_val.insert(0, str(v))
        except Exception as e:
            self.lbl_insp_resultado.configure(
                text=f"Error: {e}", fg='#ff6b6b')

    def _escribir_macro_torno(self):
        try:
            num = int(self.ent_insp_num.get())
            v = float(self.ent_insp_val.get())
            if not messagebox.askyesno(
                "Confirmar escritura",
                f"¿Escribir #{num} = {v}?\n\n"
                f"OJO: esto puede afectar al programa NC del torno."):
                return
            torno.write_macro(num, v)
            self.lbl_insp_resultado.configure(
                text=f"#{num} <- {v} OK", fg='#FAC775')
            self._agregar_log(f"[TORNO] Escrito manual: #{num} = {v}")
        except Exception as e:
            self.lbl_insp_resultado.configure(
                text=f"Error: {e}", fg='#ff6b6b')

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

    def _toggle_cinta_manual(self):
        """Activado/desactivado del modo manual de cinta con el checkbox.
        Habilita/deshabilita los botones de orientacion y el capturar test."""
        activo = self.var_cinta_manual_activo.get()
        if not activo:
            # Sin tildar: deshabilitamos botones de orientacion,
            # limpiamos la seleccion
            estado.set_orientacion_manual(None)
        # Refrescar estado visual de todo
        self._aplicar_estado_modos()
        self._actualizar_label_seleccion()

    def _toggle_orientacion(self, orient):
        """Apreta un boton de orientacion (ARRIBA/ABAJO/VACIO).
        Solo funciona si el checkbox Activar esta tildado.
        Si ya estaba activo, lo desactiva. Si no, lo activa.
        """
        if not self.var_cinta_manual_activo.get():
            return  # checkbox desactivado: ignorar
        est = estado.get_estado()
        if est["orientacion_manual"] == orient:
            estado.set_orientacion_manual(None)
        else:
            estado.set_orientacion_manual(orient)
        self._refrescar_botones_orientacion()
        self._actualizar_label_seleccion()
        # Si se selecciono una orientacion, el capturar ahora se desactiva
        self._aplicar_estado_modos()

    def _aplicar_estado_modos(self):
        """Aplica las reglas de habilitar/deshabilitar controles segun los modos:
        
        AUTO cinta:
          - btns_orient deshabilitados, chk_cinta_manual deshabilitado
          - btn_capturar_test_cinta deshabilitado
        MANUAL cinta + checkbox SIN tildar:
          - btns_orient deshabilitados
          - btn_capturar_test_cinta HABILITADO
        MANUAL cinta + checkbox TILDADO:
          - btns_orient HABILITADOS
          - btn_capturar_test_cinta deshabilitado
        
        Mismo esquema para mesa con chk_mesa_manual.
        """
        est = estado.get_estado()
        
        # ========== CINTA ==========
        if est["modo_auto_cinta"]:
            # AUTO: nada manual disponible
            self.chk_cinta_manual.configure(state='disabled')
            for btn in self.btns_orient:
                btn.configure(state='disabled',
                              bg='#3a3a3a', fg='#777777',
                              relief='flat',
                              disabledforeground='#777777')
            self.btn_capturar_test_cinta.configure(state='disabled',
                                                    bg='#3a3a3a',
                                                    disabledforeground='#777777')
        else:
            # MANUAL
            self.chk_cinta_manual.configure(state='normal')
            if self.var_cinta_manual_activo.get():
                # Checkbox tildado: orientaciones disponibles, capturar OFF
                for btn in self.btns_orient:
                    btn.configure(state='normal')
                self._refrescar_botones_orientacion()
                self.btn_capturar_test_cinta.configure(state='disabled',
                                                        bg='#3a3a3a',
                                                        disabledforeground='#777777')
            else:
                # Checkbox sin tildar: orientaciones OFF, capturar disponible
                for btn in self.btns_orient:
                    btn.configure(state='disabled',
                                  bg='#3a3a3a', fg='#777777',
                                  relief='flat',
                                  disabledforeground='#777777')
                self.btn_capturar_test_cinta.configure(state='normal',
                                                        bg='#4a90e2',
                                                        fg='white')

        # ========== MESA ==========
        if est["modo_auto_mesa"]:
            # AUTO
            self.chk_mesa_manual.configure(state='disabled')
            self.spn_fila.configure(state='disabled')
            self.spn_col.configure(state='disabled')
            self.btn_capturar_test_mesa.configure(state='disabled',
                                                   bg='#3a3a3a',
                                                   disabledforeground='#777777')
        else:
            # MANUAL
            self.chk_mesa_manual.configure(state='normal')
            if self.var_mesa_manual_activo.get():
                self.spn_fila.configure(state='normal')
                self.spn_col.configure(state='normal')
                self.btn_capturar_test_mesa.configure(state='disabled',
                                                       bg='#3a3a3a',
                                                       disabledforeground='#777777')
            else:
                self.spn_fila.configure(state='disabled')
                self.spn_col.configure(state='disabled')
                self.btn_capturar_test_mesa.configure(state='normal',
                                                       bg='#ffd47f',
                                                       fg='black')

    def _refrescar_botones_orientacion(self):
        """Pinta los botones de orientacion segun cual esta activo."""
        est = estado.get_estado()
        activa = est["orientacion_manual"]
        colores = {"ARRIBA": "#2d8f3a", "ABAJO": "#cc7a00",
                   "VACIO": "#777777"}
        for btn in self.btns_orient:
            txt = btn['text']
            if txt == activa:
                btn.configure(bg=colores[txt], fg='white',
                              relief='sunken')
            else:
                btn.configure(bg='SystemButtonFace', fg='black',
                              relief='raised')

    def _seleccionar_orientacion(self, orient):
        """Wrapper viejo. Setea sin toggle (lo deja activo)."""
        estado.set_orientacion_manual(orient)
        self._refrescar_botones_orientacion()
        self._actualizar_label_seleccion()

    def _cambiar_modo_cinta(self, auto):
        if auto:
            if not self._asegurar_camara(detector_cinta):
                return
            estado.set_modo_auto_cinta(True)
            self.btn_auto_cinta.configure(bg='#2d8f3a', fg='white',
                                           relief='sunken')
            self.btn_manual_cinta.configure(bg='SystemButtonFace', fg='black',
                                             relief='raised')
        else:
            estado.set_modo_auto_cinta(False)
            self.btn_manual_cinta.configure(bg='#4a90e2', fg='white',
                                             relief='sunken')
            self.btn_auto_cinta.configure(bg='SystemButtonFace', fg='black',
                                           relief='raised')

        self._aplicar_estado_modos()
        self._actualizar_label_seleccion()

    def _cambiar_modo_mesa(self, auto):
        if auto:
            if not self._asegurar_camara(detector_mesa):
                return
            estado.set_modo_auto_mesa(True)
            self.btn_auto_mesa.configure(bg='#2d8f3a', fg='white',
                                          relief='sunken')
            self.btn_manual_mesa.configure(bg='SystemButtonFace', fg='black',
                                            relief='raised')
        else:
            estado.set_modo_auto_mesa(False)
            self.btn_manual_mesa.configure(bg='#4a90e2', fg='white',
                                            relief='sunken')
            self.btn_auto_mesa.configure(bg='SystemButtonFace', fg='black',
                                          relief='raised')

        self._aplicar_estado_modos()
        self._actualizar_label_seleccion()

    def _capturar_test_cinta(self):
        """Captura una foto y la analiza como cinta. Muestra resultado
        en el panel pero NO afecta al robot. Funciona en cualquier modo.
        """
        est_actual = estado.get_estado()
        tipo = est_actual["tipo"]

        def trabajo():
            detector = detector_cinta  # camara de la cinta
            camara_iniciada_aqui = False
            try:
                if not detector.esta_activo():
                    self._log_thread_safe("[TEST CINTA] Iniciando camara...")
                    detector.start(requiere_referencia=False)
                    camara_iniciada_aqui = True
                    self._log_thread_safe("[TEST CINTA] Camara lista.")

                self._log_thread_safe(f"[TEST CINTA] Capturando Tipo {tipo}...")
                consultar_vision_cinta(tipo, self._log_thread_safe)
            except Exception as e:
                self._log_thread_safe(f"!!! Error en test cinta: {e}")
            finally:
                if camara_iniciada_aqui and detector.esta_activo():
                    try:
                        detector.stop()
                        self._log_thread_safe("[TEST CINTA] Camara apagada.")
                    except Exception as e:
                        self._log_thread_safe(f"!!! Error apagando: {e}")

        threading.Thread(target=trabajo, daemon=True).start()

    def _capturar_test_mesa(self):
        """Captura una foto y la analiza como mesa. Muestra matriz en
        el panel pero NO afecta al robot. Funciona en cualquier modo.
        """
        est_actual = estado.get_estado()
        tipo = est_actual["tipo"]

        def trabajo():
            detector = detector_mesa  # camara de la mesa
            camara_iniciada_aqui = False
            try:
                if not detector.esta_activo():
                    self._log_thread_safe("[TEST MESA] Iniciando camara...")
                    detector.start(requiere_referencia=False)
                    camara_iniciada_aqui = True
                    self._log_thread_safe("[TEST MESA] Camara lista.")

                self._log_thread_safe(f"[TEST MESA] Capturando Tipo {tipo}...")
                consultar_vision_mesa(tipo, self._log_thread_safe)
            except Exception as e:
                self._log_thread_safe(f"!!! Error en test mesa: {e}")
            finally:
                if camara_iniciada_aqui and detector.esta_activo():
                    try:
                        detector.stop()
                        self._log_thread_safe("[TEST MESA] Camara apagada.")
                    except Exception as e:
                        self._log_thread_safe(f"!!! Error apagando: {e}")

        threading.Thread(target=trabajo, daemon=True).start()

    def _asegurar_camara(self, det=None):
        """Si la camara del detector no esta activa, intenta encenderla.
        True si quedo encendida. Si det es None usa el detector de calibracion."""
        if det is None:
            det = detector
        if det.esta_activo():
            return True

        est = det.estacion_activa()
        self._agregar_log(f"[VISION] Iniciando camara ({est})...")
        self.root.update_idletasks()
        try:
            det.start(requiere_referencia=False)
            self._agregar_log(f"[VISION] Camara ({est}) lista.")
            return True
        except Exception as e:
            self._agregar_log(f"!!! Error iniciando camara ({est}): {e}")
            messagebox.showerror("Camara",
                                 f"No se pudo iniciar la camara ({est}):\n\n{e}")
            return False

    def _toggle_mesa_manual(self):
        """Activado/desactivado del modo manual de mesa con el checkbox."""
        if self.var_mesa_manual_activo.get():
            try:
                fila = int(self.spn_fila.get())
                col = int(self.spn_col.get())
            except ValueError:
                fila, col = 1, 1
            estado.set_mesa_manual(fila, col)
        else:
            estado.set_mesa_manual(0, 0)  # 0/0 = no hay manual
        self._aplicar_estado_modos()
        self._actualizar_label_seleccion()

    def _cambio_mesa_manual(self):
        # Solo aplica si el checkbox de activacion esta tildado
        if not self.var_mesa_manual_activo.get():
            return
        try:
            fila = int(self.spn_fila.get())
            col = int(self.spn_col.get())
        except ValueError:
            return
        estado.set_mesa_manual(fila, col)
        self._actualizar_label_seleccion()

    def _actualizar_label_seleccion(self):
        est = estado.get_estado()
        cinta_cal = "calibracion:OK" if tiene_referencia(est["tipo"], "cinta") else "calibracion:sin cal"
        mesa_cal = "calibracion:OK" if tiene_referencia(est["tipo"], "mesa") else "calibracion:sin cal"

        if est["modo_auto_cinta"]:
            cinta_modo = "AUTO"
        elif est["orientacion_manual"]:
            cinta_modo = f"manual:{est['orientacion_manual']}"
        else:
            cinta_modo = "manual:(sin seleccion)"

        if est["modo_auto_mesa"]:
            mesa_modo = "AUTO"
        elif est["mesa_manual_fila"] > 0 and est["mesa_manual_columna"] > 0:
            mesa_modo = f"manual:({est['mesa_manual_fila']},{est['mesa_manual_columna']})"
        else:
            mesa_modo = "manual:(sin seleccion)"

        self.lbl_seleccion.configure(
            text=f"Tipo: {est['tipo']}   |   "
                 f"Cinta ({cinta_cal}, {cinta_modo})   |   "
                 f"Mesa ({mesa_cal}, {mesa_modo})"
        )

    # ==================================================================
    #  Handlers CALIBRAR
    # ==================================================================
    def _set_estacion_calibrar(self, estacion):
        global detector
        venia_preview = self._preview_activo
        if venia_preview:
            self._detener_preview()

        # Si veniamos calibrando otra estacion con su camara abierta,
        # apagarla antes de pasar a la nueva (no dejar dos camaras vivas).
        det_anterior = detector
        det_nuevo = detector_cinta if estacion == "cinta" else detector_mesa
        if det_anterior is not det_nuevo and det_anterior.esta_activo():
            try:
                det_anterior.stop()
            except Exception as e:
                self._agregar_log(f"!!! Error apagando camara anterior: {e}")

        self._estacion_calibrar = estacion
        # El alias 'detector' (usado por toda la pestania Calibrar/preview)
        # apunta al detector de la estacion elegida.
        detector = det_nuevo

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

        # Refrescar los botones de orientacion para mostrar los valores
        # de ESTA estacion (cada camara tiene su propia rotacion/flip).
        self._actualizar_label_rotacion()
        self._actualizar_label_flip_h()
        self._actualizar_label_espejo_col_salida()

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
            self.scl_umbral.set(detector.umbral_ocupacion)
            self.scl_margen.set(detector.margen_celda)
        except Exception:
            pass

        # Sliders de umbral y margen solo activos en estacion mesa
        if est == "mesa":
            self.scl_umbral.configure(state='normal',
                                       fg='white', troughcolor='#1a1a1a')
            self.lbl_umbral_titulo.configure(fg='white')
            self.scl_margen.configure(state='normal',
                                       fg='white', troughcolor='#1a1a1a')
            self.lbl_margen_titulo.configure(fg='white')
        else:
            self.scl_umbral.configure(state='disabled',
                                       fg='#666666', troughcolor='#2a2a2a')
            self.lbl_umbral_titulo.configure(fg='#666666')
            self.scl_margen.configure(state='disabled',
                                       fg='#666666', troughcolor='#2a2a2a')
            self.lbl_margen_titulo.configure(fg='#666666')

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
            from vision import cargar_config
            import numpy as np

            ref = cv2.imread(ref_path)
            if ref is None:
                raise RuntimeError("No se pudo leer la imagen")

            cfg = cargar_config(tipo, est)
            h, w = ref.shape[:2]

            debug = ref.copy()
            esquinas_cfg = cfg.get("esquinas")

            if esquinas_cfg is not None:
                # Modo nuevo: 4 esquinas (cuadrilatero)
                esq = [
                    (esquinas_cfg["sup_izq"][0] * w,
                     esquinas_cfg["sup_izq"][1] * h),
                    (esquinas_cfg["sup_der"][0] * w,
                     esquinas_cfg["sup_der"][1] * h),
                    (esquinas_cfg["inf_der"][0] * w,
                     esquinas_cfg["inf_der"][1] * h),
                    (esquinas_cfg["inf_izq"][0] * w,
                     esquinas_cfg["inf_izq"][1] * h),
                ]
                pts_int = np.array([[int(p[0]), int(p[1])] for p in esq],
                                    dtype=np.int32)

                # Oscurecer fuera del cuadrilatero
                mask = np.zeros((h, w), dtype=np.uint8)
                cv2.fillPoly(mask, [pts_int], 255)
                oscuro = (debug * 0.4).astype(np.uint8)
                debug = np.where(mask[..., None] == 255, debug, oscuro)
                debug = debug.astype(np.uint8)

                # Contorno
                cv2.polylines(debug, [pts_int], True, (0, 255, 255), 2)

                # Grilla deformada (solo mesa)
                if est == "mesa":
                    def interp(p1, p2, t):
                        return (p1[0] + (p2[0] - p1[0]) * t,
                                p1[1] + (p2[1] - p1[1]) * t)
                    for c in range(1, GRILLA_COLS):
                        t = c / GRILLA_COLS
                        p_top = interp(esq[0], esq[1], t)
                        p_bot = interp(esq[3], esq[2], t)
                        cv2.line(debug,
                                 (int(p_top[0]), int(p_top[1])),
                                 (int(p_bot[0]), int(p_bot[1])),
                                 (50, 200, 255), 1)
                    for f in range(1, GRILLA_FILAS):
                        t = f / GRILLA_FILAS
                        p_izq = interp(esq[0], esq[3], t)
                        p_der = interp(esq[1], esq[2], t)
                        cv2.line(debug,
                                 (int(p_izq[0]), int(p_izq[1])),
                                 (int(p_der[0]), int(p_der[1])),
                                 (50, 200, 255), 1)

                # Puntitos en las esquinas
                for p in esq:
                    cv2.circle(debug, (int(p[0]), int(p[1])), 5,
                               (0, 255, 255), -1)

                info_extra = "4 esquinas"
            else:
                # Modo legacy: rectangulo
                y_top = int(h * cfg.get("corte_y_top_pct", 0.0))
                y_bot = int(h * cfg.get("corte_y_pct", 1.0))
                x_izq = int(w * cfg.get("corte_x_izq_pct", 0.0))
                x_der = int(w * cfg.get("corte_x_der_pct", 1.0))

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

                info_extra = f"rect Y {y_top}-{y_bot} X {x_izq}-{x_der}"

            rgb = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            img.thumbnail((PANEL_W, PANEL_H), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            self.canvas_referencia.configure(image=photo, text='')
            self._img_referencia = photo

            ts = datetime.fromtimestamp(os.path.getmtime(ref_path))
            extra_grilla = f"  |  grilla {GRILLA_COLS}x{GRILLA_FILAS}" if est == "mesa" else ""
            self.lbl_ref_info.configure(
                text=(f"Tipo {tipo} ({est}) - {ts.strftime('%Y-%m-%d %H:%M')}"
                      f"  |  {info_extra}{extra_grilla}"))
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

        # IMPORTANTE: siempre capturamos un frame en VIVO para el recorte.
        # No leemos la referencia del disco porque puede estar guardada con
        # una orientacion (rotacion/flip) distinta a la activa ahora — en
        # ese caso las 4 esquinas marcadas quedarian en coords del frame
        # viejo y no coincidirian con la captura en operacion real.
        # El frame en vivo siempre pasa por aplicar_orientacion() asi que
        # esta sincronizado con lo que va a ver el detector.
        if not self._asegurar_camara():
            return
        try:
            imagen = detector.capturar_frame()
        except Exception as e:
            messagebox.showerror("Captura", f"No se pudo capturar:\n\n{e}")
            return

        if os.path.exists(ref_path):
            self._agregar_log(f"[CALIBRAR] Ajustando recorte Tipo {tipo} ({est}) "
                              f"(usando frame en vivo, no la ref guardada)")
        else:
            self._agregar_log(f"[CALIBRAR] Ajustando recorte Tipo {tipo} ({est}) "
                              f"(no hay referencia guardada todavia)")

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

    # ------------------------------------------------------------------
    #  Rotacion global de camara
    # ------------------------------------------------------------------
    def _ciclar_rotacion(self):
        """Cicla la rotacion de la camara de ESTA estacion 0->90->180->270->0."""
        est = self._estacion_calibrar
        k_actual = cargar_rotacion(est)
        k_nuevo = (k_actual + 1) % 4
        guardar_rotacion(k_nuevo, est)
        self._actualizar_label_rotacion()
        grados = k_nuevo * 90
        self._agregar_log(f"[CALIBRAR] Rotacion camara ({est}): {grados} grados")
        self._agregar_log(
            "[CALIBRAR] >>> ATENCION: hay que RECAPTURAR REFERENCIAS y "
            "REAJUSTAR RECORTES de esta estacion. Las viejas "
            "quedaron desfasadas.")
        # Si hay preview corriendo, los frames siguientes ya salen rotados:
        # no hace falta nada mas. El usuario va a ver el cambio al toque.

    def _actualizar_label_rotacion(self):
        """Refresca el texto del boton segun la rotacion de la estacion."""
        try:
            k = cargar_rotacion(self._estacion_calibrar)
            grados = k * 90
            self.btn_rotacion.configure(text=f"Rotacion: {grados}\u00b0")
        except Exception:
            pass

    # ------------------------------------------------------------------
    #  Flip horizontal de camara
    # ------------------------------------------------------------------
    def _alternar_flip_h(self):
        """Toggle del flip horizontal de la camara de ESTA estacion."""
        est = self._estacion_calibrar
        nuevo = not cargar_flip_h(est)
        guardar_flip_h(nuevo, est)
        self._actualizar_label_flip_h()
        estado = "ON" if nuevo else "OFF"
        self._agregar_log(f"[CALIBRAR] Flip H ({est}): {estado}")
        self._agregar_log(
            "[CALIBRAR] >>> ATENCION: hay que RECAPTURAR REFERENCIAS y "
            "REAJUSTAR RECORTES de esta estacion. Las viejas "
            "quedaron desfasadas.")

    def _actualizar_label_flip_h(self):
        """Refresca el texto del boton segun el flip de la estacion."""
        try:
            estado = "ON" if cargar_flip_h(self._estacion_calibrar) else "OFF"
            self.btn_flip_h.configure(text=f"Flip H: {estado}")
        except Exception:
            pass

    # ------------------------------------------------------------------
    #  Espejo de columna en la SALIDA al robot
    # ------------------------------------------------------------------
    def _alternar_espejo_col_salida(self):
        """Toggle del espejo de columna en la salida hacia el KUKA.

        Solo aplica a la MESA (la cinta no maneja columnas). NO toca la
        imagen ni la deteccion: solo invierte el numero de columna que se
        manda al robot via EKI. No requiere recapturar referencias.
        """
        nuevo = not cargar_espejo_col_salida("mesa")
        guardar_espejo_col_salida(nuevo, "mesa")
        self._actualizar_label_espejo_col_salida()
        estado = "ON" if nuevo else "OFF"
        self._agregar_log(f"[CALIBRAR] Espejo Col -> Robot (mesa): {estado}")
        if nuevo:
            self._agregar_log(
                "[CALIBRAR] Vision sigue mostrando la celda detectada en "
                "pantalla, pero al robot le manda col_robot = 11 - col_vision.")
        else:
            self._agregar_log(
                "[CALIBRAR] Vision manda al robot la misma columna que muestra.")

    def _actualizar_label_espejo_col_salida(self):
        """Refresca el texto del boton segun el estado persistido (mesa)."""
        try:
            estado = "ON" if cargar_espejo_col_salida("mesa") else "OFF"
            self.btn_espejo_col.configure(text=f"Espejo Col \u2192 Robot: {estado}")
        except Exception:
            pass

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

    def _on_umbral_change(self, valor):
        try:
            detector.set_umbral_ocupacion(int(valor), persist=False)
        except Exception as e:
            self._agregar_log(f"!!! umbral: {e}")

    def _on_margen_change(self, valor):
        try:
            detector.set_margen_celda(int(valor), persist=False)
        except Exception as e:
            self._agregar_log(f"!!! margen: {e}")

    def _persistir_sliders(self, _evt=None):
        try:
            detector.set_exposure(detector.exposure, persist=True)
            extra = ""
            if self._estacion_calibrar == "mesa":
                extra = (f" umbral={detector.umbral_ocupacion}%"
                         f" margen={detector.margen_celda}%")
            self._agregar_log(
                f"[CONFIG] Tipo {self._tipo_calibrar} ({self._estacion_calibrar}): "
                f"exp={detector.exposure} gain={detector.gain} "
                f"thr={detector.bg_diff_thresh}{extra}"
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
            # Calibrar y Automatico NO conviven (usan las mismas camaras).
            # Al entrar a Calibrar, cerramos las camaras del modo auto y
            # pasamos ambas estaciones a MANUAL, para que el preview pueda
            # abrir su camara sin chocar con un pipeline ya abierto.
            self._apagar_camaras_auto()
            # Refrescar el toggle de estacion activa
            self._set_estacion_calibrar(self._estacion_calibrar)

    def _apagar_camaras_auto(self):
        """Apaga las camaras del modo automatico y pasa a MANUAL. Se usa al
        entrar a Calibrar (auto y calibrar no pueden usar las camaras a la vez)."""
        cambio = False
        if estado.get_estado()["modo_auto_cinta"]:
            self._cambiar_modo_cinta(False)
            cambio = True
        if estado.get_estado()["modo_auto_mesa"]:
            self._cambiar_modo_mesa(False)
            cambio = True
        for det, nombre in ((detector_cinta, "cinta"), (detector_mesa, "mesa")):
            if det.esta_activo():
                try:
                    det.stop()
                    self._agregar_log(f"[VISION] Camara ({nombre}) apagada "
                                      f"(entrando a Calibrar).")
                except Exception as e:
                    self._agregar_log(f"!!! Error apagando camara "
                                      f"({nombre}): {e}")
        if cambio:
            self._agregar_log("[VISION] Modo AUTOMATICO desactivado "
                              "(entraste a Calibrar).")

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

        # Iniciar tambien el thread de sincronizacion del torno
        torno._log_callback = self._log_thread_safe
        if FANUC_DISPONIBLE:
            ok = torno.start()
            if ok:
                self._log_thread_safe("[TORNO] Thread sincronizador iniciado")
        else:
            self._log_thread_safe(
                "[TORNO] fanuc_bridge NO disponible. "
                "Pesta\u00f1a TORNO se ve pero no se conecta hasta instalarlo.")

    # ==================================================================
    #  Loop refresco
    # ==================================================================
    def _mostrar_popup_manual_cinta(self):
        """Aviso al operario que tiene que elegir una orientacion manual."""
        messagebox.showwarning(
            "Cinta MANUAL sin seleccion",
            "El robot pidio vision de la CINTA pero estas en modo MANUAL\n"
            "sin haber elegido ARRIBA / ABAJO / VACIO.\n\n"
            "Apreta uno de los botones manuales o cambia a AUTOMATICO.\n\n"
            "Mientras tanto, le respondi VACIO al robot."
        )

    def _mostrar_popup_manual_mesa(self):
        """Aviso al operario que tiene que elegir una posicion manual."""
        messagebox.showwarning(
            "Mesa MANUAL sin seleccion",
            "El robot pidio vision de la MESA pero estas en modo MANUAL\n"
            "sin haber tildado 'Activar' en fila/columna.\n\n"
            "Tilda el checkbox 'Activar' y elegi fila/columna,\n"
            "o cambia a AUTOMATICO.\n\n"
            "Mientras tanto, le respondi VACIO al robot."
        )

    def _actualizar_estado_loop(self):
        # Chequear si el server pidio popup manual
        if estado.pedir_popup_cinta:
            estado.pedir_popup_cinta = False
            self._mostrar_popup_manual_cinta()
        if estado.pedir_popup_mesa:
            estado.pedir_popup_mesa = False
            self._mostrar_popup_manual_mesa()

        if estado.robot_conectado:
            self.lbl_robot.configure(text="Robot: CONECTADO", fg='#7fff7f')
        else:
            self.lbl_robot.configure(text="Robot: desconectado", fg='#ff6b6b')

        self.lbl_consultas.configure(text=f"Consultas recibidas: {estado.consultas}")

        if detector_cinta.esta_activo():
            self.lbl_cam_cinta.configure(text="Camara cinta: conectada", fg='#7fff7f')
        else:
            self.lbl_cam_cinta.configure(text="Camara cinta: apagada", fg='#ff6b6b')

        if detector_mesa.esta_activo():
            self.lbl_cam_mesa.configure(text="Camara mesa: conectada", fg='#7fff7f')
        else:
            self.lbl_cam_mesa.configure(text="Camara mesa: apagada", fg='#ff6b6b')

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

        # Refrescar matriz Y panel bg subtract segun cual este visible
        self._dibujar_matriz(matriz if matriz is not None else matriz_vacia(),
                              fila_m, col_m)
        # Si la vista es bg subtract, tambien refrescar la imagen
        try:
            if self.var_vista_mesa.get() == "bg":
                self._refrescar_panel_bg_mesa()
        except AttributeError:
            pass

        if matriz is not None:
            n_piezas = sum(sum(1 for c in f if c) for f in matriz)
            self.lbl_piezas_mesa.configure(text=f"Piezas: {n_piezas}")
        else:
            n_piezas = 0
            self.lbl_piezas_mesa.configure(text="Piezas: ---")

        if fila_m > 0 and col_m > 0:
            self.lbl_deteccion_mesa.configure(
                text=f"Elegida: ({fila_m}, {col_m})", fg='#FAC775')
        else:
            self.lbl_deteccion_mesa.configure(text="Elegida: ---",
                                                fg='#cccccc')

        # ===== Refresco pesta\u00f1a TORNO =====
        try:
            if torno.conectado:
                self.lbl_torno_estado.configure(text="Estado: CONECTADO",
                                                 fg='#7fff7f')
            else:
                err = torno.ultimo_error or "desconectado"
                self.lbl_torno_estado.configure(
                    text=f"Estado: {err[:40]}", fg='#ff6b6b')
            self.lbl_torno_terminadas.configure(
                text=f"Piezas terminadas: {torno.piezas_terminadas}")
            self.lbl_torno_ultimo.configure(
                text=f"Ultimo evento: {torno.ultimo_evento or '---'}")
            self._refrescar_cola_torno()
        except AttributeError:
            # Pesta\u00f1a torno todavia no construida
            pass

        self.root.after(500, self._actualizar_estado_loop)

    def _ver_en_grande_cinta(self):
        """Abre ventana con la imagen actual de cinta a tamano completo."""
        with estado.lock:
            panel = estado.ultimo_panel_cinta
        if panel is None:
            messagebox.showinfo("Sin imagen",
                                 "Todavia no hay captura de cinta. Apreta "
                                 "'Capturar Ahora' primero.")
            return
        self._abrir_ventana_zoom(panel, "Cinta - BG Subtract (zoom)")

    def _ver_en_grande_mesa(self):
        """Abre ventana con la imagen actual de mesa (BG subtract) a tamano completo."""
        with estado.lock:
            panel = estado.ultimo_panel_mesa
        if panel is None:
            messagebox.showinfo("Sin imagen",
                                 "Todavia no hay captura de mesa. Apreta "
                                 "'Capturar Ahora' primero.")
            return
        self._abrir_ventana_zoom(panel, "Mesa - BG Subtract (zoom)")

    def _abrir_ventana_zoom(self, panel_bgr, titulo):
        """Abre una ventana Toplevel con la imagen y soporte de zoom +
        pan con drag.

        Controles:
          - Rueda del mouse: zoom in/out
          - Botones +/- : zoom
          - Boton "100%": volver a tamano original
          - Click-drag: pan
        """
        win = tk.Toplevel(self.root)
        win.title(titulo)
        win.configure(bg='#1a1a1a')
        # Tamano inicial razonable
        win.geometry("1000x700")

        # Convertir BGR -> RGB -> PIL para que se pueda escalar
        rgb = cv2.cvtColor(panel_bgr, cv2.COLOR_BGR2RGB)
        img_original = Image.fromarray(rgb)

        # Barra de herramientas arriba
        toolbar = tk.Frame(win, bg='#2b2b2b')
        toolbar.pack(side='top', fill='x')

        lbl_zoom = tk.Label(toolbar, text="Zoom: 100%",
                             font=("Arial", 10), bg='#2b2b2b', fg='white',
                             padx=10)
        lbl_zoom.pack(side='left')

        tk.Label(toolbar, text="(rueda del mouse: zoom | arrastrar: pan)",
                 font=("Arial", 9, "italic"),
                 bg='#2b2b2b', fg='#a0a0a0').pack(side='left', padx=10)

        # Canvas con scrollbars
        c_canvas = tk.Frame(win, bg='#1a1a1a')
        c_canvas.pack(fill='both', expand=True)

        h_scroll = tk.Scrollbar(c_canvas, orient='horizontal')
        v_scroll = tk.Scrollbar(c_canvas, orient='vertical')
        h_scroll.pack(side='bottom', fill='x')
        v_scroll.pack(side='right', fill='y')

        canvas = tk.Canvas(c_canvas, bg='#1a1a1a',
                            xscrollcommand=h_scroll.set,
                            yscrollcommand=v_scroll.set,
                            highlightthickness=0)
        canvas.pack(side='left', fill='both', expand=True)
        h_scroll.config(command=canvas.xview)
        v_scroll.config(command=canvas.yview)

        # Estado del zoom
        estado_zoom = {"factor": 1.0, "img_tk": None}

        def redibujar():
            factor = estado_zoom["factor"]
            w = int(img_original.width * factor)
            h = int(img_original.height * factor)
            # Limites
            if w < 50 or h < 50:
                return
            if w > 6000 or h > 6000:
                return
            img_scaled = img_original.resize((w, h), Image.LANCZOS)
            img_tk = ImageTk.PhotoImage(img_scaled)
            estado_zoom["img_tk"] = img_tk
            canvas.delete("all")
            canvas.create_image(0, 0, image=img_tk, anchor='nw')
            canvas.config(scrollregion=(0, 0, w, h))
            lbl_zoom.configure(text=f"Zoom: {int(factor * 100)}%")

        def zoom_in():
            estado_zoom["factor"] *= 1.25
            redibujar()

        def zoom_out():
            estado_zoom["factor"] *= 0.8
            redibujar()

        def zoom_100():
            estado_zoom["factor"] = 1.0
            redibujar()

        def fit_window():
            # Ajustar al tamano de la ventana
            cw = canvas.winfo_width()
            ch = canvas.winfo_height()
            if cw <= 1 or ch <= 1:
                return
            fx = cw / img_original.width
            fy = ch / img_original.height
            estado_zoom["factor"] = min(fx, fy)
            redibujar()

        # Botones zoom
        tk.Button(toolbar, text="−", width=3, command=zoom_out,
                  font=("Arial", 12, "bold")).pack(side='right', padx=2)
        tk.Button(toolbar, text="+", width=3, command=zoom_in,
                  font=("Arial", 12, "bold")).pack(side='right', padx=2)
        tk.Button(toolbar, text="100%", command=zoom_100).pack(side='right', padx=2)
        tk.Button(toolbar, text="Ajustar", command=fit_window).pack(side='right', padx=2)

        # Rueda del mouse zoom
        def on_wheel(event):
            # Windows / Mac
            if event.delta > 0:
                zoom_in()
            elif event.delta < 0:
                zoom_out()
        def on_wheel_linux_up(event):
            zoom_in()
        def on_wheel_linux_down(event):
            zoom_out()

        canvas.bind("<MouseWheel>", on_wheel)
        canvas.bind("<Button-4>", on_wheel_linux_up)
        canvas.bind("<Button-5>", on_wheel_linux_down)

        # Pan con drag
        def on_drag_start(event):
            canvas.scan_mark(event.x, event.y)

        def on_drag_move(event):
            canvas.scan_dragto(event.x, event.y, gain=1)

        canvas.bind("<ButtonPress-1>", on_drag_start)
        canvas.bind("<B1-Motion>", on_drag_move)

        # Cursor estilo "agarrar"
        canvas.configure(cursor="fleur")

        # Dibujado inicial: hacemos un dibujado a 100% YA para que se vea algo,
        # y luego ajustamos a la ventana cuando termine de renderizar.
        # update_idletasks() solo no alcanza porque el canvas todavia no tiene
        # tamano. Usamos after(50) para asegurar que ya quedo dibujado.
        redibujar()  # dibujar al 100% inmediatamente
        win.after(50, fit_window)  # ajustar al tamano de ventana despues

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
        for det in (detector_cinta, detector_mesa):
            try:
                if det.esta_activo():
                    det.stop()
            except Exception:
                pass
        try:
            torno.stop()
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