"""
Spirax Vision - Detector de orientacion (Background Subtraction)

Multi-modelo, multi-estacion:
    modelos/tipoN/cinta/  -> referencia.png + config.json para la cinta
    modelos/tipoN/mesa/   -> referencia.png + config.json para la mesa

La HMI le dice al detector "usa Tipo N en Estacion X" antes de cada
consulta y el detector levanta la referencia y la config correspondiente.

Estaciones validas:
    "cinta" -> analisis de orientacion (ARRIBA/ABAJO/VACIO)
    "mesa"  -> analisis de matriz 10x8 (ver mesa.py)

Migracion: si existe modelos/tipoN/referencia.png al nivel viejo, se mueve
automaticamente a modelos/tipoN/cinta/referencia.png al cargar.
"""

import cv2
import numpy as np
import sys
import os
import json
import shutil
import threading

USAR_REALSENSE = True

MODELOS_DIR = "modelos"
TIPOS_VALIDOS = (1, 2, 3, 4, 5, 6)
ESTACIONES_VALIDAS = ("cinta", "mesa")
BG_DIFF_THRESH_DEFAULT = 40

# Grilla overlay para visualizar en modo_recorte (solo mesa)
GRILLA_FILAS_OVERLAY = 8
GRILLA_COLS_OVERLAY = 10


# ======================================================================
#  Helpers de paths por tipo + estacion
# ======================================================================

def carpeta_tipo(tipo):
    """Devuelve la ruta a la carpeta base del tipo. La crea si no existe."""
    if tipo not in TIPOS_VALIDOS:
        raise ValueError(f"Tipo invalido: {tipo}. Validos: {TIPOS_VALIDOS}")
    ruta = os.path.join(MODELOS_DIR, f"tipo{tipo}")
    os.makedirs(ruta, exist_ok=True)
    return ruta


def carpeta_estacion(tipo, estacion):
    """Devuelve la ruta a modelos/tipoN/<estacion>/. La crea si no existe."""
    if estacion not in ESTACIONES_VALIDAS:
        raise ValueError(f"Estacion invalida: {estacion}. Validas: {ESTACIONES_VALIDAS}")
    base = carpeta_tipo(tipo)
    ruta = os.path.join(base, estacion)
    os.makedirs(ruta, exist_ok=True)

    # Migracion: si hay archivos viejos al nivel base (modelos/tipoN/
    # referencia.png o config.json), moverlos a cinta/ una sola vez
    if estacion == "cinta":
        _migrar_archivos_viejos(base, ruta)

    return ruta


def _migrar_archivos_viejos(base, destino_cinta):
    """Mueve archivos viejos modelos/tipoN/{referencia.png,config.json}
    a modelos/tipoN/cinta/ si no existen ya en destino.
    """
    for nombre in ("referencia.png", "config.json"):
        viejo = os.path.join(base, nombre)
        nuevo = os.path.join(destino_cinta, nombre)
        if os.path.exists(viejo) and not os.path.exists(nuevo):
            try:
                shutil.move(viejo, nuevo)
                print(f"[MIGRACION] Movido {viejo} -> {nuevo}")
            except Exception as e:
                print(f"[MIGRACION] Error moviendo {viejo}: {e}")


def path_referencia(tipo, estacion="cinta"):
    return os.path.join(carpeta_estacion(tipo, estacion), "referencia.png")


def path_config(tipo, estacion="cinta"):
    return os.path.join(carpeta_estacion(tipo, estacion), "config.json")


def tiene_referencia(tipo, estacion="cinta"):
    return os.path.exists(path_referencia(tipo, estacion))


def tiene_config(tipo, estacion="cinta"):
    return os.path.exists(path_config(tipo, estacion))


def cargar_config(tipo, estacion="cinta"):
    """Carga la config del tipo+estacion. Si no existe, devuelve defaults."""
    cfg_path = path_config(tipo, estacion)
    if os.path.exists(cfg_path):
        with open(cfg_path, "r") as f:
            cfg = json.load(f)
        cfg.setdefault("corte_y_top_pct", 0.0)
        cfg.setdefault("corte_y_pct", 1.0)
        cfg.setdefault("corte_x_izq_pct", 0.0)
        cfg.setdefault("corte_x_der_pct", 1.0)
        cfg.setdefault("exposure", 10)
        cfg.setdefault("gain", 60)
        cfg.setdefault("saturation", 50)
        cfg.setdefault("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT)
        cfg.setdefault("umbral_ocupacion", 0.15)
        # Grilla interna de la mesa: 9 verticales y 7 horizontales, en
        # porcentaje (0-1) dentro del cuadrilatero del recorte. Defaults
        # uniformes (1/10, 2/10, ... y 1/8, 2/8, ...).
        cfg.setdefault("grid_x", [i / GRILLA_COLS_OVERLAY
                                   for i in range(1, GRILLA_COLS_OVERLAY)])
        cfg.setdefault("grid_y", [i / GRILLA_FILAS_OVERLAY
                                   for i in range(1, GRILLA_FILAS_OVERLAY)])
        # Margen interno por celda (0.0 a 0.4). 0 = se analiza la celda
        # entera, 0.25 = se ignora el 25% de cada borde (queda el 50%
        # central).
        cfg.setdefault("margen_celda", 0.0)
        return cfg
    return {
        "corte_y_top_pct": 0.0,
        "corte_y_pct": 1.0,
        "corte_x_izq_pct": 0.0,
        "corte_x_der_pct": 1.0,
        "exposure": 10,
        "gain": 60,
        "saturation": 50,
        "bg_diff_thresh": BG_DIFF_THRESH_DEFAULT,
        "umbral_ocupacion": 0.15,
        "grid_x": [i / GRILLA_COLS_OVERLAY
                   for i in range(1, GRILLA_COLS_OVERLAY)],
        "grid_y": [i / GRILLA_FILAS_OVERLAY
                   for i in range(1, GRILLA_FILAS_OVERLAY)],
        "margen_celda": 0.0,
    }


def guardar_config(tipo, config, estacion="cinta"):
    cfg_path = path_config(tipo, estacion)
    with open(cfg_path, "w") as f:
        json.dump(config, f, indent=2)
    print(f"[OK] Config Tipo {tipo} ({estacion}) guardada en {cfg_path}")


# ======================================================================
#  CAPTURA (modo standalone)
# ======================================================================

def capturar_realsense(exposure=10, gain=60, saturation=50):
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)

    sensor = profile.get_device().query_sensors()[1]
    sensor.set_option(rs.option.enable_auto_exposure, 0)
    # 50 Hz para evitar flicker (1=50Hz Arg/Europa, 2=60Hz USA)
    try:
        sensor.set_option(rs.option.power_line_frequency, 1)
    except Exception:
        pass
    sensor.set_option(rs.option.exposure, exposure)
    sensor.set_option(rs.option.gain, gain)
    sensor.set_option(rs.option.saturation, saturation)

    print("[CAM] RealSense D435 capturando...")
    try:
        for _ in range(60):
            pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("No se obtuvo frame de color.")
        return np.asanyarray(color_frame.get_data())
    finally:
        pipeline.stop()


def capturar_webcam(dispositivo=0):
    cap = cv2.VideoCapture(dispositivo)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir camara.")
    for _ in range(10):
        cap.read()
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("No se pudo capturar.")
    return frame


def capturar(tipo=1, estacion="cinta"):
    """Captura una imagen usando los parametros del tipo+estacion."""
    cfg = cargar_config(tipo, estacion)
    if USAR_REALSENSE:
        return capturar_realsense(
            exposure=cfg.get("exposure", 10),
            gain=cfg.get("gain", 60),
            saturation=cfg.get("saturation", 50),
        )
    return capturar_webcam()


# ======================================================================
#  PREPROCESAMIENTO
# ======================================================================

def preparar_roi(imagen, config):
    """Aplica el recorte configurado y devuelve el ROI en gris + blur.

    Si la config tiene 'esquinas' (4 puntos), hace transformacion de
    perspectiva para convertir el cuadrilatero en rectangulo recto.
    Si no, usa el recorte rectangular tradicional con corte_*_pct.
    """
    h_orig, w_orig = imagen.shape[:2]

    esquinas_cfg = config.get("esquinas")
    if esquinas_cfg is not None:
        # Modo nuevo: warp perspective
        sup_izq = (esquinas_cfg["sup_izq"][0] * w_orig,
                   esquinas_cfg["sup_izq"][1] * h_orig)
        sup_der = (esquinas_cfg["sup_der"][0] * w_orig,
                   esquinas_cfg["sup_der"][1] * h_orig)
        inf_der = (esquinas_cfg["inf_der"][0] * w_orig,
                   esquinas_cfg["inf_der"][1] * h_orig)
        inf_izq = (esquinas_cfg["inf_izq"][0] * w_orig,
                   esquinas_cfg["inf_izq"][1] * h_orig)

        # Tamano del rectangulo destino: promedio de los lados opuestos
        ancho_sup = ((sup_der[0] - sup_izq[0]) ** 2 +
                     (sup_der[1] - sup_izq[1]) ** 2) ** 0.5
        ancho_inf = ((inf_der[0] - inf_izq[0]) ** 2 +
                     (inf_der[1] - inf_izq[1]) ** 2) ** 0.5
        alto_izq = ((inf_izq[0] - sup_izq[0]) ** 2 +
                    (inf_izq[1] - sup_izq[1]) ** 2) ** 0.5
        alto_der = ((inf_der[0] - sup_der[0]) ** 2 +
                    (inf_der[1] - sup_der[1]) ** 2) ** 0.5

        ancho_destino = int(max(ancho_sup, ancho_inf))
        alto_destino = int(max(alto_izq, alto_der))

        if ancho_destino < 10 or alto_destino < 10:
            # Esquinas degeneradas, caer al recorte rectangular
            return _preparar_roi_legacy(imagen, config)

        pts_src = np.float32([sup_izq, sup_der, inf_der, inf_izq])
        pts_dst = np.float32([[0, 0],
                              [ancho_destino, 0],
                              [ancho_destino, alto_destino],
                              [0, alto_destino]])

        matriz = cv2.getPerspectiveTransform(pts_src, pts_dst)
        roi_color = cv2.warpPerspective(imagen, matriz,
                                         (ancho_destino, alto_destino))
        gris = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gris, (7, 7), 0)
        return roi_color, blur

    # Modo legacy: rectangulo recto
    return _preparar_roi_legacy(imagen, config)


def _preparar_roi_legacy(imagen, config):
    """Recorte rectangular tradicional (compatibilidad)."""
    h_orig, w_orig = imagen.shape[:2]
    corte_y_top = int(h_orig * config.get("corte_y_top_pct", 0.0))
    corte_y_bot = int(h_orig * config.get("corte_y_pct", 1.0))
    corte_x_izq = int(w_orig * config.get("corte_x_izq_pct", 0.0))
    corte_x_der = int(w_orig * config.get("corte_x_der_pct", 1.0))
    roi_color = imagen[corte_y_top:corte_y_bot, corte_x_izq:corte_x_der]
    gris = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gris, (7, 7), 0)
    return roi_color, blur


# ======================================================================
#  RECORTE INTERACTIVO
# ======================================================================

def modo_recorte(imagen, tipo, estacion="cinta"):
    """Permite ajustar el recorte como cuadrilatero de 4 esquinas.

    Controles:
      - Click + drag sobre una esquina (puntos amarillos): mueve esa esquina
      - Click + drag sobre una linea EXTERNA: mueve esa linea
      - Click + drag sobre una linea INTERNA (solo mesa): mueve esa linea
      - Shift + click + drag: mueve TODO el cuadrilatero
      - Ctrl + click + drag: ROTA todo alrededor del centro
      - Tecla R: resetea al recorte original (rectangulo lleno)
      - Tecla G (solo mesa): toggle ver/ocultar grilla interna
      - Tecla D (solo mesa): reset grilla interna a posiciones uniformes
      - Tecla S: guarda
      - Tecla Q / ESC: cancela

    Compatibilidad con versiones viejas: si la config solo tiene los 4
    porcentajes (corte_y_top_pct etc.), se construyen las 4 esquinas como
    rectangulo recto. Cuando se guarda, se guardan AMBOS formatos.
    """
    config = cargar_config(tipo, estacion)
    h, w = imagen.shape[:2]

    # Cargar esquinas. Si no existen, construirlas desde el recorte legacy.
    esquinas_cfg = config.get("esquinas")
    if esquinas_cfg is None:
        y_top = config.get("corte_y_top_pct", 0.0) * h
        y_bot = config.get("corte_y_pct", 1.0) * h
        x_izq = config.get("corte_x_izq_pct", 0.0) * w
        x_der = config.get("corte_x_der_pct", 1.0) * w
        # Margenes minimos
        if y_top < 5: y_top = 5
        if y_bot > h - 5: y_bot = h - 5
        if x_izq < 5: x_izq = 5
        if x_der > w - 5: x_der = w - 5
        esquinas = [
            [x_izq, y_top],   # 0 = sup_izq
            [x_der, y_top],   # 1 = sup_der
            [x_der, y_bot],   # 2 = inf_der
            [x_izq, y_bot],   # 3 = inf_izq
        ]
    else:
        # Esquinas guardadas en porcentaje
        esquinas = [
            [esquinas_cfg["sup_izq"][0] * w, esquinas_cfg["sup_izq"][1] * h],
            [esquinas_cfg["sup_der"][0] * w, esquinas_cfg["sup_der"][1] * h],
            [esquinas_cfg["inf_der"][0] * w, esquinas_cfg["inf_der"][1] * h],
            [esquinas_cfg["inf_izq"][0] * w, esquinas_cfg["inf_izq"][1] * h],
        ]

    # Snapshot original para reset
    esquinas_orig = [list(p) for p in esquinas]

    # Grilla interna (solo mesa). Cargada como lista de porcentajes
    # dentro del cuadrilatero. Los defaults son uniformes.
    es_mesa = (estacion == "mesa")
    if es_mesa:
        grid_x = list(config.get(
            "grid_x",
            [i / GRILLA_COLS_OVERLAY for i in range(1, GRILLA_COLS_OVERLAY)]))
        grid_y = list(config.get(
            "grid_y",
            [i / GRILLA_FILAS_OVERLAY for i in range(1, GRILLA_FILAS_OVERLAY)]))
        # Asegurar longitudes correctas (por si la config esta corrupta)
        if len(grid_x) != GRILLA_COLS_OVERLAY - 1:
            grid_x = [i / GRILLA_COLS_OVERLAY
                      for i in range(1, GRILLA_COLS_OVERLAY)]
        if len(grid_y) != GRILLA_FILAS_OVERLAY - 1:
            grid_y = [i / GRILLA_FILAS_OVERLAY
                      for i in range(1, GRILLA_FILAS_OVERLAY)]
    else:
        grid_x = []
        grid_y = []

    # Estado de interaccion
    estado = {
        "modo": None,            # 'esquina', 'linea', 'linea_int_v',
                                 # 'linea_int_h', 'mover_todo', 'rotar'
        "indice": None,          # indice del elemento agarrado
        "last_x": 0,
        "last_y": 0,
        "shift": False,
        "ctrl": False,
        "show_grid": True,
        "grid_filas": GRILLA_FILAS_OVERLAY if es_mesa else 0,
        "grid_cols": GRILLA_COLS_OVERLAY if es_mesa else 0,
    }

    def dist_punto_punto(p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def dist_punto_segmento(p, a, b):
        ax, ay = a
        bx, by = b
        dx = bx - ax
        dy = by - ay
        if dx == 0 and dy == 0:
            return dist_punto_punto(p, a)
        t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx = ax + t * dx
        cy = ay + t * dy
        return ((p[0] - cx) ** 2 + (p[1] - cy) ** 2) ** 0.5

    def centro_actual():
        cx = sum(p[0] for p in esquinas) / 4
        cy = sum(p[1] for p in esquinas) / 4
        return cx, cy

    def lineas_internas_segmentos():
        """Devuelve dos listas de segmentos para las lineas internas.

        verticales: lista de ((x_top, y_top), (x_bot, y_bot)) para cada
                    grid_x. Va de borde superior a borde inferior del cuad.
        horizontales: idem con grid_y, va de borde izq a borde der.
        """
        verticales = []
        for t in grid_x:
            p_top = (esquinas[0][0] + (esquinas[1][0] - esquinas[0][0]) * t,
                     esquinas[0][1] + (esquinas[1][1] - esquinas[0][1]) * t)
            p_bot = (esquinas[3][0] + (esquinas[2][0] - esquinas[3][0]) * t,
                     esquinas[3][1] + (esquinas[2][1] - esquinas[3][1]) * t)
            verticales.append((p_top, p_bot))
        horizontales = []
        for t in grid_y:
            p_izq = (esquinas[0][0] + (esquinas[3][0] - esquinas[0][0]) * t,
                     esquinas[0][1] + (esquinas[3][1] - esquinas[0][1]) * t)
            p_der = (esquinas[1][0] + (esquinas[2][0] - esquinas[1][0]) * t,
                     esquinas[1][1] + (esquinas[2][1] - esquinas[1][1]) * t)
            horizontales.append((p_izq, p_der))
        return verticales, horizontales

    def xy_a_uv(x, y):
        """Mapea un punto (x,y) en pixels al sistema (u,v) del cuadrilatero
        donde u=0 es lado izq, u=1 es lado der, v=0 es lado sup, v=1 inf.
        Robusto a rotacion y perspectiva.
        """
        src = np.array([esquinas[0], esquinas[1],
                        esquinas[2], esquinas[3]], dtype=np.float32)
        dst = np.array([[0.0, 0.0], [1.0, 0.0],
                        [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
        try:
            M = cv2.getPerspectiveTransform(src, dst)
        except Exception:
            return None, None
        pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
        res = cv2.perspectiveTransform(pt, M)
        return float(res[0][0][0]), float(res[0][0][1])

    def on_mouse(event, x, y, flags, param):
        # Detectar Shift/Ctrl en el momento del click
        shift = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
        ctrl = bool(flags & cv2.EVENT_FLAG_CTRLKEY)

        if event == cv2.EVENT_LBUTTONDOWN:
            estado["shift"] = shift
            estado["ctrl"] = ctrl
            estado["last_x"] = x
            estado["last_y"] = y

            if ctrl:
                estado["modo"] = "rotar"
                estado["indice"] = None
                return
            if shift:
                estado["modo"] = "mover_todo"
                estado["indice"] = None
                return

            # Buscar esquina cercana (mayor prioridad)
            tol_esquina = 18
            mejor_i = None
            mejor_d = tol_esquina
            for i, p in enumerate(esquinas):
                d = dist_punto_punto([x, y], p)
                if d < mejor_d:
                    mejor_d = d
                    mejor_i = i
            if mejor_i is not None:
                estado["modo"] = "esquina"
                estado["indice"] = mejor_i
                return

            # Buscar linea EXTERNA cercana
            tol_linea = 12
            mejor_l = None
            mejor_dl = tol_linea
            for i in range(4):
                a = esquinas[i]
                b = esquinas[(i + 1) % 4]
                d = dist_punto_segmento([x, y], a, b)
                if d < mejor_dl:
                    mejor_dl = d
                    mejor_l = i

            # Buscar linea INTERNA cercana (solo mesa con grilla visible)
            mejor_iv = None
            mejor_div = 10  # tolerancia para internas
            mejor_ih = None
            mejor_dih = 10
            if es_mesa and estado["show_grid"]:
                verticales, horizontales = lineas_internas_segmentos()
                for i, (a, b) in enumerate(verticales):
                    d = dist_punto_segmento([x, y], a, b)
                    if d < mejor_div:
                        mejor_div = d
                        mejor_iv = i
                for i, (a, b) in enumerate(horizontales):
                    d = dist_punto_segmento([x, y], a, b)
                    if d < mejor_dih:
                        mejor_dih = d
                        mejor_ih = i

            # Elegir el match mas cercano entre las tres opciones
            opciones = []
            if mejor_l is not None:
                opciones.append(("linea", mejor_l, mejor_dl))
            if mejor_iv is not None:
                opciones.append(("linea_int_v", mejor_iv, mejor_div))
            if mejor_ih is not None:
                opciones.append(("linea_int_h", mejor_ih, mejor_dih))

            if opciones:
                opciones.sort(key=lambda o: o[2])
                modo_, idx_, _ = opciones[0]
                estado["modo"] = modo_
                estado["indice"] = idx_
                return

            # Nada agarrado
            estado["modo"] = None
            estado["indice"] = None

        elif event == cv2.EVENT_MOUSEMOVE:
            if estado["modo"] is None:
                return
            dx = x - estado["last_x"]
            dy = y - estado["last_y"]
            estado["last_x"] = x
            estado["last_y"] = y

            if estado["modo"] == "esquina":
                i = estado["indice"]
                esquinas[i][0] = max(0, min(w - 1, esquinas[i][0] + dx))
                esquinas[i][1] = max(0, min(h - 1, esquinas[i][1] + dy))

            elif estado["modo"] == "linea":
                i = estado["indice"]
                # Mover los dos extremos de la linea
                a_i = i
                b_i = (i + 1) % 4
                for idx in (a_i, b_i):
                    esquinas[idx][0] = max(0, min(w - 1, esquinas[idx][0] + dx))
                    esquinas[idx][1] = max(0, min(h - 1, esquinas[idx][1] + dy))

            elif estado["modo"] == "linea_int_v":
                # Arrastrar linea interna vertical: el nuevo grid_x es la
                # coordenada u del puntero dentro del cuadrilatero.
                i = estado["indice"]
                u, _v = xy_a_uv(x, y)
                if u is not None:
                    eps = 0.005
                    minimo = grid_x[i - 1] + eps if i > 0 else eps
                    maximo = (grid_x[i + 1] - eps
                              if i < len(grid_x) - 1 else 1.0 - eps)
                    grid_x[i] = max(minimo, min(maximo, u))

            elif estado["modo"] == "linea_int_h":
                i = estado["indice"]
                _u, v = xy_a_uv(x, y)
                if v is not None:
                    eps = 0.005
                    minimo = grid_y[i - 1] + eps if i > 0 else eps
                    maximo = (grid_y[i + 1] - eps
                              if i < len(grid_y) - 1 else 1.0 - eps)
                    grid_y[i] = max(minimo, min(maximo, v))

            elif estado["modo"] == "mover_todo":
                for p in esquinas:
                    p[0] = max(0, min(w - 1, p[0] + dx))
                    p[1] = max(0, min(h - 1, p[1] + dy))

            elif estado["modo"] == "rotar":
                cx, cy = centro_actual()
                # Calcular angulo entre punto anterior y nuevo respecto al centro
                import math
                ang_prev = math.atan2(estado["last_y"] - dy - cy,
                                       estado["last_x"] - dx - cx)
                ang_new = math.atan2(y - cy, x - cx)
                d_ang = ang_new - ang_prev
                cos_a = math.cos(d_ang)
                sin_a = math.sin(d_ang)
                for p in esquinas:
                    px = p[0] - cx
                    py = p[1] - cy
                    p[0] = cx + px * cos_a - py * sin_a
                    p[1] = cy + px * sin_a + py * cos_a
                    p[0] = max(0, min(w - 1, p[0]))
                    p[1] = max(0, min(h - 1, p[1]))

        elif event == cv2.EVENT_LBUTTONUP:
            estado["modo"] = None
            estado["indice"] = None
            estado["shift"] = False
            estado["ctrl"] = False

    win = f"Recorte Tipo {tipo} ({estacion}) | drag esquinas/lineas | Shift=mover | Ctrl=rotar | R=reset S=guardar Q=salir"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        debug = imagen.copy()

        # Crear mascara del cuadrilatero para oscurecer afuera
        mask_full = np.zeros((h, w), dtype=np.uint8)
        pts_int = np.array([[int(p[0]), int(p[1])] for p in esquinas],
                            dtype=np.int32)
        cv2.fillPoly(mask_full, [pts_int], 255)

        # Oscurecer afuera
        overlay = debug.copy()
        overlay[mask_full == 0] = (overlay[mask_full == 0] * 0.4).astype(np.uint8)
        debug = overlay

        # Dibujar contorno del cuadrilatero
        cv2.polylines(debug, [pts_int], True, (0, 255, 255), 2)

        # Dibujar grilla 10x8 si es mesa y show_grid esta activo
        if es_mesa and estado["show_grid"]:
            verticales, horizontales = lineas_internas_segmentos()

            # Si estoy arrastrando una linea interna, resalto esa en cian
            # vivo y las demas en color habitual.
            modo_act = estado["modo"]
            idx_act = estado["indice"]

            for i, (p_top, p_bot) in enumerate(verticales):
                color = (50, 200, 255)
                grosor = 1
                if modo_act == "linea_int_v" and idx_act == i:
                    color = (0, 255, 255)
                    grosor = 2
                cv2.line(debug,
                         (int(p_top[0]), int(p_top[1])),
                         (int(p_bot[0]), int(p_bot[1])),
                         color, grosor)
            for i, (p_izq, p_der) in enumerate(horizontales):
                color = (50, 200, 255)
                grosor = 1
                if modo_act == "linea_int_h" and idx_act == i:
                    color = (0, 255, 255)
                    grosor = 2
                cv2.line(debug,
                         (int(p_izq[0]), int(p_izq[1])),
                         (int(p_der[0]), int(p_der[1])),
                         color, grosor)

        # Dibujar esquinas como puntitos amarillos
        labels = ["SI", "SD", "ID", "II"]  # sup-izq, sup-der, inf-der, inf-izq
        for i, p in enumerate(esquinas):
            cv2.circle(debug, (int(p[0]), int(p[1])), 8, (0, 255, 255), -1)
            cv2.circle(debug, (int(p[0]), int(p[1])), 9, (0, 0, 0), 1)
            cv2.putText(debug, labels[i],
                        (int(p[0]) + 12, int(p[1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # Texto de ayuda
        if es_mesa:
            ayuda_top = (f"Tipo {tipo} (MESA) | Drag esquina/linea ext/linea int"
                         " | Shift=mover Ctrl=rotar")
            ayuda_bot = ("R=reset esquinas  D=reset grilla  "
                         "G=mostrar/ocultar grilla  S=guardar  Q=salir")
        else:
            ayuda_top = (f"Tipo {tipo} (CINTA) | Drag esquina/linea"
                         " | Shift=mover Ctrl=rotar")
            ayuda_bot = "R=reset  S=guardar  Q=salir"
        cv2.putText(debug, ayuda_top,
                    (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(debug, ayuda_bot,
                    (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow(win, debug)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('s'):
            # Guardar las 4 esquinas en porcentaje
            config["esquinas"] = {
                "sup_izq": [round(esquinas[0][0] / w, 4),
                             round(esquinas[0][1] / h, 4)],
                "sup_der": [round(esquinas[1][0] / w, 4),
                             round(esquinas[1][1] / h, 4)],
                "inf_der": [round(esquinas[2][0] / w, 4),
                             round(esquinas[2][1] / h, 4)],
                "inf_izq": [round(esquinas[3][0] / w, 4),
                             round(esquinas[3][1] / h, 4)],
            }
            # Mantener compatibilidad: tambien actualizar el bounding box
            # rectangular como aproximacion para codigo viejo que aun lea
            # los corte_*_pct.
            xs = [p[0] for p in esquinas]
            ys = [p[1] for p in esquinas]
            config["corte_y_top_pct"] = round(min(ys) / h, 4)
            config["corte_y_pct"] = round(max(ys) / h, 4)
            config["corte_x_izq_pct"] = round(min(xs) / w, 4)
            config["corte_x_der_pct"] = round(max(xs) / w, 4)
            # Guardar grilla interna (solo si es mesa)
            if es_mesa:
                config["grid_x"] = [round(v, 4) for v in grid_x]
                config["grid_y"] = [round(v, 4) for v in grid_y]
            guardar_config(tipo, config, estacion)
            cv2.destroyAllWindows()
            return True

        elif key == ord('r'):
            # Reset esquinas al original. La grilla interna NO se toca
            # (usar D para eso).
            for i in range(4):
                esquinas[i][0] = esquinas_orig[i][0]
                esquinas[i][1] = esquinas_orig[i][1]

        elif key == ord('g') and es_mesa:
            # Toggle visibilidad de la grilla interna
            estado["show_grid"] = not estado["show_grid"]

        elif key == ord('d') and es_mesa:
            # Reset grilla interna a posiciones uniformes
            for i in range(GRILLA_COLS_OVERLAY - 1):
                grid_x[i] = (i + 1) / GRILLA_COLS_OVERLAY
            for i in range(GRILLA_FILAS_OVERLAY - 1):
                grid_y[i] = (i + 1) / GRILLA_FILAS_OVERLAY

        elif key == ord('q') or key == 27:
            print("[X] Recorte cancelado.")
            cv2.destroyAllWindows()
            return False


# ======================================================================
#  BACKGROUND SUBTRACTION
# ======================================================================

def metodo_d_bgsub(blur, referencia_blur, diff_thresh):
    if referencia_blur is None:
        return None
    if blur.shape != referencia_blur.shape:
        referencia_blur = cv2.resize(referencia_blur, (blur.shape[1], blur.shape[0]))
    diff = cv2.absdiff(blur, referencia_blur)
    _, thresh = cv2.threshold(diff, diff_thresh, 255, cv2.THRESH_BINARY)
    return thresh


# ======================================================================
#  POSTPROCESO (analisis de orientacion - solo cinta)
# ======================================================================

def procesar_mascara(thresh):
    if thresh is None:
        return None, None, None, None, "N/A", None, None

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return mask, None, None, None, "VACIO", None, None

    rh, rw = mask.shape[:2]
    roi_area = rh * rw

    candidatos = [c for c in contornos
                  if cv2.contourArea(c) >= roi_area * 0.01
                  and cv2.boundingRect(c)[2] <= rw * 0.90]

    if not candidatos:
        return mask, None, None, None, "VACIO", None, None

    contorno = max(candidatos, key=cv2.contourArea)
    area = cv2.contourArea(contorno)
    if area < roi_area * 0.05:
        return mask, None, None, None, "VACIO", None, None

    bbox = cv2.boundingRect(contorno)
    bx, by, bw, bh = bbox

    mascara_pieza = np.zeros((rh, rw), dtype=np.uint8)
    cv2.drawContours(mascara_pieza, [contorno], -1, 255, -1)

    anchos = []
    for row in range(by, by + bh):
        fila = mascara_pieza[row, :]
        pixels = np.where(fila > 0)[0]
        if len(pixels) >= 2:
            anchos.append((row, int(pixels[0]), int(pixels[-1]), int(pixels[-1] - pixels[0])))

    if len(anchos) < 5:
        return mask, contorno, bbox, None, "DUDOSO", None, None

    ancho_max_raw = max(a[3] for a in anchos)
    umbral = ancho_max_raw * 0.10
    validos = [a for a in anchos if a[3] > umbral]
    if len(validos) < 5:
        return mask, contorno, bbox, None, "DUDOSO", None, None

    ventana = 30
    promedios = []
    for i in range(0, len(validos) - ventana + 1):
        grupo = validos[i:i + ventana]
        avg = np.mean([a[3] for a in grupo])
        centro = grupo[len(grupo) // 2]
        promedios.append((centro[0], centro[1], centro[2], avg))

    if len(promedios) < 2:
        return mask, contorno, bbox, None, "DUDOSO", None, None

    fila_max_p = max(promedios, key=lambda x: x[3])
    separacion_min = bh * 0.05
    candidatas = [p for p in promedios if abs(p[0] - fila_max_p[0]) > separacion_min]
    if not candidatas:
        return mask, contorno, bbox, None, "DUDOSO", None, None

    fila_min_p = min(candidatas, key=lambda x: x[3])
    orientacion = "ARRIBA" if fila_min_p[0] < fila_max_p[0] else "ABAJO"
    ratio = fila_min_p[3] / fila_max_p[3] if fila_max_p[3] > 0 else 0

    fila_max = (fila_max_p[0], fila_max_p[1], fila_max_p[2], int(fila_max_p[3]))
    fila_min = (fila_min_p[0], fila_min_p[1], fila_min_p[2], int(fila_min_p[3]))

    return mask, contorno, bbox, ratio, orientacion, fila_max, fila_min


# ======================================================================
#  VISUALIZACION
# ======================================================================

def panel_debug(roi_color, mask, contorno, bbox, ratio, decision, nombre,
                fila_max=None, fila_min=None):
    h, w = roi_color.shape[:2]
    if mask is None:
        # Imagen con franja arriba
        franja = np.full((45, w, 3), (60, 60, 60), dtype=np.uint8)
        cv2.putText(franja, f"{nombre}: N/A", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return np.vstack([franja, roi_color])

    mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    panel = cv2.addWeighted(roi_color, 0.5, mask_color, 0.5, 0)

    if contorno is not None:
        cv2.drawContours(panel, [contorno], -1, (0, 255, 0), 2)
    if bbox is not None:
        bx, by, bw, bh = bbox
        cv2.rectangle(panel, (bx, by), (bx + bw, by + bh), (255, 0, 0), 1)

    def dibujar_medicion(fila, color):
        if fila is None:
            return
        row, x1, x2, ancho = fila
        cv2.line(panel, (x1, row), (x2, row), color, 2)
        cv2.line(panel, (x1, row - 6), (x1, row + 6), color, 2)
        cv2.line(panel, (x2, row - 6), (x2, row + 6), color, 2)
        cv2.putText(panel, f"{ancho}px", (x2 + 5, row + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    dibujar_medicion(fila_max, (0, 0, 255))
    dibujar_medicion(fila_min, (255, 0, 255))

    colores = {"ARRIBA": (0, 200, 0), "ABAJO": (0, 0, 255),
               "VACIO": (150, 150, 150), "DUDOSO": (0, 165, 255),
               "N/A": (128, 128, 128)}
    color = colores.get(decision, (255, 255, 255))

    # Franja arriba SEPARADA de la imagen (no la tapa)
    franja = np.full((45, w, 3), color, dtype=np.uint8)
    cv2.putText(franja, f"{nombre}: {decision}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    ratio_txt = f"ratio: {ratio:.3f}" if ratio is not None else "ratio: ---"
    cv2.putText(franja, ratio_txt, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    return np.vstack([franja, panel])


def analizar_imagen(imagen, referencia_blur, config, diff_thresh=None):
    """Pipeline completo de orientacion (cinta) sobre una imagen ya
    capturada."""
    if diff_thresh is None:
        diff_thresh = config.get("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT)
    roi_color, blur = preparar_roi(imagen, config)
    thresh = metodo_d_bgsub(blur, referencia_blur, diff_thresh=diff_thresh)
    mask, contorno, bbox, ratio, decision, fmax, fmin = procesar_mascara(thresh)
    panel = panel_debug(roi_color, mask, contorno, bbox, ratio, decision,
                        "BG Subtract", fmax, fmin)
    return {
        "orientacion": decision,
        "ratio": ratio,
        "panel": panel,
    }


# ======================================================================
#  CLASE PARA USO DESDE LA HMI (camara persistente)
# ======================================================================

class DetectorOrientacion:
    """Mantiene la camara abierta y permite analizar bajo demanda.

    Multi-modelo, multi-estacion: usa_tipo(N, "cinta"|"mesa") hace que
    las siguientes capturas/analisis usen la referencia y config
    del Tipo N + Estacion.
    """

    def __init__(self, tipo_inicial=1, estacion_inicial="cinta"):
        self._pipeline = None
        self._sensor = None
        self._lock = threading.Lock()
        self._activo = False

        self._tipo = tipo_inicial
        self._estacion = estacion_inicial
        self._config = cargar_config(tipo_inicial, estacion_inicial)
        self._referencia_blur = None

    # ---------- API ----------

    def usa_tipo(self, tipo, estacion=None):
        """Cambia tipo y/o estacion activa. Recarga config y referencia."""
        if tipo not in TIPOS_VALIDOS:
            raise ValueError(f"Tipo invalido: {tipo}")
        if estacion is not None and estacion not in ESTACIONES_VALIDAS:
            raise ValueError(f"Estacion invalida: {estacion}")

        with self._lock:
            self._tipo = tipo
            if estacion is not None:
                self._estacion = estacion
            self._config = cargar_config(self._tipo, self._estacion)

            ref_path = path_referencia(self._tipo, self._estacion)
            if os.path.exists(ref_path):
                ref = cv2.imread(ref_path)
                if ref is not None:
                    _, self._referencia_blur = preparar_roi(ref, self._config)
                else:
                    self._referencia_blur = None
            else:
                self._referencia_blur = None

            if self._activo and USAR_REALSENSE and self._sensor is not None:
                try:
                    import pyrealsense2 as rs
                    self._sensor.set_option(rs.option.exposure,
                                            int(self._config.get("exposure", 10)))
                    self._sensor.set_option(rs.option.gain,
                                            int(self._config.get("gain", 60)))
                    self._sensor.set_option(rs.option.saturation,
                                            int(self._config.get("saturation", 50)))
                except Exception:
                    pass

    def tipo_activo(self):
        with self._lock:
            return self._tipo

    def estacion_activa(self):
        with self._lock:
            return self._estacion

    def tiene_referencia_activa(self):
        with self._lock:
            return self._referencia_blur is not None

    # ---------- start/stop ----------

    def start(self, requiere_referencia=True):
        with self._lock:
            if self._activo:
                return

            ref_path = path_referencia(self._tipo, self._estacion)
            if os.path.exists(ref_path):
                ref = cv2.imread(ref_path)
                if ref is None:
                    raise RuntimeError(f"No se pudo leer {ref_path}")
                _, self._referencia_blur = preparar_roi(ref, self._config)
            else:
                if requiere_referencia:
                    raise FileNotFoundError(
                        f"Falta referencia para Tipo {self._tipo} "
                        f"({self._estacion}).\nEsperada en: {ref_path}\n"
                        f"Capturala desde la pestania Calibrar."
                    )
                self._referencia_blur = None

            if USAR_REALSENSE:
                import pyrealsense2 as rs
                self._pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                profile = self._pipeline.start(cfg)

                sensor = profile.get_device().query_sensors()[1]
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                # 50 Hz para evitar flicker (1=50Hz Arg/Europa, 2=60Hz USA)
                try:
                    sensor.set_option(rs.option.power_line_frequency, 1)
                except Exception:
                    pass
                sensor.set_option(rs.option.exposure,
                                  int(self._config.get("exposure", 10)))
                sensor.set_option(rs.option.gain,
                                  int(self._config.get("gain", 60)))
                sensor.set_option(rs.option.saturation,
                                  int(self._config.get("saturation", 50)))
                self._sensor = sensor

                for _ in range(60):
                    self._pipeline.wait_for_frames()
            else:
                self._pipeline = cv2.VideoCapture(0)
                if not self._pipeline.isOpened():
                    raise RuntimeError("No se pudo abrir webcam.")
                for _ in range(10):
                    self._pipeline.read()

            self._activo = True

    def stop(self):
        with self._lock:
            if not self._activo:
                return
            try:
                if USAR_REALSENSE:
                    self._pipeline.stop()
                else:
                    self._pipeline.release()
            except Exception:
                pass
            self._pipeline = None
            self._sensor = None
            self._activo = False

    def esta_activo(self):
        return self._activo

    # ---------- captura y analisis ----------

    def _capturar_frame_raw(self, descartar_buffer=False):
        if USAR_REALSENSE:
            if descartar_buffer:
                for _ in range(5):
                    self._pipeline.wait_for_frames()
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("Frame de color no disponible.")
            return np.asanyarray(color_frame.get_data())
        else:
            ret, frame = self._pipeline.read()
            if not ret:
                raise RuntimeError("No se pudo leer frame de webcam.")
            return frame

    def analizar(self):
        """Captura un frame y lo analiza segun la estacion activa.

        Para estacion="cinta": devuelve dict {tipo, orientacion, ratio, panel}.
        Para estacion="mesa":  devuelve dict {tipo, matriz, fila, columna, panel}.
        """
        with self._lock:
            if not self._activo:
                raise RuntimeError("Detector no activo. Llamar start() primero.")
            frame = self._capturar_frame_raw(descartar_buffer=True)
            ref = self._referencia_blur
            cfg = self._config
            tipo = self._tipo
            est = self._estacion

        if est == "mesa":
            # Importacion local para evitar circular
            from mesa import analizar_imagen_mesa
            if ref is None:
                roi_color, _ = preparar_roi(frame, cfg)
                panel = roi_color.copy()
                cv2.putText(panel, f"Tipo {tipo} (mesa): SIN REFERENCIA",
                            (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                return {
                    "tipo": tipo,
                    "matriz": None,
                    "fila": 0,
                    "columna": 0,
                    "panel": panel,
                }
            res = analizar_imagen_mesa(frame, ref, cfg)
            res["tipo"] = tipo
            return res

        # Estacion cinta
        if ref is None:
            roi_color, _ = preparar_roi(frame, cfg)
            panel = roi_color.copy()
            cv2.putText(panel, f"Tipo {tipo} (cinta): SIN REFERENCIA",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return {
                "tipo": tipo,
                "orientacion": "N/A",
                "ratio": None,
                "panel": panel,
            }

        res = analizar_imagen(frame, ref, cfg)
        res["tipo"] = tipo
        return res

    def capturar_frame(self):
        with self._lock:
            if not self._activo:
                raise RuntimeError("Detector no activo. Llamar start() primero.")
            return self._capturar_frame_raw(descartar_buffer=False)

    # ---------- ajuste y persistencia ----------

    def _persistir(self):
        try:
            guardar_config(self._tipo, self._config, self._estacion)
        except Exception:
            pass

    def set_exposure(self, valor, persist=True):
        valor = int(valor)
        with self._lock:
            self._config["exposure"] = valor
            if self._activo and USAR_REALSENSE and self._sensor is not None:
                try:
                    import pyrealsense2 as rs
                    self._sensor.set_option(rs.option.exposure, valor)
                except Exception:
                    pass
        if persist:
            with self._lock:
                self._persistir()

    def set_gain(self, valor, persist=True):
        valor = int(valor)
        with self._lock:
            self._config["gain"] = valor
            if self._activo and USAR_REALSENSE and self._sensor is not None:
                try:
                    import pyrealsense2 as rs
                    self._sensor.set_option(rs.option.gain, valor)
                except Exception:
                    pass
        if persist:
            with self._lock:
                self._persistir()

    def set_threshold(self, valor, persist=True):
        with self._lock:
            self._config["bg_diff_thresh"] = int(valor)
        if persist:
            with self._lock:
                self._persistir()

    def set_umbral_ocupacion(self, valor, persist=True):
        """valor entero 0-100 (porcentaje). Se guarda como 0.0-1.0 en config."""
        valor_pct = max(0, min(100, int(valor)))
        with self._lock:
            self._config["umbral_ocupacion"] = valor_pct / 100.0
        if persist:
            with self._lock:
                self._persistir()

    def set_margen_celda(self, valor, persist=True):
        """valor entero 0-40 (porcentaje). Se guarda como 0.0-0.4 en config.

        Solo aplica a mesa. Se recorta ese porcentaje de cada borde de la
        celda antes de contar pixels. 0 = celda entera, 25 = solo el 50%
        central.
        """
        valor_pct = max(0, min(40, int(valor)))
        with self._lock:
            self._config["margen_celda"] = valor_pct / 100.0
        if persist:
            with self._lock:
                self._persistir()

    @property
    def exposure(self):
        return int(self._config.get("exposure", 10))

    @property
    def gain(self):
        return int(self._config.get("gain", 60))

    @property
    def bg_diff_thresh(self):
        return int(self._config.get("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT))

    @property
    def umbral_ocupacion(self):
        """Devuelve el porcentaje 0-100 (no la fraccion 0.0-1.0)."""
        return int(self._config.get("umbral_ocupacion", 0.15) * 100)

    @property
    def margen_celda(self):
        """Devuelve el porcentaje 0-40 (no la fraccion 0.0-0.4)."""
        return int(round(self._config.get("margen_celda", 0.0) * 100))

    def recargar_referencia(self):
        with self._lock:
            ref_path = path_referencia(self._tipo, self._estacion)
            if not os.path.exists(ref_path):
                raise FileNotFoundError(f"Falta {ref_path}")
            ref = cv2.imread(ref_path)
            if ref is None:
                raise RuntimeError(f"No se pudo leer {ref_path}")
            _, self._referencia_blur = preparar_roi(ref, self._config)


# ======================================================================
#  MAIN STANDALONE (sin cambios respecto a la version anterior, salvo
#  que ahora hay que pasarle estacion)
# ======================================================================

def _parse_tipo(arg):
    try:
        t = int(arg)
        if t in TIPOS_VALIDOS:
            return t
    except (TypeError, ValueError):
        pass
    return None


def main():
    print("==================================================")
    print("  Spirax Vision - Multi-modelo Multi-estacion")
    print("==================================================\n")
    print("Uso:")
    print("  python vision.py referencia <1-6> [cinta|mesa]")
    print("  python vision.py <1-6> [cinta|mesa]")
    print("  Default estacion: cinta")

    args = sys.argv[1:]
    if not args:
        sys.exit(1)

    if args[0] == "referencia":
        tipo = _parse_tipo(args[1]) if len(args) > 1 else 1
        estacion = args[2] if len(args) > 2 else "cinta"
        if tipo is None or estacion not in ESTACIONES_VALIDAS:
            print("[X] Argumentos invalidos")
            sys.exit(1)
        print(f"[CAM] Capturando referencia Tipo {tipo} ({estacion})...")
        input("Presiona ENTER cuando este listo (sin pieza en setup)...")
        imagen = capturar(tipo, estacion)
        ref_path = path_referencia(tipo, estacion)
        cv2.imwrite(ref_path, imagen)
        print(f"[OK] Referencia guardada en {ref_path}\n")
        modo_recorte(imagen, tipo, estacion)
        return

    tipo = _parse_tipo(args[0])
    estacion = args[1] if len(args) > 1 else "cinta"
    if tipo is None or estacion not in ESTACIONES_VALIDAS:
        print(f"[X] Argumentos invalidos")
        sys.exit(1)

    if not tiene_referencia(tipo, estacion):
        print(f"[X] No hay referencia para Tipo {tipo} ({estacion})")
        sys.exit(1)

    print(f"[CAM] Capturando con Tipo {tipo} ({estacion})...")
    imagen = capturar(tipo, estacion)
    cfg = cargar_config(tipo, estacion)
    referencia = cv2.imread(path_referencia(tipo, estacion))
    _, referencia_blur = preparar_roi(referencia, cfg)

    if estacion == "mesa":
        from mesa import analizar_imagen_mesa
        res = analizar_imagen_mesa(imagen, referencia_blur, cfg)
        print(f"Matriz:\n{res['matriz']}")
        print(f"Elegida: fila={res['fila']} columna={res['columna']}")
    else:
        res = analizar_imagen(imagen, referencia_blur, cfg)
        print(f"Orientacion: {res['orientacion']}  Ratio: {res['ratio']}")

    out_path = f"comparacion_tipo{tipo}_{estacion}.png"
    cv2.imwrite(out_path, res["panel"])
    print(f"[OK] Resultado en {out_path}")

    cv2.imshow(f"Resultado Tipo {tipo} ({estacion})", res["panel"])
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
