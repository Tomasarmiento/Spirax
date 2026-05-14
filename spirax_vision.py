"""
Spirax Vision - Detector de orientacion (Background Subtraction)

Multi-modelo: cada Tipo de pieza (1-6) tiene su propia carpeta en
modelos/tipoN/ con su referencia.png y su config.json.

La HMI le dice al detector "usa Tipo N" antes de cada consulta y el
detector levanta la referencia y la config de esa carpeta.

Modos standalone:
    python spirax_vision.py referencia 3     -> Captura referencia + recorte del Tipo 3
    python spirax_vision.py 3                -> Analiza en vivo usando Tipo 3
    python spirax_vision.py 3 imagen.jpg     -> Analiza imagen.jpg usando Tipo 3
"""

import cv2
import numpy as np
import sys
import os
import json
import threading

USAR_REALSENSE = True

MODELOS_DIR = "modelos"
TIPOS_VALIDOS = (1, 2, 3, 4, 5, 6)
BG_DIFF_THRESH_DEFAULT = 40


# ======================================================================
#  Helpers de paths por tipo
# ======================================================================

def carpeta_tipo(tipo):
    """Devuelve la ruta a la carpeta del tipo. La crea si no existe."""
    if tipo not in TIPOS_VALIDOS:
        raise ValueError(f"Tipo invalido: {tipo}. Validos: {TIPOS_VALIDOS}")
    ruta = os.path.join(MODELOS_DIR, f"tipo{tipo}")
    os.makedirs(ruta, exist_ok=True)
    return ruta


def path_referencia(tipo):
    return os.path.join(carpeta_tipo(tipo), "referencia.png")


def path_config(tipo):
    return os.path.join(carpeta_tipo(tipo), "config.json")


def tiene_referencia(tipo):
    return os.path.exists(path_referencia(tipo))


def tiene_config(tipo):
    return os.path.exists(path_config(tipo))


def cargar_config(tipo):
    """Carga la config del tipo. Si no existe, devuelve defaults."""
    cfg_path = path_config(tipo)
    if os.path.exists(cfg_path):
        with open(cfg_path, "r") as f:
            cfg = json.load(f)
        # Defaults para claves que falten
        cfg.setdefault("corte_y_top_pct", 0.0)
        cfg.setdefault("corte_y_pct", 1.0)
        cfg.setdefault("corte_x_izq_pct", 0.0)
        cfg.setdefault("corte_x_der_pct", 1.0)
        cfg.setdefault("exposure", 10)
        cfg.setdefault("gain", 60)
        cfg.setdefault("saturation", 50)
        cfg.setdefault("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT)
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
    }


def guardar_config(tipo, config):
    cfg_path = path_config(tipo)
    with open(cfg_path, "w") as f:
        json.dump(config, f, indent=2)
    print(f"[OK] Config Tipo {tipo} guardada en {cfg_path}")


# ======================================================================
#  CAPTURA (modo standalone - una sola foto)
# ======================================================================

def capturar_realsense(exposure=10, gain=60, saturation=50):
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)

    sensor = profile.get_device().query_sensors()[1]
    sensor.set_option(rs.option.enable_auto_exposure, 0)
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


def capturar(tipo=1):
    """Captura una imagen usando los parametros del tipo dado."""
    cfg = cargar_config(tipo)
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
    config debe ser el dict del tipo (no lee de disco).
    """
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

def modo_recorte(imagen, tipo):
    """Permite arrastrar 4 lineas y guarda el recorte en la config del tipo."""
    config = cargar_config(tipo)
    h, w = imagen.shape[:2]

    y_top = int(h * config.get("corte_y_top_pct", 0.0))
    y_bot = int(h * config.get("corte_y_pct", 1.0))
    x_izq = int(w * config.get("corte_x_izq_pct", 0.0))
    x_der = int(w * config.get("corte_x_der_pct", 1.0))

    if y_top < 5: y_top = 5
    if y_bot > h - 5: y_bot = h - 5
    if x_izq < 5: x_izq = 5
    if x_der > w - 5: x_der = w - 5

    arrastrando = [None]

    def on_mouse(event, x, y, flags, param):
        nonlocal y_top, y_bot, x_izq, x_der
        if event == cv2.EVENT_LBUTTONDOWN:
            tolerancia = 15
            distancias = {
                'y_top': abs(y - y_top),
                'y_bot': abs(y - y_bot),
                'x_izq': abs(x - x_izq),
                'x_der': abs(x - x_der),
            }
            mas_cercana = min(distancias, key=distancias.get)
            if distancias[mas_cercana] < tolerancia:
                arrastrando[0] = mas_cercana
        elif event == cv2.EVENT_MOUSEMOVE and arrastrando[0]:
            if arrastrando[0] == 'y_top':
                y_top = max(0, min(y, y_bot - 10))
            elif arrastrando[0] == 'y_bot':
                y_bot = max(y_top + 10, min(y, h - 1))
            elif arrastrando[0] == 'x_izq':
                x_izq = max(0, min(x, x_der - 10))
            elif arrastrando[0] == 'x_der':
                x_der = max(x_izq + 10, min(x, w - 1))
        elif event == cv2.EVENT_LBUTTONUP:
            arrastrando[0] = None

    win = f"Recorte Tipo {tipo} - Arrastra las 4 lineas | S=guardar Q=salir"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        debug = imagen.copy()
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

        cv2.putText(debug,
                    f"TIPO {tipo} | Y top: {y_top}px ({y_top/h:.1%}) | Y bot: {y_bot}px ({y_bot/h:.1%}) | "
                    f"X izq: {x_izq}px ({x_izq/w:.1%}) | X der: {x_der}px ({x_der/w:.1%})",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(debug, "Arrastra cualquier linea | S=guardar Q=salir",
                    (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow(win, debug)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('s'):
            config["corte_y_top_pct"] = round(y_top / h, 4)
            config["corte_y_pct"] = round(y_bot / h, 4)
            config["corte_x_izq_pct"] = round(x_izq / w, 4)
            config["corte_x_der_pct"] = round(x_der / w, 4)
            guardar_config(tipo, config)
            cv2.destroyAllWindows()
            return True
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
#  POSTPROCESO
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
        panel = roi_color.copy()
        cv2.putText(panel, f"{nombre}: N/A", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return panel

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

    cv2.rectangle(panel, (0, 0), (w, 45), color, -1)
    cv2.putText(panel, f"{nombre}: {decision}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    ratio_txt = f"ratio: {ratio:.3f}" if ratio is not None else "ratio: ---"
    cv2.putText(panel, ratio_txt, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return panel


def analizar_imagen(imagen, referencia_blur, config, diff_thresh=None):
    """Pipeline completo sobre una imagen ya capturada.
    config es el dict del tipo (necesario para preparar_roi).
    diff_thresh: si None se toma de config.
    """
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

    Multi-modelo: usa_tipo(N) hace que las siguientes capturas/analisis
    usen la referencia y config del Tipo N.

    El tipo activo se puede leer/cambiar desde cualquier thread.
    """

    def __init__(self, tipo_inicial=1):
        self._pipeline = None
        self._sensor = None
        self._lock = threading.Lock()
        self._activo = False

        # Tipo activo y datos derivados
        self._tipo = tipo_inicial
        self._config = cargar_config(tipo_inicial)
        self._referencia_blur = None  # se carga en start() o en usa_tipo()

    # ---------- API multi-modelo ----------

    def usa_tipo(self, tipo):
        """Cambia el tipo activo. Recarga config y referencia.
        Si la camara esta activa, ajusta exposure/gain/saturation en runtime.
        Si no hay referencia para ese tipo, _referencia_blur queda en None
        (analizar() devolvera N/A hasta que se calibre).
        """
        if tipo not in TIPOS_VALIDOS:
            raise ValueError(f"Tipo invalido: {tipo}")

        with self._lock:
            self._tipo = tipo
            self._config = cargar_config(tipo)

            # Recargar referencia si existe
            ref_path = path_referencia(tipo)
            if os.path.exists(ref_path):
                ref = cv2.imread(ref_path)
                if ref is not None:
                    _, self._referencia_blur = preparar_roi(ref, self._config)
                else:
                    self._referencia_blur = None
            else:
                self._referencia_blur = None

            # Aplicar parametros de camara si esta activa
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

    def tiene_referencia_activa(self):
        with self._lock:
            return self._referencia_blur is not None

    # ---------- start/stop ----------

    def start(self, requiere_referencia=True):
        """Abre la camara, descarta frames de estabilizacion y carga
        la referencia del tipo activo.

        Si requiere_referencia=False, arranca aunque no haya referencia
        para el tipo activo (util para Calibrar).
        """
        with self._lock:
            if self._activo:
                return

            ref_path = path_referencia(self._tipo)
            if os.path.exists(ref_path):
                ref = cv2.imread(ref_path)
                if ref is None:
                    raise RuntimeError(f"No se pudo leer {ref_path}")
                _, self._referencia_blur = preparar_roi(ref, self._config)
            else:
                if requiere_referencia:
                    raise FileNotFoundError(
                        f"Falta referencia para Tipo {self._tipo}.\n"
                        f"Esperada en: {ref_path}\n"
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
        """Captura un frame de la camara ya abierta.
        descartar_buffer=True: descarta 5 frames antes para evitar el
        buffer interno de la D435 (frame viejo).
        """
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
        """Captura un frame, lo procesa y devuelve dict:
            {
                "tipo": int,
                "orientacion": "ARRIBA" | "ABAJO" | "VACIO" | "DUDOSO" | "N/A",
                "ratio": float | None,
                "panel": ndarray BGR,
            }
        Usa la referencia y config del tipo activo.
        Si no hay referencia para el tipo activo, devuelve N/A.
        """
        with self._lock:
            if not self._activo:
                raise RuntimeError("Detector no activo. Llamar start() primero.")
            frame = self._capturar_frame_raw(descartar_buffer=True)
            ref = self._referencia_blur
            cfg = self._config
            tipo = self._tipo

        # El analisis no necesita el lock
        if ref is None:
            # Devolver N/A con el frame crudo como panel
            roi_color, _ = preparar_roi(frame, cfg)
            panel = roi_color.copy()
            cv2.putText(panel, f"Tipo {tipo}: SIN REFERENCIA",
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
        """Captura un frame crudo (sin analizar). Usado para preview en
        vivo y para sacar referencia desde la HMI.
        """
        with self._lock:
            if not self._activo:
                raise RuntimeError("Detector no activo. Llamar start() primero.")
            return self._capturar_frame_raw(descartar_buffer=False)

    # ---------- ajuste y persistencia de parametros del tipo activo ----------

    def _persistir(self):
        """Guarda la config actual del tipo activo en disco."""
        try:
            guardar_config(self._tipo, self._config)
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

    @property
    def exposure(self):
        return int(self._config.get("exposure", 10))

    @property
    def gain(self):
        return int(self._config.get("gain", 60))

    @property
    def bg_diff_thresh(self):
        return int(self._config.get("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT))

    def recargar_referencia(self):
        """Vuelve a leer la referencia del tipo activo (despues de
        capturar una nueva)."""
        with self._lock:
            ref_path = path_referencia(self._tipo)
            if not os.path.exists(ref_path):
                raise FileNotFoundError(f"Falta {ref_path}")
            ref = cv2.imread(ref_path)
            if ref is None:
                raise RuntimeError(f"No se pudo leer {ref_path}")
            _, self._referencia_blur = preparar_roi(ref, self._config)


# ======================================================================
#  MAIN STANDALONE
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
    print("  Spirax Vision - Multi-modelo")
    print("==================================================\n")

    args = sys.argv[1:]

    # Modo referencia: python spirax_vision.py referencia [tipo]
    if args and args[0] == "referencia":
        tipo = _parse_tipo(args[1]) if len(args) > 1 else 1
        if tipo is None:
            print("[X] Tipo invalido. Uso: python spirax_vision.py referencia <1-6>")
            sys.exit(1)
        print(f"[CAM] Capturando escenario vacio para Tipo {tipo}...")
        print("      Asegurate de que NO hay pieza en el setup.\n")
        input("Presiona ENTER cuando este listo...")
        imagen = capturar(tipo)
        ref_path = path_referencia(tipo)
        cv2.imwrite(ref_path, imagen)
        print(f"[OK] Referencia Tipo {tipo} guardada en {ref_path}\n")
        print("[!] Ahora defini el area util (las 4 lineas de recorte).\n")
        modo_recorte(imagen, tipo)
        return

    # Modo analisis: primer arg = tipo, segundo arg opcional = imagen
    if not args:
        print("Uso:")
        print("  python spirax_vision.py referencia <1-6>     # capturar referencia")
        print("  python spirax_vision.py <1-6>                # analizar en vivo")
        print("  python spirax_vision.py <1-6> imagen.jpg     # analizar imagen")
        sys.exit(1)

    tipo = _parse_tipo(args[0])
    if tipo is None:
        print(f"[X] Tipo invalido: {args[0]}. Validos: {TIPOS_VALIDOS}")
        sys.exit(1)

    if not tiene_referencia(tipo):
        print(f"[X] No hay referencia para Tipo {tipo} en {path_referencia(tipo)}")
        print(f"    Capturala primero: python spirax_vision.py referencia {tipo}")
        sys.exit(1)

    if len(args) > 1:
        ruta = args[1]
        if not os.path.exists(ruta):
            print(f"[X] Archivo no encontrado: {ruta}")
            sys.exit(1)
        imagen = cv2.imread(ruta)
    else:
        print(f"[CAM] Capturando imagen con params del Tipo {tipo}...")
        imagen = capturar(tipo)

    if imagen is None:
        print("[X] No se pudo obtener imagen.")
        sys.exit(1)

    cfg = cargar_config(tipo)
    referencia = cv2.imread(path_referencia(tipo))
    _, referencia_blur = preparar_roi(referencia, cfg)
    print(f"[OK] Referencia Tipo {tipo} cargada de {path_referencia(tipo)}")

    print(f"[..] Analizando con Tipo {tipo}...\n")
    res = analizar_imagen(imagen, referencia_blur, cfg)

    print(f"{'Tipo':<6}{'Decision':<10}{'Ratio':<10}")
    print("-" * 28)
    ratio_str = f"{res['ratio']:.3f}" if res['ratio'] is not None else "---"
    print(f"{tipo:<6}{res['orientacion']:<10}{ratio_str:<10}")
    print()

    out_path = f"comparacion_tipo{tipo}.png"
    cv2.imwrite(out_path, res["panel"])
    print(f"[OK] Resultado guardado en {out_path}\n")

    cv2.imshow(f"Background Subtraction - Tipo {tipo}", res["panel"])
    print("Presiona cualquier tecla para cerrar.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()