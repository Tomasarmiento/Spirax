"""
mesa.py - Analizador de matriz para la mesa de piezas

La mesa tiene 8 filas x 10 columnas = 80 posiciones fijas con guias
fisicas. Cada tipo de pieza usa su propia referencia (foto de la mesa
vacia con la grilla ajustada para ese tipo).

El flujo es:
1. Tomar foto.
2. Background subtract contra la referencia (zona recortada).
3. Dividir el area recortada en grilla 10x8 uniforme.
4. Para cada celda, contar pixels distintos a referencia. Si supera
   el umbral, la celda esta "ocupada".
5. Elegir la primera celda ocupada empezando por (0,0) - barrido por
   filas, de izquierda a derecha.
6. Devolver matriz 10x8 (lista de listas) + (fila, columna) elegidas.
"""

import cv2
import numpy as np

from vision import preparar_roi, metodo_d_bgsub, cargar_espejo_col_salida

# Tamano de la grilla de la mesa (fijo para todos los tipos)
GRILLA_FILAS = 8
GRILLA_COLS = 10

# Porcentaje minimo de pixels distintos en una celda para considerarla
# "ocupada". Se aplica sobre el area de la celda.
UMBRAL_OCUPACION_DEFAULT = 0.15  # 15%


def analizar_mesa_pipeline(imagen, referencia_blur, config,
                            diff_thresh=None,
                            umbral_ocupacion=None,
                            margen_celda=None):
    """Pipeline completo de analisis de mesa.

    Recibe imagen capturada + referencia ya cargada + config del tipo.
    Devuelve dict con matriz, fila/columna elegidas y panel de debug.

    matriz: lista de FILAS listas de COLS booleanos (True = ocupada).
            matriz[0] = primera fila (la mas cercana al "origen").
            matriz[i][j] = celda fila i, columna j.

    fila, columna: 1-indexed, 0/0 = sin piezas disponibles.

    umbral_ocupacion: si None, se toma del config (clave 'umbral_ocupacion').
                      Si no esta en el config, se usa UMBRAL_OCUPACION_DEFAULT.
                      Valor entre 0 y 1 (porcentaje de pixels cambiados
                      por celda para considerarla ocupada).

    margen_celda: 0.0 a 0.4. Si None, se toma del config. Se ignora ese
                  porcentaje de cada borde de la celda antes de contar.
                  0.0 = celda entera, 0.25 = solo el 50% central.

    Los bordes de las celdas no son uniformes: se toman de config['grid_x']
    (9 valores) y config['grid_y'] (7 valores), porcentajes dentro del ROI.
    Si no estan, se usan posiciones uniformes (compatibilidad).
    """
    if diff_thresh is None:
        from vision import BG_DIFF_THRESH_DEFAULT
        diff_thresh = config.get("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT)
    if umbral_ocupacion is None:
        umbral_ocupacion = config.get("umbral_ocupacion",
                                       UMBRAL_OCUPACION_DEFAULT)
    if margen_celda is None:
        margen_celda = config.get("margen_celda", 0.0)
    margen_celda = max(0.0, min(0.4, float(margen_celda)))

    # 1. ROI con el mismo recorte que usa la cinta
    roi_color, blur = preparar_roi(imagen, config)

    # 2. Background subtract
    thresh = metodo_d_bgsub(blur, referencia_blur, diff_thresh=diff_thresh)
    if thresh is None:
        panel = _panel_debug_mesa(roi_color, None, None, None, None,
                                   "SIN REFERENCIA", config, margen_celda)
        return {
            "matriz": None,
            "fila": 0,
            "columna": 0,
            "panel": panel,
        }

    # Limpieza morfologica leve (eliminar pixels sueltos)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

    # 3. Calcular bordes de celdas a partir de grid_x / grid_y
    h, w = mask.shape[:2]
    grid_x_pct, grid_y_pct = _grid_pcts(config)

    # Bordes absolutos en pixels (0..w para columnas, 0..h para filas)
    x_borders = [0] + [int(round(p * w)) for p in grid_x_pct] + [w]
    y_borders = [0] + [int(round(p * h)) for p in grid_y_pct] + [h]

    matriz = []

    for fi in range(GRILLA_FILAS):
        fila = []
        y1 = y_borders[fi]
        y2 = y_borders[fi + 1]
        for ci in range(GRILLA_COLS):
            x1 = x_borders[ci]
            x2 = x_borders[ci + 1]
            # Aplicar margen interno: encoger la celda hacia el centro
            cw = x2 - x1
            ch = y2 - y1
            mx = int(round(cw * margen_celda))
            my = int(round(ch * margen_celda))
            xa = x1 + mx
            xb = x2 - mx
            ya = y1 + my
            yb = y2 - my
            # Proteccion contra celdas degeneradas (margen muy grande)
            if xb <= xa or yb <= ya:
                xa, xb = x1, x2
                ya, yb = y1, y2
            celda = mask[ya:yb, xa:xb]
            area = max(1, (xb - xa) * (yb - ya))
            pixels_distintos = int(np.count_nonzero(celda))
            ocupacion = pixels_distintos / area
            fila.append(ocupacion >= umbral_ocupacion)
        matriz.append(fila)

    # 4. Elegir primera celda ocupada barriendo por filas
    elegida_fila = 0
    elegida_col = 0
    for fi in range(GRILLA_FILAS):
        for ci in range(GRILLA_COLS):
            if matriz[fi][ci]:
                # 1-indexed para mandar al KUKA
                elegida_fila = fi + 1
                elegida_col = ci + 1
                break
        if elegida_fila != 0:
            break

    # 5. Espejo de columna en la salida (si esta activo).
    # IMPORTANTE: La MATRIZ y los pixels del ROI quedan en coordenadas de
    # IMAGEN (no se tocan). Solo "traducimos" el numero de columna a la
    # convencion del robot para mostrarlo en el overlay y devolverlo al KUKA.
    # El highlight visual (circulo verde) sigue cayendo donde esta la pieza
    # fisicamente en el frame, lo cual es lo correcto: el usuario ve la
    # pieza ahi y el numero de robot al lado.
    espejar = cargar_espejo_col_salida("mesa")
    if espejar and elegida_col > 0:
        elegida_col_robot = GRILLA_COLS - elegida_col + 1
    else:
        elegida_col_robot = elegida_col

    if elegida_fila > 0:
        texto = f"({elegida_fila},{elegida_col_robot})"
    else:
        texto = "VACIO"

    # Para dibujar el highlight en el panel pasamos elegida_col en coords
    # de IMAGEN (donde realmente esta la pieza en los pixels del ROI).
    panel = _panel_debug_mesa(roi_color, mask, matriz,
                               elegida_fila, elegida_col,
                               texto,
                               config, margen_celda)
    return {
        "matriz": matriz,
        "fila": elegida_fila,
        "columna": elegida_col_robot,        # <- ROBOT space (lo que va al KUKA)
        "columna_imagen": elegida_col,       # <- IMAGE space (para que HMI sepa
                                              #    donde dibujar el highlight)
        "espejo_activo": espejar,
        "panel": panel,
    }


def _grid_pcts(config):
    """Devuelve (grid_x, grid_y) leidos del config con defaults uniformes."""
    grid_x = list(config.get(
        "grid_x", [i / GRILLA_COLS for i in range(1, GRILLA_COLS)]))
    grid_y = list(config.get(
        "grid_y", [i / GRILLA_FILAS for i in range(1, GRILLA_FILAS)]))
    # Validacion defensiva contra configs viejas/corruptas
    if len(grid_x) != GRILLA_COLS - 1:
        grid_x = [i / GRILLA_COLS for i in range(1, GRILLA_COLS)]
    if len(grid_y) != GRILLA_FILAS - 1:
        grid_y = [i / GRILLA_FILAS for i in range(1, GRILLA_FILAS)]
    return grid_x, grid_y


# Alias publico mas corto
def analizar_imagen_mesa(imagen, referencia_blur, config, diff_thresh=None):
    return analizar_mesa_pipeline(imagen, referencia_blur, config, diff_thresh)


def _panel_debug_mesa(roi_color, mask, matriz, elegida_fila, elegida_col,
                       texto_decision, config=None, margen_celda=0.0):
    """Genera un panel de debug visual con la grilla superpuesta.

    Si config tiene grid_x/grid_y los usa para las divisiones. Si no,
    cae a divisiones uniformes. Si margen_celda > 0, dibuja tambien el
    rectangulo interno (zona efectivamente analizada) en cada celda.
    """
    if roi_color is None:
        return None

    h, w = roi_color.shape[:2]

    # Empezamos con el ROI a color
    panel = roi_color.copy()

    # Si tenemos mascara, la superponemos en azul tenue
    if mask is not None:
        mask_blue = np.zeros_like(panel)
        mask_blue[:, :, 0] = mask  # canal B
        panel = cv2.addWeighted(panel, 0.7, mask_blue, 0.3, 0)

    # Calcular bordes de la grilla
    if config is not None:
        grid_x_pct, grid_y_pct = _grid_pcts(config)
    else:
        grid_x_pct = [i / GRILLA_COLS for i in range(1, GRILLA_COLS)]
        grid_y_pct = [i / GRILLA_FILAS for i in range(1, GRILLA_FILAS)]

    x_borders = [0] + [int(round(p * w)) for p in grid_x_pct] + [w]
    y_borders = [0] + [int(round(p * h)) for p in grid_y_pct] + [h]

    # Lineas de la grilla externa (en celeste-amarillo tenue)
    for y in y_borders:
        cv2.line(panel, (0, y), (w, y), (255, 200, 50), 1)
    for x in x_borders:
        cv2.line(panel, (x, 0), (x, h), (255, 200, 50), 1)

    # Si hay margen interno > 0, dibujar el rectangulo interno punteado
    # de cada celda asi se ve la zona que realmente se analiza
    if margen_celda > 0.0:
        for fi in range(GRILLA_FILAS):
            y1 = y_borders[fi]
            y2 = y_borders[fi + 1]
            for ci in range(GRILLA_COLS):
                x1 = x_borders[ci]
                x2 = x_borders[ci + 1]
                cw = x2 - x1
                ch = y2 - y1
                mx = int(round(cw * margen_celda))
                my = int(round(ch * margen_celda))
                xa = x1 + mx
                xb = x2 - mx
                ya = y1 + my
                yb = y2 - my
                if xb > xa and yb > ya:
                    cv2.rectangle(panel, (xa, ya), (xb, yb),
                                  (180, 180, 180), 1)

    # Marcar celdas ocupadas y la elegida (centro segun bordes)
    if matriz is not None:
        for fi in range(GRILLA_FILAS):
            y1 = y_borders[fi]
            y2 = y_borders[fi + 1]
            cell_h = y2 - y1
            for ci in range(GRILLA_COLS):
                if not matriz[fi][ci]:
                    continue
                x1 = x_borders[ci]
                x2 = x_borders[ci + 1]
                cell_w = x2 - x1
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                radius = max(4, int(min(cell_h, cell_w) * 0.18))
                # Verde para ocupadas
                cv2.circle(panel, (cx, cy), radius, (0, 200, 0), -1)
                # Si es la elegida, anillo amarillo gordo
                if (fi + 1) == elegida_fila and (ci + 1) == elegida_col:
                    cv2.circle(panel, (cx, cy), radius + 4,
                               (0, 255, 255), 3)

    # Contar piezas para el header
    n_piezas = 0
    if matriz is not None:
        n_piezas = sum(sum(1 for c in f if c) for f in matriz)

    # Header como franja SEPARADA arriba (no tapa la imagen)
    color_header = (40, 80, 100)  # gris oscuro tono mesa
    if elegida_fila > 0:
        color_header = (50, 180, 255)  # amarillo dorado cuando hay eleccion
    franja = np.full((45, w, 3), color_header, dtype=np.uint8)
    cv2.putText(franja, f"Mesa: {texto_decision}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(franja, f"Piezas detectadas: {n_piezas}",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    return np.vstack([franja, panel])


# ======================================================================
#  Helpers para selecciones manuales (modo Manual de la HMI)
# ======================================================================

def matriz_vacia():
    """Devuelve una matriz 10x8 toda en False."""
    return [[False for _ in range(GRILLA_COLS)] for _ in range(GRILLA_FILAS)]


def matriz_con_celda(fila, columna):
    """Devuelve una matriz 10x8 con solo (fila, columna) en True
    (1-indexed). Util para mostrar la seleccion manual.
    Si fila o columna estan fuera de rango, devuelve matriz vacia.
    """
    m = matriz_vacia()
    if 1 <= fila <= GRILLA_FILAS and 1 <= columna <= GRILLA_COLS:
        m[fila - 1][columna - 1] = True
    return m