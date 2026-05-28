"""
spirax_mesa.py - Analizador de matriz para la mesa de piezas

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

from spirax_vision import preparar_roi, metodo_d_bgsub

# Tamano de la grilla de la mesa (fijo para todos los tipos)
GRILLA_FILAS = 8
GRILLA_COLS = 10

# Porcentaje minimo de pixels distintos en una celda para considerarla
# "ocupada". Se aplica sobre el area de la celda.
UMBRAL_OCUPACION_DEFAULT = 0.15  # 15%


def analizar_mesa_pipeline(imagen, referencia_blur, config,
                            diff_thresh=None,
                            umbral_ocupacion=None):
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
    """
    if diff_thresh is None:
        from spirax_vision import BG_DIFF_THRESH_DEFAULT
        diff_thresh = config.get("bg_diff_thresh", BG_DIFF_THRESH_DEFAULT)
    if umbral_ocupacion is None:
        umbral_ocupacion = config.get("umbral_ocupacion",
                                       UMBRAL_OCUPACION_DEFAULT)

    # 1. ROI con el mismo recorte que usa la cinta
    roi_color, blur = preparar_roi(imagen, config)

    # 2. Background subtract
    thresh = metodo_d_bgsub(blur, referencia_blur, diff_thresh=diff_thresh)
    if thresh is None:
        panel = _panel_debug_mesa(roi_color, None, None, None, None,
                                   "SIN REFERENCIA")
        return {
            "matriz": None,
            "fila": 0,
            "columna": 0,
            "panel": panel,
        }

    # Limpieza morfologica leve (eliminar pixels sueltos)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

    # 3. Dividir en grilla 10x8 uniforme
    h, w = mask.shape[:2]
    cell_h = h / GRILLA_FILAS
    cell_w = w / GRILLA_COLS

    matriz = []
    area_celda = cell_h * cell_w

    for fi in range(GRILLA_FILAS):
        fila = []
        y1 = int(round(fi * cell_h))
        y2 = int(round((fi + 1) * cell_h))
        for ci in range(GRILLA_COLS):
            x1 = int(round(ci * cell_w))
            x2 = int(round((ci + 1) * cell_w))
            celda = mask[y1:y2, x1:x2]
            # Cuenta de pixels distintos a referencia
            pixels_distintos = int(np.count_nonzero(celda))
            ocupacion = pixels_distintos / max(1, area_celda)
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

    panel = _panel_debug_mesa(roi_color, mask, matriz,
                               elegida_fila, elegida_col,
                               f"({elegida_fila},{elegida_col})" if elegida_fila else "VACIO")
    return {
        "matriz": matriz,
        "fila": elegida_fila,
        "columna": elegida_col,
        "panel": panel,
    }


# Alias publico mas corto
def analizar_imagen_mesa(imagen, referencia_blur, config, diff_thresh=None):
    return analizar_mesa_pipeline(imagen, referencia_blur, config, diff_thresh)


def _panel_debug_mesa(roi_color, mask, matriz, elegida_fila, elegida_col,
                       texto_decision):
    """Genera un panel de debug visual con la grilla superpuesta."""
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

    # Dibujar la grilla
    cell_h = h / GRILLA_FILAS
    cell_w = w / GRILLA_COLS

    # Lineas de la grilla
    for fi in range(GRILLA_FILAS + 1):
        y = int(round(fi * cell_h))
        cv2.line(panel, (0, y), (w, y), (255, 200, 50), 1)
    for ci in range(GRILLA_COLS + 1):
        x = int(round(ci * cell_w))
        cv2.line(panel, (x, 0), (x, h), (255, 200, 50), 1)

    # Marcar celdas ocupadas y la elegida
    if matriz is not None:
        for fi in range(GRILLA_FILAS):
            for ci in range(GRILLA_COLS):
                if not matriz[fi][ci]:
                    continue
                cx = int(round((ci + 0.5) * cell_w))
                cy = int(round((fi + 0.5) * cell_h))
                radius = max(4, int(min(cell_h, cell_w) * 0.18))
                # Verde para ocupadas
                cv2.circle(panel, (cx, cy), radius, (0, 200, 0), -1)
                # Si es la elegida, anillo amarillo gordo
                if (fi + 1) == elegida_fila and (ci + 1) == elegida_col:
                    cv2.circle(panel, (cx, cy), radius + 4,
                               (0, 255, 255), 3)

    # Header
    cv2.rectangle(panel, (0, 0), (w, 40), (40, 40, 40), -1)
    cv2.putText(panel, f"Mesa: {texto_decision}",
                (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    return panel


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