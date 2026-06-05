"""
inspect_macros.py — Escanea las macros del torno e identifica cuáles están en uso.

SOLO LECTURA. No escribe nada al torno.

Estrategia: lee rangos comunes de macros usando cnc_rdmacror (batch, más
rápido que de a una) y muestra las que tienen valor distinto a "vacío".

Convenciones Fanuc estándar:
  #1   - #33     : variables locales (por ejecución)
  #100 - #199    : variables comunes volátiles (se borran al apagar)
  #500 - #999    : variables comunes retentivas (sobreviven apagado)
  #1000 - #...   : variables del sistema (interfaz PMC, posición, etc.)

Uso:
    python -m src.inspect_macros                    # todos los rangos comunes
    python -m src.inspect_macros --range 500 599    # rango específico
    python -m src.inspect_macros --watch 1000 1010  # monitor en vivo
"""

from __future__ import annotations

import sys
import time
import argparse
import configparser
from pathlib import Path

from .client import FanucClient, FocasError
from .focas import EW_OK


# Rangos típicos de Fanuc que vale la pena escanear
RANGOS_ESTANDAR = [
    (100, 199, "Comunes volátiles (#100-#199)"),
    (500, 599, "Comunes retentivas low (#500-#599)"),
    (600, 999, "Comunes retentivas high (#600-#999)"),
    (1000, 1099, "Sistema / PMC interface (#1000-#1099)"),
]


def is_empty(value: float) -> bool:
    """Una macro 'vacía' devuelve NaN según nuestra convención en focas.py."""
    return value != value  # NaN != NaN


def fmt_value(v: float) -> str:
    """Formato lindo para mostrar el valor."""
    if is_empty(v):
        return "vacía"
    if v == int(v):
        return str(int(v))
    return f"{v:.4f}".rstrip("0").rstrip(".")


def scan_range(client: FanucClient, start: int, end: int,
               show_empty: bool = False) -> dict:
    """Lee macros en [start, end] una por una y devuelve las que tienen valor."""
    encontradas = {}
    total = end - start + 1
    print(f"  Escaneando {total} macros...", end="", flush=True)
    errores = 0

    for num in range(start, end + 1):
        try:
            v = client.get_macro(num)
            if not is_empty(v) or show_empty:
                encontradas[num] = v
        except FocasError:
            errores += 1
            # Algunas macros pueden no ser legibles según el control
            continue

    print(f" {len(encontradas)} con valor, {errores} no legibles")
    return encontradas


def cmd_scan(client: FanucClient, args) -> int:
    """Escanea rangos completos y muestra qué encontró."""
    if args.range:
        rangos = [(args.range[0], args.range[1], f"Custom #{args.range[0]}-#{args.range[1]}")]
    else:
        rangos = RANGOS_ESTANDAR

    print("=" * 60)
    print(" INSPECCIÓN DE MACROS DEL TORNO")
    print("=" * 60)

    total = 0
    for start, end, descripcion in rangos:
        print(f"\n▶ {descripcion}")
        found = scan_range(client, start, end, show_empty=args.show_empty)
        for num in sorted(found.keys()):
            v = found[num]
            print(f"    #{num:<5d} = {fmt_value(v)}")
        total += len(found)

    print("\n" + "=" * 60)
    print(f" Total de macros con valor: {total}")
    print("=" * 60)
    return 0


def cmd_watch(client: FanucClient, args) -> int:
    """Monitorea un rango chico en vivo, refrescando cada segundo."""
    start, end = args.watch
    print(f"Monitor en vivo de #{start} a #{end}. Ctrl-C para salir.\n")

    valores_anteriores = {}
    try:
        while True:
            cambios = []
            for num in range(start, end + 1):
                try:
                    v = client.get_macro(num)
                except FocasError:
                    continue

                anterior = valores_anteriores.get(num)
                if anterior is None or (v != anterior and not (is_empty(v) and is_empty(anterior))):
                    cambios.append((num, anterior, v))
                    valores_anteriores[num] = v

            if cambios:
                ts = time.strftime("%H:%M:%S")
                for num, ant, nuevo in cambios:
                    if ant is None:
                        print(f"[{ts}] #{num} = {fmt_value(nuevo)}")
                    else:
                        print(f"[{ts}] #{num} : {fmt_value(ant)} → {fmt_value(nuevo)}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nDetenido.")
    return 0


def main() -> int:
    cfg = configparser.ConfigParser()
    cfg_path = Path(__file__).resolve().parent.parent / "config.ini"
    cfg.read(cfg_path)

    parser = argparse.ArgumentParser(
        description="Inspector de variables macro del torno (SOLO LECTURA)"
    )
    parser.add_argument("--host", default=cfg.get("torno", "host", fallback="172.31.1.99"))
    parser.add_argument("--port", type=int, default=cfg.getint("torno", "port", fallback=8193))
    parser.add_argument("--dlls", default=cfg.get("dlls", "path", fallback="./dlls"))

    parser.add_argument(
        "--range", nargs=2, type=int, metavar=("START", "END"),
        help="escanear un rango específico (ej: --range 500 599)",
    )
    parser.add_argument(
        "--watch", nargs=2, type=int, metavar=("START", "END"),
        help="monitorear cambios en vivo (ej: --watch 1000 1010)",
    )
    parser.add_argument(
        "--show-empty", action="store_true",
        help="también mostrar macros vacías (default: solo las que tienen valor)",
    )
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="segundos entre lecturas en modo watch (default 1)",
    )
    args = parser.parse_args()

    try:
        with FanucClient(host=args.host, port=args.port, dll_path=args.dlls) as t:
            if args.watch:
                return cmd_watch(t, args)
            return cmd_scan(t, args)
    except FocasError as e:
        print(f"ERROR FOCAS: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
