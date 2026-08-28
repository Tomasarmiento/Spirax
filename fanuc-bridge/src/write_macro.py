"""
write_macro.py — Lee/escribe variables macro del torno.

Importante: el primer experimento usa la macro #100, que es de prueba.
NO toques las del programa de producción hasta validar todo.

Uso:
    # Leer macros
    python -m src.write_macro --read 500 501 502 503
    python -m src.write_macro --read 100

    # Escribir (cuidado!)
    python -m src.write_macro --write 100 42
    python -m src.write_macro --write 500 1 --write 501 10 --write 502 1

    # Demo del handshake Spirax
    python -m src.write_macro --spirax 1 10
"""

from __future__ import annotations

import sys
import argparse
import configparser
from pathlib import Path

from .client import FanucClient, FocasError


def main() -> int:
    cfg = configparser.ConfigParser()
    cfg_path = Path(__file__).resolve().parent.parent / "config.ini"
    cfg.read(cfg_path)

    parser = argparse.ArgumentParser(
        description="Leer y escribir variables macro del torno"
    )
    parser.add_argument("--host", default=cfg.get("torno", "host", fallback="192.168.1.10"))
    parser.add_argument("--port", type=int, default=cfg.getint("torno", "port", fallback=8193))
    parser.add_argument("--dlls", default=cfg.get("dlls", "path", fallback="./dlls"))

    parser.add_argument(
        "--read", nargs="+", type=int, metavar="NUM",
        help="leer una o más macros (ej: --read 500 501 502)",
    )
    parser.add_argument(
        "--write", nargs=2, action="append", metavar=("NUM", "VAL"),
        help="escribir macro NUM = VAL (puede repetirse)",
    )
    parser.add_argument(
        "--spirax", nargs=2, type=int, metavar=("MODELO", "OP"),
        help="demo Spirax: setea #500=MODELO, #501=OP, #502=1 y espera",
    )
    args = parser.parse_args()

    if not args.read and not args.write and args.spirax is None:
        parser.print_help()
        return 1

    try:
        with FanucClient(host=args.host, port=args.port, dll_path=args.dlls) as t:
            if args.read:
                print("─── Lectura ───")
                for num in args.read:
                    try:
                        v = t.get_macro(num)
                        print(f"  #{num} = {v}")
                    except FocasError as e:
                        print(f"  #{num} ERROR: {e}")

            if args.write:
                print("─── Escritura ───")
                for num_str, val_str in args.write:
                    num = int(num_str)
                    val = float(val_str)
                    decs = 0
                    if "." in val_str:
                        decs = len(val_str.split(".")[1])
                    try:
                        t.set_macro(num, val, decimals=decs)
                        nuevo = t.get_macro(num)
                        print(f"  #{num} = {val} (escrito) → leído: {nuevo}")
                    except FocasError as e:
                        print(f"  #{num} ERROR: {e}")

            if args.spirax is not None:
                modelo, op = args.spirax
                print(f"─── Spirax: pieza={modelo}, op={op} ───")
                ok = t.enviar_pieza_a_torno(modelo=modelo, operacion=op)
                if ok:
                    print("Torno tomó los datos (flag #502 bajó a 0).")
                else:
                    print("Timeout: el torno no leyó los datos.")
                    print("  Verificá que el programa NC esté en RUN.")

    except FocasError as e:
        print(f"ERROR FOCAS: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
