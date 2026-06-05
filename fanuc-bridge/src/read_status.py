"""
read_status.py — Lectura continua del estado del torno.

Muestra en pantalla todo lo que se puede leer del torno, refrescando cada
N segundos. Es lo que vas a usar para monitorear durante el desarrollo.

Uso:
    python -m src.read_status
    python -m src.read_status --interval 1.0
    python -m src.read_status --once       # una sola lectura y sale
"""

from __future__ import annotations

import os
import sys
import time
import argparse
import configparser
from pathlib import Path

from .client import FanucClient, TornoStatus, FocasError


def clear_screen() -> None:
    os.system("cls" if os.name == "nt" else "clear")


def render(st: TornoStatus) -> str:
    """Render lindo del status para pantalla."""
    lines = []
    lines.append("┌─ TORNO FANUC ─────────────────────────────────────────┐")
    lines.append(f"│ Tipo:        {st.tipo:42s}│")
    lines.append(f"│ Modo:        {st.modo:42s}│")
    lines.append(f"│ Ejecución:   {st.ejecucion:42s}│")
    lines.append(f"│ Emergencia:  {st.emergencia:42s}│")
    lines.append(f"│ Alarma:      {st.alarma:42s}│")
    if st.alarma_bitmask:
        lines.append(f"│   bitmask:   {st.alarma_bitmask:#010x}".ljust(56) + "│")
    lines.append("├─ SPINDLE ─────────────────────────────────────────────┤")
    lines.append(f"│ RPM:         {st.spindle_rpm:>10.0f}".ljust(56) + "│")
    lines.append(f"│ Carga:       {st.spindle_load_pct:>10d} %".ljust(56) + "│")
    lines.append("├─ EJES ────────────────────────────────────────────────┤")
    for eje, pos in st.posicion_ejes.items():
        lines.append(f"│   {eje}:        {pos:>10.3f} mm".ljust(56) + "│")
    lines.append("├─ PROGRAMA ────────────────────────────────────────────┤")
    bloque = st.programa_actual.replace("\n", " ").strip()
    if len(bloque) > 40:
        bloque = bloque[:37] + "..."
    lines.append(f"│ Línea:       {st.linea:>10d}".ljust(56) + "│")
    lines.append(f"│ Bloque:      {bloque:40s}│")
    lines.append("├─ PRODUCCIÓN ──────────────────────────────────────────┤")
    lines.append(f"│ Part count:  {st.part_count:>10d}".ljust(56) + "│")
    if st.mensaje_operador:
        msg = st.mensaje_operador[:40]
        lines.append(f"│ Msg op:      {msg:40s}│")
    lines.append("└───────────────────────────────────────────────────────┘")
    return "\n".join(lines)


def main() -> int:
    cfg = configparser.ConfigParser()
    cfg_path = Path(__file__).resolve().parent.parent / "config.ini"
    cfg.read(cfg_path)

    parser = argparse.ArgumentParser(description="Monitor del torno Fanuc")
    parser.add_argument("--host", default=cfg.get("torno", "host", fallback="172.31.1.99"))
    parser.add_argument("--port", type=int, default=cfg.getint("torno", "port", fallback=8193))
    parser.add_argument("--dlls", default=cfg.get("dlls", "path", fallback="./dlls"))
    parser.add_argument("--interval", type=float, default=2.0, help="segundos entre lecturas")
    parser.add_argument("--once", action="store_true", help="una sola lectura y salir")
    args = parser.parse_args()

    try:
        with FanucClient(host=args.host, port=args.port, dll_path=args.dlls) as t:
            if args.once:
                st = t.status()
                print(render(st))
                return 0

            print("Iniciando monitor (Ctrl-C para salir)...")
            time.sleep(1)
            while True:
                try:
                    st = t.status()
                    clear_screen()
                    print(render(st))
                    print(f"\n[refresca cada {args.interval}s, Ctrl-C para salir]")
                except FocasError as e:
                    print(f"Error en lectura: {e}")
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
        return 0
    except FocasError as e:
        print(f"ERROR FOCAS: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
