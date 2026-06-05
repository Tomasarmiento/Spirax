"""
test_connection.py — Verifica que se puede conectar al torno por FOCAS.

Lo primero que hay que correr. Si esto anda, todo lo demás también.

Uso:
    python -m src.test_connection
    python -m src.test_connection --host 172.31.1.99
"""

from __future__ import annotations

import sys
import argparse
import configparser
from pathlib import Path

from .focas import Focas, err_name, EW_OK


def main() -> int:
    # Leer config.ini si existe
    cfg = configparser.ConfigParser()
    cfg_path = Path(__file__).resolve().parent.parent / "config.ini"
    cfg.read(cfg_path)

    default_host = cfg.get("torno", "host", fallback="172.31.1.99")
    default_port = cfg.getint("torno", "port", fallback=8193)
    default_timeout = cfg.getint("torno", "timeout", fallback=10)
    default_dlls = cfg.get("dlls", "path", fallback="./dlls")

    parser = argparse.ArgumentParser(description="Test conexión FOCAS")
    parser.add_argument("--host", default=default_host)
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--timeout", type=int, default=default_timeout)
    parser.add_argument("--dlls", default=default_dlls)
    args = parser.parse_args()

    print(f"DLLs en: {args.dlls}")
    print(f"Cargando Fwlib32.dll...")

    try:
        focas = Focas(dll_path=args.dlls)
    except (FileNotFoundError, OSError) as e:
        print(f"ERROR cargando DLLs: {e}")
        print("\nVerificá que en la carpeta dlls/ estén (Windows 64 bits):")
        print("  - Fwlib64.dll")
        print("  - fwlibe64.dll")
        print("  - fwlib30i64.dll")
        return 1
    except RuntimeError as e:
        print(f"ERROR: {e}")
        return 1

    print(f"OK. Conectando a {args.host}:{args.port}...")
    handle, ret = focas.allclibhndl3(args.host, args.port, args.timeout)

    if ret != EW_OK:
        print(f"FALLO al conectar: {ret} ({err_name(ret)})")
        print("\nPosibles causas:")
        print("  - El torno no responde (probar ping).")
        print("  - FOCAS no está habilitado como opción en el control.")
        print("  - El puerto está mal (debería ser 8193).")
        print("  - Hay un firewall bloqueando.")
        return 1

    print(f"CONECTADO. Handle = {handle}")

    # Lectura mínima para verificar que realmente habla
    print("\nLeyendo info del sistema...")
    sys_, ret = focas.sysinfo(handle)
    if ret == EW_OK:
        from .focas import decode_sysinfo
        info = decode_sysinfo(sys_)
        print(f"  Tipo de máquina: {info['mt_type']}  (T = torno, M = fresa)")
        print(f"  Series:         {info['series']}")
        print(f"  Version:        {info['version']}")
        print(f"  Max ejes:       {info['max_axis']}")
        print(f"  Ejes actuales:  {info['axes']}")
    else:
        print(f"  cnc_sysinfo falló: {ret} ({err_name(ret)})")

    print("\nLeyendo estado...")
    st, ret = focas.statinfo2(handle)
    if ret == EW_OK:
        from .focas import decode_status
        for k, v in decode_status(st).items():
            print(f"  {k:15s} {v}")
    else:
        print(f"  cnc_statinfo2 falló: {ret} ({err_name(ret)})")

    print("\nCerrando handle...")
    focas.freelibhndl(handle)
    print("Listo.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
