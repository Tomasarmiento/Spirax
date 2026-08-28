"""
test_pmc_sensor.py - Prueba de lectura del sensor de salida del husillo.

Lee SQX10.6 (bit 6 del byte X10 del PMC) en loop. Pasa un pallet por el
sensor y confirma que el bit cambia 0 -> 1.

Correr desde la carpeta src:
    python test_pmc_sensor.py
    python test_pmc_sensor.py 172.31.1.99 8193
"""

import sys
import time
import os

# --- Resolver imports funcione como sea que se corra ---
# Agregamos tanto src/ como fanuc-bridge/ al path y armamos el paquete.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))      # ...\fanuc-bridge\src
_PARENT   = os.path.dirname(_THIS_DIR)                       # ...\fanuc-bridge
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

# Importar el cliente. Probamos como paquete (src.client) y si no, directo.
try:
    from src.client import FanucClient, FocasError
except ImportError:
    # correr suelto: cargar focas y client como modulos planos
    import importlib.util

    def _load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        return mod

    # focas primero (client depende de el)
    _focas = _load("focas", os.path.join(_THIS_DIR, "focas.py"))
    # parchear el import relativo de client: lo cargamos a mano
    _client_path = os.path.join(_THIS_DIR, "client.py")
    src_code = open(_client_path, encoding="utf-8").read()
    src_code = src_code.replace("from .focas import", "from focas import")
    _mod = type(sys)("client")
    sys.modules["client"] = _mod
    exec(compile(src_code, _client_path, "exec"), _mod.__dict__)
    FanucClient = _mod.FanucClient
    FocasError = _mod.FocasError

# --- Config del sensor ---
ADR_X = 0      # tipo de direccion PMC: X (entradas) = 0
BYTE_NUM = 10  # X10
BIT = 6        # SQX10.6

HOST = sys.argv[1] if len(sys.argv) > 1 else "172.31.1.99"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8193

DLL_DIR = _PARENT   # las DLLs estan en fanuc-bridge/


def main():
    print(f"Conectando a {HOST}:{PORT} ...")
    print(f"DLLs desde: {DLL_DIR}")
    if hasattr(os, "add_dll_directory") and os.path.isdir(DLL_DIR):
        try:
            os.add_dll_directory(DLL_DIR)
        except OSError:
            pass

    cli = FanucClient(host=HOST, port=PORT, timeout=10, dll_path=DLL_DIR)
    try:
        cli.connect()
    except Exception as e:
        print(f"[X] No se pudo conectar: {e}")
        return

    print("Conectado. Leyendo SQX10.6 en loop. Ctrl+C para salir.")
    print("Pasa un pallet por el sensor y observa el cambio 0 -> 1.\n")

    # Escanear VARIOS tipos de direccion PMC y rango de bytes, para
    # descubrir donde aparece el cambio del sensor.
    # Codigos de tipo FOCAS: G=0, F=1, Y=2, X=3, A=4, R=5, T=6, K=7,
    #                        C=8, D=9  (segun version; probamos los usuales)
    TIPOS = {
        "G": 0, "F": 1, "Y": 2, "X": 3, "R": 5, "D": 9,
    }
    B_INI = 0
    B_FIN = 20
    ultimo = {}

    # Primer barrido: dejar registrado el estado inicial (y ver que tipos
    # responden sin error).
    print("Barrido inicial (que tipos responden)...")
    tipos_ok = {}
    for nombre, code in TIPOS.items():
        try:
            _ = cli.read_pmc_byte(code, B_INI)
            tipos_ok[nombre] = code
            print(f"  tipo {nombre} (code {code}): OK")
        except FocasError as e:
            print(f"  tipo {nombre} (code {code}): no disponible ({e})")

    if not tipos_ok:
        print("Ningun tipo de PMC respondio. Revisar pmc_rdpmcrng.")
        return

    print(f"\nVigilando {list(tipos_ok)} bytes {B_INI}..{B_FIN}. "
          f"Pasa el pallet AHORA.\n")

    try:
        while True:
            for nombre, code in tipos_ok.items():
                for bnum in range(B_INI, B_FIN + 1):
                    key = (nombre, bnum)
                    try:
                        val = cli.read_pmc_byte(code, bnum)
                    except FocasError:
                        continue
                    if key not in ultimo:
                        ultimo[key] = val
                        continue
                    if val != ultimo[key]:
                        prev = ultimo[key]
                        cambiaron = prev ^ val
                        bits = [str(i) for i in range(8) if (cambiaron >> i) & 1]
                        print(f"{nombre}{bnum:<3} cambio: "
                              f"0x{prev:02X}({prev:08b}) -> "
                              f"0x{val:02X}({val:08b})  bit(s) {','.join(bits)}")
                        ultimo[key] = val
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nSaliendo.")
    finally:
        cli.disconnect()


if __name__ == "__main__":
    main()