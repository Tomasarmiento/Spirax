"""
leer_sensor_salida.py - Lee el sensor de salida del husillo: SQX10.6.
Tipo X (codigo 3), byte 10, bit 6. Imprime cada cambio 0<->1.
"""
import sys, time, os
_THIS = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_THIS)
for pth in (_PARENT, _THIS):
    if pth not in sys.path:
        sys.path.insert(0, pth)

try:
    from src.client import FanucClient, FocasError
except ImportError:
    import importlib.util
    def _load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        m = importlib.util.module_from_spec(spec); sys.modules[name] = m
        spec.loader.exec_module(m); return m
    _load("focas", os.path.join(_THIS, "focas.py"))
    code = open(os.path.join(_THIS, "client.py"), encoding="utf-8").read()
    code = code.replace("from .focas import", "from focas import")
    _m = type(sys)("client"); sys.modules["client"] = _m
    exec(compile(code, "client.py", "exec"), _m.__dict__)
    FanucClient, FocasError = _m.FanucClient, _m.FocasError

HOST = sys.argv[1] if len(sys.argv) > 1 else "172.31.1.99"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8193
PMC_X, BYTE, BIT = 3, 10, 1

def main():
    if hasattr(os, "add_dll_directory") and os.path.isdir(_PARENT):
        try: os.add_dll_directory(_PARENT)
        except OSError: pass
    cli = FanucClient(host=HOST, port=PORT, timeout=10, dll_path=_PARENT)
    cli.connect()
    print(f"Conectado. Leyendo SQX10.6 (X{BYTE}.{BIT}). Ctrl+C para salir.\n")
    ultimo = None
    try:
        while True:
            try:
                b = cli.read_pmc_byte(PMC_X, BYTE)
                bit = bool((b >> BIT) & 1)
            except FocasError as e:
                print(f"[ERROR] {e}"); time.sleep(1); continue
            if bit != ultimo:
                print(f"SQX10.6 = {int(bit)}  -> "
                      f"{'PALLET' if bit else 'libre '}   (X10=0x{b:02X})")
                ultimo = bit
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nSaliendo.")
    finally:
        cli.disconnect()

if __name__ == "__main__":
    main()
