"""
test_r50_control.py - Control manual + handshake de R50.0 (un solo hilo).

Todas las llamadas FOCAS van por el hilo principal (evita EW_PROTOCOL -8
que aparece con llamadas concurrentes).

Comandos (escribi y ENTER):
    1   -> forzar R50.0 = 1
    0   -> forzar R50.0 = 0
    a   -> modo automatico ON/OFF (si esta en 0, la pone en 1 cada 2s)
    r   -> leer R50 ahora
    q   -> salir

En modo automatico, si no tecleas nada, sigue solo. Para teclear un
comando estando en auto, escribilo y ENTER igual (se procesa entre ciclos).

Uso:
    python test_r50_control.py
    python test_r50_control.py 172.31.1.99 8193
"""
import sys, os, time, threading, queue
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
        m = importlib.util.module_from_spec(spec); sys.modules[name]=m
        spec.loader.exec_module(m); return m
    _load("focas", os.path.join(_THIS, "focas.py"))
    code = open(os.path.join(_THIS,"client.py"),encoding="utf-8").read()
    code = code.replace("from .focas import","from focas import")
    _m = type(sys)("client"); sys.modules["client"]=_m
    exec(compile(code,"client.py","exec"), _m.__dict__)
    FanucClient, FocasError = _m.FanucClient, _m.FocasError

HOST = sys.argv[1] if len(sys.argv)>1 else "172.31.1.99"
PORT = int(sys.argv[2]) if len(sys.argv)>2 else 8193

PMC_R = 5
BYTE  = 50
BIT   = 0
PERIODO_S = 0.2

# Cola de comandos tecleados. El hilo de input SOLO lee del teclado y
# encola; NUNCA toca FOCAS. Todas las llamadas FOCAS las hace el hilo
# principal -> sin concurrencia -> sin EW_PROTOCOL.
_cmds = queue.Queue()

def lector_teclado():
    while True:
        try:
            linea = input()
        except (EOFError, KeyboardInterrupt):
            _cmds.put("q"); return
        _cmds.put(linea.strip().lower())

def set_bit(cli, valor):
    b = cli.read_pmc_byte(PMC_R, BYTE)
    mask = 1 << BIT
    b = (b | mask) if valor else (b & ~mask)
    cli.write_pmc_byte(PMC_R, BYTE, b)
    return b

def main():
    if hasattr(os,"add_dll_directory") and os.path.isdir(_PARENT):
        try: os.add_dll_directory(_PARENT)
        except OSError: pass
    cli = FanucClient(host=HOST, port=PORT, timeout=10, dll_path=_PARENT)
    cli.connect()
    print(f"Conectado a {HOST}:{PORT}.")
    print("Comandos: 1=forzar 1 | 0=forzar 0 | a=auto ON/OFF | "
          "r=leer | q=salir")
    print("(escribi el comando y ENTER en cualquier momento)\n")

    threading.Thread(target=lector_teclado, daemon=True).start()

    auto = False
    peticion = 0
    ultimo_auto = 0.0

    while True:
        # 1) Procesar comandos tecleados (sin bloquear)
        try:
            cmd = _cmds.get_nowait()
        except queue.Empty:
            cmd = None

        if cmd is not None:
            if cmd == "1":
                set_bit(cli, 1); print(f"R{BYTE}.{BIT} = 1 (forzado)")
            elif cmd == "0":
                set_bit(cli, 0); print(f"R{BYTE}.{BIT} = 0 (forzado)")
            elif cmd == "a":
                auto = not auto
                print(f"Modo automatico: {'ON' if auto else 'OFF'}")
            elif cmd == "r":
                b = cli.read_pmc_byte(PMC_R, BYTE)
                print(f"R{BYTE} = {b} ({b:08b})  R{BYTE}.{BIT} = {(b>>BIT)&1}")
            elif cmd == "q":
                break
            elif cmd == "":
                pass
            else:
                print("Comando: 1/0/a/r/q")

        # 2) Handshake automatico (cada PERIODO_S) - reporta cada ciclo
        if auto and (time.time() - ultimo_auto) >= PERIODO_S:
            ultimo_auto = time.time()
            try:
                b = cli.read_pmc_byte(PMC_R, BYTE)
                bit = (b >> BIT) & 1
                if bit == 0:
                    peticion += 1
                    set_bit(cli, 1)
                    print(f"[AUTO Peticion #{peticion}] R{BYTE}.{BIT} estaba 0 "
                          f"-> la puse en 1  (R{BYTE}={b:08b})")
                else:
                    print(f"[AUTO] R{BYTE}.{BIT} = 1 (ya estaba)  "
                          f"(R{BYTE}={b:08b})  peticiones={peticion}")
            except Exception as e:
                print(f"[AUTO error] {e}")

        time.sleep(0.1)

    cli.disconnect()
    print("Desconectado.")

if __name__ == "__main__":
    main()