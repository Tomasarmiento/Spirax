#!/usr/bin/env python3
"""
nkio.py - DI/DO del Nodka TPC6000-C2154-B desde Python.

Wrapper sobre NKIOLIB, la libreria oficial del SDK NKDIO de Nodka.
Reemplaza a la version anterior de este archivo, que intentaba hablarle
al hardware directamente. Ya no hace falta InpOutx64 ni portio.py.

COMO FUNCIONA EL HARDWARE (confirmado desde el SDK)
---------------------------------------------------
Los 8DI + 8DO no estan en el Super I/O: son un expansor I2C PCA9555
colgado del bus SMBus del chipset, en PCI 00:1F.4. El nkio_config.ini lo
dice: Dev=0x1F, Fun=0x04, y la libreria llama a PCA9555_Config().

Para la serie XXX4 (tu panel), el config trae:
    DeviceNum   = 1        -> un solo expansor
    Port0Config = 0x00     -> puerto 0 todo salidas  -> los 8 DO
    Port1Config = 0xFF     -> puerto 1 todo entradas -> los 8 DI

REQUISITOS
----------
  1. Instalar NKDIOLC_Driver_Setup_x86_V5.0.6.exe (viene en el SDK).
     Instala el driver de kernel y NKIOLIB.dll.
  2. Tener nkio_config.ini de la carpeta ConfigFile/XXX4/ del SDK.
     Ponelo junto a este archivo, o pasa la ruta a NodkaIO(config=...).
  3. Correr como Administrador. El acceso al SMBus lo exige.

OJO CON LA ARQUITECTURA: si el instalador deja una NKIOLIB.dll de 32 bits,
necesitas Python de 32 bits. Este modulo detecta el desajuste y te lo dice
con un mensaje claro en vez de fallar de forma confusa.

TU PANEL TIENE SALIDAS PNP
--------------------------
Una salida activa entrega +24 V. Para medir sin carga: tester en tension
entre el DOn y el pin 2 (DOGND), no continuidad. Para cablear una carga:
DOn -> carga -> 0 V.

USO
---
    from nkio import NodkaIO

    with NodkaIO() as io:
        print(io.read_all_di())        # [False, True, ...] 8 entradas
        io.write_do(3, True)           # prende DO3
        if io.read_di(0):
            io.write_do(0, True)

CLI
---
    python nkio.py info               # version y estado
    python nkio.py monitor            # entradas en vivo
    python nkio.py set 3 on           # prende DO3
    python nkio.py blink 0            # parpadea DO0
    python nkio.py walk               # recorre DO0..DO7 de a una
    python nkio.py probe 0            # determina la polaridad de DO0
"""

import ctypes
import os
import sys
import time

# --- Codigos de error de NKIOLIB.h ----------------------------------------
NKIO_ERRORS = {
    0: "sin error",
    1: "el bus SMBus esta ocupado",
    2: "el bus SMBus esta en uso por otro proceso",
    3: "error de bus SMBus",
    4: "timeout",
    5: "no se pudo acceder al dispositivo de IO "
       "(revisá que el panel tenga la placa de expansión DIO)",
    6: "argumento invalido",
    7: "error al leer el archivo de configuracion",
}

CONFIG_NAME = "nkio_config.ini"

# El instalador puede dejar la DLL con distintos nombres segun version y
# arquitectura. Se prueban en orden de preferencia: primero la que coincida
# con el Python en uso.
if sys.maxsize > 2 ** 32:
    DLL_NAMES = ["NKIOLIBx64.dll", "NKIOLIB.dll", "NKIOLIBx86.dll"]
else:
    DLL_NAMES = ["NKIOLIBx86.dll", "NKIOLIB.dll", "NKIOLIBx64.dll"]

# Rutas donde el instalador suele dejar la DLL.
DLL_SEARCH = [
    r"C:\NODKA\NKDIOLC_SDK\Lib\x64",
    r"C:\NODKA\NKDIOLC_SDK\Lib\x86",
    r"C:\NODKA\NKDIOLC_SDK\Bin",
    r"C:\NODKA\NKDIO_SDK\Lib\x64",
    r"C:\NODKA\NKDIO_SDK\Lib\x86",
    r"C:\NODKA\NKDIO_SDK\Bin",
    r"C:\Program Files (x86)\Nodka",
    r"C:\Program Files\Nodka",
]

CONFIG_SEARCH = [
    r"C:\NODKA\NKDIOLC_SDK\ConfigFile\XXX4",
    r"C:\NODKA\NKDIO_SDK\ConfigFile\XXX4",
    r"C:\NODKA\NKDIOLC_SDK\ConfigFile",
    r"C:\NODKA\NKDIO_SDK\ConfigFile",
]

_HERE = os.path.dirname(os.path.abspath(__file__))

# ===========================================================================
# POLARIDAD  -  VERIFICAR CON TESTER ANTES DE CONECTAR CARGAS
# ===========================================================================
#
# Evidencia observada en el panel: con nada conectado, el registro de
# salidas lee 0xFF (todo unos) y el de entradas 0x00 (todo ceros).
#
# 0xFF es el valor por defecto del registro de salida del PCA9555 al
# arrancar. Como seria un diseno inaceptable que el panel arranque con las
# 8 salidas energizadas, lo mas probable es que la logica de salida este
# invertida: un 1 en el bit = salida APAGADA.
#
# Por eso DO_INVERT arranca en True: con ese valor, "apagar todo" escribe
# 0xFF, que es el estado de reposo del hardware. Es la opcion que falla del
# lado seguro. Si resulta al reves, cambialo a False.
#
# COMO VERIFICARLO, sin cargas conectadas:
#     python nkio.py probe 0
# Ese comando cambia UNA salida y te dice que medir. Con el tester en
# tension entre DO0 (pin 3) y DOGND (pin 2) confirmas la polaridad real.
DO_INVERT = True     # True: bit 1 = salida apagada
DI_INVERT = False    # False: bit 1 = entrada activa
# ===========================================================================


class NkioError(RuntimeError):
    """Error devuelto por NKIOLIB, con el codigo original."""

    def __init__(self, code, context=""):
        self.code = code
        desc = NKIO_ERRORS.get(code, f"codigo desconocido {code}")
        msg = f"{context}: {desc} (codigo {code})" if context else desc
        super().__init__(msg)


def _find(name, extra_dirs):
    """Busca un archivo junto al script, en el cwd y en rutas de instalacion."""
    candidates = [os.path.join(_HERE, name), os.path.join(os.getcwd(), name)]
    candidates += [os.path.join(d, name) for d in extra_dirs]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def _load_dll():
    """Busca y carga la DLL de NKIOLIB, probando los nombres conocidos."""
    found = []
    for name in DLL_NAMES:
        path = _find(name, DLL_SEARCH)
        if path:
            found.append(path)

    if not found:
        raise NkioError(
            5,
            "no encontre la DLL de NKIOLIB.\n"
            f"  Busque estos nombres: {', '.join(DLL_NAMES)}\n"
            "  Copiala junto a este script. Suele estar en la carpeta de la\n"
            "  utilidad de test de Nodka, o en C:\\NODKA\\ despues de instalar.\n"
            "  Copia tambien NKLCLIBx86.dll si esta al lado: NKIOLIB puede\n"
            "  depender de ella",
        )

    py_bits = 64 if sys.maxsize > 2 ** 32 else 32
    errors = []
    for path in found:
        folder = os.path.dirname(path)
        if hasattr(os, "add_dll_directory") and os.path.isdir(folder):
            try:
                os.add_dll_directory(folder)
            except OSError:
                pass
        try:
            return ctypes.CDLL(path), path
        except OSError as exc:
            errors.append(f"    {os.path.basename(path)}: {exc}")

    # Ninguna cargo. El caso tipico es desajuste de arquitectura.
    hint = ""
    names = " ".join(os.path.basename(p).lower() for p in found)
    if py_bits == 64 and "x86" in names and "x64" not in names:
        hint = (
            "\n\n  La DLL disponible es de 32 bits (x86) y estas usando Python\n"
            "  de 64 bits. No se pueden mezclar. Dos opciones:\n"
            "    a) instalar Python de 32 bits y correr este script con ese\n"
            "    b) buscar si el SDK trae una version x64 de la DLL\n"
            "  La opcion (a) es la mas rapida y no interfiere con tu Python\n"
            "  actual: pueden convivir los dos."
        )
    elif py_bits == 32 and "x64" in names and "x86" not in names:
        hint = (
            "\n\n  La DLL es de 64 bits y estas usando Python de 32 bits.\n"
            "  Usa un Python x64."
        )

    raise NkioError(
        5,
        "encontre la DLL pero no pude cargarla.\n"
        + "\n".join(errors)
        + f"\n  Python en uso: {py_bits} bits"
        + hint,
    )


class NodkaIO:
    """8 entradas + 8 salidas digitales del TPC6000-C2154-B (salidas PNP)."""

    N_DI = 8
    N_DO = 8
    DI_INDEX = 0   # puerto 1 del PCA9555, mapeado por la libreria al indice 0
    DO_INDEX = 0   # puerto 0 del PCA9555

    def __init__(self, config=None, verbose=False):
        if os.name != "nt":
            raise NkioError(6, "este wrapper es para Windows")

        self.dll, self.dll_path = _load_dll()

        cfg = config or _find(CONFIG_NAME, CONFIG_SEARCH)
        if cfg is None:
            raise NkioError(
                7,
                f"no encontre {CONFIG_NAME}.\n"
                "  Copiá el de ConfigFile/XXX4/ del SDK junto a este script.\n"
                "  Para la serie XXX4 debe decir Dev=0x1F, Fun=0x04,\n"
                "  DeviceNum=1, Port0Config=0x00, Port1Config=0xFF",
            )
        self.config_path = os.path.abspath(cfg)

        self._bind()

        # La DLL espera char*. En Windows la codificacion nativa es mbcs;
        # el fallback existe para que el modulo sea testeable fuera de Windows.
        try:
            cfg_bytes = self.config_path.encode("mbcs")
        except LookupError:
            cfg_bytes = self.config_path.encode("utf-8")

        rc = self.dll.NKDIO_LibraryInit(cfg_bytes)
        if rc != 0:
            extra = ""
            if rc in (1, 2, 3, 4):
                extra = (
                    "\n  Error de bus SMBus. Verificá que corras como "
                    "Administrador\n  y que no haya otro proceso usando el DIO "
                    "(la utilidad de test de Nodka, por ejemplo)."
                )
            elif rc == 5:
                extra = (
                    "\n  La librería no ve el expansor PCA9555 en el SMBus.\n"
                    "  Puede ser que esta unidad no tenga la placa de "
                    "expansión DIO."
                )
            raise NkioError(rc, f"NKDIO_LibraryInit falló{extra}")

        self._open = True
        if verbose:
            print(f"[nkio] DLL:    {self.dll_path}")
            print(f"[nkio] config: {self.config_path}")

        # Espejo del estado de salidas, sincronizado leyendo el hardware.
        try:
            self._do_state = self._read_do_byte()
        except NkioError:
            self._do_state = 0x00

    # --- binding de firmas -------------------------------------------------

    def _bind(self):
        d = self.dll
        d.NKDIO_LibraryInit.argtypes = [ctypes.c_char_p]
        d.NKDIO_LibraryInit.restype = ctypes.c_int

        d.NKDIO_LibraryDeinit.argtypes = []
        d.NKDIO_LibraryDeinit.restype = None

        for name in ("NKDIO_PollingReadDiByte", "NKDIO_PollingReadDoByte"):
            fn = getattr(d, name)
            fn.argtypes = [ctypes.c_ubyte, ctypes.POINTER(ctypes.c_ubyte)]
            fn.restype = ctypes.c_int

        d.NKDIO_PollingWriteDoByte.argtypes = [ctypes.c_ubyte, ctypes.c_ubyte]
        d.NKDIO_PollingWriteDoByte.restype = ctypes.c_int

    # --- acceso crudo a bytes ---------------------------------------------

    def read_di_byte(self, index=None):
        """Los 8 DI como un byte, tal cual lo entrega el hardware."""
        buf = ctypes.c_ubyte(0)
        idx = self.DI_INDEX if index is None else index
        rc = self.dll.NKDIO_PollingReadDiByte(idx, ctypes.byref(buf))
        if rc != 0:
            raise NkioError(rc, f"lectura de DI byte {idx}")
        return buf.value

    def _read_do_byte(self, index=None):
        buf = ctypes.c_ubyte(0)
        idx = self.DO_INDEX if index is None else index
        rc = self.dll.NKDIO_PollingReadDoByte(idx, ctypes.byref(buf))
        if rc != 0:
            raise NkioError(rc, f"relectura de DO byte {idx}")
        return buf.value

    def write_do_byte(self, value, index=None):
        """Escribe los 8 DO de una vez."""
        idx = self.DO_INDEX if index is None else index
        rc = self.dll.NKDIO_PollingWriteDoByte(idx, value & 0xFF)
        if rc != 0:
            raise NkioError(rc, f"escritura de DO byte {idx}")
        self._do_state = value & 0xFF

    # --- entradas ---------------------------------------------------------

    @staticmethod
    def _di_level(raw, channel):
        bit = bool(raw & (1 << channel))
        return (not bit) if DI_INVERT else bit

    @staticmethod
    def _do_level(raw, channel):
        bit = bool(raw & (1 << channel))
        return (not bit) if DO_INVERT else bit

    def read_di(self, channel):
        """True si la entrada esta activa."""
        self._check(channel, self.N_DI, "DI")
        return self._di_level(self.read_di_byte(), channel)

    def read_all_di(self):
        raw = self.read_di_byte()
        return [self._di_level(raw, ch) for ch in range(self.N_DI)]

    # --- salidas ----------------------------------------------------------

    def write_do(self, channel, state):
        """Prende o apaga una salida sin tocar las otras."""
        self._check(channel, self.N_DO, "DO")
        mask = 1 << channel
        # El bit fisico puede ser el inverso del estado logico.
        bit_high = (not state) if DO_INVERT else bool(state)
        if bit_high:
            self._do_state |= mask
        else:
            self._do_state &= ~mask & 0xFF
        self.write_do_byte(self._do_state)

    def write_all_do(self, states):
        if len(states) != self.N_DO:
            raise ValueError(
                f"se esperaban {self.N_DO} valores, llegaron {len(states)}"
            )
        value = 0
        for ch, state in enumerate(states):
            bit_high = (not state) if DO_INVERT else bool(state)
            if bit_high:
                value |= 1 << ch
        self.write_do_byte(value)

    def read_do(self, channel):
        """Relee del hardware el estado real de una salida."""
        self._check(channel, self.N_DO, "DO")
        return self._do_level(self._read_do_byte(), channel)

    def read_all_do(self):
        raw = self._read_do_byte()
        return [self._do_level(raw, ch) for ch in range(self.N_DO)]

    def all_off(self):
        """Apaga las 8 salidas, respetando la polaridad configurada."""
        self.write_do_byte(0xFF if DO_INVERT else 0x00)

    # --- infraestructura --------------------------------------------------

    @staticmethod
    def _check(channel, count, kind):
        if not 0 <= channel < count:
            raise ValueError(
                f"{kind} fuera de rango: {channel} (validos 0..{count - 1})"
            )

    def close(self):
        if getattr(self, "_open", False):
            self.dll.NKDIO_LibraryDeinit()
            self._open = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _fmt(values):
    return "  ".join("X" if v else "." for v in values)


def _info():
    with NodkaIO(verbose=True) as io:
        print()
        print(f"DI byte: 0x{io.read_di_byte():02X}")
        print(f"DO byte: 0x{io._read_do_byte():02X}")
        print()
        print("        DI0 DI1 DI2 DI3 DI4 DI5 DI6 DI7")
        print(f"  DI:    {_fmt(io.read_all_di())}")
        print(f"  DO:    {_fmt(io.read_all_do())}")
        print()
        print("Salidas PNP: activa = +24 V en el pin. Medí tensión contra")
        print("el pin 2 (DOGND), no continuidad.")


def _monitor():
    with NodkaIO(verbose=True) as io:
        print()
        print("Entradas en vivo. Ctrl+C para salir.")
        print("            DI0 DI1 DI2 DI3 DI4 DI5 DI6 DI7")
        prev = None
        while True:
            cur = io.read_all_di()
            if cur != prev:
                print(f"{time.strftime('%H:%M:%S')}     {_fmt(cur)}")
                prev = cur
            time.sleep(0.05)


def _set(channel, state):
    with NodkaIO(verbose=True) as io:
        io.write_do(channel, state)
        back = io.read_do(channel)
        print(f"DO{channel} -> {'ON' if state else 'OFF'}")
        print(f"relectura del hardware: {'ON' if back else 'OFF'}")
        if back != state:
            print("Ojo: la relectura no coincide con lo que escribimos.")


def _blink(channel, period=0.5):
    with NodkaIO(verbose=True) as io:
        print(f"Parpadeando DO{channel}. Ctrl+C para cortar.")
        state = False
        try:
            while True:
                state = not state
                io.write_do(channel, state)
                print("  ON " if state else "  OFF", end="\r", flush=True)
                time.sleep(period)
        finally:
            io.write_do(channel, False)
            print("\nApagada.")


def _walk(period=0.6):
    with NodkaIO(verbose=True) as io:
        print("Recorriendo DO0..DO7. Ctrl+C para cortar.")
        try:
            while True:
                for ch in range(io.N_DO):
                    io.write_all_do([i == ch for i in range(io.N_DO)])
                    print(f"  DO{ch} activa   ", end="\r", flush=True)
                    time.sleep(period)
        finally:
            io.all_off()
            print("\nTodas apagadas.")


def _probe(channel):
    """Cambia UNA salida y guia la medicion para determinar la polaridad."""
    do_pin = channel + 3          # DO0 = pin 3
    with NodkaIO(verbose=True) as io:
        raw0 = io._read_do_byte()
        print()
        print(f"Registro de salidas en reposo: 0x{raw0:02X}  [{raw0:08b}]")
        print(f"DO_INVERT esta en {DO_INVERT}")
        print()
        print("Poné el tester en TENSION CONTINUA entre:")
        print(f"    pin {do_pin} (DO{channel})   y   pin 2 (DOGND)")
        print()
        print("Y acordate de alimentar las salidas: pin 1 (DO-24V) a +24 V,")
        print("pin 2 (DOGND) a 0 V. Sin eso no vas a medir nada.")
        print()
        input("Enter cuando tengas el tester puesto...")

        for label, state in (("ENCENDIDA", True), ("APAGADA", False)):
            io.write_do(channel, state)
            raw = io._read_do_byte()
            print()
            print(f"  DO{channel} -> {label}")
            print(f"      registro: 0x{raw:02X}  [{raw:08b}]")
            print(f"      deberias medir {'~24 V' if state else '~0 V'}")
            input("      Enter para seguir...")

        io.all_off()
        raw = io._read_do_byte()
        print()
        print(f"Todas apagadas. Registro: 0x{raw:02X}  [{raw:08b}]")
        print()
        if raw == raw0:
            print("Coincide con el estado de reposo inicial: DO_INVERT esta bien.")
        else:
            print("NO coincide con el reposo inicial (0x%02X)." % raw0)
            print("Si al pedir ENCENDIDA medias 0 V y al pedir APAGADA medias")
            print("24 V, la polaridad esta al reves: cambiá DO_INVERT en nkio.py.")


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 0

    try:
        cmd = args[0]
        if cmd == "info":
            _info()
        elif cmd == "monitor":
            _monitor()
        elif cmd == "set" and len(args) == 3:
            _set(int(args[1]), args[2].lower() in ("on", "1", "true"))
        elif cmd == "blink" and len(args) >= 2:
            _blink(int(args[1]))
        elif cmd == "walk":
            _walk()
        elif cmd == "probe" and len(args) >= 2:
            _probe(int(args[1]))
        else:
            print(__doc__)
            return 1
    except KeyboardInterrupt:
        print()
    except NkioError as exc:
        print(f"\nERROR NKIOLIB: {exc}")
        return 1
    except ValueError as exc:
        print(f"\nERROR: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
