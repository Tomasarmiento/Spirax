"""
focas.py — Wrapper ctypes de la librería Fanuc FOCAS2.

Encapsula las funciones de Fwlib32.dll que necesitamos para hablar con
el torno Fanuc 0i-TF. Diseñado a partir del header oficial Fwlib32.h.

Notas importantes:
- Las structs usan _pack_ = 4 (alignment de 4 bytes) como exige Fanuc.
- MAX_AXIS = 32 y MAX_SPINDLE = 8 para Series 0i-F (FOCAS2).
- Las DLLs deben estar en la carpeta que se pasa al __init__ de Focas.
- Solo Windows. Si fueras a Linux usarías libfwlib32.so en su lugar.
"""

from __future__ import annotations

import os
import sys
import ctypes
from ctypes import (
    Structure, POINTER, byref,
    c_char, c_byte, c_short, c_ushort, c_long, c_ulong, c_int,
)

# Constantes del control 0i-F
MAX_AXIS = 32
MAX_SPINDLE = 8

# Códigos de retorno comunes (de Fwlib32.h)
EW_OK = 0
EW_NUMBER = 7
EW_NOOPT = 6
EW_PROTOCOL = -8
EW_SOCKET = -9
EW_ALARM = -10
EW_STOP = -11

# Mapeo legible para imprimir errores
RETURN_CODE_NAMES = {
    0: "EW_OK",
    1: "EW_FUNC",
    2: "EW_LENGTH",
    3: "EW_NUMBER",
    4: "EW_ATTRIB",
    5: "EW_DATA",
    6: "EW_NOOPT",
    7: "EW_PROT",
    8: "EW_BUSY",
    9: "EW_RESET",
    10: "EW_ALARM",
    11: "EW_STOP",
    12: "EW_MMCSYS",
    13: "EW_SYSTEM",
    14: "EW_PARITY",
    15: "EW_OVRFLOW",
    16: "EW_RS232C",
    17: "EW_SOCKET",
    -8: "EW_PROTOCOL",
    -9: "EW_SOCKET",
    -10: "EW_ALARM",
    -11: "EW_STOP",
    -16: "EW_HANDLE",
    -17: "EW_VERSION",
    -18: "EW_UNEXP",
}


def err_name(code: int) -> str:
    return RETURN_CODE_NAMES.get(code, f"UNKNOWN({code})")


# ─────────────────────────────────────────────────────────────────
# Estructuras de datos (copiadas de Fwlib32.h, alignment 4 bytes)
# ─────────────────────────────────────────────────────────────────

class ODBSYS(Structure):
    """cnc_sysinfo — info de sistema del CNC."""
    _pack_ = 4
    _fields_ = [
        ("dummy",    c_short),
        ("max_axis", c_char * 2),
        ("cnc_type", c_char * 2),
        ("mt_type",  c_char * 2),
        ("series",   c_char * 4),
        ("version",  c_char * 4),
        ("axes",     c_char * 2),
    ]


class ODBST2(Structure):
    """cnc_statinfo2 — estado del CNC."""
    _pack_ = 4
    _fields_ = [
        ("hdck",      c_short),  # handle retrace status
        ("tmmode",    c_short),  # T/M mode
        ("aut",       c_short),  # automatic mode selected
        ("run",       c_short),  # running status
        ("motion",    c_short),  # axis, dwell status
        ("mstb",      c_short),  # M/S/T/B status
        ("emergency", c_short),  # emergency stop
        ("alarm",     c_short),  # alarm status
        ("edit",      c_short),  # editing status
        ("warning",   c_short),
        ("o3dchk",    c_short),
        ("ext_opt",   c_short),
        ("restart",   c_short),
    ]


class ODBM(Structure):
    """cnc_rdmacro — lee una variable macro (#100, #500, etc.)."""
    _pack_ = 4
    _fields_ = [
        ("datano",  c_short),
        ("dummy",   c_short),
        ("mcr_val", c_long),   # valor entero * 10^dec_val
        ("dec_val", c_short),  # cantidad de decimales
    ]


class IODBPMC(Structure):
    """pmc_rdpmcrng — lee un rango de la memoria PMC.
    Layout para lectura de bytes: type_a (addr_t), type_d (data_t),
    datano_s, datano_e, y el buffer de datos. Usamos un buffer de 8 bytes
    (suficiente para leer 1-8 bytes contiguos, p.ej. X10)."""
    _pack_ = 4
    _fields_ = [
        ("type_a",   c_short),        # tipo de direccion PMC (X=0, Y=1, ...)
        ("type_d",   c_short),        # tipo de dato (0 = byte)
        ("datano_s", c_short),        # numero de byte inicial
        ("datano_e", c_short),        # numero de byte final
        ("cdata",    c_byte * 8),     # buffer de datos (bytes leidos)
    ]


class ODBACT(Structure):
    """cnc_acts — velocidad real del spindle."""
    _pack_ = 4
    _fields_ = [
        ("dummy", c_short * 2),
        ("data",  c_long),
    ]


class ODBAXIS(Structure):
    """cnc_absolute2 — posición de ejes."""
    _pack_ = 4
    _fields_ = [
        ("dummy", c_short),
        ("type",  c_short),
        ("data",  c_long * MAX_AXIS),
    ]


class ODBSPN(Structure):
    """cnc_rdspload — carga del spindle."""
    _pack_ = 4
    _fields_ = [
        ("datano", c_short),
        ("type",   c_short),
        ("data",   c_short * MAX_SPINDLE),
    ]


class OPMSG(Structure):
    """cnc_rdopmsg — mensaje del operador (256 chars)."""
    _pack_ = 4
    _fields_ = [
        ("datano",   c_short),
        ("type",     c_short),
        ("char_num", c_short),
        ("data",     c_char * 256),
    ]


# ─────────────────────────────────────────────────────────────────
# Clase principal — carga las DLLs y expone las funciones
# ─────────────────────────────────────────────────────────────────

class Focas:
    """Wrapper de Fwlib32.dll. Crear UNA instancia y reusarla."""

    def __init__(self, dll_path: str = "./dlls"):
        if sys.platform != "win32":
            raise RuntimeError(
                "Este wrapper es solo Windows. En Linux usar libfwlib32.so."
            )

        dll_path = os.path.abspath(dll_path)
        if not os.path.isdir(dll_path):
            raise FileNotFoundError(f"No existe la carpeta de DLLs: {dll_path}")

        # Importante: agregar la carpeta al PATH del proceso ANTES de cargar
        # Fwlib32.dll, porque ella busca fwlibe1.dll y fwlib30i.dll por nombre
        # en el mismo directorio.
        try:
            # Python 3.8+: forma recomendada
            os.add_dll_directory(dll_path)
        except AttributeError:
            os.environ["PATH"] = dll_path + os.pathsep + os.environ["PATH"]

        # En Windows 64 bits usamos Fwlib64.dll, en 32 bits Fwlib32.dll.
        # Detección por arquitectura del intérprete Python.
        import struct
        is_64bit = struct.calcsize("P") == 8
        dll_name = "Fwlib64.dll"
        dll_file = os.path.join(dll_path, dll_name)
        if not os.path.isfile(dll_file):
            raise FileNotFoundError(
                f"Falta {dll_name} en {dll_path}. "
                f"Python es {'64' if is_64bit else '32'} bits, "
                f"necesitás la DLL de esa arquitectura."
            )

        self._lib = ctypes.WinDLL(dll_file)
        self._setup_prototypes()

    def _setup_prototypes(self) -> None:
        """Define argtypes/restype de cada función para evitar bugs sutiles."""
        L = self._lib

        # cnc_allclibhndl3(host, port, timeout, &handle)
        L.cnc_allclibhndl3.argtypes = [
            ctypes.c_char_p, c_ushort, c_long, POINTER(c_ushort)
        ]
        L.cnc_allclibhndl3.restype = c_short

        # cnc_freelibhndl(handle)
        L.cnc_freelibhndl.argtypes = [c_ushort]
        L.cnc_freelibhndl.restype = c_short

        # cnc_sysinfo(handle, &odbsys)
        L.cnc_sysinfo.argtypes = [c_ushort, POINTER(ODBSYS)]
        L.cnc_sysinfo.restype = c_short

        # cnc_statinfo2(handle, &odbst2)
        L.cnc_statinfo2.argtypes = [c_ushort, POINTER(ODBST2)]
        L.cnc_statinfo2.restype = c_short

        # cnc_rdmacro(handle, num, length, &odbm)
        L.cnc_rdmacro.argtypes = [c_ushort, c_short, c_short, POINTER(ODBM)]
        L.cnc_rdmacro.restype = c_short

        # cnc_wrmacro(handle, num, length, val, dec) — para escribir una macro
        # Firma del header: short cnc_wrmacro(unsigned short, short, short, long, short)
        L.cnc_wrmacro.argtypes = [c_ushort, c_short, c_short, c_long, c_short]
        L.cnc_wrmacro.restype = c_short

        # cnc_acts(handle, &odbact)
        L.cnc_acts.argtypes = [c_ushort, POINTER(ODBACT)]
        L.cnc_acts.restype = c_short

        # cnc_absolute2(handle, axis, length, &odbaxis)
        # axis = -1 para todos los ejes
        L.cnc_absolute2.argtypes = [c_ushort, c_short, c_short, POINTER(ODBAXIS)]
        L.cnc_absolute2.restype = c_short

        # cnc_rdspload(handle, type, &odbspn)
        L.cnc_rdspload.argtypes = [c_ushort, c_short, POINTER(ODBSPN)]
        L.cnc_rdspload.restype = c_short

        # cnc_rdopmsg(handle, type, length, &opmsg)
        L.cnc_rdopmsg.argtypes = [c_ushort, c_short, c_short, POINTER(OPMSG)]
        L.cnc_rdopmsg.restype = c_short

        # cnc_alarm2(handle, &long)
        L.cnc_alarm2.argtypes = [c_ushort, POINTER(c_long)]
        L.cnc_alarm2.restype = c_short

        # pmc_rdpmcrng(handle, adr_type, data_type, start, end, length, &iodbpmc)
        L.pmc_rdpmcrng.argtypes = [c_ushort, c_short, c_short, c_short,
                                   c_short, c_short, POINTER(IODBPMC)]
        L.pmc_rdpmcrng.restype = c_short

        # pmc_wrpmcrng(handle, length, &iodbpmc)  -- MISMA estructura que rd
        L.pmc_wrpmcrng.argtypes = [c_ushort, c_ushort, POINTER(IODBPMC)]
        L.pmc_wrpmcrng.restype = c_short

        # cnc_rdparam(handle, num, axis, length, &iodbpsd)
        # Lo dejamos en raw bytes porque IODBPSD es polimórfico — para el part
        # count (param 6711) sirve con esta firma genérica.
        # Lo usaremos con un buffer de tipo c_long.
        L.cnc_rdparam.argtypes = [c_ushort, c_short, c_short, c_short,
                                  ctypes.c_void_p]
        L.cnc_rdparam.restype = c_short

        # cnc_rdexecprog(handle, &length, &blknum, *buf)
        L.cnc_rdexecprog.argtypes = [c_ushort, POINTER(c_ushort),
                                     POINTER(c_short), ctypes.c_char_p]
        L.cnc_rdexecprog.restype = c_short

    # ─── Connection ──────────────────────────────────────────────

    def allclibhndl3(self, host: str, port: int = 8193, timeout: int = 10):
        """Abre handle FOCAS. Devuelve (handle, ret_code)."""
        handle = c_ushort(0)
        ret = self._lib.cnc_allclibhndl3(
            host.encode("ascii"), port, timeout, byref(handle)
        )
        return handle.value, ret

    def freelibhndl(self, handle: int) -> int:
        return self._lib.cnc_freelibhndl(handle)

    # ─── Lecturas ────────────────────────────────────────────────

    def sysinfo(self, handle: int):
        s = ODBSYS()
        ret = self._lib.cnc_sysinfo(handle, byref(s))
        return s, ret

    def statinfo2(self, handle: int):
        s = ODBST2()
        ret = self._lib.cnc_statinfo2(handle, byref(s))
        return s, ret

    def rdmacro(self, handle: int, num: int):
        m = ODBM()
        ret = self._lib.cnc_rdmacro(handle, num, ctypes.sizeof(ODBM), byref(m))
        return m, ret

    def rdpmcrng_byte(self, handle: int, adr_type: int, start: int, end: int):
        """Lee bytes contiguos del PMC (data_type=0). adr_type: X=0,Y=1,R=5,...
        Devuelve (IODBPMC, ret). Los bytes quedan en .cdata[0..(end-start)]."""
        p = IODBPMC()
        n = (end - start) + 1              # cantidad de bytes
        length = 8 + n                     # header (8) + datos
        ret = self._lib.pmc_rdpmcrng(
            handle, c_short(adr_type), c_short(0),
            c_short(start), c_short(end), c_short(length), byref(p))
        return p, ret

    def wrpmcrng_byte(self, handle: int, adr_type: int, byte_num: int,
                      value: int, length: int = None):
        """Escribe UN byte en el PMC (data_type=0). adr_type: R=5, D=9, ...
        Firma FOCAS: pmc_wrpmcrng(handle, length, &IODBPMC) usando la MISMA
        estructura que la lectura. length = 8 (header) + n_bytes de datos."""
        n = 1
        if length is None:
            length = 8 + n     # header 8 + 1 byte de dato = 9
        p = IODBPMC()
        p.type_a = adr_type
        p.type_d = 0                 # byte
        p.datano_s = byte_num
        p.datano_e = byte_num
        p.cdata[0] = value & 0xFF
        ret = self._lib.pmc_wrpmcrng(handle, c_ushort(length), byref(p))
        return ret

    def acts(self, handle: int):
        a = ODBACT()
        ret = self._lib.cnc_acts(handle, byref(a))
        return a, ret

    def absolute2(self, handle: int):
        """Posición absoluta de todos los ejes."""
        a = ODBAXIS()
        ret = self._lib.cnc_absolute2(handle, -1, ctypes.sizeof(ODBAXIS), byref(a))
        return a, ret

    def rdspload(self, handle: int):
        s = ODBSPN()
        ret = self._lib.cnc_rdspload(handle, -1, byref(s))
        return s, ret

    def rdopmsg(self, handle: int):
        m = OPMSG()
        # type=0 → todos los mensajes; length = sizeof(OPMSG) — 256 + 6 header
        ret = self._lib.cnc_rdopmsg(handle, 0, ctypes.sizeof(OPMSG), byref(m))
        return m, ret

    def alarm2(self, handle: int):
        """Devuelve bitmask de tipos de alarma activos."""
        a = c_long(0)
        ret = self._lib.cnc_alarm2(handle, byref(a))
        return a.value, ret

    def rdpartcount(self, handle: int):
        """Lee parámetro 6711 (contador de piezas)."""
        # IODBPSD para un parámetro tipo long ocupa 16 bytes:
        # short datano, short type, long ldata, padding
        buf = (ctypes.c_byte * 16)()
        ret = self._lib.cnc_rdparam(handle, 6711, 0, ctypes.sizeof(buf), buf)
        # El long ldata empieza en offset 4 (después de datano + type)
        ldata = ctypes.cast(ctypes.addressof(buf) + 4,
                            POINTER(c_long)).contents.value
        return ldata, ret

    def rdexecprog(self, handle: int):
        """Devuelve (bloque, número_línea, ret_code).

        El buffer trae el texto crudo del bloque que se está ejecutando.
        """
        length = c_ushort(256)
        blknum = c_short(0)
        buf = ctypes.create_string_buffer(256)
        ret = self._lib.cnc_rdexecprog(handle, byref(length), byref(blknum), buf)
        return buf.value.decode("latin-1", errors="replace"), blknum.value, ret

    # ─── Escritura ───────────────────────────────────────────────

    def wrmacro(self, handle: int, num: int, value: int, dec: int = 0) -> int:
        """Escribe una variable macro.

        value es el entero ya multiplicado por 10^dec.
        Para escribir 1.5 en #500: wrmacro(h, 500, 15, dec=1)
        Para escribir 10 entero en #501: wrmacro(h, 501, 10, dec=0)
        """
        return self._lib.cnc_wrmacro(handle, num, 10, value, dec)


# ─────────────────────────────────────────────────────────────────
# Decodificadores para los campos enumerados de ODBST2
# Referencia: doc oficial de cnc_statinfo2 y código de Sinapsis
# ─────────────────────────────────────────────────────────────────

AUT_MODES = {
    0: "MDI", 1: "MEMORY", 2: "****", 3: "EDIT", 4: "HANDLE",
    5: "JOG", 6: "TEACH IN JOG", 7: "TEACH IN HANDLE", 8: "INC FEED",
    9: "REFERENCE", 10: "REMOTE",
}

RUN_STATUS = {
    0: "STOP", 1: "HOLD", 2: "START", 3: "MSTR",
    4: "RESTART", 5: "PRSR", 6: "NSRC",
}

EMERGENCY_STATUS = {
    0: "NORMAL", 1: "EMERGENCY", 2: "RESET", 3: "WAIT",
}

ALARM_STATUS = {
    0: "NORMAL", 1: "ALARM",
}

TM_MODE = {0: "T (torno)", 1: "M (fresadora)"}


def decode_status(s: ODBST2) -> dict:
    """Convierte un ODBST2 en dict legible."""
    return {
        "modo":        AUT_MODES.get(s.aut, f"?{s.aut}"),
        "ejecucion":   RUN_STATUS.get(s.run, f"?{s.run}"),
        "emergencia":  EMERGENCY_STATUS.get(s.emergency, f"?{s.emergency}"),
        "alarma":      "SÍ" if s.alarm else "no",
        "tipo":        TM_MODE.get(s.tmmode, f"?{s.tmmode}"),
        "edicion":     s.edit,
        "motion":      s.motion,
    }


def decode_sysinfo(s: ODBSYS) -> dict:
    """ODBSYS → dict legible (los campos son chars no nul-terminated)."""
    return {
        "max_axis": bytes(s.max_axis).decode("ascii", errors="replace").strip(),
        "cnc_type": bytes(s.cnc_type).decode("ascii", errors="replace").strip(),
        "mt_type":  bytes(s.mt_type).decode("ascii", errors="replace").strip(),
        "series":   bytes(s.series).decode("ascii", errors="replace").strip(),
        "version":  bytes(s.version).decode("ascii", errors="replace").strip(),
        "axes":     bytes(s.axes).decode("ascii", errors="replace").strip(),
    }