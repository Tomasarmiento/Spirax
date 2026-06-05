"""
client.py — Cliente de alto nivel para el torno Fanuc.

Encapsula el ciclo handle FOCAS y expone métodos pythónicos:

    with FanucClient("172.31.1.99") as t:
        print(t.status())
        print(t.position())
        t.set_macro(500, 1)   # modelo de pieza
        t.set_macro(501, 10)  # operación
        t.set_macro(502, 1)   # flag listo
"""

from __future__ import annotations

import time
from typing import Optional
from dataclasses import dataclass

from .focas import (
    Focas, decode_status, decode_sysinfo,
    EW_OK, err_name,
)


class FocasError(RuntimeError):
    """Error en una llamada FOCAS. Incluye el código de retorno."""

    def __init__(self, fn: str, code: int):
        super().__init__(f"{fn} falló: {code} ({err_name(code)})")
        self.fn = fn
        self.code = code


@dataclass
class TornoStatus:
    modo: str
    ejecucion: str
    emergencia: str
    alarma: str
    tipo: str
    spindle_rpm: float
    spindle_load_pct: int
    posicion_ejes: dict       # {"X": 125.43, "Z": -42.10, ...}
    programa_actual: str      # texto del bloque
    linea: int                # número de línea
    part_count: int
    mensaje_operador: str     # texto del mensaje activo, "" si no hay
    alarma_bitmask: int       # 0 = sin alarmas


class FanucClient:
    """Cliente conveniente con context manager."""

    def __init__(
        self,
        host: str,
        port: int = 8193,
        timeout: int = 10,
        dll_path: str = "./dlls",
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._focas = Focas(dll_path=dll_path)
        self._handle: Optional[int] = None

    # ─── Context manager ────────────────────────────────────────

    def __enter__(self) -> "FanucClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ─── Connection ─────────────────────────────────────────────

    def connect(self) -> None:
        h, ret = self._focas.allclibhndl3(self.host, self.port, self.timeout)
        if ret != EW_OK:
            raise FocasError("cnc_allclibhndl3", ret)
        self._handle = h

    def disconnect(self) -> None:
        if self._handle is not None:
            self._focas.freelibhndl(self._handle)
            self._handle = None

    @property
    def handle(self) -> int:
        if self._handle is None:
            raise RuntimeError("No conectado. Usar .connect() o context manager.")
        return self._handle

    # ─── Lecturas individuales ──────────────────────────────────

    def sysinfo(self) -> dict:
        s, ret = self._focas.sysinfo(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_sysinfo", ret)
        return decode_sysinfo(s)

    def status_raw(self) -> dict:
        s, ret = self._focas.statinfo2(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_statinfo2", ret)
        return decode_status(s)

    def spindle_rpm(self) -> float:
        a, ret = self._focas.acts(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_acts", ret)
        return float(a.data)

    def spindle_load_pct(self) -> int:
        s, ret = self._focas.rdspload(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_rdspload", ret)
        return int(s.data[0])

    def position(self) -> dict:
        """Posición absoluta de los ejes. Devuelve {axis_name: pos_mm}.

        Como FOCAS no nos da los nombres de los ejes acá, usamos los típicos
        del 0i-TF (torno): eje 0 = X, eje 1 = Z. Si fuera otro torno con más
        ejes, hay que mapearlo distinto.

        El valor data[i] es entero. El factor de conversión a mm depende del
        parámetro 1013 (típicamente 1000 = milésimas de mm). Para empezar
        asumimos /1000.
        """
        a, ret = self._focas.absolute2(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_absolute2", ret)
        n_axes = a.type if a.type > 0 else 2  # típico torno = 2
        nombres = ["X", "Z", "Y", "C", "B", "A"]
        out = {}
        for i in range(min(n_axes, len(nombres))):
            out[nombres[i]] = a.data[i] / 1000.0
        return out

    def part_count(self) -> int:
        v, ret = self._focas.rdpartcount(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_rdparam(6711)", ret)
        return v

    def current_block(self) -> tuple[str, int]:
        text, blk, ret = self._focas.rdexecprog(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_rdexecprog", ret)
        return text, blk

    def operator_message(self) -> str:
        m, ret = self._focas.rdopmsg(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_rdopmsg", ret)
        if m.datano < 0:
            return ""
        return m.data.decode("latin-1", errors="replace").strip("\x00").strip()

    def alarm_bitmask(self) -> int:
        v, ret = self._focas.alarm2(self.handle)
        if ret != EW_OK:
            raise FocasError("cnc_alarm2", ret)
        return v

    # ─── Macros (lectura y escritura) ───────────────────────────

    def get_macro(self, num: int) -> float:
        m, ret = self._focas.rdmacro(self.handle, num)
        if ret != EW_OK:
            raise FocasError(f"cnc_rdmacro(#{num})", ret)
        if m.dec_val == -1:
            return float("nan")  # macro vacía
        return m.mcr_val / (10 ** m.dec_val) if m.dec_val > 0 else float(m.mcr_val)

    def set_macro(self, num: int, value: float, decimals: int = 0) -> None:
        """Escribe una macro. Para enteros, decimals=0."""
        int_value = int(round(value * (10 ** decimals)))
        ret = self._focas.wrmacro(self.handle, num, int_value, decimals)
        if ret != EW_OK:
            raise FocasError(f"cnc_wrmacro(#{num})", ret)

    # ─── Status compuesto (todo de una pasada) ──────────────────

    def status(self) -> TornoStatus:
        """Lectura completa, robusta a errores individuales.

        Si una sub-lectura falla, se rellena con un placeholder y se sigue.
        Así podés mostrar 'lo que pude leer' aunque el torno esté en un
        estado raro (ej. en alarma puede fallar cnc_acts).
        """
        st = self.status_raw()

        def safe(fn, default):
            try:
                return fn()
            except FocasError:
                return default

        return TornoStatus(
            modo=st["modo"],
            ejecucion=st["ejecucion"],
            emergencia=st["emergencia"],
            alarma=st["alarma"],
            tipo=st["tipo"],
            spindle_rpm=safe(self.spindle_rpm, 0.0),
            spindle_load_pct=safe(self.spindle_load_pct, 0),
            posicion_ejes=safe(self.position, {}),
            programa_actual=safe(lambda: self.current_block()[0], ""),
            linea=safe(lambda: self.current_block()[1], 0),
            part_count=safe(self.part_count, 0),
            mensaje_operador=safe(self.operator_message, ""),
            alarma_bitmask=safe(self.alarm_bitmask, 0),
        )

    # ─── Helpers de alto nivel para Spirax ──────────────────────

    def enviar_pieza_a_torno(
        self,
        modelo: int,
        operacion: int,
        macro_modelo: int = 500,
        macro_operacion: int = 501,
        macro_flag: int = 502,
        timeout_torno: float = 30.0,
    ) -> bool:
        """Handshake: envía pieza + operación al torno y espera que arranque.

        Convención:
          #500 = modelo de pieza
          #501 = operación (10 o 20)
          #502 = flag 'datos listos' (PC pone 1, torno limpia a 0 cuando arranca)

        Devuelve True si el torno tomó los datos (flag bajó a 0), False si
        se cumplió el timeout.
        """
        self.set_macro(macro_modelo, modelo)
        self.set_macro(macro_operacion, operacion)
        self.set_macro(macro_flag, 1)

        t0 = time.time()
        while time.time() - t0 < timeout_torno:
            time.sleep(0.5)
            if self.get_macro(macro_flag) == 0:
                return True
        return False

    def esperar_fin_mecanizado(
        self,
        macro_fin: int = 503,
        timeout_torno: float = 300.0,
    ) -> bool:
        """Espera a que el torno setee #503=1 indicando que terminó."""
        t0 = time.time()
        while time.time() - t0 < timeout_torno:
            time.sleep(0.5)
            if self.get_macro(macro_fin) == 1:
                self.set_macro(macro_fin, 0)  # limpio el flag
                return True
        return False
