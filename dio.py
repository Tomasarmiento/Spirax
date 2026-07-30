"""
dio.py - Sensores del pre-stopper del husillo via modulo DIO del panel Nodka.

Dos sensores fisicos cableados a las entradas digitales del panel:

    DI0 = op_20           (la pieza esta en orientacion de OP20)
    DI1 = presencia_pieza (hay pieza en el pallet)

Interpretacion (definida en planta):
    op_20 == 1                     -> la pieza es OP20   -> op = 2
    presencia == 1 y op_20 == 0    -> la pieza es OP10   -> op = 1
    los dos en 0                   -> NO hay pieza       -> op = 0

Se usa para CONFIRMAR la lectura de la camara antes de mandarle la receta al
torno: lo que dice la cola (que vino de la camara) tiene que coincidir con lo
que dicen estos sensores. Si no coinciden, no se libera el pre-stopper.

NOTAS DE INTEGRACION (del handoff del panel):
- El SMBus es un recurso compartido y NKIOLIB no es thread-safe: hay UNA sola
  instancia global, protegida por un lock. Todo el mundo entra por aca.
- Requiere privilegios de ADMINISTRADOR.
- WinRing0x64.sys tiene que estar en la carpeta del ejecutable del proceso
  (la del python.exe / del venv, o al lado del .exe con PyInstaller). Si no,
  NKDIO_LibraryInit devuelve 3.
- Si el driver no esta disponible, este modulo NO explota: queda en estado
  "no disponible" y quien lo use decide que hacer (ver torno.py, config
  verificacion_obligatoria).
"""

import os
import sys
import threading

# --- Canales de entrada ---
DI_OP20 = 0        # DI0
DI_PRESENCIA = 1   # DI1

# Resultado de la lectura fisica
OP_NINGUNA = 0     # no hay pieza
OP_10 = 1          # pieza en OP10  -> coincide con op=1 de la cola
OP_20 = 2          # pieza en OP20  -> coincide con op=2 de la cola


class DioNoDisponible(RuntimeError):
    """El modulo DIO no se pudo inicializar (driver, permisos, config)."""


class _DioSingleton:
    """Dueño unico del bus. Inicializacion perezosa y tolerante a fallos."""

    def __init__(self):
        self._lock = threading.Lock()
        self._io = None
        self._intentado = False
        self._error = None

    # ---------- ciclo de vida ----------

    def _asegurar(self):
        """Inicializa el DIO la primera vez. Asume lock tomado."""
        if self._io is not None:
            return
        if self._intentado:
            # Ya fallo antes: no reintentar en cada lectura (cada intento
            # carga el driver y es lento).
            raise DioNoDisponible(self._error or "DIO no disponible")

        self._intentado = True
        try:
            # Buscar nkio.py junto a este archivo
            aqui = os.path.dirname(os.path.abspath(__file__))
            if aqui not in sys.path:
                sys.path.insert(0, aqui)
            from nkio import NodkaIO
            self._io = NodkaIO()
        except Exception as e:
            self._error = f"{type(e).__name__}: {e}"
            raise DioNoDisponible(self._error)

    def reintentar(self):
        """Permite volver a intentar la inicializacion (boton en la HMI)."""
        with self._lock:
            self._intentado = False
            self._error = None
            if self._io is not None:
                try:
                    self._io.close()
                except Exception:
                    pass
                self._io = None

    def cerrar(self):
        with self._lock:
            if self._io is not None:
                try:
                    self._io.close()
                except Exception:
                    pass
                self._io = None

    # ---------- consultas ----------

    def disponible(self):
        """True si el DIO esta listo para leer. No lanza."""
        with self._lock:
            try:
                self._asegurar()
                return True
            except DioNoDisponible:
                return False

    def ultimo_error(self):
        return self._error

    def leer_sensores(self):
        """Devuelve (op20, presencia) como booleanos. Lanza DioNoDisponible."""
        with self._lock:
            self._asegurar()
            entradas = self._io.read_all_di()
            return bool(entradas[DI_OP20]), bool(entradas[DI_PRESENCIA])

    def leer_todas(self):
        """Las 8 entradas, para mostrar en la HMI. Lanza DioNoDisponible."""
        with self._lock:
            self._asegurar()
            return list(self._io.read_all_di())

    def op_fisica(self):
        """Traduce los dos sensores a la operacion que corresponde.

        Devuelve OP_20 (2), OP_10 (1) u OP_NINGUNA (0).
        Lanza DioNoDisponible si el modulo no esta.
        """
        op20, presencia = self.leer_sensores()
        if op20:
            return OP_20
        if presencia:
            return OP_10
        return OP_NINGUNA


    # ---------- monitor en background ----------
    # El handoff del panel avisa: NO hacer polling I2C desde el hilo de la UI.
    # Este hilo lee cada `periodo` y deja el ultimo valor cacheado; la HMI lee
    # el cache (ultimas_entradas / ultimas_salidas) sin tocar el bus.

    def iniciar_monitor(self, periodo=0.1):
        """Arranca el hilo que refresca el cache de entradas/salidas."""
        if getattr(self, "_mon_thread", None) is not None:
            return
        self._mon_stop = threading.Event()
        self._ultimas_di = None
        self._ultimas_do = None
        self._mon_error = None
        self._mon_periodo = periodo
        self._mon_thread = threading.Thread(
            target=self._loop_monitor, daemon=True, name="DioMonitor")
        self._mon_thread.start()

    def detener_monitor(self):
        ev = getattr(self, "_mon_stop", None)
        if ev is not None:
            ev.set()
        t = getattr(self, "_mon_thread", None)
        if t is not None:
            t.join(timeout=1.0)
        self._mon_thread = None

    def _loop_monitor(self):
        while not self._mon_stop.is_set():
            try:
                with self._lock:
                    self._asegurar()
                    raw_di = self._io.read_di_byte()
                    di = [self._io._di_level(raw_di, ch)
                          for ch in range(self._io.N_DI)]
                    do = list(self._io.read_all_do())
                self._ultimas_di = di
                self._ultimas_do = do
                self._mon_error = None
            except Exception as e:
                self._ultimas_di = None
                self._ultimas_do = None
                self._mon_error = f"{type(e).__name__}: {e}"
                self._mon_stop.wait(1.0)
                continue
            self._mon_stop.wait(self._mon_periodo)

    def ultimas_entradas(self):
        """Cache de las 8 entradas (o None si no hay lectura valida)."""
        return getattr(self, "_ultimas_di", None)

    def ultimas_salidas(self):
        """Cache de las 8 salidas (o None)."""
        return getattr(self, "_ultimas_do", None)

    def error_monitor(self):
        return getattr(self, "_mon_error", None)


# Instancia global: UN solo dueño del bus en toda la aplicacion.
dio = _DioSingleton()


def nombre_op(valor):
    """Texto legible para logs y HMI."""
    return {OP_NINGUNA: "sin pieza",
            OP_10: "OP10",
            OP_20: "OP20"}.get(valor, f"?{valor}")


if __name__ == "__main__":
    # Prueba rapida: python dio.py
    if not dio.disponible():
        print(f"[X] DIO no disponible: {dio.ultimo_error()}")
        raise SystemExit(1)
    import time
    print("Leyendo sensores del pre-stopper. Ctrl+C para salir.\n")
    ultimo = None
    try:
        while True:
            op20, pres = dio.leer_sensores()
            op = dio.op_fisica()
            actual = (op20, pres)
            if actual != ultimo:
                print(f"DI0 op_20={int(op20)}  DI1 presencia={int(pres)}"
                      f"   -> {nombre_op(op)}")
                ultimo = actual
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("\nSaliendo.")
    finally:
        dio.cerrar()