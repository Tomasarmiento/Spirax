"""
torno.py - Integracion con torno Fanuc para Spirax

Componentes:
- Cola FIFO persistida en disco (cola_torno.json)
- Cliente Focas conectado en background
- Thread sincronizador que:
    1. Mantiene escrito en #500/#501 el primer elemento de la cola
    2. Pollea #503 (flag "termine"), cuando lo ve hace popleft + actualiza macros + baja #503
    3. Si la cola esta vacia, escribe ceros en #500/#501

Convencion de macros (configurable):
  #500 = TIPO de pieza (1-6, 0 = sin pieza)
  #501 = OPERACION (10 o 20, 0 = sin pieza)
  #503 = FLAG fin de mecanizado (torno escribe 1, PC limpia a 0)

El programa NC del torno debe usar #100/#101 como snapshot local
de #500/#501 al pickear, y al terminar setear #503=1 y esperar a 0.
"""

import json
import os
import threading
import time
from collections import deque
from datetime import datetime

# Importar el cliente Fanuc. Si no esta instalado, el modulo igual carga
# pero las llamadas a start() van a fallar con un mensaje claro.
try:
    from fanuc_bridge.client import FanucClient, FocasError
    FANUC_DISPONIBLE = True
except ImportError:
    try:
        # Fallback: si el repo esta clonado en src/ y se accede directo
        import sys
        from pathlib import Path
        posibles = [
            Path(__file__).parent / "fanuc-bridge" / "src",
            Path(__file__).parent / "fanuc_bridge",
            Path.cwd() / "fanuc-bridge" / "src",
        ]
        for p in posibles:
            if (p / "client.py").exists():
                sys.path.insert(0, str(p.parent))
                from src.client import FanucClient, FocasError
                FANUC_DISPONIBLE = True
                break
        else:
            FANUC_DISPONIBLE = False
            FanucClient = None
            FocasError = Exception
    except ImportError:
        FANUC_DISPONIBLE = False
        FanucClient = None
        FocasError = Exception


# ======================================================================
#  Configuracion por defecto
# ======================================================================

DEFAULTS = {
    "torno_host": "172.31.1.99",
    "torno_port": 8193,
    "torno_timeout_focas": 10,
    "torno_dll_path": "./dlls",
    "macro_tipo": 500,
    "macro_op": 501,
    "macro_fin": 503,
    "polling_ms": 500,        # intervalo de polling de #503
    "persist_path": "cola_torno.json",
}


# ======================================================================
#  Cola FIFO persistida
# ======================================================================

class ColaTorno:
    """Cola FIFO thread-safe persistida en disco como JSON.

    Cada elemento es un dict: {"tipo": int, "op": int, "ts": str}
    El timestamp es para auditoria/HMI, no afecta logica.
    """

    def __init__(self, path):
        self._path = path
        self._lock = threading.Lock()
        self._cola = deque()
        self._cargar()

    def _cargar(self):
        if not os.path.exists(self._path):
            return
        try:
            with open(self._path, "r") as f:
                data = json.load(f)
            if isinstance(data, list):
                self._cola = deque(data)
        except Exception as e:
            print(f"[COLA] Error cargando {self._path}: {e}")

    def _persistir(self):
        """Guarda la cola actual a disco. Asume que ya tenemos el lock."""
        try:
            tmp = self._path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(list(self._cola), f, indent=2)
            os.replace(tmp, self._path)
        except Exception as e:
            print(f"[COLA] Error guardando {self._path}: {e}")

    def encolar(self, tipo, op):
        """Agrega al final de la cola."""
        with self._lock:
            item = {
                "tipo": int(tipo),
                "op": int(op),
                "ts": datetime.now().strftime("%H:%M:%S"),
            }
            self._cola.append(item)
            self._persistir()
        return item

    def primero(self):
        """Devuelve el primero sin sacarlo (snapshot dict, o None)."""
        with self._lock:
            if not self._cola:
                return None
            return dict(self._cola[0])

    def desencolar(self):
        """Saca el primero y lo devuelve. None si esta vacia."""
        with self._lock:
            if not self._cola:
                return None
            item = self._cola.popleft()
            self._persistir()
            return dict(item)

    def __len__(self):
        with self._lock:
            return len(self._cola)

    def snapshot(self):
        """Lista snapshot inmutable para la HMI."""
        with self._lock:
            return [dict(x) for x in self._cola]

    def limpiar(self):
        """Vacia toda la cola."""
        with self._lock:
            self._cola.clear()
            self._persistir()

    def quitar_indice(self, indice):
        """Quita el elemento en el indice dado. Devuelve True si lo saco."""
        with self._lock:
            if 0 <= indice < len(self._cola):
                # deque no soporta del[i] directo en versiones viejas,
                # pero si .remove(). Como tenemos que sacar por indice,
                # convertimos a list temporal.
                items = list(self._cola)
                items.pop(indice)
                self._cola = deque(items)
                self._persistir()
                return True
            return False


# ======================================================================
#  Cliente del torno con thread sincronizador
# ======================================================================

class TornoFanuc:
    """Conexion + cola + sync con el torno.

    Uso:
        torno = TornoFanuc(config)
        torno.start()
        ...
        torno.encolar(1, 10)
        ...
        torno.stop()

    Estados publicos para la HMI:
        torno.conectado          -> bool
        torno.ultimo_error       -> str | None
        torno.ultimo_evento      -> str
        torno.cola               -> ColaTorno
    """

    def __init__(self, config=None, log_callback=None):
        self.config = dict(DEFAULTS)
        if config:
            self.config.update(config)

        self.cola = ColaTorno(self.config["persist_path"])

        self.conectado = False
        self.ultimo_error = None
        self.ultimo_evento = ""
        self.piezas_terminadas = 0

        self._log_callback = log_callback or (lambda m: None)
        self._client = None
        self._client_lock = threading.Lock()
        self._sync_thread = None
        self._shutdown = False
        # Para saber si tenemos que reescribir macros (cuando cambia cola[0])
        self._ultimo_escrito = (None, None)  # (tipo, op)

    # ------------------------------------------------------------------
    #  Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Lanza el thread sincronizador. La conexion se intenta dentro."""
        if not FANUC_DISPONIBLE:
            self._log("!!! fanuc_bridge no disponible (revisa instalacion)")
            self.ultimo_error = "fanuc_bridge no instalado"
            return False

        if self._sync_thread is not None and self._sync_thread.is_alive():
            return True

        self._shutdown = False
        self._sync_thread = threading.Thread(
            target=self._loop_sync, daemon=True, name="TornoSync")
        self._sync_thread.start()
        return True

    def stop(self):
        self._shutdown = True
        # Esperar al thread (poco)
        if self._sync_thread is not None:
            self._sync_thread.join(timeout=2.0)
            self._sync_thread = None
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.disconnect()
                except Exception:
                    pass
                self._client = None
        self.conectado = False

    # ------------------------------------------------------------------
    #  Operaciones publicas
    # ------------------------------------------------------------------

    def encolar(self, tipo, op):
        """Encola una pieza (tipo, op). Si la cola estaba vacia, el thread
        sincronizador la escribe a las macros en el proximo ciclo."""
        item = self.cola.encolar(tipo, op)
        self._log(f"[TORNO] Encolado: tipo={tipo} op={op} | cola={len(self.cola)}")
        return item

    def limpiar_cola(self):
        n = len(self.cola)
        self.cola.limpiar()
        self._log(f"[TORNO] Cola limpiada ({n} items removidos)")
        # Forzar reescritura de ceros
        self._ultimo_escrito = (None, None)

    def quitar_indice(self, indice):
        ok = self.cola.quitar_indice(indice)
        if ok:
            self._log(f"[TORNO] Item indice {indice} quitado")
            self._ultimo_escrito = (None, None)
        return ok

    def reconectar(self):
        """Forzar reconexion. El thread se da cuenta solo, pero esto lo apura."""
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.disconnect()
                except Exception:
                    pass
                self._client = None
        self.conectado = False

    # ------------------------------------------------------------------
    #  Inspeccion (para la HMI)
    # ------------------------------------------------------------------

    def read_macro(self, num):
        """Lee una macro arbitraria. Devuelve float o lanza."""
        with self._client_lock:
            if self._client is None or not self.conectado:
                raise RuntimeError("Torno no conectado")
            return self._client.get_macro(int(num))

    def write_macro(self, num, valor, decimals=0):
        """Escribe una macro arbitraria."""
        with self._client_lock:
            if self._client is None or not self.conectado:
                raise RuntimeError("Torno no conectado")
            self._client.set_macro(int(num), float(valor), int(decimals))

    def status(self):
        """Devuelve el TornoStatus del cliente Fanuc, o None si no conectado."""
        with self._client_lock:
            if self._client is None or not self.conectado:
                return None
            try:
                return self._client.status()
            except Exception as e:
                self._log(f"!!! Error leyendo status: {e}")
                return None

    def position(self):
        """Dict con posicion de ejes (X, Z, ...)"""
        with self._client_lock:
            if self._client is None or not self.conectado:
                return None
            try:
                return self._client.position()
            except Exception as e:
                self._log(f"!!! Error leyendo posicion: {e}")
                return None

    def part_count(self):
        with self._client_lock:
            if self._client is None or not self.conectado:
                return None
            try:
                return self._client.part_count()
            except Exception as e:
                return None

    # ------------------------------------------------------------------
    #  Thread sincronizador
    # ------------------------------------------------------------------

    def _loop_sync(self):
        """Loop principal del thread:

        - Si no esta conectado, intenta conectar.
        - Si esta conectado:
          - Lee #503. Si == 1 -> popleft, baja a 0.
          - Compara cola[0] con _ultimo_escrito. Si difiere, escribe #500/#501.
          - Si cola vacia y _ultimo_escrito != (0,0), escribe ceros.
        - Sleep polling_ms.
        """
        polling_s = self.config["polling_ms"] / 1000.0

        while not self._shutdown:
            # Conectar si hace falta
            if not self.conectado:
                self._intentar_conectar()
                if not self.conectado:
                    # Esperar mas antes del proximo intento
                    time.sleep(2.0)
                    continue

            # Conectado: hacer la sincronizacion
            try:
                self._tick_sincronizar()
            except FocasError as e:
                self._log(f"!!! FocasError en sync: {e}")
                self.ultimo_error = str(e)
                self._desconectar_silencioso()
            except Exception as e:
                self._log(f"!!! Error en sync: {e}")
                self.ultimo_error = str(e)
                self._desconectar_silencioso()

            time.sleep(polling_s)

        self._log("[TORNO] Loop sincronizador detenido")

    def _intentar_conectar(self):
        try:
            client = FanucClient(
                host=self.config["torno_host"],
                port=self.config["torno_port"],
                timeout=self.config["torno_timeout_focas"],
                dll_path=self.config["torno_dll_path"],
            )
            client.connect()
            with self._client_lock:
                self._client = client
            self.conectado = True
            self.ultimo_error = None
            self._ultimo_escrito = (None, None)  # forzar reescritura
            self._log(f"[TORNO] Conectado a {self.config['torno_host']}:{self.config['torno_port']}")
        except Exception as e:
            self.ultimo_error = str(e)
            # No spamear log con cada intento fallido
            if "ultimo_log_error_conn" not in self.__dict__ or self.__dict__["ultimo_log_error_conn"] != str(e):
                self._log(f"[TORNO] No se pudo conectar: {e}")
                self.__dict__["ultimo_log_error_conn"] = str(e)
            self.conectado = False

    def _desconectar_silencioso(self):
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.disconnect()
                except Exception:
                    pass
                self._client = None
        self.conectado = False

    def _tick_sincronizar(self):
        """Una iteracion del sync. Asume conectado."""
        mac_tipo = self.config["macro_tipo"]
        mac_op = self.config["macro_op"]
        mac_fin = self.config["macro_fin"]

        # 1. Chequear flag de fin (#503)
        with self._client_lock:
            valor_fin = self._client.get_macro(mac_fin)

        if valor_fin == 1:
            # Torno termino una pieza
            quitado = self.cola.desencolar()
            if quitado is not None:
                self.piezas_terminadas += 1
                self._log(f"[TORNO] Pieza terminada: tipo={quitado['tipo']} "
                          f"op={quitado['op']} | total terminadas={self.piezas_terminadas}")
            else:
                self._log("[TORNO] Torno aviso fin pero cola estaba vacia (?)")
            # Forzar reescritura del nuevo primero
            self._ultimo_escrito = (None, None)
            # Bajar flag
            with self._client_lock:
                self._client.set_macro(mac_fin, 0)

        # 2. Asegurarse que #500/#501 reflejan el primero de la cola
        primero = self.cola.primero()
        if primero is None:
            objetivo = (0, 0)
        else:
            objetivo = (int(primero["tipo"]), int(primero["op"]))

        if objetivo != self._ultimo_escrito:
            with self._client_lock:
                self._client.set_macro(mac_tipo, objetivo[0])
                self._client.set_macro(mac_op, objetivo[1])
            self._ultimo_escrito = objetivo
            if objetivo == (0, 0):
                self.ultimo_evento = f"[TORNO] Cola vacia -> #{mac_tipo}=0 #{mac_op}=0"
            else:
                self.ultimo_evento = (f"[TORNO] Macros actualizadas: "
                                      f"#{mac_tipo}={objetivo[0]} #{mac_op}={objetivo[1]}")
            self._log(self.ultimo_evento)

    def _log(self, msg):
        try:
            self._log_callback(msg)
        except Exception:
            pass
