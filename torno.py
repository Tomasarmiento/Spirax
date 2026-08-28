"""
torno.py - Integracion con torno Fanuc para Spirax

Componentes:
- Cola FIFO persistida en disco (cola_torno.json)
- Cliente Focas conectado en background
- Thread sincronizador que:
    1. Mantiene escrito en #551/#550/#552 el primer elemento de la cola
    2. Pollea #553 (flag "termine"), cuando lo ve hace popleft + actualiza macros + baja #553
    3. Si la cola esta vacia, escribe ceros en #551/#550/#552

Convencion de macros (configurable):
  #551 = NUMERO DE PIEZA (1-12, 0 = sin pieza)   -> 1-6 aluminio, 7-12 fundicion
  #550 = OPERACION A MECANIZAR (10 o 20, 0 = sin pieza)
  #552 = TIPO DE ROSCA (1, 2 o 3, 0 = sin rosca)
  #553 = FLAG fin de mecanizado (torno escribe 1, PC limpia a 0)

El programa NC del torno debe tomar #551/#550/#552 como snapshot local
al pickear, y al terminar setear #553=1 y esperar a 0.
NOTA: la rosca y el numero de pieza (1-12) solo se usan en el torno;
el robot sigue trabajando igual con la forma de la pieza.
"""

import json
import os
import queue
import threading
import time
from concurrent.futures import Future
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
    # Path de las DLLs FOCAS. Se resuelve relativo a ESTE archivo (torno.py)
    # para no depender del directorio desde donde se lanza el HMI.
    "torno_dll_path": os.path.join(os.path.dirname(os.path.abspath(__file__)), "dlls"),
    "macro_tipo": 551,        # NUMERO DE PIEZA (1-12)
    "macro_op": 550,          # OPERACION A MECANIZAR (10 o 20)
    "macro_rosca": 552,       # TIPO DE ROSCA (1, 2 o 3)
    "macro_fin": 553,         # FLAG fin de mecanizado
    "macro_cola": 554,        # CANTIDAD de piezas en la cola (informativo)
    # --- Sensor de salida de pallet (PMC) ---
    # Descuenta 1 de la cola en el flanco de bajada (1->0): el pallet
    # termino de pasar. El ladder ESTIRA el pulso del sensor
    # (que fisicamente es muy corto) a ~2s en R54.0, para que el polling
    # de la PC lo agarre seguro.
    "sensor_salida_pmc_tipo": 5,   # R = 5
    "sensor_salida_byte": 54,      # R54
    "sensor_salida_bit": 0,        # R54.0 (señal estirada 2s por el ladder)
    "usar_sensor_salida": True,    # True = descontar por sensor (no por #553)
    # --- Heartbeat / linea de vida PC<->torno por R50.0 ---
    # El torno (ladder) baja R50.0 a 0; la PC la vuelve a poner en 1 en cada
    # tick. Si la PC muere, R50.0 queda en 0 y el ladder detecta "PC caida".
    "heartbeat_pmc_tipo": 5,       # R = 5
    "heartbeat_byte": 50,          # R50
    "heartbeat_bit": 0,            # R50.0
    "usar_heartbeat": True,
    # --- WATCHDOG DE LA LINEA DE VIDA DEL ROBOT (DI2 del panel Nodka) ---
    # El SPS del robot INVIERTE $OUT[16] cada ~500ms. Esa salida entra por
    # DI2. Lo que se mide NO es el nivel sino que CAMBIE: si el robot se
    # apaga con la salida en 1, la señal se queda en 1 para siempre y un
    # chequeo por nivel no se daria cuenta nunca.
    #
    # PARA QUE SIRVE: con el robot caido la cinta la sigue moviendo el
    # Fammar, asi que los pallets pasan por el spot 1 sin que nadie los
    # fotografie ni los encole. La cola se desfasa de la realidad fisica y
    # ese desfase no se recupera. Al detectar la caida, la PC DEJA DE
    # REPONER R50.0 y el ladder del Fammar corta la cinta.
    "usar_watchdog_robot": True,
    "watchdog_robot_ms": 5000,     # sin cambio de DI2 por mas de esto = caido
    # True  = con el robot caido, tampoco liberar el pre-stopper (doble red)
    "watchdog_bloquea_liberacion": True,
    # --- SYSTEM LINK del Fammar (R55.0) ---
    # En 1 el Fammar obedece las condiciones que le pusimos. En 0 funciona
    # de fabrica: el pre-stopper libera solo y NADA de esto protege.
    "system_link_pmc_tipo": 5,     # R = 5
    "system_link_byte": 55,        # R55
    "system_link_bit": 0,          # R55.0
    "leer_system_link": True,
    # MEDIDO EN LA MAQUINA (13/08): la señal esta INVERTIDA.
    # Con el boton del System Link PRENDIDO en el torno, R55.0 vale 0.
    #   R55.0 = 0  ->  System Link PRENDIDO
    #   R55.0 = 1  ->  System Link APAGADO
    # Si algun dia el ladder cambia y queda directa, poner esto en False.
    "system_link_invertido": True,
    # --- Señal de "receta cargada" para el ladder del Fammar (R55.3) ---
    # La PC la pone en 1 cuando bajo #553 (receta lista). El ladder ve
    # R55.3=1, libera el pre-stopper del husillo y BAJA la R55.3 el mismo.
    "receta_lista_pmc_tipo": 5,    # R = 5
    "receta_lista_byte": 55,       # R55
    "receta_lista_bit": 3,         # R55.3
    # --- VERIFICACION DEL PRE-STOPPER -------------------------------------
    # Antes de mandarle la receta al torno y liberar el pre-stopper, se
    # confirma que la pieza fisica coincide con lo que dice la cola.
    # True = verifica y puede frenar.  False = libera sin mirar nada.
    "verificar_prestopper": True,

    # Sensor del PMC que dice si el pallet esta clampeado en el pre-stopper.
    # Sin esto los sensores DI0/DI1 no son validos todavia.
    "pallet_en_pos_pmc_tipo": 3,   # X = 3
    "pallet_en_pos_byte": 10,      # X10
    "pallet_en_pos_bit": 1,        # X10.1

    # --- QUE ENTRADA DE LA COLA SE USA ------------------------------------
    # En el instante del descuento el pallet mecanizado ya salio y el stopper
    # quedo VACIO. El pre-stopper todavia retiene el suyo (se suelta recien
    # cuando subimos R55.3). O sea que hay UN solo pallet medible y es
    # cola[0]: el mismo que se verifica, se libera y se mecaniza.
    # Los dos en 0 = receta y verificacion apuntan al mismo item.
    "indice_receta": 0,
    "indice_verif": 0,

    # --- MODO OBSERVACION -------------------------------------------------
    # True  = compara y LOGUEA la discrepancia pero libera igual (no frena).
    #         Para juntar datos con la celda produciendo.
    # False = frena de verdad: no escribe receta, no sube R55.3, alarma.
    "verif_solo_log": False,

    # --- INVERTIR LA LECTURA DEL SENSOR -----------------------------------
    # Hipotesis DESCARTADA el 30/07: se midio que op=1 corresponde a OP10 y
    # op=2 a OP20, directo, sin invertir. Dejar en False.
    # Si se pone en True invierte una lectura que ya era correcta y marca
    # discrepancia en todas las piezas buenas.
    "invertir_op_sensor": False,

    # --- R55.3 ANTES DE LIBERAR -------------------------------------------
    # La PC sube R55.3 y el LADDER la baja; la PC nunca la baja. Si quedo en
    # 1, el pre-stopper esta liberado y los pallets pasan sin nuestra orden.
    # CONFIRMADO (30/07): el ladder la baja cuando libera el pre-stopper.
    # Por eso esto va en True: si al ir a liberar la encontramos en 1, algo
    # anda mal (el ladder no la bajo o la libero por su cuenta) y mandar la
    # orden de nuevo no sirve. Se espera y se avisa, en vez de escribir a
    # ciegas y dejar la cola corrida.
    "exigir_receta_lista_en_cero": True,

    # --- PIEZA EN UN PALLET op=3 ------------------------------------------
    # op=3 es "dejar pasar": sale de pallet ABAJO, hueco/recuperacion, o cola
    # sin entrada. En ninguno de los tres el torno mecaniza, asi que no se
    # compara nada. Si los sensores igual ven una pieza, se deja constancia
    # en el log (no frena: los ABAJO son op=3 legitimos y tienen pieza).
    "alertar_pieza_en_op3": True,

    # --- ESPERA ANTES DE MEDIR --------------------------------------------
    # Se cuenta desde el flanco del sensor de salida (el descuento). Durante
    # la espera NO se sube R55.3, asi que el pre-stopper sigue frenado y su
    # pallet no se mueve. 0 = medir al toque.
    # ASENTAMIENTO antes de medir con DI0/DI1.
    # X10.1 dice "hay pallet", no "esta quieto y bien apoyado". Si se mide
    # apenas llega, los sensores pueden leer mal. Este es el tiempo que se
    # espera desde el descuento antes de comparar, en milisegundos.
    # Mismo criterio que el SP_DELAY_CLAMP de los spots en el SPS.
    # 0 = desactivado (compara al toque, como antes).
    "espera_asentamiento_ms": 2000,
    # Si el modulo DIO no esta disponible (driver/permisos):
    #   True  = NO liberar (seguro, pero para la linea)
    #   False = liberar sin verificar, avisando en el log
    "verificacion_obligatoria": True,
    # TRADUCCION op interno -> valor que se escribe en #550.
    # Internamente 1 = OP10 y 2 = OP20 (asi coincide con los sensores del
    # pre-stopper: presencia sin op_20 = OP10, op_20 = OP20). Pero el
    # programa NC del torno puede esperar otros numeros. Esta tabla traduce
    # SOLO al escribir la macro, sin tocar la logica interna ni la
    # verificacion.
    #   Si el torno hace OP20 cuando le mandas OP10, invertir 1 y 2.
    "mapa_op_macro": {1: 1, 2: 2, 3: 3},   # identidad: el NC usa 1/2/3 tal cual
    # Fallos FOCAS consecutivos antes de declarar desconectado y reconectar.
    # Con polling de 200ms, 5 fallos ~= 1 segundo.
    "max_fallos_focas": 5,
    "polling_ms": 200,        # intervalo de polling (sensor + #553 + heartbeat R50)
    # Tiempo MAXIMO reintentando conectar al torno antes de rendirse.
    # Cuenta desde el primer intento fallido; se reinicia al conectar.
    # Cuando se agota, deja de intentar y hay que apretar "Reconectar".
    # 0 = reintentar para siempre.
    "reintento_conexion_max_s": 300,   # 5 minutos
    "reintento_conexion_espera_s": 2.0,
    "persist_path": "cola_torno.json",
}


# ======================================================================
#  Cola FIFO persistida
# ======================================================================

class ColaTorno:
    """Cola FIFO thread-safe persistida en disco como JSON.

    Cada elemento es un dict: {"tipo": int, "op": int, "rosca": int, "ts": str}
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

    def encolar(self, tipo, op, rosca=0):
        """Agrega al final de la cola."""
        with self._lock:
            item = {
                "tipo": int(tipo),
                "op": int(op),
                "rosca": int(rosca),
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

    def todos_op3(self):
        """Cambia el op de TODAS las entradas a 3 (dejar pasar), sin borrar.
        Los pallets fisicos que ya estan en la fila pasan sin mecanizar y la
        cola drena al ritmo del sensor, manteniendo la correspondencia.
        Devuelve cuantas cambio."""
        with self._lock:
            n = 0
            for item in self._cola:
                if item.get("op") != 3:
                    item["op"] = 3
                    n += 1
            self._persistir()
            return n

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

    def recuperar_ultimo_op3(self):
        """Cambia el op del ULTIMO item de la cola a 3 (dejar pasar sin
        mecanizar). Se usa al retomar automatico cuando quedo un pallet
        clampeado en el spot 2: ese pallet ya se proceso y encolo antes
        del corte, es el ultimo de la cola, y hay que reciclarlo -> op=3.
        Devuelve el item modificado, o None si la cola estaba vacia.
        """
        with self._lock:
            if not self._cola:
                return None
            item = self._cola[-1]
            item["op"] = 3
            self._persistir()
            return dict(item)


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
        self._ultimo_escrito = (None, None, None)  # (tipo, op, rosca)
        # Pedido pendiente de PRIMERA PIEZA (lo atiende el thread sync)
        self._pedido_primera_pieza = False
        # Pedido de DEJAR PASAR un pallet frenado por la verificacion
        self._pedido_dejar_pasar = False
        # TRUE cuando el pallet que esta en el torno salio como op=3: ese no
        # levanta #553 (el NC se clava en el M80), asi que el ciclo siguiente
        # no tiene que esperar esa señal. Se consume una sola vez.
        self._op3_en_el_torno = False
        # --- Watchdog de la linea de vida del robot (DI2) ---
        self.robot_lv_ultimo = None       # ultimo nivel visto del pulso
        self.robot_lv_cambio_t = None     # cuando cambio por ultima vez
        self.robot_vivo = None            # None = todavia sin dato
        self.robot_en_auto = None         # DI3
        self._avisado_robot_caido = False
        # Estado del System Link del Fammar (R55.0)
        self.system_link = None
        self._avisado_sl_apagado = False
        # Control de reintentos de conexion
        self._reintento_desde = None
        self._reintento_agotado = False
        # Verificacion del pre-stopper
        self.verificacion_fallida = False
        self.ultimo_pallet_en_pos = None
        self._receta_esperada = None
        self._verif_bloqueada = False
        self.ultimo_sensor_salida = None
        # True desde que se usa PRIMERA PIEZA hasta que se libera el
        # siguiente pallet por el ciclo normal. La HMI lo usa para dejar el
        # checkbox tildado y bloqueado mientras dura.
        self.modo_primera_pieza = False
        self._verif_fallida_msg = None
        self._avisado_esperando_pallet = False
        self._avisado_dio_falla = False
        # Fallos FOCAS consecutivos (para detectar torno apagado y reconectar)
        self._fallos_focas = 0
        self._ultimo_fallo_log = None
        # Cola de pedidos para que TODAS las llamadas FOCAS se hagan
        # desde el thread sincronizador. La fwlib32 de FANUC exige que
        # el handle se use desde el mismo thread que lo creo, sino
        # devuelve EW_PROTOCOL (-8) y similares. Cada item es (Future, fn).
        self._task_queue = queue.Queue()

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
        # Cancelar cualquier pedido pendiente para que los callers no
        # se queden esperando un Future que nunca se va a resolver.
        self._cancelar_queue("Torno detenido")
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

    def encolar(self, tipo, op, rosca=0):
        """Encola una pieza (tipo, op, rosca). Si la cola estaba vacia, el
        thread sincronizador la escribe a las macros en el proximo ciclo.
        rosca solo se manda al torno (el robot no la usa)."""
        item = self.cola.encolar(tipo, op, rosca)
        self._log(f"[TORNO] Encolado: tipo={tipo} op={op} rosca={rosca} "
                  f"| cola={len(self.cola)}")
        return item

    def limpiar_cola(self):
        n = len(self.cola)
        self.cola.limpiar()
        self._log(f"[TORNO] Cola limpiada ({n} items removidos)")

    def liberar_primera_pieza(self):
        """PRIMERA PIEZA / CINTA VACIA.

        Arranque en frio con la cinta vacia: la primera pieza queda trabada
        en el pre-stopper del husillo porque nunca salio un pallet anterior
        (no hay flanco del sensor de salida) ni el torno mando su "termine"
        (#553=1). Este pedido da el empujon inicial:

          1) escribe op=3 en las macros (NO mecanizar: no sabemos en que
             estado quedo la pieza que estaba en el spot 2),
          2) baja #553=0  -> el NC lee las macros y arranca,
          3) sube R55.3=1 -> el ladder libera el pre-stopper.

        Despues el ciclo se normaliza solo: el NC "mecaniza aire" (op=3),
        levanta #553=1, el pallet sale, pasa por el sensor de salida y la
        PC retoma el ciclo normal (descuenta + escribe receta + libera).

        NO ejecuta las llamadas FOCAS aca: solo marca el pedido. El thread
        sincronizador lo atiende en su proximo tick (~polling_ms), porque
        la fwlib32 exige que todas las llamadas salgan de ese thread.
        Si se pide varias veces antes del tick, se ejecuta UNA sola.
        """
        if not self.conectado:
            self._log("[PRIMERA PIEZA] Torno no conectado")
            return False
        # Cinta vacia = no hay pallets en circulacion, asi que la cola no
        # puede corresponder a nada fisico: se borra entera. Si quedara algo,
        # el primer pallet real se mecanizaria con la receta de un fantasma.
        n = len(self.cola)
        if n > 0:
            self.cola.limpiar()
            self._log(f"[PRIMERA PIEZA] Cola borrada ({n} items): la cinta "
                      f"esta vacia, no corresponden a pallets reales")
        # NO se encola nada a mano. El pallet que se libera aca ya fue
        # soltado por el robot con op=3 y el SPS ya mando su aviso, asi que
        # su entrada en la cola la pone el HMI por ese camino. Encolar aca
        # tambien generaba una entrada duplicada y la cola quedaba corrida.
        # Cuando este pallet salga por el sensor con la cola vacia, el
        # descuento no encuentra nada y no pasa nada: la verificacion lee la
        # cabeza EN VIVO, asi que toma la entrada real del pallet siguiente
        # cuando llegue.
        self._pedido_primera_pieza = True
        self.modo_primera_pieza = True
        self._log("[PRIMERA PIEZA] Pedido registrado, ejecutando...")
        return True

    def dejar_pasar_frenado(self):
        """El operador acepto el cartel de 'no coincide' y quiere que el pallet
        frenado SALGA para poder sacar la pieza a mano.
        Escribe op=3 (dejar pasar), baja #553 y sube R55.3: el torno no la
        mecaniza y el pallet sigue de largo. La entrada de la cola se descuenta
        sola cuando cruce el sensor de salida.
        Devuelve False si no habia nada frenado."""
        if not (self.verificacion_fallida or
                getattr(self, "_verif_bloqueada", False)):
            return False
        self._pedido_dejar_pasar = True
        return True

    def _ejecutar_dejar_pasar(self):
        """Corre en el thread sincronizador (todas las FOCAS salen de aca)."""
        if not self.conectado:
            self._log("!!! [DEJAR PASAR] Sin conexion con el torno")
            return
        self._log("[DEJAR PASAR] Aceptado por el operador: escribo op=3 y "
                  "libero para poder sacar la pieza. El torno NO la mecaniza.")
        # Forzar la escritura aunque la receta anterior ya fuera op=3
        self._ultimo_escrito = (None, None, None)
        if not self._escribir_receta((0, 3, 0)):
            self._log("!!! [DEJAR PASAR] No pude escribir op=3. NO libero.")
            return
        # Este pallet va al torno con op=3 y NO va a levantar #553 (el NC se
        # clava en el M80). Avisar para que el ciclo siguiente no la espere.
        self._op3_en_el_torno = True
        self._log("[DEJAR PASAR] Es op=3: el proximo ciclo NO va a esperar "
                  "#553")
        try:
            with self._client_lock:
                self._client.set_macro(self.config["macro_fin"], 0)
            self._log("[DEJAR PASAR] #553=0")
        except Exception as e:
            self._log(f"!!! [DEJAR PASAR] No pude bajar #553: {e}. NO libero.")
            return
        if not self._avisar_receta_lista():
            self._log("!!! [DEJAR PASAR] No pude subir R55.3.")
            return
        # limpiar el estado de la verificacion para que el ciclo siga solo
        self._receta_pendiente = False
        self._receta_esperada = None
        self._verif_bloqueada = False
        self.verificacion_fallida = False
        self._verif_fallida_msg = None
        self._ctx_verif_log = None
        self._t_condiciones_ok = None
        self._avisado_asentando = False
        self._avisado_sin_entrada = False
        self._avisado_rl_alta = False
        self.ultimo_error = None
        self._log("[DEJAR PASAR] Listo. El pallet sale sin mecanizar; su "
                  "entrada se descuenta al cruzar el sensor de salida.")

    def cancelar_primera_pieza(self):
        """Sale del modo PRIMERA PIEZA sin esperar el ciclo normal.
        Existe para que el operador nunca quede atrapado con el checkbox
        tildado si por cualquier motivo no llega a liberarse un siguiente
        pallet (verificacion bloqueada, pallet que no llega, etc.)."""
        if not self.modo_primera_pieza:
            return False
        self.modo_primera_pieza = False
        self._log("[PRIMERA PIEZA] Modo cancelado a mano desde la pantalla")
        return True

    def _ejecutar_primera_pieza(self):
        """Hace el trabajo real de PRIMERA PIEZA. Corre SIEMPRE en el thread
        sincronizador (llamado desde _tick_sincronizar)."""
        mac_tipo = self.config["macro_tipo"]
        mac_op = self.config["macro_op"]
        mac_rosca = self.config["macro_rosca"]
        try:
            with self._client_lock:
                # 1) op=3 explicito (no confiar en el estado previo de las
                #    macros: si el arranque ya estaba confirmado podrian
                #    tener una receta real y mecanizaria de verdad).
                self._client.set_macro(mac_tipo, 0)
                self._client.set_macro(mac_op, self._op_a_macro(3))
                self._client.set_macro(mac_rosca, 0)
            self._ultimo_escrito = (0, 3, 0)
            self._log("[PRIMERA PIEZA] Macros -> op=3 (dejar pasar sin mecanizar)")
            # Este pallet se va al torno con op=3 y NO va a levantar #553 (el
            # NC se clava en el M80 y no entra a la rutina de op=3). Avisar
            # para que el ciclo SIGUIENTE no espere esa señal.
            self._op3_en_el_torno = True
            self._log("[PRIMERA PIEZA] Es op=3: el proximo ciclo NO va a "
                      "esperar #553")

            # 2) bajar #553 (el NC ya puede leer las macros)
            with self._client_lock:
                self._client.set_macro(self.config["macro_fin"], 0)
            self._log("[PRIMERA PIEZA] #553=0")

            # 3) subir R55.3 (el ladder libera el pre-stopper)
            self._avisar_receta_lista()

            # Esta liberacion manual REEMPLAZA cualquier liberacion que
            # hubiera quedado pendiente (apuntaba a un estado anterior y a
            # una cola que acabamos de borrar). Si no se limpia, cuando este
            # pallet salga por el sensor salta la anomalia de "salio con la
            # liberacion todavia pendiente" y la cola queda corrida.
            self._receta_pendiente = False
            self.verificacion_fallida = False
            self._verif_fallida_msg = None
            self._ctx_verif_log = None
            self._ultimo_desencolado = None
            # CRITICO: sin esto, si la verificacion habia quedado bloqueada,
            # _procesar_liberacion sale en su primera linea y NUNCA llega a
            # _liberar_ahora. Resultado: la linea no vuelve a liberar y el
            # checkbox de PRIMERA PIEZA queda tildado y deshabilitado para
            # siempre, sin forma de destrabarlo desde la pantalla.
            self._verif_bloqueada = False
            self._avisado_rl_alta = False
            self._avisado_esperando_pallet = False
            self._avisado_asentando = False
            self.ultimo_error = None

            self._log("[PRIMERA PIEZA] Liberada. El ciclo se normaliza al "
                      "pasar por el sensor de salida.")
            return True
        except Exception as e:
            self._log(f"[PRIMERA PIEZA] Error: {type(e).__name__}: {e}")
            return False

    def todos_op3(self):
        """Marca todas las entradas de la cola como op=3 (dejar pasar).
        No borra: los pallets recirculan sin mecanizar y la cola drena
        con el sensor."""
        n = self.cola.todos_op3()
        self._log(f"[TORNO] {n} items pasados a op=3 (dejar pasar / recircular)")
        return n

    def quitar_indice(self, indice):
        ok = self.cola.quitar_indice(indice)
        if ok:
            self._log(f"[TORNO] Item indice {indice} quitado")
        return ok

    def reconectar(self):
        """Forzar reconexion. El thread se da cuenta solo, pero esto lo apura.

        Tambien resetea el anti-spam del log de errores asi el proximo
        intento de conectar SIEMPRE logea el error completo (util cuando
        algo falla y solo se ve un mensaje viejo).
        """
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.disconnect()
                except Exception:
                    pass
                self._client = None
        self.conectado = False
        # Rehabilitar los reintentos: si nos habiamos rendido por timeout,
        # este es el boton que vuelve a arrancar la cuenta desde cero.
        self._reintento_agotado = False
        self._reintento_desde = None
        # Borrar el cache del ultimo error logueado para que el proximo
        # intento de _intentar_conectar lo loguee de nuevo (aunque sea
        # el mismo error).
        if "ultimo_log_error_conn" in self.__dict__:
            del self.__dict__["ultimo_log_error_conn"]
        if self.ultimo_error:
            self._log(f"[TORNO] Ultimo error registrado: {self.ultimo_error}")

    # ------------------------------------------------------------------
    #  Inspeccion (para la HMI)
    # ------------------------------------------------------------------

    def _run_in_focas_thread(self, fn, timeout=10.0):
        """Ejecuta fn() en el thread sincronizador y devuelve su resultado.

        Indispensable: la fwlib32 de FANUC requiere que TODAS las llamadas
        a un handle FOCAS se hagan desde el mismo thread que lo creo. Si
        un thread distinto (ej. el thread principal de Tkinter) llama
        cnc_statinfo2/cnc_rdmacro/etc directamente, devuelve EW_PROTOCOL.

        Esta funcion encola una task en _task_queue. El thread sync, en
        cada vuelta de _loop_sync, drena la cola y resuelve los Futures.
        """
        if not self.conectado or self._client is None:
            raise RuntimeError("Torno no conectado")
        # Si el caller esta corriendo *dentro* del thread sync, hay que
        # ejecutar directo: encolar generaria deadlock (esperar al mismo
        # thread que esta esperando).
        if threading.current_thread() is self._sync_thread:
            return fn()
        fut = Future()
        self._task_queue.put((fut, fn))
        return fut.result(timeout=timeout)

    def _drenar_queue(self):
        """Procesa todos los pedidos pendientes en la cola. Se llama desde
        _loop_sync, asi cualquier fn() corre en el thread correcto."""
        while True:
            try:
                fut, fn = self._task_queue.get_nowait()
            except queue.Empty:
                return
            if fut.cancelled():
                continue
            try:
                fut.set_result(fn())
            except Exception as e:
                fut.set_exception(e)

    def _cancelar_queue(self, mensaje="Torno desconectado"):
        """Si la conexion se cae, los Futures pendientes deben fallar
        en vez de quedar colgados esperando timeout."""
        while True:
            try:
                fut, _fn = self._task_queue.get_nowait()
            except queue.Empty:
                return
            if not fut.done():
                fut.set_exception(RuntimeError(mensaje))

    def read_macro(self, num):
        """Lee una macro arbitraria. Devuelve float o lanza."""
        return self._run_in_focas_thread(
            lambda: self._client.get_macro(int(num)))

    def write_macro(self, num, valor, decimals=0):
        """Escribe una macro arbitraria."""
        self._run_in_focas_thread(
            lambda: self._client.set_macro(int(num), float(valor), int(decimals)))

    # ---- PMC (senales del ladder: X, Y, R, D, ...) ----
    # Codigos de area: G=0, F=1, Y=2, X=3, A=4, R=5, T=6, K=7, C=8, D=9
    AREAS_PMC = {"G": 0, "F": 1, "Y": 2, "X": 3, "A": 4,
                 "R": 5, "T": 6, "K": 7, "C": 8, "D": 9}

    def read_pmc_byte(self, area, byte_num):
        """Lee un byte del PMC. area puede ser letra ('R') o codigo (5)."""
        code = self.AREAS_PMC.get(str(area).upper(), area) \
            if not isinstance(area, int) else area
        return self._run_in_focas_thread(
            lambda: self._client.read_pmc_byte(int(code), int(byte_num)))

    def read_pmc_bit(self, area, byte_num, bit):
        """Lee un bit puntual del PMC (ej: R54.0 -> 'R', 54, 0)."""
        code = self.AREAS_PMC.get(str(area).upper(), area) \
            if not isinstance(area, int) else area
        return self._run_in_focas_thread(
            lambda: self._client.read_pmc_bit(int(code), int(byte_num), int(bit)))

    def write_pmc_byte(self, area, byte_num, valor):
        """Escribe un byte COMPLETO del PMC. Cuidado: pisa los 8 bits."""
        code = self.AREAS_PMC.get(str(area).upper(), area) \
            if not isinstance(area, int) else area
        self._run_in_focas_thread(
            lambda: self._client.write_pmc_byte(int(code), int(byte_num),
                                                int(valor) & 0xFF))

    def write_pmc_bit(self, area, byte_num, bit, valor):
        """Escribe UN bit del PMC respetando los otros 7 del byte."""
        code = self.AREAS_PMC.get(str(area).upper(), area) \
            if not isinstance(area, int) else area
        b_num = int(byte_num)
        b = int(bit)
        estado = bool(valor)

        def _trabajo():
            actual = self._client.read_pmc_byte(int(code), b_num)
            mask = 1 << b
            nuevo = (actual | mask) if estado else (actual & ~mask)
            self._client.write_pmc_byte(int(code), b_num, nuevo & 0xFF)
            return nuevo

        return self._run_in_focas_thread(_trabajo)

    def status(self):
        """Devuelve el TornoStatus del cliente Fanuc, o None si no conectado."""
        if not self.conectado or self._client is None:
            return None
        try:
            return self._run_in_focas_thread(lambda: self._client.status())
        except Exception as e:
            self._log(f"!!! Error leyendo status: {e}")
            return None

    def position(self):
        """Dict con posicion de ejes (X, Z, ...)"""
        if not self.conectado or self._client is None:
            return None
        try:
            return self._run_in_focas_thread(lambda: self._client.position())
        except Exception as e:
            self._log(f"!!! Error leyendo posicion: {e}")
            return None

    def part_count(self):
        if not self.conectado or self._client is None:
            return None
        try:
            return self._run_in_focas_thread(lambda: self._client.part_count())
        except Exception:
            return None

    # ------------------------------------------------------------------
    #  Thread sincronizador
    # ------------------------------------------------------------------

    def _loop_sync(self):
        """Loop principal del thread:

        - Si no esta conectado, intenta conectar.
        - Si esta conectado:
          - Lee #553. Si == 1 -> popleft, baja a 0.
          - Compara cola[0] con _ultimo_escrito. Si difiere, escribe #551/#550/#552.
          - Si cola vacia y _ultimo_escrito != (0,0,0), escribe ceros.
        - Sleep polling_ms.
        """
        polling_s = self.config["polling_ms"] / 1000.0

        while not self._shutdown:
            # Conectar si hace falta
            if not self.conectado:
                # Antes de intentar conectar, fallar pedidos colgados:
                # vienen de la HMI esperando una respuesta que no llegara.
                self._cancelar_queue("Torno desconectado")

                # Si ya nos rendimos, no intentar mas hasta que la HMI
                # llame a reconectar().
                if getattr(self, "_reintento_agotado", False):
                    time.sleep(1.0)
                    continue

                # Marcar cuando arranco esta racha de intentos fallidos
                if getattr(self, "_reintento_desde", None) is None:
                    self._reintento_desde = time.time()

                self._intentar_conectar()

                if not self.conectado:
                    max_s = self.config.get("reintento_conexion_max_s", 300)
                    transcurrido = time.time() - self._reintento_desde
                    if max_s and transcurrido >= max_s:
                        self._reintento_agotado = True
                        self._log(f"[TORNO] Sin conexion despues de "
                                  f"{int(transcurrido)}s ({int(max_s/60)} min): "
                                  f"dejo de intentar. Apreta 'Reconectar' "
                                  f"cuando el torno este disponible.")
                        self.ultimo_error = (f"Sin conexion tras {int(max_s/60)} "
                                             f"min de intentos")
                        continue
                    time.sleep(self.config.get("reintento_conexion_espera_s", 2.0))
                    continue
                # Conecto: limpiar el control de reintentos
                self._reintento_desde = None
                self._reintento_agotado = False

            # Procesar pedidos de la HMI (read_macro, status, etc).
            # Se hace en el mismo thread que creo el handle.
            try:
                self._drenar_queue()
            except Exception as e:
                self._log(f"!!! Error procesando pedidos: {e}")

            # Conectado: hacer la sincronizacion
            try:
                self._tick_sincronizar()
            except ConnectionError as e:
                # Torno apagado / sin red: no es un bug, es reconexion normal.
                self._log(f"[TORNO] {e}")
                self.ultimo_error = "Sin comunicacion con el torno"
                self._desconectar_silencioso()
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
            # Registrar el directorio de DLLs en el search path de Windows
            # para que las dependencias internas de fwlib*.dll se resuelvan.
            # Sin esto, la primera DLL carga pero al pedir las dependientes
            # Windows las busca en cwd/System32/PATH y falla -> EW_SOCKET.
            dll_path = self.config["torno_dll_path"]
            if hasattr(os, "add_dll_directory") and dll_path:
                dll_abs = os.path.abspath(dll_path)
                if os.path.isdir(dll_abs):
                    # add_dll_directory se puede llamar varias veces sin
                    # problema; devuelve un handle pero no nos importa.
                    try:
                        os.add_dll_directory(dll_abs)
                    except (OSError, FileNotFoundError):
                        pass

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
            self._ultimo_escrito = (None, None, None)  # forzar reescritura
            self._fallos_focas = 0
            self._ultimo_fallo_log = None
            self._sensor_prev = None
            self._sensor_visto_alto = False
            # Dejar #553=0 (por si quedo en 1 de la sesion anterior) para
            # que el torno pueda arrancar cuando se confirme.
            try:
                with self._client_lock:
                    self._client.set_macro(self.config["macro_fin"], 0)
            except Exception:
                pass
            self._log(f"[TORNO] Conectado a {self.config['torno_host']}:{self.config['torno_port']}")
            self._log("[TORNO] Arranque: receta segura op=3 (dejar pasar) hasta la primera liberacion verificada")
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
        # Limpiar estado del sensor: al reconectar hay que re-sincronizar el
        # nivel, si no un 1->0 espurio descontaria un pallet que no salio.
        self._sensor_prev = None
        self._sensor_visto_alto = False
        self._receta_pendiente = False
        self._fallos_focas = 0
        self._ultimo_fallo_log = None
        self._cancelar_queue("Torno desconectado durante operacion")

    def _registrar_fallo_focas(self, donde, e):
        """Cuenta fallos FOCAS consecutivos. Si se pasa del umbral, lanza para
        que _loop_sync marque desconectado y reintente conectar solo (caso
        tipico: apagaron el torno). Loguea una sola vez por tipo de error
        para no spamear el log cada 200ms."""
        self._fallos_focas += 1
        msg = f"{donde}: {e}"
        if msg != self._ultimo_fallo_log:
            self._log(f"[TORNO] {msg}")
            self._ultimo_fallo_log = msg
        if self._fallos_focas >= self.config.get("max_fallos_focas", 5):
            raise ConnectionError(
                f"{self._fallos_focas} fallos FOCAS seguidos ({donde}). "
                f"Torno apagado o sin red: voy a reconectar.")

    def _ok_focas(self):
        """Una llamada FOCAS anduvo: resetear el contador de fallos."""
        if self._fallos_focas:
            self._log(f"[TORNO] Comunicacion recuperada")
        self._fallos_focas = 0
        self._ultimo_fallo_log = None

    def _procesar_sensor_salida(self):
        """Sensor de salida de pallet (R54.0, pulso estirado 2s por ladder).

        En el flanco de bajada (1->0) el pallet termino de pasar: DESCUENTA
        uno de la cola (verdad fisica: salio un pallet) y marca que hay que
        liberar el siguiente.

        La liberacion NO se hace aca: la maneja _procesar_liberacion(), que
        antes verifica #553, el pallet en posicion y que la pieza fisica
        coincida con la cola.

        Exige el ciclo completo 0->1->0 para no contar de mas si el sensor
        tiembla con el pallet encima."""
        tipo = self.config["sensor_salida_pmc_tipo"]
        byte = self.config["sensor_salida_byte"]
        bit = self.config["sensor_salida_bit"]
        try:
            with self._client_lock:
                estado = self._client.read_pmc_bit(tipo, byte, bit)
            self._ok_focas()
            # Publicar para la HMI (pestaña I/O)
            self.ultimo_sensor_salida = estado
        except Exception as e:
            self._registrar_fallo_focas(f"Error leyendo sensor R{byte}.{bit}", e)
            return

        prev = getattr(self, "_sensor_prev", None)
        if prev is None:
            self._sensor_prev = estado
            self._sensor_visto_alto = estado
            return

        if estado and not prev:
            # Flanco de subida: el pallet llego al sensor
            self._sensor_visto_alto = True

        if (not estado) and prev and getattr(self, "_sensor_visto_alto", False):
            # Flanco de bajada: el pallet SALIO.
            # 1) DESCONTAR siempre (verdad fisica: salio un pallet).
            # ANOMALIA: si todavia habia una liberacion pendiente, significa
            # que salio un pallet SIN que hayamos liberado el del pre-stopper.
            # Eso no deberia poder pasar (el pre-stopper lo retiene) y deja la
            # cola corrida en uno.
            if getattr(self, "_receta_pendiente", False):
                self._log("!!! [SENSOR] Salio un pallet con la liberacion "
                          "TODAVIA PENDIENTE: el pre-stopper solto sin "
                          "nuestra señal (revisar si R55.3 quedo en 1). "
                          "Descarto la liberacion vieja y reevaluo desde el "
                          "estado actual para no arrastrar el desfase.")
                # La liberacion pendiente apuntaba a un pallet que ya se fue:
                # queda obsoleta. Limpiar el estado para que la ventana nueva
                # (la del descuento de abajo) se evalue limpia, en vez de
                # seguir comparando contra una foto vieja.
                self._receta_pendiente = False
                self.verificacion_fallida = False
                self._verif_fallida_msg = None
                self._ctx_verif_log = None
                self._receta_esperada = None

            quitado = self.cola.desencolar()
            if quitado is not None:
                self.piezas_terminadas += 1
                self._ultimo_desencolado = dict(quitado)
                self._log(f"[SENSOR] Pallet salio -> desencolado: "
                          f"tipo={quitado['tipo']} op={quitado['op']} "
                          f"rosca={quitado.get('rosca', 0)} "
                          f"| total salidos={self.piezas_terminadas}")
            else:
                self._ultimo_desencolado = None
                self._log("[SENSOR] Pallet salio pero la cola estaba vacia "
                          "(paso uno sin encolar)")

            # CAPTURA EN EL INSTANTE DEL DESCUENTO.
            # Posiciones fisicas en este momento: el mecanizado ya salio y el
            # STOPPER quedo VACIO (el pre-stopper todavia retiene al
            # siguiente, lo suelta recien cuando subimos R55.3).
            #   PRE-STOPPER = cola #1 -> es el UNICO pallet en juego: es el
            #                            que leen los sensores fisicos
            #                            (op_20 / presencia), el que se va a
            #                            liberar, y el que se va a mecanizar.
            #                            Receta y verificacion apuntan AL
            #                            MISMO item.
            snap = self.cola.snapshot()

            def _receta(item):
                if item is None:
                    return (0, 3, 0)
                return (int(item["tipo"]), int(item["op"]),
                        int(item.get("rosca", 0)))

            self._receta_esperada = _receta(snap[0] if len(snap) >= 1 else None)
            self._log(f"[VERIF] Tras el descuento: receta de cola #"
                      f"{int(self.config.get('indice_receta', 0)) + 1} = "
                      f"op={self._receta_esperada[1]} | verificar contra cola #"
                      f"{int(self.config.get('indice_verif', 0)) + 1} | "
                      f"{len(snap)} en cola")

            self._sensor_visto_alto = False

            # Queda pendiente liberar el siguiente. NO se libera aca:
            # primero hay que esperar que el torno confirme fin (#553==1),
            # que el pallet siguiente este EN POSICION en el pre-stopper
            # (X10.1) y que la pieza fisica COINCIDA con la cola.
            # Lo maneja _procesar_liberacion().
            # Ventana NUEVA: la captura recien se tomo, se puede comparar.
            self._verif_bloqueada = False
            self._receta_pendiente = True
            # Momento del descuento: desde aca se mide el asentamiento del
            # pallet en el pre-stopper antes de leer DI0/DI1.
            self._t_descuento = time.monotonic()
            self._t_condiciones_ok = None
            self._avisado_asentando = False

        self._sensor_prev = estado

        # Intentar liberar el siguiente (si hay algo pendiente).
        if getattr(self, "_receta_pendiente", False):
            self._procesar_liberacion()

    def reintentar_verificacion(self):
        """Desbloquea la verificacion y reevalua desde el estado ACTUAL de la
        cola. La receta y la verificacion las recalcula _procesar_liberacion()
        con los indices de config, asi que aca solo se limpia el bloqueo.
        Usar despues de corregir un 'NO COINCIDE'."""
        snap = self.cola.snapshot()
        i_rec = int(self.config.get("indice_receta", 0))
        if len(snap) > i_rec:
            it = snap[i_rec]
            self._receta_esperada = (int(it["tipo"]), int(it["op"]),
                                     int(it.get("rosca", 0)))
        else:
            self._receta_esperada = (0, 3, 0)
        self._verif_bloqueada = False
        self.verificacion_fallida = False
        self._verif_fallida_msg = None
        self._ctx_verif_log = None
        self._receta_pendiente = True
        self._t_descuento = time.monotonic()
        self._avisado_asentando = False
        self._log(f"[VERIF] Reintento manual: receta de cola #{i_rec + 1} = "
                  f"op={self._receta_esperada[1]} | verificar contra cola #"
                  f"{int(self.config.get('indice_verif', 0)) + 1} | "
                  f"{len(snap)} en cola")
        return True

    def _leer_pallet_en_posicion(self):
        """Sensor del PMC que dice si el pallet esta clampeado en el
        pre-stopper (X10.1). Sin esto los sensores del pre-stopper no son
        validos todavia. Devuelve True/False, o None si fallo la lectura."""
        try:
            with self._client_lock:
                v = self._client.read_pmc_bit(
                    self.config["pallet_en_pos_pmc_tipo"],
                    self.config["pallet_en_pos_byte"],
                    self.config["pallet_en_pos_bit"])
            # Publicar para la HMI (que no debe llamar a FOCAS desde la UI)
            self.ultimo_pallet_en_pos = v
            return v
        except Exception as e:
            self._log(f"[VERIF] Error leyendo pallet en posicion "
                      f"(X{self.config['pallet_en_pos_byte']}."
                      f"{self.config['pallet_en_pos_bit']}): {e}")
            return None

    def _procesar_liberacion(self):
        """Libera el pre-stopper para el pallet SIGUIENTE, pero solo despues
        de verificar que todo cierra. Corre en el thread sincronizador.

        PRECONDICION (no se chequea aca, la garantiza quien llama):
          0) Ya salio un pallet por el sensor de salida y se descontó de la
             cola. Esta funcion SOLO se llama si _receta_pendiente == True,
             y esa bandera se pone en True unicamente en el flanco de bajada
             del sensor de salida (ver _procesar_sensor_salida). Sin flanco no
             se lee ningun sensor del pre-stopper ni se escribe nada al torno.
             Es lo que garantiza que cola[0] sea el pallet del PRE-STOPPER:
             el que se mecanizo ya se fue de la cola y el stopper quedo vacio,
             asi que el unico pallet clampeado y medible es el del
             pre-stopper. La unica excepcion es PRIMERA PIEZA (manual).

        Condiciones que si se chequean aca (si alguna no se cumple, no libera
        y vuelve a intentar en el proximo tick):
          1) #553 == 1        -> el torno confirmo que termino el anterior.
                                 En regimen ya esta en 1 (es lo que hizo abrir
                                 el stopper), asi que no hace esperar: es una
                                 guarda para el pallet que cruza el sensor sin
                                 que el torno haya terminado.
          2) X10.1 == 1       -> el pallet siguiente esta EN POSICION en el
                                 pre-stopper (recien ahi los sensores valen).
                                 Aca SI se espera: puede tardar en llegar, o
                                 no haber ninguno si la cinta quedo con un
                                 hueco. Reintenta cada tick, sin timeout.
          3) op fisica == op de la cola  -> la pieza real coincide con lo que
                                 leyo la camara. Es la confirmacion cruzada.
                                 Se lee cola[0] EN VIVO, no la foto tomada en
                                 el descuento, y de esa misma lectura sale la
                                 receta que se escribe.

        Con las tres OK: escribe la receta, baja #553 y sube R55.3.
        Si la 3 falla -> ALARMA y NO libera (el pallet queda frenado).
        """
        if getattr(self, "_verif_bloqueada", False):
            return   # bloqueada por un NO COINCIDE: espera accion del operador

        # --- ROBOT CAIDO: no liberar ---
        # Doble red. El corte principal es el heartbeat (el Fammar frena la
        # cinta), pero mientras el ladder reacciona no tiene sentido seguir
        # metiendo pallets al torno: el robot no los proceso y su entrada en
        # la cola puede no existir.
        if (self.config.get("watchdog_bloquea_liberacion", True)
                and self.robot_vivo is False):
            if not getattr(self, "_avisado_lib_robot_caido", False):
                self._log("!!! [VERIF] Robot caido: NO libero el "
                          "pre-stopper. El pallet queda frenado hasta que "
                          "vuelva la linea de vida.")
                self._avisado_lib_robot_caido = True
            self.ultimo_error = "Robot caido: liberacion bloqueada"
            return
        self._avisado_lib_robot_caido = False

        # --- 0) LEER LA COLA PRIMERO ---
        # Hay que saber si el pallet es op=3 (dejar pasar) ANTES de mirar
        # #553, porque con op=3 no se espera esa señal (ver punto 1).
        #   indice_receta -> de ahi sale la RECETA que se escribe
        #   indice_verif  -> contra eso se VERIFICA con DI0/DI1
        # Lectura EN VIVO, no la foto del descuento: entre el descuento y
        # este momento puede pasar tiempo (esperando X10.1) y el robot puede
        # haber encolado o el operador editado la cola.
        i_rec = int(self.config.get("indice_receta", 0))
        i_ver = int(self.config.get("indice_verif", 0))
        snap = self.cola.snapshot()

        if len(snap) > i_rec:
            it = snap[i_rec]
            receta = (int(it["tipo"]), int(it["op"]),
                      int(it.get("rosca", 0)))
        else:
            receta = (0, 3, 0)   # sin entrada -> dejar pasar
        op_cola = int(snap[i_ver]["op"]) if len(snap) > i_ver else None

        # DEJAR PASAR: ni el que se va a mecanizar ni el que se mide llevan
        # operacion real.
        es_dejar_pasar = (receta[1] == 3) or (op_cola == 3)

        foto = getattr(self, "_receta_esperada", None)
        if foto is not None and foto != receta:
            self._log(f"[VERIF] La cola cambio desde el descuento "
                      f"(era op={foto[1]}, ahora op={receta[1]}). "
                      f"Uso el valor en vivo.")
        self._receta_esperada = receta

        # --- 1) el torno tiene que haber terminado (#553 == 1) ---
        # EXCEPCION: si en el ciclo ANTERIOR liberamos un pallet op=3, ese
        # pallet esta AHORA en el torno y NO va a levantar #553.
        # Motivo (medido en la maquina): con el pallet sin pieza el programa
        # del NC se queda clavado en un M80 y nunca entra a la rutina de
        # op=3, asi que nunca setea la macro. Si esperaramos esa señal la
        # linea se para para siempre.
        # El flag lo levanta _liberar_ahora() al escribir una receta op=3, y
        # se CONSUME aca una sola vez: el ciclo siguiente vuelve a exigir
        # #553 normalmente.
        if getattr(self, "_op3_en_el_torno", False):
            # NO consumir el flag aca. Esta funcion se reintenta cada tick y
            # puede volver a salir mas abajo esperando X10.1: si lo apagamos
            # en el primer tick, los siguientes vuelven a exigir #553 y la
            # linea queda esperando para siempre una señal que ese pallet no
            # va a levantar. Lo apaga _liberar_ahora() cuando se libera de
            # verdad con una receta real.
            if not getattr(self, "_avisado_op3_sin_553", False):
                self._log("[VERIF] El pallet que esta en el torno salio como "
                          "op=3: NO espero #553 (el NC se clava en el M80 y "
                          "no levanta la señal).")
                self._avisado_op3_sin_553 = True
        else:
            self._avisado_op3_sin_553 = False
            try:
                with self._client_lock:
                    fin = self._client.get_macro(self.config["macro_fin"])
            except Exception as e:
                self._log(f"[VERIF] No se pudo leer #553: {e}")
                return
            if fin != 1:
                return   # todavia mecanizando: esperar

        # Si la verificacion esta apagada, liberar como antes.
        # --- 1b) estado de R55.3 (SIEMPRE se lee y se loguea) ---
        # Si quedo en 1, el ladder no bajo la del ciclo anterior: el
        # pre-stopper ya esta liberado y los pallets pasan sin nuestra orden.
        # Se avisa siempre, incluso con la verificacion apagada, porque es un
        # sintoma del handshake y no de los sensores del pre-stopper.
        rl = self._leer_receta_lista()
        if rl is True:
            if not getattr(self, "_avisado_rl_alta", False):
                byte = self.config["receta_lista_byte"]
                bit = self.config["receta_lista_bit"]
                self._log(f"!!! [HANDSHAKE] R{byte}.{bit} SIGUE EN 1: el "
                          f"ladder no la bajo. El pre-stopper esta liberado "
                          f"y los pallets pasan sin nuestra orden.")
                self._avisado_rl_alta = True
            if self.config.get("exigir_receta_lista_en_cero", False):
                self.ultimo_error = (f"R{self.config['receta_lista_byte']}."
                                     f"{self.config['receta_lista_bit']} "
                                     f"quedo en 1 (ladder no la bajo)")
                return
        else:
            self._avisado_rl_alta = False

        if not self.config.get("verificar_prestopper", True):
            self._liberar_ahora()
            return

        # --- 2) el pallet tiene que estar en posicion ---
        en_pos = self._leer_pallet_en_posicion()
        if en_pos is None:
            return          # error de lectura: reintentar
        if not en_pos:
            # Todavia no llego / no clampeo. Log una sola vez para no spamear.
            if not getattr(self, "_avisado_esperando_pallet", False):
                self._log("[VERIF] Esperando que el pallet clampee en el "
                          "pre-stopper para poder medir...")
                self._avisado_esperando_pallet = True
            return
        self._avisado_esperando_pallet = False

        # --- 2b) ESPERAR desde que el pallet paso por el SENSOR DE SALIDA ---
        # El reloj arranca en el DESCUENTO (flanco del sensor de salida), que
        # es el instante en que el pallet mecanizado se fue y el stopper queda
        # vacio. Durante esta espera NO se levanta R55.3, asi que el
        # pre-stopper sigue frenado y su pallet no se mueve.
        espera_s = self.config.get("espera_asentamiento_ms", 0) / 1000.0
        if espera_s > 0:
            t0 = getattr(self, "_t_descuento", None)
            if t0 is not None:
                transcurrido = time.monotonic() - t0
                if transcurrido < espera_s:
                    if not getattr(self, "_avisado_asentando", False):
                        self._log(f"[VERIF] Paso por el sensor de salida. "
                                  f"Espero {espera_s*1000:.0f}ms con el "
                                  f"pre-stopper frenado antes de medir...")
                        self._avisado_asentando = True
                    return
        self._avisado_asentando = False

        # --- 3) comparar la pieza fisica con la cola ---
        # snap / receta / op_cola ya se leyeron en el punto 0.
        try:
            from dio import dio, nombre_op, OP_NINGUNA, OP_10, OP_20
            op_fisica = dio.op_fisica()
        except Exception as e:
            # El modulo DIO no esta (driver, permisos, WinRing0 mal ubicado).
            if self.config.get("verificacion_obligatoria", True):
                if not getattr(self, "_avisado_dio_falla", False):
                    self._log(f"!!! [VERIF] DIO no disponible ({e}). NO libero "
                              f"(verificacion_obligatoria=True). Revisar "
                              f"permisos de administrador y WinRing0x64.sys.")
                    self._avisado_dio_falla = True
                self.ultimo_error = "DIO no disponible: verificacion bloqueada"
                return
            self._log(f"!!! [VERIF] DIO no disponible ({e}). Libero SIN "
                      f"verificar (verificacion_obligatoria=False)")
            self._liberar_ahora()
            return
        self._avisado_dio_falla = False

        if self.config.get("invertir_op_sensor", False):
            if op_fisica == OP_10:
                op_fisica = OP_20
            elif op_fisica == OP_20:
                op_fisica = OP_10

        # --- op=3 (DEJAR PASAR): no se mecaniza, no hay nada que comparar ---
        # Se mira la entrada de VERIFICACION y la de RECETA, porque pueden ser
        # distintas. Un op=3 sale de: pallet ABAJO, hueco/recuperacion, o cola
        # sin entrada. En ninguno de los tres casos el torno mecaniza, asi que
        # comparar op_fisica contra 3 daria "no coincide" siempre (3 nunca es
        # 0, 1 ni 2). Se libera, pero se DEJA CONSTANCIA de lo que vio el
        # sensor para poder auditar despues.
        if op_cola == 3 or receta[1] == 3:
            cual = []
            if op_cola == 3:
                cual.append(f"cola#{i_ver+1} (verificacion)")
            if receta[1] == 3:
                cual.append(f"cola#{i_rec+1} (receta)")
            self._log(f"[VERIF] op=3 DEJAR PASAR en {' y '.join(cual)}: "
                      f"el torno no la mecaniza, no comparo. "
                      f"El sensor veia: {nombre_op(op_fisica)}. Libero.")
            if (op_fisica in (OP_10, OP_20) and
                    self.config.get("alertar_pieza_en_op3", True)):
                self._log(f"~~~ [VERIF] OJO: el pallet que sale como op=3 "
                          f"TIENE una pieza en {nombre_op(op_fisica)}. Si no "
                          f"era un pallet ABAJO, esa pieza sale sin mecanizar.")
            self._liberar_ahora()
            return

        ctx = (f"[VERIF] comparando pre-stopper: cola#{i_ver+1}="
               f"{'op' + str(op_cola) if op_cola is not None else 'SIN ENTRADA'}"
               f" vs sensor={nombre_op(op_fisica)} | receta (cola#{i_rec+1})="
               f"op{receta[1]} | {len(snap)} en cola")
        if getattr(self, "_ctx_verif_log", None) != ctx:
            self._log(ctx)
            self._ctx_verif_log = ctx

        if op_cola is None:
            # Hay pallet fisico en el pre-stopper (X10.1 en 1) pero la cola
            # todavia no tiene entrada en ese indice. Puede ser transitorio:
            # el aviso del SPS llega por EKI y el encolado puede demorar unos
            # ciclos. ESPERAR en vez de bloquear, y avisar una sola vez.
            if not getattr(self, "_avisado_sin_entrada", False):
                self._log(f"[VERIF] Todavia no hay entrada en cola#{i_ver+1} "
                          f"({len(snap)} en cola): espero a que el robot "
                          f"encole antes de comparar...")
                self._avisado_sin_entrada = True
            self.ultimo_error = (f"esperando entrada en cola#{i_ver+1} "
                                 f"({len(snap)} en cola)")
            return
        self._avisado_sin_entrada = False

        if op_fisica == op_cola:
            self._log(f"[VERIF] OK: pre-stopper coincide (cola#{i_ver+1} op={op_cola} "
                      f"= sensor {nombre_op(op_fisica)}). Escribo receta "
                      f"op={receta[1]} y libero.")
            self._liberar_ahora()
            return

        # --- NO COINCIDE ---
        if op_fisica == OP_NINGUNA:
            detalle = ("los sensores no ven pieza en el pallet")
        else:
            detalle = (f"el sensor dice {nombre_op(op_fisica)} "
                       f"(op={op_fisica})")
        esperado_txt = ("cola vacia" if op_cola == -1
                        else f"cola#{i_ver+1} dice op={op_cola}")

        # MODO OBSERVACION: registrar la discrepancia y liberar igual. Sirve
        # para dejar la linea produciendo mientras se junta evidencia de
        # varios ciclos, en vez de frenar en el primero.
        if self.config.get("verif_solo_log", False):
            snap_txt = ", ".join(f"#{k+1}:op{it['op']}"
                                 for k, it in enumerate(snap[:5]))
            self._log(f"~~~ [VERIF-OBS] DISCREPANCIA (no bloqueo): "
                      f"{esperado_txt} pero {detalle} | receta que mando="
                      f"op{receta[1]} | cola: {snap_txt} | "
                      f"{len(snap)} en cola")
            self._liberar_ahora()
            return

        msg = (f"!!! [VERIF] NO COINCIDE en el pre-stopper: {esperado_txt} "
               f"pero {detalle}. NO libero.")
        if getattr(self, "_verif_fallida_msg", None) != msg:
            self._log(msg)
            self._log("!!! [VERIF] Verificacion BLOQUEADA: no vuelvo a "
                      "comparar con esta foto (seria contra un pallet que "
                      "puede haber cambiado). Corregir y apretar 'Reintentar "
                      "verificacion', o esperar el proximo descuento.")
            self._verif_fallida_msg = msg
        # BLOQUEAR: cerrar la ventana para no seguir comparando la captura
        # vieja contra el pallet que este ahora en el pre-stopper.
        self._verif_bloqueada = True
        self._receta_pendiente = False
        self.ultimo_error = (f"Verificacion fallida: cola op={op_cola} vs "
                             f"sensor {nombre_op(op_fisica)}")
        self.verificacion_fallida = True

    def _liberar_ahora(self):
        """Escribe la receta, baja #553 y sube R55.3 (el ladder suelta el
        pre-stopper). Solo se llama con las verificaciones ya pasadas."""
        # Escribir EXACTAMENTE la receta que se verifico (no releer la cola:
        # cola[0] a esta altura es el SIGUIENTE, escribirlo seria mandar la
        # receta corrida en uno).
        r = getattr(self, "_receta_esperada", None)
        if r is None:
            self._log("!!! [VERIF] _liberar_ahora sin receta capturada: NO "
                      "escribo (evito mandar la receta de otro pallet)")
            self._receta_pendiente = False
            return
        self._escribir_receta(r)
        # Si lo que acabamos de mandar es op=3 (dejar pasar), este pallet se
        # va al torno y NO va a levantar #553: dejar constancia para que el
        # ciclo SIGUIENTE no espere esa señal.
        if r[1] == 3:
            self._op3_en_el_torno = True
            self._log("[VERIF] Liberado un op=3: el proximo ciclo NO va a "
                      "esperar #553")
        else:
            self._op3_en_el_torno = False
            self._avisado_op3_sin_553 = False
        try:
            with self._client_lock:
                self._client.set_macro(self.config["macro_fin"], 0)
        except Exception as e:
            self._log(f"[VERIF] No se pudo bajar #553: {e}")
        self._avisar_receta_lista()
        self._receta_pendiente = False
        self._t_condiciones_ok = None
        self._verif_fallida_msg = None
        self._ctx_verif_log = None
        self._receta_esperada = None
        self.verificacion_fallida = False
        # Se libero el SIGUIENTE por el ciclo normal: sale el modo primera
        # pieza (la HMI destilda y desbloquea el checkbox).
        if self.modo_primera_pieza:
            self.modo_primera_pieza = False
            self._log("[PRIMERA PIEZA] Ciclo normal retomado (se libero el "
                      "siguiente): modo primera pieza desactivado")

    def _op_a_macro(self, op_interno):
        """Traduce el op interno (1=OP10, 2=OP20, 3=dejar pasar) al valor que
        espera el programa NC en #550. Ver config mapa_op_macro."""
        mapa = self.config.get("mapa_op_macro") or {}
        try:
            return int(mapa.get(int(op_interno), int(op_interno)))
        except Exception:
            return int(op_interno)

    def _escribir_receta(self, objetivo):
        """Escribe una receta explicita (tipo, op, rosca) en las macros.
        Se usa para que lo escrito sea EXACTAMENTE lo verificado."""
        mac_tipo = self.config["macro_tipo"]
        mac_op = self.config["macro_op"]
        mac_rosca = self.config["macro_rosca"]
        objetivo = (int(objetivo[0]), int(objetivo[1]), int(objetivo[2]))
        if objetivo != self._ultimo_escrito:
            op_macro = self._op_a_macro(objetivo[1])
            with self._client_lock:
                self._client.set_macro(mac_tipo, objetivo[0])
                self._client.set_macro(mac_op, op_macro)
                self._client.set_macro(mac_rosca, objetivo[2])
            self._ultimo_escrito = objetivo
            extra = ("" if op_macro == objetivo[1]
                     else f"  (op interno {objetivo[1]} -> macro {op_macro})")
            self.ultimo_evento = (f"[TORNO] Receta -> #{mac_tipo}={objetivo[0]} "
                                  f"#{mac_op}={op_macro} "
                                  f"#{mac_rosca}={objetivo[2]}{extra}")
            self._log(self.ultimo_evento)
        return True   # ya estaba escrito o se escribio ahora

    def _escribir_receta_actual(self):
        """Escribe en las macros la receta del primero de la cola (o op=3 si
        la cola esta vacia). Solo escribe si cambio respecto de lo ultimo."""
        mac_tipo = self.config["macro_tipo"]
        mac_op = self.config["macro_op"]
        mac_rosca = self.config["macro_rosca"]
        primero = self.cola.primero()
        if primero is None:
            objetivo = (0, 3, 0)
        else:
            objetivo = (int(primero["tipo"]), int(primero["op"]),
                        int(primero.get("rosca", 0)))
        if objetivo != self._ultimo_escrito:
            op_macro = self._op_a_macro(objetivo[1])
            with self._client_lock:
                self._client.set_macro(mac_tipo, objetivo[0])
                self._client.set_macro(mac_op, op_macro)
                self._client.set_macro(mac_rosca, objetivo[2])
            self._ultimo_escrito = objetivo
            if objetivo == (0, 3, 0):
                self.ultimo_evento = (f"[TORNO] Cola vacia -> op=3 (dejar pasar) "
                                      f"#{mac_tipo}=0 #{mac_op}=3 #{mac_rosca}=0")
            else:
                self.ultimo_evento = (f"[TORNO] Receta -> #{mac_tipo}={objetivo[0]} "
                                      f"#{mac_op}={objetivo[1]} #{mac_rosca}={objetivo[2]}")
            self._log(self.ultimo_evento)

    def _avisar_receta_lista(self):
        """Pone R55.3 = 1 para avisar al ladder del Fammar que la receta ya
        esta cargada (#553 ya bajo). El ladder libera el pre-stopper y baja
        el mismo la R55.3. La PC NO la baja."""
        tipo = self.config["receta_lista_pmc_tipo"]
        byte = self.config["receta_lista_byte"]
        bit = self.config["receta_lista_bit"]
        try:
            with self._client_lock:
                b = self._client.read_pmc_byte(tipo, byte)
                self._client.write_pmc_byte(tipo, byte, b | (1 << bit))
            self._log(f"[TORNO] R{byte}.{bit}=1 (receta cargada -> "
                      f"ladder libera pre-stopper)")
            return True
        except Exception as e:
            self._log(f"[TORNO] No se pudo poner R{byte}.{bit}: {e}")
            return False

    def _refrescar_debug(self):
        """Junta en un dict todo lo que muestra la pestaña DEBUG. Corre en el
        thread sincronizador; la UI solo lee estado_debug()."""
        cfg = self.config
        try:
            with self._client_lock:
                m_fin = self._client.get_macro(cfg["macro_fin"])
                m_op = self._client.get_macro(cfg["macro_op"])
                m_tipo = self._client.get_macro(cfg["macro_tipo"])
                m_rosca = self._client.get_macro(cfg["macro_rosca"])
                m_cola = self._client.get_macro(cfg["macro_cola"])
                b_sal = self._client.read_pmc_byte(
                    cfg["sensor_salida_pmc_tipo"], cfg["sensor_salida_byte"])
                b_rl = self._client.read_pmc_byte(
                    cfg["receta_lista_pmc_tipo"], cfg["receta_lista_byte"])
        except Exception as e:
            self._debug = {"error": f"{type(e).__name__}: {e}"}
            return

        sal = bool((int(b_sal) >> cfg["sensor_salida_bit"]) & 1)
        rl = bool((int(b_rl) >> cfg["receta_lista_bit"]) & 1)
        self.ultimo_receta_lista = rl

        t0 = getattr(self, "_t_descuento", None)
        desde = None if t0 is None else round(time.monotonic() - t0, 1)

        snap = self.cola.snapshot()
        i_rec = int(cfg.get("indice_receta", 0))
        i_ver = int(cfg.get("indice_verif", 0))

        try:
            from dio import dio, nombre_op, OP_10, OP_20
            op20_s, pres_s = dio.leer_sensores()
            op_fis = dio.op_fisica()
            op_fis_inv = op_fis
            if cfg.get("invertir_op_sensor", False):
                if op_fis == OP_10:
                    op_fis_inv = OP_20
                elif op_fis == OP_20:
                    op_fis_inv = OP_10
            dio_txt = (f"op_20={int(op20_s)}  presencia={int(pres_s)}  "
                       f"-> {nombre_op(op_fis)}")
            if op_fis_inv != op_fis:
                dio_txt += f"  (invertido: {nombre_op(op_fis_inv)})"
            op_comparado = op_fis_inv
        except Exception as e:
            dio_txt = f"DIO no disponible ({type(e).__name__})"
            op_comparado = None

        self._debug = {
            "macro_fin": int(m_fin),
            "macro_receta": f"tipo={int(m_tipo)} op={int(m_op)} rosca={int(m_rosca)}",
            "macro_cola": int(m_cola),
            "sensor_salida": sal,
            "sensor_visto_alto": bool(getattr(self, "_sensor_visto_alto", False)),
            "receta_lista": rl,
            "pallet_en_pos": getattr(self, "ultimo_pallet_en_pos", None),
            "dio": dio_txt,
            "op_comparado": op_comparado,
            "receta_pendiente": bool(getattr(self, "_receta_pendiente", False)),
            "verif_bloqueada": bool(getattr(self, "_verif_bloqueada", False)),
            "seg_desde_descuento": desde,
            "cola_len": len(snap),
            "cola_head": [f"#{k+1} op={it['op']}" for k, it in enumerate(snap[:6])],
            "indice_receta": i_rec,
            "indice_verif": i_ver,
            "op_esperado": (int(snap[i_ver]["op"]) if len(snap) > i_ver else None),
            "receta_a_mandar": (f"op={snap[i_rec]['op']}" if len(snap) > i_rec
                                else "op=3 (sin entrada)"),
            "piezas_terminadas": self.piezas_terminadas,
            "robot_vivo": self.robot_vivo,
            "robot_en_auto": self.robot_en_auto,
            "seg_sin_pulso": getattr(self, "segundos_sin_pulso", None),
            "system_link": self.system_link,
            "ultimo_desencolado": getattr(self, "_ultimo_desencolado", None),
            "error": None,
        }

    def estado_debug(self):
        """Copia del snapshot de debug para la UI."""
        return dict(getattr(self, "_debug", {}) or {})

    def _leer_receta_lista(self):
        """Lee R55.3. Devuelve True/False, o None si no se pudo leer.
        Publica en self.ultimo_receta_lista para la pestaña DEBUG."""
        tipo = self.config["receta_lista_pmc_tipo"]
        byte = self.config["receta_lista_byte"]
        bit = self.config["receta_lista_bit"]
        try:
            with self._client_lock:
                v = self._client.read_pmc_bit(tipo, byte, bit)
            self.ultimo_receta_lista = bool(v)
            return bool(v)
        except Exception:
            self.ultimo_receta_lista = None
            return None

    def _procesar_watchdog_robot(self):
        """Vigila la linea de vida del robot (DI2 del panel Nodka).

        El SPS invierte $OUT[16] cada ~500ms. Aca NO se mira el nivel sino
        que CAMBIE: si el robot se apaga con la salida en 1, la señal queda
        en 1 para siempre y un chequeo por nivel nunca se daria cuenta.

        Lee del CACHE del monitor de dio.py, no del bus: el SMBus no es
        thread-safe y el monitor ya lo pollea cada 100ms.
        """
        if not self.config.get("usar_watchdog_robot", True):
            self.robot_vivo = None
            return
        try:
            from dio import dio
            lectura = dio.robot_desde_cache()
        except Exception:
            lectura = None
        if lectura is None:
            # Sin DIO no se puede saber. No se declara caido para no frenar
            # la linea por un problema del panel de sensores.
            return
        lv, auto = lectura
        self.robot_en_auto = auto
        ahora = time.monotonic()

        if self.robot_lv_ultimo is None:
            # Primera lectura: arrancar el reloj, todavia no se sabe nada
            self.robot_lv_ultimo = lv
            self.robot_lv_cambio_t = ahora
            return
        if lv != self.robot_lv_ultimo:
            self.robot_lv_ultimo = lv
            self.robot_lv_cambio_t = ahora

        limite = self.config.get("watchdog_robot_ms", 2000) / 1000.0
        sin_cambio = ahora - (self.robot_lv_cambio_t or ahora)
        vivo = (sin_cambio <= limite)

        if vivo != self.robot_vivo:
            if vivo:
                self._log("[ROBOT] Linea de vida RECUPERADA. OJO: mientras "
                          "estuvo caida pasaron pallets por el spot 1 sin "
                          "foto, asi que la cola puede haber quedado CORTA. "
                          "Verificar antes de seguir produciendo.")
                self._avisado_robot_caido = False
            else:
                self._log(f"!!! [ROBOT] LINEA DE VIDA CAIDA: el pulso de DI2 "
                          f"no cambia desde hace {sin_cambio:.1f}s. El robot "
                          f"esta apagado o murio el Submit. Dejo de reponer "
                          f"R50.0 para que el Fammar corte la cinta.")
                self._avisado_robot_caido = True
        self.robot_vivo = vivo
        self.segundos_sin_pulso = round(sin_cambio, 1)

    def _leer_system_link(self):
        """Lee R55.0. En 1 el Fammar obedece nuestras condiciones; en 0
        funciona de fabrica y NADA de lo que hacemos protege."""
        if not self.config.get("leer_system_link", True):
            return None
        try:
            with self._client_lock:
                v = self._client.read_pmc_bit(
                    self.config["system_link_pmc_tipo"],
                    self.config["system_link_byte"],
                    self.config["system_link_bit"])
            v = bool(v)
        except Exception:
            return None
        # La señal esta invertida en el ladder: R55.0=0 significa PRENDIDO.
        # Se invierte aca, en el unico lugar donde se lee, asi el resto del
        # codigo y la pantalla trabajan siempre con "True = prendido".
        if self.config.get("system_link_invertido", True):
            v = not v
        if v != self.system_link:
            if v:
                self._log("[FAMMAR] System Link PRENDIDO (R55.0=1): el "
                          "Fammar obedece las condiciones de la PC.")
                self._avisado_sl_apagado = False
            else:
                self._log("!!! [FAMMAR] System Link APAGADO (R55.0=0): el "
                          "Fammar esta funcionando de fabrica. El "
                          "pre-stopper libera solo y las protecciones de la "
                          "PC NO estan activas.")
                self._avisado_sl_apagado = True
        self.system_link = v
        return v

    def _procesar_heartbeat(self):
        """Linea de vida PC<->torno por R50.0. El ladder del torno baja
        R50.0 a 0; la PC la vuelve a poner en 1 en cada tick. Mientras la
        PC este viva, R50.0 vuelve a 1 rapido. Si la PC muere, R50.0 queda
        en 0 y el ladder lo detecta (timeout de su lado)."""
        # CORTE POR ROBOT CAIDO: si la linea de vida del robot se cayo, NO
        # se repone R50.0. El ladder del Fammar ve la linea de vida en 0 y
        # corta la cinta. Es la unica forma que tiene la PC de frenar la
        # cinta, y hace falta: con el robot caido los pallets siguen pasando
        # por el spot 1 sin que nadie los fotografie ni los encole, y la cola
        # se desfasa de la realidad fisica sin recuperacion posible.
        if (self.config.get("usar_watchdog_robot", True)
                and self.robot_vivo is False):
            return
        tipo = self.config["heartbeat_pmc_tipo"]
        byte = self.config["heartbeat_byte"]
        bit = self.config["heartbeat_bit"]
        try:
            with self._client_lock:
                b = self._client.read_pmc_byte(tipo, byte)
                if ((b >> bit) & 1) == 0:
                    self._client.write_pmc_byte(tipo, byte, b | (1 << bit))
                    self._hb_pulsos = getattr(self, "_hb_pulsos", 0) + 1
            self._ok_focas()
        except Exception as e:
            self._registrar_fallo_focas(f"Error en heartbeat R{byte}.{bit}", e)

    def _tick_sincronizar(self):
        """Una iteracion del sync. Asume conectado."""
        mac_cola = self.config["macro_cola"]

        # Informar al torno cuantas piezas hay en la cola (#554).
        # Solo se escribe cuando cambia, para no saturar FOCAS.
        n_cola = len(self.cola)
        if n_cola != getattr(self, "_ultima_cola_len", None):
            with self._client_lock:
                self._client.set_macro(mac_cola, n_cola)
            self._ultima_cola_len = n_cola

        # ============================================================
        #  El SENSOR DE SALIDA es el evento maestro. En el flanco 1->0
        #  (pallet salio) descuenta la cola, escribe la receta del nuevo
        #  primero y baja #553 (libera al torno). Ver _procesar_sensor_salida.
        #
        #  #553 = handshake: el torno lo pone en 1 al terminar y espera el 0.
        #  El 0 lo baja el sensor cuando sale el pallet (receta ya lista).
        # ============================================================
        # Pedido de PRIMERA PIEZA / CINTA VACIA (desde la HMI). Se atiende
        # aca para que las llamadas FOCAS salgan de este thread.
        if self._pedido_dejar_pasar:
            self._pedido_dejar_pasar = False
            self._ejecutar_dejar_pasar()

        if self._pedido_primera_pieza:
            self._pedido_primera_pieza = False
            self._ejecutar_primera_pieza()

        # Watchdog de la linea de vida del robot: va ANTES del sensor de
        # salida, porque _procesar_liberacion consulta self.robot_vivo.
        self._procesar_watchdog_robot()
        self._leer_system_link()

        self._procesar_sensor_salida()

        # Refrescar el sensor de "pallet en posicion" para la pestaña I/O
        # (el metodo publica en self.ultimo_pallet_en_pos).
        self._leer_pallet_en_posicion()

        # Snapshot para la pestaña DEBUG. Se cachea ACA porque la UI no puede
        # hacer llamadas FOCAS (todas tienen que salir de este thread).
        self._refrescar_debug()

        # Heartbeat / linea de vida PC<->torno (R50.0). Si el ladder la
        # bajo a 0, la volvemos a 1: "PC viva". Si la PC muere, queda en 0.
        if self.config.get("usar_heartbeat", True):
            self._procesar_heartbeat()

        # Respaldo SOLO para el arranque: dejar op=3 (dejar pasar) puesto.
        # NUNCA escribir cola[0] aca: una receta real solo puede salir de
        # una liberacion VERIFICADA (_liberar_ahora), si no se manda la
        # receta de un pallet que nadie confirmo (y corrida en uno).
        if self._ultimo_escrito == (None, None, None):
            self._escribir_receta((0, 3, 0))

    def _log(self, msg):
        try:
            self._log_callback(msg)
        except Exception:
            pass