# Fanuc Bridge — Spirax

Cliente Python para comunicarse con el torno Fanuc 0i-TF vía FOCAS2 sobre Ethernet.

## Qué hace

- **Lee** estado del torno: modo, ejecución, programa, posición de ejes, RPM del spindle, alarmas, contador de piezas, mensajes del operador.
- **Escribe** variables macro (`#500`, `#501`, etc.) para indicarle al torno qué pieza/operación mecanizar.
- Pensado para el proyecto Spirax: visión RealSense detecta pieza → este bridge le avisa al torno.

## Hardware

- **Torno**: Fanuc 0i-TF en `192.168.1.10:8193`
- **PC visión**: Windows 64 bits + RealSense, en la misma red (`172.31.0.0/16`)

## Setup

### 1. Python

Instalar Python 3.11 (64 bits) desde python.org. La versión normal sirve.

Verificar:
```cmd
python --version
python -c "import platform; print(platform.architecture())"
```
Debería decir `('64bit', 'WindowsPE')`.

### 2. DLLs FOCAS

Necesitamos tres archivos en `dlls/`:

- `Fwlib64.dll`
- `fwlibe64.dll`
- `fwlib30i64.dll`

Bajarlas desde:
https://github.com/TrakHound/Fanuc-MTConnect-Agent/tree/master/Adapter%20Templates/30i

(Click en cada archivo → botón "Download raw file" arriba a la derecha).

### 3. Verificar conectividad

Antes de correr nada:
```cmd
ping 192.168.1.10
telnet 192.168.1.10 8193
```
Si el ping responde y telnet conecta (pantalla negra), todo OK.

### 4. Probar

```cmd
python -m src.test_connection
python -m src.read_status
```

## Estructura

```
fanuc-bridge/
├── dlls/                  # Aquí van las DLLs (no se versionan)
├── src/
│   ├── focas.py           # Wrapper ctypes de la librería FOCAS
│   ├── client.py          # Cliente de alto nivel (FanucClient)
│   ├── test_connection.py # Test mínimo de conectividad
│   ├── read_status.py     # Lee y muestra estado del torno
│   └── write_macro.py     # Escribe variables macro (Spirax)
├── config.ini             # IP, puerto, etc.
└── README.md
```

## Variables macro de Spirax

Convención propuesta para el handshake PC ↔ torno:

| Macro | Significado                       | Quién escribe | Quién lee |
|-------|-----------------------------------|---------------|-----------|
| `#500`| Modelo de pieza (1=WCB, 2=T-alu)  | PC visión     | Torno     |
| `#501`| Operación (10 ó 20)               | PC visión     | Torno     |
| `#502`| Flag "datos listos" (0/1)         | PC visión     | Torno     |
| `#503`| Flag "torno terminó" (0/1)        | Torno         | PC visión |

El torno corre un programa dispatcher que espera `#502=1`, ramifica según `#500` y `#501`, mecaniza, y al final setea `#503=1` para que la PC sepa que terminó.

## Próximos pasos

1. ✅ Verificar conectividad (ping y telnet OK).
2. ✅ Conseguir DLLs (de TrakHound).
3. ⏳ Correr `test_connection.py`.
4. ⏳ Correr `read_status.py` y validar lecturas contra lo que muestra el panel del torno.
5. Recién después: probar escritura en una macro de prueba (`#100`).
6. Recién después: integrar con pipeline de visión.

## Referencias

- Documentación FOCAS: https://www.inventcom.net/fanuc-focas-library/general/fwlib32
- Repo Sinapsis (adapter Linux): `agudecima-sinapsis/fanuc-connector`
- DLLs Windows (las que usamos): https://github.com/TrakHound/Fanuc-MTConnect-Agent/tree/master/Adapter%20Templates/30i
