"""Lista los seriales de las camaras RealSense conectadas (D555 por DDS).

Uso:
    python listar_seriales.py

Corre esto con las DOS camaras conectadas. Anota cual serial corresponde
a la CINTA y cual a la MESA, y completalos en vision.py:

    SERIAL_POR_ESTACION = {
        "cinta": "XXXXXXXX",
        "mesa":  "YYYYYYYY",
    }

Para identificar cual es cual: corre el script, despues desconecta UNA camara
y volve a correrlo; el serial que desaparece es el de la que desconectaste.
"""
import time

import pyrealsense2 as rs

# Mismo contexto DDS que usa vision.py para el D555.
try:
    from vision import DDS_SETTINGS
    ctx = rs.context(DDS_SETTINGS)
except Exception as e:
    print(f"[DDS] No pude crear contexto DDS ({e}). Uso contexto por defecto.")
    ctx = rs.context()

print("Esperando discovery DDS (puede tardar unos segundos)...")
devices = ctx.query_devices()
for _ in range(30):
    if len(devices) > 0:
        break
    time.sleep(0.2)
    devices = ctx.query_devices()

if len(devices) == 0:
    print("\nNo se descubrio ninguna camara.")
    print("Verifica: IP de la PC en 192.168.11.x, cable de red, y que el "
          "realsense-viewer vea la camara.")
else:
    print(f"\n{len(devices)} camara(s) encontrada(s):\n")
    for i, dev in enumerate(devices):
        try:
            nombre = dev.get_info(rs.camera_info.name)
        except Exception:
            nombre = "?"
        try:
            serial = dev.get_info(rs.camera_info.serial_number)
        except Exception:
            serial = "?"
        try:
            fw = dev.get_info(rs.camera_info.firmware_version)
        except Exception:
            fw = "?"
        try:
            ip = dev.get_info(rs.camera_info.ip_address)
        except Exception:
            ip = "(no IP / no DDS)"
        print(f"  [{i}] {nombre}")
        print(f"      serial = {serial}")
        print(f"      fw     = {fw}")
        print(f"      ip     = {ip}\n")
