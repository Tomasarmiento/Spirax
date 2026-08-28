"""Diagnostico de red de las camaras D555.

Equivale a 'rs-enumerate-devices --eth': lista TODAS las camaras que la SDK
descubre por DDS, con toda la info de red disponible (IP, MAC, MTU, etc.).

Uso:
    python diag_red.py

Corre esto con LAS DOS camaras conectadas a la vez. Asi vemos si la SDK ve
las dos por red o si una tapa a la otra (conflicto de IP).
"""
import time

import pyrealsense2 as rs

print("pyrealsense2 version:", rs.__version__)
print("-" * 60)

# 1) Probar contexto con DDS explicito
print("Creando contexto con DDS habilitado...")
ctx = None
for settings in ('{"dds": {"enabled": true, "domain": 0}}',
                 '{"dds": true}'):
    try:
        ctx = rs.context(settings)
        print(f"  OK con settings: {settings}")
        break
    except Exception as e:
        print(f"  Fallo con settings {settings}: {e}")
if ctx is None:
    print("  Caigo a contexto por defecto (puede NO ver DDS).")
    ctx = rs.context()

print("-" * 60)
print("Esperando discovery (hasta ~8s)...")
devices = ctx.query_devices()
for _ in range(40):
    if len(devices) > 0:
        break
    time.sleep(0.2)
    devices = ctx.query_devices()

print(f"\nTOTAL de camaras descubiertas: {len(devices)}\n")
print("=" * 60)

# Todos los campos de info que puede tener un device
CAMPOS = [
    ("name", "name"),
    ("serial_number", "serial_number"),
    ("firmware_version", "firmware_version"),
    ("physical_port", "physical_port"),
    ("product_line", "product_line"),
    ("ip_address", "ip_address"),
    ("dds_domain_id", "dds_domain_id"),
]

for i, dev in enumerate(devices):
    print(f"\n--- Camara [{i}] ---")
    for etiqueta, attr in CAMPOS:
        info_enum = getattr(rs.camera_info, attr, None)
        if info_enum is None:
            print(f"  {etiqueta:18s}: (este campo no existe en esta SDK)")
            continue
        try:
            if dev.supports(info_enum):
                valor = dev.get_info(info_enum)
                print(f"  {etiqueta:18s}: {valor}")
            else:
                print(f"  {etiqueta:18s}: (no soportado por este device)")
        except Exception as e:
            print(f"  {etiqueta:18s}: ERROR ({e})")

print("\n" + "=" * 60)
print("Listo. Si ves 2 camaras con IPs distintas -> todo OK.")
print("Si ves 1 sola con las dos conectadas -> conflicto de IP (misma 192.168.11.55).")
