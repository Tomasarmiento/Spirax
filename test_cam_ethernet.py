import pyrealsense2 as rs

# Crear settings con DDS habilitado
settings = '{"dds": {"enabled": true, "domain": 0}}'
ctx = rs.context(settings)

import time
time.sleep(5)  # darle tiempo al discovery DDS

devices = ctx.query_devices()
print("Dispositivos encontrados:", len(devices))
for d in devices:
    print(" -", d.get_info(rs.camera_info.name))
    for i, s in enumerate(d.query_sensors()):
        print(f"    sensor[{i}]:", s.get_info(rs.camera_info.name))