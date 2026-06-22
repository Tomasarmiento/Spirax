import pyrealsense2 as rs
import time

ctx = rs.context('{"dds": {"enabled": true, "domain": 0}}')
time.sleep(5)
dev = ctx.query_devices()[0]
print("Camara:", dev.get_info(rs.camera_info.name))

for s in dev.query_sensors():
    nombre = s.get_info(rs.camera_info.name)
    if nombre != "RGB Camera":
        continue
    print(f"\n=== Perfiles de {nombre} ===")
    for p in s.get_stream_profiles():
        vp = p.as_video_stream_profile()
        if vp:
            print(f"  {p.stream_name()} | {vp.width()}x{vp.height()} | "
                  f"{p.format()} | {p.fps()}fps")