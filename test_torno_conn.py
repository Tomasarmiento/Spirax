"""
test_torno_conn.py - Diagnostico de conexion al torno FANUC.

Corre tres tests aislados:
  1. Ping (ICMP) - chequea conectividad de red basica
  2. TCP raw al puerto 8193 - chequea si el puerto FOCAS escucha
  3. Handshake FOCAS via fanuc_bridge - chequea si la lib puede hablarle

Ejecuta:
    python test_torno_conn.py
    python test_torno_conn.py 172.31.1.99 8193

Util para discriminar entre problema de red, puerto cerrado, o
problema de la libreria FOCAS / DLL.
"""

import sys
import socket
import subprocess


def test_ping(host):
    print(f"\n[1/3] PING a {host}...")
    try:
        # -n 2 = 2 paquetes en Windows, -c 2 en Linux. Probamos Windows.
        res = subprocess.run(
            ["ping", "-n", "2", "-w", "1500", host],
            capture_output=True, text=True, timeout=10
        )
        if res.returncode == 0 and "TTL=" in res.stdout:
            print("    OK - el host responde a ICMP")
            return True
        print("    FALLO - el host NO responde a ping")
        print("    Stdout:", res.stdout.strip()[:200])
        return False
    except Exception as e:
        print(f"    ERROR ejecutando ping: {e}")
        return False


def test_tcp(host, port):
    print(f"\n[2/3] TCP a {host}:{port}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    try:
        s.connect((host, port))
        print(f"    OK - el puerto {port} acepta conexiones TCP")
        s.close()
        return True
    except socket.timeout:
        print(f"    FALLO - timeout (5s) conectando a {port}")
        print("    Causa probable: firewall bloqueando, o puerto no escucha")
        return False
    except ConnectionRefusedError:
        print(f"    FALLO - conexion rechazada (puerto cerrado)")
        print("    Causa probable: FOCAS Ethernet desactivado en el CNC,")
        print("    parametro mal seteado, u otro puerto que el 8193.")
        return False
    except OSError as e:
        print(f"    FALLO - OSError: {e}")
        return False
    except Exception as e:
        print(f"    FALLO - {type(e).__name__}: {e}")
        return False
    finally:
        try:
            s.close()
        except Exception:
            pass


def test_focas(host, port):
    print(f"\n[3/3] HANDSHAKE FOCAS a {host}:{port}...")
    try:
        from fanuc_bridge.client import FanucClient, FocasError
    except ImportError:
        # Fallback igual que en torno.py
        import sys
        from pathlib import Path
        posibles = [
            Path(__file__).parent / "fanuc-bridge" / "src",
            Path(__file__).parent / "fanuc_bridge",
            Path.cwd() / "fanuc-bridge" / "src",
        ]
        client_class = None
        for p in posibles:
            if (p / "client.py").exists():
                sys.path.insert(0, str(p.parent))
                try:
                    from src.client import FanucClient, FocasError
                    client_class = FanucClient
                    break
                except ImportError:
                    pass
        if client_class is None:
            print("    NO PUEDO TESTEAR - fanuc_bridge no esta instalado")
            print("    Buscar en: fanuc-bridge/src/client.py o fanuc_bridge/")
            return False

    try:
        client = FanucClient(host=host, port=port, timeout=10)
        client.connect()
        print(f"    OK - conexion FOCAS establecida")
        # Probar leer una macro para confirmar
        try:
            val = client.get_macro(500)
            print(f"    OK - #500 = {val}")
        except Exception as e:
            print(f"    WARN - conectado pero falla leer macro: {e}")
        client.disconnect()
        return True
    except Exception as e:
        print(f"    FALLO - {type(e).__name__}: {e}")
        # Pistas segun el mensaje
        msg = str(e).lower()
        if "ew_protocol" in msg or "-7" in msg:
            print("    --> EW_PROTOCOL: el puerto responde pero no habla FOCAS.")
            print("        Verificar que sea realmente el puerto FOCAS (no HSSB/EthIO).")
        elif "ew_socket" in msg or "-15" in msg:
            print("    --> EW_SOCKET: no se pudo abrir socket. Firewall? Puerto cerrado?")
        elif "ew_busy" in msg or "-16" in msg:
            print("    --> EW_BUSY: el CNC tiene todos los slots FOCAS ocupados.")
            print("        Cerrar otros clientes FOCAS (MTConnect, OPC, FOCAS demo, etc.)")
        elif "ew_nodll" in msg or "dll" in msg:
            print("    --> Falta la fwlib32.dll o no se puede cargar.")
            print("        Revisar torno_dll_path en config y arquitectura x64/x86.")
        elif "timeout" in msg:
            print("    --> Timeout. Puede ser red lenta o el CNC no esta respondiendo.")
        return False


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "172.31.1.99"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8193

    print("=" * 60)
    print(f"  Diagnostico de conexion al torno FANUC")
    print(f"  Host: {host}   Puerto: {port}")
    print("=" * 60)

    ok_ping = test_ping(host)
    ok_tcp = test_tcp(host, port)
    ok_focas = test_focas(host, port) if ok_tcp else False

    print("\n" + "=" * 60)
    print("  RESUMEN")
    print("=" * 60)
    print(f"  Ping ICMP ........... {'OK' if ok_ping else 'FALLO'}")
    print(f"  TCP {port} ............ {'OK' if ok_tcp else 'FALLO'}")
    print(f"  Handshake FOCAS ..... {'OK' if ok_focas else 'FALLO/SKIP'}")
    print()
    if ok_ping and not ok_tcp:
        print("  >> Red OK pero puerto FOCAS cerrado.")
        print("     Revisar en el CNC: FOCAS Ethernet habilitado y param 8193.")
    elif ok_tcp and not ok_focas:
        print("  >> Puerto abierto pero handshake FOCAS falla.")
        print("     Ver mensaje de error arriba para mas pistas.")
    elif not ok_ping:
        print("  >> Sin conectividad de red basica.")
        print("     Revisar cable, IP, switch, VLAN.")
    elif ok_focas:
        print("  >> Todo OK. Si la HMI sigue desconectada,")
        print("     el problema es del lado de la HMI/thread.")


if __name__ == "__main__":
    main()
