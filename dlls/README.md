# Carpeta de DLLs

Acá van las DLLs de Fanuc FOCAS. No se versionan (gitignore).

## Qué archivos van acá (Windows 64 bits)

- `Fwlib64.dll`
- `fwlibe64.dll`
- `fwlib30i64.dll`

## De dónde sacarlas

Repositorio público de TrakHound:
https://github.com/TrakHound/Fanuc-MTConnect-Agent/tree/master/Adapter%20Templates/30i

Click en cada archivo → "Download raw file" → guardar acá.

## Si tu PC fuera 32 bits

Bajar las versiones equivalentes sin el "64":
- `Fwlib32.dll`
- `fwlibe1.dll`
- `fwlib30i.dll`

El código detecta automáticamente la arquitectura de Python y carga la DLL correcta.
