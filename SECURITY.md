# Política de seguridad

FlyWithMe controla el seguimiento de aeronaves reales. Trata cualquier fallo de seguridad como de alta
prioridad, incluidos los que afecten al enlace LoRa, a la WebUI o a la configuración de las placas.

## Versiones soportadas

Solo se mantiene la última release publicada (`v*`) y la rama `main`. Las ramas de desarrollo pueden
cambiar sin aviso.

## Cómo reportar una vulnerabilidad

**No abras un issue público** con detalles de explotación.

1. Usa **Security → Report a vulnerability** en GitHub (reporte privado de vulnerabilidades).
2. Incluye: versión o commit, perfil de firmware (`ttgo-lora32-v1-flight` o `-sitl`), pasos para
   reproducir, impacto esperado (vuelo, configuración, datos) y si requiere acceso al AP WiFi o a la radio.

Responderemos para confirmar la recepción y acordaremos un plazo de corrección y divulgación.

## Alcance

- Firmware de `src/`, herramientas de `tools/` y workflows de `.github/`.
- Flasheador web publicado en GitHub Pages.

Fuera de alcance: fallos de ArduPilot/PX4, del hardware de terceros o de configuraciones inseguras
documentadas (por ejemplo, usar la clave WiFi por defecto en un entorno compartido).

## Riesgos conocidos

Consulta [`docs/AUDITORIA_SEGURIDAD.md`](docs/AUDITORIA_SEGURIDAD.md). En particular:

- El enlace LoRa **no está autenticado ni cifrado**; el `netid` solo separa redes.
- La clave WiFi de fábrica es pública y común a todas las placas. Puede usarse, pero no se obliga a cambiarla.
- FlyWithMe no sustituye al piloto ni a los failsafes del autopiloto.
