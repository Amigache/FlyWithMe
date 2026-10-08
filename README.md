# FlyWithMe

**Vuelo en formación con ArduPlane y LoRa.** Un avión líder transmite su posición y un avión
seguidor calcula y ejecuta una posición relativa mediante MAVLink. El seguidor debe estar en modo
`GUIDED`. PX4 no está validado.

> FlyWithMe sigue en desarrollo. No sustituye al piloto, a los failsafes del autopiloto ni a un
> sistema anticolisión certificado. Realiza los primeros vuelos con un piloto al mando, observador,
> espacio amplio y un plan independiente para recuperar el control.

## Equipo necesario

- Dos aviones con ArduPlane y TTGO LoRa32 V1 (ESP32 + SX1276).
- Antenas LoRa compatibles con la frecuencia/regulación de tu región; conecta las antenas antes de
  encender las placas.
- Cableado MAVLink UART entre cada TTGO y un puerto TELEM libre del autopiloto.
- Ordenador con PlatformIO para cargar el firmware.

| Conexión UART | TTGO LoRa32 V1 |
|---|---|
| FC TX → ESP32 RX | GPIO12 |
| ESP32 TX → FC RX | GPIO13 |
| Tierra | GND común |

Configura el puerto TELEM seleccionado en ArduPlane para MAVLink2 (`SERIALx_PROTOCOL=2`) y 57600
baudios (`SERIALx_BAUD=57`). `x` depende del puerto físico que uses. Mantén el cableado cruzado y no
conectes botones al menú OLED: sus pines entran en conflicto con las señales de la placa.

## Cargar el firmware

Instala PlatformIO y abre una terminal en la carpeta del proyecto. Todas las placas de vuelo usan
el mismo perfil y el mismo archivo de firmware: `ttgo-lora32-v1-flight`. El rol no se compila dentro
del firmware; se guarda por separado en la NVS de cada placa.

En Windows, identifica primero el puerto COM actual de cada placa. Los CP210x pueden cambiar de COM
al reconectarlos. No des por hecho que un número concreto pertenece siempre al líder o al seguidor.

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"

# Cada comando carga LA MISMA imagen y guarda el rol indicado en esa placa
& $py tools\flash_firmware.py --port COMx --role leader
& $py tools\flash_firmware.py --port COMy --role follower
```

Sustituye los COM por los puertos identificados. El comando compila/carga el perfil común y después
provisiona el rol por USB serie; requiere `pyserial` en el Python usado. Si actualizas desde un
firmware anterior, la placa arrancará en `OFF` hasta que le asignes el rol. Para cambiar solo el rol
de una placa ya cargada, añade `--provision-only`. También se puede modificar `role` desde la WebUI o
Mission Planner en tierra; al cambiarlo, la placa se reinicia para aplicar la identidad.

No flashees mientras un monitor serie u otro programa esté usando ese puerto. Configura
`SYSID_THISMAV=1` en el autopiloto líder y `SYSID_THISMAV=2` en el seguidor; los perfiles asignan
esos mismos SYSID según el parámetro `role`. Una placa nueva queda en `OFF` hasta provisionarla:
`OFF=0`, `FOLLOWER=1`, `LEADER=2`. Al arrancar, comprueba en cada placa que no haya errores y que el
autopiloto esté conectado.

## Configurar en tierra

1. Enciende cada avión por separado y conéctate al Wi-Fi de su TTGO. Cada placa anuncia
   `FWM XXXXXX`, donde `XXXXXX` son los tres últimos bytes de la MAC SoftAP en hexadecimal; por
   ejemplo, `30:AE:A4:07:0D:64` produce `FWM 070D64`. La clave inicial es `12345678`.
2. Abre `http://192.168.4.1`, cambia la clave inicial desde la página del punto de acceso y guarda.
   La placa se reinicia: vuelve a conectarte usando la clave nueva. Repite en la otra placa y revisa
   los parámetros de **ambas**. El punto de acceso se desactiva en vuelo; no dependas de la WebUI
   durante el seguimiento.
3. Confirma `role=LEADER` en la placa del líder y `role=FOLLOWER` en la del seguidor; deja
   `foll_enable` activado en el seguidor. El SSID es único por MAC y no depende del rol. La línea
   `FWM_ID ap_mac=... ap_ssid="..." role=... sysid=...` aparece por serie al arrancar; también se
   puede solicitar enviando `FWM ID` a 57600 baudios.
4. Configura el mismo `netid` en ambos (valor inicial `4660`). El `netid` separa redes, pero no cifra
   ni autentica las comunicaciones.
5. En el seguidor selecciona una formación: **TRAIL**, **LEFT**, **RIGHT**, **ABOVE** o **BELOW**.
   Para los primeros vuelos usa **TRAIL**, conserva la separación inicial configurada (96 m) y deja
   `approach_dist` en 300 m. No reduzcas la separación hasta haber comprobado el enlace y la respuesta
   de los aviones en condiciones reales y controladas.
6. Comprueba antenas, alimentación, GPS, sentidos de control, modos, límites y failsafes de cada
   autopiloto. Verifica que el piloto pueda salir de `GUIDED` y tomar el control en cualquier momento.

Los parámetros se guardan en cada placa, así que revisa sus valores efectivos; cambiar uno no cambia
automáticamente el del otro.

## Preparación y vuelo en formación

1. Despega ambos aviones de forma independiente y mantén el control manual/autopiloto supervisado.
   Espera a que ambos tengan posición GPS válida, estén por encima del umbral de altitud del firmware
   (50 m) y exista enlace entre líder y seguidor.
2. Establece una separación amplia y una trayectoria predecible. Mantén el líder en un modo estable
   admitido por el firmware (FBWA, FBWB, CRUISE, AUTO, RTL, LOITER, TAKEOFF o GUIDED), especialmente
   al acercarte a menos de `approach_dist`.
3. Cuando el seguidor esté estabilizado y a distancia segura, selecciona `GUIDED` en el seguidor para
   habilitar el seguimiento. Verifica que mantiene la formación TRAIL antes de continuar.
4. Empieza con tramos rectos y giros amplios. Vigila continuamente la separación, la altitud, el
   enlace y los avisos del autopiloto. El piloto debe estar listo para abandonar `GUIDED` si algo no
   se comporta como se espera.
5. Para terminar el seguimiento, el piloto debe cambiar el seguidor a un modo aprobado y pilotarlo
   de forma independiente antes de aproximarse o aterrizar. FlyWithMe no realiza una transición
   automática a `LOITER`, recuperación ni aterrizaje.

### Límites importantes

- El umbral de 50 m es una comprobación de altitud del firmware, no un sistema de evitación del
  terreno. Respeta siempre las alturas y límites legales y de tu autopiloto.
- Si el modo del líder deja de ser estable cerca del umbral, FlyWithMe deja de enviar nuevas órdenes;
  el autopiloto puede conservar el último objetivo. Esto **no** equivale a mantener posición.
- No hagas aproximaciones frente a frente, inversiones bruscas cerca ni vuelos con separaciones de
  5–20 m. La guarda frontal es experimental y no debe considerarse un sistema anticolisión.
- El comportamiento depende del enlace de radio, GPS, viento, configuración y autopiloto; no hay
  certificación de seguridad para vuelo real.

## Más información

- [`DEVELOP.md`](DEVELOP.md): documentación técnica para desarrolladores.
- [`docs/ROADMAP.md`](docs/ROADMAP.md): estado de validación y trabajo pendiente.
- [`LICENSE`](LICENSE): GNU GPL v3.0.
