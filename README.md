# 🚁 ROS Workspace - Control del Drone Parrot Bebop 2

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

Este repositorio contiene un **workspace ROS Noetic** con paquetes para controlar un dron **Parrot Bebop 2** mediante ROS.
Incluye instalación desde cero, configuración del entorno, comandos básicos de vuelo y ejemplos de Python.


---

<a id="indice"></a>

## 📋 Índice
- [📦 Estructura del Proyecto](#estructura-del-proyecto)
- [✅ Requisitos](#requisitos)
- [🔧 Instalación desde Cero](#instalacion-desde-cero)
- [📁 Creación de Packages ROS para el Bebop 2](#creacion-de-packages)
  - [1. ¿Por qué usar packages ROS?](#por-que-usar-packages-ros)
  - [2. Estructura del Workspace](#estructura-del-workspace)
  - [3. Crear un Package para el Bebop 2](#crear-un-package-para-el-bebop-2)
  - [4. Organización del Código](#organizacion-del-codigo)
- [▶️ Uso del Drone Parrot Bebop 2](#uso-del-drone-parrot-bebop-2)
  - [1. Conexión con el dron](#conexion-con-el-dron)
  - [2. Iniciar ROS](#iniciar-ros)
  - [3. Lanzar el Nodo Principal](#lanzar-el-nodo-principal)
  - [4. Comandos Básicos](#comandos-basicos)
  - [5. Cámara del Bebop](#ver-la-camara)
  - [6. Verificar Tópicos Disponibles](#verificar-topicos-disponibles)
  - [7. Sensores y Estimación de Movimiento](#estimacion-movimiento)
  - [8. Transformaciones tf en ROS](#transformaciones-tf)
  - [9. Visualizar Nodos y Tópicos (rqt_graph)](#visualizar-nodos-y-topicos-rqt-graph)
  - [10. Ejemplo Python - Vuelo Simple](#ejemplo-python-vuelo-simple)



 
---

<a id="estructura-del-proyecto"></a>

## 📦 Estructura del Proyecto

```
bebop_ws/
 ├── build/
 ├── devel/
 └── src/
      ├── parrot_arsdk       # Package del Wrapper SDK Parrot
      └── bebop_autonomy     # Package del Driver principal ROS
  Package del 

```

[🔙 Volver al Índice](#indice)

---

<a id="requisitos"></a>

## ✅ Requisitos

* **Sistema operativo:** Ubuntu 20.04 LTS
* **ROS Noetic** → [Instalación oficial](http://wiki.ros.org/noetic/Installation/Ubuntu)

**Dependencias ROS:**

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-cmake-modules \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-message-filters \
    ros-noetic-camera-info-manager
```

**Librerías del sistema:**

```bash
sudo apt-get install -y \
    build-essential cmake git python3-catkin-tools \
    libavcodec-dev libavdevice-dev libavformat-dev \
    libavutil-dev libswscale-dev libeigen3-dev \
    libopencv-dev libsdl1.2-dev libusb-1.0-0-dev \
    libgles2-mesa-dev libcurl4-openssl-dev unzip
```

**Extras:**

```bash
sudo apt-get install -y \
    ros-noetic-joy ros-noetic-geometry-msgs ros-noetic-cv-bridge \
    python3-numpy python3-opencv python3-yaml python3-pip
```

[🔙 Volver al Índice](#indice)

---
<a id="instalacion-desde-cero"></a>

## 🔧 Instalación desde Cero

### [1] Crear workspace y compilar

```bash
mkdir -p ~/bebop_ws/src
cd ~/bebop_ws
catkin_make
```

### [2] Configurar entorno

```bash
echo "source ~/bebop_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### [3] Clonar repositorios [como se indica](https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev):

* **Parrot ARSDK (Parrot Augmented Reality Software Development Kit)**

Parrot ARSDK (Augmented Reality Software Development Kit) es la librería oficial de Parrot que permite controlar sus drones desde software externo. Proporciona herramientas para enviar comandos de vuelo, mover la cámara, capturar fotos y video, así como acceder a datos en tiempo real como batería, GPS, altitud y velocidad. Es el Software Development Kit (SDK) oficial de Parrot que permite a desarrolladores comunicarse y controlar los drones de la marca. 


```bash
cd ~/bebop_ws/src
git clone https://github.com/antonellabarisic/parrot_arsdk.git
cd parrot_arsdk
git checkout noetic_dev
sudo apt-get install libavahi-client-dev
sudo ln -s /usr/bin/python3 /usr/bin/python
cd ~/bebop_ws
catkin_make
```

* **Bebop Autonomy**

Eso que ves es Bebop Autonomy, un driver ROS (Robot Operating System) desarrollado por AutonomyLab para controlar los drones Parrot Bebop y Bebop 2. Básicamente, es un paquete ROS que actúa como interfaz entre el dron y ROS, traduciendo los comandos y publicando la información del dron en topics.

```bash
cd ~/bebop_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

* **Ajustes necesarios para compilar bebop_autonomy**

Para que el driver bebop_autonomy compile y funcione correctamente en sistemas modernos, se deben realizar los siguientes pasos:

###### ◦ a) Modificar *bebop\_driver/src/bebop\_video\_decoder.cpp*:

En versiones recientes de FFmpeg/libavcodec, algunas macros y banderas cambiaron de nombre. Por lo tanto, abre:

```
bebop_driver/src/bebop_video_decoder.cpp
```

y reemplaza:

```
línea 93: CODEC_AP_TRUNCATED -> AV_CODEC_CAP_TRUNCATED
línea 95: CODEC_FLAG_TRUNCATED -> AV_CODEC_FLAG_TRUNCATED
línea 97: CODEC_FLAG2_CHUNKS -> AV_CODEC_FLAG2_CHUNKS
```

Estos cambios se hacen porque el código original de bebop_autonomy fue escrito para versiones antiguas de FFmpeg, y sin esas correcciones la compilación fallaría con errores de símbolos no definidos.


###### b) Configurar la variable de entorno `LD_LIBRARY_PATH`

Añade lo siguiente a tu `~/.bashrc` para que el sistema encuentre las librerías del ARSDK:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/bebop_ws/devel/lib/parrot_arsdk
```

Luego ejecuta:

```bash
source ~/.bashrc
```

Esto se hace para que el sistema sepa dónde encontrar las librerías del ARSDK (las de Parrot) al momento de ejecutar los nodos. Si no lo añades, al correr los launch files de bebop_autonomy puedes obtener errores del tipo:

```bash
error while loading shared libraries: libarsdk.so: cannot open shared object file: No such file or directory
```

### [4] Compilar todo

Es importante **compilar en un solo hilo** para evitar errores de linking con librerías externas:

```bash
cd ~/bebop_ws
catkin_make -j1
source devel/setup.bash
```
[🔙 Volver al Índice](#indice)

---

<a id="creacion-de-packages"></a>

## 📁 Creación de Packages ROS para el Bebop 2

Para el desarrollo de scripts personalizados, control autónomo y futuros algoritmos de navegación,
se recomienda crear uno o más **packages ROS** dentro del workspace.
Aunque el driver del Parrot Bebop 2 permite controlar el dron directamente desde la terminal,
el uso de packages facilita la organización, reutilización y escalabilidad del código.

---
<a id="por-que-usar-packages-ros"></a>

### 1. ¿Por qué usar packages ROS?

El uso de packages ROS permite:

* Organizar el código de control del dron de forma estructurada
* Desarrollar nodos propios en Python o C++
* Facilitar la ejecución mediante archivos `launch`
* Preparar el sistema para control autónomo, visión y navegación
* Mantener separado el código del usuario del driver del Bebop

---
<a id="estructura-del-workspace"></a>

### 2. Estructura del Workspace

Se asume el uso de un workspace `catkin_ws` ubicado en el directorio home del usuario:

```bash
~/catkin_ws/
├── src/
│   ├── bebop_autonomy/      # Driver del Bebop 2
│   └── bebop_control/       # Package del usuario
├── devel/
└── build/
```

El package `bebop_autonomy` corresponde al driver oficial del dron, mientras que
`bebop_control` será utilizado para el desarrollo de código propio.

---
<a id="crear-un-package-para-el-bebop-2"></a>

### 3. Crear un Package para el Bebop 2

Desde el directorio `src` del workspace, crear el package:

```bash
cd ~/catkin_ws/src
catkin_create_pkg bebop_control rospy geometry_msgs sensor_msgs std_msgs
```

> **Nota:** El nombre `bebop_control` es solo una recomendación.
> Puede sustituirse por cualquier otro nombre que se ajuste a las necesidades del proyecto,
> por ejemplo `bebop_autonomy`, `bebop_navigation` o `drone_control`.
> En caso de cambiar el nombre del package, deberá utilizarse el mismo nombre al ejecutar
> nodos con `rosrun` o al crear archivos `launch`.

Luego compilar el workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Este package será utilizado para almacenar scripts de control, nodos de prueba y ejemplos de vuelo.

---
<a id="organizacion-del-codigo"></a>

### 4. Organización del Código

Se recomienda la siguiente estructura dentro del package:

```bash
bebop_control/
├── scripts/
│   ├── takeoff_land.py
│   ├── simple_flight.py
│   └── keyboard_control.py
├── launch/
│   └── simple_flight.launch
├── CMakeLists.txt
└── package.xml
```

* `scripts/`: nodos en Python para control del dron
* `launch/`: archivos para lanzar nodos automáticamente
* `package.xml`: dependencias del package

Los scripts deben tener permisos de ejecución:

```bash
chmod +x scripts/*.py
```

---

Al finalizar esta sección, el sistema queda listo para ejecutar tanto los comandos básicos del driver como scripts personalizados desde ROS.

[🔙 Volver al Índice](#indice)

---

<a id="uso-del-drone-parrot-bebop-2"></a>

## ▶️ Uso del Drone Parrot Bebop 2

Esta sección describe el uso del Parrot Bebop 2 mediante los comandos y nodos
proporcionados directamente por el driver, sin el uso de packages personalizados.
El objetivo es familiarizarse con la conexión, los tópicos disponibles y el
comportamiento básico del dron utilizando únicamente la terminal. Este enfoque
permite validar el correcto funcionamiento del sistema antes de introducir
código propio o control programado.

<a id="conexion-con-el-dron"></a>

### [1] Conexión con el dron

---

#### 🔹 Conectar a la red WiFi del Bebop

Conéctate desde la configuración de tu sistema o usando la siguiente línea de comando:

```bash
nmcli dev wifi connect "Bebop2-XXXXXX"
```

> Reemplaza `"Bebop2-XXXXXX"` con el nombre exacto de la red de tu dron.

---

#### 🔹 Verificar IP y conexión de red

Después de conectarte, debes asegurarte de que tu equipo tiene la IP correcta y puede comunicarse con el dron.

##### 1. **Comprobar la red conectada**

   ```bash
   iwconfig
   ```

   * **Para qué sirve:** Muestra la interfaz inalámbrica y la red actual.
   * **Qué deberías ver:** El nombre de la interfaz (ej. `wlo1`) y el SSID del dron (`Bebop2-XXXXXX`).

##### 2. **Confirmar la IP asignada**

   ```bash
   ifconfig wlo1
   ```

   * **Para qué sirve:** Verifica la configuración de la interfaz WiFi.
   * **Qué deberías ver:** Una IP en el rango `192.168.42.xx`. Ejemplo:

     ```
     inet 192.168.42.22  netmask 255.255.255.0
     ```

##### 3. **Asignar IP manualmente si no hay**

   ```bash
   sudo dhclient wlo1
   ```

   * **Para qué sirve:** Solicita una dirección IP al dron.
   * **Qué deberías ver:** Tras ejecutar de nuevo `ifconfig wlo1`, ahora aparece la IP correcta.

---

#### 🔹 Verificar conexión con ping

```bash
ping 192.168.42.1
```

Al probar la conexión, pueden ocurrir dos casos:

❌ **Respuesta incorrecta (sin conexión activa):**

```
PING 192.168.42.1 (192.168.42.1) 56(84) bytes of data.
From 192.168.42.22 icmp_seq=1 Destination Host Unreachable
^C
--- 192.168.42.1 ping statistics ---
3 packets transmitted, 0 received, +1 errors, 100% packet loss, time 2033ms
```

✅ **Respuesta correcta (conexión activa):**

```
PING 192.168.42.1 (192.168.42.1) 56(84) bytes of data.
64 bytes from 192.168.42.1: icmp_seq=1 ttl=64 time=2.76 ms
64 bytes from 192.168.42.1: icmp_seq=2 ttl=64 time=3.01 ms
...
64 bytes from 192.168.42.1: icmp_seq=27 ttl=64 time=83.4 ms
^C
--- 192.168.42.1 ping statistics ---
27 packets transmitted, 27 received, 0% packet loss, time 26050ms
rtt min/avg/max/mdev = 1.567/23.327/166.341/43.711 ms
```

💡 **Interpretación:**

* Si ves el ejemplo ✅, la conexión con el dron está activa y puedes continuar con los comandos de ROS.
* Si aparece el ejemplo ❌, revisa la conexión WiFi, la IP y asegúrate de que el dron esté encendido.

[🔙 Volver al Índice](#indice)

---

<a id="iniciar-ros"></a>

### [2] Iniciar ROS

Antes de ejecutar cualquier nodo o comando, debes iniciar el **roscore**, que es el núcleo de ROS.
`roscore` es un servicio que permite que todos los nodos y tópicos de ROS se comuniquen entre sí.

```bash
roscore
```

> Debe mantenerse ejecutando en una terminal mientras usas ROS.

[🔙 Volver al Índice](#indice)

---

<a id="lanzar-el-nodo-principal"></a>

### [3] Lanzar el Nodo Principal

El nodo principal del Bebop (`bebop_node`) controla la comunicación con el dron, recibe datos de sensores y envía comandos de vuelo.
Para iniciarlo:

```bash
roslaunch bebop_driver bebop_node.launch
```

> Este comando se ejecuta en una nueva terminal con `setup.bash` cargado.
> Una vez lanzado, el dron estará listo para recibir comandos y enviar datos a ROS.

[🔙 Volver al Índice](#indice)

---

<a id="comandos-basicos"></a>

### [4] Comandos Básicos

Esta sección te permite **controlar el dron desde la terminal** mediante `rostopic pub`, publicando mensajes en los tópicos correspondientes.

> ⚠️ **Precaución:** Antes de ejecutar cualquier comando, asegúrate de tener suficiente espacio libre alrededor del dron y que no haya obstáculos. Cada comando debe ejecutarse en una nueva terminal con el `setup.bash` cargado, mientras `roscore` y el nodo principal están corriendo.

---


#### ▶🔹 Diferencia entre `--once` y `-r <rate>`

> 🟢 `--once` → Movimiento **instantáneo**, solo un impulso breve.
> 🔵 `-r 10` → Movimiento **continuo**, se repite 10 veces por segundo hasta detenerlo (Ctrl+C o Detener movimiento).

---

#### ▶🔹 Despegar y aterrizar

* *Despegar*

El dron despega y se mantiene flotando a baja altura (\~1 m).

```bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

* *Aterrizar*

El dron desciende suavemente hasta tocar el suelo.

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

---

#### ▶ Movimientos Básicos del Bebop

##### 1) Avanzar 

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

Avanza solo un instante (\~unos centímetros).

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

Avanza continuamente a 0.1 m/s hasta que presiones Ctrl+C o publiques **Detener movimiento**.

---
##### 2) Retroceder

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: -0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

Retrocede solo un instante (\~unos centímetros).

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: -0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

Retrocede continuamente a -0.1 m/s hasta que presiones Ctrl+C o publiques **Detener movimiento**.

---

##### 3) Giros izquierda

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5'
```

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5'

```

---

##### 4) Giros derecha

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.5'
```

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.5'

```

---

##### 5) Subir

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: 0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: 0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

---

##### 6) Bajar

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: -0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.0
  z: -0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

---

##### 7) Movimiento lateral derecha

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: -0.2
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

###### * **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: -0.2
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

---

##### 8) Movimiento lateral izquierda

###### * **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.2
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

###### * **🔵 Continuo:**
  
```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'linear:
  x: 0.0
  y: 0.2
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0'
```

---


#### ▶🔹 Detener o emergencia

* *Detener movimiento:* Frenar inmediatamente cualquier movimiento continuo:

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* *Emergencia:* Apaga los motores de inmediato y reinicia los sistemas del dron.

```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```

---

> ⚠️ **Recomendaciones generales:**
>
> * Ajusta los valores `x`, `y`, `z` para controlar velocidad y dirección.
> * Mantén siempre suficiente espacio alrededor del dron antes de ejecutar comandos de vuelo.
> * Para detener cualquier movimiento continuo, puedes usar **Ctrl+C** o publicar un **Detener movimiento**.
> * Para situaciones de riesgo o emergencia, usa **`/bebop/reset`** para **apagar los motores de inmediato**.

[🔙 Volver al Índice](#indice)

---
<a id="ver-la-camara"></a>

### [5] Cámara del Bebop 🎥


El Bebop 2 permite mover y acceder a la cámara delantera en tiempo real, así como guardar imágenes o grabar video para análisis posterior.

---

#### 1. Ver la cámara en tiempo real

Permite **visualizar la cámara delantera del dron en tiempo real**.

```bash
rqt_image_view /bebop/image_raw
```

Esto abrirá una ventana con el video en vivo.

> Útil para inspeccionar el entorno o realizar pruebas de visión por computadora.

Para ver la posición actual de la cámara:

```bash
rostopic echo /bebop/states/ardrone3/CameraState/Orientation
```

---

#### 2. Mover la cámara

El **Bebop 2** permite mover su cámara mediante el tópico `/bebop/camera_control`.
Este tópico utiliza mensajes del tipo `geometry_msgs/Twist`, donde:

* `angular.y` → **Tilt** (arriba / abajo, rango `-83°` a `+83°`).
* `angular.z` → **Pan** (izquierda / derecha, rango `-180°` a `+180°`).
* Los demás campos (`linear.*`, `angular.x`) se mantienen en `0`.

### Comandos de ejemplo

👉 **Apuntar al piso (tilt -83°):**

```bash
rostopic pub --once /bebop/camera_control geometry_msgs/Twist \
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: -83.0
  z: 0.0"
```

👉 **Apuntar al techo (tilt +83°):**

```bash
rostopic pub --once /bebop/camera_control geometry_msgs/Twist \
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 83.0
  z: 0.0"
```

👉 **Girar a la derecha (pan +90°):**

```bash
rostopic pub --once /bebop/camera_control geometry_msgs/Twist \
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 90.0"
```

👉 **Girar a la izquierda (pan -90°):**

```bash
rostopic pub --once /bebop/camera_control geometry_msgs/Twist \
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -90.0"
```


---

#### 3. Guardar imágenes

Se pueden guardar capturas directamente desde el tópico de la cámara:

```bash
rosrun image_view image_saver image:=/bebop/image_raw _filename_format:=/tmp/frame%04d.jpg
```

Esto guardará imágenes en `/tmp/` con nombres como `frame0001.jpg`, `frame0002.jpg`, etc.

---

#### 4. Grabar video (rosbag)

También es posible grabar un **rosbag** con el stream de la cámara para análisis posterior:

```bash
rosbag record /bebop/image_raw
```

Con este archivo se pueden reproducir y extraer imágenes o reconstruir video después.


[🔙 Volver al Índice](#indice)

---

<a id="verificar-topicos-disponibles"></a>

### [6] Verificar Tópicos Disponibles

Los **tópicos** son canales de comunicación entre nodos de ROS.
Esta sección permite **ver qué información envía y recibe el dron**, como la cámara, la odometría o la batería.

```bash
rostopic list | grep bebop
```

> Esto mostrará todos los tópicos relacionados con el dron.
> Para ver la información de un tópico en tiempo real, usa `rostopic echo <nombre_del_topic>`.

Principales:

```
/bebop/cmd_vel
/bebop/takeoff
/bebop/land
/bebop/reset
/bebop/image_raw
/bebop/odom
/bebop/imu/data
/bebop/states/common/CommonState/BatteryStateChanged
```

Ver datos de batería:

```bash
rostopic echo /bebop/states/common/CommonState/BatteryStateChanged
```

Ver Altura:

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/AltitudeChanged
```


Ver Ángulos de pitch, roll y yaw:

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/AttitudeChanged
```

Ver Posición estimada en GPS o local (si está disponible):

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/PositionChanged
```

Ver Si el dron está sobrecalentado:

```bash
rostopic echo /bebop/states/common/OverHeatState/OverHeatChanged
```

Ver Intensidad de la señal WiFi:

```bash
rostopic echo /bebop/states/common/CommonState/WifiSignalChanged
```

Ver Estado de vuelo: tierra, despegando, volando, aterrizando:

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/FlyingStateChanged
```

Ver Estado de vuelo: tierra, despegando, volando, aterrizando:

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/FlyingStateChanged
```

Perfecto 👌, aquí tienes los comandos siguiendo la misma **sintaxis de tu README**:

---

Ver Odometría completa (posición y orientación en 3D):

```bash
rostopic echo /bebop/odom
```

Ver Solo posición (x, y, z):

```bash
rostopic echo /bebop/odom/pose/pose
```

Ver Solo velocidades lineales y angulares:

```bash
rostopic echo /bebop/odom/twist/twist
```

Ver Frecuencia de odometría (Hz):

```bash
rostopic hz /bebop/odom
```

Ver Datos completos del IMU (orientación, aceleración, giroscopio):

```bash
rostopic echo /bebop/imu/data
```


[🔙 Volver al Índice](#indice)

---

<a id="estimacion-movimiento"></a>

### [7] Sensores y Estimación de Movimiento

El **Bebop 2** cuenta con varios sensores que permiten estimar su posición, velocidad y orientación.
En ROS, se puede acceder a esta información a través de tópicos como `/bebop/imu/data`, `/bebop/odom`, `/bebop/states/ardrone3/PilotingState/PositionChanged` y `/bebop/image_raw`.

---

#### IMU (Unidad de Medición Inercial)

**Características:**

* Sensor interno que mide aceleraciones y velocidades angulares.
* Alta frecuencia de actualización.
* Propenso a **deriva** si se integra para calcular posición.

**Cuándo usar:**

* Mantener estabilidad del dron (control de actitud).
* Detectar vibraciones o movimientos rápidos.
* Base para calcular velocidad o posición en combinación con otros sensores.

**Cuándo no usar:**

* Como única fuente para obtener posición absoluta o altitud exacta.

**Tópicos ROS:**

```bash
# IMU completa (aceleración, orientación y giroscopio)
rostopic echo /bebop/imu/data
```

---

#### Odometría

**Características:**

* Estimación de posición y velocidad del dron en el espacio.
* Calculada fusionando IMU, cámara, altímetro y GPS.
* Publica pose y velocidades lineales y angulares.

**Cuándo usar:**

* Navegación autónoma y planificación de trayectorias.
* Control de posición y velocidad en interiores y exteriores.

**Cuándo no usar:**

* Solo en exteriores sin GPS, si se necesita precisión global absoluta.

**Tópicos ROS:**


Odometría completa (pose + velocidades)
```bash
rostopic echo /bebop/odom
```

Solo posición (x, y, z)
```bash
rostopic echo /bebop/odom/pose/pose
```

Solo orientación (cuaternión)
```bash
rostopic echo /bebop/odom/pose/pose/orientation
```

Solo velocidades lineales y angulares
```bash
rostopic echo /bebop/odom/twist/twist
```

Frecuencia de publicación
```bash
rostopic hz /bebop/odom
```

---

#### GPS

**Características:**

* Proporciona coordenadas globales (latitud, longitud, altitud).
* Precisión moderada, depende de la señal satelital.

**Cuándo usar:**

* Vuelos exteriores para referencia global.
* Complementar la odometría interna para mejorar precisión.

**Cuándo no usar:**

* Interiores o zonas con señal GPS débil o nula.

**Tópico ROS:**

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/PositionChanged
```

---

#### Cámara frontal → Visual Odometry

**Características:**

* Estima desplazamiento relativo mediante imágenes.
* Permite complementar la odometría con referencia visual.
* Requiere buena iluminación y texturas en el entorno.

**Cuándo usar:**

* Interiores o zonas sin GPS.
* Detectar movimiento relativo o obstáculos.

**Cuándo no usar:**

* Escenas homogéneas (paredes lisas) o poca luz.
* Como única fuente para posicionamiento global en exteriores.

**Tópico ROS:**

```bash
rostopic echo /bebop/image_raw
```

---

#### Altímetro / Barómetro → Estimación de altura sobre el suelo

**Características:**

* Mide presión atmosférica y la convierte en altura relativa.
* Alta frecuencia y confiable a corto plazo.

**Cuándo usar:**

* Mantener altura constante en interiores y exteriores.
* Complementar GPS para control de altitud.

**Cuándo no usar:**

* No reemplaza medición precisa de altitud global si se necesita para navegación exterior exacta.

**Tópico ROS:**

```bash
rostopic echo /bebop/states/ardrone3/PilotingState/AltitudeChanged
```

---

💡 **Resumen visual rápido:**

```
Sensor      → Datos                         → Uso principal
IMU         → aceleración + giros          → Control de actitud, detección de vibraciones
Cámara      → imágenes (Visual Odometry)  → Estimación de desplazamiento relativo
Altímetro   → presión atmosférica          → Control de altura sobre el suelo
GPS         → coordenadas globales         → Posición global en exteriores
-----------------------------------------------------------
Odometría   → posición + orientación + velocidades → Navegación y planificación
```


[🔙 Volver al Índice](#indice)

---

<a id="transformaciones-tf"></a>

### [8] Transformaciones tf en ROS

El paquete **`tf`** de ROS permite **gestionar los marcos de referencia (frames)** del dron y transformar posiciones y orientaciones entre ellos.
No es un sensor: **no mide nada**, sino que organiza y relaciona los datos que vienen de IMU, cámara, GPS, altímetro y odometría.

---

#### 🔹 Qué hace `tf`

* Mantiene un **árbol de frames** para todo el dron y su entorno.
* Permite **transformar coordenadas de un frame a otro** automáticamente.
* Facilita la **planificación de trayectorias, seguimiento de objetos y visualización en RViz**.

**Ejemplo de frames en Bebop 2:**

| Frame          | Descripción                                 |
| -------------- | ------------------------------------------- |
| `/odom`        | Referencia de odometría (posición estimada) |
| `/base_link`   | Centro del dron                             |
| `/camera_link` | Cámara frontal                              |
| `/map`         | Referencia global opcional                  |

---

#### 🔹 Comandos ROS importantes

Ver la posición y orientación de un frame respecto a otro:

```bash
rosrun tf tf_echo /odom /base_link
```

Ver el frame de la cámara respecto al dron:

```bash
rosrun tf tf_echo /odom /camera_link
```

Ver el **árbol completo de frames** y generar un PDF con las relaciones:

```bash
rosrun tf view_frames
```

Visualizar en tiempo real en RViz:

* Añade un **TF Display** y selecciona `/odom` como marco base.
* Verás cómo todos los frames (cámara, base, sensores) se posicionan en el espacio.

---

#### 🔹 Cómo se relaciona con los sensores

* **IMU** → orientación y velocidad angular → se refleja en `/base_link`.
* **Cámara** → Visual Odometry → posición relativa de `/camera_link`.
* **Altímetro** → altura sobre el suelo → se refleja en `/base_link`.
* **GPS** → posición global → opcionalmente se relaciona con `/map`.
* **Odometría** → estimación de posición → frame principal `/odom` para referencia de todos los demás.

---

#### 🔹 Cuándo conviene usar `tf`

* Cuando necesitas **relacionar sensores con la posición del dron** para planificar movimientos.
* Para **visualización en RViz** y depuración de vuelo.
* En **seguimiento de objetos**: convierte coordenadas de la cámara a un frame del dron.
* Para **control de cámaras** o coordinación de múltiples sensores.
* En **fusión de sensores**, para mantener consistencia entre IMU, altímetro, GPS y odometría.

---

#### 🔹 Casos reales de uso

1. **Navegación autónoma en interiores:**

   * Transformas puntos de un mapa (`/map`) al frame del dron (`/base_link`) para planificar trayectorias.

2. **Seguimiento de un objeto detectado por la cámara:**

   * Transformas la posición del objeto (`/camera_link`) al frame del dron (`/base_link`) y ajustas comandos de vuelo.

3. **Visualización en RViz:**

   * Comprobar que la odometría y la posición estimada coinciden con la posición real.

4. **Control de sensores:**

   * Coordinar IMU, cámara y altímetro para mantener estabilidad y altura constante.

---

#### 🔹 Mini-diagrama conceptual de `tf` en un caso real

```
                /map (opcional)
                     |
                   /odom
                     |
                 /base_link
                /          \
      /camera_link       /sensor_frames (IMU, altímetro)
                \           /
              Objetos detectados
```

> Cada frame se actualiza en tiempo real, permitiendo al dron saber la posición relativa de sensores, cámara, objetos y su propia odometría.


[🔙 Volver al Índice](#indice)

---

<a id="visualizar-nodos-y-topicos-rqt-graph"></a>

### [9] Visualizar Nodos y Tópicos (`rqt_graph`)

`rqt_graph` muestra un **diagrama visual de los nodos y sus conexiones** en ROS.
Esto te ayuda a entender cómo se comunican los distintos componentes del dron, por ejemplo:

* Qué nodo envía comandos a los motores (`cmd_vel`)
* Qué nodo publica las imágenes de la cámara
* Qué nodo informa el estado de la batería o la odometría

```bash
rqt_graph
```

> Ideal para depurar problemas o entender la arquitectura de ROS si eres nuevo en el sistema.

Ejemplo de flujo básico en Bebop:

```
         +-------------+
         |bebop_node   |
         +-------------+
          /     |      \
     cmd_vel  camera    state
       |        |         |
   [motores]  [video]   [info]
```
[🔙 Volver al Índice](#indice)

---

<a id="ejemplo-python-vuelo-simple"></a>

### [10] Ejemplo Python - Vuelo Simple

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

rospy.init_node('bebop_test_flight')

takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

time.sleep(1)

# Despegar
takeoff_pub.publish(Empty())
rospy.loginfo("Despegando...")
time.sleep(5)

# Avanzar
move = Twist()
move.linear.x = 0.2
cmd_pub.publish(move)
time.sleep(3)

# Girar a la izquierda
move = Twist()
move.angular.z = 0.3
cmd_pub.publish(move)
time.sleep(3)

cmd_pub.publish(Twist())  # Detener
land_pub.publish(Empty())
rospy.loginfo("Aterrizando...")
time.sleep(5)
```
[🔙 Volver al Índice](#indice)


---

