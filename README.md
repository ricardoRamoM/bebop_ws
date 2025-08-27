Perfecto. Te propongo la **versión definitiva y profesional** de tu README para GitHub, con imágenes reales simuladas de nodos, badges, y estilo visual listo para publicación. Te daré **Markdown que incluya placeholders para imágenes** que luego puedes reemplazar con capturas de pantalla de tu simulación en RViz, `rqt_graph` y cámara del Bebop.

---

# 🚁 ROS Workspace - Control del Drone Parrot Bebop 2

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

Este repositorio contiene un **workspace ROS Noetic** para controlar un dron **Parrot Bebop 2**.
Incluye instalación, configuración, comandos básicos, y ejemplos de vuelo en Python.

---

## 📦 Estructura del Proyecto

```
bebop_ws/
 ├── build/
 ├── devel/
 └── src/
      ├── parrot_arsdk       # Wrapper SDK Parrot
      └── bebop_autonomy     # Driver principal ROS
```

---

## ✅ Requisitos

* Ubuntu 20.04 LTS
* ROS Noetic → [Instalación oficial](http://wiki.ros.org/noetic/Installation/Ubuntu)

**Dependencias ROS y Sistema:**

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-cmake-modules \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-message-filters \
    ros-noetic-camera-info-manager \
    ros-noetic-joy \
    ros-noetic-geometry-msgs \
    ros-noetic-cv-bridge \
    build-essential cmake git python3-catkin-tools \
    libavcodec-dev libavdevice-dev libavformat-dev \
    libavutil-dev libswscale-dev libeigen3-dev \
    libopencv-dev libsdl1.2-dev libusb-1.0-0-dev \
    libgles2-mesa-dev libcurl4-openssl-dev unzip \
    python3-numpy python3-opencv python3-yaml
```

---

## 🔧 Instalación desde Cero

1️⃣ Crear workspace y compilar:

```bash
mkdir -p ~/bebop_ws/src
cd ~/bebop_ws
catkin_make
```

2️⃣ Configurar entorno:

```bash
echo "source ~/bebop_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3️⃣ Clonar repositorios:

**Parrot ARSDK**

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

**Bebop Autonomy**

```bash
cd ~/bebop_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

Modificar en `bebop_driver/src/bebop_video_decoder.cpp`:

```
línea 93: CODEC_AP_TRUNCATED -> AV_CODEC_CAP_TRUNCATED
línea 95: CODEC_FLAG_TRUNCATED -> AV_CODEC_FLAG_TRUNCATED
línea 97: CODEC_FLAG2_CHUNKS -> AV_CODEC_FLAG2_CHUNKS
```

Añadir en `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/bebop_ws/devel/lib/parrot_arsdk
```

4️⃣ Compilar todo:

```bash
cd ~/bebop_ws
catkin_make -j1
source devel/setup.bash
```

---

# ▶️ Uso del Drone Parrot Bebop 2

## 1️⃣ Conexión con el dron

Conectar a la red WiFi del Bebop:

```bash
nmcli dev wifi connect "Bebop2-XXXXXX"
```

Probar conexión:

```bash
ping 192.168.42.1
```

✅ Respuesta correcta indica conexión activa.

---

## 2️⃣ Iniciar ROS

```bash
roscore
```

---

## 3️⃣ Lanzar Nodo Principal

```bash
roslaunch bebop_driver bebop_node.launch
```

---

## 4️⃣ Comandos Básicos

| Acción     | Comando Terminal                                                                                                              |
| ---------- | ----------------------------------------------------------------------------------------------------------------------------- |
| Despegar   | `rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"`                                                                      |
| Aterrizar  | `rostopic pub --once /bebop/land std_msgs/Empty "{}"`                                                                         |
| Emergencia | `rostopic pub --once /bebop/reset std_msgs/Empty "{}"`                                                                        |
| Avanzar    | `rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x:0.0, y:0.0, z:0.0}}'` |

---

## 5️⃣ Tópicos Principales

```bash
rostopic list | grep bebop
```

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

---

## 6️⃣ Visualizar Cámara

```bash
rqt_image_view /bebop/image_raw
```

![Camera Feed Placeholder](docs/images/bebop_camera.png)

---

## 7️⃣ Visualizar Nodos y Topics

```bash
rqt_graph
```

Ejemplo de diagrama de nodos Bebop:

![rqt\_graph Placeholder](docs/images/bebop_rqt_graph.png)

**Flujo de comunicación:**

```
         +-------------+
         |bebop_node   |
         +-------------+
          /     |      \
     cmd_vel  camera    state
       |       |        |
   [motores] [video] [info]
```

---

## 8️⃣ Ejemplo Python - Vuelo Simple

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

---

## 9️⃣ Flujo Básico de Vuelo

```
[ TAKEOFF ] → [ CMD_VEL (mover/girar) ] → [ LAND ]
```

---

## 10️⃣ Capturas de Ejemplo

| RViz                                            | rqt\_graph                                                 |
| ----------------------------------------------- | ---------------------------------------------------------- |
| ![RViz Placeholder](docs/images/bebop_rviz.png) | ![rqt\_graph Placeholder](docs/images/bebop_rqt_graph.png) |

---

Si quieres, puedo **generarte directamente los diagramas SVG/PNG de RViz y rqt\_graph listos para GitHub**, de modo que solo tengas que descargar y añadirlos a la carpeta `docs/images`. Esto haría tu README **completamente profesional y visual**.

¿Quieres que haga eso también?


Perfecto, aquí te propongo un **README completo y visual** para tu proyecto del Bebop 2 con ROS, usando Markdown con badges, secciones claras y diagramas visuales estilo texto (ASCII/diagramas simples), listo para GitHub. Posteriormente puedes reemplazar los diagramas con imágenes reales si lo deseas.

---
*********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
# 🚁 ROS Workspace - Control del Drone Parrot Bebop 2

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

Este repositorio contiene un **workspace ROS Noetic** con paquetes para controlar un dron **Parrot Bebop 2** mediante ROS.
Incluye instalación desde cero, configuración del entorno, comandos básicos de vuelo y ejemplos de Python.

---

## 📦 Estructura del Proyecto

```
bebop_ws/
 ├── build/
 ├── devel/
 └── src/
      ├── parrot_arsdk       # Wrapper SDK Parrot
      └── bebop_autonomy     # Driver principal ROS
```

---

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
    python3-numpy python3-opencv python3-yaml
```

---

## 🔧 Instalación desde Cero

### 1️⃣ Crear workspace y compilar

```bash
mkdir -p ~/bebop_ws/src
cd ~/bebop_ws
catkin_make
```

### 2️⃣ Configurar entorno

```bash
echo "source ~/bebop_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3️⃣ Clonar repositorios [como se indica](https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev):

**Parrot ARSDK**

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

**Bebop Autonomy**

```bash
cd ~/bebop_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

Modificar en **bebop\_driver/src/bebop\_video\_decoder.cpp**:

```
línea 93: CODEC_AP_TRUNCATED -> AV_CODEC_CAP_TRUNCATED
línea 95: CODEC_FLAG_TRUNCATED -> AV_CODEC_FLAG_TRUNCATED
línea 97: CODEC_FLAG2_CHUNKS -> AV_CODEC_FLAG2_CHUNKS
```

Añadir a `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/bebop_ws/devel/lib/parrot_arsdk
```

### 4️⃣ Compilar todo

```bash
cd ~/bebop_ws
catkin_make -j1
source devel/setup.bash
```

---

# ▶️ Uso del Drone Parrot Bebop 2

## 1️⃣ Conexión con el dron

Para controlar el Bebop 2, primero debes conectarte a su red WiFi y verificar la conexión.

### 🔹 Conectar a la red WiFi del Bebop

Conéctate desde la **configuración de tu sistema** o usando la siguiente línea de comando:

```bash
nmcli dev wifi connect "Bebop2-XXXXXX"
```

> Reemplaza `"Bebop2-XXXXXX"` con el nombre exacto de la red de tu dron.

### 🔹 Verificar conexión con `ping`

```bash
ping 192.168.42.1
```

Al probar la conexión, pueden ocurrir dos casos:

**❌ Respuesta incorrecta (sin conexión activa):**

```bash
PING 192.168.42.1 (192.168.42.1) 56(84) bytes of data.
From 192.168.42.22 icmp_seq=1 Destination Host Unreachable
^C
--- 192.168.42.1 ping statistics ---
3 packets transmitted, 0 received, +1 errors, 100% packet loss, time 2033ms
```

**✅ Respuesta correcta (conexión activa):**

```bash
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
Si ves el ejemplo ✅, la conexión con el dron está activa y puedes continuar con los comandos de ROS.
Si aparece el ejemplo ❌, revisa la conexión WiFi, la dirección IP y asegúrate de que el dron esté encendido.

---

## 2️⃣ Iniciar ROS

Antes de ejecutar cualquier nodo o comando, debes iniciar el **roscore**, que es el núcleo de ROS.
`roscore` es un servicio que permite que todos los nodos y tópicos de ROS se comuniquen entre sí.

```bash
roscore
```

> Debe mantenerse ejecutando en una terminal mientras usas ROS.

---

## 3️⃣ Lanzar el Nodo Principal

El nodo principal del Bebop (`bebop_node`) controla la comunicación con el dron, recibe datos de sensores y envía comandos de vuelo.
Para iniciarlo:

```bash
roslaunch bebop_driver bebop_node.launch
```

> Este comando se ejecuta en una nueva terminal con `setup.bash` cargado.
> Una vez lanzado, el dron estará listo para recibir comandos y enviar datos a ROS.

---

## 4️⃣ Comandos Básicos

Esta sección te permite **controlar el dron desde la terminal** mediante `rostopic pub`, publicando mensajes en los tópicos correspondientes.

> ⚠️ **Precaución:** Antes de ejecutar cualquier comando, asegúrate de tener suficiente espacio libre alrededor del dron y que no haya obstáculos. Cada comando debe ejecutarse en una nueva terminal con el `setup.bash` cargado, mientras `roscore` y el nodo principal están corriendo.

---


### 🔹 Diferencia entre `--once` y `-r <rate>`

> 🟢 `--once` → Movimiento **instantáneo**, solo un impulso breve.
> 🔵 `-r 10` → Movimiento **continuo**, se repite 10 veces por segundo hasta detenerlo (Ctrl+C o Detener movimiento).

---

### 🔹 Despegar y aterrizar

| Acción    | Comando                                                  | Explicación                                                   |
| --------- | -------------------------------------------------------- | ------------------------------------------------------------- |
| Despegar  | `rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"` | El dron despega y se mantiene flotando a baja altura (\~1 m). |
| Aterrizar | `rostopic pub --once /bebop/land std_msgs/Empty "{}"`    | El dron desciende suavemente hasta tocar el suelo.            |

---

### 🔹 Movimientos Básicos del Bebop

#### 1️⃣ Avanzar / Retroceder

* **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.2, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

Avanza solo un instante (\~unos centímetros).

* **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.2, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

Avanza continuamente a 0.2 m/s hasta que presiones Ctrl+C o publiques **Detener movimiento**.

* Retroceder: cambia `x` a negativo (`x:-0.2`).

---

#### 2️⃣ Giros (izquierda / derecha)

* **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.3}}'
```

* **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.3}}'
```

* Girar a la derecha: cambia `z` a negativo (`z:-0.3`).

---

#### 3️⃣ Subir / Bajar

* **🟢 Instantáneo:**

```bash
# Subir
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.2}, angular: {x:0.0, y:0.0, z:0.0}}'

# Bajar
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:-0.2}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* **🔵 Continuo:**

```bash
# Subir
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.2}, angular: {x:0.0, y:0.0, z:0.0}}'

# Bajar
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:-0.2}, angular: {x:0.0, y:0.0, z:0.0}}'
```

---

#### 4️⃣ Movimiento lateral (izquierda / derecha)

* **🟢 Instantáneo:**

```bash
# Derecha
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'

# Izquierda
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:-0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* **🔵 Continuo:**

```bash
# Derecha
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'

# Izquierda
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:-0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

---

### 🔹 Detener o emergencia

* **Detener movimiento:** Frenar inmediatamente cualquier movimiento continuo:

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* **Emergencia:** Apaga los motores de inmediato y reinicia los sistemas del dron.

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

---


## 5️⃣ Verificar Tópicos Disponibles

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

---

## 6️⃣ Ver la Cámara

Permite **visualizar la cámara delantera del dron en tiempo real**.
Se usa `rqt_image_view` para abrir una ventana donde se muestra el video:

```bash
rqt_image_view /bebop/image_raw
```

> Esto es útil para inspeccionar el entorno o hacer pruebas de visión por computadora.

---

## 7️⃣ Visualizar Nodos y Tópicos (`rqt_graph`)

```bash
rqt_graph
```

Esto muestra un **diagrama interactivo** de todos los nodos y cómo se comunican mediante topics.
Ejemplo de flujo básico en Bebop:

```
         +-------------+
         |bebop_node   |
         +-------------+
          /     |      \
     cmd_vel  camera    state
       |       |        |
   [motores] [video] [info]
```
## 7️⃣ Visualizar Nodos y Tópicos (rqt\_graph)

`rqt_graph` muestra un **diagrama visual de los nodos y sus conexiones** en ROS.
Esto te ayuda a entender cómo se comunican los distintos componentes del dron, por ejemplo:

* Qué nodo envía comandos a los motores (`cmd_vel`)
* Qué nodo publica las imágenes de la cámara
* Qué nodo informa el estado de la batería o la odometría

```bash
rqt_graph
```

> Ideal para depurar problemas o entender la arquitectura de ROS si eres nuevo en el sistema.

---

## 8️⃣ Ejemplo Python - Vuelo Simple

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

---

## 9️⃣ Diagrama Básico del Flujo de Vuelo

```
[ TAKEOFF ] → [ CMD_VEL (mover/girar) ] → [ LAND ]
```

---

