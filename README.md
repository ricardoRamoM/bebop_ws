Perfecto. Te propongo la **versión definitiva y profesional** de tu README para GitHub, con imágenes reales simuladas de nodos, badges, y estilo visual listo para publicación. Te daré **Markdown que incluya placeholders para imágenes** que luego puedes reemplazar con capturas de pantalla de tu simulación en RViz, `rqt_graph` y cámara del Bebop.

---

# 🚁 ROS Workspace - Control del Drone Parrot Bebop 2

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Este repositorio contiene un **workspace ROS Noetic** para controlar un dron **Parrot Bebop 2**.
Incluye instalación, configuración, comandos básicos, y ejemplos de vuelo en Python.

---

## 📦 Estructura del Proyecto

```
bebop_ws/
 ├── src/
 │    ├── parrot_arsdk       # Wrapper SDK Parrot
 │    └── bebop_autonomy     # Driver principal ROS
 ├── devel/
 └── build/
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
 ├── src/
 │    ├── parrot_arsdk       # Wrapper SDK Parrot
 │    └── bebop_autonomy     # Driver principal ROS
 ├── devel/
 └── build/
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

### 3️⃣ Clonar repositorios

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

Abrir una terminal:

```bash
roscore
```

---

## 3️⃣ Lanzar el Nodo Principal

En otra terminal:

```bash
roslaunch bebop_driver bebop_node.launch
```

---

## 4️⃣ Comandos Básicos

**Despegar**

```bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

**Aterrizar**

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

**Emergencia**

```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```

**Mover adelante**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

---

## 5️⃣ Verificar Tópicos Disponibles

```bash
rostopic list | grep bebop
```

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

```bash
rqt_image_view /bebop/image_raw
```

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

Si quieres, puedo **hacer otra versión con imágenes reales de nodos en RViz/rqt\_graph y badges de GitHub más avanzados** que se vea **profesional y lista para README público**.

¿Quieres que haga esa versión con imágenes reales y estilo GitHub?
