# ROS Workspace - Drone Bebop Parrot 2 Control

If you use this repository, you will be able to fly the Drone Bebop Parrot 2 with ROS Noetic.

Este repositorio contiene un workspace de **ROS Noetic** con paquetes para controlar un dron Bebop Parrot 2 desde ROS Noetic.

---

## 📂 Estructura del proyecto

## ✅ Requisitos

- Ubuntu 20.04 LTS
- ROS Noetic instalado ([Instrucciones oficiales](http://wiki.ros.org/noetic/Installation/Ubuntu))
- Dependencias ROS. Primero asegúrate de tener las de ROS:

        sudo apt-get update
        sudo apt-get install -y \
            ros-$ROS_DISTRO-cmake-modules \
            ros-$ROS_DISTRO-image-transport \
            ros-$ROS_DISTRO-diagnostic-updater \
            ros-$ROS_DISTRO-tf \
            ros-$ROS_DISTRO-tf2-ros \
            ros-$ROS_DISTRO-message-filters \
            ros-$ROS_DISTRO-camera-info-manager

- Librerías del sistema (Parrot SDK + multimedia)

        sudo apt-get install -y \
            build-essential \
            cmake \
            git \
            python3-catkin-tools \
            libavcodec-dev \
            libavdevice-dev \
            libavformat-dev \
            libavutil-dev \
            libswscale-dev \
            libeigen3-dev \
            libopencv-dev \
            libsdl1.2-dev \
            libusb-1.0-0-dev \
            libgles2-mesa-dev \
            libcurl4-openssl-dev \
            unzip


- Dependencies extras:

        sudo apt-get install ros-noetic-joy ros-noetic-geometry-msgs ros-noetic-tf ros-noetic-cv-bridge
        sudo apt-get install libavcodec-dev libswscale-dev libavutil-dev
        sudo apt-get install -y python3-numpy python3-opencv python3-yaml


## 🔧 Instalación desde cero

1. Crear catkin workspace. Crear carpeta "bebop_ws" y dentro la carpeta "src". Y en la primera hacer catkin_make.

        mkdir bebop_ws
        cd bebop_ws
        mkdir src
        catkin_make

2. Hacer Sourcing. Entrar a carpeta devel y ejecutar

        cd devel
        source setup.bashrc 

- O para automatizarlo ejecuta la siguiente linea 

        echo "source ~/bebop_ws/devel/setup.bash" >> ~/.bashrc

3. Clonar 2 carpetas "bebop autonomy" (drone driver)  y "parrot_arsdk" wrapper, siguiendo las instrucciones de: https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev que son las mismas mostradas a continuación:

Parrot_arsdk

	cd <path_to_your_catkin_ws>/src
	git clone https://github.com/antonellabarisic/parrot_arsdk.git
	cd parrot_arsdk
	git checkout noetic_dev
	sudo apt-get install libavahi-client-dev
	sudo ln -s /usr/bin/python3 /usr/bin/python
	cd <path_to_your_catkin_ws>
	catkin_make

Bebop autonomy

	cd <path_to_your_catkin_ws>/src
	git clone https://github.com/AutonomyLab/bebop_autonomy.git

Modifica el archivo /bebop_driver/src/bebop_video_decoder.cpp

    linea 93: CODEC_AP_TRUNCATED -> AV_CODEC_CAP_TRUNCATED
    linea 95: CODEC_FLAG_TRUNCATED -> AV_CODEC_FLAG_TRUNCATED
    linea 97: CODEC_FLAG2_CHUNKS -> AV_CODEC_FLAG2_CHUNKS

Añade esta línea en tu ~/.bashrc :

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path_to_your_catkin_ws>/devel/lib/parrot_arsdk

Build:

	cd ..
	catkin_make -j1

No olvides actualizar tu entorno:

	source <path_to_your_catkin_ws>/devel/setup.bash 	



---

# Uso del dron **Parrot Bebop 2** con ROS Noetic

## 1. Verificar conexión con el dron

Para asegurarte de que tu computadora está conectada al Bebop 2, primero debes conectarte a su red WiFi:

```bash
# Lista de redes disponibles
nmcli dev wifi list

# Conectarse a la red del Bebop (normalmente "Bebop2-XXXXXX")
nmcli dev wifi connect "Bebop2-XXXXXX"
```

Después de conectarte, prueba el **ping** al dron (por defecto suele estar en la IP `192.168.42.1`):

```bash
ping 192.168.42.1
```

Si recibes respuesta, la conexión está activa.

---

## 2. Lanzar el nodo principal en ROS

Una vez conectado, inicia el nodo del dron con el **bebop\_autonomy**:

```bash
roslaunch bebop_driver bebop_node.launch
```

Esto levantará el nodo principal y empezará a publicar y escuchar tópicos relacionados con el dron.

---

## 3. Comandos básicos desde terminal

Con el nodo corriendo, puedes enviar comandos básicos al dron usando `rostopic pub`.
Ejemplos:

* **Despegar:**

```bash
rostopic pub --once /bebop/takeoff std_msgs/Empty
```

* **Aterrizar:**

```bash
rostopic pub --once /bebop/land std_msgs/Empty
```

* **Emergencia (motores off inmediato):**

```bash
rostopic pub --once /bebop/reset std_msgs/Empty
```

* **Moverse (usando Twist):**

```bash
rostopic pub /bebop/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

*(Este ejemplo hace que avance hacia adelante lentamente).*

---

## 4. Ver otros tópicos disponibles

Para listar todos los tópicos que el dron publica/escucha:

```bash
rostopic list
```

Algunos de los más comunes son:

* `/bebop/image_raw` → Cámara delantera.
* `/bebop/odom` → Odometry del dron.
* `/bebop/cmd_vel` → Comandos de velocidad.
* `/bebop/takeoff`, `/bebop/land`, `/bebop/reset`.

Si quieres inspeccionar mensajes específicos:

```bash
rostopic info /bebop/cmd_vel
rostopic echo /bebop/odom
```

---

## 5. Tips adicionales

* Siempre inicia **`roscore`** en otra terminal antes de lanzar el nodo.
* Revisa que tu red esté estable antes de volar.
* Usa `rqt_graph` para visualizar gráficamente todos los nodos y tópicos.

---





Claro, aquí tienes todo listo en **texto plano Markdown** para que lo copies directamente a tu README.md:


# 🚁 Conexión y Comandos Básicos del Bebop con ROS

Este apartado explica cómo **verificar la conexión con el dron**, lanzar el nodo principal de ROS y ejecutar comandos básicos desde la terminal.

---

## 🔹 1. Verificar conexión con el dron

1. **Encender el Bebop** y conectarse a su red WiFi desde la computadora (`BebopDrone-XXXX`).  
2. Comprobar conexión con **ping**:

```bash
ping 192.168.42.1
````

✅ Si hay respuesta, la conexión es correcta.

---

## 🔹 2. Lanzar el nodo principal

En una terminal con el `setup.bash` cargado:

```bash
roslaunch bebop_driver bebop_node.launch
```

Deberías ver algo similar a:

```
[ INFO] [xxxx]: Connected to Bebop ...
[ INFO] [xxxx]: Camera calibration ...
```

---

## 🔹 3. Verificar los topics disponibles

En otra terminal:

```bash
rostopic list | grep bebop
```

**Principales topics:**

```
/bebop/cmd_vel                  # Comandos de movimiento (Twist)
/bebop/takeoff                  # Despegar
/bebop/land                     # Aterrizar
/bebop/reset                    # Emergencia
/bebop/image_raw                # Video de la cámara
/bebop/camera_info              # Info de calibración de cámara
/bebop/odom                     # Odometría
/bebop/imu/data                 # Datos IMU
/bebop/states/common/CommonState/BatteryStateChanged  # Estado de batería
```

Para ver más información de un topic:

```bash
rostopic info /bebop/cmd_vel
```

---

## 🔹 4. Comandos básicos desde la terminal

### ✅ Despegar

```bash
rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"
```

### ✅ Aterrizar

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

### ✅ Emergencia (corte inmediato de motores)

```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```

### ✅ Movimiento básico (adelante)

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### ✅ Giro (izquierda)

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
```

---

## 🔹 5. Comprobar la cámara

```bash
rqt_image_view /bebop/image_raw
```

---

## 🔹 6. Ver estado de batería

```bash
rostopic echo /bebop/states/common/CommonState/BatteryStateChanged
```

---

## 🔹 7. Consultar más comandos disponibles

```bash
rostopic list | grep bebop
```

y luego inspeccionar cada uno con:

```bash
rostopic info <nombre_del_topic>
```

---

## 🔹 8. Script de ejemplo en Python

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

rospy.init_node('bebop_test_flight')

# Publishers
takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

time.sleep(1)

# Despegar
takeoff_pub.publish(Empty())
rospy.loginfo("Despegando...")
time.sleep(5)

# Mover hacia adelante
move = Twist()
move.linear.x = 0.2
cmd_pub.publish(move)
rospy.loginfo("Moviendo hacia adelante...")
time.sleep(3)

# Girar a la izquierda
move = Twist()
move.angular.z = 0.3
cmd_pub.publish(move)
rospy.loginfo("Girando a la izquierda...")
time.sleep(3)

# Detener movimiento
cmd_pub.publish(Twist())

# Aterrizar
land_pub.publish(Empty())
rospy.loginfo("Aterrizando...")
time.sleep(5)

rospy.loginfo("Vuelo de prueba finalizado")
```

---

## 🔹 9. Diagrama de flujo básico

```
+---------+       +-------------+       +--------+
| Despegar| --->  | Cmd_vel     | --->  | Land   |
+---------+       | (mover/girar)|       +--------+
                  +-------------+
```

Este flujo muestra la secuencia de **vuelo de prueba**:

1. Despegar
2. Moverse y/o girar
3. Aterrizar

---

**Título sugerido para este apartado:**

```
# Conexión y Comandos Básicos del Bebop con ROS
```
