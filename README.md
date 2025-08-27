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


Dependencies extras:

    sudo apt-get install ros-noetic-joy ros-noetic-geometry-msgs ros-noetic-tf ros-noetic-cv-bridge
    sudo apt-get install libavcodec-dev libswscale-dev libavutil-dev


## 🔧 Instalación descargando el repositorio

Clonar el repositorio:

    git clone https://github.com/ricardoRamoM/bebop_ws.git
    cd bebop_ws

Compilar el workspace:

    catkin_make

Hacer el sourcing

    echo "source <path_to_your_catkin_ws>/devel/setup.bash" >> ~/.bashrc


## Instalación desde cero

1. Crear catkin workspace. Crear carpeta "bebop_ws" y dentro la carpeta "src". Y en la primera hacer catkin_make.

        mkdir bebop_ws
        cd bebop_ws
        mkdir src
        catkin_make

2. Hacer Sourcing. Entrar a carpeta devel y ejecutar

        cd devel
        source setup.bashrc 

O para automatizarlo ejecuta la siguiente linea 

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



