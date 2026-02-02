#!/bin/bash

# Permitir conexiones gráficas
xhost +local:docker > /dev/null

# Definir la imagen
IMAGE_NAME="visensehu/mycobot_280:v1"

# Obtener la ruta absoluta
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Variables de identidad (detectan quién eres tú fuera)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
USER_NAME=$USER

# Ejecutar el contenedor
# NOTA: Eliminamos 'sudo' antes de docker si tu usuario está en el grupo docker.
# Si no, déjalo, pero lo ideal es mapear los IDs.
docker run -it --rm \
    --name ros2_jazzy_erobotics \
    --network host \
    --privileged \
    --device /dev/dri:/dev/dri \
    --device /dev/ttyACM*:/dev/ttyACM* \
    --device /dev/ttyUSB*:/dev/ttyUSB* \
    --group-add video \
    --group-add dialout \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$DIR":/home/sistemas/erobotics_ws:rw \
    --user root \
    --entrypoint /bin/bash \
    $IMAGE_NAME -c "
        # Crear grupo y usuario dentro si no existen con tus IDs
        groupadd -g $GROUP_ID dockergroup 2>/dev/null
        useradd -u $USER_ID -g $GROUP_ID -m -s /bin/bash $USER_NAME 2>/dev/null
        # Dar permisos de sudo al usuario sin contraseña
        echo '$USER_NAME ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
        # Cambiar al usuario y abrir bash
        exec su $USER_NAME
    "
