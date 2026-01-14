#!/bin/bash

# Permitir conexiones gráficas
xhost +local:docker

# Definir la imagen
IMAGE_NAME="nhcar/erobotics:v1"

# Obtener la ruta absoluta de la carpeta donde ESTÁ el script
# Esto evita que el volumen falle si lanzas el script desde otra carpeta
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ejecutar el contenedor
sudo docker run -it --rm \
    --name ros2_jazzy \
    --network host \
    --privileged \
    --device /dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-sistemas" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$DIR":/home/sistemas/erobotics_ws:rw \
    --hostname sistemas \
    $IMAGE_NAME
