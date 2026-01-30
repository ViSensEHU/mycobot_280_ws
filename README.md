Erobotics Workspace - Control de MyCobot 280 con ROS 2

Este repositorio contiene un espacio de trabajo (workspace) de ROS 2 configurado para controlar el brazo robótico MyCobot 280 (versión M5/Arduino) de Elephant Robotics.

El proyecto integra MoveIt 2 para la planificación de movimientos y un Driver personalizado en Python que comunica ROS 2 con el hardware real mediante la librería pymycobot, solucionando problemas de saturación de comandos en el puerto serie.

🤖 Sobre el Robot

Este proyecto está diseñado para el MyCobot 280, un brazo robótico colaborativo de 6 grados de libertad.

Fabricante: Elephant Robotics

Repositorio Oficial: mycobot_ros2

Documentación Oficial: Gitbook MyCobot

🐳 Ejecución con Docker (Recomendado)

Este proyecto está diseñado para ejecutarse dentro de un contenedor Docker para garantizar que todas las dependencias de ROS 2 (Jazzy/Humble) y las librerías de Python estén correctamente instaladas sin afectar tu sistema anfitrión.

Prerrequisitos

Docker instalado.

El robot conectado por USB al ordenador (generalmente en /dev/ttyUSB0 o /dev/ttyACM0).

Permisos para acceder a los puertos USB (o ejecutar docker con sudo).

1. Construir la Imagen

Para crear la imagen de Docker con todas las dependencias:

docker build -t erobotics_image .


2. Ejecutar el Contenedor

Hemos incluido un script de utilidad run.sh que configura los permisos gráficos (para ver RViz) y monta los dispositivos USB automáticamente.

# Dar permisos de ejecución si es la primera vez
chmod +x run.sh

# Ejecutar el contenedor
./run.sh


Esto te dejará en una terminal dentro del contenedor lista para ejecutar los comandos de ROS 2.

🚀 Cómo Ejecutar el Robot

El sistema se divide en dos partes: el Driver (que habla con el hardware) y MoveIt (que planifica los movimientos). Debes ejecutar cada uno en una terminal diferente (dentro del docker).

Paso 1: Lanza el Driver de Hardware

Este nodo conecta con el robot vía puerto serie y expone la interfaz FollowJointTrajectory.

ros2 launch erobotics_driver driver.launch.py


Si la conexión es exitosa, verás un log indicando el puerto detectado y "Driver Listo".

Paso 2: Lanza MoveIt y RViz

En otra terminal, lanza la interfaz de planificación y visualización:

ros2 launch erobotics_moveit demo.launch.py


Una vez abierto RViz:

Mueve el "Target" (bola azul) del efector final.

Pulsa Plan & Execute.

El robot real debería moverse a la posición deseada.

⚠️ Limitaciones Actuales y Funcionamiento

Debido a limitaciones en el ancho de banda del microcontrolador del MyCobot 280 (comunicación serial), el robot no puede procesar trayectorias densas de cientos de puntos a alta velocidad sin saturarse o moverse a saltos.

Solución Implementada: "Modo Directo"

El controlador erobotics_interface.py implementa una estrategia de Punto a Punto:

Recibe la trayectoria completa calculada por MoveIt (que evita colisiones).

Ignora los puntos intermedios de la trayectoria.

Extrae el Punto Final (Meta).

Envía un único comando al robot para ir a esa meta.

Consecuencias

Movimiento Fluido: El robot se mueve suavemente gestionando su propia aceleración interna.

Trayectoria Lineal en Espacio de Juntas: El robot irá del punto A al B moviendo todos los motores a la vez. NO seguirá una línea recta en el espacio cartesiano ni esquivará obstáculos complejos durante el trayecto, ya que ignora la curva planificada por MoveIt.

Uso: Ideal para aplicaciones de "Pick and Place" sencillas donde no hay obstáculos entre el punto de inicio y el final.

📂 Estructura del Repositorio

erobotics_driver: Paquete con el script de Python (erobotics_interface.py) que actúa como puente entre ROS 2 y pymycobot.

erobotics_description: Contiene los archivos URDF/Xacro y las mallas (meshes) 3D del robot.

erobotics_moveit: Configuración generada por el MoveIt Setup Assistant para la planificación.

erobotics_controller: Configuraciones adicionales de controladores ROS 2.