# 1. Usamos la base oficial de ROS Jazzy Desktop que ya tienes
FROM osrf/ros:jazzy-desktop-full

# 2. Evitar preguntas interactivas durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# 3. Crear el usuario 'sistemas' con el mismo ID que tu Ubuntu (1000)
# Esto es lo que evita los problemas de permisos al hacer colcon build
ARG USERNAME=sistemas
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN usermod -l sistemas ubuntu && \
    groupmod -n sistemas ubuntu && \
    usermod -d /home/sistemas -m sistemas && \
    apt-get update && apt-get install -y sudo && \
    echo "sistemas ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/sistemas && \
    chmod 0440 /etc/sudoers.d/sistemas

# 4. Instalar herramientas y GZ_ROS2_CONTROL
RUN apt-get update && apt-get install -y \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-tf-transformations \
    python3-transforms3d \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-setup-assistant \
    python3-lark \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 5. Configurar el entorno de trabajo
USER $USERNAME
WORKDIR /home/$USERNAME/erobotics_ws

# 6. Cargar ROS automáticamente para el usuario
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/sistemas/.bashrc && \
    echo 'if [ -f "/home/sistemas/erobotics_ws/install/setup.bash" ]; then source "/home/sistemas/erobotics_ws/install/setup.bash"; fi' >> /home/sistemas/.bashrc

RUN echo 'export QT_QPA_PLATFORM=xcb' >> /home/sistemas/.bashrc

CMD ["bash"]

