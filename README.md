# 🤖 Erobotics Workspace

Control del MyCobot 280 con ROS 2

Este repositorio contiene un workspace de ROS 2 Jazzy configurado para controlar el brazo robótico **MyCobot 280** (versión M5) de Elephant Robotics.

El proyecto integra **MoveIt 2** para la planificación de movimientos y un **Driver personalizado en Python** que comunica ROS 2 con el hardware real mediante la librería `pymycobot`aunque de momento da problemas de saturación de comandos en el puerto serie.

## Tabla de Contenidos

- [Sobre el Robot](#sobre-el-robot)
- [Requisitos](#requisitos)
- [Instalación con Docker](#instalación-con-docker)
- [Ejecución](#ejecución)
- [Limitaciones Actuales](#limitaciones-actuales)
- [Estructura del Repositorio](#estructura-del-repositorio)
- [Recursos](#recursos)

---

## Sobre el Robot

Este proyecto está diseñado para el **MyCobot 280**, un brazo robótico colaborativo de **6 grados de libertad (DoF)**.

| Propiedad | Valor |
|-----------|-------|
| Fabricante | Elephant Robotics |
| Modelo | MyCobot 280 M5/Arduino |
| DoF | 6 |
| Repositorio Oficial | [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) |
| Documentación | [Gitbook MyCobot](https://docs.elephantrobotics.com/docs/mycobot280/) |

---

## Requisitos

### Hardware
- **Ordenador anfitrión** con Linux
- **Robot MyCobot 280** conectado por USB (típicamente `/dev/ttyUSB0` o `/dev/ttyACM0`)
- Permisos de acceso a puertos USB

### Software
- **Docker** instalado y configurado
- Permisos de ejecución (o acceso a sudo)

---

## Instalación con Docker

> ℹ️ Se recomienda usar Docker para garantizar que todas las dependencias de ROS 2 (Jazzy) y librerías de Python estén correctamente instaladas sin afectar tu sistema anfitrión.

### Paso 1: Construir la Imagen

```bash
docker build -t <image_name> .
```

### Paso 2: Ejecutar el Contenedor

Se incluye un script de utilidad `run.sh` que configura automáticamente:
- Permisos gráficos (para visualizar RViz)
- Montaje de dispositivos USB y ACM
- Variables de entorno necesarias

```bash
# Dar permisos de ejecución (primera ejecución)
chmod +x run.sh

# Ejecutar el contenedor
./run.sh
```

Te encontrarás en una terminal dentro del contenedor lista para ejecutar comandos de ROS 2. El source se hace automáticamente

---

## Ejecución

El sistema se divide en **dos componentes** que deben ejecutarse en terminales separadas (dentro del Docker):

1. **Driver**: Comunica con el hardware vía puerto serie
2. **MoveIt + RViz**: Planifica movimientos y visualización

### Terminal 1: Lanzar el Driver de Hardware

```bash
ros2 launch erobotics_driver driver.launch.py
```

**Descripción:**
- Conecta con el robot vía puerto serie
- Expone la interfaz `FollowJointTrajectory`

**Éxito esperado:** Log mostrando puerto detectado y mensaje "Driver Listo"

### Terminal 2: Lanzar MoveIt y RViz

```bash
ros2 launch erobotics_moveit demo.launch.py
```

**Uso en RViz:**

1. **Configurar destino:** Arrastra la esfera azul ("Target") a la posición deseada del efector final
2. **Planificar:** Haz clic en **"Plan"** (MoveIt calcula la trayectoria)
3. **Ejecutar:** Haz clic en **"Execute"** (el robot real se mueve)

---

## Limitaciones Actuales

### Problema

El microcontrolador del MyCobot 280 tiene **limitaciones de ancho de banda** en la comunicación serie. El robot no puede procesar trayectorias densas (cientos de puntos) a alta velocidad sin:
- Saturarse
- Moverse con saltos discontinuos

### Solución Implementada: "Modo Directo"

El controlador `erobotics_interface.py` implementa una estrategia **Punto a Punto**:

```
Trayectoria completa (MoveIt)
         ↓
  Ignora puntos intermedios
         ↓
  Extrae punto final (Meta)
         ↓
  Envía comando único al robot
```

**Ventajas:**
- ✅ **Movimiento fluido:** El robot gestiona su propia aceleración interna
- ✅ **Linealidad en espacio de juntas:** Todos los motores se mueven simultáneamente

**Limitaciones:**
- ⚠️ **No sigue trayectoria planificada:** Mueve todos los motores a la vez (no sigue curva de MoveIt)
- ⚠️ **No evita obstáculos en trayecto:** Solo planifica el punto inicial y final
- ✅ **Ideal para:** Aplicaciones simples de "Pick and Place" sin obstáculos intermedios

### Casos de Uso Recomendados

| Aplicación | Recomendado | Motivo |
|-----------|:---:|--------|
| Pick & Place simple | ✅ | Trayectoria A → B directa |
| Tareas con obstáculos | ❌ | Ignora curva planificada |
| Trayectorias complejas | ❌ | Satura el puerto serie |

---

## Estructura del Repositorio

```
erobotics_ws/
├── src/
│   ├── erobotics_driver/          # Driver personalizado Python ↔ ROS 2
│   │   └── erobotics_interface.py # Puente con pymycobot
│   ├── erobotics_description/     # URDF/Xacro + mallas 3D
│   ├── erobotics_moveit/          # Configuración MoveIt Setup Assistant
│   └── erobotics_controller/      # Configuraciones de controladores ROS 2
├── build/                         # Artefactos compilados (colcon)
├── install/                       # Archivos instalados
├── log/                           # Logs de construcción
├── Dockerfile                     # Definición de imagen Docker
├── run.sh                         # Script de ejecución Docker
└── README.md                      # Este archivo
```

### Paquetes Principales

| Paquete | Descripción |
|---------|-------------|
| `erobotics_driver` | Script Python que actúa como puente entre ROS 2 y `pymycobot` |
| `erobotics_description` | Archivos URDF/Xacro y mallas (meshes) 3D del robot |
| `erobotics_moveit` | Configuración generada por MoveIt Setup Assistant |
| `erobotics_controller` | Configuraciones adicionales de controladores ROS 2 |

---

## Recursos

### Enlaces Útiles

- 📚 [Documentación oficial MyCobot](https://docs.elephantrobotics.com/docs/mycobot280/)
- 🔗 [Repositorio mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)
- 📦 [pymycobot en PyPI](https://pypi.org/project/pymycobot/)
- 🎯 [MoveIt 2 Documentation](https://moveit.ros.org/documentation/getting_started/)

### Troubleshooting

**El robot no se conecta:**
- Verifica que el dispositivo USB está en `/dev/ttyUSB0` o `/dev/ttyACM0`
- Comprueba permisos: `ls -la /dev/ttyUSB0`
- Si usas Docker, verifica que `run.sh` monta el dispositivo correctamente

**RViz no aparece:**
- Asegúrate de que ejecutaste `./run.sh` (configura permisos gráficos)
- Verifica variables de entorno: `echo $DISPLAY`

---
