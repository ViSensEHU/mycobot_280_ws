"""
Script interactivo para mover libremente el myCobot 280 y registrar posiciones.

Flujo:
1. Activa modo libre (free mode) para permitir movimiento manual
2. Espera a que pulses ENTER para capturar la posición actual
3. Registra ángulos y coordenadas en cada captura
4. Escribe 'q' + ENTER para terminar y guardar posiciones
"""

import logging
import ast
import os
import sys
import time
from typing import List, Tuple

from pymycobot import MyCobot280


# Configuración
PORT = "/dev/ttyACM0"
BAUD = 115200

# Almacenamiento de posiciones capturadas
recorded_positions: List[Tuple[List[float], List[float]]] = []
DEFAULT_SPEED = 20


def configure_logging() -> None:
    """Configura logging básico."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )


def init_robot() -> MyCobot280:
    """Inicializa conexión con el robot."""
    logging.info("Conectando con myCobot280 en %s...", PORT)
    mc = MyCobot280(PORT, BAUD)
    time.sleep(0.5)
    return mc



def main() -> int:
    configure_logging()
    
    print("\n" + "="*60)
    print("  MYCOBOT 280 - MODOS DE TRABAJO")
    print("="*60)
    print("Selecciona modo:")
    print("  1) Power on motors")
    print("  2) Power off motors")
    print("="*60 + "\n")
    
    mc = None
    try:
        mode = input("Elige opción (1/2): ").strip()
        mc = init_robot()

        if mode == "2":
            logging.info("Apagando motores...")
            mc.power_off()
            logging.info("✓ Motores apagados.")
            return 0

        logging.info("Activando motores...")
        mc.power_on()
        logging.info("✓ Motores activados.")
        return 0
        
    except KeyboardInterrupt:
        logging.warning("\n⚠ Interrumpido por el usuario")
        return 0
    except Exception as e:
        logging.exception("✗ Error inesperado: %s", e)
        return 1
    finally:
        if mc:
            try:
                mc.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())