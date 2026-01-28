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


def enable_free_mode(mc: MyCobot280) -> bool:
    """Activa el modo libre para permitir movimiento manual."""
    try:
        logging.info("Paso 1: Apagando servos...")
        
        # Apaga la energía de todos los servos (esto es CRÍTICO para myCobot280)
        mc.power_off()
        time.sleep(0.5)
        
        logging.info("Paso 2: Liberando servos...")
        # Libera todos los servos
        mc.release_all_servos()
        time.sleep(0.5)
        
        logging.info("Paso 3: Activando modo libre...")
        # Activa el modo libre
        result = mc.set_free_mode(1)
        time.sleep(0.3)
        
        # Verifica el estado
        is_free = mc.is_free_mode()
        logging.info("Estado de modo libre: %s (esperado: 1)", is_free)
        
        if is_free == 1:
            logging.info("✓ Modo libre ACTIVADO - Intenta mover el brazo ahora")
            logging.info("   Si aún está rígido, verifica:")
            logging.info("   - LEDs del robot (deben estar apagados o en estado inactivo)")
            logging.info("   - Cable de alimentación conectado correctamente")
            return True
        else:
            logging.warning("⚠ Modo libre reportado como: %s", is_free)
            logging.warning("   Continuando de todos modos - intenta mover el brazo")
            # Aún así permitimos continuar porque a veces el estado no se reporta correctamente
            return True
    except Exception as e:
        logging.error("Error al activar modo libre: %s", e)
        logging.error("Detalles completos:", exc_info=True)
        return False


def capture_position(mc: MyCobot280, index: int) -> None:
    """Captura y registra la posición actual del robot."""
    try:
        # IMPORTANTE: Antes de capturar, re-energizamos momentáneamente
        logging.info("Energizando servos para lectura...")
        mc.power_on()
        time.sleep(0.3)
        
        angles = mc.get_angles()
        coords = mc.get_coords()
        
        # Volvemos a liberar
        mc.power_off()
        mc.release_all_servos()
        time.sleep(0.2)
        
        if not angles or not coords:
            logging.error("✗ No se pudo obtener la posición actual")
            return
        
        recorded_positions.append((angles, coords))
        
        logging.info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        logging.info("📍 Posición #%d CAPTURADA", index)
        logging.info("   Ángulos (grados): %s", 
                    ", ".join(f"J{i+1}={a:.2f}°" for i, a in enumerate(angles)))
        logging.info("   Coords (mm/°):    X=%.2f, Y=%.2f, Z=%.2f, Rx=%.2f, Ry=%.2f, Rz=%.2f",
                    coords[0], coords[1], coords[2], coords[3], coords[4], coords[5])
        logging.info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        logging.info("Brazo liberado nuevamente - puedes continuar moviéndolo")
        
    except Exception as e:
        logging.error("✗ Error al capturar posición: %s", e)


def save_positions_to_file() -> None:
    """Guarda las posiciones registradas en un archivo."""
    if not recorded_positions:
        logging.info("No hay posiciones para guardar")
        return
    
    filename = f"positions_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    try:
        with open(filename, "w") as f:
            f.write("# Posiciones capturadas del myCobot 280\n")
            f.write(f"# Fecha: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# Total: {len(recorded_positions)} posiciones\n\n")
            
            for i, (angles, coords) in enumerate(recorded_positions, 1):
                f.write(f"# Posición {i}\n")
                f.write(f"angles_{i} = {angles}\n")
                f.write(f"coords_{i} = {coords}\n\n")
        
        logging.info("✓ Posiciones guardadas en: %s", filename)
    except Exception as e:
        logging.error("✗ Error al guardar archivo: %s", e)


def list_saved_position_files() -> List[str]:
    """Lista archivos de posiciones guardados en el directorio actual."""
    files = [f for f in os.listdir(".") if f.startswith("positions_") and f.endswith(".txt")]
    files.sort()
    return files


def load_positions_from_file(filename: str) -> List[Tuple[List[float], List[float]]]:
    """Carga posiciones desde un archivo generado por este script."""
    positions: List[Tuple[List[float], List[float]]] = []
    try:
        with open(filename, "r") as f:
            lines = f.readlines()
        angles: List[float] = []
        coords: List[float] = []
        for line in lines:
            line = line.strip()
            if line.startswith("angles_"):
                # Parseo seguro para listas simples
                angles = ast.literal_eval(line.split("=", 1)[1].strip())
            elif line.startswith("coords_"):
                coords = ast.literal_eval(line.split("=", 1)[1].strip())
                if angles and coords:
                    positions.append((angles, coords))
                    angles, coords = [], []
        return positions
    except Exception as e:
        logging.error("✗ Error al leer archivo %s: %s", filename, e)
        return []


def replay_positions(mc: MyCobot280, positions: List[Tuple[List[float], List[float]]]) -> None:
    """Reproduce posiciones guardadas una a una."""
    if not positions:
        logging.error("No hay posiciones para reproducir")
        return
	
    logging.info("Reproducción iniciada. Total posiciones: %d", len(positions))
    logging.info("Usa ENTER para avanzar, 'q' para salir")
    mc.power_on()
    

    for idx, (angles, _coords) in enumerate(positions, 1):
        user_input = input(f"Ir a posición {idx}/{len(positions)} (ENTER = mover, q = salir): ").strip().lower()
        if user_input == "q":
            logging.info("Reproducción cancelada por el usuario")
            break

        try:
            # Energiza servos para mover
            mc.power_on()
            time.sleep(0.2)
            mc.send_angles(angles, DEFAULT_SPEED)
            logging.info("Moviendo a posición #%d", idx)
            # Espera a que llegue
            mc.sync_send_angles(angles, DEFAULT_SPEED, timeout=20)
        except Exception as e:
            logging.error("✗ Error moviendo a la posición #%d: %s", idx, e)
            break
    user_input = input(f"Desactivar servos y acabar. Enter - SI; q - NO").strip().lower()
    if user_input != "q":
        mc.power_off()
        mc.release_all_servos()
        logging.info("Servos desactivados. Sesión finalizada.")



def main() -> int:
    configure_logging()
    
    print("\n" + "="*60)
    print("  MYCOBOT 280 - MODOS DE TRABAJO")
    print("="*60)
    print("Selecciona modo:")
    print("  1) Modo libre + registro de posiciones")
    print("  2) Reproducir posiciones guardadas")
    print("="*60 + "\n")
    
    mc = None
    try:
        mode = input("Elige opción (1/2): ").strip()
        mc = init_robot()

        if mode == "2":
            files = list_saved_position_files()
            if not files:
                logging.error("No hay archivos de posiciones en el directorio actual")
                return 1
            print("\nArchivos disponibles:")
            for i, f in enumerate(files, 1):
                print(f"  {i}) {f}")
            idx_str = input("Selecciona archivo (número): ").strip()
            if not idx_str.isdigit() or int(idx_str) < 1 or int(idx_str) > len(files):
                logging.error("Selección inválida")
                return 1
            filename = files[int(idx_str) - 1]
            positions = load_positions_from_file(filename)
            if not positions:
                return 1
            replay_positions(mc, positions)
            return 0

        # Modo 1: libre + registro
        if not enable_free_mode(mc):
            logging.error("No se pudo activar el modo libre. Verifica la conexión.")
            return 1

        print("\n💡 El brazo está ahora en modo libre. Comienza a moverlo...\n")

        position_count = 0
        while True:
            try:
                user_input = input("Pulsa ENTER para capturar posición (o 'q' para salir): ").strip().lower()

                if user_input == 'q':
                    logging.info("Finalizando captura de posiciones...")
                    break

                position_count += 1
                capture_position(mc, position_count)

            except EOFError:
                break

        # Desactiva modo libre y guarda
        logging.info("Desactivando modo libre...")
        mc.set_free_mode(0)

        save_positions_to_file()

        logging.info("✓ Sesión finalizada. Total de posiciones: %d", position_count)
        return 0
        
    except KeyboardInterrupt:
        logging.warning("\n⚠ Interrumpido por el usuario")
        if recorded_positions:
            save_positions_to_file()
        return 1
    except Exception as e:
        logging.exception("✗ Error inesperado: %s", e)
        return 1
    finally:
        if mc:
            try:
                mc.set_free_mode(0)
                mc.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())