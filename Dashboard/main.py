"""
=============================================
 Vehicle Dashboard Main Entry Point
=============================================
"""

import os
import sys
import argparse
import pygame
import csv
import serial
import time
from datetime import datetime

# --- IMPORT YOUR MODULES ---
from ui import DashboardUI
from model import DataModel
from sensors.can_reader import CANReader

# --- CONFIGURATION ---
WIDTH, HEIGHT = 800, 480
FPS = 30
ZIGBEE_PORT = None 
ZIGBEE_BAUD = 9600
ENABLE_LOGGING = True

def main(args):
    # --- Pygame Setup ---
    print("[main] Initializing Pygame...")
    
    # Set the framebuffer environment variable for the Pi
    os.environ["SDL_FBDEV"] = "/dev/fb0"
    
    pygame.init()
    
    # FORCE HIDE THE MOUSE CURSOR
    pygame.mouse.set_visible(False)
    
    # FORCE FULLSCREEN MODE
    flags = pygame.FULLSCREEN | pygame.DOUBLEBUF
    screen = pygame.display.set_mode((args.width, args.height), flags)
    
    pygame.display.set_caption("EV Dashboard")
    clock = pygame.time.Clock()

    # --- Zigbee Setup ---
    zigbee_port = None
    if ZIGBEE_PORT:
        try:
            zigbee_port = serial.Serial(ZIGBEE_PORT, ZIGBEE_BAUD, timeout=0.1)
        except Exception as e:
            pass

    # --- DataModel Setup ---
    print(f"[main] Initializing DataModel (Simulate: {args.simulate})")
    model = DataModel(simulate=args.simulate)
    
    # --- CAN Sensor Setup ---
    can_reader = None
    if not args.simulate:
        print("[main] Starting CAN Reader...")
        can_reader = CANReader(model)
        can_reader.start()
    else:
        print("[main] Simulation Mode: CAN Reader NOT started.")

    # --- UI Setup ---
    ui = DashboardUI(screen, units=args.units, speed_max=args.speed_max)

    # --- CSV Setup ---
    log_file = None
    log_writer = None
    if ENABLE_LOGGING:
        log_folder = "logs"
        if not os.path.exists(log_folder):
            os.makedirs(log_folder)
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        full_log_path = os.path.join(log_folder, f"vehicle_log_{timestamp_str}.csv")
        try:
            log_file = open(full_log_path, 'w', newline='')
            log_writer = csv.writer(log_file)
            log_writer.writerow(["Timestamp", "SOC", "Pack_Voltage", "Pack_Current", "Bat_Temp_Avg", "Bat_Temp_High", "Pack_DCL", "Pack_CCL", "Pack_Ah", "Pack_Res", "High_Cell_V", "Low_Cell_V"])
        except:
            pass

    running = True
    last_log_time = time.time()
    
    print("[main] Starting main loop...")
    try:
        while running:
            dt = clock.tick(FPS) / 1000.0  

            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    # Pressing Escape or 'q' will close the fullscreen app
                    if e.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False

            # Tell the model to tick the simulation math forward
            model.update(dt)

            # Retrieve data and draw
            data = model.get_all()
            ui.render(model) 
            
            if args.show_fps:
                try:
                    fps_surf = ui.font_small.render(f"FPS: {clock.get_fps():.1f}", True, (255, 255, 255))
                    screen.blit(fps_surf, (10, 10))
                except:
                    pass

            pygame.display.flip()

            # Logging
            current_time = time.time()
            if current_time - last_log_time >= 0.5:
                last_log_time = current_time
                if log_writer:
                    try:
                        timestamp_csv = datetime.now().strftime("%H:%M:%S.%f")
                        log_writer.writerow([
                            timestamp_csv,
                            f"{data.get('soc', 0)}",
                            f"{data.get('pack_voltage', 0):.2f}",
                            f"{data.get('current', 0):.2f}",
                            f"{data.get('battery_temp', 0)}",
                            f"{data.get('temp_high', 0)}",
                            f"{data.get('pack_dcl', 0):.2f}",
                            f"{data.get('pack_ccl', 0):.2f}",
                            f"{data.get('pack_ah', 0)}",
                            f"{data.get('pack_res', 0)}",
                            f"{data.get('high_cell_voltage', 0):.2f}",
                            f"{data.get('low_cell_voltage', 0):.2f}"
                        ])
                    except:
                        pass
                        
    except Exception as ex:
        import traceback
        traceback.print_exc()
    finally:
        if can_reader: can_reader.stop()
        if zigbee_port and zigbee_port.is_open: zigbee_port.close()
        if log_file: log_file.close()
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    parser.add_argument("--windowed", action="store_true") # Left in so it doesn't crash if you passed it before
    parser.add_argument("--show-fps", action="store_true")
    parser.add_argument("--cursor", action="store_true")   # Left in so it doesn't crash
    parser.add_argument("--units", type=str, default="mph", choices=["km/h", "mph"])
    parser.add_argument("--speed-max", type=int, default=100) # Changed default to 100 just to be safe
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()
    main(args)
