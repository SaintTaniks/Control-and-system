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

# --- Set your Zigbee/Serial port here ---
ZIGBEE_PORT = None 
# Example: ZIGBEE_PORT = "/dev/ttyUSB0" 
ZIGBEE_BAUD = 9600

# --- Set to False to disable CSV logging ---
ENABLE_LOGGING = True

def main(args):
    """Application entry point."""

    # --- Pygame Setup ---
    print("[main] Initializing Pygame...")
    if not args.windowed:
        os.environ["SDL_FBDEV"] = "/dev/fb0"
    
    pygame.init()
    if not args.cursor and not args.windowed:
        pygame.mouse.set_visible(False)
    
    flags = pygame.FULLSCREEN | pygame.DOUBLEBUF if not args.windowed else 0
    screen = pygame.display.set_mode((args.width, args.height), flags)
    pygame.display.set_caption("EV Dashboard")
    clock = pygame.time.Clock()

    # --- Zigbee (Serial) Setup ---
    zigbee_port = None
    if ZIGBEE_PORT:
        try:
            zigbee_port = serial.Serial(ZIGBEE_PORT, ZIGBEE_BAUD, timeout=0.1)
            print(f"[main] Opened Zigbee (Serial) port at {ZIGBEE_PORT}")
        except Exception as e:
            print(f"[main] WARNING: Could not open Zigbee port {ZIGBEE_PORT}: {e}")
            zigbee_port = None

    # --- DataModel Setup ---
    print(f"[main] Initializing DataModel (Simulate: {args.simulate})")
    # We initialize the model without arguments (matching the model.py provided earlier)
    model = DataModel()
    
    # --- CAN Sensor Setup ---
    can_reader = None
    if not args.simulate:
        print("[main] Starting CAN Reader...")
        # Create the reader and give it the model so it can store data there
        can_reader = CANReader(model)
        can_reader.start()
    else:
        print("[main] Simulation Mode: CAN Reader NOT started.")

    # --- UI Setup ---
    # Pass necessary args to UI (assuming your UI class accepts them)
    ui = DashboardUI(
        screen, 
        units=args.units, 
        speed_max=args.speed_max
    )

    # --- CSV Log File Setup (NEW FOLDER LOGIC) ---
    log_file = None
    log_writer = None
    
    if ENABLE_LOGGING:
        # 1. Create logs folder
        log_folder = "logs"
        if not os.path.exists(log_folder):
            os.makedirs(log_folder)
            print(f"[main] Created new folder: {log_folder}")

        # 2. Generate Timestamped Filename
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        log_filename = f"vehicle_log_{timestamp_str}.csv"
        full_log_path = os.path.join(log_folder, log_filename)

        try:
            log_file = open(full_log_path, 'w', newline='')
            log_writer = csv.writer(log_file)
            
            # 3. Write Header with NEW COLUMNS
            log_writer.writerow([
                "Timestamp", 
                "SOC", 
                "Pack_Voltage", 
                "Pack_Current", 
                "Bat_Temp_Avg", 
                "Bat_Temp_High", 
                "Pack_DCL", 
                "Pack_CCL", 
                "Pack_Ah",
                "Pack_Res"
            ])
            print(f"[main] Logging data to {full_log_path}")
        except Exception as e:
            print(f"[main] WARNING: Could not create log file: {e}")

    
    running = True
    last_log_time = time.time()
    
    print("[main] Starting main loop...")
    try:
        while running:
            dt = clock.tick(FPS) / 1000.0  # Delta-time in seconds

            # --- Event Handling ---
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    if e.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False

            # --- Data Retrieval ---
            # Get the raw dictionary from the model
            data = model.get_all()

            # --- Render UI ---
            # We pass the full model object to the UI
            ui.render(model) 
            
            # FPS Counter
            if args.show_fps:
                fps = clock.get_fps()
                try:
                    # Assuming UI has a font_small, otherwise use default
                    fps_surf = ui.font_small.render(f"FPS: {fps:.1f}", True, (255, 255, 255))
                    screen.blit(fps_surf, (10, 10))
                except:
                    pass

            pygame.display.flip()

            # --- Data Logging and Sending (every 0.5s) ---
            current_time = time.time()
            if current_time - last_log_time >= 0.5:
                last_log_time = current_time
                
                # 1. Send to Zigbee (Using NEW keys)
                if zigbee_port:
                    try:
                        zigbee_msg = (
                            f"V:{data.get('pack_voltage', 0):.1f},"
                            f"B:{data.get('soc', 0):.1f},"
                            f"T:{data.get('battery_temp', 0):.1f}\n"
                        )
                        zigbee_port.write(zigbee_msg.encode('ascii'))
                    except Exception as e:
                        print(f"[main] Zigbee write error: {e}")

                # 2. Write to CSV Log (Using NEW keys)
                if log_writer:
                    try:
                        timestamp_csv = datetime.now().strftime("%H:%M:%S.%f")
                        data_row = [
                            timestamp_csv,
                            f"{data.get('soc', 0)}",
                            f"{data.get('pack_voltage', 0):.2f}",
                            f"{data.get('current', 0):.2f}",
                            f"{data.get('battery_temp', 0)}",
                            f"{data.get('temp_high', 0)}",
                            f"{data.get('pack_dcl', 0):.2f}",
                            f"{data.get('pack_ccl', 0):.2f}",
                            f"{data.get('pack_ah', 0)}",
                            f"{data.get('pack_res', 0)}"
                        ]
                        log_writer.writerow(data_row)
                    except Exception as e:
                        print(f"[main] CSV write error: {e}")
                        
    except Exception as ex:
        print(f"[main] CRITICAL ERROR: {ex}")
        import traceback
        traceback.print_exc()
    
    finally:
        # --- Cleanup ---
        print("[main] Shutting down...")
        
        # Stop CAN Reader
        if can_reader:
            can_reader.stop()
            
        if zigbee_port and zigbee_port.is_open:
            zigbee_port.close()
            print("[main] Zigbee port closed.")
            
        if log_file:
            log_file.close()
            print("[main] Log file saved.")
            
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="ERA Vehicle Dashboard")
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    parser.add_argument("--windowed", action="store_true", help="Run in a window")
    parser.add_argument("--show-fps", action="store_true")
    parser.add_argument("--cursor", action="store_true", help="Show mouse cursor")
    parser.add_argument("--units", type=str, default="mph", choices=["km/h", "mph"])
    parser.add_argument("--speed-max", type=int, default=120, help="Max speed on gauge")

    parser.add_argument(
        "--simulate", 
        action="store_true", 
        help="Run without hardware (CAN Reader will NOT start)"
    )
    
    args = parser.parse_args()
    main(args)
