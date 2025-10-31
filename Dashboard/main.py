"""
=============================================
 Vehicle Dashboard Main Entry Point
=============================================
"""

import os
import argparse
import pygame
import csv
import serial
import time
from datetime import datetime

from ui import DashboardUI
from model import DataModel

# --- CONFIGURATION ---
WIDTH, HEIGHT = 800, 480
FPS = 30

# --- Set your Zigbee/Serial port here ---
# Set to None to disable. This is for your future module.
ZIGBEE_PORT = None 
# Example: ZIGBEE_PORT = "/dev/ttyUSB0" 
ZIGBEE_BAUD = 9600

# --- Set to False to disable CSV logging ---
ENABLE_LOGGING = True
# ---------------------

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

    # --- CSV Log File Setup ---
    log_file = None
    log_writer = None
    if ENABLE_LOGGING:
        log_filename = f"vehicle_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        try:
            log_file = open(log_filename, 'w', newline='')
            log_writer = csv.writer(log_file)
            log_writer.writerow([
                "Timestamp", "Speed", "SOC", "Pack_Voltage", 
                "Pack_Current", "Motor_Temp", "Controller_Temp"
            ])
            print(f"[main] Logging data to {log_filename}")
        except Exception as e:
            print(f"[main] WARNING: Could not create log file: {e}")
            log_file = None
            log_writer = None

    # --- DataModel and UI Setup ---
    print(f"[main] Initializing DataModel (Simulate: {args.simulate})")
    model = DataModel(
        units=args.units,
        simulate=args.simulate,
        enable_can=(not args.simulate), # Only enable CAN if not simulating
        enable_gpio=args.enable_gpio    # Use the new flag
    )
    
    ui = DashboardUI(
        screen, 
        units=args.units, 
        speed_max=args.speed_max
    )
    
    model.start_sensors() # Start background threads (CAN, GPIO)
    
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

            # --- Update & Render ---
            model.update(dt) # This updates simulation OR processes real data
            ui.render(model) # Draw the UI based on model state
            
            if args.show_fps:
                fps = clock.get_fps()
                fps_surf = ui.font_small.render(f"FPS: {fps:.1f}", True, (255, 255, 255))
                screen.blit(fps_surf, (10, 10))

            pygame.display.flip()

            # --- Data Logging and Sending (every 0.5s) ---
            current_time = time.time()
            if current_time - last_log_time >= 0.5:
                last_log_time = current_time
                
                data_to_log = model.get_all_data()

                # 1. Send to Zigbee
                if zigbee_port:
                    try:
                        # Create a simple string message. Example: "S:50.1,B:88.2\n"
                        zigbee_msg = (
                            f"S:{data_to_log.get('speed', 0):.1f},"
                            f"B:{data_to_log.get('soc', 0):.1f},"
                            f"T:{data_to_log.get('temp', 0):.1f}\n"
                        )
                        zigbee_port.write(zigbee_msg.encode('ascii'))
                    except Exception as e:
                        print(f"[main] Zigbee write error: {e}")

                # 2. Write to CSV Log
                if log_writer:
                    try:
                        data_row = [
                            f"{current_time:.2f}",
                            f"{data_to_log.get('speed', 0):.2f}",
                            f"{data_to_log.get('soc', 0):.2f}",
                            f"{data_to_log.get('v_pack', 0):.2f}",
                            f"{data_to_log.get('i_pack', 0):.2f}",
                            f"{data_to_log.get('temp', 0):.1f}",
                            f"{data_to_log.get('controller_temp', 0):.1f}"
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
        model.stop() # Tell sensors to stop
        if zigbee_port and zigbee_port.is_open:
            zigbee_port.close()
            print("[main] Zigbee port closed.")
        if log_file:
            log_file.close()
            print("[main] Log file saved.")
        pygame.quit()
        print("[main] Shutdown complete.")

if __name__ == "__main__":
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="ERA Vehicle Dashboard")
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    parser.add_argument("--windowed", action="store_true", help="Run in a window")
    parser.add_argument("--units", type=str, default="km/h", choices=["km/h", "mph"])
    parser.add_argument("--show-fps", action="store_true")
    parser.add_argument("--cursor", action="store_true", help="Show mouse cursor")
    parser.add_argument("--speed-max", type=int, default=100, help="Max speed on gauge")
    
    parser.add_argument(
        "--simulate", 
        action="store_true", 
        help="Run with simulated data instead of hardware"
    )
    # --- NEW FLAG FOR GPIO ---
    parser.add_argument(
        "--enable-gpio", 
        action="store_true", 
        help="Enable hardware GPIO inputs (blinkers, etc.)"
    )
    
    args = parser.parse_args()
    main(args)
