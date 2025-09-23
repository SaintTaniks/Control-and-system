# dashboard/main.py

# dashboard/main.py
"""
=============================================
 Vehicle Dashboard Main Entry Point
=============================================

This script launches the Raspberry Pi vehicle dashboard
using pygame for rendering. It initializes the data model
(sensor inputs or simulated data), the UI, and runs the
main event loop.

Usage
-----
Run in fullscreen mode on a Pi LCD (800x480):
    python3 -m dashboard.main --width 800 --height 480 --fps 30 --units km/h

Run windowed for development on your laptop:
    python3 -m dashboard.main --windowed --width 1280 --height 720 --show-fps

Arguments
---------
--width <int>       : Screen width (default: 800)
--height <int>      : Screen height (default: 480)
--windowed          : Run in a window instead of fullscreen
--fps <int>         : Target frames per second (default: 30)
--units [km/h|mph]  : Units for speed display (default: km/h)
--show-fps          : Show FPS counter overlay
--cursor            : Show mouse cursor (hidden by default)

Notes
-----
- On Raspberry Pi with official LCD, SDL_FBDEV is set to /dev/fb0
  to enable direct framebuffer rendering.
- Exit with 'q' or 'ESC'.
- Designed to auto-start via systemd service if deployed in a vehicle.
"""

import os
import sys
import time
import signal
import argparse
import pygame

# Local modules
from model import DataModel
from ui import DashboardUI


def parse_args():
    parser = argparse.ArgumentParser(
        description="Vehicle Dashboard (pygame)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--width", type=int, default=800, help="Screen width")
    parser.add_argument("--height", type=int, default=480, help="Screen height")
    parser.add_argument("--windowed", action="store_true", help="Run windowed")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS")
    parser.add_argument("--units", choices=["km/h", "mph"], default="km/h",
                        help="Speed units")
    parser.add_argument("--show-fps", action="store_true", help="Show FPS overlay")
    parser.add_argument("--cursor", action="store_true", help="Show mouse cursor")
    return parser.parse_args()


def configure_env_for_pi(fullscreen: bool):
    """
    Make SDL behave nicely on Raspberry Pi when using the LCD framebuffer.
    Safe to call on other platforms (no-ops).
    """
    if fullscreen and sys.platform.startswith("linux"):
        # If you use the Pi official LCD (framebuffer), SDL_FBDEV helps.
        os.environ.setdefault("SDL_FBDEV", "/dev/fb0")
        # Prevent screensaver/blanking if running under X:
        os.environ.setdefault("SDL_VIDEO_CENTERED", "1")


def install_sigint_handler():
    # Let Ctrl+C kill the app immediately even inside pygame loop
    signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))


def main():
    args = parse_args()
    fullscreen = not args.windowed
    configure_env_for_pi(fullscreen)
    install_sigint_handler()

    # Initialize pygame
    pygame.init()
    flags = pygame.FULLSCREEN if fullscreen else 0
    screen = pygame.display.set_mode((args.width, args.height), flags)
    pygame.display.set_caption("Vehicle Dashboard")

    # Optional: hide cursor for kiosk mode
    pygame.mouse.set_visible(args.cursor)

    clock = pygame.time.Clock()

    # Instantiate model & UI
    model = DataModel(units=args.units) if "units" in DataModel.__init__.__code__.co_varnames else DataModel()
    ui = DashboardUI(screen, units=args.units) if "units" in DashboardUI.__init__.__code__.co_varnames else DashboardUI(screen)

    # If UI exposes a way to set units, do it; ignore if not present
    if hasattr(ui, "set_units"):
        try:
            ui.set_units(args.units)
        except Exception:
            pass

    running = True
    last_fps_stamp = time.time()
    fps_value = 0

    try:
        while running:
            dt = clock.tick(args.fps) / 1000.0  # seconds

            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    if e.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False

            # Update data model (read sensors, CAN, etc.)
            model.update(dt)

            # Draw
            ui.render(model)

            # Optional FPS overlay (top-left corner)
            if args.show_fps:
                fps_value = 1.0 / dt if dt > 0 else 0.0
                # Try to use UI's small font if exposed; otherwise basic system font
                font = getattr(ui, "font_small", pygame.font.SysFont("DejaVu Sans", 16))
                txt = font.render(f"FPS: {fps_value:5.1f}", True, (180, 180, 180))
                screen.blit(txt, (8, 6))

            pygame.display.flip()

            # Log FPS once every ~5s when flag is on (useful for systemd logs)
            if args.show_fps and (time.time() - last_fps_stamp) > 5:
                print(f"[dashboard] FPS ~ {fps_value:0.1f}")
                last_fps_stamp = time.time()

    except SystemExit:
        # Normal exit via sys.exit or SIGINT
        pass
    except Exception as ex:
        # Print traceback so systemd/journalctl can capture it
        print(f"[dashboard] Unhandled exception: {ex}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
