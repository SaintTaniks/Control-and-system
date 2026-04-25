# dashboard/ui.py
"""
=============================================
 Vehicle Dashboard UI (pygame)
=============================================
"""

import math
from typing import Tuple
import pygame
from pygame import gfxdraw

# --- Type hint for colors ---
Color = Tuple[int, int, int]

# --- HELPER FUNCTIONS ---

def clamp(v: float, lo: float, hi: float) -> float:
    """Clamps a value between a low and high bound."""
    return max(lo, min(hi, v))

def map_range(v: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    """Linearly maps a value from one range to another."""
    in_span = in_max - in_min
    out_span = out_max - out_min
    if in_span == 0:
        return out_min
    scaled = (v - in_min) / in_span
    return out_min + (scaled * out_span)

def aa_line(surf: pygame.Surface, x1: int, y1: int, x2: int, y2: int, color: Color, blend: int = 1):
    """Draws a simple anti-aliased line."""
    try:
        gfxdraw.line(surf, x1, y1, x2, y2, color)
    except Exception:
        pygame.draw.line(surf, color, (x1, y1), (x2, y2), 1)

def draw_arc_gauge(
    surf: pygame.Surface,
    center_x: int,
    center_y: int,
    radius: int,
    start_angle_rad: float, # Pygame angles (Radians, CCW, 0=East)
    end_angle_rad: float,   # Must be > start_angle for CCW
    line_width: int,
    color_bg: Color
):
    """
    Draws a single-color background arc gauge that fills COUNTER-CLOCKWISE.
    """
    pygame.draw.arc(
        surf,
        color_bg,
        (center_x - radius, center_y - radius, radius * 2, radius * 2),
        start_angle_rad,
        end_angle_rad,
        line_width
    )

def draw_label(
    surf: pygame.Surface,
    text: str,
    pos: Tuple[int, int],
    font: pygame.font.Font,
    color: Color,
    align: str = "center"
):
    """Draws a text label with specified alignment."""
    text_surf = font.render(text, True, color)
    text_rect = text_surf.get_rect()
    if align == "center":
        text_rect.center = pos
    elif align == "left":
        text_rect.midleft = pos
    elif align == "right":
        text_rect.midright = pos
    surf.blit(text_surf, text_rect)


# --- MAIN UI CLASS ---

class DashboardUI:
    """
    Handles all rendering for the dashboard.
    """
    
    def __init__(
        self,
        screen: pygame.Surface,
        units: str = "mph",
        speed_max: int = 100,  
        theme: str = "dark"
    ):
        self.screen = screen
        self.W, self.H = screen.get_size()
        self.units = units
        
        # FORCING speed_max to 100 here so main.py cannot override it!
        self.speed_max = 100 
        
        self.last_warn = False
        
        # --- Colors ---
        self.BG = (15, 15, 20)          # Deep dark gray/black background
        self.FG = (240, 240, 240)       # Bright white/light-gray
        self.FG_LIGHT = (100, 100, 110) # Dim gray for labels and background gauge
        self.DANGER = (255, 50, 50)     # Red for needle and warnings
        self.AMBER = (255, 191, 0)      # Amber for warnings
        self.GREEN = (0, 200, 100)      # Green for SOC/headlights
        
        # --- Font loading ---
        try:
            font_name = None 
            self.font_small = pygame.font.Font(font_name, self.scale(24)) 
            self.font_mid = pygame.font.Font(font_name, self.scale(36))
            self.font_large = pygame.font.Font(font_name, self.scale(130))
            self.font_corner_lbl = pygame.font.Font(font_name, self.scale(30))
            self.font_corner_val = pygame.font.Font(font_name, self.scale(48))
        except Exception as e:
            if font_name:
                print(f"Warning: Could not load custom font '{font_name}'. {e}")
                print("Falling back to default system font 'Arial'.")
            self.font_small = pygame.font.SysFont("Arial", self.scale(24))
            self.font_mid = pygame.font.SysFont("Arial", self.scale(36), bold=True)
            self.font_large = pygame.font.SysFont("Arial", self.scale(130), bold=True)
            self.font_corner_lbl = pygame.font.SysFont("Arial", self.scale(30))
            self.font_corner_val = pygame.font.SysFont("Arial", self.scale(48), bold=True)

    def scale(self, x: int) -> int:
        """Scales a pixel dimension based on screen height (baseline 480)."""
        return int(x * (self.H / 480.0))

    def render(self, model):
        """Main render loop, called every frame."""
        
        # --- Fill background ---
        self.screen.fill(self.BG)
        
        # --- Safely Fetch Data ---
        speed = getattr(model, 'speed', 0.0)
        soc = getattr(model, 'soc', 0.0)
        temp = getattr(model, 'battery_temp', getattr(model, 'temp', 0.0))
        voltage = getattr(model, 'pack_voltage', 0.0)
        current = getattr(model, 'current', 0.0)
        high_cell = getattr(model, 'high_cell_voltage', 0.0)
        low_cell = getattr(model, 'low_cell_voltage', 0.0)
        
        turn_left = getattr(model, 'turn_left', False)
        turn_right = getattr(model, 'turn_right', False)
        gear = getattr(model, 'gear', "P")
        headlight = getattr(model, 'headlight', False)
        warn = getattr(model, 'warn', False)

        # --- Draw Center Speedometer ---
        self._draw_speed_gauge(
            speed,
            (int(self.W * 0.5), int(self.H * 0.55)), # Center
            self.scale(180), # Radius
            self.scale(25)   # Line width
        )
        
        # --- Draw Corner Data Blocks ---
        pad_x = self.scale(40)
        pad_y = self.scale(40)
        
        # -- TOP LEFT: State of Charge --
        soc_color = self.GREEN if soc > 20 else self.DANGER
        self._draw_text_block("SOC", f"{soc:.1f} %", pad_x, pad_y, align="left", val_color=soc_color)
        
        # -- TOP RIGHT: Voltage & Current --
        self._draw_text_block("PACK VOLTAGE", f"{voltage:.1f} V", self.W - pad_x, pad_y, align="right")
        self._draw_text_block("CURRENT", f"{current:.1f} A", self.W - pad_x, pad_y + self.scale(75), align="right")
        
        # -- BOTTOM LEFT: Cell Voltages (Stacked) --
        # CHANGED: Increased the offset from 50 to 85 to lift the elements further up
        bottom_y = self.H - pad_y - self.scale(85)
        self._draw_text_block("MAX CELL", f"{high_cell:.2f} V", pad_x, bottom_y - self.scale(75), align="left")
        self._draw_text_block("MIN CELL", f"{low_cell:.2f} V", pad_x, bottom_y, align="left")
        
        # -- BOTTOM RIGHT: Temperature --
        temp_color = self.DANGER if temp > 50 else self.FG
        self._draw_text_block("TEMP", f"{temp:.1f} °C", self.W - pad_x, bottom_y, align="right", val_color=temp_color)

        # --- Draw Indicators ---
        self._draw_indicators(turn_left, turn_right, gear, headlight, warn)

    def _draw_text_block(self, label: str, value_str: str, x: int, y: int, align: str = "left", val_color=None):
        """Helper to cleanly draw stacked Label/Value text in the corners."""
        if val_color is None:
            val_color = self.FG
            
        lbl_surf = self.font_corner_lbl.render(label, True, self.FG_LIGHT)
        val_surf = self.font_corner_val.render(value_str, True, val_color)
        
        if align == "left":
            lbl_rect = lbl_surf.get_rect(bottomleft=(x, y - 2))
            val_rect = val_surf.get_rect(topleft=(x, y + 2))
        else: # align == "right"
            lbl_rect = lbl_surf.get_rect(bottomright=(x, y - 2))
            val_rect = val_surf.get_rect(topright=(x, y + 2))
            
        self.screen.blit(lbl_surf, lbl_rect)
        self.screen.blit(val_surf, val_rect)

    def _draw_speed_gauge(self, speed: float, center: Tuple[int, int], radius: int, line_width: int):
        """
        --- FINAL GAUGE (CLOCKWISE) ---
        Draws the main speed gauge.
        """
        cx, cy = center
        
        # --- 1. Draw Background Arc (Open at bottom) ---
        ARC_START_RAD = math.radians(330) # 5 o'clock
        ARC_END_RAD = math.radians(210 + 360) # 7 o'clock (one turn later)
        
        draw_arc_gauge(
            self.screen, cx, cy, radius,
            ARC_START_RAD, ARC_END_RAD,
            line_width,
            self.FG_LIGHT,  # Background color
        )
        
        # --- 2. Draw Text Labels (Clockwise) ---
        LABEL_START_ANGLE_MATH = 210 # 7 o'clock
        LABEL_END_ANGLE_MATH = -30   # 5 o'clock (or 330)
        
        # 6 labels: 0, 20, 40, 60, 80, 100
        num_labels = 6 
        for i in range(num_labels):
            val_frac = i / (num_labels - 1.0)
            val = int(val_frac * self.speed_max)
            
            angle_deg = map_range(val_frac, 0.0, 1.0, LABEL_START_ANGLE_MATH, LABEL_END_ANGLE_MATH)
            angle_rad = math.radians(angle_deg)
            
            x = cx + math.cos(angle_rad) * (radius + self.scale(30))
            y = cy - math.sin(angle_rad) * (radius + self.scale(30)) # <-- NEGATIVE SIN
            
            draw_label(self.screen, str(val), (int(x), int(y)), self.font_mid, self.FG)

        # --- 3. Draw Units (MPH) ---
        draw_label(self.screen, self.units.upper(), (cx, cy + self.scale(50)), self.font_mid, self.FG_LIGHT)

        # --- 4. Draw Big Speed Number ---
        speed_str = f"{speed:3.0f}"
        speed_color = self.FG
        draw_label(self.screen, speed_str, (cx, cy + self.scale(100)), self.font_large, speed_color)
        
        # --- 5. Draw Needle (Red, Clockwise) ---
        speed_frac = clamp(speed, 0, self.speed_max) / self.speed_max
        
        needle_angle_deg = map_range(speed_frac, 0.0, 1.0, LABEL_START_ANGLE_MATH, LABEL_END_ANGLE_MATH)
        needle_angle_rad = math.radians(needle_angle_deg)
        
        x_end = cx + math.cos(needle_angle_rad) * (radius - self.scale(5))
        y_end = cy - math.sin(needle_angle_rad) * (radius - self.scale(5)) # <-- NEGATIVE SIN
        
        x_base1 = cx + math.cos(needle_angle_rad + 1.57) * self.scale(8)
        y_base1 = cy - math.sin(needle_angle_rad + 1.57) * self.scale(8) # <-- NEGATIVE SIN
        x_base2 = cx + math.cos(needle_angle_rad - 1.57) * self.scale(8)
        y_base2 = cy - math.sin(needle_angle_rad - 1.57) * self.scale(8) # <-- NEGATIVE SIN

        pygame.draw.polygon(self.screen, self.DANGER, [
            (x_end, y_end),
            (x_base1, y_base1),
            (x_base2, y_base2)
        ])
        pygame.draw.circle(self.screen, self.FG_LIGHT, (cx, cy), self.scale(15))

    def _draw_indicators(
        self,
        left_on: bool,
        right_on: bool,
        gear: str,
        headlight: bool,
        warn: bool
    ):
        """Draws all the top/bottom indicators."""
        # --- Warning (Bottom Right) ---
        if warn:
            # Flash the warning
            warn_color = self.DANGER if int(pygame.time.get_ticks() / 300) % 2 == 0 else self.BG
            # Lifted this slightly as well just in case!
            x, y = int(self.W * 0.92), int(self.H * 0.88)
            pygame.draw.polygon(self.screen, warn_color, [
                (x, y - self.scale(20)),
                (x - self.scale(25), y + self.scale(15)),
                (x + self.scale(25), y + self.scale(15))
            ])
            # Draw "!" inside
            draw_label(self.screen, "!", (x, y + self.scale(5)), self.font_mid, self.BG)
