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
        speed_max: int = 120,
        theme: str = "dark"
    ):
        self.screen = screen
        self.W, self.H = screen.get_size()
        self.units = units
        self.speed_max = speed_max
        self.last_warn = False
        
        # --- Colors ---
        self.BG = (0, 0, 0)             # Black background
        self.FG = (240, 240, 240)       # Bright white/light-gray
        self.FG_LIGHT = (100, 100, 100) # Dim gray for background gauge
        self.DANGER = (255, 0, 0)       # Red for needle and warnings
        self.AMBER = (255, 191, 0)      # Amber for warnings
        self.GREEN = (0, 200, 0)        # Green for SOC/headlights
        
        # --- Font loading ---
        try:
            # Load default font. 
            # Replace 'None' with "YourFont.ttf" to load a custom font file.
            font_name = None 
            self.font_small = pygame.font.Font(font_name, self.scale(24))
            self.font_mid = pygame.font.Font(font_name, self.scale(36))
            self.font_large = pygame.font.Font(font_name, self.scale(130))
        except Exception as e:
            if font_name:
                print(f"Warning: Could not load custom font '{font_name}'. {e}")
                print("Falling back to default system font 'Arial'.")
            self.font_small = pygame.font.SysFont("Arial", self.scale(24))
            self.font_mid = pygame.font.SysFont("Arial", self.scale(36))
            self.font_large = pygame.font.SysFont("Arial", self.scale(130))

    def scale(self, x: int) -> int:
        """Scales a pixel dimension based on screen height (baseline 480)."""
        return int(x * (self.H / 480.0))

    def render(self, model):
        """Main render loop, called every frame."""
        
        # --- Fill background ---
        self.screen.fill(self.BG)
        
        # --- Draw Gauges ---
        self._draw_speed_gauge(
            model.speed,
            (int(self.W * 0.5), int(self.H * 0.55)), # Center
            self.scale(180), # Radius
            self.scale(25)   # Line width
        )
        
        self._draw_soc_gauge(
            model.soc,
            (int(self.W * 0.15), int(self.H * 0.7)), # Moved from 0.2
            self.scale(60),
            self.scale(10)
        )
        
        self._draw_temp_gauge(
            model.temp,
            (int(self.W * 0.85), int(self.H * 0.7)), # Moved from 0.8
            self.scale(60),
            self.scale(10)
        )

        # --- Draw Indicators ---
        self._draw_indicators(
            model.turn_left,
            model.turn_right,
            model.gear,
            model.headlight,
            model.warn
        )

    def _draw_speed_gauge(self, speed: float, center: Tuple[int, int], radius: int, line_width: int):
        """
        --- FINAL GAUGE (CLOCKWISE) ---
        Draws the main speed gauge.
        0 (bottom-left) -> 60 (top) -> 120 (bottom-right)
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
        
        num_labels = 7 # 0, 20, 40, 60, 80, 100, 120
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


    def _draw_soc_gauge(self, soc: float, center: Tuple[int, int], radius: int, line_width: int):
        """Draws the SOC gauge (battery)."""
        cx, cy = center
        soc_frac = clamp(soc, 0, 100) / 100.0
        
        # --- NEW SOC COLORS ---
        soc_color = self.GREEN  # Default green
        if soc < 20:
            soc_color = self.DANGER
        elif soc < 50: # 20-49%
            soc_color = self.AMBER

        # --- 1. Draw Background Arc (Open at bottom) ---
        ARC_START_RAD = math.radians(330) # 5 o'clock
        ARC_END_RAD = math.radians(210 + 360) # 7 o'clock (one turn later)
        
        pygame.draw.arc(
            self.screen, self.FG_LIGHT,
            (cx - radius, cy - radius, radius * 2, radius * 2),
            ARC_START_RAD, ARC_END_RAD, line_width
        )
        
        # --- 2. Draw FG (Fill CLOCKWISE from left to right) ---
        # 
        # *** FIX ***
        # The fill must use the same angle space as the background arc to line up.
        # Background arc draws CCW from 330 to 570 (210+360).
        # Fill (CW) must map from 0-100% to the range [570, 330].
        
        FILL_START_ANGLE_MATH = 210 + 360 # 7 o'clock (570)
        FILL_END_ANGLE_MATH = 330         # 5 o'clock (330)
        
        # Calculate the angle for the current value
        fill_angle_deg_current = map_range(soc_frac, 0.0, 1.0, FILL_START_ANGLE_MATH, FILL_END_ANGLE_MATH)

        # We only draw if the value is > 0 
        if soc_frac > 0.0:
            # Pygame draws CCW. To draw CW from 570 to current, we draw CCW from current to 570.
            start_rad_pygame = math.radians(fill_angle_deg_current)
            end_rad_pygame = math.radians(FILL_START_ANGLE_MATH) # End at 570
            
            pygame.draw.arc(
                self.screen, soc_color,
                (cx - radius, cy - radius, radius * 2, radius * 2),
                start_rad_pygame,
                end_rad_pygame,
                line_width
            )
        
        draw_label(self.screen, f"{soc:3.0f}%", (cx, cy), self.font_mid, soc_color)
        draw_label(self.screen, "SOC", (cx, cy + self.scale(30)), self.font_small, self.FG_LIGHT)

    def _draw_temp_gauge(self, temp: float, center: Tuple[int, int], radius: int, line_width: int):
        """Draws the Temperature gauge."""
        cx, cy = center
        temp_max = 100.0  # Assume 100 C max
        temp_frac = clamp(temp, 0, temp_max) / temp_max
        
        temp_color = self.FG
        if temp > 90:
            temp_color = self.DANGER
        elif temp > 80:
            temp_color = self.AMBER

        # --- 1. Draw Background Arc (Open at bottom) ---
        ARC_START_RAD = math.radians(330) # 5 o'clock
        ARC_END_RAD = math.radians(210 + 360) # 7 o'clock (one turn later)
        
        pygame.draw.arc(
            self.screen, self.FG_LIGHT,
            (cx - radius, cy - radius, radius * 2, radius * 2),
            ARC_START_RAD, ARC_END_RAD, line_width
        )
        
        # --- 2. Draw FG (Fill CLOCKWISE from left to right) ---
        # 
        # *** FIX ***
        # The fill must use the same angle space as the background arc to line up.
        # Background arc draws CCW from 330 to 570 (210+360).
        # Fill (CW) must map from 0-100% to the range [570, 330].
        
        FILL_START_ANGLE_MATH = 210 + 360 # 7 o'clock (570)
        FILL_END_ANGLE_MATH = 330         # 5 o'clock (330)
        
        # Calculate the angle for the current value
        fill_angle_deg_current = map_range(temp_frac, 0.0, 1.0, FILL_START_ANGLE_MATH, FILL_END_ANGLE_MATH)

        # We only draw if the value is > 0 
        if temp_frac > 0.0:
            # Pygame draws CCW. To draw CW from 570 to current, we draw CCW from current to 570.
            start_rad_pygame = math.radians(fill_angle_deg_current)
            end_rad_pygame = math.radians(FILL_START_ANGLE_MATH) # End at 570
            
            pygame.draw.arc(
                self.screen, temp_color,
                (cx - radius, cy - radius, radius * 2, radius * 2),
                start_rad_pygame,
                end_rad_pygame,
                line_width
            )
        
        # --- NEW LABELS ---
        draw_label(self.screen, f"{temp:3.0f}°C", (cx, cy), self.font_mid, temp_color)
        draw_label(self.screen, "TEMP", (cx, cy + self.scale(30)), self.font_small, self.FG_LIGHT)

    def _draw_indicators(
        self,
        left_on: bool,
        right_on: bool,
        gear: str,
        headlight: bool,
        warn: bool
    ):
        """Draws all the top/bottom indicators."""
        
        # --- Blinkers (Top) ---
        # [DELETED BY USER REQUEST]
        
        # --- Gear (Center Top) ---
        # [DELETED BY USER REQUEST]
        
        # --- Headlight (Bottom Left) ---
        # [REMOVED BY USER REQUEST]

        # --- Warning (Bottom Right) ---
        if warn:
            # Flash the warning
            warn_color = self.DANGER if int(pygame.time.get_ticks() / 300) % 2 == 0 else self.BG
            x, y = int(self.W * 0.92), int(self.H * 0.9)
            pygame.draw.polygon(self.screen, warn_color, [
                (x, y - self.scale(20)),
                (x - self.scale(25), y + self.scale(15)),
                (x + self.scale(25), y + self.scale(15))
            ])
            # Draw "!" inside
            draw_label(self.screen, "!", (x, y + self.scale(5)), self.font_mid, self.BG)


