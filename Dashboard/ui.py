# dashboard/ui.py
"""
=============================================
 Vehicle Dashboard UI (pygame)
=============================================

This module defines the DashboardUI class and all drawing helpers.
It is display-agnostic (no hardware code): given a DataModel with
attributes like speed, soc, temp, etc., it renders the dashboard.

Usage
-----
from ui import DashboardUI
ui = DashboardUI(screen, units="km/h", speed_max=160, theme="dark")
ui.render(model)  # call every frame

Expected DataModel fields
-------------------------
model.speed (float)     : current speed in chosen units
model.soc (float)       : battery state-of-charge (0..100)
model.temp (float)      : temperature in °C
model.turn_left (bool)  : left blinker on/off
model.turn_right (bool) : right blinker on/off
model.headlight (bool)  : headlight status
model.gear (str)        : "P","R","N","D", etc.
model.warn (bool)       : general warning flag
"""

import math
from typing import Tuple
import pygame
from pygame import gfxdraw


Color = Tuple[int, int, int]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def aa_circle(surface: pygame.Surface, x: int, y: int, r: int, color: Color, thickness: int = 0):
    """Antialiased circle or ring (fixed color passing; do NOT unpack)."""
    if thickness <= 0:
        gfxdraw.filled_circle(surface, x, y, r, color)
        gfxdraw.aacircle(surface, x, y, r, color)
    else:
        for i in range(thickness):
            gfxdraw.aacircle(surface, x, y, r - i, color)


def draw_arc(surface: pygame.Surface, center: Tuple[int, int], r: int,
             start_deg: float, end_deg: float, color: Color, width: int = 10, segments: int = 120):
    """Simple polyline arc with configurable width."""
    cx, cy = center
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    step = (end - start) / max(1, segments)
    px, py = None, None
    for i in range(segments + 1):
        a = start + i * step
        x, y = int(cx + r * math.cos(a)), int(cy + r * math.sin(a))
        if px is not None:
            pygame.draw.line(surface, color, (px, py), (x, y), width)
        px, py = x, y


def draw_label(surface: pygame.Surface, text: str, pos: Tuple[int, int],
               font: pygame.font.Font, color: Color, anchor: str = "center"):
    s = font.render(text, True, color)
    rect = s.get_rect()
    setattr(rect, anchor, pos)
    surface.blit(s, rect)


class DashboardUI:
    def __init__(self, screen: pygame.Surface, *,
                 units: str = "km/h",
                 speed_max: float = 160.0,
                 temp_range: Tuple[float, float] = (0.0, 120.0),
                 theme: str = "dark"):
        self.screen = screen
        self.W, self.H = screen.get_size()
        self.units = units
        self.speed_max = speed_max
        self.temp_min, self.temp_max = temp_range
        self.set_theme(theme)

        # Fonts
        pygame.font.init()
        self.font_big = pygame.font.SysFont("DejaVu Sans", max(60, self.scale(120)))
        self.font_mid = pygame.font.SysFont("DejaVu Sans", max(22, self.scale(42)))
        self.font_small = pygame.font.SysFont("DejaVu Sans", max(14, self.scale(24)))

        # Layout
        self.center = (self.W // 2, int(self.H * 0.60))
        base_r = int(min(self.W, self.H) * 0.36)
        self.r_speed_outer = base_r
        self.r_speed_inner = self.r_speed_outer - self.scale(20)

        self.battery_center = (int(self.W * 0.12), int(self.H * 0.25))
        self.battery_r = self.scale(60)

        self.temp_center = (int(self.W * 0.88), int(self.H * 0.25))
        self.temp_r = self.scale(60)

    # ---------- Public API ----------
    def set_units(self, units: str):
        self.units = units

    def set_theme(self, theme: str = "dark"):
        if theme.lower() == "light":
            self.BG = (245, 246, 248)
            self.FG = (20, 22, 24)
            self.ACCENT = (0, 115, 230)
            self.DANGER = (210, 50, 50)
            self.OK = (34, 160, 70)
            self.AMBER = (230, 160, 20)
            self.MUTED = (120, 130, 140)
        else:
            self.BG = (10, 12, 14)
            self.FG = (235, 237, 240)
            self.ACCENT = (70, 170, 255)
            self.DANGER = (255, 80, 80)
            self.OK = (120, 220, 120)
            self.AMBER = (255, 195, 0)
            self.MUTED = (120, 130, 140)

    def render(self, model) -> None:
        """Draw all widgets for the current frame."""
        self.screen.fill(self.BG)
        self.draw_speed_gauge(self.screen, float(getattr(model, "speed", 0.0)))
        self.draw_battery(self.screen, float(getattr(model, "soc", 0.0)))
        self.draw_temp(self.screen, float(getattr(model, "temp", 0.0)))
        self.draw_indicators(
            self.screen,
            bool(getattr(model, "turn_left", False)),
            bool(getattr(model, "turn_right", False)),
            bool(getattr(model, "headlight", False)),
            str(getattr(model, "gear", "N")),
            bool(getattr(model, "warn", False)),
        )

    # ---------- Internals ----------
    def scale(self, v: int) -> int:
        """Rough proportional scaling vs 800x480 baseline."""
        sx = self.W / 800.0
        sy = self.H / 480.0
        return int(round(v * min(sx, sy)))

    def draw_speed_gauge(self, surf: pygame.Surface, speed: float):
        cx, cy = self.center
        start_deg, end_deg = 210, -30  # sweep of the dial

        # Dial arc & ticks
        draw_arc(surf, (cx, cy), self.r_speed_outer, start_deg, end_deg, self.MUTED, width=self.scale(8))
        ticks = 9
        for i in range(ticks + 1):
            frac = i / ticks
            ang = math.radians(start_deg + frac * (end_deg - start_deg))
            r1 = self.r_speed_outer - self.scale(8)
            r2 = r1 - (self.scale(14) if i % 2 == 0 else self.scale(8))
            x1, y1 = int(cx + r1 * math.cos(ang)), int(cy + r1 * math.sin(ang))
            x2, y2 = int(cx + r2 * math.cos(ang)), int(cy + r2 * math.sin(ang))
            pygame.draw.line(surf, self.MUTED, (x1, y1), (x2, y2), 2)

            if i % 2 == 0:
                val = int(i * (self.speed_max / ticks))
                tx = int(cx + (r2 - self.scale(24)) * math.cos(ang))
                ty = int(cy + (r2 - self.scale(24)) * math.sin(ang))
                draw_label(surf, f"{val}", (tx, ty), self.font_small, self.MUTED, "center")

        # Needle
        spd = clamp(speed, 0.0, self.speed_max)
        frac = spd / max(1e-6, self.speed_max)
        ang = math.radians(start_deg + frac * (end_deg - start_deg))
        rN = self.r_speed_inner
        xN, yN = int(cx + rN * math.cos(ang)), int(cy + rN * math.sin(ang))
        pygame.draw.line(surf, self.ACCENT, (cx, cy), (xN, yN), self.scale(6))
        aa_circle(surf, cx, cy, self.scale(10), self.ACCENT)

        # Big readout
        draw_label(surf, f"{int(round(spd))}", (cx, cy - self.scale(20)), self.font_big, self.FG, "center")
        draw_label(surf, self.units, (cx, cy + self.scale(90)), self.font_mid, self.MUTED, "center")

    def draw_battery(self, surf: pygame.Surface, soc: float):
        cx, cy = self.battery_center
        soc = clamp(soc, 0.0, 100.0)
        draw_arc(surf, (cx, cy), self.battery_r, -210, 30, self.MUTED, width=self.scale(10))
        span = -210 + int(240 * (soc / 100.0))
        color = self.OK if soc >= 25 else (self.AMBER if soc >= 10 else self.DANGER)
        draw_arc(surf, (cx, cy), self.battery_r, -210, span, color, width=self.scale(12))
        draw_label(surf, f"{int(soc)}%", (cx, cy - self.scale(5)), self.font_mid, self.FG, "center")
        draw_label(surf, "BAT", (cx, cy + self.scale(35)), self.font_small, self.MUTED, "center")

    def draw_temp(self, surf: pygame.Surface, tempC: float):
        cx, cy = self.temp_center
        draw_arc(surf, (cx, cy), self.temp_r, -210, 30, self.MUTED, width=self.scale(10))
        tnorm = clamp((tempC - self.temp_min) / max(1e-6, (self.temp_max - self.temp_min)), 0.0, 1.0)
        span = -210 + int(240 * tnorm)
        color = self.OK if tempC < 70 else (self.AMBER if tempC < 90 else self.DANGER)
        draw_arc(surf, (cx, cy), self.temp_r, -210, span, color, width=self.scale(12))
        draw_label(surf, f"{int(tempC)}°C", (cx, cy - self.scale(5)), self.font_mid, self.FG, "center")
        draw_label(surf, "TEMP", (cx, cy + self.scale(35)), self.font_small, self.MUTED, "center")

    def draw_indicators(self, surf: pygame.Surface, left_on: bool, right_on: bool,
                        headlight: bool, gear: str, warn: bool):
        # Turn arrows
        y = int(self.H * 0.12)
        xL, xR = int(self.W * 0.30), int(self.W * 0.70)
        if left_on:
            pygame.draw.polygon(surf, self.AMBER, [(xL - self.scale(50), y),
                                                   (xL + self.scale(10), y - self.scale(25)),
                                                   (xL + self.scale(10), y + self.scale(25))])
        if right_on:
            pygame.draw.polygon(surf, self.AMBER, [(xR + self.scale(50), y),
                                                   (xR - self.scale(10), y - self.scale(25)),
                                                   (xR - self.scale(10), y + self.scale(25))])

        # Gear
        draw_label(surf, gear, (self.W // 2, int(self.H * 0.17)), self.font_mid, self.FG, "center")

        # Headlight icon
        if headlight:
            x, y = int(self.W * 0.08), int(self.H * 0.85)
            pygame.draw.circle(surf, self.FG, (x, y), self.scale(14), 2)
            for i in range(4):
                pygame.draw.line(surf, self.FG,
                                 (x + self.scale(18), y - self.scale(12) + self.scale(8) * i),
                                 (x + self.scale(36), y - self.scale(20) + self.scale(8) * i), 2)

        # Warning triangle
        if warn:
            x, y = int(self.W * 0.92), int(self.H * 0.85)
            pygame.draw.polygon(surf, self.DANGER,
                                [(x - self.scale(16), y + self.scale(16)),
                                 (x + self.scale(16), y + self.scale(16)),
                                 (x, y - self.scale(18))], self.scale(3))
            pygame.draw.line(surf, self.DANGER,
                             (x, y - self.scale(10)), (x, y + self.scale(6)), self.scale(3))
            pygame.draw.circle(surf, self.DANGER, (x, y + self.scale(12)), self.scale(3))
