"""
=============================================
 Vehicle Dashboard Data Model
=============================================

This module defines the DataModel class: a single source of truth
for all runtime values used by the UI (speed, SOC, temp, indicators).
By default it simulates data so the app is runnable out-of-the-box.
Later, you can switch to real sensors (CAN/I2C/GPIO) with minimal changes.

Usage
-----
from model import DataModel
model = DataModel(units="km/h")  # or "mph"
while True:
    model.update(dt)             # dt in seconds
    # read model.speed, model.soc, ...

Public attributes (read by UI)
------------------------------
model.speed (float)     : vehicle speed in configured units (km/h or mph)
model.soc (float)       : battery state of charge, percent [0..100]
model.temp (float)      : controller/motor temperature in °C
model.turn_left (bool)  : left turn indicator
model.turn_right (bool) : right turn indicator
model.headlight (bool)  : headlight status
model.gear (str)        : "P", "R", "N", "D", ...
model.warn (bool)       : general warning light

Configuration knobs
-------------------
- Start in simulation:    DataModel(simulate=True)           (default)
- Switch to real sensors: DataModel(simulate=False, enable_can=True, enable_i2c=True, enable_gpio=True)
- CAN IDs & scaling:      pass can_config=... (see CANConfig below)

Notes
-----
- All external imports (python-can, adafruit_ina219, gpiozero) are optional
  and only attempted if the corresponding enable_* flag is True.
- Minimal low-pass filters smooth noisy inputs.
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import Optional


# -------------------- Optional Interfaces (lazy) --------------------
def _try_import(modname: str):
    try:
        return __import__(modname, fromlist=["*"])
    except Exception:
        return None


# -------------------- Small helpers --------------------
class LowPass:
    """Simple 1st-order low-pass filter: y += alpha * (x - y); alpha in (0,1]."""
    def __init__(self, alpha: float, init: float = 0.0):
        self.alpha = max(0.0, min(1.0, alpha))
        self.y = float(init)

    def reset(self, value: float):
        self.y = float(value)

    def step(self, x: float) -> float:
        self.y += self.alpha * (x - self.y)
        return self.y


@dataclass
class CANConfig:
    """Map CAN arbitration IDs to signals and provide scaling functions."""
    channel: str = "can0"
    bustype: str = "socketcan"  # MCP2515 on Pi via SocketCAN
    bitrate: int = 500000

    # Example message mapping
    id_speed: int = 0x123   # 2 bytes (0..65535), scale -> km/h
    id_gear: int = 0x124    # 1 byte ASCII or enum
    id_temp: int = 0x125    # 1 byte °C
    id_blink: int = 0x126   # bits: L/R indicators

    def decode_speed(self, data: bytes) -> float:
        """Return speed in km/h from CAN payload (example scaling)."""
        if len(data) < 2:
            return 0.0
        raw = int.from_bytes(data[0:2], "big")
        return raw * 0.01  # 1 unit = 0.01 km/h

    def decode_gear(self, data: bytes) -> str:
        if not data:
            return "N"
        code = data[0]
        mapping = {0: "P", 1: "R", 2: "N", 3: "D"}
        return mapping.get(code, "N")

    def decode_temp(self, data: bytes) -> float:
        if not data:
            return 25.0
        return float(data[0])  # already °C in this example

    def decode_blinkers(self, data: bytes) -> tuple[bool, bool]:
        if not data:
            return False, False
        b = data[0]
        return bool(b & 0x01), bool(b & 0x02)  # LSB: left, next bit: right


@dataclass
class BatteryConfig:
    """I2C battery monitor config; used with INA219/INA260, etc."""
    i2c_bus: Optional[int] = None  # auto
    shunt_ohms: float = 0.1        # for coulomb-counting if you add it
    nominal_wh: float = 500.0      # pack energy for range/SOC estimation
    ocv_soc_enabled: bool = False  # set True if you implement an OCV curve
    # You can add an OCV table later: List[Tuple[voltage, soc]]


@dataclass
class GPIOConfig:
    pin_left: int = 17
    pin_right: int = 27
    pin_headlight: Optional[int] = None  # if None, leave as-is


# -------------------- DataModel --------------------
class DataModel:
    def __init__(self,
                 units: str = "km/h",
                 simulate: bool = True,
                 enable_can: bool = False,
                 enable_i2c: bool = False,
                 enable_gpio: bool = False,
                 can_config: Optional[CANConfig] = None,
                 bat_config: Optional[BatteryConfig] = None,
                 gpio_config: Optional[GPIOConfig] = None):
        # Units
        self.units = units.lower().strip()  # "km/h" or "mph"
        self._kmh_to_mph = 0.621371

        # Public state (UI reads these)
        self.speed: float = 0.0    # in configured units
        self.soc: float = 65.0     # %
        self.temp: float = 42.0    # °C
        self.turn_left: bool = False
        self.turn_right: bool = False
        self.headlight: bool = True
        self.gear: str = "D"
        self.warn: bool = False

        # Simulation & timing
        self.simulate = simulate
        self._t0 = time.time()

        # Filters for smoothing
        self._f_speed = LowPass(alpha=0.25, init=self.speed)
        self._f_temp = LowPass(alpha=0.10, init=self.temp)
        self._f_soc  = LowPass(alpha=0.02, init=self.soc)

        # Configs
        self.can_cfg = can_config or CANConfig()
        self.bat_cfg = bat_config or BatteryConfig()
        self.gpio_cfg = gpio_config or GPIOConfig()

        # Optional backends (initialized only if enabled_* is True)
        self._can = None
        self._gpio = None
        self._ina = None

        if not self.simulate:
            if enable_can:
                self._setup_can()
            if enable_i2c:
                self._setup_i2c()
            if enable_gpio:
                self._setup_gpio()

    # -------------- Setup backends --------------
    def _setup_can(self):
        can = _try_import("can")
        if not can:
            print("[model] python-can not installed; CAN disabled.")
            return
        try:
            self._can = can.interface.Bus(
                channel=self.can_cfg.channel,
                bustype=self.can_cfg.bustype,
                bitrate=self.can_cfg.bitrate
            )
            print(f"[model] CAN bus up on {self.can_cfg.channel} ({self.can_cfg.bitrate} bps)")
        except Exception as ex:
            print(f"[model] Failed to open CAN: {ex}")
            self._can = None

    def _setup_i2c(self):
        board = _try_import("board")
        busio = _try_import("busio")
        adafruit_ina219 = _try_import("adafruit_ina219")
        if not (board and busio and adafruit_ina219):
            print("[model] I2C or INA219 libs not installed; battery monitor disabled.")
            return
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._ina = adafruit_ina219.INA219(i2c)
            print("[model] INA219 battery monitor initialized")
        except Exception as ex:
            print(f"[model] INA219 init failed: {ex}")
            self._ina = None

    def _setup_gpio(self):
        gpiozero = _try_import("gpiozero")
        if not gpiozero:
            print("[model] gpiozero not installed; GPIO inputs disabled.")
            return
        try:
            self._gpio = {
                "left": gpiozero.Button(self.gpio_cfg.pin_left, pull_up=True),
                "right": gpiozero.Button(self.gpio_cfg.pin_right, pull_up=True),
            }
            if self.gpio_cfg.pin_headlight is not None:
                self._gpio["head"] = gpiozero.Button(self.gpio_cfg.pin_headlight, pull_up=True)
            print("[model] GPIO inputs initialized")
        except Exception as ex:
            print(f"[model] GPIO init failed: {ex}")
            self._gpio = None

    # -------------- Public API --------------
    def set_units(self, units: str):
        """Change display units at runtime; converts current speed accordingly."""
        units = units.lower().strip()
        if units == self.units:
            return
        if units not in ("km/h", "mph"):
            return
        # convert current speed to new units
        if self.units == "km/h" and units == "mph":
            self.speed *= self._kmh_to_mph
        elif self.units == "mph" and units == "km/h":
            self.speed /= self._kmh_to_mph
        self.units = units

    def update(self, dt: float):
        """Update all values once per frame. dt is time delta in seconds."""
        if self.simulate:
            self._simulate(dt)
        else:
            self._read_sensors(dt)

        # Basic warning logic
        self.warn = (self.soc < 15.0) or (self.temp > 90.0)

    # -------------- Internals --------------
    def _simulate(self, dt: float):
        """Nice-looking demo motion for the UI."""
        t = time.time() - self._t0

        # Speed oscillates between ~20..100 km/h
        speed_kmh = 60.0 + 40.0 * math.sin(t * 0.6)

        # SOC slowly drains
        soc_target = max(0.0, self.soc - 0.03 * dt * 60.0)  # ~0.03%/s ~ 1.8%/min

        # Temperature wiggle 30..55 C
        temp_target = 40.0 + 15.0 * math.sin(t * 0.3)

        # Indicators blink on simple timers
        self.turn_left  = (int(t * 1.0) % 2) == 0
        self.turn_right = (int(t * 0.7) % 2) == 0
        self.headlight  = True
        self.gear = ["P", "R", "N", "D"][int(t / 5) % 4]

        # Apply filters and unit conversion
        speed_kmh = self._f_speed.step(speed_kmh)
        temp_sm   = self._f_temp.step(temp_target)
        soc_sm    = self._f_soc.step(soc_target)

        self.temp = float(temp_sm)
        self.soc  = float(soc_sm)

        if self.units == "km/h":
            self.speed = float(speed_kmh)
        else:
            self.speed = float(speed_kmh * self._kmh_to_mph)

    def _read_sensors(self, dt: float):
        """Poll enabled backends. Missing backends fall back to last value."""
        # ---- CAN (speed, gear, temp, blinkers) ----
        if self._can is not None:
            self._poll_can()

        # ---- I2C battery monitor (voltage/current -> SOC) ----
        if self._ina is not None:
            self._poll_battery(dt)

        # ---- GPIO indicators ----
        if self._gpio is not None:
            try:
                left  = not self._gpio["left"].is_pressed
                right = not self._gpio["right"].is_pressed
                self.turn_left  = bool(left)
                self.turn_right = bool(right)
                if "head" in self._gpio:
                    self.headlight = not self._gpio["head"].is_pressed
            except Exception:
                pass  # keep last values

        # Clamp sanity
        self.soc = max(0.0, min(100.0, self.soc))

    def _poll_can(self):
        """Non-blocking CAN read; decode known IDs; apply smoothing & unit conversion."""
        try:
            # Use timeout=0 for non-blocking; pull all pending messages quickly
            msg = self._can.recv(timeout=0.0)
            cnt = 0
            while msg and cnt < 20:  # limit per frame
                arb = msg.arbitration_id
                data = bytes(msg.data)

                if arb == self.can_cfg.id_speed:
                    kmh = self.can_cfg.decode_speed(data)
                    kmh = self._f_speed.step(kmh)
                    self.speed = kmh if self.units == "km/h" else kmh * self._kmh_to_mph

                elif arb == self.can_cfg.id_gear:
                    self.gear = self.can_cfg.decode_gear(data)

                elif arb == self.can_cfg.id_temp:
                    self.temp = self._f_temp.step(self.can_cfg.decode_temp(data))

                elif arb == self.can_cfg.id_blink:
                    left, right = self.can_cfg.decode_blinkers(data)
                    self.turn_left, self.turn_right = left, right

                msg = self._can.recv(timeout=0.0)
                cnt += 1

        except Exception:
            # If CAN hiccups, keep last values
            pass

    def _poll_battery(self, dt: float):
        """Read voltage/current; estimate SOC (very simple placeholder)."""
        try:
            volts = float(self._ina.bus_voltage)              # V
            amps  = float(self._ina.current) / 1000.0         # A (mA -> A)
            # Placeholder: SOC estimate from voltage only (NOT accurate for load).
            soc_est = self._estimate_soc_from_voltage(volts)
            self.soc = self._f_soc.step(soc_est)
        except Exception:
            pass

    # -------------------- Battery SOC estimation (placeholder) --------------------
    def _estimate_soc_from_voltage(self, v: float) -> float:
        """
        Naive Li-ion pack OCV model (single cell equivalent).
        Replace with your pack's OCV curve under rest conditions.
        Assumes 3.2V (0%) .. 4.2V (100%), clamps outside.
        If you have N-series cells, pass per-cell volts or refine this model.
        """
        v_cell = max(3.0, min(4.25, v))  # clamp for safety
        soc = (v_cell - 3.2) / (4.2 - 3.2) * 100.0
        return max(0.0, min(100.0, soc))
