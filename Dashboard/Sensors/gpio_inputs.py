# dashboard/sensors/gpio_inputs.py
"""
=============================================
 GPIO Inputs for Vehicle Dashboard
=============================================

Abstracts Raspberry Pi digital inputs (turn signals, headlights, etc.)
with debounce and active-low handling. Supports three backends:

  1) gpiozero  (recommended)
  2) RPi.GPIO  (fallback)
  3) Mock/keyboard (dev machines without GPIO)

Usage
-----
from sensors.gpio_inputs import GPIOConfig, GPIOInputs

cfg = GPIOConfig(
    pin_left=17, pin_right=27, pin_headlight=22,
    pin_hazard=None, pin_brake=None,
    active_low=True, debounce_ms=40
)
gpio = GPIOInputs(cfg)

snap = gpio.read()  # dict snapshot
print(snap["left"], snap["right"], snap["headlight"])

# Or read attributes:
print(gpio.left, gpio.right, gpio.headlight)


if self.gpio:
    snap = self.gpio.read()
    self.turn_left  = snap["left"]
    self.turn_right = snap["right"]
    self.headlight  = snap["headlight"]
    # You can also use snap["hazard"], snap["brake"], snap["ignition"]


Notes
-----
- Wire your switches to GPIO with pull-ups (active_low=True) or pull-downs.
- Debounce is software; for very noisy lines add hardware debounce if needed.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict
import time
import sys

# Try preferred libraries
try:
    import gpiozero  # type: ignore
except Exception:
    gpiozero = None

try:
    import RPi.GPIO as RPiGPIO  # type: ignore
except Exception:
    RPiGPIO = None


@dataclass
class GPIOConfig:
    # BCM pin numbers (None to disable a channel)
    pin_left: Optional[int] = 17
    pin_right: Optional[int] = 27
    pin_headlight: Optional[int] = None
    pin_hazard: Optional[int] = None
    pin_brake: Optional[int] = None
    pin_ignition: Optional[int] = None

    active_low: bool = True       # True if switch pulls to GND when ON
    debounce_ms: int = 40         # software debounce
    use_keyboard_mock: Optional[bool] = None  # force mock; None = auto on non-Pi

# ---------------- Backends ---------------- #

class _BackendBase:
    def __init__(self, cfg: GPIOConfig):
        self.cfg = cfg
        self._state: Dict[str, bool] = {
            "left": False, "right": False, "headlight": False,
            "hazard": False, "brake": False, "ignition": False,
        }
        self._last_raw: Dict[str, Optional[bool]] = {k: None for k in self._state}
        self._last_t: Dict[str, float] = {k: 0.0 for k in self._state}
        self._debounce_s = max(0.0, cfg.debounce_ms / 1000.0)

    def _debounced_set(self, key: str, raw_val: bool, now: float):
        prev_raw = self._last_raw[key]
        if prev_raw is None or raw_val != prev_raw:
            # edge: reset timer
            self._last_raw[key] = raw_val
            self._last_t[key] = now
            return  # wait out debounce period
        # stable long enough?
        if (now - self._last_t[key]) >= self._debounce_s:
            self._state[key] = raw_val

    # Must be implemented by concrete backends
    def _read_raw_channel(self, name: str) -> Optional[bool]:
        raise NotImplementedError

    def read(self) -> Dict[str, bool]:
        """Returns debounced, polarity-corrected dict of states."""
        now = time.time()
        names = list(self._state.keys())
        for name in names:
            raw = self._read_raw_channel(name)
            if raw is None:
                continue
            # Apply polarity: active_low means electrical LOW = True (ON)
            effective = (not raw) if self.cfg.active_low else bool(raw)
            self._debounced_set(name, effective, now)
        return dict(self._state)

    # Convenience properties
    @property
    def left(self): return self._state["left"]
    @property
    def right(self): return self._state["right"]
    @property
    def headlight(self): return self._state["headlight"]
    @property
    def hazard(self): return self._state["hazard"]
    @property
    def brake(self): return self._state["brake"]
    @property
    def ignition(self): return self._state["ignition"]


class _GPIOZeroBackend(_BackendBase):
    def __init__(self, cfg: GPIOConfig):
        super().__init__(cfg)
        self._pins = {
            "left": cfg.pin_left,
            "right": cfg.pin_right,
            "headlight": cfg.pin_headlight,
            "hazard": cfg.pin_hazard,
            "brake": cfg.pin_brake,
            "ignition": cfg.pin_ignition,
        }
        self._buttons = {}
        if gpiozero is None:
            raise RuntimeError("gpiozero not available")
        for name, pin in self._pins.items():
            if pin is None:
                continue
            # pull_up=True means internal pull-up enabled; adjust by active_low
            pull_up = cfg.active_low
            try:
                self._buttons[name] = gpiozero.Button(pin, pull_up=pull_up, bounce_time=None)
            except Exception as ex:
                print(f"[gpio_inputs] gpiozero.Button init failed for {name}@{pin}: {ex}")

    def _read_raw_channel(self, name: str) -> Optional[bool]:
        btn = self._buttons.get(name)
        if btn is None:
            return None
        # Button.is_pressed == True when circuit is pulled to ground (if pull_up True).
        # We want 'raw electrical level' (True=HIGH). Map is_pressed accordingly:
        if self.cfg.active_low:
            # pull-up: released => HIGH, pressed => LOW
            return not btn.is_pressed
        else:
            # pull-down: pressed => HIGH, released => LOW
            return btn.is_pressed


class _RPiGPIOBackend(_BackendBase):
    def __init__(self, cfg: GPIOConfig):
        super().__init__(cfg)
        if RPiGPIO is None:
            raise RuntimeError("RPi.GPIO not available")
        RPiGPIO.setmode(RPiGPIO.BCM)
        self._pins = {
            "left": cfg.pin_left,
            "right": cfg.pin_right,
            "headlight": cfg.pin_headlight,
            "hazard": cfg.pin_hazard,
            "brake": cfg.pin_brake,
            "ignition": cfg.pin_ignition,
        }
        pud = RPiGPIO.PUD_UP if cfg.active_low else RPiGPIO.PUD_DOWN
        for name, pin in self._pins.items():
            if pin is None:
                continue
            try:
                RPiGPIO.setup(pin, RPiGPIO.IN, pull_up_down=pud)
            except Exception as ex:
                print(f"[gpio_inputs] RPi.GPIO setup failed for {name}@{pin}: {ex}")

    def _read_raw_channel(self, name: str) -> Optional[bool]:
        pin = self._pins.get(name)
        if pin is None:
            return None
        try:
            level = RPiGPIO.input(pin)  # 0=LOW, 1=HIGH
            return bool(level)
        except Exception:
            return None


class _MockBackend(_BackendBase):
    """
    Dev backend: toggles via keyboard if stdin is a TTY.
    Keys: a=left, d=right, h=headlight, z=hazard, b=brake, i=ignition
    """
    def __init__(self, cfg: GPIOConfig):
        super().__init__(cfg)
        try:
            import termios, tty, select  # POSIX only
            self._termios = termios
            self._tty = tty
            self._select = select
            self._kb_ok = sys.stdin.isatty()
            if self._kb_ok:
                self._fd = sys.stdin.fileno()
                self._old = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
                print("[gpio_inputs] Using keyboard mock: a/d/h/z/b/i to toggle")
            else:
                print("[gpio_inputs] Mock backend without keyboard (non-tty stdin)")
        except Exception:
            self._kb_ok = False

    def _read_raw_channel(self, name: str) -> Optional[bool]:
        # Mock returns last debounced state; we only update on keypress
        self._poll_keyboard()
        # Convert current effective state back to "raw electrical high/low"
        effective = self._state[name]
        raw = (not effective) if self.cfg.active_low else effective
        return raw

    def _poll_keyboard(self):
        if not getattr(self, "_kb_ok", False):
            return
        try:
            if self._select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                mapping = {
                    "a": "left",
                    "d": "right",
                    "h": "headlight",
                    "z": "hazard",
                    "b": "brake",
                    "i": "ignition",
                }
                if ch in mapping:
                    key = mapping[ch]
                    self._state[key] = not self._state[key]
        except Exception:
            pass

    def __del__(self):
        # best-effort restore terminal
        if getattr(self, "_kb_ok", False):
            try:
                self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)
            except Exception:
                pass

# --------------- Factory --------------- #

class GPIOInputs(_BackendBase):
    """
    Public facade that chooses the best backend.
    Exposes .read() and properties (.left, .right, .headlight, .hazard, .brake, .ignition)
    """
    def __init__(self, cfg: GPIOConfig = GPIOConfig()):
        # Choose backend
        force_mock = (cfg.use_keyboard_mock is True) or (cfg.use_keyboard_mock is None and not _on_pi())
        if not force_mock and gpiozero is not None:
            self._impl = _GPIOZeroBackend(cfg)
        elif not force_mock and gpiozero is None and RPiGPIO is not None:
            self._impl = _RPiGPIOBackend(cfg)
        else:
            self._impl = _MockBackend(cfg)

        # Share state/props via composition
        self.cfg = cfg
        self._state = self._impl._state
        self._last_raw = self._impl._last_raw
        self._last_t = self._impl._last_t
        self._debounce_s = self._impl._debounce_s

    # delegate
    def _read_raw_channel(self, name: str):  # pragma: no cover
        return self._impl._read_raw_channel(name)

    def read(self) -> Dict[str, bool]:
        return self._impl.read()

    @property
    def left(self): return self._impl.left
    @property
    def right(self): return self._impl.right
    @property
    def headlight(self): return self._impl.headlight
    @property
    def hazard(self): return self._impl.hazard
    @property
    def brake(self): return self._impl.brake
    @property
    def ignition(self): return self._impl.ignition


def _on_pi() -> bool:
    """Heuristic: running on Linux with Raspberry Pi proc info."""
    try:
        if sys.platform != "linux":
            return False
        with open("/proc/cpuinfo", "r") as f:
            txt = f.read().lower()
        return ("raspberry pi" in txt) or ("bcm" in txt)
    except Exception:
        return False
