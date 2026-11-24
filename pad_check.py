"""Read-only DS4 mapping check: prompts for each control, reports which axis/button index moved.
Does not touch the arm or the serial port. Run: python pad_check.py"""
import os
import time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

pygame.display.init()
pygame.joystick.init()
if not pygame.joystick.get_count():
    raise SystemExit("No controller found.")
js = pygame.joystick.Joystick(0)
print("Controller:", js.get_name(), "| axes", js.get_numaxes(), "| buttons", js.get_numbuttons(), "| hats", js.get_numhats())

STEPS = [("axis", "Push LEFT stick fully RIGHT and hold"), ("axis", "Push LEFT stick fully DOWN (toward you) and hold"),
         ("axis", "Push RIGHT stick fully RIGHT and hold"), ("axis", "Push RIGHT stick fully DOWN and hold"),
         ("axis", "Squeeze L2 fully and hold"), ("axis", "Squeeze R2 fully and hold"),
         ("button", "Press and hold L1"), ("button", "Press PS button"), ("button", "Press OPTIONS"),
         ("button", "Press D-pad UP"), ("button", "Press D-pad DOWN"), ("button", "Press D-pad LEFT"),
         ("button", "Press D-pad RIGHT")]


def snap():
    pygame.event.pump()
    return [js.get_axis(i) for i in range(js.get_numaxes())], [js.get_button(i) for i in range(js.get_numbuttons())]


def wait_release(base_ax):
    t = time.time() + 5
    while time.time() < t:
        ax, bt = snap()
        if not any(bt) and all(abs(a - b) < 0.3 for a, b in zip(ax, base_ax)):
            return
        time.sleep(0.05)


import sys  # noqa: E402
if "sticks" in sys.argv:  # re-check only the four stick steps
    STEPS = STEPS[:4]
print("Get ready; the first prompt comes in 3 seconds.")
time.sleep(3)
base_ax, _ = snap()
for kind, prompt in STEPS:
    print(f"\n>> {prompt}  (10 s)", flush=True)
    found = None
    end = time.time() + 10
    while time.time() < end and not found:
        ax, bt = snap()
        if kind == "axis":
            d = [(abs(a - b), i, a) for i, (a, b) in enumerate(zip(ax, base_ax))]
            m = max(d)
            if m[0] > 0.7:
                found = f"axis {m[1]} value {m[2]:+.2f}"
        else:
            p = [i for i, b in enumerate(bt) if b]
            if p:
                found = f"button {p}"
        time.sleep(0.05)
    print("   ->", found or "nothing detected", flush=True)
    wait_release(base_ax)
    time.sleep(0.4)
print("\nDone.")
