"""Capture only a specified Gazebo window for local scene visual QA (X11)."""
import argparse
import subprocess
import time
from pathlib import Path
from PIL import ImageGrab


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('window', type=int)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    title = subprocess.check_output(['xdotool', 'getwindowname', str(args.window)], text=True).strip()
    if title != 'Gazebo Sim':
        raise ValueError('Only the Gazebo Sim window may be captured')
    subprocess.run(['xdotool', 'windowsize', str(args.window), '1100', '800'], check=True)
    subprocess.run(['xdotool', 'windowmove', str(args.window), '100', '80'], check=True)
    subprocess.run(['xdotool', 'windowactivate', '--sync', str(args.window)], check=True)
    time.sleep(.5)
    geometry = subprocess.check_output(['xdotool', 'getwindowgeometry', '--shell', str(args.window)], text=True)
    values = dict(line.split('=', 1) for line in geometry.splitlines() if '=' in line)
    x, y, width, height = [int(values[k]) for k in ('X', 'Y', 'WIDTH', 'HEIGHT')]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ImageGrab.grab(bbox=(x, y, x+width, y+height)).save(args.output)


if __name__ == '__main__':
    main()
