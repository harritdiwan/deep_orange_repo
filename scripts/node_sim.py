#!/usr/bin/env python

import os
import sys

# Change working directory
script_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_path)
sys.path.append(script_path)

# import pygame_sdl2
# pygame_sdl2.import_as_pygame()
from graphics.gui_main import App


if __name__ == "__main__":
    pyApp = App()
    pyApp.on_execute()
