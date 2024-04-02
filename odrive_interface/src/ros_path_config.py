# ros_path_config.py
import sys
import os

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append(os.path.join(currentdir, "../.."))
