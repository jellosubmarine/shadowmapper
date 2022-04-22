import open3d as o3d
import numpy as np
import os
import time

import re

def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """
    l.sort(key=alphanum_key)

shadow_files = [k for k in os.listdir() if 'shadow' in k]
sort_nicely(shadow_files)

####

vis = o3d.visualization.Visualizer()
vis.create_window()

geometry = o3d.io.read_point_cloud("points.ply")
vis.add_geometry(geometry)

for i in shadow_files:
    illu = np.load(i)
    geometry.colors = o3d.utility.Vector3dVector((np.ones((3,illu.shape[0]))*illu).T)
    vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(i.split(".")[0]+".png")
vis.destroy_window()