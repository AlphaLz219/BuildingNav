#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build a complete Gazebo .world by embedding a model SDF.

Usage:
  python3 build_world.py [SDF_PATH] [OUT_PATH]

Defaults:
  SDF_PATH = /home/cjx/catkin_ws/src/py_hybrid_planner/maps2/maps2.sdf
  OUT_PATH = /home/cjx/catkin_ws/src/py_hybrid_planner/worlds/indoor.world
"""
import os
import re
import sys

DEFAULT_SDF = "/home/cjx/catkin_ws/src/py_hybrid_planner/maps2/maps2.sdf"
DEFAULT_OUT = "/home/cjx/catkin_ws/src/py_hybrid_planner/worlds/indoor.world"

def main():
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SDF
    out_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    with open(sdf_path) as f:
        sdf_text = f.read()

    m = re.search(r"<model\b.*?</model>", sdf_text, re.DOTALL)
    if not m:
        raise SystemExit("No <model>...</model> block found in " + sdf_path)
    model_block = m.group(0)

    world = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="indoor">

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>3.5 2.5 15.0 0 1.5708 0</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    {model_block}
  </world>
</sdf>
'''
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(world)
    print(f"Wrote {out_path}  size={os.path.getsize(out_path)}")

if __name__ == "__main__":
    main()
