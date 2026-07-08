#!/usr/bin/env python3
"""Generate conveyor_compare.sdf: three physics-driven conveyors side by side,
each carrying a cube, so we can compare and pick one.

  Lane y=+0.9 : flat friction belt   -> gz-sim TrackController (surface velocity)
  Lane y= 0.0 : powered roller table  -> revolute rollers + JointController
  Lane y=-0.9 : Fuel conveyor_block   -> mesh visual + TrackController belt surface

Belt top surface for all three sits at z = 0.50.
"""

GLB = "/root/.gz/fuel/fuel.gazebosim.org/open-rmf/models/conveyor_block/4/meshes/conveyor_block_visual.glb"

BELT_TOP = 0.50


def leg(name, x, y, z_top, size=0.05):
    h = z_top
    return f"""      <link name="{name}">
        <pose>{x} {y} {h/2:.4f} 0 0 0</pose>
        <visual name="visual">
          <geometry><box><size>{size} {size} {h:.4f}</size></box></geometry>
          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material>
        </visual>
      </link>"""


def cube(name, x, y, rgb):
    z = BELT_TOP + 0.031
    r, g, b = rgb
    return f"""    <model name="{name}">
      <pose>{x} {y} {z:.3f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.2</mass>
          <inertia><ixx>0.00012</ixx><iyy>0.00012</iyy><izz>0.00012</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.06 0.06 0.06</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.06 0.06 0.06</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


# ---------------------------------------------------------------- Conveyor 1
def belt_track():
    cz = BELT_TOP - 0.05  # box half-height 0.05 -> top at BELT_TOP
    legs = "\n".join(
        leg(f"leg_{i}", x, 0.0, cz - 0.05)
        for i, x in enumerate((-0.7, -0.7, 0.7, 0.7))
    )
    # override the y of legs manually below by regenerating (simpler inline):
    legs = "\n".join(
        leg(f"leg_{i}", x, y, cz - 0.05)
        for i, (x, y) in enumerate([(-0.7, -0.12), (-0.7, 0.12), (0.7, -0.12), (0.7, 0.12)])
    )
    return f"""  <!-- ===== Conveyor 1: flat friction belt (TrackController) ===== -->
  <model name="belt_track">
    <pose>0 0.9 0 0 0 0</pose>
    <static>1</static>
    <link name="base_link">
      <pose>0 0 {cz:.3f} 0 0 0</pose>
      <collision name="belt_col">
        <geometry><box><size>1.5 0.30 0.10</size></box></geometry>
        <surface><friction><ode><mu>0.7</mu><mu2>150</mu2><fdir1>0 1 0</fdir1></ode></friction></surface>
      </collision>
      <visual name="belt_vis">
        <geometry><box><size>1.5 0.30 0.10</size></box></geometry>
        <material><ambient>0.05 0.05 0.6 1</ambient><diffuse>0.08 0.08 0.75 1</diffuse></material>
      </visual>
      <collision name="roll_a_col">
        <pose>0.75 0 0 -1.5708 0 0</pose>
        <geometry><cylinder><length>0.30</length><radius>0.06</radius></cylinder></geometry>
      </collision>
      <visual name="roll_a_vis">
        <pose>0.75 0 0 -1.5708 0 0</pose>
        <geometry><cylinder><length>0.30</length><radius>0.06</radius></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="roll_b_col">
        <pose>-0.75 0 0 -1.5708 0 0</pose>
        <geometry><cylinder><length>0.30</length><radius>0.06</radius></cylinder></geometry>
      </collision>
      <visual name="roll_b_vis">
        <pose>-0.75 0 0 -1.5708 0 0</pose>
        <geometry><cylinder><length>0.30</length><radius>0.06</radius></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="lip_col"><pose>0.74 0 0.05 0 0 0</pose><geometry><box><size>0.02 0.30 0.10</size></box></geometry></collision>
      <visual name="lip_vis"><pose>0.74 0 0.05 0 0 0</pose><geometry><box><size>0.02 0.30 0.10</size></box></geometry>
        <material><ambient>0.6 0.1 0.1 1</ambient><diffuse>0.7 0.12 0.12 1</diffuse></material></visual>
    </link>
{legs}
    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>base_link</link>
    </plugin>
  </model>"""


# ---------------------------------------------------------------- Conveyor 2
def roller_conv():
    rz = BELT_TOP - 0.05  # roller radius 0.05 -> top at BELT_TOP
    xs = [round(-0.54 + i * 0.12, 3) for i in range(10)]  # gap 0.02 << cube 0.06
    rails = f"""      <collision name="rail_l_col"><pose>0 0.19 {rz:.3f} 0 0 0</pose><geometry><box><size>1.4 0.03 0.10</size></box></geometry></collision>
      <visual name="rail_l_vis"><pose>0 0.19 {rz:.3f} 0 0 0</pose><geometry><box><size>1.4 0.03 0.10</size></box></geometry><material><ambient>0.5 0.35 0.05 1</ambient><diffuse>0.6 0.4 0.06 1</diffuse></material></visual>
      <collision name="rail_r_col"><pose>0 -0.19 {rz:.3f} 0 0 0</pose><geometry><box><size>1.4 0.03 0.10</size></box></geometry></collision>
      <visual name="rail_r_vis"><pose>0 -0.19 {rz:.3f} 0 0 0</pose><geometry><box><size>1.4 0.03 0.10</size></box></geometry><material><ambient>0.5 0.35 0.05 1</ambient><diffuse>0.6 0.4 0.06 1</diffuse></material></visual>
      <collision name="lip_col"><pose>0.60 0 {rz+0.05:.3f} 0 0 0</pose><geometry><box><size>0.02 0.40 0.10</size></box></geometry></collision>
      <visual name="lip_vis"><pose>0.60 0 {rz+0.05:.3f} 0 0 0</pose><geometry><box><size>0.02 0.40 0.10</size></box></geometry><material><ambient>0.6 0.1 0.1 1</ambient><diffuse>0.7 0.12 0.12 1</diffuse></material></visual>"""
    frame_legs = "\n".join(
        leg(f"fleg_{i}", x, y, rz - 0.05)
        for i, (x, y) in enumerate([(-0.6, -0.19), (-0.6, 0.19), (0.6, -0.19), (0.6, 0.19)])
    )
    rollers = []
    for i, x in enumerate(xs):
        rollers.append(f"""    <link name="roller_{i}">
      <pose>{x} 0 {rz:.3f} 1.5708 0 0</pose>
      <inertial><mass>0.3</mass><inertia><ixx>0.0006</ixx><iyy>0.0006</iyy><izz>0.0004</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="col"><geometry><cylinder><length>0.34</length><radius>0.05</radius></cylinder></geometry>
        <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>
      <visual name="vis"><geometry><cylinder><length>0.34</length><radius>0.05</radius></cylinder></geometry>
        <material><ambient>0.55 0.55 0.58 1</ambient><diffuse>0.7 0.7 0.72 1</diffuse></material></visual>
    </link>
    <joint name="roller_{i}_joint" type="revolute">
      <parent>frame_link</parent><child>roller_{i}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><friction>0.0</friction></dynamics></axis>
    </joint>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>roller_{i}_joint</joint_name>
      <initial_velocity>-4.0</initial_velocity>
      <p_gain>0.4</p_gain>
    </plugin>""")
    rollers = "\n".join(rollers)
    return f"""  <!-- ===== Conveyor 2: powered roller conveyor (spinning rollers) ===== -->
  <model name="roller_conv">
    <pose>0 0 0 0 0 0</pose>
    <link name="frame_link">
{rails}
    </link>
    <joint name="frame_fixed" type="fixed"><parent>world</parent><child>frame_link</child></joint>
{frame_legs}
{rollers}
  </model>"""


# ---------------------------------------------------------------- Conveyor 3
def belt_mesh():
    cz = BELT_TOP - 0.02  # belt-surface collision half-height 0.02 -> top at BELT_TOP
    meshes = "\n".join(
        f"""      <visual name="seg_{i}">
        <pose>{x} 0 0 0 0 0</pose>
        <geometry><mesh><uri>file://{GLB}</uri></mesh></geometry>
      </visual>"""
        for i, x in enumerate((-0.26, 0.26))
    )
    return f"""  <!-- ===== Conveyor 3: Fuel conveyor_block mesh + TrackController ===== -->
  <model name="belt_mesh">
    <pose>0 -0.9 0 0 0 0</pose>
    <static>1</static>
    <link name="base_link">
{meshes}
      <collision name="belt_col">
        <pose>0 0 {cz:.3f} 0 0 0</pose>
        <geometry><box><size>1.02 0.45 0.04</size></box></geometry>
        <surface><friction><ode><mu>0.7</mu><mu2>150</mu2><fdir1>0 1 0</fdir1></ode></friction></surface>
      </collision>
      <collision name="lip_col"><pose>0.50 0 {BELT_TOP+0.03:.3f} 0 0 0</pose><geometry><box><size>0.02 0.45 0.06</size></box></geometry></collision>
      <visual name="lip_vis"><pose>0.50 0 {BELT_TOP+0.03:.3f} 0 0 0</pose><geometry><box><size>0.02 0.45 0.06</size></box></geometry>
        <material><ambient>0.6 0.1 0.1 1</ambient><diffuse>0.7 0.12 0.12 1</diffuse></material></visual>
    </link>
    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>base_link</link>
    </plugin>
  </model>"""


WORLD = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="conveyor_compare">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene><ambient>1 1 1</ambient><background>0.8 0.85 0.9</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

{belt_track()}

{roller_conv()}

{belt_mesh()}

{cube("cube_track", -0.6, 0.9, (0.85, 0.1, 0.1))}

{cube("cube_roller", -0.5, 0.0, (0.1, 0.7, 0.15))}

{cube("cube_mesh", -0.4, -0.9, (0.95, 0.8, 0.05))}
  </world>
</sdf>
"""

if __name__ == "__main__":
    import os
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "conveyor_compare.sdf")
    with open(out, "w") as f:
        f.write(WORLD)
    print("wrote", out, len(WORLD), "bytes")
