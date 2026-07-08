# ros_plc_sim

Gazebo Harmonic simulation of a **FANUC CRX-10iA + Robotiq 2F-85** pick cell:
a real **friction conveyor** feeds a workpiece to a fixed pick spot, the robot picks it
and drops it into a **collection box on a pedestal**. Built to be driven from a **PLC
(OpenPLC Runtime)** over **Modbus TCP**.

- ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim 8)
- Depends on `fanuc_crx_description`, `robotiq_description`, `ros_gz_sim`, `ros_gz_bridge`,
  `gz_ros2_control`, plus the usual controllers.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/arms_ws/install/setup.bash
ros2 launch ros_plc_sim crx_gripper_gz_sim.launch.py
```

The workspace uses a **colcon symlink install**, so edits to `worlds/`, `launch/` and
`config/` take effect on the next launch with no rebuild.

## Cell layout (world = `worlds/gripper_cell.sdf`)

| Model | What it is | Key pose |
|---|---|---|
| `belt_track` | Friction belt, top face at **z=0.55**, driven by TrackController | center x=1.15, y=0.35 |
| `conveyor_end_stop` | End stop lip + contact sensor at the delivery end | x=0.40, y=0.35 |
| `collection_box` | Open crate on a 4-leg pedestal (rim ≈ z=0.62) | x=0.0, y=-0.45 |
| `workpiece_1` | 40 mm cube, spawns at the belt feed end | x=1.75, y=0.35 |

The belt drives the part along **−X** from the feed end (x≈1.75) to the end stop (x≈0.42),
which is where the old pickup table used to be — i.e. the robot's pick pose.

---

## How the conveyor works

The belt is **not** teleported. `belt_track` is a static slab whose **contact surface
velocity** is set by the gz-sim `TrackController` plugin; the workpiece rides on top by
friction. The end rollers have `radius = belt_height/2 = 0.05`, so their crowns are flush
with the belt top (no bump at the ends).

**Command topic** (gz transport):

```
/model/belt_track/link/base_link/track_cmd_vel      type: gz.msgs.Double   (m/s)
```

- **negative** → belt moves the part toward the robot (−X)
- **0** → stop
- **positive** → reverse (away from the robot)

Drive it by hand:

```bash
# run at 0.15 m/s toward the robot
gz topic -t /model/belt_track/link/base_link/track_cmd_vel -m gz.msgs.Double -p "data: -0.15"
# stop
gz topic -t /model/belt_track/link/base_link/track_cmd_vel -m gz.msgs.Double -p "data: 0.0"
```

**Auto-start:** the launch file publishes `data: -0.15` once, ~6 s after startup
(a `TimerAction` + `ExecuteProcess` in `crx_gripper_gz_sim.launch.py`), so the belt runs by
default. TrackController holds the last commanded speed until a new value arrives.

## How the end sensor works

`conveyor_end_stop` carries a **contact sensor** (`end_contact`). It is a **level** signal:
it reports contacts continuously while the workpiece is pressed against the stop, and reports
nothing when the pick spot is clear.

| Layer | Topic | Type |
|---|---|---|
| gz | `/world/gripper_cell/model/conveyor_end_stop/link/stop/sensor/end_contact/contact` | `gz.msgs.Contacts` |
| ROS | `/conveyor_end_sensor/contacts` | `ros_gz_interfaces/msg/Contacts` |

`part_present` = `len(msg.contacts) > 0`. (A gz `TouchPlugin` was tried first but it is a
one-shot latch, not a level signal, so it was replaced by the contact sensor.)

The ROS bridge (`config/gz_bridge.yaml`, run by `ros_gz_bridge`) currently bridges `/clock`
and this sensor.

---

## Controlling it from OpenPLC Runtime (Modbus TCP)

OpenPLC Runtime talks **Modbus TCP**, not ROS. The bridge between them is a small ROS node
that exposes a **Modbus TCP slave**; OpenPLC is configured to poll it as a *slave device* and
maps its coils/inputs to PLC variables. Data flow:

```
 OpenPLC Runtime  ──Modbus TCP──►  plc_bridge (ROS node, Modbus slave)  ──►  Gazebo
   (ladder/ST)     ◄──────────────  (rclpy + pymodbus)                  ◄──  sim
```

### Signal / address map (suggested)

| Signal | PLC var | Modbus object | Meaning |
|---|---|---|---|
| `belt_run` | `%QX0.0` (coil, PLC output) | coil 0 | 1 = run belt (−0.15 m/s), 0 = stop |
| `part_present` | `%IX0.0` (discrete input) | discrete input 0 | 1 = workpiece at pick spot |
| `belt_speed` *(optional)* | `%QW0` (holding reg) | holding reg 0 | analog belt speed setpoint |

The `plc_bridge` node maps these to the sim:

- **coil `belt_run` → belt:** when it changes, publish `data: -0.15` (run) or `data: 0.0`
  (stop) to `/model/belt_track/link/base_link/track_cmd_vel`.
- **contact sensor → discrete input `part_present`:** subscribe `/conveyor_end_sensor/contacts`,
  set the Modbus discrete input to `len(contacts) > 0`.

### Bring-up steps

1. **Build the bridge** (`plc_bridge` node — *not yet in this repo*, see below). It runs a
   pymodbus TCP slave on `0.0.0.0:502` and does the mapping above.
2. **OpenPLC Runtime** → web UI → *Slave Devices* → add a **Modbus TCP** device pointing at the
   bridge's IP:502, and assign its coil/discrete-input to the `%QX0.0` / `%IX0.0` addresses.
3. **Write the PLC program** (Ladder or ST). Minimal example logic:
   *run the belt until the part reaches the end, then stop it.*
   ```
   belt_run := NOT part_present;   (* keep feeding until a part is at the stop *)
   ```
4. **Start** the sim + the `plc_bridge` node, then upload & run the PLC program in OpenPLC.

### What already exists vs. still to build

- ✅ Belt command topic, auto-start, flush rollers.
- ✅ Level part-present sensor, bridged to ROS.
- ⏳ `plc_bridge` ROS↔Modbus node — the piece that connects OpenPLC to the two topics above.
- ⏳ Bridging the belt command to a plain ROS topic (so the bridge doesn't have to call gz
  transport directly) — optional convenience.

> Ask if you want the `plc_bridge` node + a starter OpenPLC program scaffolded next.
