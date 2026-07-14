# FANUC CRX gripper cell — Build & Run runbook

End-to-end pick-&-place demo: **OpenPLC → ROS 2 cell (Gazebo + MoveIt) → WebSocket bridge → web HMI**,
with an AUTO/MANUAL mode selector.

---

## 1. Architecture / data flow

```
        ┌───────────────────────────── OpenPLC runtime ─────────────────────────────┐
        │  program.st: FB_CellCycle (AUTO seq) + FB_CellManual, gated by auto_mode   │
        │  exposes over OPC UA: belt_run, robot_start, state, auto_mode(rw),         │
        │                       robot_busy(rw), cycle_done(rw), man_*(rw)            │
        └───────────▲───────────────────────────────────────────────▲───────────────┘
            OPC UA  │ 4840                                   Modbus   │
                    │                                                 │
             plc_bridge (bridge_node)                     modbus_sensor_bridge
             ── /plc/belt_run, /plc/robot_start ──┐        ▲ /sensors/part_present, motion, door
             ── /plc/state (Int32) ──────────┐    │        │
             ◄─ /plc/write/{auto_mode,        │    │        │
                robot_busy,cycle_done,man_*}  │    ▼        │
                                          cell_mode ──/cmd/*──► cell_io ──► Gazebo cell + MoveIt
        HMI ─/manual/*, /cell/set_mode──────────┘   (arbiter)   │  (robot arm + Robotiq gripper,
         ▲          │ /cell/mode ◄──────────────────────────────┘   conveyor); publishes
         │          │                                                /sensors/part_present,
         │   ros_gui_bridge  ◄── to_gui: /cmd/*, /plc/state,         /plc/write/{robot_busy,cycle_done}
         │   (WebSocket :9093)     /cell/mode, /joint_states, ...    joint_state_broadcaster ─/joint_states
         │          ▲
         └── React HMI (bun/vite :8080)  ── ws://<host>:9093/ros
```

**Modes.** `cell_mode` arbitrates the effective command (`/cmd/*` → `cell_io`):
- **AUTO** → follows the PLC (`/plc/*`); the PLC runs `FB_CellCycle`.
- **MANUAL** → follows the HMI (`/manual/*`); fail-safe (cell idle until the operator commands). Default.
- Mode is set from the HMI via the `/cell/set_mode` service and echoed on `/cell/mode`;
  `cell_mode` also forwards it to the PLC (`/plc/write/auto_mode`).

---

## 2. What was built (this integration)

- **`ros_gui_bridge`** (CroboticSolutions, branch `ros2-rclpy-port`): ported ROS 1 → **ROS 2 (rclpy)**,
  ament_python package. Type-agnostic msg↔JSON via `rosidl_runtime_py`; NaN/Inf scrubbed to `null`;
  per-topic rate limiting (e.g. `/joint_states` capped at 20 Hz). Config points at the CRX cell.
- **`plc_bridge`** (bb53192/plc-ros2-bridge, branch `crx-cell-integration`): now supports **non-BOOL**
  OPC UA variables (Bool / Int32 / Float64) — enables `/plc/state` (INT). PLC program gained
  `FB_CellCycle` + `FB_CellManual` and the `auto_mode` gate; `opcua.json` exposes the new vars.
- **`ros_plc_sim`** (bb53192, branch `conveyor-belt-and-end-sensor`): new **`cell_mode`** AUTO/MANUAL
  arbiter node, wired into `crx_gripper_plc.launch.py` (remaps `cell_io` inputs to `/cmd/*`).
- **`idustrial_demo_gui`** (bb53192, branch `hmi-live-controls`): live WebSocket feed (replaces the
  Lovable sim), authoritative sequencer state from `/plc/state`, AUTO/MANUAL toggle + manual belt/robot
  controls, and a new **Robot** tab (joint positions, gripper, pick/place phase).

---

## 3. One-time setup

```bash
# ROS 2 Jazzy + Gazebo Harmonic + MoveIt assumed installed.
# Python deps for the bridges:
pip3 install --break-system-packages websockets asyncua pyyaml

# Bun (for the GUI):
curl -fsSL https://bun.sh/install | bash   # needs unzip: apt-get install -y unzip

# Private ROS graph (avoid clashing with other containers on domain 0):
echo 'export ROS_DOMAIN_ID=10' >> ~/.bashrc
echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc
source ~/.bashrc     # every terminal must share these
```

---

## 4. Build

```bash
# ROS 2 workspaces
source /opt/ros/jazzy/setup.bash
cd /root/arms_ws  && colcon build --symlink-install
cd /root/ros2_ws  && colcon build --packages-select plc_bridge --symlink-install
#   NOTE: plc_bridge has two source copies — only .../plc-ros2-bridge/arms_ws/src/plc_bridge
#   is the real package (the arms_ws/src/plc_bridge symlink target). Always --symlink-install.

# GUI
cd /root/arms_ws/src/idustrial_demo_gui
export PATH="$HOME/.bun/bin:$PATH"
bun install
```

---

## 5. OpenPLC program (manual step — required for AUTO)

Load `plc-ros2-bridge/openplc_runtime/program.st` into OpenPLC, then in the **graphical `main`**:
1. Instantiate `FB_CellCycle` as `CELL_AUTO`, `enable := auto_mode`; inputs `part_present`,
   `robot_busy`, `cycle_done`.
2. Instantiate `FB_CellManual` as `CELL_MAN`, `enable := NOT auto_mode`; inputs `man_belt_cmd`,
   `man_robot_cmd`.
3. Combine: `belt_run := CELL_AUTO.belt_run OR CELL_MAN.belt_run` (same for `robot_start`);
   wire `CELL_AUTO.state → state`.
4. Compile, then confirm in OpenPLC that `auto_mode / robot_busy / cycle_done / man_*` are **writable**
   over OPC UA and `belt_run / robot_start / state` are readable (OpenPLC may regenerate `opcua.json`).

> Until this is wired, run the cell in **MANUAL** (default) — AUTO has no PLC sequence to follow.

---

## 6. Run (startup order)

Each in its own terminal (all sharing `ROS_DOMAIN_ID=10`, sources sourced):

```bash
# 0) OpenPLC runtime — start it & run the loaded program (its UI / your usual method)

# 1) PLC bridges (OPC UA + Modbus). Point at your OpenPLC endpoint:
source /opt/ros/jazzy/setup.bash && source /root/arms_ws/install/setup.bash
#   OPC UA client user is `user` (admin is only for the PLC runtime/web UI).
OPCUA_URL=opc.tcp://127.0.0.1:4840/openplc/opcua OPCUA_USER=user OPCUA_PASS=1234 \
  ros2 run plc_bridge bridge_node
ros2 run plc_bridge modbus_sensor_bridge

# 2) Cell (Gazebo + MoveIt + cell_io + cell_mode). Defaults: start_mode=manual, manual_source=ros
ros2 launch ros_plc_sim crx_gripper_plc.launch.py
#    (only ONE cell_mode — don't also start it standalone, or /cmd/* gets two publishers)

# 3) WebSocket bridge for the HMI
ros2 launch ros_gui_bridge bridge.launch.py

# 4) Web HMI
cd /root/arms_ws/src/idustrial_demo_gui && export PATH="$HOME/.bun/bin:$PATH" && bun run dev
#    open http://localhost:8080/hmi   (Robot tab: /hmi/robot)
#    if the browser is on another host, set VITE_BRIDGE_URL=ws://<bridge-host>:9093/ros
```

---

## 7. Verify

```bash
ros2 node list                       # fast & only your nodes (thanks to LOCALHOST)
ros2 topic echo /plc/state           # Int32 0..3 tracking the sequence (AUTO)
ros2 topic echo /cmd/belt_run        # single publisher; steady (no oscillation)
```
In the HMI: header shows **AUTO/MANUAL** (toggle to switch), Overview highlights the live
sequencer step, Robot tab shows live joints/gripper. In MANUAL: **Belt Run/Stop** (Overview)
and **Start Robot** (Robot tab) drive the cell.

---

## 8. Known TODOs / gotchas

- **OpenPLC wiring (§5)** must be done for AUTO to run and for `/plc/state` to publish.
- **Pause/Stop robot**: not implemented — `cell_io._run_cycle` runs a cycle in a thread with no
  abort hook. Only "Start" exists in MANUAL for now.
- **One `cell_mode` only**: starting a standalone `cell_mode` while the launch already runs one gives
  two publishers on `/cmd/*` → belt command oscillates (blinking button, stuttering conveyor).
- **`manual_source`**: default `ros` (MANUAL arbitrated in ROS = safe). Use `manual_source:=plc`
  only *after* the OpenPLC auto/manual gate is wired, else the cell follows the ungated PLC in MANUAL.
- **Shared `/joint_states`**: other robots on the same DDS domain can publish here; the GUI ignores
  messages that don't contain our arm joints. The `ROS_DOMAIN_ID=10` isolation prevents such leaks.
- **GUI ↔ Lovable**: `idustrial_demo_gui` is Lovable-connected; only pushes to `main` sync to Lovable.
  Work lives on the `hmi-live-controls` branch (PR to main when ready).

---

## 9. Repos & branches

| Repo | Remote | Branch |
|---|---|---|
| idustrial_demo_gui | github.com/bb53192/idustrial_demo_gui | `hmi-live-controls` |
| ros_plc_sim | github.com/bb53192/ros_plc_sim | `conveyor-belt-and-end-sensor` |
| plc-ros2-bridge | github.com/bb53192/plc-ros2-bridge | `crx-cell-integration` |
| ros_gui_bridge | github.com/CroboticSolutions/ros_gui_bridge | `ros2-rclpy-port` |

---

## 10. Two-arm color-sorting demo (`dual-arm-color-sorting` branch)

A **second CRX-10iA + Robotiq arm** turns the cell into a color sorter. A part rides the belt
past two in-series stations; overhead cameras classify its color; each arm takes its color, the
rest fall into an unsorted bin. This is a **sim-side demo** — no OpenPLC/OPC-UA/Modbus wiring yet
(see "PLC follow-up" below). The single-arm PLC cell (§1–9) is untouched.

**Data flow**
```
belt (−X) →  Station B (green, x=1.40)  →  Station A (blue, x=0.40)  →  belt end → unsorted bin
             cam /station_b/image           cam /station_a/image
             gate /gate_b/cmd_pos           gate /gate_a/cmd_pos
             green_ arm → green box         blue_ arm → blue box

 color_classifier ×2  /station_{a,b}/image → /station_{a,b}/part_color  (blue|green|other|none)
 sort_cell  ── reads contacts + colors, drives belt + gates, runs each arm's pick-place
            ── /compute_ik (one move_group, groups blue_/green_manipulator) + *_arm/*_gripper controllers
```
Sorting logic: part stops at **B**; if **green** → green arm picks it into the green box; else
gate B raises and it passes under to **A**; if **blue** → blue arm picks it into the blue box;
else gate A raises and it drops off the belt end into the **unsorted bin**. One part at a time;
a fresh, randomly-colored part is spawned at the feed end each cycle.

**Run** (isolated graph, one terminal):
```bash
export ROS_DOMAIN_ID=10 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # keep off the live sim's graph
cd /root/arms_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch ros_plc_sim dual_arm_sort.launch.py            # gazebo_gui:=false launch_rviz:=false for headless
```
Layered launch pieces (each usable alone): `dual_arm_sort_gz.launch.py` (Gazebo + dual
ros2_control) → `dual_arm_sort_moveit.launch.py` (+ move_group /compute_ik) →
`dual_arm_sort.launch.py` (+ classifiers + sort_cell).

**Key files:** `urdf/dual_crx_gz.urdf.xacro` (blue_/green_ prefixed arms, one controller_manager),
`srdf/dual_crx_robotiq.srdf`, `config/ros2_controllers_dual.yaml`, `worlds/sorting_cell.sdf`
(belt, two retractable gates, two cameras, two boxes, unsorted bin), `config/gz_bridge_dual.yaml`,
`scripts/color_classifier.py`, `scripts/sort_cell.py`, `config/dual_pick_place_waypoints.yaml`.

**Gotchas**
- **Camera render**: `sorting_cell.sdf` needs the `gz-sim-sensors-system` plugin (ogre2). Needs a
  GPU/EGL render node; verify with `gz topic -e -t /station_a_image -n 1`.
- **Belt + gate barriers are neutral gray** on purpose — the color classifier only trusts
  saturated pixels, so a colored belt/gate would be misread. Cameras sit just +X of each gate
  (over the part's rest spot), not over the barrier.
- **Gates** are a barrier on a Z-prismatic joint held by a `JointPositionController`
  (0 = closed/blocks, 0.12 = open/part passes under). `sort_cell` republishes belt + gate
  commands at 10 Hz so the gz plugins hold their targets.
- **Process hygiene when testing**: use a unique `GZ_PARTITION` per run and kill stale
  `gz sim`/`parameter_bridge`/`color_classifier` by PID between runs — leftovers on domain 10
  publish stale classifications and cause confusing results.

**PLC follow-up (not in this branch):** expose per-arm `robot_busy`/`cycle_done`, a `part_color`
input, and `gate_open` coils over the OpenPLC contract; add AUTO sequencing for both arms and
HMI controls (mirrors §5 for the single arm).
