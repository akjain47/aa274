# Section 7 cheat sheet

Tasks 1–3 use the code already on `section7`. Task 4 below is a reference for
changes to make during the lab; those changes are not implemented in this branch.
Paths below assume the lab laptop layout: `~/autonomy_ws/src/aa274`.

In **every new laptop terminal**, source the course environment; after building,
also source your workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
source ~/autonomy_ws/install/setup.bash  # after the first build
```

## 1.1–1.3 — Workspace, clone, branch

On the lab laptop, save any old workspace before replacing it. Skip setup if
your checkout is already prepared.

```bash
# Only if an old workspace exists and needs replacing:
mv ~/autonomy_ws ~/autonomy_ws.backup-$(date +%s)

mkdir -p ~/autonomy_ws/src
git clone --branch section7 https://github.com/akjain47/aa274.git ~/autonomy_ws/src/aa274
cd ~/autonomy_ws/src/aa274
git branch --show-current  # section7; already created on GitHub
```

## 1.4 — Homework code

Already included: `autonomy_repo/scripts/frontier_exploration.py`, its supporting
`navigator.py`, and `autonomy_repo/launch/frontier_exploration.launch.py`.
The explorer sends one `/cmd_nav` goal, waits for `/nav_success`, and retries
failed regions after a cooldown. No copying is needed from the homework folder.

## 1.5 — Build

Executable registration and dependencies are already included.

```bash
cd ~/autonomy_ws/src/aa274
chmod +x autonomy_repo/scripts/{frontier_exploration,navigator}.py
cd ~/autonomy_ws
colcon build --packages-select autonomy_repo
source install/setup.bash
```

Rebuild and source again after changing Python files or launch files.

## 2.1–2.2 — Frontier exploration in simulation

**Terminal A:**

```bash
ros2 launch asl_tb3_sim root.launch.py
# Alternative: ros2 launch asl_tb3_sim maze.launch.py
```

**Terminal B:**

```bash
ros2 launch autonomy_repo frontier_exploration.launch.py
```

**Checkpoint:** autonomous movement, live RViz map, and new goals after arrival
or planning failure. Show the CA.

## 3.1–3.2 — Frontier exploration on hardware

Stop both simulation terminals with **Ctrl+C**. Replace `ROBOT_NAME` below.
The laptop and robot must use the same assigned `ROS_DOMAIN_ID`.

**Robot terminal:**

```bash
ssh aa274@ROBOT_NAME.local
echo "$ROS_DOMAIN_ID"
ros2 launch asl_tb3_driver bringup.launch.py
```

**Laptop terminal:**

```bash
echo "$ROS_DOMAIN_ID"
ros2 launch autonomy_repo frontier_exploration.launch.py use_sim_time:=false
```

**Checkpoint:** demonstrate hardware exploration to the CA.

## 4.1 — Stop-sign behavior: make copies, then edit

Stop the hardware launches. Test the following changes in simulation first.
Run only one exploration launch at a time; leave the Section 6 perception
controller stopped because it also publishes velocity commands.

```bash
cd ~/autonomy_ws/src/aa274/autonomy_repo
cp scripts/frontier_exploration.py scripts/frontier_exploration_stop.py
cp scripts/navigator.py scripts/navigator_stop.py
cp launch/frontier_exploration.launch.py launch/frontier_exploration_stop.launch.py
```

**A. In the copied explorer**, add these fields inside `__init__` (`Bool` is
already imported):

```python
self.paused = False
self.pause_until = 0.0
self.ignore_until = 0.0
self.pause_pub = self.create_publisher(Bool, "/exploration_paused", 10)
self.detector_sub = self.create_subscription(
    Bool, "/detector_bool", self.detector_callback, 10)
self.resume_timer = self.create_timer(0.1, self.resume_if_ready)
```

Add these methods inside `FrontierExploration`:

```python
def detector_callback(self, msg):
    now = self.now_seconds()
    if not msg.data or self.paused or now < self.ignore_until:
        return
    self.paused = True
    self.pause_until = now + 5.0
    self.pause_pub.publish(Bool(data=True))

def resume_if_ready(self):
    now = self.now_seconds()
    if self.paused and now >= self.pause_until:
        self.paused = False
        self.ignore_until = now + 3.0  # chosen cooldown after resuming
        self.pause_pub.publish(Bool(data=False))
```

Add this guard at the **start of both** `select_goal()` and `result_callback()`.
Keep `active_goal` while paused so the navigator can resume that destination.

```python
if self.paused:
    return
```

**B. In the copied navigator**, pause the actual velocity controller too.
Add inside `Navigator.__init__` (`Bool` is already imported):

```python
self.exploration_paused = False
self.pause_sub = self.create_subscription(
    Bool, "/exploration_paused", self.pause_callback, 10)
```

Add these methods inside `Navigator`:

```python
def pause_callback(self, msg):
    self.exploration_paused = msg.data
    if msg.data:
        self.is_planned = False
        self.stop()
    elif self.goal is not None:
        self.replan(self.goal)  # fresh trajectory and timing after the pause

def can_compute_control(self):
    return not self.exploration_paused and super().can_compute_control()
```

Add this guard at the **start of the existing** `replan()` method:

```python
if self.exploration_paused:
    return
```

The navigator remains the velocity-command publisher. Its base controller
publishes zero velocity while `can_compute_control()` is false. Use timers,
not `time.sleep()`, so map and detection callbacks keep running.

**C. Wire the copies.** In `frontier_exploration_stop.launch.py`, change only:

| Existing executable | Replacement |
| --- | --- |
| `navigator.py` | `navigator_stop.py` |
| `frontier_exploration.py` | `frontier_exploration_stop.py` |

Keep `known_free_only: True` and `use_sim_time`. Add both new scripts to the
existing `install(PROGRAMS ...)` block in `autonomy_repo/CMakeLists.txt`:

```cmake
    scripts/navigator_stop.py
    scripts/frontier_exploration_stop.py
```

```bash
chmod +x scripts/{navigator_stop,frontier_exploration_stop}.py
cd ~/autonomy_ws
colcon build --packages-select autonomy_repo
source install/setup.bash
```

**D. Simulation test.** Start the simulator as in Task 2.1, then launch the copy:

```bash
ros2 launch autonomy_repo frontier_exploration_stop.launch.py
```

From another sourced terminal, publish individual test messages:

```bash
ros2 topic pub --once /detector_bool std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /detector_bool std_msgs/msg/Bool '{data: false}'
```

Expected: stop for about 5 seconds, resume, and ignore new detections for 3
seconds after resuming. `false` does not cancel the timed stop. Repeat `true`
during the ignore window, then again after it expires. Check `/cmd_vel` is zero
throughout the stop (`ros2 topic echo /cmd_vel`). **Show the CA.**

## 4.2–4.3 — Stop-sign behavior on hardware

Stop simulation and its exploration launch. On the robot, replace the old
bringup with:

```bash
ros2 launch asl_tb3_driver bringup.launch.py camera:=true
```

On the laptop:

```bash
ros2 topic echo /detector_bool  # verify detections; Ctrl+C to stop echo
ros2 launch autonomy_repo frontier_exploration_stop.launch.py use_sim_time:=false
```

`camera:=true` starts the course camera/detector pipeline. **Checkpoint:**
show the CA exploration, a timed stop at a stop sign, and resumption.

## Cleanup — Save and hand off

Stop launches with Ctrl+C, then save only your intended lab files:

```bash
cd ~/autonomy_ws/src/aa274
git status
git add autonomy_repo/CMakeLists.txt autonomy_repo/scripts/*_stop.py autonomy_repo/launch/*_stop.launch.py
git commit -m "Add stop-sign pause and resume for Section 7"
git push origin section7
gh pr create --base main --head section7 --fill
git status -sb
```

Verify your commit on GitHub. After your work is saved and checked, follow the
lab's workstation cleanup, GitHub logout, and CA sign-out instructions.
