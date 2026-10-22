We are taking aa274

## Section 4 setup

The `section4` branch includes the completed HW1 `autonomy_repo` ROS package
alongside `s2_basic` and `s3_basic`. Clone the repository into the lab workspace
with its default directory name, `aa274`:

```sh
mkdir -p ~/autonomy_ws/src
cd ~/autonomy_ws/src
git clone --branch section4 https://github.com/akjain47/aa274.git
cd ~/autonomy_ws
colcon build
source install/setup.bash
```

This produces `~/autonomy_ws/src/aa274/autonomy_repo`; no package folders need
to be moved after cloning. Use the lab's ROS environment and dependencies.

Run the simulator, controller, and plotting script in separate terminals as
described in Section 4. Source the workspace in each terminal. Run the plotting
script from the workspace root:

```sh
cd ~/autonomy_ws
source install/setup.bash
ros2 run autonomy_repo p3_plot.py
```

The plot is saved to `src/aa274/autonomy_repo/plots/p3_output.png`, relative to
the workspace root. The controller retains the HW1 gain of 2.0; the ROS parameter
changes are exercises to complete during Section 4.
