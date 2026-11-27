# Fetch resolved-rate twist controller — ROS 1 Noetic

A simulation-oriented implementation for Ubuntu 20.04, Fetch Gazebo's `gazebo11`
branch, and the IRVL `fetch_description` model. It uses Fetch's
`robot_controllers` framework, not `ros_control`.

## Validation status

Implemented: controller, native velocity plugin, keyboard input, YAML/CSV motion
player, frame transformations, evaluator, report plots, launch/config files.
Five ROS-independent numerical tests pass; Python syntax and XML/YAML structure
were checked. **This package has NOT been compiled against ROS or executed in
Gazebo in the authoring environment (ROS/Gazebo are unavailable there).**
Your local build and simulator test are still required. No tracking measurements
or claimed 100 Hz performance have been fabricated. The included two-frame
runner generates the required CSVs, RMS metrics, and plots from your simulation.

## Architecture

`TwistStamped /fetch_twist/command` -> Python resolved-rate node -> named
`JointState /fetch_joint_velocity/command` (velocity field) -> C++ Fetch plugin
-> `JointHandle::setVelocity(velocity, 0)`.

The plugin is loaded by the existing Gazebo controller manager. It is not a
standalone node. It exclusively claims seven arm joints; gravity compensation
can continue as a nonexclusive stacked controller. The torso is not commanded.
The node reads measured joint positions, constructs a KDL geometric Jacobian
for base_link -> gripper_link, selects arm columns, and solves via SVD with
singularity-dependent damping. It publishes at nominal 100 Hz with overrun
warnings; this Python/ROS/Linux loop is not a hard-real-time guarantee.

## 1. Install and build

Extract this directory to `~/IRVL_ws_Official/src/fetch_twist_controller`.
If a skeleton with that name already exists, move it outside src first:

```bash
mkdir -p ~/IRVL_package_backups
mv ~/IRVL_ws_Official/src/fetch_twist_controller \
   ~/IRVL_package_backups/fetch_twist_controller_$(date +%Y%m%d_%H%M%S)
```

Run in Ubuntu 20.04 WSL, not Ubuntu 24.04:

```bash
source /opt/ros/noetic/setup.bash
cd ~/IRVL_ws_Official
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin_make -j2
source devel/setup.bash
```

If a dependency cannot be resolved, stop and retain the exact error; do not
substitute ROS 2 packages. This needs the Noetic robot_controllers interface,
PyKDL, kdl_parser_py, NumPy, PyYAML, and Matplotlib.

## 2. Start simulation, control node, and velocity plugin

Stop any older Fetch simulation with Ctrl-C first. Gazebo must start from a
shell sourced AFTER this package was built so pluginlib sees the new library.
Use one terminal for each long-running command.

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source ~/IRVL_ws_Official/devel/setup.bash
roslaunch fetch_twist_controller simulation.launch
```

Wait for Fetch to spawn and finish its initial arm preparation. Do not start
MoveIt executions or other arm-command tools during the tests.

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source ~/IRVL_ws_Official/devel/setup.bash
roslaunch fetch_twist_controller control.launch
```

Terminal 3:

```bash
source /opt/ros/noetic/setup.bash
source ~/IRVL_ws_Official/devel/setup.bash
rosrun fetch_twist_controller activate.py
rosrun fetch_twist_controller keyboard.py --frame base_link
```

Activation stops the arm and arm-with-torso trajectory controllers if running,
then loads/starts the velocity plugin using QueryControllerStates. It verifies
the action result. A failure may leave trajectory controllers stopped: restart
the simulator to restore the original configuration. Activation alone sends no
motion. Do not launch the built-in CartesianTwistController concurrently.

Keyboard: w/s = +/-x, a/d = +/-y, r/f = +/-z, u/j = +/-roll,
i/k = +/-pitch, o/l = +/-yaw. Space stops; Ctrl-C exits.
1 selects base_link; 2 gripper_link; 3 user_frame.
Terminal input has no key-release events. Motion expires 0.2 s after the last
key-repeat event, so the first held key may pause until OS repeat begins.
Only one twist publisher should run at a time.

## 3. Play a motion sequence

Exit keyboard control first. Run in terminal 3:

```bash
rosrun fetch_twist_controller trajectory.py \
  $(rospack find fetch_twist_controller)/config/test_motion.yaml --frame base_link
```

Repeat with `--frame gripper_link` for end-effector axes.
YAML has `steps:`, each with `duration`, optional `frame`, and a six-element
`twist: [vx, vy, vz, wx, wy, wz]`. CSV columns are:

```csv
duration,frame,vx,vy,vz,wx,wy,wz
2,base_link,0,0,0,0,0,0
1,base_link,0.02,0,0,0,0,0
1,base_link,-0.02,0,0,0,0,0
```

Linear units: m/s. Angular units: rad/s. Duration: simulation seconds.
Playback aborts if simulation time goes backwards or stalls for one wall second.
The sample uses small reversible motions, but is not collision-aware. Inspect
the simulated arm and its clearances before playback.

## 4. Record two frames and generate evaluation results

Keep Gazebo and control.launch running, with the plugin activated. Stop keyboard
input. In terminal 3:

```bash
bash $(rospack find fetch_twist_controller)/scripts/run_evaluation.sh
```

This performs base_link then gripper_link motions and creates a timestamped
results directory under `~/IRVL_ws_Official/results/` containing:

- base_link.csv, base_link.json, base_link.png
- gripper_link.csv, gripper_link.json, gripper_link.png

These are your real evaluation deliverables, generated only after execution.
The two runs need not start at exactly identical poses; for controlled comparisons
restart the simulator to the same pose and run each frame separately.

Manual evaluation (use another terminal for the trajectory):

```bash
rosrun fetch_twist_controller evaluate.py --output /home/alegc/IRVL_ws_Official/results/base_link.csv
# Ctrl-C after the motion, then:
rosrun fetch_twist_controller report.py /home/alegc/IRVL_ws_Official/results/base_link.csv
```

The controller publishes both the transformed requested tip twist and
`J(q_measured) * qdot_measured`. The latter includes torso velocity if present.
Evaluation pairs measured state timestamps to the latest target no later than
that timestamp, rejects targets older than 0.1 s, and skips duplicate states.
All six components are expressed in base_link at gripper_link for both runs.
Metrics compare the requested target BEFORE damping/limiting with measured
motion, so saturation, singularity damping, and physical lag remain visible.

RMS linear error = sqrt(mean(||v_measured-v_command||^2)); angular likewise.
JSON includes all-sample and nonzero-command RMS, per-axis RMS, effective
measurement rate and duration. CSV includes the target age. This is kinematic
tracking evaluation, not independent external ground truth. Model errors and
wheel/base motion relative to the world are not measured by this arm Jacobian;
keep the base stationary during these tests. The configured loop frequency is
not the same as the measurement frequency. Watch loop-overrun warnings and use
`rostopic hz /fetch_joint_velocity/command` to measure output rate.

## Frame selection and adjoint convention

A nonempty incoming header.frame_id takes precedence. If empty, the node reads
its `~default_frame` parameter (which may be changed while running):

```bash
rosparam set /twist_controller/default_frame gripper_link
```

Keyboard and trajectory tools explicitly set frame_id, so use their --frame
option or keyboard 1/2/3 instead of this parameter for those tools.

Supported: base_link; gripper_link; any TF-connected world or custom frame.
`user_frame` is a static transform configured by parent, translation [x,y,z],
and quaternion [x,y,z,w] in config/controller.yaml. Restart control.launch after
editing it. A frame named `world` works only if it actually exists in TF.
Missing/invalid transforms cause zero velocity output.

Twist convention is [linear; angular]. Adjoint matrix is
`Ad(T) = [[R, skew(p)R], [0,R]]`. A geometric Jacobian maps to linear velocity
at the tip, not at the root origin. The code applies the adjoint with reference
point shifts before/after it so these definitions agree:

- `twist_reference_point: tip` (default): input linear velocity belongs to the
  gripper point but is expressed in the selected axes. This reduces to rotation
  of linear/angular components; translations must not introduce false motion.
- `twist_reference_point: origin`: input linear component is referenced at the
  selected coordinate origin. Apply the full adjoint, then shift to the gripper.

For a moving world/base relationship, commands are interpreted as relative arm
motion expressed in those axes, not whole-robot absolute world compensation.

## Parameters and stop behavior

config/controller.yaml: rate, damping, singularity_threshold, velocity_limits,
position_margin, command_timeout, state_timeout, root, tip, frames.
config/plugin.yaml: plugin limits, position margin, independent timeout.
Adjust both sets of velocity limits consistently; URDF limits also cap values.
The singularity threshold uses the raw mixed-unit geometric Jacobian; tuning
may be needed. The solver smoothly increases damping below that threshold.

Fresh nonzero ROS timestamps are required for commands and state. Invalid,
stale or missing data result in zero velocity. The plugin also checks wall-clock
receipt age and message age. Losing the Python node therefore stops commands
without needing a final message. Zero commanded velocity is not an instantaneous
physical stop; the simulator's joint servo dynamics still apply. Neither this
package nor its limits implement collision avoidance or physical-robot safety.
Test in simulation; physical Fetch deployment has not been validated.

To stop, exit the twist publisher and press Ctrl-C in control.launch. The plugin
will keep commanding zero after its watchdog expires. To unload control use:

```bash
rosrun fetch_twist_controller activate.py --stop
```

Restart Fetch Gazebo to restore its original controllers. No /opt/ros files or
existing Fetch YAML files are modified by this package.

## Numerical verification

```bash
python3 -m unittest discover -s $(rospack find fetch_twist_controller)/tests -v
```

Tests cover well-conditioned tracking, singularity behavior, limit scaling,
rotation of tip-referenced commands, and offset-origin adjoint behavior.

## Source references

- https://github.com/fetchrobotics/robot_controllers/tree/ros1
- https://github.com/fetchrobotics/fetch_gazebo/tree/gazebo11
- https://github.com/IRVLUTD/fetch_ros_IRVL/tree/ros1

Created as a new implementation against the public Fetch interface. No upstream
controller source is copied. Change maintainer details in package.xml as needed.
