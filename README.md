# Test-Arm-Controller
This features the headless velocity controller of the fetch robotic arm that accounts for the velocity limits, dampening and overall metric errors of the robotic end effector.


## Workspace layout

- `src/fetch_twist_controller/`: custom twist controller, velocity plugin,
  keyboard/trajectory tools, evaluation scripts and tests.
- `src/fetch_gazebo/`: vendored Fetch simulation packages.
- `src/fetch_ros_IRVL/`: vendored IRVL robot description and supporting packages.
- `tools/`: feedback timing diagnostic.
- `results/`: retained experiment recordings and exported data.

See [controller instructions](src/fetch_twist_controller/README.md) for build,
launch, keyboard and frame-selection commands. See
[cleanup review](docs/cleanup-review.md) for removed drafts and retained assets.
Generated catkin build outputs are excluded from version control.
