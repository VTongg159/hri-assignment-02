# Report notes

## System

The CLI sends a natural-language command plus a programmatically calculated student mapping to Gemini through the local 9Router OpenAI-compatible endpoint. JSON is accepted only after exact-field schema checks, skill/object/zone whitelists, and pick/place state validation. The LLM never supplies joints, poses, or trajectories. The C++ server owns the poses and calls MoveIt for every arm motion.

The robot is the combined UR3e plus the official Robotiq 2F-85 description, meshes, UR flange adapter, and mimic-joint linkage. MoveIt plans to `gripper_tcp`, located in the useful fingertip-pad region. Pick performs approach, open, descend, close, attach, and lift; place performs approach, descend, open, detach, and retreat. The held Gazebo cube follows the TCP continuously at 20 Hz. `reset_scene` opens the gripper, returns the robot home, and restores deterministic cube poses; reset is not exposed in the LLM prompt or validator.

Scene count: eight assignment entities (UR3e, table, three cubes, three zones) plus the ground plane, nine models in total after the robot is spawned.

## Student mapping

```text
Student ID: 23020766
XX = 66
P = 66 mod 6 = 0

Zone A = Red
Zone B = Yellow
Zone C = Blue
```

The modulo and permutation are calculated in `student_task.py`; they are not hard-coded into the prompt.

## Verified results — 25 September 2026

- Environment: Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Harmonic 8.11.0, MoveIt 2 2.12.4, UR simulation 2.5.0, 9Router 0.5.86, Node.js 24.21.0.
- Live model: `gemini/gemini-3.6-flash` through `http://localhost:20128/v1`.
- Build: workspace `colcon build --symlink-install` passed with the unused serial-hardware, hardware-test, and TSF packages explicitly ignored; official `robotiq_msgs`, `robotiq_controllers`, and `robotiq_description` are built. `colcon test-result --verbose` reported 20 assignment tests, 0 errors, 0 failures, 0 skipped.
- Robotiq: official upstream commit `8d7b8412ad685ffe1db5719da6e8fce6c1896e5e`; Jazzy `parallel_gripper_action_controller/GripperActionController` exposes `control_msgs/action/ParallelGripperCommand` at `/robotiq_gripper_controller/gripper_cmd`.
- Headless Bullet Featherstone run loaded all three controllers as active. Closing to 0.35 rad reached 0.330 rad within the 0.02 rad goal tolerance; the final open state was approximately 0 rad.
- Live basic command: Gemini returned `pick(red_cube)`, `place(red_cube, zone_b)`, `home()`; all three completed with `SUCCESS`.
- Red final target/actual: `(0.380, 0.000, 0.085)` m / `(0.380, 0.000, 0.085)` m; Euclidean position error `0.000 m`.
- Live Vietnamese command: `Hãy lấy khối màu vàng và đặt nó vào ô A.` returned the correct three-step plan. After the continuous-joint fix, the physical mock-provider replay completed and Gazebo reported yellow at `(0.380, -0.120, 0.085)` m.
- Live second-object command: `Move the blue cube to zone C.` completed all three skills; Gazebo reported `(0.380, 0.120, 0.085)` m.
- Live personalized command: Gemini returned seven steps for red→A, yellow→B, blue→C, then home. Every skill completed successfully. All three Gazebo positions exactly matched their configured targets.
- Offline cycle coverage: the deterministic world-state test moves one cube through `buffer_zone` to break a three-zone cycle.
- Invalid `system`, `green_cube`, and `zone_d` plans are rejected by the validator before executor/service invocation.

## Controller diagnosis and fixes

The original scaled controller aborted around the path threshold. The simulation now uses `joint_trajectory_controller`; transient path lag from Gazebo's position backend is not used as an abort condition, while final joint goals retain a 10 s allowance. This does not bypass MoveIt's start-state and collision validation.

Two separate endpoint faults were found from desired/actual joint logs: IK branches outside the reliably simulated shoulder range stalled at the lower or upper endpoint, so MoveIt restricts `shoulder_lift` to the verified `[-3.0, 0.0]` range; and the continuous `wrist_3` could differ from a normalized plan by exactly `2π`, so trajectory points are unwrapped around the measured joint angle before MoveIt's unchanged 0.01 rad start-state check. A folded collision-free startup pose prevents the upstream straight pose from intersecting the table.

## Simulation-grasp boundary

- The official gripper geometry and driven/mimic finger motion are simulated. Gazebo cubes are visual-only because a static collision cube blocks the mimic linkage instead of yielding to a grasp. Their 50 mm box collisions remain authoritative in MoveIt, and attach/detach occurs only after close/open respectively.
- Bullet Featherstone is selected explicitly because Gazebo's DART backend does not reliably support this mimic-joint mechanism.
- The provider issue described during setup was resolved after upgrading from the previous incompatible Node.js version. Node 20 was the known fix; the machine used for final verification now reports Node 24.21.0.
- GUI screenshots still need to be captured during the classroom recording; runtime and pose evidence above was collected headlessly.
