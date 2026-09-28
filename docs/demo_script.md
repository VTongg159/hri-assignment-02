# Short classroom demo script

1. Show the Gazebo scene: UR3e with the recognizable Robotiq 2F-85, table, three cubes, three zones, and ground plane.
2. Show `student_config.yaml` and the CLI calculation: ID `23020766`, `XX=66`, `P=0`, A=red, B=yellow, C=blue.
3. Confirm 9Router is running and the three environment variables are set; never show either API key.
4. Run `Please put the red cube in zone B.` in default live mode. Point out approach, visible finger open/close, cube following the TCP during lift/transfer, release, and retreat. Show `TASK SUCCESS`, then `gz model -m red_cube -p`.
5. Run `ros2 run ur3_llm_control reset_scene`.
6. Run `Hãy lấy khối màu vàng và đặt nó vào ô A.` to show live Vietnamese planning.
7. Reset again, then run `Arrange all objects according to my student ID.`. Show all seven successful skills and the final A=red, B=yellow, C=blue scene.
8. Briefly show the prompt and validator. Explain that Gemini selects only semantic skills; MoveIt alone generates arm motion, while the Jazzy parallel-gripper action drives the official 2F-85 joint.
9. End with `colcon test-result --verbose` and hide debug logs/credentials from the recording.
