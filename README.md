# robotarm-interface
 WOR-World robotarm opdracht


`socat -d -d pty,rawer,echo=0 pty,rawer,echo=0`

`cat /dev/pts/...` getal hierachter is de 2de

Example servo positions command:
`ros2 action send_goal /servo robotarm_hld/action/ServoPositions "{servo_ids: [1, 2, 3], joint_angles: [60, 45, -30], speed: 50}" --feedback`


`ros2 service call /emergency_stop robotarm_hld/srv/EmergencyStop`

## Todo list:
- state logging
- diagrammen


example commands:
- `ros2 action send_goal /position_preset robotarm_hld/action/PositionPreset "{position: 0, speed: 2300}" --feedback`
- `ros2 action send_goal /servo robotarm_hld/action/ServoPositions "{servo_ids: [1, 2, 3], joint_angles: [60, 45, -30], speed: 50}" --feedback`
- ``


- `ros2 service call /emergency_stop robotarm_hld/srv/EmergencyStop`