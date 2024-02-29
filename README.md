# robotarm-interface
 WOR-World robotarm opdracht


`socat -d -d pty,rawer,echo=0 pty,rawer,echo=0`
`cat /dev/pts/...`

Example servo positions command:
`ros2 action send_goal /servo robotarm_hld/action/ServoPositions "{servo_ids: [1, 2, 3], joint_angles: [90, 45, -30], speed: 50}" --feedback`