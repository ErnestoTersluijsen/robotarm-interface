# robotarm-interface
 WOR-World robotarm opdracht

### Compileren
 Om het programma te compileren draai `colcon build` in de huidige folder. 

 Om de opdracht op te starten gebruik de volgende commands:
 - `. install/setup.sh`
 - `ros2 run robotarm_hld robotarm_interface`

### Virtuele port
Dit zij de commando's om een virtuele serial port te openen

`socat -d -d pty,rawer,echo=0 pty,rawer,echo=0`

`cat /dev/pts/...`

### Mogelijke commando's
Voorbeeld servo positions command:
- `ros2 action send_goal /servo robotarm_hld/action/ServoPositions "{servo_ids: [1, 2, 3], joint_angles: [60, 45, -30], speed: 2300}" --feedback`

Voorbeeld preset position command:
- `ros2 action send_goal /position_preset robotarm_hld/action/PositionPreset "{position: 0, speed: 2300}" --feedback`

Voorbeeld emergency stop command:
- `ros2 service call /emergency_stop robotarm_hld/srv/EmergencyStop`



## Todo list:
- state logging
- diagrammen

