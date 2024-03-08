# robotarm-interface
 WOR-World robotarm opdracht

### Compileren
 Om het programma te compileren draai `colcon build` in de huidige folder. 

 Om de opdracht op te starten gebruik de volgende commando's:
 - `. install/setup.sh`
 - `ros2 run robotarm_hld robotarm_interface`

 Om debug logging te zien gebruik de volgende commando's:
 - `. install/setup.sh`
- `ros2 run robotarm_hld robotarm_interface --ros-args --log-level debug`

### Virtuele port
Dit zij de commando's om een virtuele serial port te openen

`socat -d -d pty,rawer,echo=0 pty,rawer,echo=0`

`cat /dev/pts/...`

### Mogelijke commando's:
Voorbeeld servo positions command:

`ros2 action send_goal /servo robotarm_hld/action/ServoPositions "{servo_ids: [1, 2, 3], joint_angles: [60, 45, -30], speed: 2300}" --feedback`


Voorbeeld preset position command:

`ros2 action send_goal /position_preset robotarm_hld/action/PositionPreset "{position: 0, speed: 2300}" --feedback`

Voorbeeld emergency stop command:

`ros2 service call /emergency_stop robotarm_hld/srv/EmergencyStop`



## Todo list:
- event logging
- diagrammen checken (Klasse Diagram, timing diagrammen, state diagram, sequence diagram, protocol state diagram, usecase diagram)


vragen:
i.p.v "move", "moving"