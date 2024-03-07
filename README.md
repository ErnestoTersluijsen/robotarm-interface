# robotarm-interface
 WOR-World robotarm opdracht


`socat -d -d pty,rawer,echo=0 pty,rawer,echo=0`

`cat /dev/pts/...` getal hierachter is de 2de

Example servo positions command:
`ros2 action send_goal /servo robotarm_hld/action/ServoPositions "{servo_ids: [1, 2, 3], joint_angles: [90, 45, -30], speed: 50}" --feedback`


## Todo list:
- hoek omzetten naar PWM -> LLD
- action server aanmaken voor preset posities
- positie presets terug toevoegen/compatable maken
- noodstop toegoeven
- state logging
- diagrammen (Klasse Diagram, timing diagrammen, state diagram, sequence diagram, protocol state diagram, usecase diagram)
- 
