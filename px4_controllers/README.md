# uav_emb



## Uav controller : PX4 accel setpoint/velocity setpoint - Status : fieldtested
* Q: What does it do ?
* A: It control all uav behaviors which related to physical dynamic. These behaviors are covered in 5 controller modes : Onground, TakeOff , Hold , Land and MissionExecute.The state machine is included bellow.
  - ![Screenshot](docs/state_machine.png)
* Q: Why using this system instead of writing predefined scenarios ?
* A: These are some benefits:
  - It generalizes the use case ,using one code for all scenarios and types of input.
  - It restricts the possible actions, thus reducing the chance of exceptions or dangerous outcomes.
  - It gives up and claims control status only when needed.
* Q: How to use it?
* A: Use rosservice to trigger 4 active modes ( Onground is a rest mode, that cannot be triggered manually) 
  - TakeOff , Hold and Land does what its name said. They are immediate modes, which means its behavior is defined on calling and cannot be altered. ( Ex: TakeOff will take the drone from ground to a fixed height h and straight up _ 1.5,2.0,0.5 -> 1.5,2.0,h. The drone will stay there and cannot be moved unless the mode is changed)
  - MissionExecute is a mode that the controller gives up control status, receives continous commands from a higher level layer such as a planner/trajectory generator then converts it into Px4 commands. As a common sense, after a planner has finished its job, the mode must be switched back to either Hold/Land.
* Q: How to use MissionExecute ( send it my command ) ?
* A: Publish continuous msg to one of these supported input types : 
  - Separate Position and Yaw msg :
    - topic: /controller/flatsetpoint , type: controller_msgs::FlatTarget , description: p,v,a,.. and controller_mask ( =0 if using p or p,v ; =1 if p,v,a ; =2 if using just v )
    - topic: /controller/yaw , type std_msgs::Float32 
  - MultiDof 
    - topic: /controller/trajectory , description [p,v,a,yaw,yaw_dot,..] queue , the controller will only take the first one in the queue and drop the others.
  - QuadMsg (the default output of EgoPlanner & FastPlanner)
    - topic: /controller/pos_cmd ,type: controller_msgs::PositionCommand, description , p,v,a,yaw,yaw_dot,.. 
* Q: Example ?
* A: Let say you want the uav to simulate uav flight Home -> A -> B -> Land at B
  - copy px4_controllers folder into your ws and build it, other packages is not related to the controller.
  - create a folder to store controller log and specify it into the launch files.
  - launch px4 mavros sitl
  - launch geometric_controller automatic.launch ( this will auto trigger px4 mode and arm state, for simulation purposes or no Rc mission. manual.launch is the opposite)
  - rosservice call /controller/set_mode with mode = 1 and sub = 4.0 ( switch to mode TakeOff at height of 4.0 ; takeoff height is restricted to be lower than 5.1 meter) 
  - ![Screenshot](docs/setmode.png)
  - rosservice call /controller/set_mode with mode = 3 (switch to mode MissionExecute. If your are unfamiliar with the setmode service, just call it with mode=a for more information) 
  - rostopic pub /controller/flatsetpoint with p = ( 2,0,4) and mask = 0 (send a command to the controller, this should be done by a planner/trajectory generator unless you are testing controller code)
  - rostopic pub /controller/flatsetpoint with p = ( 2,2,4) and mask = 0
  - rosservice call /controller/set_mode with mode = 4 ( switch to mode land, the end of mode Execute, the drone now is landing toward 2,2,0)
  - After the drone is landed and disarmed successfully, mode is automatically switched to Onground, which indicates that the program is running correctly.

## Marker landing : Aruco Gridboard 9 * 4x4 marker ; Whycon *1 : 
