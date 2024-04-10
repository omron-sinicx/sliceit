# Manual

How to save the code in GitKraken / Gitlab
1. Select WIP row message (Work-In-Progress)
2. Check changes 
    Select stage(or not stage) on each change 
4. Add comment on "Summary area"
5. Select "Commit" 
3. Select "Push"

---

## Reinstalling YoloV5
TODO: add configuration to Dockerfile
get the latest version of pytorch for CUDA 11.0: https://pytorch.org/get-started/locally/
`pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113`

# Set-up

## Set-up real robots
---

Set-up robot locally:
1. Power on all of the tablets (Teach-Pendant/Polyscope)
   1. Push "Power button" on both tablets
2. Power on the robot on the controllers
   1. Allow set-up on the tablet screen
   2. Open up the powering window by tapping on the red "power off" indicator (bottom left)
   3. Press start twice to set the robot up for manipulation
3. Activate Grippers
   1. Touch the top right button "UR+" on the panel
   2. Open up "gripper" window within UR+
   3. Press the "activate" button on the panel
   
---

Manually move the robot using tablet:
1. Change the "Remote" mode to the "Local" mode (upper right section)
2. Stop the running program
    - Press the `Pause` or `Stop` button in the bottom bar 
3. Change robot position manually
    Move the robot arm while pushing the button behind the tablet 
4. Change the "Local" mode to the "Remote" mode
5. Get the Joints position
    - `rosrun ur_control joint_position_keyboard.py --namespace b_bot`
    - Press `P` to print joint configuration


---
Set-up robot for remote use through TERMINATOR terminal (from local):
1. Launch the cooking docker environment
    ```shell
    cd ~/o2ac-ur 
    ./LAUNCH-TERMINATOR-TERMINAL.sh
    ```
1. Launch roscore

    (In terminal "roscore" ) press "r"

3. run bring-up robots
   
    (In terminal "bring-up robots" ) press "r"
5. Launch Rviz
    
    (In terminal "moveit/rviz" ) press "r"

6. Launch camera watcher:
    
    `roslaunch o2ac_scene_description osx_bringup_cam1.launch` (change number for 
    different cameras)
7. Launch vision server:
    
    `roslaunch o2ac_vision o2ac_vision_cooking.launch` in Terminal
---

## Bring-up Rviz simulation

Launching Simulation environment Rviz/Moveit 
1. Launch the cooking docker environment
    ```shell
    cd ~/o2ac-ur 
    ./LAUNCH-TERMINATOR-TERMINAL.sh
    ```
2. Launch the demo environment
   
    `roslaunch o2ac_moveit_config demo.launch`

---

## Working with Gazebo
1. Launch Gazebo (This should launch first)
`roslaunch o2ac_gazebo o2ac_gazebo.launch`
2. Launch MoveIt with Gazebo
`roslaunch o2ac_moveit_config o2ac_moveit_planning_execution.launch sim:=true`

### Simulating vision with Gazebo
1. Start vision server `roslaunch o2ac_vision o2ac_vision_cooking.launch`
2. Insert `picture` to the gazebo environment from the "insert" tab
3. Move `picture` to tray or cooking board (Be careful not to move the robots or you have to restart gazebo)
4. Test same as in the real robot

---

### Adding a picture to Gazebo
1. Duplicate one of the picture model folder in o2ac_gazebo (remember to change name accordingly)
2. Put the picture to be simulated into the materials texture folder
3. Change material file, config file and sdf file to match the picture content and the description. Keep track of the file names and the **material** name (e.g. `Vegetable/cucumber`)

---
## Helper scripts

---

- Manually move the robot from keyboard (No Motion Planning/No MoveIt), **no collision checking!**:

    `rosrun ur_control joint_position_keyboard.py --namespace b_bot`


- In order to get the relative position between two frames (cucumber/tip2 and b_bot gripper):

    `rosrun tf tf_echo moveit_group/cucumber/tip2 b_bot_gripper_tip_link `

- Helper for spawning a cube in Gazebo:
    
    `rosrun ur3_gazebo spawner.py --cube`


---
## Configuring MoveIt environment
### Adding a new virtual object (vegetables)

1. Get STL file 
    - How to change the size object
      1. Launch meshlab
      1. Import/Drag object
      1. `Filter` > `Normals Curvatures and orientation` > `Transform:Scale, Normalize`
      1. Set scale (multiplier) from current size (2 is twice / 0.5 is half)
      2. Save 
    - How to reduce size of STL FILE
      1. Launch meshlab
      1. Import/Drag object
      1. `Filter/Remeshing ` > `simplification:Quadric edge`  
      1. Set Target number of faces to about (1000~3000)
      1. Save 
2. Add STL file to the `o2ac_assembly_database/config/cooking/meshes`
3. Add YAML file to the `o2ac_assembly_database/config/cooking/object_metadata`
    1. Define subframes
    2. Define grasp poses 
        - to visualize grasp pose: `roslaunch o2ac_assembly_database visualize_object_metadata.launch`
4. Add "part" (object) to the `o2ac_assembly_database/config/parts_list.yaml`
5) Add the object and its initial pose to the `load_object()` function in `cooking.py`

---
### Adding pick and hold poses (advanced)
1. Create a function for each object
2. Define grasp pose and subframes
    - to fix manually :activate "Planning Request Query Goal State"
    - to get results : `rosrun tf tf_echo **target subframe** **target robot**_bot_gripper_tip_link`
        
      e.g: `rosrun tf tf_echo moveit_group/banana/tip2 a_bot_gripper_tip_link`

### How to change the tools collision meshes and robot's frames
1. Change object pose on `o2ac_assembly/config/cooking/tool_collision_objects.yaml`
2. Change primitive size and pose on `o2ac_assembly/config/cooking/tool_collision_objects.yaml`
    
    Also add primitive BOX/CYLINDER on `o2ac_assembly/config/cooking/tool_collision_objects.yaml`
3. Change subframe position/orientation on `o2ac_assembly/config/cooking/tool_collision_objects.yaml`
    
    If you change initial position of tools : change equip_**tool_name** function's position and orientation
4. Add frames/link on `o2ac_scene_description/urdf/components/o2ac_end effector.urdf.xacro` 
    
    Define joint_name,  parent_link, child link, origin rpy xyz, link_name
5. Change frame position on `o2ac_scene_description/urdf/components/o2ac_end effector.urdf.xacro`
---



### How to do linear push
1. linear_push(force, direction, timeout, max_distance)
    
    timeout: continue to moving time 
    
    max_distance: continue to moving distance
2. Only force control

    ```python 
    force_control(target_force=target_force, target_positions=target_position, force_position_selection_matrix=force_position_selection_matrix, relative_to_ee=relative_to_ee, end_effector_link=end_effector_link, timeout=timeout)
    ```
    
    - timeout: continue to apply force
    - target_force: direction and how much power to apply force
    - target_position: one or many position to go
    - relative_to_ee: 

        (True) Set direction with respect to the FRAME x, y, z axis
    
        (True)The FRAME is set on "end_effector_link" parameter
    - relative_to_ee: 
        
        (False)Set direction with respect to the WORLD FRAME x, y, z axis
4. Move and apply force
    
    - Use force_control function
        
        Set time how long continue to move on "timeout" parameter
        
        Define positions to go on "target_position" parameter 
        
        Define force direction and power on "target_force" parameter


---
## Configuring vision system
### Calibrating cameras (ex: a_bot_inside_camera)
0) Set up robot for remote usage
1) Launch camera watcher
    
    `roslaunch o2ac_scene_description osx_bringup_cam0.launch`
2) Launch calibration server (in another Terminal)
    
    `roslaunch aist_handeye_calibration o2ac_handeye_calibration.launch camera_name:=a_bot_inside_camera`
3) Launch calibration client (in another Terminal)
    
    `roslaunch aist_handeye_calibration run_calibration.launch camera_name:=a_bot_inside_camera`
4) `Ctrl+C` to exit out once done

---

### Training vision system with ssd
1. collect dataset with `test_cooking.py`'s functions
   1.  `c.collect_db_tray()` for tray
   2.  `c.collect_db_cutting_board()` for cutting board
2. label dataset with [LabelImg](https://github.com/heartexlabs/labelImg)
    1. RuntimeError: copy_if failed to synchronize -> add argument Cuda false
    2. RuntimeError: Invalid index in gather -> (note: number of classes is +1 as background counts as additional class)

---

### Training vision system with Yolov5
1. collect dataset with `test_cooking.py`'s functions
   1.  `c.collect_db_tray()` for tray
   2.  `c.collect_db_cutting_board()` for cutting board
2. label dataset with [roLabelImg](https://github.com/cgvict/roLabelImg)
   1.  reinstalling seems to solve most problems for roLabelImg - not stable on anything other than python 2 i.e. python roLabelImg.py
   2.  check shortcuts on GitHub - useful for time efficiency if nothing else
   3.  remember too take breaks (not more than 3 hours!!!) to not hurt arm/.wrist/hand
3. Train the network: `python train.py --img 400 --batch 32 --evolve --epochs 100 --data /root/o2ac-ur/yolov5-pip/yolov5/data/synthetic_cooking.yaml --hyp /root/o2ac-ur/yolov5-pip/yolov5/data/hyps/obb/hyp.cooking.yaml --weights yolov5s.pt --device 0 --cache --name synthetic_cooking` from yolov5 (10 epochs ~=2 mins)
---

## Preparing environment to run cutting demos (tomato/cucumber)
- Wrap entire environment with  plastic wrap
- Replace finger protection for robot grippers
- Place cutting board (cutting board sheet for metal plate)
- Place plate in bottom left corner of tray
- Place vegetables to be cut on tray (anywhere, really)
- Put knife in holder, well pushed in so that the handle touches the holder
- Launch robot (Local and Remote)

---
# TODO:
1. Fix camera multiplexer issue (vision system set-up)
2. Finetune position of cutting board to real pose (to fix collision on MoveIt)
3. Recognize the dish plate position (currently manually input)
4. Have the size of the vegetables change in MoveIt (Dynamically scale the mesh)
5. Make better holder for knife (reason: unequipping non-straight knife after force control is hard)
6. Unequip knife autonomously
7. Try with tomato (check position of tomato on cutting board)
8. Make demo with more cut pieces
9.  Solve the sticking to knife problem - experiment with different movements after cut
10. Solve the bent cucumber problem
11. Make the movements faster
12. (Once Joaquín's research is done or by using the camera that sees the gripper) add check to see if the gripper grasped something
13. Move the cucumber to keep cutting after putting away cut cucumbers
14. Cutting depth/check that it cut through the cucumber - avoid cutting the metallic cutting board and dulling the knife
15. Add vision checks to see if cucumber fell between tray and ccutting board (I don't know what can be done with that information)
16. Add other vegetables in the database + think about how to manipulate them (easy would be cutting carrots/tofu/cheese/daikon - force control problem - or grating carrots - different movement)
17. Ask Hamaya-san what kind of salad we are looking at
18. Get an actual cooking board!!! (big one wooden table) + extra protection 100yen small cutting board - needed for force control tests
19. **Get new PC as current one has GrEAt limitations**
