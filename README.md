

# Master Thesis - Dual-Arm Robot Task Learning via Teleoperation with Improved Dynamic Movement Primitives Adaptation

**Author:** Mohiyuddin Al Muhsin

For a copy of the thesis PDF, contact: muhsin168@gmail.com.



This repository contains the necessary workspaces and files used in the thesis which involves Learning from Demonstration (LfD) based on  Dynamic Movement Primitives (DPMs) using dual-arm robots.
It includes resources for teleoperation, vizualization, robot control, DMP framework etc used in the work.

<img src="https://github.com/user-attachments/assets/ac687ec8-7c5c-4043-93a6-773e922ca2fe" alt="peg_in_hole" width="400"/>


## Repository Structure

- **robot_bringup**: Contains launch files and configurations for visualizing the dual-arm robot in RViz.
- **motion_mapping**: Includes scripts for teleoperation using either a mocap suit or HTC Vive controller.
- **dmp_inter**: Provides the scripts and configurations for loading and executing DMPs.
- **dual_arm_controller**: Contains the controller for managing dual-arm operations on real robots via Libfranka and in simulation (using Gazebo).

## Instructions

### 1. Robot Visualization

To visualize the dual-arm robot in RViz:

1. Navigate to the `robot_bringup` workspace.
2. Launch the robot URDF using the following command:

   ```bash
   roslaunch robot_bringup panda_dual.launch
   ```


   
### 2. Teleoperation (workspace: motion_mapping)

#### - Using a Mocap Suit

**Dependencies:**

- [CppServer](https://github.com/chronoxor/CppServer.git) - https://github.com/chronoxor/CppServer.git
- Axis Neuron software: Software for capturing and processing mocap data.
- BVH data (Motion capture data) from the MoCap suit.

**Instructions:**

1. **Set Up the Mocap Suit:**
   - Ensure the MoCap suit is properly calibrated. Publish the BVH data via TCP (in Axis Neuron software [Windows OS])

2. **Run the MoCap TCL client Script (in Ubuntu 20.04):**
   - Receives the BVH data from Windows system via TCP :
     ```bash
     rosrun motion_mapping convert_joints_tcp.cpp
     ```

3. **Run the BVH parser and transformations publisher Script:**
   
     ```bash
     rosrun motion_mapping pn_tf.cpp
     ```

#### - Using HTC Vive Controller

**Dependencies:**

- HTC Vive hardware: Ensure you have the HTC Vive setup and operational.
- [ROS-based teleop](https://github.com/uts-magic-lab/htc_vive_teleop_stuff.git): ROS package for HTC Vive teleoperation. - https://github.com/uts-magic-lab/htc_vive_teleop_stuff.git

**Instructions:**

1. **Set Up HTC Vive:**
   - Connect and calibrate the HTC Vive controller ( VR steam)

2. **Launch HTC Teleoperation:**
   - Start the HTC Vive teleoperation node to publsih the VR controller data.
   - Open RViz to visualize the TFs of the controllers, trackers etc.

#### - **perform inverse kinematics :**

     **Dependencies:**
	
     - [Panda Analytical Kinematics](https://git.rob.cs.tu-bs.de/public_repos/irp_papers/panda_analytical_kinematics.git) - https://git.rob.cs.tu-bs.de/public_repos/irp_papers/panda_analytical_kinematics.git



      **Run the IK Script:**
      - Perform IK for the given input poses to compute the required joint angles:
      ```bash
      rosrun motion_mapping pn_inv.cpp
      ```

### 3. DMP Execution (Workspace: dmp_inter)

**Dependencies:**

- [DMP Library](https://github.com/mginesi/dmp_pp.git) -  https://github.com/mginesi/dmp_pp.git

**Instructions:**

1. **Navigate to the `dmp_inter` Workspace:**

2. **Run the Main DMP Script:**
 
     ```bash
     rosrun dmp_int dmp_main.py
     ```

3. **Run the Weights Client:**
  
     ```bash
     rosrun dmp_int learn_client_multiple.py
     ```

4. **Run the Execute Client:**
   
     ```bash
     rosrun dmp_int execute_client.py
     ```

5. **Run the GUI Buttons Application:**
   - Use the GUI to load required weights and execute the trajectory:
     ```bash
     rosrun dmp_int gui_buttons_list.py
     ```
   - In the GUI:
     - Load the required weights.
     - Select the required execution.
     - Can be used to Play/Pause the execution.

### 4. Dual-Arm Robot Control 

The dual-arm robot control can be executed either in simulation (using Gazebo) or on real robots (via Libfranka).


**For real robot execution:** 

Compile the workspace `dual_arm_controller` which is based on libfranka and customized to run in ROS environment.
Execute the controller file  - `dual_arm_controller.cpp` -  with the ip addresses of the robots
	
**For both simulation and real robot execution:**

Include the files in the for_simulation folder into the franka_ros (https://github.com/frankaemika/franka_ros) repository. 
Compile and execute the respective controller file (left arm or right arm).



## Videos

- **Teleoperation Task:**
  - Teleoperation using VR controllers:

    <img src="https://github.com/user-attachments/assets/16e4f347-2e4b-49d4-8e55-4dc9d8027afc" alt="peg_in_hole" width="300"/>

     https://youtu.be/MeMfTPp9wsI- 

  - Teleoperation using the MoCap suit:

    https://youtube.com/shorts/3EEvYbyEjWQ?feature=share
    



- **Carrying Task:**
  - Dual-arm robots perform a carrying task:
    
   <img src="https://github.com/user-attachments/assets/a40529cc-b13c-4125-b669-03a9b564eb7a" alt="peg_in_hole" width="200"/>

   https://youtu.be/XueQkZspCuc
   

- **Peg-in-Hole Task:**
  - Peg-in-hole task execution:
    
   <img src="https://github.com/user-attachments/assets/ba312654-873a-4e6a-99fd-9a7dd1d18076" alt="peg_in_hole" width="200"/>

   https://youtu.be/GvobSitCnNw
  




