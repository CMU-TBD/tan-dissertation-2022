# Change log

#### 2021-05-28
- **[Added]** CFG for human layer and interaction layer.
- **[Added]** Now allow slack in front of the robot in interaction layer.
- **[Changed]** Human is now a elipse - Prithu
- **[Fixed]** Bug where human layer and interaction layer wasn't showing up correctly in local cost map.
#### 2021-04-28
- **[Added]** New map for the lab.
- **[Fixed]** A bug where e-stopping Podi at the beginning makes it not able to calibrate.
- **[Changed]** Updated parameters for different layers.
#### 2021-04-19
- **[Added]** Added a `25`mm and distinguish between version
- **[Changed]** Local planner can now take in plugins
- **[Fixed]** human and itneraction layers can now pick the right frame to convert to.
#### 2021-04-15
- **[Changed]** Decreased the rotation tolerance to 0.1 radian. 15 degrees (0.25 radian) is way too much.
- **[Changed]** Interaction Space's cost is now lower, to prevent Podi from getting stuck.
#### 2021-04-13
- **[Added]** parameters for the human and interaction space layers to ignore time stamps.
- **[Changed]** Made the Podi footprint smaller and tuned some ROS Nav variables.
- **[Fixed]** Bugs where the interaction space is not being remove.
#### 2021-04-12
- **[Added]** A plugin layer map that listen to the `/humans` topic to locate detected humans and add them to the global navigation map.
- **[Added]** A plugin layer map that listen to the `/interaction_space` topic to locate potential spatial groups and add them to the global navigaiton map. 
- **[Changed]** Higher friction co-efficient in the wheels to better mirror real life situation
- **[Changed]** Change Podi simulator odometry to use `world` instead, this is to better reflect real life situation
- **[Changed]** Tuned the ros navigation configurations to intergrate new layers. 
#### 2021-04-09
- **[Changed]** Cleanup the code that handled joystick commands and also use enum for easy changing in the future.
- **[Changed]** Skipped local node that transform and cutoff laser reading and now relies on p3dx configuration to set the laser.
- **[Added]** the software e-stop now always disable/enable the robot's motor as an additional level of safety.

## [1.6.0] - 2021-03-25
- **[Added]** Map for simulated lab.
- **[Changed]** Fix the simulator map being the wrong map.
- **[Changed]** Change simulator's output topic name to match names from RosAria.
- **[Changed]** Controller will now publish [0,0] movement if there is no incoming movement signals.
- **[Changed]** simplify map building in simulator.
- **[Changed]** Increase Podi's minimum/maximum rotation speed
- **[Fixed]** Podi's description was set incorrectly.

## [1.5.0]
- **[Added]** Added enabling switch to Podi. When true, the button must be hold down to move.
- **[Added]** 3rd floor hallway map in NSH.

## [1.4.0] 
- **[BREAKING]** Changed namespace to work with ZED by prefixing by `podi_` instead through `podi/` 
- **[Fixed]** Issues arised from the namespace problems.

## [1.3.1] - 2020-09-20
- **[Fixed]** Issue where robot will stop turning after 1 second of the same command.
- **[Added]** Infrastructure to add different joystick types and customize buttons.

## [1.3.0] - 2020-09-15
- **[Changed]** Change Camera from AzureKinect to Zed2
- **[Changed]** Change namespace to be more consistent
- **[Changed]** Improved `computer_state_node`
- **[Added]** ability to set whether the handle to be full rotation mode or fixed.

## [1.2.0] - 2020-09-08
- **[Added]** Merged in simulation gazebo package to simplify dependencies.
- **[Added]** VCS repo code to build the system.
- **[Changed]** Migrated all files to be runnable on ROS Noetic (updated CMAKE file etc.) 

## [1.1.5] - 2020-08-17
- **[Changed]** Tweaked the inertia models of podi to try improve odom readings.
- **[Added]** Modified Azure Kinect URDF.
- **[Changed]** Camera of Podi is now an Azure Kinect. Gazebo will now publish color, depth, and point clound image. 

## [1.1.4] - 2020-06-23
#### Changed
- Remove `navigation` from `package.xml`

## [1.1.3] - 2020-06-12
#### Changed
- Changed podi laserscanner's type from `gpu_ray` to `ray`

## [1.1.2] - 2020-05-23
#### Added 
- Ability to set initial pose for ROS Navigation.
- Continuous Integration using Github Action. Check for catkin build errors.

#### Changed
- Updated the Camera and LiDAR's Gazebo numbers to better match what the real system has. 
- Cleaned up `package.xml` to enable CI.

## [1.1.1] - 2020-04-30
#### Changed
- Incorrect topic publishing name for front camera changed from `podi/front_camera/podi/image_raw` to `podi/front_camera/image_raw`

## [1.1.0] - 2020-04-23
#### Added
- `tbd_podi_2dnav` that acts an interface for ROS Navigation packages. Migrated in old settings from other podi repositories.
- `maps` folder and multiple maps. Including `sim_NSH_04_2020.yaml` that describe the gazebo environment.
- added rosparam `vel_latch_duration` that specifies how long old velocities are published if no need velocities are received.

#### Changed
- Relied on TF_prefix and namespace to specify type of robot instead of encoding the name into the URDF.
- Moved the maps originally in `tbd_podi_common` into `tbd_podi_2dnav`.
- Refactor `control_robot_node` to improve readibility and control flow. 

#### Fixed
- Incorrect namespace caused Gazebo to crash.
- Incorrect Laser parameter in the Gazebo description of the front laser
- Incorrect calculation for the laser limiter node that ignores certain readings.
- Incorrect differential drive Gazebo setting causing wildly in-accurate URDF.
- Bug where empty twist will be sent to the robot if there is no new command at the current control loop. Added latch duration that republish old velocities.

#### Removed 
- unused launch files.

## [1.0.1] - 2020-04-21
#### Added
- This changelog.
#### Changed
- Have all topics and TF frame to be prefixed with `podi` to prevent collision in multi-robot situations.
- Changed the differential driver gazebo plugin to use the `podi` prefix
- Changed some arguments in roslaunch file to use lower_underscore_case instead of Camelcase.

## [1.0.0] - 2020-XX-XX
Initial Release with face, msgs, description, and control components.
