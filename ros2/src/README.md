# Intrinsic Tactile Sensing (ITS) ROS Package

## ROS Nodes
### This package ( _its_ )contains the ROS nodes for the Intrinsic Tactile Sensing (ITS) algorithm. It includes the following nodes:

- **its_node**: The main node that implements the ITS algorithm (i.e., exploiting the soft finger contact model on a rigid surface)
- **soft_its_node**: The main node that implements the Soft ITS algorithm (i.e., its variation by taking into account the force-deformation relationship of a deformable surface)
- **soft_its_viz**: A node that visualizes the ITS solution on a new window.

- **tactip_markers_tracker**: A python node that tracks markers on a TacTip frame.
- **tactip_gaussian_kernel**: A python node that estimates the markers density over a Tactip frame by using a Gaussian Kernel Density (GKD) model.
- **gkd_node**: A C++ node that estimates the markers density over a Tactip frame by using a Gaussian Kernel Density (GKD) model.
- **its_tactip_viz**: A node that visualizes the Tactip GKD in a new window.

## ROS Messages
### This package ( _its_msgs_ ) contains the ROS messages for the Intrinsic Tactile Sensing (ITS) algorithm.

- **its_msgs/Point2D**: A message to represent the coordinates of a point in a camera plane.
- **its_msgs/SoftContactSensingProblemSolution**: A message that contains the solution of the soft contact sensing problem. (i.e., the contact points, the contact forces and torques, the contact normals and the contact depths).
- **its_msgs/TacTipDensity**: A message that contains the Tactip density estimation in a frame.
- **its_msgs/TacTipMarkers**: A message that contains all the markers centroid positions in a frame.

## ROS Parameters
### This package ( _its_ ) exploits the definition of the following parameter to address the solution.


#### ITS parameters:
- **fingertip**: Fingertip parameters
  - **id**: Fingertip name id
  - **displacement**: Displacement [mm] Fingertip Frame {B} w.r.t Sensor Frame {S}
    - **x**: Along x-axis of {B}, [mm]
    - **y**: Along y-axis of {B}, [mm]
    - **z**: Along z-axis of {B}, [mm]
  - **orientation**: Orientation [rad] Fingertip Frame {B} w.r.t Sensor Frame {S} in Roll-Pitch-Yaw encoding
    - **roll**: Along x-axis of {B}, [rad]
    - **pitch**: Along y-axis of {B}, [rad]
    - **yaw**: Along z-axis of {B}, [rad]
  - **principalSemiAxis**: Fingertip Surfaces principal Semi-Axis
    - **a**: Along x-axis of {B}, [mm]
    - **b**: Along y-axis of {B}, [mm]
    - **c**: Along z-axis of {B}, [mm]
  - **stiffnessType**: Fingertip Surfaces stiffness model: $F = a * \Delta d + b * \Delta d^2$
    - **a**: linear coeff. [N/mm]
    - **b**: quadratic coeff.   [N/mm^2]


- **sensor**: Force/Torque Sensor parameters
  - **id**: F/T Sensor name id


- **soft_its**: Soft Intrinsic Tactile Sensing solver parameters
  - **rate**: Rate of node in [Hz]
  - **algorithm**:
    - **verbose**: print output on each step
    - **force_threshold**: Force threshold to enable solver [N]
    - **method**: Parameters of ITS solver
      - **name**: solver method name(Default Levenberg-Marquardt)
      - **params**:
        - **count_max**: num of max iteration until forced stop
        - **stop_threshold**: Xi_square threshold for convergence condition
        - **epsilon**: Lavenberg-Marquantd updating param of lambda


#### TacTip parameters:
- **tactip**: TacTip parameters
  - **camera**: TacTip camera intrinsic parameter
    - **depth**: distance camera lens to tip [mm]
    - **fx**: x focal lenght
    - **fy**: y focal lenght
    - **cx**: x camera center
    - **cy**: y camera center
    - **distortion_coeff**: distortion
  - **markers**: Number of markes
  - **mm2pxl**: conversion millimeters to pixel in a frame
- **image_processing**: Image processing parameters
  - **shape**: Reshaped image dimension
    - **width**: width in pixels
    - **height**: height in pixels
  - **mask**: Image mask
  - **blur_kernel_size**: Image noise filtering (Gaussian) kernel size
  - **circle_recog**: Hough Gradient Circle recogition params
    - **min_radius**: min circle radius
    - **max_radius**: min circle radius
    - **param1**: Upper threshold for the internal Canny edge detecto
    - **param2**: Threshold for center detection
    - **min_distance**: min distance of adjacent circle
- **markers_density**: Params for Gaussian Kernel Algorithm
  - **rate**: rate [Hz]
  - **gaussian_kernel**: Density Estimator Gaussian Kernel
    - **h**: Density Estimator Gaussian Kernel std. dev, (TacTip: 15, DigiTac: 30)
    - **verbose**: Print kernel estimation elapsed time
  - **resolution**: Resolution of pixels for Density estimation
    (i.e.,  d(u,v) = density[u//resolution,v//resolution])
  - **threshold**: Maximum density variation allowed in a pixel before contact detection [1/mm^2]
  - **fit**: Fit parameters for deformation estimation
    - **type**: type of curve (i.e linear, quadratic, power, logarithmic)
    - **a**: a
    - **b**: b


## ROS Launch
## This package (_its_ros_) contains the ROS launches to ease the setup of the Intrinsic Tactile Sensing (ITS) framework.

- **demo.launch**: Launch file to run the demo on the TacTip. It starts the following nodes/launch:
    - **netft_rdt_driver/ft_sensor.launch**: Launch file to run the f/t sensor.
    - **tactip_driver/tactip_camera.launch**: Launch file to run the TacTip camera.
    - **its/its_node**: Launch file to run the ITS algorithm.
    - **its/tactip_markers_tracker**: Launch file to run the TacTip Markers Tracker.
    - **its/gkd_node**: Launch file to run the Gaussian Kernel Density algorithm.
    - **tactip_viz**: Launch file to run the TacTip visualization.

- **setup.launch**: Launch file to bringup the environment used of the validation framework.

- **its.launch**: Launch file to run the ITS algorithm. It starts the following nodes/launch:
    - **its/its_node** or **its/soft_its_node**: Launch file to run the ITS algorithm.
    - **its/soft_its_viz**: Launch file to run the Soft ITS visualization.

- **tactip.launch**: Launch file to run the Tactip processing nodes. It starts the following nodes/launch:
    - **tactip_driver/tactip_camera.launch**: Launch file to run the TacTip camera.
    - **its/tactip_markers_tracker**: Launch file to run the TacTip Markers Tracker.
    - **its/gkd_node**: Launch file to run the TacTip Gaussian Kernel Density algorithm.
    - **its/tactip_viz**: Launch file to run the TacTip Gaussian Kernel Density visualization.



## ITS Algorithm

## Soft ITS Algorithm

## TacTip analysis


# To Do
[] Riorganizza Fingertip in una classe
[] Convex hull viz
[] Rviz config
[] Launch
