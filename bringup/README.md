# bringup
Handles sensor start-up and configuration

## Installation
1. **Install ZED 2i packages**
   - Install [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)

    Notes: 
    - [An NVIDIA® GPU with Compute Capabilities >= 5.3 based on Pascal or more recent micro-architecture is required.](https://www.stereolabs.com/docs/development/zed-sdk/specifications) 
    - Select the ZED SDK with CUDA and TensorRT versions compatible with your specific GPU.

2. **Install VLP-16 packages**
   ```bash
   sudo apt install ros-${ROS_DISTRO}-velodyne
   ```

## Configuration

### ZED 2i Configuration
**IMPORTANT:**
If you are running any of the NEURAL depth modes on a new device, optimization will take time so **Do not use NEURAL depth modes at dynamic events unless it is already optimized for the compute platform** due to the limited time per run.
- **Config files location:** `zed_wrapper/config/`
  - `common_stereo.yaml` - General stereo camera settings
  - `zed2i.yaml` - ZED 2i specific parameters

### VLP-16 Configuration
**IMPORTANT:** Use `VLP_hires_db.yaml` instead of `VLP16_db.yaml` in the calibration params in `velodyne/launch/velodyne-all-nodes-VLP16-launch.py` since we are using the VLP-16 Hi-Res, which has a narrower VFOV. Otherwise, the LiDAR point cloud would appear angularly stretched vertically compared to the ZED 2i's point cloud, even when perfectly aligned.

- **Config files location:** `velodyne_driver/config/`
  - `VLP16-velodyne_driver_node-params.yaml` - Driver parameters
  - `VLP16-velodyne_transform_node-params.yaml` - Transform parameters
  - `default-velodyne_laserscan_node-params.yaml` - Laser scan parameters
- **Launch file:** `velodyne/launch/velodyne-all-nodes-VLP16-launch.py`

    **Note:** If this is the first time setting up the VLP-16 LiDAR on a new device, follow the [ROS Velodyne tutorial](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

Refer to the individual config files for detailed parameter descriptions and default values.

## Usage
1. **Source your virtual environment (for the ZED python dependencies)**
2. **Launch sensor bringup:**
   ```bash
   ros2 launch bringup sensor_bringup.launch.py
   ```



## Sensors

| Sensor | Model | Datasheet | Notes |
|--------|-------|-----------|-------|
| ZED 2i | 4mm (with polarizer) | [ZED 2i Datasheet](https://cdn.sanity.io/files/s18ewfw4/staging/c059860f8fe49f3856f6b8da770eb13cc543ac2c.pdf/ZED%202i%20Datasheet%20v1.2.pdf) | Main stereo camera for perception |
| VLP-16 | Hi-Res | [VLP-16 Hi-Res Datasheet](https://www.mapix.com/wp-content/uploads/2018/07/63-9318_Rev-E_Puck-Hi-Res_Datasheet_Web.pdf) | High-resolution LiDAR sensor |






