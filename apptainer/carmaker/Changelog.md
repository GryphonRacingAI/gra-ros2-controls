## [FSAI_CM](https://leeds365.sharepoint.com/:u:/r/sites/LeedsGryphonRacing/Shared%20Documents/Formula%20Student%20Shared%20Drive/Software/CarMaker_FSAI/FSAI_CM.zip?csf=1&web=1&e=2aXHV3) 

### 1. Changes to CMakeLists.txt of `ros/ros2_ws/src/carmaker_rds_client` & `ros/ros2_ws/src/cmrosutils`
@deletions
- if(NOT ${GET_CM_VERSION_FROM_CMD_LINE_OR_CACHE})
-   set(CARMAKER_VER 12.0.1)
-   set(CARMAKER_DIR /opt/ipg/carmaker/linux64)
- else()
-   set(CARMAKER_VER
-       CACHE STRING "CarMaker Version, e.g. 12.0.1")
- 
-   set(CARMAKER_DIR /opt/ipg/carmaker/linux64}
-       CACHE STRING "CarMaker installation directory")
- endif()

@edits
- set(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)
+ set(CARMAKER_INC_DIR /opt/ipg/carmaker/linux64/include)


### 2. /CMStart.sh
@edit last line to
`/opt/ipg/bin/CM . -apphost localhost -ext GUI/CMExt-CMRosIF.mod`


### 3. If you are using ROS2 Jazzy: make the following changes
@file: `FSAI_CM/ros/ros2_ws/src/hellocm/src/ROS2_HelloCM.cpp`
- Line 105 to:
```cpp
timer_ = this->create_wall_timer(std::chrono::milliseconds(cycle_time_),
                                HelloCM::on_timer);
```
- Line 202: 
```cpp
timer_ = this->create_wall_timer(std::chrono::milliseconds(cycle_time_), HelloCM::on_timer);
```

- Line 18: no need this line