### Changes to CMakeLists.txt of `ros/ros2_ws/src/carmaker_rds_client` & `ros/ros2_ws/src/cmrosutils`
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
