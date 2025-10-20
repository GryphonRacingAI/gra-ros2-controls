## [FSAI_CM](https://leeds365.sharepoint.com/:u:/r/sites/LeedsGryphonRacing/Shared%20Documents/Formula%20Student%20Shared%20Drive/Software/CarMaker_FSAI/FSAI_CM.zip?csf=1&web=1&e=2aXHV3) 
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
