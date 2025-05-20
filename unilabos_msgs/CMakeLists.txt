cmake_minimum_required(VERSION 3.5)
project(unilabos_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(action_files
  "action/SendCmd.action"
  "action/Stir.action"
  "action/HeatChill.action"
  "action/HeatChillStart.action"
  "action/HeatChillStop.action"

  "action/LiquidHandlerAspirate.action"
  "action/LiquidHandlerDiscardTips.action"
  "action/LiquidHandlerDispense.action"
  "action/LiquidHandlerDropTips.action"
  "action/LiquidHandlerDropTips96.action"
  "action/LiquidHandlerMoveLid.action"
  "action/LiquidHandlerMovePlate.action"
  "action/LiquidHandlerMoveResource.action"
  "action/LiquidHandlerPickUpTips.action"
  "action/LiquidHandlerPickUpTips96.action"
  "action/LiquidHandlerReturnTips.action"
  "action/LiquidHandlerReturnTips96.action"
  "action/LiquidHandlerStamp.action"
  "action/LiquidHandlerTransfer.action"

  "action/DPLiquidHandlerAddLiquid.action"
  "action/DPLiquidHandlerCustomDelay.action"
  "action/DPLiquidHandlerMix.action"
  "action/DPLiquidHandlerMoveTo.action"
  "action/DPLiquidHandlerRemoveLiquid.action"
  "action/DPLiquidHandlerSetTiprack.action"
  "action/DPLiquidHandlerTouchTip.action"
  "action/DPLiquidHandlerTransferLiquid.action"

  "action/EmptyIn.action"
  "action/FloatSingleInput.action"
  "action/IntSingleInput.action"
  "action/StrSingleInput.action"
  "action/Point3DSeparateInput.action"

  "action/ResourceCreateFromOuter.action"

  "action/SolidDispenseAddPowderTube.action"
  
  "action/PumpTransfer.action"
  "action/Clean.action"
  "action/Separate.action"
  "action/Evaporate.action"
  "action/EvacuateAndRefill.action"

  "action/WorkStationRun.action"
  "action/AGVTransfer.action"
)

set(srv_files
  "srv/Stop.srv"
  "srv/SerialCommand.srv"
  "srv/ResourceGet.srv"
  "srv/ResourceList.srv"
  "srv/ResourceAdd.srv"
  "srv/ResourceUpdate.srv"
  "srv/ResourceDelete.srv"
)

set(msg_files
  "msg/State.msg"
  "msg/Resource.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
