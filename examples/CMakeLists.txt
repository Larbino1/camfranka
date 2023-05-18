#project(libfranka-examples CXX)

#list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(Franka REQUIRED)
#find_package(Eigen3 REQUIRED)
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

#add_library(examples_common STATIC
#  examples_common.cpp
#)
#target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(MYEXAMPLES
  combined_impedance_control
  compliant_milling
  geometric_port_impedance_control
  instrument_impedance_control
  joint_identification
  julia
  lettuce_shaker
  port_impedance_control
  port_impedance_control_line
  register_frame
  sandbox
  virtual_instrument
)


foreach(example ${MYEXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
  target_link_libraries(${example} MyLib MyJson JoyInputLib)
endforeach()

target_link_libraries(sandbox Threads::Threads)
target_link_libraries(instrument_impedance_control Threads::Threads)
target_link_libraries(virtual_instrument VirtualInstrumentLib)

#include(GNUInstallDirs)
#install(TARGETS ${EXAMPLES}
#  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#)
