
#find_package(Franka REQUIRED)

add_subdirectory(joyInput)

add_library(MyLib myLib.cpp)
add_library(VirtualInstrumentLib virtualInstrument.cpp)
add_library(MyJson myJson.cpp)

target_link_libraries(MyLib PUBLIC Franka::Franka Eigen3::Eigen3)
target_link_libraries(VirtualInstrumentLib PUBLIC Franka::Franka Eigen3::Eigen3)
target_link_libraries(MyJson PUBLIC Franka::Franka Eigen3::Eigen3 nlohmann_json::nlohmann_json MyLib)
target_link_libraries(MyJson PUBLIC -lstdc++fs)
target_include_directories(MyLib
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)
