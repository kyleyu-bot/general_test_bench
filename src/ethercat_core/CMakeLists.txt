add_library(ethercat_core STATIC
    master.cpp
    loop.cpp
    devices/beckhoff/el2004/adapter.cpp
    devices/beckhoff/elm3002/adapter.cpp
    devices/beckhoff/el5032/adapter.cpp
    devices/motor_drives/drive_bases/ds402/pdo.cpp
    devices/motor_drives/Novanta/Everest/pdo.cpp
    devices/motor_drives/Novanta/Everest/adapter.cpp
    devices/motor_drives/Novanta/Volcano/pdo.cpp
    devices/motor_drives/Novanta/Volcano/adapter.cpp
)

target_include_directories(ethercat_core
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
    PRIVATE
        # SOEM v1.4.0 uses the old-style include_directories() which does not
        # propagate through the 'soem' target.  Add the paths explicitly here.
        # For SOEM v2.x (cmake_minimum_required 3.28+) these may be unnecessary
        # if that version properly uses target_include_directories(INTERFACE ...).
        ${soem_SOURCE_DIR}/soem
        ${soem_SOURCE_DIR}/osal
        ${soem_SOURCE_DIR}/osal/linux
        ${soem_SOURCE_DIR}/oshw/linux
)

target_link_libraries(ethercat_core
    PUBLIC
        nlohmann_json::nlohmann_json
    PRIVATE
        soem
        pthread
        rt
)

target_compile_options(ethercat_core PRIVATE
    -Wall
    -Wextra
    # -Wpedantic is omitted: SOEM v1.4.0 headers contain anonymous structs and
    # zero-size arrays that trip pedantic warnings in C++ mode.
)
