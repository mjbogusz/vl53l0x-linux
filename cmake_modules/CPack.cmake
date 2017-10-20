set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "vl53l0x-linux")
set(CPACK_PACKAGE_VERSION_MAJOR ${LIB_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${LIB_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${LIB_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION ${LIB_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "wiringpi-odroid (>=2.42)")
set(CPACK_PACKAGE_CONTACT "MJBogusz <mjbogusz.find.me.on.github@not.a.valid.email.address.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "libraries")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_PACKAGE_DESCRIPTION
        "VL53L0X ToF sensor library, aimed at Single Board Computers like Raspberry Pi and Odroid boards."
        )
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${CPACK_PACKAGE_DESCRIPTION})
set(CPACK_SOURCE_STRIP_FILES true)
set(CPACK_STRIP_FILES true)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
        "${CMAKE_CURRENT_SOURCE_DIR}/build/debian/postinst;" +
        "${CMAKE_CURRENT_SOURCE_DIR}/build/debian/postrm;"
        )
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

# Architecture auto-detection. Idea from http://stackoverflow.com/questions/11944060/how-to-detect-target-architecture-using-cmake
# x86* architectures added for development purposes - this library won't work on these.
execute_process(COMMAND uname -m COMMAND tr -d '\n\r' OUTPUT_VARIABLE ARCHITECTURE)
if ("${ARCHITECTURE}" STREQUAL "x86_64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    message(STATUS "Detected architecture: amd64")
elseif ("${ARCHITECTURE}" MATCHES "i.86")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
    message(STATUS "Detected architecture: i386")
elseif ("${ARCHITECTURE}" STREQUAL "armhf" OR "${ARCHITECTURE}" STREQUAL "armv7l")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
    message(STATUS "Detected architecture: armhf")
elseif ("${ARCHITECTURE}" STREQUAL "aarch64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
    message(STATUS "Detected architecture: arm64")
else ()
    message(FATAL_ERROR "Unrecognized processor architecture! Want it added? Create a pull request.")
endif ()

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

include(CPack)
