
if(AADC_PYLON_FOUND)
    # This is required by CMake
    cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
    #set project name at beginning this is cleaner than just a folder
    project(aadc_baslerCamera)

    set(FILTER_NAME ${PROJECT_NAME})
    set(MAIN_FOLDER "plugins")

    include_directories(
        ${PROJECT_SOURCE_DIR}
        ../../include
        SYSTEM ${PYLON_INCLUDE_DIRS}
    )

    link_directories(
        ${PYLON_LIBRARY_DIRS}
    )

    set(HEADER
        stdafx.h 
        cBaslerCamera.h
    )

    set(SOURCE
        cBaslerCamera.cpp
    )


    adtf_add_filter(${FILTER_NAME}
        ${HEADER}
        ${SOURCE}  
    
    )

    target_link_libraries(${FILTER_NAME}
        ${PYLON_LIBS}
    )

    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

    # set folder (only needed and available for MSVC)
	adtf_set_folder(${FILTER_NAME} camera) 

    if (WIN32)
        install(FILES "${GENICAM_BINARY_DIRS}/GenApi_MD_VC100_v2_4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${GENICAM_BINARY_DIRS}/GCBase_MD_VC100_v2_4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${GENICAM_BINARY_DIRS}/Log_MD_VC100_v2_4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${GENICAM_BINARY_DIRS}/log4cpp_MD_VC100_v2_4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${GENICAM_BINARY_DIRS}/MathParser_MD_VC100_v2_4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_BINARY_DIRS}/PylonBase_MD_VC100.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_BINARY_DIRS}/PylonUtility_MD_VC100.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_BINARY_DIRS}/PylonUsb_MD_VC100_TL.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_BINARY_DIRS}/MVGenICamConfig.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_BINARY_DIRS}/Xerces-C_MD_VC100_v2_7_1.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_GENICAM_BINARY_DIR}/GCBase_MD_VC100_v2_3.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_GENICAM_BINARY_DIR}/GenApi_MD_VC100_v2_3.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_GENICAM_BINARY_DIR}/Log_MD_VC100_v2_3.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_GENICAM_BINARY_DIR}/MathParser_MD_VC100_v2_3.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
        install(FILES "${PYLON_GENICAM_BINARY_DIR}/log4cpp_MD_VC100_v2_3.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
    endif (WIN32)

endif(AADC_PYLON_FOUND)
