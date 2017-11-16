


set(FILTER_NAME aadc_arduinoCommunication)

include_directories(
    ${Boost_INCLUDE_DIR} 
)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
        stdafx.h      
		arduino_com_helpers.h
		arduino_com_client.h
        arduino_com_client.cpp
        cArduinoCommunication.h
        cArduinoCommunication.cpp   
        SerialDeviceBoost.h
        SerialDeviceBoost.cpp
        SimpleTimer.h
        Logger.h
    )

TARGET_LINK_LIBRARIES( ${FILTER_NAME} ${Boost_LIBRARIES} )

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} arduino) 
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

if(WIN32)
    install(FILES "${BOOST_ROOT}/bin/boost_system-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
    install(FILES "${BOOST_ROOT}/bin/boost_chrono-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
    install(FILES "${BOOST_ROOT}/bin/boost_thread-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
    install(FILES "${BOOST_ROOT}/bin/boost_date_time-vc100-mt-1_58.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo Debug)
endif(WIN32)