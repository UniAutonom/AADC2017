set(FILTER_NAME aadc_cameraCalibration)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

	#set include directories
	include_directories(${OpenCV_INCLUDE_DIR}) 

    set(HEADER_MOC
        displaywidget.h
        cCameraCalibration.h
        )
		
	adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )



    # add the source
    adtf_add_filter(${FILTER_NAME}
        stdafx.h
        calibrationSettings.h
		displaywidget.cpp
        cCameraCalibration.h
        cCameraCalibration.cpp
        )

	# linking libs
	target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

	# set folder (only needed and available for MSVC)
	adtf_set_folder(${FILTER_NAME} camera) 

	# Specify where it should be installed to
	adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
		
endif(QT_FOUND)
