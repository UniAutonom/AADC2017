set(FILTER_NAME aadc_steeringCalibration)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        cCalibrationView.h
        cCalibrationModel.h
        )
    
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )

    # declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cSteeringCalibrationFilter.h
    cSteeringCalibrationFilter.cpp
    cCalibrationView.h
    cCalibrationView.cpp
    angleDiff.h
    angleDiff.cpp
    cCalibrationModel.h
    cCalibrationModel.cpp
    )

    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} calibration) 

    # Specify where it should be installed to
    adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

endif(QT_FOUND)