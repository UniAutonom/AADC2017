set(FILTER_NAME aadc_calibrationXml)

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                 stdafx.h
                 cCalibrationXml.h
                 cCalibrationXml.cpp
                 cCubic.h
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} calibration) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
