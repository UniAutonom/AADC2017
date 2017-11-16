# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_SpeedGovernor)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cSpeedGovernor.h
    cSpeedGovernor.cpp
)

adtf_set_folder(${FILTER_NAME} SpeedGovernor) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
