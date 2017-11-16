/*********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hartlan $  $Date:: 2017-05-12 13:29:08#$ $Rev:: 63140   $
**********************************************************************/

#ifndef _AADC_ENUMS_H
#define _AADC_ENUMS_H

namespace SensorDefinition
{
    /*! enums that represent sensors. */
    enum sensorEnums{
        US_FRONTLEFT = 0,
        US_FRONTCENTERLEFT,
        US_FRONTCENTER,
        US_FRONTCENTERRIGHT,
        US_FRONTRIGHT,
        US_SIDERIGHT,
        US_SIDELEFT,
        US_REARCENTERLEFT,
        US_REARCENTER,
        US_REARCENTERRIGHT,
        WHEEL_LEFT,
        WHEEL_RIGHT,
        VOLTAGE,
        VOLTAGE_SENSORS,
        VOLTAGE_ACTUATORS,
        IMU        
    };
    
    /*! Number of sensors */
    const int sensorCount = 16;

}

namespace Unia
{
    enum maneuverIds{
        LEFT = 0,
        RIGHT,
        STRAIGHT,
        PARKING_IN,
        PARKING_OUT_LEFT,
        PARKING_OUT_RIGHT,
        PARKING_SPACE_SEARCH,
        PEDESTRIAN_CROSSING,
        OVERTAKING_CHECK,
        OVERTAKING_RUN,
        ResumeDefault
    };
}

#endif
