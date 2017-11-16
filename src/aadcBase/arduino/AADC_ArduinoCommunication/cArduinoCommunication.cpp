/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra  $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#include "stdafx.h"
#include "cArduinoCommunication.h"

ADTF_FILTER_PLUGIN("AADC Arduino Communication", OID_ADTF_ARDUINOCOM_FILTER, cArduinoCommunication)

cArduinoCommunication::cArduinoCommunication(const tChar* __info) : adtf::cTimeTriggeredFilter(__info)
{
    m_bLoggingModeEnabled = tFalse;
    SetPropertyBool("Log serial communication", m_bLoggingModeEnabled);
    SetPropertyStr("Log serial communication" NSSUBPROP_DESCRIPTION, "Enable logging of arduino communication to file here");
}

cArduinoCommunication::~cArduinoCommunication()
{

}

tResult cArduinoCommunication::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr))
    m_bLoggingModeEnabled = GetPropertyBool("Log serial communication");

    if (eStage == StageFirst)
    {
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to create Output Pins");

        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {
        for (int i = 0; i < NUM_ARDUINO; i++)
        {
            std::stringstream ss;
            ss << "/dev/ttyACM" << i;
            if (m_com[i].begin(ss.str().c_str()))
            {
                m_valid_ids.push_back(i);
                LOG_INFO(cString::Format("Device found on port: /dev/ttyACM%d", i));
            }
            else
            {
                LOG_INFO(cString::Format("No device found on port: /dev/ttyACM%d", i));
            }
        }

        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            LOG_INFO(cString::Format("Start reading at port: /dev/ttyACM%d", m_valid_ids[i]));
            bool result = m_com[m_valid_ids[i]].start_reading();
            if (result)
            {
                LOG_INFO(cString::Format("Connected to arduino on: /dev/ttyACM%d\t ID: %d\t Software version: %d",
                                         m_valid_ids[i], m_com[m_valid_ids[i]].get_id(), m_com[m_valid_ids[i]].get_software_version()));
                if (m_bLoggingModeEnabled)m_com[m_valid_ids[i]].start_logging();
            }
            else
            {
                LOG_ERROR(cString::Format("Could not find an arduino with correct software on: /dev/ttyACM%d", m_valid_ids[i]));
            }
        }

        // every 1ms
        this->SetInterval(1000);
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::Start(__exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Start(__exception_ptr));
    RETURN_NOERROR;
}

tResult cArduinoCommunication::Stop(__exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}

tResult cArduinoCommunication::Shutdown(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr));

    if (eStage == StageNormal)
    {
        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            LOG_INFO(cString::Format("Stop reading at port: /dev/ttyACM%d", m_valid_ids[i]));
            m_com[m_valid_ids[i]].stop_reading();
            if (m_bLoggingModeEnabled)m_com[m_valid_ids[i]].stop_logging();
        }


        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            LOG_INFO(cString::Format("Closing port: /dev/ttyACM%d", m_valid_ids[i]));
            m_com[m_valid_ids[i]].end();
        }
        m_valid_ids.clear();
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    //mutex to prevent racing conditions for samples coming from different threads
    __synchronized_obj(m_oCritSectionInputData);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_oInputWatchdog)
        {
            ProcessWatchdog(pMediaSample);
        }
        else if (pSource == &m_oInputEmergencyStop)
        {
            ProcessEmergencyStop(pMediaSample);
        }
        else if (pSource == &m_oInputAccelerate)
        {
            ProcessActuatorValue(pMediaSample, ID_ARD_ACT_SPEED_CONTR);
        }
        else if (pSource == &m_oInputSteering)
        {
            ProcessActuatorValue(pMediaSample, ID_ARD_ACT_STEER_SERVO);
        }
        else if (pSource == &m_oInputReverseLight)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_REVERSE);
        }
        else if (pSource == &m_oInputHeadlight)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_HEAD);
        }
        else if (pSource == &m_oInputTurnSignalLeft)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_TURNLEFT);
        }
        else if (pSource == &m_oInputTurnSignalRight)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_TURNRIGHT);
        }
        else if (pSource == &m_oInputHazardLights)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_HAZARD);
        }
        else if (pSource == &m_oInputBrakeLight)
        {
            ProcessLights(pMediaSample, ID_ARD_ACT_LIGHT_MASK_BRAKE);
        }
        else if (pSource == &m_oInputUssEnableFront)
        {
            ProcessUssControlFront(pMediaSample);
        }
        else if (pSource == &m_oInputUssEnableRear)
        {
            ProcessUssControlRear(pMediaSample);
        }
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::Cycle(__exception)
{
    std::vector<uint8_t> frame;
    for (size_t i = 0; i < m_valid_ids.size(); i++)
    {
        if (m_com[m_valid_ids[i]].get_next_frame(frame) == true)
        {
            tArduinoHeader header;
            memcpy(&header, frame.data(), sizeof(tArduinoHeader));
            tDataUnion data;
            memcpy(&data, frame.data() + sizeof(tArduinoHeader), header.ui8DataLength);
            int id = header.ui8ID;
            unsigned long timestamp = header.ui32Timestamp;

            switch (id)
            {
            case ID_ARD_SENSOR_INFO:
                LOG_INFO(cString::Format("Info frame received.\tID: %d\tSoftware version: %d", data.info.ui8ArduinoAddress, data.info.ui16ArduinoVersion));
                break;
            case ID_ARD_SENS_ERROR:
                break;
            case ID_ARD_SENS_US_FRONT_LEFT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_FRONT_CENTER_LEFT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_FRONT_CENTER:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_FRONT_RIGHT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;


            case ID_ARD_SENS_US_REAR_RIGHT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_REAR_CENTER_RIGHT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_REAR_CENTER:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_REAR_CENTER_LEFT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_REAR_LEFT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;

            case ID_ARD_SENS_US_SIDE_LEFT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;
            case ID_ARD_SENS_US_SIDE_RIGHT:
                TransmitUltrasonicData(id, static_cast<tFloat32>(data.us.i16Distance), timestamp);
                break;


            case ID_ARD_SENS_WHEEL_RIGHT:
                TransmitWheelData(id, data.wheel.ui32WheelTach, data.wheel.i8WheelDir, timestamp);
                break;
            case ID_ARD_SENS_WHEEL_LEFT:
                TransmitWheelData(id, data.wheel.ui32WheelTach, data.wheel.i8WheelDir, timestamp);
                break;


            case ID_ARD_SENS_IMU:
                TransmitInerMeasUnitData(timestamp,
                                         data.imu.f32ax, data.imu.f32ay, data.imu.f32az,
                                         data.imu.f32gx, data.imu.f32gy, data.imu.f32gz,
                                         data.imu.f32mx, data.imu.f32my, data.imu.f32mz,
                                         data.imu.f32roll, data.imu.f32pitch, data.imu.f32yaw);
                break;

            case ID_ARD_SENS_VOLT_ACTUATOR:
                m_voltageDataStruct.tActuatorVoltage.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tActuatorVoltage.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_ACTUATOR_CELL1:
                m_voltageDataStruct.tActuatorCell1.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tActuatorCell1.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_ACTUATOR_CELL2:
                m_voltageDataStruct.tActuatorCell2.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tActuatorCell2.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS:
                m_voltageDataStruct.tSensorVoltage.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorVoltage.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS_CELL1:
                m_voltageDataStruct.tSensorCell1.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorCell1.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS_CELL2:
                m_voltageDataStruct.tSensorCell2.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorCell2.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS_CELL3:
                m_voltageDataStruct.tSensorCell3.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorCell3.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS_CELL4:
                m_voltageDataStruct.tSensorCell4.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorCell4.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS_CELL5:
                m_voltageDataStruct.tSensorCell5.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorCell5.ui32ArduinoTimestamp = timestamp;
                break;
            case ID_ARD_SENS_VOLT_SENSORS_CELL6:
                m_voltageDataStruct.tSensorCell6.f32Value = static_cast<tFloat32>(data.voltage.ui16VoltageData);
                m_voltageDataStruct.tSensorCell6.ui32ArduinoTimestamp = timestamp;
                TransmitVoltStructData();
                break;
            default:
                break;
            }
        }
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue1 = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionLightsBoolSignalInput));
    cObjectPtr<IMediaType> pTypeBoolSignalValue2 = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUssEnableRearBoolSignalInput));
    cObjectPtr<IMediaType> pTypeBoolSignalValue3 = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUssEnableFrontBoolSignalInput));
    cObjectPtr<IMediaType> pTypeBoolSignalValue4 = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue4->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWatchdogBoolSignalInput));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionAccelerateSignalInput));
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSteeringSignalInput));

    tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
    RETURN_IF_POINTER_NULL(strDescEmergencyStop);
    cObjectPtr<IMediaType> pTypeEmergencyStop = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeEmergencyStop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));


    RETURN_IF_FAILED(m_oInputAccelerate.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputAccelerate));

    RETURN_IF_FAILED(m_oInputSteering.Create("SteeringController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSteering));

    RETURN_IF_FAILED(m_oInputHeadlight.Create("headLightEnabled", pTypeBoolSignalValue1, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputHeadlight));

    RETURN_IF_FAILED(m_oInputBrakeLight.Create("brakeLightEnabled", pTypeBoolSignalValue1, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputBrakeLight));

    RETURN_IF_FAILED(m_oInputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue1, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputReverseLight));

    RETURN_IF_FAILED(m_oInputTurnSignalLeft.Create("turnSignalLeftEnabled", pTypeBoolSignalValue1, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTurnSignalLeft));

    RETURN_IF_FAILED(m_oInputTurnSignalRight.Create("turnSignalRightEnabled", pTypeBoolSignalValue1, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTurnSignalRight));

    RETURN_IF_FAILED(m_oInputHazardLights.Create("hazardLightEnabled", pTypeBoolSignalValue1, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputHazardLights));

    RETURN_IF_FAILED(m_oInputUssEnableFront.Create("USSFrontEnabled", pTypeBoolSignalValue2, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputUssEnableFront));

    RETURN_IF_FAILED(m_oInputUssEnableRear.Create("USSRearEnabled", pTypeBoolSignalValue2, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputUssEnableRear));

    RETURN_IF_FAILED(m_oInputWatchdog.Create("WatchdogAlive", pTypeBoolSignalValue3, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputWatchdog));

    RETURN_IF_FAILED(m_oInputEmergencyStop.Create("EmergencyStop", pTypeEmergencyStop, this));
    RETURN_IF_FAILED(RegisterPin(&m_oInputEmergencyStop));

    RETURN_NOERROR;
}

tResult cArduinoCommunication::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    RETURN_IF_POINTER_NULL(strDescUsStruct);
    cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescVoltageStruct = pDescManager->GetMediaDescription("tVoltageStruct");
    RETURN_IF_POINTER_NULL(strDescVoltageStruct);
    cObjectPtr<IMediaType> pTypeVoltageStruct = new cMediaType(0, 0, 0, "tVoltageStruct", strDescVoltageStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
    RETURN_IF_POINTER_NULL(strDescWheelData);
    cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));
    RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsDataStruct));
    //RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltData));
    RETURN_IF_FAILED(pTypeVoltageStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltDataStruct));
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));
    RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelData));

    RETURN_IF_FAILED(m_oOutputUsFrontLeft.Create("UltrasonicFrontLeft", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontLeft));

    RETURN_IF_FAILED(m_oOutputUsFrontCenterLeft.Create("UltrasonicFrontCenterLeft", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontCenterLeft));

    RETURN_IF_FAILED(m_oOutputUsFrontCenter.Create("UltrasonicFrontCenter", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontCenter));

    RETURN_IF_FAILED(m_oOutputUsFrontCenterRight.Create("UltrasonicFrontCenterRight", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontCenterRight));

    RETURN_IF_FAILED(m_oOutputUsFrontRight.Create("UltrasonicFrontRight", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsFrontRight));

    RETURN_IF_FAILED(m_oOutputUsSideLeft.Create("UltrasonicSideLeft", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsSideLeft));

    RETURN_IF_FAILED(m_oOutputUsSideRight.Create("UltrasonicSideRight", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsSideRight));

    RETURN_IF_FAILED(m_oOutputUsRearCenterLeft.Create("UltrasonicRearCenterLeft", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsRearCenterLeft));

    RETURN_IF_FAILED(m_oOutputUsRearCenter.Create("UltrasonicRearCenter", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsRearCenter));

    RETURN_IF_FAILED(m_oOutputUsRearCenterRight.Create("UltrasonicRearCenterRight", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsRearCenterRight));

    RETURN_IF_FAILED(m_oOutputUsStruct.Create("UltrasonicStruct", pTypeUsStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputUsStruct));

    RETURN_IF_FAILED(m_oOutputInerMeasUnit.Create("InerMeasUnitStruct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputInerMeasUnit));

    RETURN_IF_FAILED(m_oOutputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputWheelLeft));

    RETURN_IF_FAILED(m_oOutputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputWheelRight));

    RETURN_IF_FAILED(m_oOutputVoltageStruct.Create("VoltageStruct", pTypeVoltageStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputVoltageStruct));

    RETURN_NOERROR;
}

tResult cArduinoCommunication::TransmitInerMeasUnitData(tUInt32 timestamp,
        tFloat32 ax, tFloat32 ay, tFloat32 az,
        tFloat32 gx, tFloat32 gy, tFloat32 gz,
        tFloat32 mx, tFloat32 my, tFloat32 mz,
        tFloat32 roll, tFloat32 pitch, tFloat32 yaw)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionInerMeasUnitData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    static bool hasID = false;
    static tBufferID szIDInerMeasUnitArduinoTimestamp;
    static tBufferID szIDInerMeasUnitF32A_x;
    static tBufferID szIDInerMeasUnitF32A_y;
    static tBufferID szIDInerMeasUnitF32A_z;

    static tBufferID szIDInerMeasUnitF32G_x;
    static tBufferID szIDInerMeasUnitF32G_y;
    static tBufferID szIDInerMeasUnitF32G_z;

    static tBufferID szIDInerMeasUnitF32M_x;
    static tBufferID szIDInerMeasUnitF32M_y;
    static tBufferID szIDInerMeasUnitF32M_z;

    static tBufferID szIDInerMeasUnitF32roll;
    static tBufferID szIDInerMeasUnitF32pitch;
    static tBufferID szIDInerMeasUnitF32yaw;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionInerMeasUnitData, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("ui32ArduinoTimestamp", szIDInerMeasUnitArduinoTimestamp);

            pCoder->GetID("f32A_x", szIDInerMeasUnitF32A_x);
            pCoder->GetID("f32A_y", szIDInerMeasUnitF32A_y);
            pCoder->GetID("f32A_z", szIDInerMeasUnitF32A_z);

            pCoder->GetID("f32G_x", szIDInerMeasUnitF32G_x);
            pCoder->GetID("f32G_y", szIDInerMeasUnitF32G_y);
            pCoder->GetID("f32G_z", szIDInerMeasUnitF32G_z);

            pCoder->GetID("f32M_x", szIDInerMeasUnitF32M_x);
            pCoder->GetID("f32M_y", szIDInerMeasUnitF32M_y);
            pCoder->GetID("f32M_z", szIDInerMeasUnitF32M_z);

            pCoder->GetID("f32roll", szIDInerMeasUnitF32roll);
            pCoder->GetID("f32pitch", szIDInerMeasUnitF32pitch);
            pCoder->GetID("f32yaw", szIDInerMeasUnitF32yaw);


            hasID = true;
        }

        pCoder->Set(szIDInerMeasUnitF32A_x, (tVoid*)&ax);
        pCoder->Set(szIDInerMeasUnitF32A_y, (tVoid*)&ay);
        pCoder->Set(szIDInerMeasUnitF32A_z, (tVoid*)&az);

        pCoder->Set(szIDInerMeasUnitF32G_x, (tVoid*)&gx);
        pCoder->Set(szIDInerMeasUnitF32G_y, (tVoid*)&gy);
        pCoder->Set(szIDInerMeasUnitF32G_z, (tVoid*)&gz);

        pCoder->Set(szIDInerMeasUnitF32M_x, (tVoid*)&mx);
        pCoder->Set(szIDInerMeasUnitF32M_y, (tVoid*)&my);
        pCoder->Set(szIDInerMeasUnitF32M_z, (tVoid*)&mz);

        pCoder->Set(szIDInerMeasUnitF32roll, (tVoid*)&roll);
        pCoder->Set(szIDInerMeasUnitF32pitch, (tVoid*)&pitch);
        pCoder->Set(szIDInerMeasUnitF32yaw, (tVoid*)&yaw);

        pCoder->Set(szIDInerMeasUnitArduinoTimestamp, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputInerMeasUnit.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult cArduinoCommunication::TransmitUltrasonicData(tUInt8 sensorID, tFloat32 f32Distance, tUInt32 ui32ArduinoTimestamp)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionUsData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    static bool hasID = false;
    static tBufferID szIDUltrasonicF32Value;
    static tBufferID szIDUltrasonicArduinoTimestamp;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionUsData, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("f32Value", szIDUltrasonicF32Value);
            pCoder->GetID("ui32ArduinoTimestamp", szIDUltrasonicArduinoTimestamp);
            hasID = true;
        }

        pCoder->Set(szIDUltrasonicArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);
        pCoder->Set(szIDUltrasonicF32Value, (tVoid*)&f32Distance);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    switch (sensorID)
    {
    case ID_ARD_SENS_US_FRONT_LEFT:
        m_ultrasonicDataStruct.tFrontLeft.f32Value = f32Distance;
        m_ultrasonicDataStruct.tFrontLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsFrontLeft.Transmit(pMediaSample));
        RETURN_IF_FAILED(TransmitUltrasonicStructData());
        break;
    case ID_ARD_SENS_US_FRONT_CENTER_LEFT:
        m_ultrasonicDataStruct.tFrontCenterLeft.f32Value = f32Distance;
        m_ultrasonicDataStruct.tFrontCenterLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsFrontCenterLeft.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_FRONT_CENTER:
        m_ultrasonicDataStruct.tFrontCenter.f32Value = f32Distance;
        m_ultrasonicDataStruct.tFrontCenter.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsFrontCenter.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_FRONT_CENTER_RIGHT:
        m_ultrasonicDataStruct.tFrontCenterRight.f32Value = f32Distance;
        m_ultrasonicDataStruct.tFrontCenterRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsFrontCenterRight.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_FRONT_RIGHT:
        m_ultrasonicDataStruct.tFrontRight.f32Value = f32Distance;
        m_ultrasonicDataStruct.tFrontRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsFrontRight.Transmit(pMediaSample));
        break;


    case ID_ARD_SENS_US_REAR_RIGHT:
        m_ultrasonicDataStruct.tRearRight.f32Value = f32Distance;
        m_ultrasonicDataStruct.tRearRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        //RETURN_IF_FAILED(m_oOutputUsRearCenterRight.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_REAR_CENTER_RIGHT:
        m_ultrasonicDataStruct.tRearRight.f32Value = f32Distance;
        m_ultrasonicDataStruct.tRearRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsRearCenterRight.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_REAR_CENTER:
        m_ultrasonicDataStruct.tRearCenter.f32Value = f32Distance;
        m_ultrasonicDataStruct.tRearCenter.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsRearCenter.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_REAR_CENTER_LEFT:
        m_ultrasonicDataStruct.tRearLeft.f32Value = f32Distance;
        m_ultrasonicDataStruct.tRearLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsRearCenterLeft.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_REAR_LEFT:
        m_ultrasonicDataStruct.tRearLeft.f32Value = f32Distance;
        m_ultrasonicDataStruct.tRearLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        //RETURN_IF_FAILED(m_oOutputUsRearCenterLeft.Transmit(pMediaSample));
        break;

    case ID_ARD_SENS_US_SIDE_LEFT:
        m_ultrasonicDataStruct.tSideLeft.f32Value = f32Distance;
        m_ultrasonicDataStruct.tSideLeft.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsSideLeft.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_US_SIDE_RIGHT:
        m_ultrasonicDataStruct.tSideRight.f32Value = f32Distance;
        m_ultrasonicDataStruct.tSideRight.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        RETURN_IF_FAILED(m_oOutputUsSideRight.Transmit(pMediaSample));
        break;
    default:
        break;
    }

    RETURN_NOERROR;
}
tResult cArduinoCommunication::TransmitUltrasonicStructData()
{
    // create new pointer for media sample
    cObjectPtr<IMediaSample> pMediaSample = new cMediaSample();

    // update media sample with whole struct and current time
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), &m_ultrasonicDataStruct, sizeof(m_ultrasonicDataStruct), 0));

    // transmit media sample
    RETURN_IF_FAILED(m_oOutputUsStruct.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult cArduinoCommunication::TransmitWheelData(tUInt8 sensorID, tUInt32 wheelTach, tUInt8 wheelDir, tUInt32 ui32ArduinoTimestamp)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionWheelData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    static bool hasID = false;
    static tBufferID szIDWheelDataI8WheelDir;
    static tBufferID szIDWheelDataUi32WheelTach;
    static tBufferID szIDWheelDataArduinoTimestamp;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionWheelData, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("i8WheelDir", szIDWheelDataI8WheelDir);
            pCoder->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
            pCoder->GetID("ui32ArduinoTimestamp", szIDWheelDataArduinoTimestamp);
            hasID = true;
        }

        pCoder->Set(szIDWheelDataI8WheelDir, (tVoid*)&(wheelDir));
        pCoder->Set(szIDWheelDataUi32WheelTach, (tVoid*)&(wheelTach));
        pCoder->Set(szIDWheelDataArduinoTimestamp, (tVoid*)&(ui32ArduinoTimestamp));
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    switch (sensorID)
    {
    case ID_ARD_SENS_WHEEL_LEFT:
        RETURN_IF_FAILED(m_oOutputWheelLeft.Transmit(pMediaSample));
        break;
    case ID_ARD_SENS_WHEEL_RIGHT:
        RETURN_IF_FAILED(m_oOutputWheelRight.Transmit(pMediaSample));
        break;
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::TransmitVoltStructData()
{
    // create new pointer for media sample
    cObjectPtr<IMediaSample> pMediaSample = new cMediaSample();

    // update media sample with whole struct and current time
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), &m_voltageDataStruct, sizeof(m_voltageDataStruct), 0));

    // transmit media sample
    RETURN_IF_FAILED(m_oOutputVoltageStruct.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult cArduinoCommunication::ProcessEmergencyStop(IMediaSample* pMediaSample)
{
    if (pMediaSample != NULL && m_pDescriptionEmergencyStop != NULL)
    {
        tBool bValue = tFalse;
        static bool hasID = false;
        static tBufferID szIDEmergencyStopBEmergencyStop;

        {
            __adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyStop, pMediaSample, pCoder);

            if (!hasID)
            {
                pCoder->GetID("bEmergencyStop", szIDEmergencyStopBEmergencyStop);
                hasID = true;
            }

            pCoder->Get(szIDEmergencyStopBEmergencyStop, (tVoid*)&bValue);
        }

        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            if (m_com[i].get_id() == ARDUINO_CENTER_ACTUATORS)
            {
                m_com[i].send_emergency_stop();
                break;
            }
        }
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::ProcessWatchdog(IMediaSample* pMediaSample)
{
    tBool bValue = tFalse;
    static bool hasID = false;
    static tBufferID szIDBoolSignal;
    static tBufferID szIDArduinoTimestamp;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWatchdogBoolSignalInput, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("bValue", szIDBoolSignal);
            pCoder->GetID("ui32ArduinoTimestamp", szIDArduinoTimestamp);
            hasID = true;
        }

        pCoder->Get(szIDBoolSignal, (tVoid*)&bValue);
    }

    if (bValue == tTrue)
    {
        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            if (m_com[i].get_id() == ARDUINO_CENTER_ACTUATORS)
            {
                m_com[i].send_watchdog_trigger();
                break;
            }
        }
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::ProcessActuatorValue(IMediaSample* pMediaSample, tUInt8 ui8ChID)
{
    tFloat32 f32value = 0;

    static bool hasID_SteerServo = false;
    static tBufferID szIDF32Value_SteerServo;

    static bool hasID_SpeedContr = false;
    static tBufferID szIDF32Value_SpeedContr;


    switch (ui8ChID)
    {
    case ID_ARD_ACT_STEER_SERVO:
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSteeringSignalInput, pMediaSample, pCoderInput);

        if (!hasID_SteerServo)
        {
            pCoderInput->GetID("f32Value", szIDF32Value_SteerServo);
            hasID_SteerServo = true;
        }

        pCoderInput->Get(szIDF32Value_SteerServo, (tVoid*)&f32value);

    }
    for (size_t i = 0; i < m_valid_ids.size(); i++)
    {
        if (m_com[i].get_id() == ARDUINO_CENTER_ACTUATORS)
        {
            m_com[i].send_steering(f32value);
            break;
        }
    }

    break;
    case ID_ARD_ACT_SPEED_CONTR:
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionAccelerateSignalInput, pMediaSample, pCoderInput);

        if (!hasID_SpeedContr)
        {
            pCoderInput->GetID("f32Value", szIDF32Value_SpeedContr);
            hasID_SpeedContr = true;
        }

        pCoderInput->Get(szIDF32Value_SpeedContr, (tVoid*)&f32value);

    }
    for (size_t i = 0; i < m_valid_ids.size(); i++)
    {
        if (m_com[i].get_id() == ARDUINO_CENTER_ACTUATORS)
        {
            m_com[i].send_speed(f32value);
            break;
        }
    }

    break;
    }


    RETURN_NOERROR;
}

tResult cArduinoCommunication::ProcessLights(IMediaSample* pMediaSample, tUInt8 ui8LightID)
{
    tBool bValue = tFalse;

    static tUInt8 ui8lightMask = 0;

    static bool hasID = false;
    static tBufferID szIDBoolSignalInputF32Value;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionLightsBoolSignalInput, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("bValue", szIDBoolSignalInputF32Value);
            hasID = true;
        }

        pCoder->Get(szIDBoolSignalInputF32Value, (tVoid*)&bValue);
    }

    //Special case: If turn signal switch from left to right. Disable other turn signal.
    if (ui8LightID == ID_ARD_ACT_LIGHT_MASK_TURNLEFT)
    {
        ui8lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNRIGHT;
    }

    if (ui8LightID == ID_ARD_ACT_LIGHT_MASK_TURNRIGHT)
    {
        ui8lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNLEFT;
    }

    // set enable or disable in frame
    if (bValue == tTrue)
    {
        ui8lightMask |= ui8LightID;
        //LOG_INFO(cString::Format("Actuatorfilter Light: ID %d switched on",ui8LightID));
    }
    else
    {
        ui8lightMask &= ~ui8LightID;
        //LOG_INFO(cString::Format("Actuatorfilter Light: ID %d switched off",ui8LightID));
    }

    for (size_t i = 0; i < m_valid_ids.size(); i++)
    {
        if (m_com[i].get_id() == ARDUINO_CENTER_ACTUATORS)
        {
            m_com[i].send_light(ui8lightMask);
            break;
        }
    }

    RETURN_NOERROR;
}

tResult cArduinoCommunication::ProcessUssControlFront(IMediaSample* pMediaSample)
{
    tBool bValue = tFalse;

    static bool hasID = false;
    static tBufferID szIDBoolSignalInputF32Value;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionUssEnableFrontBoolSignalInput, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("bValue", szIDBoolSignalInputF32Value);
            hasID = true;
        }

        pCoder->Get(szIDBoolSignalInputF32Value, (tVoid*)&bValue);
    }

    if (bValue)
    {
        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            if (m_com[i].get_id() == ARDUINO_FRONT_US)
            {
                m_com[i].send_enable_uss();
                break;
            }
        }
        LOG_INFO(cString::Format("Enabling US sensors front."));
    }
    else
    {
        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            if (m_com[i].get_id() == ARDUINO_FRONT_US)
            {
                m_com[i].send_disable_uss();
                break;
            }
        }
        LOG_INFO(cString::Format("Disabling US sensors front."));

    }
    RETURN_NOERROR;
}

tResult cArduinoCommunication::ProcessUssControlRear(IMediaSample* pMediaSample)
{
    tBool bValue = tFalse;

    static bool hasID = false;
    static tBufferID szIDBoolSignalInputF32Value;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionUssEnableRearBoolSignalInput, pMediaSample, pCoder);
        if (!hasID)
        {
            pCoder->GetID("bValue", szIDBoolSignalInputF32Value);
            hasID = true;
        }

        pCoder->Get(szIDBoolSignalInputF32Value, (tVoid*)&bValue);
    }

    if (bValue)
    {
        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            if (m_com[i].get_id() == ARDUINO_REAR_US)
            {
                m_com[i].send_enable_uss();
                break;
            }
        }
        LOG_INFO(cString::Format("Enabling US sensors rear."));
    }
    else
    {
        for (size_t i = 0; i < m_valid_ids.size(); i++)
        {
            if (m_com[i].get_id() == ARDUINO_REAR_US)
            {
                m_com[i].send_disable_uss();
                break;
            }
        }
        LOG_INFO(cString::Format("Disabling US sensors rear."));
    }
    RETURN_NOERROR;
}
