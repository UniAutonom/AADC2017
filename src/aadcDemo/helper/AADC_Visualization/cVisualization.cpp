/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-04-26 13:55:11#$ $Rev:: 62570   $
**********************************************************************/

#include "stdafx.h"
#include "cVisualization.h"

#define FACTORRADTODEG 180.f/M_PI

ADTF_FILTER_PLUGIN("AADC Visualization", OID_ADTF_VISUALIZATION, cVisualization);


cVisualization::cVisualization(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info)
{
}

cVisualization::~cVisualization()
{
}

tHandle cVisualization::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
        m_pWidget = new DisplayWidget(pWidget);
    
    // make the qt connections
    connect(this, SIGNAL(SendUltrasonicData(float,int)), m_pWidget, SLOT(OnUltrasonicData(float,int)));
    connect(this, SIGNAL(SendVoltageData(float, int)), m_pWidget, SLOT(OnVoltageData(float, int)));
    connect(this, SIGNAL(SendWheelData(ulong , char, int)), m_pWidget, SLOT(OnWheelData(ulong , char, int)));
    connect(this, SIGNAL(SendInerMeasUnitData(float, float, float, float,float,float)), m_pWidget, SLOT(OnInerMeasUnitData(float, float, float, float,float,float)));    
    
    return (tHandle)m_pWidget;
}

tResult cVisualization::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cVisualization::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {

//get the description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

//get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
//get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    	

//get description for inertial measurement sensor data pin
        tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");	
        RETURN_IF_POINTER_NULL(strDescInerMeasUnit);	

        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    	
        
//get description for wheel sensors data pins
        tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");	
        RETURN_IF_POINTER_NULL(strDescWheelData);
//get mediatype for wheeldata sensor data pins
        cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    	
        
//get mediatype description for ultrasonic sensor data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData));

//get mediatype description for voltage sensor data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionVoltData));
        
//get mediatype description for inertial measurement unit sensor data type
        RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

//get mediatype description for wheel sensor data type
        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelData));
        
//create pins for ultrasonic sensor data
        RETURN_IF_FAILED(m_oInputUsFrontLeft.Create("Ultrasonic_Front_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsFrontLeft));
        
        RETURN_IF_FAILED(m_oInputUsFrontCenterLeft.Create("Ultrasonic_Front_Center_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsFrontCenterLeft));
        
        RETURN_IF_FAILED(m_oInputUsFrontCenter.Create("Ultrasonic_Front_Center", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsFrontCenter));
        
        RETURN_IF_FAILED(m_oInputUsFrontCenterRight.Create("Ultrasonic_Front_Center_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsFrontCenterRight));
        
        RETURN_IF_FAILED(m_oInputUsFrontRight.Create("Ultrasonic_Front_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsFrontRight));
        
        RETURN_IF_FAILED(m_oInputUsSideLeft.Create("Ultrasonic_Side_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsSideLeft));
        
        RETURN_IF_FAILED(m_oInputUsSideRight.Create("Ultrasonic_Side_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsSideRight));
        
        RETURN_IF_FAILED(m_oInputUsRearLeft.Create("Ultrasonic_Rear_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsRearLeft));
        
        RETURN_IF_FAILED(m_oInputUsRearCenter.Create("Ultrasonic_Rear_Center", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsRearCenter));
        
        RETURN_IF_FAILED(m_oInputUsRearRight.Create("Ultrasonic_Rear_Right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsRearRight));

//create pins for voltage sensor data
        RETURN_IF_FAILED(m_oInputVoltMeas.Create("Voltage_Measurement", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputVoltMeas));
        
        RETURN_IF_FAILED(m_oInputVoltSpeedCtr.Create("Voltage_SpeedCntrl", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputVoltSpeedCtr));

//create pin for inertial measurement unit data
        RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnit_Struct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

//create pin for wheel left sensor data
        RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeft_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

//create pin for wheel right data
        RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRight_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

    }
    else if (eStage == StageGraphReady)
    {
       // the ids for the items in the structs of the mediasamples still have to be set
       m_bIDsInerMeasUnitSet = tFalse;
       m_bIDsVoltageSet = tFalse;
       m_bIDsUltrasonicSet = tFalse;
       m_bIDsWheelDataSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cVisualization::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));       
    RETURN_NOERROR;
}

tResult cVisualization::Stop(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));
    RETURN_NOERROR;
}

tResult cVisualization::Shutdown(tInitStage eStage, __exception)
{ 
   return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult cVisualization::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {            
        // transmit sample to specific processing function
        if (pSource == &m_oInputUsFrontLeft)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usFrontLeft));
        }
        else if (pSource == &m_oInputUsFrontCenterLeft)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usFrontCenterLeft));
        }
        else if (pSource == &m_oInputUsFrontCenter)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usFrontCenter));
        }
        else if (pSource == &m_oInputUsFrontCenterRight)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usFrontCenterRight));
        }
        else if (pSource == &m_oInputUsFrontRight)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usFrontRight));
        }
        else if (pSource == &m_oInputUsSideLeft)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usSideLeft));
        }
        else if (pSource == &m_oInputUsSideRight)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usSideRight));
        }
        else if (pSource == &m_oInputUsRearLeft)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usRearLeft));
        }
        else if (pSource == &m_oInputUsRearCenter)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usRearCenter));
        }
        else if (pSource == &m_oInputUsRearRight)
        {
            RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usRearRight));
        }
        else if (pSource == &m_oInputVoltMeas)
        {
            RETURN_IF_FAILED(ProcessVoltageSample(pMediaSample, SensorDefinition::measurementCircuit));
        }
        else if (pSource == &m_oInputVoltSpeedCtr)
        {
            RETURN_IF_FAILED(ProcessVoltageSample(pMediaSample, SensorDefinition::speedCntrlCircuit));
        }
        else if (pSource == &m_oInputInerMeasUnit)
        {
            RETURN_IF_FAILED(ProcessInerMeasUnitSample(pMediaSample));
        }
        else if (pSource == &m_oInputWheelLeft)
        {
            RETURN_IF_FAILED(ProcessWheelSample(pMediaSample, SensorDefinition::wheelLeft));
        }
        else if (pSource == &m_oInputWheelRight)
        {
            RETURN_IF_FAILED(ProcessWheelSample(pMediaSample, SensorDefinition::wheelRight));
        }
    }
    RETURN_NOERROR;
}

tResult cVisualization::ProcessUltrasonicSample(IMediaSample* pMediaSample, SensorDefinition::ultrasonicSensor usSensor)
{
    __synchronized_obj(m_oProcessUsDataCritSection);
    //write values with zero
    tFloat32 f32value = 0;
    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionUsData,pMediaSample,pCoderInput);
                   
        // get the IDs for the items in the media sample 
        if(!m_bIDsUltrasonicSet)
        {
            pCoderInput->GetID("f32Value", m_szIDUltrasonicF32Value);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDUltrasonicArduinoTimestamp);
            m_bIDsUltrasonicSet = tTrue;
        }     

        //get values from media sample        
        pCoderInput->Get(m_szIDUltrasonicF32Value, (tVoid*)&f32value);

        //emit signal to gui
        //convert from meters to centimeter 
        emit SendUltrasonicData(static_cast<float>(f32value*100.f),usSensor);
    }
    RETURN_NOERROR;
}

tResult cVisualization::ProcessVoltageSample(IMediaSample* pMediaSample, SensorDefinition::voltageSensor voltageSensor)
{
    __synchronized_obj(m_oProcessVoltageDataCritSection);
    //write values with zero
    tFloat32 f32value = 0;
    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionVoltData,pMediaSample,pCoderInput);
        
        // get the IDs for the items in the media sample        
        if(!m_bIDsVoltageSet)
        {
            pCoderInput->GetID("f32Value", m_szIDVoltageF32Value);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDVoltageArduinoTimestamp);
            m_bIDsVoltageSet = tTrue;
        }     

        //get values from media sample        
        pCoderInput->Get(m_szIDVoltageF32Value, (tVoid*)&f32value);

        //emit signal to gui
        emit SendVoltageData(static_cast<float>(f32value),voltageSensor);
        
    }
    RETURN_NOERROR;
}

tResult cVisualization::ProcessInerMeasUnitSample(IMediaSample* pMediaSample)
{
    //write values with zero
    tFloat32 f32Q_w = 0;
    tFloat32 f32Q_x = 0;
    tFloat32 f32Q_y = 0;
    tFloat32 f32Q_z = 0;
    tFloat32 f32A_x = 0;
    tFloat32 f32A_y = 0;
    tFloat32 f32A_z = 0;
    tFloat32 f32Roll = 0;
    tFloat32 f32Pitch = 0;
    tFloat32 f32Yaw = 0;
    
    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData,pMediaSample,pCoderInput);
               
        // get the IDs for the items in the media sample 
        if(!m_bIDsInerMeasUnitSet)
        {
            pCoderInput->GetID("f32Q_w", m_szIDInerMeasUnitF32Q_w);
            pCoderInput->GetID("f32Q_x", m_szIDInerMeasUnitF32Q_x);
            pCoderInput->GetID("f32Q_y", m_szIDInerMeasUnitF32Q_y);
            pCoderInput->GetID("f32Q_z", m_szIDInerMeasUnitF32Q_z);
            pCoderInput->GetID("f32A_x", m_szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", m_szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", m_szIDInerMeasUnitF32A_z);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDInerMeasUnitArduinoTimestamp);
            m_bIDsInerMeasUnitSet = tTrue;
        }

        //get values from media sample        
        pCoderInput->Get(m_szIDInerMeasUnitF32Q_w, (tVoid*)&f32Q_w);
        pCoderInput->Get(m_szIDInerMeasUnitF32Q_x, (tVoid*)&f32Q_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32Q_y, (tVoid*)&f32Q_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32Q_z, (tVoid*)&f32Q_z);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
        pCoderInput->Get(m_szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);
        
        //calculate the euler values
        calulateEulerAngles(f32Q_w,f32Q_x,f32Q_y,f32Q_z,f32Yaw, f32Pitch,f32Roll);
        
        //emit signal to gui
        // multiplicate with 180/pi (*180/M_PI) to get angle in degree
        emit SendInerMeasUnitData(static_cast<float>(f32A_x), static_cast<float>(f32A_y), static_cast<float>(f32A_z), static_cast<float>(f32Yaw*FACTORRADTODEG), static_cast<float>(f32Pitch*FACTORRADTODEG), static_cast<float>(f32Roll*FACTORRADTODEG));

    }
    RETURN_NOERROR;
}


tResult cVisualization::ProcessWheelSample(IMediaSample* pMediaSample, SensorDefinition::wheelSensor wheelSensorType)
{
    __synchronized_obj(m_oProcessWheelDataCritSection);
    //write values with zero
    tUInt32 ui32Tach = 0;
    tInt8 i8Direction = 0;
    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelData,pMediaSample,pCoderInput);
        
        // get the IDs for the items in the media sample 
        if(!m_bIDsWheelDataSet)
        {
            pCoderInput->GetID("i8WheelDir", m_szIDWheelDataI8WheelDir);
            pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataArduinoTimestamp);
            m_bIDsWheelDataSet = tTrue;
        }        
        
        //get values from media sample        
        pCoderInput->Get(m_szIDWheelDataUi32WheelTach, (tVoid*)&ui32Tach);        
        pCoderInput->Get(m_szIDWheelDataI8WheelDir, (tVoid*)&i8Direction);

        //emit signal to gui
        switch (wheelSensorType)
        {
            case SensorDefinition::wheelLeft:
                emit SendWheelData(static_cast<float>(ui32Tach),static_cast<float>(i8Direction),SensorDefinition::wheelLeft);
                break;
            case SensorDefinition::wheelRight:
                emit SendWheelData(static_cast<float>(ui32Tach),static_cast<float>(i8Direction),SensorDefinition::wheelRight);
                break;
        }
    }
    RETURN_NOERROR;
}

tResult cVisualization::calulateEulerAngles(tFloat32 f32QW, tFloat32 f32Qx, tFloat32 f32Qy, tFloat32 f32Qz, tFloat32 &f32Yaw_out, tFloat32 &f32Pitch_out, tFloat32 &f32Roll_out)
{

/*! total of 24 different euler systems. We support only one, yet. */
/*! For details please see "Graphic Germs IV", Ken Shoemake, 1993 */
    f32Roll_out = atan2(2*f32Qy*f32QW - 2*f32Qx*f32Qz, 1 - 2*f32Qy*f32Qy - 2*f32Qz*f32Qz);
    f32Pitch_out = atan2(2*f32Qx*f32QW - 2*f32Qy*f32Qz, 1 - 2*f32Qx*f32Qx - 2*f32Qz*f32Qz);
    f32Yaw_out = asin(2*f32Qx*f32Qy + 2*f32Qz*f32QW); 

    /*
    f32Roll_out = atan2(2*qW*f32Qx + 2*f32Qy*f32Qz, 1 - 2*f32Qx*f32Qx - 2*f32Qy*f32Qy);
    f32Yaw_out = atan2(2*qW*f32Qz + 2*f32Qx*f32Qy, 1 - 2*f32Qy*f32Qy - 2*f32Qz*f32Qz);
    f32Pitch_out = asin(2*qW*f32Qy - 2*f32Qz*f32Qx); 
    */
        
    RETURN_NOERROR;
}