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
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cEmergencyBreak.h"
/// Create filter shell
ADTF_FILTER_PLUGIN("EmergencyBreak", OID_ADTF_TEMPLATE_FILTER, cEmergencyBreak);



cEmergencyBreak::cEmergencyBreak(const tChar* __info):cFilter(__info)
{

}

cEmergencyBreak::~cEmergencyBreak()
{

}

tResult cEmergencyBreak::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        //init mediatype for signal values
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //init mediatype for US struct pins
        tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
        RETURN_IF_POINTER_NULL(strDescUsStruct);
        cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //init mediatype for EmergencyBrakeStatus pins
        tChar const * strDescEmergencyStatus = pDescManager->GetMediaDescription("tJuryEmergencyStop");
        RETURN_IF_POINTER_NULL(strDescEmergencyStatus);
        cObjectPtr<IMediaType> pTypeEmergencyStatusStruct = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStatus, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //init mediatype for EmergencyBreakSet pins
        tChar const * strDescEmergencyBreakSet = pDescManager->GetMediaDescription("tEmergencyBreakSet");
        RETURN_IF_POINTER_NULL(strDescEmergencyBreakSet);
        cObjectPtr<IMediaType> pTypeEmergencyBreakSetStruct = new cMediaType(0, 0, 0, "tEmergencyBreakSet", strDescEmergencyBreakSet, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //get mediatype description interfaces
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
        RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
        RETURN_IF_FAILED(pTypeEmergencyStatusStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStatus));
        RETURN_IF_FAILED(pTypeEmergencyBreakSetStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyBreakSet));

        //create and register pins
        RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedcontrollerIn", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedcontrollerOut", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

        RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));

        RETURN_IF_FAILED(m_oOutputEmergencyStatus.Create("EmergencyBreakStatus", pTypeEmergencyStatusStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputEmergencyStatus));

        RETURN_IF_FAILED(m_oInputEmergencyBreakSet.Create("EmergencyBreakSet", pTypeEmergencyBreakSetStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputEmergencyBreakSet));  
    }


    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.

        m_szIdsUsStructSet          = tFalse;
        m_szIdInputSpeedSet         = tFalse;
        m_szIdOutputSpeedSet        = tFalse;
        m_szIdEmergencyStatusSet    = tFalse;
        m_szIdEmergencyBreakSet     = tFalse;

        for(int i = 0; i < SENSOR_COUNT; i++){
            m_critDistances[i]   = -1;
            m_actualDistances[i] = -1;
        }

        m_lastTriggeredTime = 0;
        m_breakActive       = false;
        speed = 0;
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cEmergencyBreak::Stop(__exception)
{
    // call the base class implementatistrDescSignalValueon
    return cFilter::Stop(__exception_ptr);
}

tResult cEmergencyBreak::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you inm_bIdsUsStructSetitiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementatistrDescSignalValueon
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cEmergencyBreak::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        //check source of MediaSample
        if(pSource == &m_oInputSpeedController)
        {
            //if mediasample is from type speedcontroller check minimumvalues and possibly stop car
            ProcessSpeedController(pMediaSample);
        }
        else if (pSource == &m_oInputUsStruct)
        {
            //if mediasample is from type ultrasonic detect the minimum ultrasonic value
            ProcessMinimumValueUs(pMediaSample);
        }
        else if (pSource == &m_oInputEmergencyBreakSet && m_pDescriptionEmergencyBreakSet != NULL)
        {
            setCriticalDistances(pMediaSample);
        }
    }

    RETURN_NOERROR;
}

tResult cEmergencyBreak::ProcessMinimumValueUs(IMediaSample* pMediaSample)
{
    //use mutex
    __synchronized_obj(m_critSecMinimumUsValue);

    //read lock
    __adtf_sample_read_lock_mediadescription(m_pDescriptionUsStruct, pMediaSample, pCoderInput);

    //Set all Ids
    if(!m_szIdsUsStructSet)
    {
        tBufferID idValue, idTimestamp;
        m_szIdUsStructValues.clear();
        m_szIdUsStructTimeStamps.clear();

        pCoderInput->GetID("tFrontLeft.f32Value", idValue);
        pCoderInput->GetID("tFrontLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tFrontCenterLeft.f32Value", idValue);
        pCoderInput->GetID("tFrontCenterLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tFrontCenter.f32Value", idValue);
        pCoderInput->GetID("tFrontCenter.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tFrontCenterRight.f32Value", idValue);
        pCoderInput->GetID("tFrontCenterRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tFrontRight.f32Value", idValue);
        pCoderInput->GetID("tFrontRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tSideLeft.f32Value", idValue);
        pCoderInput->GetID("tSideLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tSideRight.f32Value", idValue);
        pCoderInput->GetID("tSideRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tRearLeft.f32Value", idValue);
        pCoderInput->GetID("tRearLeft.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tRearCenter.f32Value", idValue);
        pCoderInput->GetID("tRearCenter.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        pCoderInput->GetID("tRearRight.f32Value", idValue);
        pCoderInput->GetID("tRearRight.ui32ArduinoTimestamp", idTimestamp);
        m_szIdUsStructValues.push_back(idValue);
        m_szIdUsStructTimeStamps.push_back(idTimestamp);

        m_szIdsUsStructSet = tTrue;
    }

    // ignore inital noisy values!
    static int counter = 0;
    if(counter < 50){
        counter++;
        RETURN_NOERROR;
    }

    //iterate through all values
    tFloat32 buf_UsSignal;
    for(int i=0; i<(int)m_szIdUsStructValues.size(); ++i)
    {
        pCoderInput->Get(m_szIdUsStructValues[i], (tVoid*)&buf_UsSignal);
        //save values
        m_actualDistances[i] = buf_UsSignal;
    }

    tBool    emergencyStop = false;
    tInt64   actualTime    = cSystem::GetTime();
    for(int i=0; i < SENSOR_COUNT; ++i) // go trough all sensors and check if there is something in critical distance
    {
        if((m_critDistances[i] != -1) && (m_actualDistances[i] < m_critDistances[i]) && (m_actualDistances[i] != -1)) // there is something in critical distance of the i.th sensor
        {
            emergencyStop = true;
            m_lastTriggeredTime = actualTime;
            //LOG_INFO(cString::Format("sensor %d triggered, dis %f!", i, m_actualDistances[i]));
        }
    }

    if(((actualTime - m_lastTriggeredTime) < MIN_BREAK_DURATION))  // at least one sensor had detected something in critical distance in the last MIN_BREAK_DURATION
    {
        emergencyStop = true; // do not release the emergency break (yet) !
        //LOG_INFO(cString::Format("do not yet release emrgency break!"));
    }

    if(emergencyStop)
    {
        ActivateEmergencyBreak();
    }
    else // !emergencyStop
    {
        DeactivateEmergencyBreak();
    }

    RETURN_NOERROR;
}

tResult cEmergencyBreak::ProcessSpeedController(IMediaSample* pMediaSample)
{
    //read-out the incoming Media Sample

    tUInt32 timestamp;

    //sobald der block verlassen wird, wird das lock aufgehoben
    {
        //read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdInputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdInputspeedControllerValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputspeedControllerTimeStamp);
            m_szIdInputSpeedSet = tTrue;
        }

        pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speed);
        pCoderInput->Get(m_szIdInputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    if(m_breakActive)
    {
        TransmitSpeed(0, timestamp);
    }
    else // !m_breakActive
    {
        TransmitSpeed(speed, timestamp);
//        LOG_INFO(cString::Format("transmit speed %f", speed));
    }

    RETURN_NOERROR;
}

void cEmergencyBreak::ActivateEmergencyBreak()
{
    if(!m_breakActive)
        LOG_INFO(cString::Format("EMERGENCY BREAK ACTIVATED!"));

    TransmitEmergencyStatus(true);
    m_breakActive = true;

    TransmitSpeed(0, 0); // stop the car immediately!
}

void cEmergencyBreak::DeactivateEmergencyBreak()
{
    if(m_breakActive)
        LOG_INFO(cString::Format("EMERGENCY BREAK RELEASED!"));

    TransmitEmergencyStatus(false);
    m_breakActive = false;

    TransmitSpeed(speed, 0);
}

void cEmergencyBreak::setCriticalDistances(IMediaSample* pMediaSample)
{
    // focus for sample read lock
    __adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyBreakSet,pMediaSample,pCoder);
    // get the IDs for the items in the media sample
    if(!m_szIdEmergencyBreakSet)
    {
        pCoder->GetID("i16EmergencyBreakFrontLeft",        m_szIdEmergencyBreakSetFrontLeftValue);
        pCoder->GetID("i16EmergencyBreakFrontCenterLeft",  m_szIdEmergencyBreakSetFrontCenterLeftValue);
        pCoder->GetID("i16EmergencyBreakFrontMiddle",      m_szIdEmergencyBreakSetFrontMiddleValue);
        pCoder->GetID("i16EmergencyBreakFrontCenterRight", m_szIdEmergencyBreakSetFrontCenterRightValue);
        pCoder->GetID("i16EmergencyBreakFrontRight",       m_szIdEmergencyBreakSetFrontRightValue);
        pCoder->GetID("i16EmergencyBreakSideLeft",         m_szIdEmergencyBreakSetSideLeftValue);
        pCoder->GetID("i16EmergencyBreakSideRight",        m_szIdEmergencyBreakSetSideRightValue);
        pCoder->GetID("i16EmergencyBreakBackLeft",         m_szIdEmergencyBreakSetBackLeftValue);
        pCoder->GetID("i16EmergencyBreakBackMiddle",       m_szIdEmergencyBreakSetBackMiddleValue);
        pCoder->GetID("i16EmergencyBreakBackRight",        m_szIdEmergencyBreakSetBackRightValue);
        m_szIdEmergencyBreakSet = tTrue;
    }

    pCoder->Get(m_szIdEmergencyBreakSetFrontLeftValue,        (tVoid*)&m_critDistances[0]);
    pCoder->Get(m_szIdEmergencyBreakSetFrontCenterLeftValue,  (tVoid*)&m_critDistances[1]);
    pCoder->Get(m_szIdEmergencyBreakSetFrontMiddleValue,      (tVoid*)&m_critDistances[2]);
    pCoder->Get(m_szIdEmergencyBreakSetFrontCenterRightValue, (tVoid*)&m_critDistances[3]);
    pCoder->Get(m_szIdEmergencyBreakSetFrontRightValue,       (tVoid*)&m_critDistances[4]);
    pCoder->Get(m_szIdEmergencyBreakSetSideLeftValue,         (tVoid*)&m_critDistances[5]);
    pCoder->Get(m_szIdEmergencyBreakSetSideRightValue,        (tVoid*)&m_critDistances[6]);
    pCoder->Get(m_szIdEmergencyBreakSetBackLeftValue,         (tVoid*)&m_critDistances[7]);
    pCoder->Get(m_szIdEmergencyBreakSetBackMiddleValue,       (tVoid*)&m_critDistances[8]);
    pCoder->Get(m_szIdEmergencyBreakSetBackRightValue,        (tVoid*)&m_critDistances[9]);

//    LOG_INFO(cString::Format("new crit distances were set: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", m_critDistances[0], m_critDistances[1], m_critDistances[2], m_critDistances[3],
//           m_critDistances[4], m_critDistances[5], m_critDistances[6], m_critDistances[7], m_critDistances[8], m_critDistances[9]));
}

tResult cEmergencyBreak::TransmitEmergencyStatus(bool status)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionEmergencyStatus->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionEmergencyStatus,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_szIdEmergencyStatusSet)
        {
            pCoder->GetID("bEmergencyStop", m_szIdEmergencyStatusValue);
            m_szIdEmergencyStatusSet = tTrue;
        }

        pCoder->Set(m_szIdEmergencyStatusValue, (tVoid*)&status);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputEmergencyStatus.Transmit(pMediaSample);

//    LOG_INFO(cString::Format("Transmitted EmergencyBreak State %d", status));

    RETURN_NOERROR;
}

tResult cEmergencyBreak::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitControl);

    //init mediasample
    cObjectPtr<IMediaSample> pMediaSample;
    //allocate memory to mediasample
    AllocMediaSample((tVoid**)&pMediaSample);

    //create interaction with ddl
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);

    //allocate buffer to write in mediasample
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    {
        //write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdOutputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdOutputspeedControllerValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdOutputspeedControllerTimeStamp);
            m_szIdOutputSpeedSet = tTrue;
        }

        pCoderInput->Set(m_szIdOutputspeedControllerValue, (tVoid*)&speed);
        pCoderInput->Set(m_szIdOutputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputSpeedController.Transmit(pMediaSample);

    RETURN_NOERROR;
}
