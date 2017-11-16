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
#include "cSpeedGovernor.h"
/// Create filter shell
ADTF_FILTER_PLUGIN("Speed Governor", OID_ADTF_TEMPLATE_FILTER, cSpeedGovernor);


cSpeedGovernor::cSpeedGovernor(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat("PID::Kp", 0);
    SetPropertyStr("PID::Kp" NSSUBPROP_DESCRIPTION, "P-Wert des PID-Reglers");
    SetPropertyBool("PID::Kp" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::Ki", 0);
    SetPropertyStr("PID::Ki" NSSUBPROP_DESCRIPTION, "I-Wert des PID-Reglers");
    SetPropertyBool("PID::Ki" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::Kd", 0);
    SetPropertyStr("PID::Kd" NSSUBPROP_DESCRIPTION, "D-Wert des PID-Reglers");
    SetPropertyBool("PID::Kd" NSSUBPROP_ISCHANGEABLE, tTrue);
}

cSpeedGovernor::~cSpeedGovernor()
{

}

tResult cSpeedGovernor::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            // in StageFirst you can create and register your static pins.
            if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        //Wheeldata input
        tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
        RETURN_IF_POINTER_NULL(strDescWheelData);
        cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelLeftData));
        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelRightData));

        RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

        RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

        //Speed Input
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

        RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedcontrollerIn", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        //Speed Output
        RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedcontrollerOut", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));
    }
    else if (eStage == StageNormal)
    {
        hasIDwheelLeft = false;
        hasIDwheelRight = false;
        m_szIdInputSpeedSet = false;
        m_szIdOutputSpeedSet = false;
        wheelCountLeft = 0;
        wheelCountRight = 0;
        wheelCountLastFrameRight = 0;
        wheelCountLastFrameLeft = 0;
        goThrough = 0;
        speedIn = 0;
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult cSpeedGovernor::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
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

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cSpeedGovernor::OnPinEvent(IPin* pSource,
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

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &m_oInputWheelLeft)
        {
            RETURN_IF_FAILED(ProcessWheelSampleLeft(pMediaSample));
        }
        else if (pSource == &m_oInputWheelRight)
        {
            RETURN_IF_FAILED(ProcessWheelSampleRight(pMediaSample));
        }
        else if(pSource == &m_oInputSpeedController)
        {
            RETURN_IF_FAILED(ProcessSpeed(pMediaSample));
        }
    }

    RETURN_NOERROR;
}

tResult cSpeedGovernor::PropertyChanged(const tChar* strName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    //associate the properties to the member
    if (cString::IsEqual(strName, "PID::Kp"))
        m_filterProperties.Kp = GetPropertyFloat("PID::Kp");
    else if (cString::IsEqual(strName, "PID::Ki"))
        m_filterProperties.Ki = GetPropertyFloat("PID::Ki");
    else if (cString::IsEqual(strName, "PID::Kd"))
        m_filterProperties.Kd = GetPropertyFloat("PID::Kd");

    RETURN_NOERROR;
}

tResult cSpeedGovernor::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
{
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelLeftData, pMediaSample, pCoderInput);

        if (!hasIDwheelLeft)
        {
            pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataLeftUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataLeftArduinoTimestamp);
            hasIDwheelLeft = tTrue;
        }

        pCoderInput->Get(m_szIDWheelDataLeftUi32WheelTach, (tVoid*)&wheelCountLeft);
        pCoderInput->Get(m_szIDWheelDataLeftArduinoTimestamp, (tVoid*)&wheelLeftTimestamp);

    }
    RETURN_NOERROR;
}

tResult cSpeedGovernor::ProcessWheelSampleRight(IMediaSample* pMediaSample)
{
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelRightData, pMediaSample, pCoderInput);

        if (!hasIDwheelRight)
        {
            pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataRightUi32WheelTach);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataRightArduinoTimestamp);
            hasIDwheelRight = tTrue;
        }

        pCoderInput->Get(m_szIDWheelDataRightUi32WheelTach, (tVoid*)&wheelCountRight);
        pCoderInput->Get(m_szIDWheelDataRightArduinoTimestamp, (tVoid*)&wheelRightTimestamp);

    }

    //read-out the incoming Media Sample

    int maxSpeedForward = -14;
    int maxSpeedBackward = 14;

    tFloat32 speed;
    tFloat32 actualSpeed;
    tUInt64 timestamp;

    float distanceSinceLastFrameRight;
    float distanceSinceLastFrameLeft;

    tUInt64 actualTime = (wheelLeftTimestamp+wheelRightTimestamp)/2;
    float timeSinceLastFrame;

    float speedDeviation;

    ///Tickänderung [m] zum letzten Frame => 1 Tick = 0.005m
    distanceSinceLastFrameRight = float(wheelCountRight - wheelCountLastFrameRight)/**0.005*/;
    distanceSinceLastFrameLeft = float(wheelCountLeft - wheelCountLastFrameLeft)/**0.005*/;

    ///Zeitänderung [s] zum letzten Frame
    timeSinceLastFrame = (actualTime - lastTime)/1000000.0;

    speed = speedIn;

    // Wenn stillstand, nächsten 5 frames nicht regeln
    if(speed == 0){
        goThrough = 0;
    }

    // nicht regeln bis 5 Frames oder NaN
    if(goThrough <= 5 || timeSinceLastFrame == 0){
         goThrough++;
    }
    else if(speed < 0 && speed >= maxSpeedForward){

        ///den echten Speed ausrechnen tick/sekunde
        actualSpeed = ((distanceSinceLastFrameRight+distanceSinceLastFrameLeft)/2)/timeSinceLastFrame;
        ///actualSpeed in Pseudoprozent umrechnen
        actualSpeed /= -16;

        speedDeviation = speed - actualSpeed;

        speed += m_filterProperties.Kp*speedDeviation;

        if(speed != speed){
            speed = 0;
            LOG_INFO("NANANANANANANA BATMAN");
        }
    }

    if(speed > maxSpeedBackward){
        speed = maxSpeedBackward;
    }
    else if(speed < maxSpeedForward){
        speed = maxSpeedForward;
    }

//    LOG_INFO(cString::Format("REAL speed %f", speed));

    ///Alte werte auf neue setzen
    wheelCountLastFrameRight = wheelCountRight;
    wheelCountLastFrameLeft = wheelCountLeft;
    lastTime = actualTime;
    TransmitSpeed(speed, timestamp);

    RETURN_NOERROR;
}

tResult cSpeedGovernor::ProcessSpeed(IMediaSample* pMediaSample)
{
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

        pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speedIn);
    }

    RETURN_NOERROR;
}

tResult cSpeedGovernor::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
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
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);

        //Set all Ids
        if(!m_szIdOutputSpeedSet)
        {
            pCoderOutput->GetID("f32Value", m_szIdOutputspeedControllerValue);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputspeedControllerTimeStamp);
            m_szIdOutputSpeedSet = tTrue;
        }
        pCoderOutput->Set(m_szIdOutputspeedControllerValue, (tVoid*)&speed);
        pCoderOutput->Set(m_szIdOutputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputSpeedController.Transmit(pMediaSample);

    RETURN_NOERROR;
}
