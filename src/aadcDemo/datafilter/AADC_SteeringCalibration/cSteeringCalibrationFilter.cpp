/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-18 16:51:13#$ $Rev:: 63512   $
**********************************************************************/

#include "stdafx.h"
#include "cSteeringCalibrationFilter.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("AADC Steering Calibration", __guid, cSteeringCalibrationFilter);


cSteeringCalibrationFilter::cSteeringCalibrationFilter(const tChar* __info):cBaseQtFilter(__info)
{
    m_prop_maxArcDistance = 5.0;
    SetPropertyFloat("max arc distance [m]",m_prop_maxArcDistance);
    SetPropertyFloat("max arc distance [m]" NSSUBPROP_MIN,1);
    SetPropertyFloat("max arc distance [m]" NSSUBPROP_MAX,20);
    SetPropertyStr("max arc distance [m]" NSSUBPROP_DESCRIPTION, "The maximum arc distance driven in calibration");

    SetPropertyFloat("max angle [°]",170.0);
    SetPropertyFloat("max angle [°]" NSSUBPROP_MIN, 10.0);
    SetPropertyFloat("max angle [°]" NSSUBPROP_MAX, 170.0);
    SetPropertyStr("max angle [°]" NSSUBPROP_DESCRIPTION, "The maximum angle driven in calibration");

    m_prop_averageSpeed = 1.0;
    SetPropertyFloat("average speed [m/s]",m_prop_averageSpeed);
    SetPropertyStr("average speed [m/s]" NSSUBPROP_DESCRIPTION, "The average speed during calibration procedure.");
}

cSteeringCalibrationFilter::~cSteeringCalibrationFilter()
{
}

tResult cSteeringCalibrationFilter::doDriving()
{
    if(m_model.isDriving())
    {
        transmitSteerAngle(m_model.getSteerAngle());
        transmitSpeed(m_prop_averageSpeed);
    }
    else
    {
        transmitSteerAngle(90);
        transmitSpeed(0);
    }

    RETURN_NOERROR;
}

tResult cSteeringCalibrationFilter::transmitSteerAngle(tFloat32 angle)
{
    //write values with zero
    tFloat32 value = angle;
    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    AllocMediaSample((tVoid**)&pNewMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pNewMediaSample->AllocBuffer(nSize);
    {
        // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignal,pNewMediaSample,pCoderOutput);

        pCoderOutput->Set("f32Value", (tVoid*)&(value));
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    }

    //transmit media sample over output pin
    pNewMediaSample->SetTime(_clock->GetTime());
    return m_oSteeringOutput.Transmit(pNewMediaSample);
}

tResult cSteeringCalibrationFilter::transmitSpeed(tFloat32 speed)
{
    //write values with zero
    tFloat32 value = speed;
    tUInt32 timeStamp = 0;

    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    AllocMediaSample((tVoid**)&pNewMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pNewMediaSample->AllocBuffer(nSize);
    {
        // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignal,pNewMediaSample,pCoderOutput);

        pCoderOutput->Set("f32Value", (tVoid*)&(value));
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    }

    //transmit media sample over output pin
    pNewMediaSample->SetTime(_clock->GetTime());
    return m_oSpeedOutput.Transmit(pNewMediaSample);
}

tResult cSteeringCalibrationFilter::Start(__exception)
{
    return cBaseQtFilter::Start(__exception_ptr);
}
tResult cSteeringCalibrationFilter::Stop(__exception )
{

    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult cSteeringCalibrationFilter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignal));

        //create pin for distance data input
        RETURN_IF_FAILED(m_oDistanceInput.Create("distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oDistanceInput));

        //create pin for yaw data input
        RETURN_IF_FAILED(m_oYawInput.Create("yaw", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oYawInput));

        //create pin for steering data output
        RETURN_IF_FAILED(m_oSteeringOutput.Create("SteeringController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringOutput));

        //create pin for speed data output
        RETURN_IF_FAILED(m_oSpeedOutput.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSpeedOutput));


    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
        m_prop_maxArcDistance = GetPropertyFloat("max arc distance [m]" );
        m_prop_maxAngle = GetPropertyFloat("max angle [°]") * cStdMath::MATH_DEG2RAD;
        m_prop_averageSpeed = GetPropertyFloat("average speed [m/s]");


    }
    else if (eStage == StageGraphReady)
    {

        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
        m_model.setLimits(m_prop_maxArcDistance,m_prop_maxAngle);

        m_bIDsSignalSet= tFalse;
    }

    RETURN_NOERROR;
}

tResult cSteeringCalibrationFilter::Shutdown(tInitStage eStage, __exception)
{
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
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}


tHandle cSteeringCalibrationFilter::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();

    m_calibrationWidget = new cCalibrationView(&m_model.m_tableViewModel, pWidget);
    QObject::connect(m_calibrationWidget,SIGNAL(startDrive(int)),&m_model,SLOT(onStartDrive(int)));
    return (tHandle) m_calibrationWidget;
}

tResult cSteeringCalibrationFilter::ReleaseView()
{
    if (m_calibrationWidget != NULL)
    {
        delete m_calibrationWidget;
        m_calibrationWidget = NULL;
    }
    RETURN_NOERROR;
}


tResult cSteeringCalibrationFilter::processDistance(tFloat32 distance)
{
    m_model.updateDistance(	distance);
    RETURN_NOERROR;
}


tResult cSteeringCalibrationFilter::processYaw(tFloat32 Yaw)
{
    m_model.updateYaw(Yaw);
    RETURN_NOERROR;
}


tResult cSteeringCalibrationFilter::OnPinEvent(IPin* pSource,
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
        if (pSource == &m_oYawInput)
        {

            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            {
                // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionSignal,pMediaSample,pCoderInput);

                // get the IDs for the items in the media sample
                if(!m_bIDsSignalSet)
                {
                    pCoderInput->GetID("f32Value", m_szIDSignalF32Value);
                    pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoderInput->Get(m_szIDSignalF32Value, (tVoid*)&value);
                pCoderInput->Get(m_szIDSignalArduinoTimestamp, (tVoid*)&timeStamp);
            }

            processYaw(value * cStdMath::MATH_DEG2RAD);


        }
        else if(pSource == &m_oDistanceInput)
        {
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            {
                // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionSignal,pMediaSample,pCoderInput);

                // get the IDs for the items in the media sample
                if(!m_bIDsSignalSet)
                {
                    pCoderInput->GetID("f32Value", m_szIDSignalF32Value);
                    pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                //get values from media sample
                pCoderInput->Get(m_szIDSignalF32Value, (tVoid*)&value);
                pCoderInput->Get(m_szIDSignalArduinoTimestamp, (tVoid*)&timeStamp);
            }

            processDistance(value);

            //eatch time we recv a distance media sample, we transmit data to actor
            doDriving();

        }
    }

    RETURN_NOERROR;
}

