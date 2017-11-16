/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

// arduinofilter.cpp : Definiert die exportierten Funktionen f�r die DLL-Anwendung.
//
#include "stdafx.h"
#include "cCalibration.h"

ADTF_FILTER_PLUGIN("AADC Calibration", OID_ADTF_CALIBRATION_SCALING, cCalibration)

cCalibration::cCalibration(const tChar* __info) : cFilter(__info)
{
    // init value for scale factor
    m_f32ScaleFactor = 1.0;

    // create the filter properties
    SetPropertyFloat("Scale Factor",m_f32ScaleFactor);
    SetPropertyFloat("Scale Factor" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Scale Factor" NSSUBPROP_DESCRIPTION, "The input value is multiplied with the here given factor");
}

cCalibration::~cCalibration()
{
}

tResult cCalibration::CreateInputPins(__exception)
{
    // create the input pins
    RETURN_IF_FAILED(m_oInput.Create("input_value", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));
    RETURN_NOERROR;
}

tResult cCalibration::CreateOutputPins(__exception)
{
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


    //get description for signal value input pin
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescSignalValue);

    //get mediatype
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get mediatype description for data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignal));

    // create output pins
    RETURN_IF_FAILED(m_oOutput.Create("output_value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutput));

    RETURN_NOERROR;
}

tResult cCalibration::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        // create all the pins
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (eStage == StageNormal)
    {
        m_f32ScaleFactor = tFloat32(GetPropertyFloat("Scale Factor"));
    }
    else if (eStage == StageGraphReady)
    {
        // ids for media description signals not set yet
        m_bIDsSignalSet = tFalse;
    }

    RETURN_NOERROR;
}

tResult cCalibration::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cCalibration::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cCalibration::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cCalibration::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pMediaSample != NULL && m_pDescriptionSignal != NULL)
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32 ui32TimeStamp = 0;
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
                pCoderInput->Get(m_szIDSignalF32Value, (tVoid*)&f32Value);
                pCoderInput->Get(m_szIDSignalArduinoTimestamp, (tVoid*)&ui32TimeStamp);
            }
            // doing the calibration
            f32Value = (m_f32ScaleFactor * f32Value);
            //if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x Value %f",ID,value));

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

                // get the IDs for the items in the media sample
                if(!m_bIDsSignalSet)
                {
                    pCoderOutput->GetID("f32Value", m_szIDSignalF32Value);
                    pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDSignalArduinoTimestamp);
                    m_bIDsSignalSet = tTrue;
                }

                // set values in new media sample
                pCoderOutput->Set(m_szIDSignalF32Value, (tVoid*)&(f32Value));
                pCoderOutput->Set(m_szIDSignalArduinoTimestamp, (tVoid*)&ui32TimeStamp);
            }

            //transmit media sample over output pin
            pNewMediaSample->SetTime(pMediaSample->GetTime());
            m_oOutput.Transmit(pNewMediaSample);
        }
    }

    RETURN_NOERROR;
}

