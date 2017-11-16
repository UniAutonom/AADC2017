/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-15 13:25:09#$ $Rev:: 63254   $
**********************************************************************/


#include "stdafx.h"
#include "cAHRSCalculator.h"

#define DEG_TO_RAD tFloat32(M_PI) / 180.0f

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cAHRSCalculator);

cAHRSCalculator::cAHRSCalculator(const tChar* __info) : cFilter(__info)
{

}

cAHRSCalculator::~cAHRSCalculator()
{
}

tResult cAHRSCalculator::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitData));

    RETURN_IF_FAILED(m_oInputInerMeasUnit.Create("InerMeasUnit_Struct", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputInerMeasUnit));

    RETURN_NOERROR;
}

tResult cAHRSCalculator::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    tChar const * strDescInerMeasUnit = pDescManager->GetMediaDescription("tInerMeasUnitData");
    RETURN_IF_POINTER_NULL(strDescInerMeasUnit);
    cObjectPtr<IMediaType> pTypeInerMeasUnit = new cMediaType(0, 0, 0, "tInerMeasUnitData", strDescInerMeasUnit, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
    RETURN_IF_FAILED(pTypeInerMeasUnit->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInerMeasUnitDataOutput));


    RETURN_IF_FAILED(m_oOutputRoll.Create("Roll", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputRoll));
    RETURN_IF_FAILED(m_oOutputPitch.Create("Pitch", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputPitch));
    RETURN_IF_FAILED(m_oOutputYaw.Create("Yaw", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputYaw));

    RETURN_IF_FAILED(m_oOutputQ0.Create("Q0w", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputQ0));
    RETURN_IF_FAILED(m_oOutputQ1.Create("Q1x", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputQ1));
    RETURN_IF_FAILED(m_oOutputQ2.Create("Q2y", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputQ2));
    RETURN_IF_FAILED(m_oOutputQ3.Create("Q3z", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputQ3));

    RETURN_IF_FAILED(m_oOutputInerMeasUnit.Create("InerMeasUnit_Struct_Updated", pTypeInerMeasUnit, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputInerMeasUnit));

    RETURN_NOERROR;
}

tResult cAHRSCalculator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageGraphReady)
    {
    }

    RETURN_NOERROR;
}

tResult cAHRSCalculator::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cAHRSCalculator::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cAHRSCalculator::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cAHRSCalculator::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oInputInerMeasUnit)
        {
            RETURN_IF_FAILED(ProcessInerMeasUnitSample(pMediaSample));
        }
    }

    RETURN_NOERROR;
}

tResult cAHRSCalculator::ProcessInerMeasUnitSample(IMediaSample* pMediaSample)
{
    tUInt32 ui32ArduinoTimestamp = 0;

    tFloat32 f32A_x = 0;
    tFloat32 f32A_y = 0;
    tFloat32 f32A_z = 0;

    tFloat32 f32G_x = 0;
    tFloat32 f32G_y = 0;
    tFloat32 f32G_z = 0;

    tFloat32 f32M_x = 0;
    tFloat32 f32M_y = 0;
    tFloat32 f32M_z = 0;

    tFloat32 f32roll = 0;
    tFloat32 f32pitch = 0;
    tFloat32 f32yaw = 0;

    static tUInt32 lastArduinoTimestamp = 0;

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
        __adtf_sample_read_lock_mediadescription(m_pDescriptionInerMeasUnitData, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("ui32ArduinoTimestamp", szIDInerMeasUnitArduinoTimestamp);

            pCoderInput->GetID("f32A_x", szIDInerMeasUnitF32A_x);
            pCoderInput->GetID("f32A_y", szIDInerMeasUnitF32A_y);
            pCoderInput->GetID("f32A_z", szIDInerMeasUnitF32A_z);

            pCoderInput->GetID("f32G_x", szIDInerMeasUnitF32G_x);
            pCoderInput->GetID("f32G_y", szIDInerMeasUnitF32G_y);
            pCoderInput->GetID("f32G_z", szIDInerMeasUnitF32G_z);

            pCoderInput->GetID("f32M_x", szIDInerMeasUnitF32M_x);
            pCoderInput->GetID("f32M_y", szIDInerMeasUnitF32M_y);
            pCoderInput->GetID("f32M_z", szIDInerMeasUnitF32M_z);

            pCoderInput->GetID("f32roll", szIDInerMeasUnitF32roll);
            pCoderInput->GetID("f32pitch", szIDInerMeasUnitF32pitch);
            pCoderInput->GetID("f32yaw", szIDInerMeasUnitF32yaw);

            hasID = true;
        }

        pCoderInput->Get(szIDInerMeasUnitF32A_x, (tVoid*)&f32A_x);
        pCoderInput->Get(szIDInerMeasUnitF32A_y, (tVoid*)&f32A_y);
        pCoderInput->Get(szIDInerMeasUnitF32A_z, (tVoid*)&f32A_z);

        pCoderInput->Get(szIDInerMeasUnitF32G_x, (tVoid*)&f32G_x);
        pCoderInput->Get(szIDInerMeasUnitF32G_y, (tVoid*)&f32G_y);
        pCoderInput->Get(szIDInerMeasUnitF32G_z, (tVoid*)&f32G_z);

        pCoderInput->Get(szIDInerMeasUnitF32M_x, (tVoid*)&f32M_x);
        pCoderInput->Get(szIDInerMeasUnitF32M_y, (tVoid*)&f32M_y);
        pCoderInput->Get(szIDInerMeasUnitF32M_z, (tVoid*)&f32M_z);

        pCoderInput->Get(szIDInerMeasUnitArduinoTimestamp, (tVoid*)&ui32ArduinoTimestamp);

    }


    tUInt32 deltat = ui32ArduinoTimestamp - lastArduinoTimestamp;
    tFloat32 delta = static_cast<tFloat32> (deltat) / 1000000.f; // for the mahony update we need seconds in floaties

    MahonyQuaternionUpdate(
        f32A_x, f32A_y, f32A_z,
        f32G_x * DEG_TO_RAD, f32G_y * DEG_TO_RAD, f32G_z * DEG_TO_RAD,
        f32M_y, f32M_x, f32M_z,
        delta);

    CalculateEulerAngles(&f32roll, &f32pitch, &f32yaw);
    
    TransmitFloatValue(&m_oOutputRoll, f32roll, ui32ArduinoTimestamp);
    TransmitFloatValue(&m_oOutputPitch, f32pitch, ui32ArduinoTimestamp);
    TransmitFloatValue(&m_oOutputYaw, f32yaw, ui32ArduinoTimestamp);

    TransmitFloatValue(&m_oOutputQ0, *(getQ()), ui32ArduinoTimestamp);
    TransmitFloatValue(&m_oOutputQ1, *(getQ()+1), ui32ArduinoTimestamp);
    TransmitFloatValue(&m_oOutputQ2, *(getQ()+2), ui32ArduinoTimestamp);
    TransmitFloatValue(&m_oOutputQ3, *(getQ()+3), ui32ArduinoTimestamp);

    TransmitInerMeasUnitData(f32A_x, f32A_y, f32A_z, f32G_x, f32G_y, f32G_z, f32M_x, f32M_y, f32M_z, f32roll, f32pitch, f32yaw, ui32ArduinoTimestamp);

    lastArduinoTimestamp = ui32ArduinoTimestamp;


    RETURN_NOERROR;
}

tResult cAHRSCalculator::TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp)
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("f32Value", szIDValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDValueOutput, (tVoid*)&value);
        pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cAHRSCalculator::TransmitInerMeasUnitData(
    tFloat32 ax, tFloat32 ay, tFloat32 az,
    tFloat32 gx, tFloat32 gy, tFloat32 gz,
    tFloat32 mx, tFloat32 my, tFloat32 mz,
    tFloat32 roll, tFloat32 pitch, tFloat32 yaw,
    tUInt32 arduinoTimestamp)
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
        __adtf_sample_write_lock_mediadescription(m_pDescriptionInerMeasUnitDataOutput, pMediaSample, pCoder);

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

        pCoder->Set(szIDInerMeasUnitArduinoTimestamp, (tVoid*)&arduinoTimestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputInerMeasUnit.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult cAHRSCalculator::CalculateEulerAngles(tFloat32* roll, tFloat32* pitch, tFloat32* yaw)
{
#define RAD_TO_DEG 180.0f / M_PI
    *yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                           *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                   - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    *pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                           *(getQ()+2)));
    *roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                           *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                   - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    *pitch *= tFloat32(RAD_TO_DEG);
    *yaw   *= tFloat32(RAD_TO_DEG);
    // Declination of Ingolstart is
    //   2.99° E  ± 0.36°  changing by  0.12° E per year on 2017-04-04
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    *yaw   += tFloat32(2.99);
    *roll  *= tFloat32(RAD_TO_DEG);
#undef RAD_TO_DEG
    RETURN_NOERROR;
}
