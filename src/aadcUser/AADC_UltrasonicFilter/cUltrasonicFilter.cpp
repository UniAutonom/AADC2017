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
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "stdafx.h"
#include "cUltrasonicFilter.h"


#define RINGBUFFER_SIZE 8

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN("Ultrasonic Filter",
                   OID_ADTF_TEMPLATE_FILTER,
                   cUltrasonicFilter);



cUltrasonicFilter::cUltrasonicFilter(const tChar* __info) : cFilter(__info)
{
}

cUltrasonicFilter::~cUltrasonicFilter()
{
}

tResult cUltrasonicFilter::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cUltrasonicFilter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult cUltrasonicFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
        RETURN_IF_POINTER_NULL(strDescUsStruct);
        cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStructInput", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));
        RETURN_IF_FAILED(m_oOutputUsStruct.Create("UsStructOutput", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputUsStruct));
        RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
    }
    else if (eStage == StageNormal)
    {
    }

    else if (eStage == StageGraphReady)
    {
    }
}



tResult cUltrasonicFilter::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cUltrasonicFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oInputUsStruct)
        {
            ProcessUsValues(pMediaSample);
        }
    }
    RETURN_NOERROR;
}

tResult cUltrasonicFilter::ProcessUsValues(IMediaSample* pMediaSample)
{
    tUltrasonicStruct usStructOut;
    //use mutex
    __synchronized_obj(m_critSecRecieveUsValue);

    {
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

        static int count = 0;
        count++;
        if(count < 20)  // ignore first 10 samples
            RETURN_NOERROR;

        static tFloat32 distanceInput[RINGBUFFER_SIZE][10];
        tUInt32 timestampInput[10];
        tFloat32 filteredDistance[10]{400.0f,400.0f, 400.0f, 400.0f, 400.0f, 400.0f, 400.0f, 400.0f, 400.0f, 400.0f};
        tFloat32 buf_UsSignal;
        tUInt32 buf_UsTime;

        static int guterStaticInt = 0;

        for(int i=0; i<m_szIdUsStructValues.size(); ++i)
        {
            pCoderInput->Get(m_szIdUsStructValues[i], (tVoid*)&buf_UsSignal);
            pCoderInput->Get(m_szIdUsStructTimeStamps[i], (tVoid*)&buf_UsTime);
            //save values
            distanceInput[guterStaticInt%RINGBUFFER_SIZE][i] = buf_UsSignal;
            timestampInput[i] = buf_UsTime;
        }

        guterStaticInt++;

        for(unsigned int i=0; i<10;i++){
            float minEle = 800.0f;
            for(unsigned int j = 0; j < RINGBUFFER_SIZE; j++){
                if(distanceInput[j][i] < minEle && distanceInput[j][i] !=-1){
                    minEle = distanceInput[j][i];
                }
            }
            filteredDistance[i]=minEle;
        }


        usStructOut.tFrontLeft.f32Value = filteredDistance[0];
        usStructOut.tFrontLeft.ui32ArduinoTimestamp = timestampInput[0];
        usStructOut.tFrontCenterLeft.f32Value = filteredDistance[1];
        usStructOut.tFrontCenterLeft.ui32ArduinoTimestamp = timestampInput[1];
        usStructOut.tFrontCenter.f32Value = filteredDistance[2];
        usStructOut.tFrontCenter.ui32ArduinoTimestamp = timestampInput[2];
        usStructOut.tFrontCenterRight.f32Value = filteredDistance[3];
        usStructOut.tFrontCenterRight.ui32ArduinoTimestamp = timestampInput[3];
        usStructOut.tFrontRight.f32Value = filteredDistance[4];
        usStructOut.tFrontRight.ui32ArduinoTimestamp = timestampInput[4];
        usStructOut.tSideLeft.f32Value = filteredDistance[5];
        usStructOut.tSideLeft.ui32ArduinoTimestamp = timestampInput[5];
        usStructOut.tSideRight.f32Value = filteredDistance[6];
        usStructOut.tSideRight.ui32ArduinoTimestamp = timestampInput[6];
        usStructOut.tRearLeft.f32Value = filteredDistance[7];
        usStructOut.tRearLeft.ui32ArduinoTimestamp = timestampInput[7];
        usStructOut.tRearCenter.f32Value = filteredDistance[8];
        usStructOut.tRearCenter.ui32ArduinoTimestamp = timestampInput[8];
        usStructOut.tRearRight.f32Value = filteredDistance[9];
        usStructOut.tRearRight.ui32ArduinoTimestamp = timestampInput[9];

    }

    TransmitUltrasonicStructData(usStructOut);


    RETURN_NOERROR;
}

tResult cUltrasonicFilter::TransmitUltrasonicStructData(tUltrasonicStruct filteredDistances)
{
    // create new pointer for media sample
    cObjectPtr<IMediaSample> pMediaSample = new cMediaSample();

    // update media sample with whole struct and current time
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), &filteredDistances, sizeof(filteredDistances), 0));

    // transmit media sample
    RETURN_IF_FAILED(m_oOutputUsStruct.Transmit(pMediaSample));

    RETURN_NOERROR;
}
