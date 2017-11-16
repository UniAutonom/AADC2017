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
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/


#include "stdafx.h"
#include "cSteeringController.h"


#define SC_PROP_DEBUG_MODE "Debug Mode"
#define SC_PROP_CONFIG_FILE "Configuration File For Mapping"

ADTF_FILTER_PLUGIN("AADC Steering Controller", OID_ADTF_STEERINGCONTROLLER, cSteeringController)

cSteeringController::cSteeringController(const tChar* __info) : cFilter(__info), m_bDebugModeEnabled(tFalse)
{
    SetPropertyBool(SC_PROP_DEBUG_MODE, tFalse);
    SetPropertyStr(SC_PROP_DEBUG_MODE NSSUBPROP_DESCRIPTION, "If true debug infos are plotted to console");

    SetPropertyStr(SC_PROP_CONFIG_FILE,"");
    SetPropertyBool(SC_PROP_CONFIG_FILE NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr(SC_PROP_CONFIG_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr(SC_PROP_CONFIG_FILE NSSUBPROP_DESCRIPTION, "The XML to be loaded has to be set here");

}

cSteeringController::~cSteeringController()
{
}


tResult cSteeringController::CreateInputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionCurvature));

    // create pins
    RETURN_IF_FAILED(m_oInputCurvature.Create("curvature", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputCurvature));

    RETURN_NOERROR;
}

tResult cSteeringController::CreateOutputPins(__exception)
{
    // create description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    // get media tayp
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // set member media description
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionServoAngle));

    // create pin
    RETURN_IF_FAILED(m_oOutputServoAngle.Create("servoAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputServoAngle));

    RETURN_NOERROR;
}

tResult cSteeringController::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (eStage == StageNormal)
    {

        m_bDebugModeEnabled = GetPropertyBool(SC_PROP_DEBUG_MODE);

        //Get path of configuration file
        cFilename filenameConfig = GetPropertyStr(SC_PROP_CONFIG_FILE);

        // check if file exits
        if (filenameConfig.IsEmpty())
        {
            LOG_ERROR("Configuration file not set in properties of Steering Controller");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        // create absolute path
        ADTF_GET_CONFIG_FILENAME(filenameConfig);
        filenameConfig = filenameConfig.CreateAbsolutePath(".");

        //Load file, parse configuration, print the data

        if (cFileSystem::Exists(filenameConfig)==tFalse)
        {
            LOG_ERROR("Configuration file set in properties of Steering Controller not found");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
        else
        {

            // load configuration file
            LoadConfigurationData(filenameConfig);

            if (m_bDebugModeEnabled) PrintConfigurationData();
        }

    }
    else if(eStage == StageGraphReady)
    {
        // set the flags which indicate if the media descriptions strings were set
        m_bCurvatureSet = tFalse;
        m_bServoAngleSet = tFalse;
    }

    RETURN_NOERROR;
}

tResult cSteeringController::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cSteeringController::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cSteeringController::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageNormal)
    {

    }
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cSteeringController::LoadConfigurationData(const cFilename& configFilename)
{
    //open configuration file
    if (cFileSystem::Exists(configFilename))
    {
        cDOM oDOM;
        oDOM.Load(configFilename);
        //load supporting points
        cDOMElementRefList oElems;
        if(IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                cDOMElement* pXValue;
                cDOMElement* pYValue;
                if (IS_OK((*itElem)->FindNode("xValue", pXValue)) && IS_OK((*itElem)->FindNode("yValue", pYValue)))
                {
                    if (pXValue->GetData().AsFloat64() < 0)
                    {
                        m_f32ValuesNeg.insert(m_f32ValuesNeg.begin(), std::make_pair(static_cast<tFloat32>(pXValue->GetData().AsFloat64()),static_cast<tFloat32>(pYValue->GetData().AsFloat64())));
                    }
                    else if (pXValue->GetData().AsFloat64() > 0)
                    {
                        m_f32ValuesPos.insert(m_f32ValuesPos.begin(), std::make_pair(static_cast<tFloat32>(pXValue->GetData().AsFloat64()),static_cast<tFloat32>(pYValue->GetData().AsFloat64())));
                    }
                    // nothing is done with 0. 0 is added manually
                }
            }
        }
        if (oElems.size() == 0)
        {
            LOG_ERROR("No supporting points in given file found!");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
    }
    else
    {
        LOG_ERROR("Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}


tResult cSteeringController::PrintConfigurationData()
{
    // print negative values
    tInt i = 0;
    for (vector<std::pair<tFloat32,tFloat32> >::iterator it = m_f32ValuesNeg.begin(); it< m_f32ValuesNeg.end(); it++)
    {
        LOG_INFO(cString::Format("Steering Controller: supportingPoint #%d: (%lf/%lf)",i,it->first,it->second));
        i++;
    }

    // print positive values
    i = 0;
    for (vector<std::pair<tFloat32,tFloat32> >::iterator it = m_f32ValuesPos.begin(); it< m_f32ValuesPos.end(); it++)
    {
        LOG_INFO(cString::Format("Steering Controller: supportingPoint #%d: (%lf/%lf)",i,it->first,it->second));
        i++;
    }

    RETURN_NOERROR;
}


tResult cSteeringController::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
    {

        RETURN_IF_POINTER_NULL( pMediaSample);
        if (pSource == &m_oInputCurvature)
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32 Ui32TimeStamp = 0;
            {
                // focus for sample write lock
                //read data from the media sample with the coder of the descriptor
                __adtf_sample_read_lock_mediadescription(m_pDescriptionCurvature,pMediaSample,pCoder);

                if(!m_bCurvatureSet)
                {
                    pCoder->GetID("f32Value", m_buIDCurvatureF32Value);
                    pCoder->GetID("ui32ArduinoTimestamp", m_buIDCurvatureArduinoTimestamp);
                    m_bCurvatureSet = tTrue;
                }
                //get values from media sample
                pCoder->Get(m_buIDCurvatureF32Value, (tVoid*)&f32Value);
                pCoder->Get(m_buIDCurvatureArduinoTimestamp, (tVoid*)&Ui32TimeStamp);
            }

            //create new media sample
            cObjectPtr<IMediaSample> pNewMediaSample;
            AllocMediaSample((tVoid**)&pNewMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pDescriptionServoAngle->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pNewMediaSample->AllocBuffer(nSize);
            f32Value = getCosineInterpolation(f32Value);
            {
                // focus for sample write lock
                //read data from the media sample with the coder of the descriptor
                __adtf_sample_write_lock_mediadescription(m_pDescriptionServoAngle,pNewMediaSample,pCoderOut);

                if(!m_bServoAngleSet)
                {
                    pCoderOut->GetID("f32Value", m_buIDServoAngleF32Value);
                    pCoderOut->GetID("ui32ArduinoTimestamp", m_buIDServoAngleArduinoTimestamp);
                    m_bServoAngleSet = tTrue;
                }
                //get values from media sample
                pCoderOut->Set(m_buIDServoAngleF32Value, (tVoid*)&f32Value);
            }

            //transmit media sample over output pin
            RETURN_IF_FAILED(pNewMediaSample->SetTime(_clock->GetStreamTime()));
            RETURN_IF_FAILED(m_oOutputServoAngle.Transmit(pNewMediaSample));
        }

    }
    RETURN_NOERROR;
}

tTimeStamp cSteeringController::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 cSteeringController::getCosineInterpolation(tFloat32 f32Value)
{
    if (f32Value<0)
    {
        // value is positive, use m_f32ValuesNeg
        if (f32Value<=m_f32ValuesNeg.front().first)
        {
            // value is smaller than minimum value ( no curvature)
            return 90.f;
        }
        else if (f32Value>=m_f32ValuesNeg.back().first)
        {
            // value is bigger than maximum value (smallest curvature)
            return m_f32ValuesNeg.back().second;
        }
        else
        {
            for (vector<std::pair<tFloat32,tFloat32> >::iterator it = m_f32ValuesNeg.begin(); it< m_f32ValuesNeg.end(); it++)
            {
                // find the two points were the value is in the middle
                if (f32Value<it->first)
                {
                    // get preceding iterator
                    vector<std::pair<tFloat32,tFloat32> >::iterator itPre = it;
                    itPre--;
                    return doCosineInterpolation(itPre->second,it->second,(f32Value-itPre->first)/(it->first-itPre->first));
                }
            }
        }
    }
    else if (f32Value>0)
    {
        // value is positive, use m_f32ValuesNeg
        if (f32Value<=m_f32ValuesPos.front().first)
        {
            // value is bigger than maximum value (smallest curvature)
            return m_f32ValuesPos.front().second;
        }
        else if (f32Value>=m_f32ValuesPos.back().first)
        {
            // value is smaller than minimum value ( no curvature)
            return 90.f;
        }
        else
        {
            for (vector<std::pair<tFloat32,tFloat32> >::iterator it = m_f32ValuesPos.begin(); it< m_f32ValuesPos.end(); it++)
            {
                // find the two points were the value is in the middle
                if (f32Value<it->first)
                {
                    // get preceding iterator
                    vector<std::pair<tFloat32,tFloat32> >::iterator itPre = it;
                    itPre--;
                    return doCosineInterpolation(itPre->second,it->second,(f32Value-itPre->first)/(it->first-itPre->first));
                }
            }
        }
    }
    return 0;
}

tFloat32 cSteeringController::doCosineInterpolation(tFloat32 f32y1, tFloat32 f32y2, tFloat32 f32mu)
{
    tFloat32 f32mu2 = tFloat32(1-cos(f32mu*M_PI))/2.0f;

    return (f32y1*(1-f32mu2)+f32y2*f32mu2);
}