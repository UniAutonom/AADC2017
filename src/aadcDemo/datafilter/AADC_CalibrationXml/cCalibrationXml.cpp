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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/


#include "stdafx.h"
#include "cCubic.h"
#include "cCalibrationXml.h"

ADTF_FILTER_PLUGIN("AADC Calibration XML", OID_ADTF_SENSOR_CALIBRATION, cCalibrationXml)

cCalibrationXml::cCalibrationXml(const tChar* __info) : cFilter(__info), m_bDebugModeEnabled(tFalse)
{
    // create the filter properties
    SetPropertyInt("Interpolation", 1);
    SetPropertyStr("Interpolation" NSSUBPROP_VALUELISTNOEDIT, "1@linear|2@cubic|3@none");
    SetPropertyStr("Interpolation" NSSUBPROP_DESCRIPTION, "Sets the mode of interpolation between the given points in the XML");

    SetPropertyStr("Configuration File For Interpolation","");
    SetPropertyBool("Configuration File For Interpolation" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration File For Interpolation" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration File For Interpolation" NSSUBPROP_DESCRIPTION, "The XML to be loaded has to be set here");

    SetPropertyBool("Border Warnings to Console",tFalse);
    SetPropertyStr("Border Warnings to Console" NSSUBPROP_DESCRIPTION, "If enabled a warning is printed to console each time the border points of the given xml are reached");

    SetPropertyBool("Print initial table to Console",tFalse);
    SetPropertyStr("Print initial table to Console" NSSUBPROP_DESCRIPTION, "If enabled the loaded points of the interpolation table of the XML are printed to console");
}

cCalibrationXml::~cCalibrationXml()
{
}

tResult cCalibrationXml::CreateInputPins(__exception)
{
    // create the input pins
    RETURN_IF_FAILED(m_oInput.Create("input_value", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));

    RETURN_NOERROR;
}

tResult cCalibrationXml::CreateOutputPins(__exception)
{
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //get description for arduino input pin
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");

    // checks if exists
    RETURN_IF_POINTER_NULL(strDescSignalValue);

    //get mediatype for tSignalValue data
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    //get mediatype description for arduino data type
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

    // creates the output pin
    RETURN_IF_FAILED(m_oOutput.Create("output_value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutput));

    RETURN_NOERROR;
}

tResult cCalibrationXml::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
        // get calibration mode
        m_iCalibrationMode = GetPropertyInt("Interpolation");
        //load xml files for linear interpolation
        THROW_IF_FAILED(LoadConfigurationData());

        //create class for cubic spline interpolation
        if (m_iCalibrationMode==2)
        {
            m_cubicInterpolation = new Cubic(tInt(m_xValues.size()),m_xValues,m_yValues);
        }

        m_bDebugModeEnabled = GetPropertyBool("Border Warnings to Console");
    }
    else if (eStage == StageGraphReady)
    {
        // ids for media description signals not set yet
        m_bIDsSignalSet = tFalse;
    }

    RETURN_NOERROR;
}

tResult cCalibrationXml::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cCalibrationXml::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cCalibrationXml::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cCalibrationXml::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32 ui32TimeStamp = 0;
            {
                // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pCoderDescSignal,pMediaSample,pCoderInput);

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
            if (m_iCalibrationMode==1)
                f32Value = getLinearInterpolatedValue(f32Value);
            else if (m_iCalibrationMode==2)
                f32Value = getCubicSplineInterpolatedValue(f32Value);
            //else if (GetPropertyInt("Interpolation", 1)==3)    //just for explanation
            //    f32Value = f32Value;

            //create new media sample
            cObjectPtr<IMediaSample> pNewMediaSample;
            AllocMediaSample((tVoid**)&pNewMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pNewMediaSample->AllocBuffer(nSize);

            {
                // focus for sample write lock
                //write date to the media sample with the coder of the descriptor
                __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal,pNewMediaSample,pCoderOutput);

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

tResult cCalibrationXml::LoadConfigurationData()
{
    //Get path of configuration file
    m_fileConfig = GetPropertyStr("Configuration File For Interpolation");

    // check if file exits
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("cCalibrationXml: Configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // create absolute path
    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data

    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        //load settings for calibration mode
        cDOMElementRefList oElemsSettings;
        if(IS_OK(oDOM.FindNodes("calibration/settings", oElemsSettings)))
        {
            for (cDOMElementRefList::iterator itElem = oElemsSettings.begin(); itElem != oElemsSettings.end(); ++itElem)
            {
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("mode", pConfigElement)))
                {

                    cString rdMode = pConfigElement->GetData();
                    LOG_INFO(adtf_util::cString::Format("cCalibrationXml: %s",rdMode.GetPtr()));
                    if (cString::Compare(rdMode,"linear")==0)
                        m_iCalibrationMode=1;
                    else if (cString::Compare(rdMode,"cubic")==0)
                        m_iCalibrationMode=2;
                    else if (cString::Compare(rdMode,"none")==0)
                        m_iCalibrationMode=3;
                }
            }
        }
        cString rdMode;
        switch (m_iCalibrationMode)
        {
        case 1:
            rdMode = "linear";
            break;
        case 2:
            rdMode = "cubic";
            break;
        case 3:
            rdMode = "none";
            break;
        }
        LOG_INFO(adtf_util::cString::Format("cCalibrationXml: Calibration mode is %s",rdMode.GetPtr()));
        //load supporting points
        if (m_iCalibrationMode!=3)
        {
            cDOMElementRefList oElems;
            if(IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems)))
            {
                for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
                {

                    cDOMElement* pConfigElement;
                    if (IS_OK((*itElem)->FindNode("xValue", pConfigElement)))
                    {
                        m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                    }
                    if (IS_OK((*itElem)->FindNode("yValue", pConfigElement)))
                    {
                        m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                    }
                }
            }
            if (oElems.size() > 0)
            {
                if (GetPropertyBool("Print initial table to Console"))
                {
                    for (tUInt i = 0; i<m_xValues.size(); i++)
                    {
                        if (i>m_yValues.size()) break;
                        LOG_INFO(cString::Format("cCalibrationXml: supportingPoint #%d: (%lf/%lf)",i,m_xValues[i],m_yValues[i]));
                    }
                }
            }
            else
            {
                LOG_ERROR("cCalibrationXml: no supporting points in given file found!");
                RETURN_ERROR(ERR_INVALID_FILE);
            }
            //checks if data are valid
            RETURN_IF_FAILED(CheckConfigurationData());
        }
    }
    else
    {
        LOG_ERROR("cCalibrationXml: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult cCalibrationXml::CheckConfigurationData()
{
    //checks if the xValues of the calibration table are increasing
    for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end() ; it++)
    {
        vector<tFloat32>::iterator it2 = it;
        it2++;
        if (it2 != m_xValues.end())
        {
            // next values is smaller than current value
            if ((tFloat32(*it) > tFloat32(*it2)))
            {
                LOG_ERROR(cString::Format("cCalibrationXml: The xValues in the file %s are not in increasing order. Please reorder the points!",m_fileConfig.GetPtr()));
                RETURN_ERROR(ERR_INVALID_FILE);
            }
        }
    }

    RETURN_NOERROR;
}

tFloat32 cCalibrationXml::getLinearInterpolatedValue(tFloat32 fl32InputValue)
{
    // requested value is smaller than smallest value in table
    if (fl32InputValue<m_xValues.front())
    {
        if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("cCalibrationXml: requested x-value %f is lower than smallest x-value in calibration table",fl32InputValue));
        return m_yValues.front();
    }
    // requested value is bigger than biggest value in table
    else if (fl32InputValue>m_xValues.back())
    {
        if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("cCalibrationXml: requested x-value %f is higher than highes x-value in calibration table",fl32InputValue));
        return m_yValues.back();
    }
    // search in vector for corresponding index (smallest neighbor)
    tUInt iIndex;
    if (m_xValues.size() > 2)
    {
        for (iIndex = 0; iIndex<m_xValues.size(); iIndex++)
        {
            if (m_xValues[iIndex]>=fl32InputValue) break;
        }
        // get smaller neighbor
        if (iIndex!=0) iIndex = iIndex -1;
    }
    else iIndex = 0;

    if ((m_xValues[iIndex+1]-m_xValues[iIndex])!=0)
    {
        // doing the linear interpolation
        tFloat32 f32Value = (fl32InputValue-m_xValues[iIndex])*(m_yValues[iIndex+1]-m_yValues[iIndex])/(m_xValues[iIndex+1]-m_xValues[iIndex])+m_yValues[iIndex];

        //tFloat32 security check to send only minimum or maximum value of table
        if (f32Value > *max_element(m_yValues.begin(),m_yValues.end() ))
            return *max_element(m_yValues.begin(),m_yValues.end()) ;
        else if (f32Value < *min_element(m_yValues.begin(),m_yValues.end()) )
            return *min_element(m_yValues.begin(),m_yValues.end()) ;
        else
            return f32Value;
    }
    else
    {
        LOG_ERROR("cCalibrationXml: invalid table in xml!");
        return 0;
    }
}

tFloat32 cCalibrationXml::getCubicSplineInterpolatedValue(tFloat32 fl32InputValue)
{
    // requested value is smaller than smallest value in table
    if (fl32InputValue<m_xValues.front())
    {
        if (m_bDebugModeEnabled) LOG_WARNING("cCalibrationXml: requested x-value lower than smallest x-value in calibration table");
        return m_yValues.front();
    }
    // requested value is bigger than biggest value in table
    else if (fl32InputValue>m_xValues.back())
    {
        if (m_bDebugModeEnabled) LOG_WARNING("cCalibrationXml: requested x-value higher than highes x-value in calibration table");
        return m_yValues.back();
    }
    // doing cubic interpolation
    return tFloat32(m_cubicInterpolation->getValue(tFloat64(fl32InputValue)));
}
