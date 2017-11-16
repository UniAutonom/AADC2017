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
#include "cDriverModule.h"


ADTF_FILTER_PLUGIN("AADC Driver Module", __guid, cDriverModule);




cDriverModule::cDriverModule(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{
    m_bDebugModeEnabled = tFalse;
    m_bIDsDriverStructSet = tFalse;
    m_bIDsJuryStructSet = tFalse;
    SetPropertyBool("Debug Output to Console",false);

}

cDriverModule::~cDriverModule()
{
}

tHandle cDriverModule::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidgetDriver(pWidget);

    connect(m_pWidget, SIGNAL(sendStruct(stateCar, tInt16)), this, SLOT(OnSendState(stateCar, tInt16)));
    connect(this, SIGNAL(SendRun(int)), m_pWidget, SLOT(OnDriverGo(int)));
    connect(this, SIGNAL(SendStop(int)), m_pWidget, SLOT(OnDriverStop(int)));
    connect(this, SIGNAL(SendRequestReady(int)), m_pWidget, SLOT(OnDriverRequestReady(int)));
    connect(this, SIGNAL(TriggerLoadManeuverList()), this, SLOT(LoadManeuverList()));

    return (tHandle)m_pWidget;
}

tResult cDriverModule::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cDriverModule::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)
    {

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager,
                                             __exception_ptr));

        // input jury struct
        tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strDesc1);
        cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pType1, this));
        RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
        RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescJuryStruct));

        // input maneuver list
        tChar const * strDesc3 = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strDesc3);
        cObjectPtr<IMediaType> pType3 = new cMediaType(0, 0, 0, "tManeuverList", strDesc3, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pType3, this));
        RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
        RETURN_IF_FAILED(pType3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescManeuverList));

        // output driver struct
        tChar const * strDesc2 = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDesc2);
        cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pType2, this));
        RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
        RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescDriverStruct));

        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if(eStage == StageGraphReady)
    {
        // disable maneuver group box until receiving maneuver list
        m_pWidget->EnableManeuverGroupBox(false);
        // no ids were set so far
        m_bIDsJuryStructSet = tFalse;
        m_bIDsDriverStructSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cDriverModule::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));


    RETURN_NOERROR;
}

tResult cDriverModule::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{

    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult cDriverModule::Stop(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult cDriverModule::Shutdown(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Shutdown(eStage, __exception_ptr));
    RETURN_NOERROR;
}

tResult cDriverModule::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);



    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {

        if (pSource == &m_JuryStructInputPin && m_pDescJuryStruct != NULL)
        {
            tInt8 i8ActionID = -2;
            tInt16 i16entry = -1;

            {
                // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescJuryStruct,pMediaSample,pCoder);
                // get the IDs for the items in the media sample
                if(!m_bIDsJuryStructSet)
                {
                    pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
                    pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
                    m_bIDsJuryStructSet = tTrue;
                }

                pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
                pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
            }

            switch (juryActions(i8ActionID))
            {
            case action_GETREADY:
                if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d",i16entry));
                emit SendRequestReady((int)i16entry);
                break;
            case action_START:
                if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d",i16entry));
                emit SendRun((int)i16entry);
                break;
            case action_STOP:
                if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d",i16entry));
                emit SendStop((int)i16entry);
                break;
            }

        }
        else if (pSource == &m_ManeuverListInputPin && m_pDescManeuverList != NULL)
        {

            {
                // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescManeuverList,pMediaSample,pCoder);


                std::vector<tSize> vecDynamicIDs;

                // retrieve number of elements by providing NULL as first paramter
                tSize szBufferSize = 0;
                if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
                {
                    // create a buffer depending on the size element
                    tChar* pcBuffer = new tChar[szBufferSize];
                    vecDynamicIDs.resize(szBufferSize);
                    // get the dynamic ids (we already got the first "static" size element)
                    if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
                    {
                        // iterate over all elements
                        for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
                        {
                            // get the value and put it into the buffer
                            pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
                        }

                        // set the resulting char buffer to the string object
                        m_strManeuverFileString = (const tChar*) pcBuffer;
                    }

                    // cleanup the buffer
                    delete pcBuffer;
                }

            }

            // trigger loading maneuver list and update the ui
            TriggerLoadManeuverList();
        }
    }
    RETURN_NOERROR;

}
tResult cDriverModule::OnSendState(stateCar stateID, tInt16 i16ManeuverEntry)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    tInt8 value = tInt8(stateID);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescDriverStruct,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_bIDsDriverStructSet)
        {
            pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
            pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
            m_bIDsDriverStructSet = tTrue;
        }


        pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&value);
        pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DriverStructOutputPin.Transmit(pMediaSample);
    if(m_bDebugModeEnabled)
    {
        switch (stateID)
        {
        case stateCar_READY:
            LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_RUNNING:
            LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_COMPLETE:
            LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_ERROR:
            LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_STARTUP:
            LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d",i16ManeuverEntry));
            break;
        }
    }
    RETURN_NOERROR;
}

tResult cDriverModule::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);
                }
            }

            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        m_pWidget->EnableManeuverGroupBox(true);
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        m_pWidget->EnableManeuverGroupBox(false);
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // update the ui
    m_pWidget->SetManeuverList(m_sectorList);
    m_pWidget->ShowManeuverList();
    m_pWidget->FillComboBox();


    RETURN_NOERROR;
}




