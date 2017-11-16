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
#ifndef _EMERGENCY_BREAK_H_
#define _EMERGENCY_BREAK_H_

#define OID_ADTF_TEMPLATE_FILTER "adtf.example.EmergencyBreak"

#define SENSOR_COUNT 10
#define MIN_BREAK_DURATION 5000000   // minimal duration of holding the break (in microseconds)

class cEmergencyBreak : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "Emergency Break", adtf::OBJCAT_DataFilter);

protected:
    cInputPin              m_oInputUsStruct; //tUltrasonicStruct
    std::vector<tBufferID> m_szIdUsStructValues;
    std::vector<tBufferID> m_szIdUsStructTimeStamps;
    tBool                  m_szIdsUsStructSet;

    cInputPin   m_oInputSpeedController;   //typ tSignalValue
    tBufferID   m_szIdInputspeedControllerValue;
    tBufferID   m_szIdInputspeedControllerTimeStamp;
    tBool       m_szIdInputSpeedSet;

    cOutputPin  m_oOutputSpeedController;  //typ tSignalValue
    tBufferID   m_szIdOutputspeedControllerValue;
    tBufferID   m_szIdOutputspeedControllerTimeStamp;
    tBool       m_szIdOutputSpeedSet;

    cOutputPin  m_oOutputEmergencyStatus;  //typ tJuryEmergencyStop
    tBufferID   m_szIdEmergencyStatusValue;
    tBool       m_szIdEmergencyStatusSet;

    cInputPin   m_oInputEmergencyBreakSet;  //typ tEmergencyBreakSet
    tBufferID   m_szIdEmergencyBreakSetFrontLeftValue;
    tBufferID   m_szIdEmergencyBreakSetFrontCenterLeftValue;
    tBufferID   m_szIdEmergencyBreakSetFrontMiddleValue;
    tBufferID   m_szIdEmergencyBreakSetFrontCenterRightValue;
    tBufferID   m_szIdEmergencyBreakSetFrontRightValue;
    tBufferID   m_szIdEmergencyBreakSetSideLeftValue;
    tBufferID   m_szIdEmergencyBreakSetSideRightValue;
    tBufferID   m_szIdEmergencyBreakSetBackLeftValue;
    tBufferID   m_szIdEmergencyBreakSetBackMiddleValue;
    tBufferID   m_szIdEmergencyBreakSetBackRightValue;
    tBool       m_szIdEmergencyBreakSet;

    //mediatype descriptions
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolSignalValue;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyStatus;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyBreakSet;

    //storage for min. US value
    //tSignalValue m_oMinimumUsValue;

    /*! The critical section Minimum Us Value */
    cCriticalSection m_critSecMinimumUsValue;

    /*! The critical section transmit control */
    cCriticalSection m_critSecTransmitControl;

    tInt16   m_critDistances[SENSOR_COUNT];
    tFloat32 m_actualDistances[SENSOR_COUNT];
    tInt64   m_lastTriggeredTime;
    tBool    m_breakActive;
    tFloat32 speed;

public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cEmergencyBreak(const tChar* __info);

    /*! default destructor */
    virtual ~cEmergencyBreak();

protected:
    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.Values
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    tResult Stop(ucom::IException** __exception_ptr = NULL);


    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult ProcessMinimumValueUs(IMediaSample* pMediaSample);
    tResult ProcessSpeedController(IMediaSample* pMediaSample);
    void    setCriticalDistances(IMediaSample* pMediaSample);
    tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
    tResult TransmitEmergencyStatus(bool status);
    void    ActivateEmergencyBreak();
    void    DeactivateEmergencyBreak();
};

//*************************************************************************************************
#endif // _EMERGENCY_BREAK_H_

/*!
*@}
*/
