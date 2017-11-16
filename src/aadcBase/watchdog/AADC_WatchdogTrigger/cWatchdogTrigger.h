/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-10 12:58:59#$ $Rev:: 62997   $
**********************************************************************/


#ifndef _WATCHDOG_FILTER_H_
#define _WATCHDOG_FILTER_H_

#define OID_ADTF_WATCHDOGTRIGGER_FILTER "adtf.aadc.watchdogTrigger"

/*!
* @defgroup WatchdogTrigger Watchdog Trigger
* @{
* \image html WatchdogTrigger.PNG "Plugin Watchdog Trigger"
*
* The filter generates the samples of type tBoolSignalValue which can be passed to the Arduino Actuators filter to keep the Speed Controller alive. The Transmit Rate can be altered in the properties, although there is usually no need for adjustment.
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
* <b> Filter Properties</b> \n
* The filter has the following properties:
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Transmit rate in ms<td>Sets the interval between two sent watchdog triggers in msec<td>250
* </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Unit
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Media Description<th>Details
* <tr><td>WatchdogAliveSignal <td>tSignalValue from ultrasonic sensor front left <td>To be sent to Arduino Communication Filter
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcBase/watchdog/AADC_WatchdogTrigger
* <tr><td>Filename<td>aadc_watchdogTrigger.plb
* <tr><td>Version<td>1.0.0
* </table>
*/


/*! \brief cWatchdogTrigger
The filter generates the samples of type tBoolSignalValue which can be passed to the Arduino Actuators filter to keep the Speed Controller alive. The Transmit Rate can be altered in the properties but normally does not need any adjustments.
*/
class cWatchdogTrigger : public adtf::cTimeTriggeredFilter
{
    /*! declare filter version here */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_WATCHDOGTRIGGER_FILTER, "AADC Watchdog Trigger", OBJCAT_Tool, "AADC Watchdog Trigger", 1, 0, 0, "");

protected:
    /*!outputpin which sends periodically the value tTrue */
    cOutputPin        m_oOutputPin;

public:
    /*! default constructor of class
    \param __info info pointer
    */
    cWatchdogTrigger(const tChar* __info);

    /*! default destructor of class */
    virtual ~cWatchdogTrigger();

protected: // overwrites cFilter

    /*! this function is called by the class thread and calls the send and recieve of frames with the arduinos
    * \param __exception_ptr exception pointer for error handling
    * \return standard error code
    */
    tResult Cycle(ucom::IException** __exception_ptr = NULL);

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
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

private:

    /*! this function creates all the output pins of filter
    * \param __exception_ptr exception pointer for error handling
    * \return standard error code
    */
    inline tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescOutput;
    /*! the id for the bValue of the media description for the bool signal value input pins */
    tBufferID m_szIDbValue;
    /*! the id for the arduino time stamp of the media description for the bool signal value input pins */
    tBufferID m_szIDArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolSet;
};

//*************************************************************************************************

#endif // _WATCHDOG_FILTER_H_

/*!
*@}
*/
