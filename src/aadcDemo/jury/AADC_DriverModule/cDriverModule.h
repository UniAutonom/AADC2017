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


#ifndef _DRIVERMODULE_HEADER
#define _DRIVERMODULE_HEADER

#define __guid "adtf.aadc.driverModule"

#include "stdafx.h"

#include "aadc_juryEnums.h"

#include "displaywidget.h"

/*! @defgroup DriverModule Driver Module
*  @{
*
* This filter was developed as a prototype to explain the interaction with the Jury Module.
* It receives the structs from the Jury module and shows them in the middle of the gui. The user can respond the the received messages with the four buttons “Ready to Start”, “Running”, “Error” and “Complete”.
* After clicking on the button the sample is transmitted on the output pin Driver_Struct an contains the following struct:
* \n typedef struct
* \n {
* \n <span style="padding-left: 2em">tInt8 i8StateID;</span>
* \n <span style="padding-left: 2em">tInt16 i16ManeuverEntry;</span>
* \n } tDriverStruct;
* \n
* \n The possible i8StateID are:
* \li stateCar_Error: This is sent if some error occurred on the car.
* \li stateCar_Ready: If the car is ready to start a maneuver ID this state is sent including the maneuver ID in i16ManeuverEntry.
* \li stateCar_Running: Sent during running the maneuver contained in i16ManeuverEntry
* \li stateCar_Complete: Sent if the car finished the whole maneuver list.
* \li stateCar_Startup: Sent at the initial phase to indicate that car is working properly
*
* The struct tDriverStruct is defined in aadc_structs.h in src/aadcBase/include and the used enums are defined in juryEnums.h in src/aadcBase/include
*
* The teams must not implement any filter containing a Qt GUI because there is no opportunity to control the car with a GUI in the competition. The only way to interact with the car is through the jury module which is controlled by the jury.
*
*  \image html DriverModule.PNG "Plugin Driver Module"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT  v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Debug Output to Console<td><td>False
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Driver_Struct<td><td>tDriverStruct
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>Jury_Struct<td><td>tJuryStruct
* <tr><td>Maneuver_List<td><td>tManeuverList
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/jury/AADC_DriverModule
* <tr><td>Filename<td>aadc_driverModule.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this is the main class of the driver module plugin */
class cDriverModule : public QObject, public cBaseQtFilter
{

    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Driver Module", OBJCAT_Tool, "Driver Module", 1, 1, 0, "");

    Q_OBJECT

public: // construction

    /*! constructor for filter class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cDriverModule(const tChar* __info);

    /*! Destructor. */
    virtual ~cDriverModule();

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

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

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
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

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

    /*!
    *  The adtf::IRunnable interface declares an object as a callback-entry point.
    *  Overwrite these function when handling system events, thread calls or other
    *  activation calls. Have a look to adtf::IRunnable and  adtf::IKernel.
    *  \n
    *  \n For receiving System Events use within your overwritten cFilter::Init function:
    *  \n
    *  \code _kernel->SignalRegister(static_cast<IRunnable*>(this)); \endcode
    *  \n Your nActivationCode for the Run call will be IRunnable::RUN_SIGNAL. The
    *     pvUserData will be a pointer to adtf::tEventInfo.
    *  \warning IMPORTANT: When registered for signals, never forget to unregister:
    *  \code _kernel->SignalUnregister(static_cast<IRunnable*>(this)); \endcode
    *
    * \param [in] nActivationCode The activation type for running.
    * \param [in] pvUserData pointer to a activation structure depending on the activation type.
    * \param [in] szUserDataSize Size of the activation structure. (in byte)
    * \param [in,out] __exception_ptr Address of variable that points to an IException interface.
    *                                    If not using the cException smart pointer, the interface has to
    *                                    be released by calling Unref()..
    *
    * \result Returns a standard result code.
    */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

protected: // Implement cBaseQtFilter

    /*! Creates the widget instance
    * \result Returns a standard result code.
    */
    tHandle CreateView();
    /*! Destroys the widget instance
    * \result Returns a standard result code.
    */
    tResult ReleaseView();

public slots:
    /*! function which transmits the state
    * \param stateID state to be sent; -1: error, 0: Ready, 1: Running
    * \param i16ManeuverEntry current entry to be sent
    */
    tResult OnSendState(stateCar stateID, tInt16 i16ManeuverEntry);

    /*! this functions loads the maneuver list given in the properties
    * \result Returns a standard result code.
    */
    tResult LoadManeuverList();

signals:
    /*! signal to the gui to show the command "run" from the jury
    * \param entryId current entry to be sent
    */
    void SendRun(int entryId);

    /*! signal to the gui to show the command "stop" from the jury
    * \param entryId current entry to be sent
    */
    void SendStop(int entryId);

    /*! signal to the gui to show the command "request ready" from the jury
    * \param entryId current entry to be sent
    */
    void SendRequestReady(int entryId);

    /*! Trigger to load maneuver list. */
    void TriggerLoadManeuverList();
private:

    /*! The displayed widget*/
    DisplayWidgetDriver *m_pWidget;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescJuryStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescDriverStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID;
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    /*! Coder description */
    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;


    /*! input pin for the run command*/
    cInputPin        m_JuryStructInputPin;
    /*! input pin for the maneuver list*/
    cInputPin        m_ManeuverListInputPin;
    /*! output pin for state from driver*/
    cOutputPin        m_DriverStructOutputPin;

    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;

    /*! The maneuver file string */
    cString     m_strManeuverFileString;
    /*! this is the filename of the maneuver list*/
    cFilename m_maneuverListFile;
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

};

#endif /** @} */ // end of group
