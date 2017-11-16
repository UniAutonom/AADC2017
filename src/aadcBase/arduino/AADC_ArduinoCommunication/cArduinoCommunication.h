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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef _ARDUINO_COMMUNICATION_FILTER_HPP
#define _ARDUINO_COMMUNICATION_FILTER_HPP

#include "stdafx.h"
#include "arduino_com_client.h"

#define OID_ADTF_ARDUINOCOM_FILTER "adtf.aadc.arduinoCommunication"

#define NUM_ARDUINO 5


/*!
* @defgroup ArduinoCommunication Arduino Communication
* @{
* \image html ArduinoCommunication.PNG "Plugin Arduino Communication Filter"
*
* This filter implements the communication with all Arduinos. On a Linux host the filter searches for the Arduino devices registered as /dev/ttyACM0 up to /dev/ttyACM4. This is hard-coded and has to be adjusted if other ports want to be used. So take care that the Arduinos are always in this range. You can check them with the command ‘ ls /dev/ttyACM* ’. If they are not at the right address you can fix this by unplugging the usb hub and plug it in again.
* The input pins of the filter receive commands which are sent to the proper Arduino.
* At initialization the filter displays the connected Arduino with their Arduino ID and the software version. In total there should be 5 Arduinos recognized with the IDs 1, 3, 4, 5 and 6. You can check these IDs in the arduino_protocol.h header if you want to know which ID belongs to the which Arduino. At initialization the Arduino frames are read but not transmitted yet.
* At start the filter transmits all frames to the output pins. So they could be visualized via Sensor Visualization filter. The input pins of the filter receive commands which are sent to the proper Arduino.
* At stop the filter stops sending frames.
* At deinitialization the filter closes all arduino ports.
* The filter holds a property to log the arduino frames. If this flag is set it creates log files corresponding to the time created and the Arduino id. The directory where they are saved is the current working directory.
*
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li Boost   v.1.58.0
*
* <b> Filter Properties</b> \n
* The filter has the following properties:
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Log serial communication<td>Enable logging of arduino communication to file here<td>false
* </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Unit
* <tr><td>SpeedController <td>tSignalValue for the speed controller <td>percent [-100, 100]
* <tr><td>SteeringController <td>tSignalValue for the steering controller <td>percent [-100, 100]
* <tr><td>headLightEnabled <td>tBoolSignalValue for the headlight <td>boolean
* <tr><td>brakeLightEnabled <td>tBoolSignalValue for the brakelight <td>boolean
* <tr><td>reverseLightsEnabled <td>tBoolSignalValue for the reverse lights <td>boolean
* <tr><td>turnSignalLeftEnabled <td>tBoolSignalValue for the turn left lights  <td>boolean
* <tr><td>turnSignalRightEnabled <td>tBoolSignalValue for the turn right lights  <td>boolean
* <tr><td>hazardLightsEnabled <td>tBoolSignalValue for the hazzard lights  <td>boolean
* <tr><td>UssFrontEnable <td>tBoolSignalValue for enabling/disabling ultrasonic sensors front <td>boolean
* <tr><td>UssRearEnable <td> tBoolSignalValue for enabling/disabling ultrasonic sensors rear <td>boolean
* <tr><td>WatchdogAliveFlag <td>tBoolSignalValue setting watchdog to alive <td>boolean
* <tr><td>EmergencyStop <td>tJuryEmergencyStop for emergency stop <td>boolean
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Unit
* <tr><td>UltrasonicFrontLeft <td>tSignalValue from ultrasonic sensor front left <td>cm
* <tr><td>UltrasonicFrontCenterLeft <td>tSignalValue from ultrasonic sensor front center left <td>cm
* <tr><td>UltrasonicFrontCenterRight <td>tSignalValue from ultrasonic sensor front center right <td>cm
* <tr><td>UltrasonicFrontRight <td>tSignalValue from ultrasonic sensor front right <td>cm
* <tr><td>UltrasonicSideLeft <td>tSignalValue from ultrasonic sensor side left <td>cm
* <tr><td>UltrasonicSideRight <td>tSignalValue from ultrasonic sensor side right <td>cm
* <tr><td>UltrasonicRearCenterLeft <td>tSignalValue from ultrasonic sensor rear center left<td>cm
* <tr><td>UltrasonicRearCenter <td>tSignalValue from ultrasonic sensor rear center <td>cm
* <tr><td>UltrasonicRearCenterRight <td>tSignalValue from ultrasonic sensor rear center right <td>cm
* <tr><td>UltrasonicStruct<td>tUltrasonicStruct from all ultrasonic sensors<td>cm
* <tr><td>InerMeasUnitStruct <td>tInerMeasUnitData from intertial measurement unit <td>
* <tr><td>WheelLeftStruct <td>tWheelData from left wheel <td> counter and direction
* <tr><td>WheelRightStruct <td>tWheelData from right wheel <td> counter and direction
* <tr><td>VoltageStruct<td>tVoltageStruct voltage from all batteries<td>mV
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcBase/arduino/AADC_ArduinoCommunication
* <tr><td>Filename<td>aadc_arduinoCommunication.plb
* <tr><td>Version<td>1.0.0
* </table>
*/


///////////////////////////////////////////////////////////////////////////////
/// \brief The Dispatcher class for all arduino communication.
///
/// This class reads all arduino data and dispatches them via its output pins. So
/// the user doesnt has to care about the underlying implementation. All member data
/// should be self explonatary as walking through the code.
///
///////////////////////////////////////////////////////////////////////////////
class cArduinoCommunication : public cTimeTriggeredFilter
{
    /*! set the filter ID and the version */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ARDUINOCOM_FILTER, "AADC Arduino Communication", OBJCAT_BridgeDevice, "Arduino Communication", 1, 0, 0, "");

public:
    /*! default constructor of class
    \param __info info pointer
    */
    cArduinoCommunication(const tChar* __info);
    /*! default destructor of class */
    virtual ~cArduinoCommunication();

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
    *   \result Returns a standard result code.
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

    /*! this function is called by the class thread and calls the send and recieve of frames with the arduinos
    * \param __exception_ptr exception pointer for error handling
    * \return standard error code
    */
    tResult Cycle(ucom::IException** __exception_ptr = NULL);

private:
    /*! this function creates all the input pins of filter
    * \param __exception_ptr exception pointer for error handling
    * \return standard error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! this function creates all the output pins of filter
    * \param __exception_ptr exception pointer for error handling
    * \return standard error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

private:
    /* Output pins for all the sensors */
    /*! output pin for signal value of ultrasonic sensor front left*/
    cOutputPin  m_oOutputUsFrontLeft;
    /*! output pin for signal value of ultrasonic sensor front center left*/
    cOutputPin  m_oOutputUsFrontCenterLeft;
    /*! output pin for signal value of ultrasonic sensor front center */
    cOutputPin  m_oOutputUsFrontCenter;
    /*! output pin for signal value of ultrasonic sensor front center right */
    cOutputPin  m_oOutputUsFrontCenterRight;
    /*! output pin for signal value of ultrasonic sensor front right */
    cOutputPin  m_oOutputUsFrontRight;
    /*! output pin for signal value of ultrasonic sensor side left*/
    cOutputPin  m_oOutputUsSideLeft;
    /*! output pin for signal value of ultrasonic sensor side right*/
    cOutputPin  m_oOutputUsSideRight;
    /*! output pin for signal value of ultrasonic sensor rear center left*/
    cOutputPin  m_oOutputUsRearCenterLeft;
    /*! output pin for signal value of ultrasonic sensor rear center */
    cOutputPin  m_oOutputUsRearCenter;
    /*! output pin for signal value of ultrasonic sensor rear center right */
    cOutputPin  m_oOutputUsRearCenterRight;
    /*! output pin for composition struct of intertial measuremnt unit  */
    cOutputPin  m_oOutputInerMeasUnit;
    /*! output pin for composition struct of wheel encoder left  */
    cOutputPin  m_oOutputWheelLeft;
    /*! output pin for composition struct of wheel encoder right  */
    cOutputPin  m_oOutputWheelRight;
    /*! output pin for composition struct of all ultrasonic sensors  */
    cOutputPin      m_oOutputUsStruct;
    /*! output pin for composition struct of all battery voltages  */
    cOutputPin      m_oOutputVoltageStruct;


    /*! in this struct all the ultrasonic signals are collected and transmitted together */
    tUltrasonicStruct m_ultrasonicDataStruct;
    /*! in this struct all the voltage signals are collected and transmitted together */
    tVoltageStruct    m_voltageDataStruct;

    /*! media desctiption for writing ultrasonic data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData;
    /*! media desctiption for writing ultrasonic struct data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsDataStruct;
    /*! media desctiption for writing wheel encoder data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelData;
    /*! media desctiption for writing voltage data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionVoltDataStruct;
    /*! media desctiption for writing inertial measurement unit struct data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;

    /*! this functions does the transmission of the intertial measurement unit struct
    * \param timestamp
    * \param ax acceleration in x-axis
    * \param ay acceleration in y-axis
    * \param az acceleration in z-axis
    * \param gx gravitation in x-axis
    * \param gy gravitation in y-axis
    * \param gz gravitation in z-axis
    * \param mx rotation rates in x-axis
    * \param my rotation rates in y-axis
    * \param mz rotation rates in z-axis
    * \param roll roll angle
    * \param pitch pitch angle
    * \param yaw yaw angle
    * \return standard error code
    */
    tResult TransmitInerMeasUnitData(tUInt32 timestamp,
                                     tFloat32 ax, tFloat32 ay, tFloat32 az,
                                     tFloat32 gx, tFloat32 gy, tFloat32 gz,
                                     tFloat32 mx, tFloat32 my, tFloat32 mz,
                                     tFloat32 roll, tFloat32 pitch, tFloat32 yaw);


    /*! here we transmit the ultrasonic data for each sensor
    * \param sensorID the id of the sensor where the measure comes from
    * \param f32Distance the measured distance from sensor
    * \param ui32ArduinoTimestamp the arduino timestamp
    * \return standard error code
    */
    tResult TransmitUltrasonicData(tUInt8 sensorID, tFloat32 f32Distance, tUInt32 ui32ArduinoTimestamp);

    /*! transmits the ultrasonic data struct saved in m_ultrasonicDataStruct
    * \return standard error code
    */
    tResult TransmitUltrasonicStructData();

    /*! transmits the struct from wheel encoder
    * \param sensorID sensor id (left or right)
    * \param wheelTach tick counter
    * \param wheelDir direction flag
    * \param ui32ArduinoTimestamp arduino timestamp in micros
    * \return standard error code
    */
    tResult TransmitWheelData(tUInt8 sensorID, tUInt32 wheelTach, tUInt8 wheelDir, tUInt32 ui32ArduinoTimestamp);

    /*! transmits the voltage data struct saved in m_voltageDataStruct
    * \return standard error code
    */
    tResult TransmitVoltStructData();

    /* Input pins for all actuators */

    /*! input pin for watchdog input */
    cInputPin   m_oInputWatchdog;
    /*! input pin for emergency stop input */
    cInputPin   m_oInputEmergencyStop;
    /*! input pin for acceleration  input */
    cInputPin   m_oInputAccelerate;
    /*! input pin for steering input */
    cInputPin   m_oInputSteering;


    /*! input pin for reverse light */
    cInputPin   m_oInputReverseLight;
    /*! input pin for head light */
    cInputPin   m_oInputHeadlight;
    /*! input pin for turn left signal light */
    cInputPin   m_oInputTurnSignalLeft;
    /*! input pin for turn right signal light */
    cInputPin   m_oInputTurnSignalRight;
    /*! input pin for hazard light */
    cInputPin   m_oInputHazardLights;
    /*! input pin for brake light */
    cInputPin   m_oInputBrakeLight;


    /*! input pin for enabling/disabling front ultrasonic sensors */
    cInputPin     m_oInputUssEnableFront;
    /*! input pin for enabling/disabling rear ultrasonic sensors */
    cInputPin     m_oInputUssEnableRear;

    /*! media desctiption for reading emergency stop data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyStop;
    /*! media desctiption for reading accelerate input data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionAccelerateSignalInput;
    /*! media desctiption for reading steering input data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSteeringSignalInput;
    /*! media desctiption for bool signal input data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionLightsBoolSignalInput;
    /*! media desctiption for bool signal input data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUssEnableRearBoolSignalInput;
    /*! media desctiption for bool signal input data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUssEnableFrontBoolSignalInput;
    /*! media desctiption for bool signal input data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWatchdogBoolSignalInput;

    /*! processes the incoming emergency stop data from input pin
    * \param pMediaSample pointer to media sample with new data
    * \return standard error code
    */
    tResult ProcessEmergencyStop(IMediaSample* pMediaSample);

    /*! processes the incoming watchdog data from input pin
    * \param pMediaSample pointer to media sample with new data
    * \return standard error code
    */
    tResult ProcessWatchdog(IMediaSample* pMediaSample);

    /*! processes the actuator value data from input pin
    * \param pMediaSample pointer to media sample with new data
    * \param ui8ChID id to identifier source input pin
    * \return standard error code
    */
    tResult ProcessActuatorValue(IMediaSample* pMediaSample, tUInt8 ui8ChID);

    /*! processes the incoming lights data from input pin
    * \param pMediaSample pointer to media sample with new data
    * \param ui8LightID id to identifier source input pin
    * \return standard error code
    */
    tResult ProcessLights(IMediaSample* pMediaSample, tUInt8 ui8LightID);

    /*! processes the incoming uss control data from input pin for rear sensors
    * \param pMediaSample pointer to media sample with new data
    * \return standard error code
    */
    tResult ProcessUssControlRear(IMediaSample* pMediaSample);

    /*! processes the incoming uss control data from input pin for front sensors
    * \param pMediaSample pointer to media sample with new data
    * \return standard error code
    */
    tResult ProcessUssControlFront(IMediaSample* pMediaSample);


private:
    /*! if logging to file is enabled*/
    tBool                               m_bLoggingModeEnabled;
    /*! the critical section for serial communication*/
    cCriticalSection                    m_oCritSectionInputData;
    /*! the clients for the arduino communictation */
    arduino_com_client                  m_com[NUM_ARDUINO];
    /*! the list of the sucessfully opened arduinos*/
    std::vector<int>                    m_valid_ids;
};

#endif


/*!
*@}
*/
