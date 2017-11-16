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
#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#define OID_ADTF_TEMPLATE_FILTER "adtf.example.stateMachine"

#include "aadc_juryEnums.h"
#include "ManeuverList.h"
#include "cStates_Enum.h"
#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)
#define DEFAULT_SPEED (-8)
#define DETECTION_THRESHOLD 0.7

/*! @defgroup TemplateFilter
*  @{
*
*  \image html User_Template.PNG "Plugin Template Filter"
*
* This is a small template which can be used by the AADC teams for their own filter implementations.
* \b Dependencies \n
* This plugin needs the following libraries:
*
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>output_template<td>An example output pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>input_template<td>An example input pin<td>MEDIA_TYPE_TEMPLATE<td>MEDIA_TYPE_TEMPLATE
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcUser/AADC_TemplateFilter
* <tr><td>Filename<td>user_templateFilter.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*
*/

typedef struct _pos
{
    tFloat32 f32X;
    tFloat32 f32Y;
    tFloat32 f32Radius;
    tFloat32 f32Speed;
    tFloat32 f32Heading;
} pos;

typedef struct _roadSign
{
    /*! road sign */
    tInt16 u16Id;

    /*! location */
    tFloat32 f32X;
    tFloat32 f32Y;

    /*! sign search radius */
    tFloat32 f32Radius;

    /*! direction (heading) of the road sign */
    tFloat32 f32Direction;

    tInt u16Cnt;

    tTimeStamp u32ticks;/*! measurement ticks*/

} roadSign;

typedef struct _parkingSpace
{
    /*! road sign */
    tInt16 u16Id;

    /*! location */
    tFloat32 f32X;
    tFloat32 f32Y;

    /*! sign search radius */
    tInt32 f32Status;

    /*! direction (heading) of the road sign */
    tFloat32 f32Direction;
} parkingSpace;

//!  Template filter for AADC Teams
/*!
* This is a example filter for the AADC
*/
class cStateMachine : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "State Machine", adtf::OBJCAT_DataFilter);

private:
    // input pin for the run command
    cInputPin m_JuryStructInputPin;  //typ tJuryStruct
    tBufferID m_szIDJuryStructI8ActionID;
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    tBool     m_bIDsJuryStructSet;

    // output pin for state from driver
    cOutputPin m_DriverStructOutputPin;  //typ tDriverStruct
    tBufferID  m_szIDDriverStructI8StateID;
    tBufferID  m_szIDDriverStructI16ManeuverEntry;
    tBool      m_bIDsDriverStructSet;

    // output pin for actual state of this state machine
    cOutputPin m_ActualStateOutputPin;  //typ tStatesStruct
    tBufferID  m_szIDStatesStructI8PrimaryState;
    tBufferID  m_szIDStatesStructI8RunState;
    tBool      m_bIDsStatesStructSet;

    // output pin for maneuvers
    cOutputPin m_ManeuverOutputPin;  //typ tManeuverValues
    tBufferID  m_szManeuverIDOutput;
    tBufferID  m_szParkingSpaceIDOutput;
    tBool      m_bIDManeuverSendSet;

    //  input pin for the emergency break status
    cInputPin m_EmergencyBreakStatusInputPin; //typ tJuryEmergencyStop
    tBufferID m_szIdEmergencyStopValue;
    tBool     m_szIdEmergencyStopSet;

    //  input pin for the emergency break status
    cInputPin m_EmergencyStopInputPin; //typ tJuryEmergencyStop

    cInputPin m_SteeringAngleInputPin; //typ tSignalValue

    cOutputPin  m_oOutputEmergencyBreakSet;  //typ tEmergencyBreakSet
    tBufferID   m_szIdEmergencyBreakSetFrontLeftValue;
    tBufferID   m_szIdEmergencyBreakSetFrontHalfLeftValue;
    tBufferID   m_szIdEmergencyBreakSetFrontMiddleValue;
    tBufferID   m_szIdEmergencyBreakSetFrontHalfRightValue;
    tBufferID   m_szIdEmergencyBreakSetFrontRightValue;
    tBufferID   m_szIdEmergencyBreakSetSideLeftValue;
    tBufferID   m_szIdEmergencyBreakSetSideRightValue;
    tBufferID   m_szIdEmergencyBreakSetBackLeftValue;
    tBufferID   m_szIdEmergencyBreakSetBackMiddleValue;
    tBufferID   m_szIdEmergencyBreakSetBackRightValue;
    tBool       m_szIdEmergencyBreakSet;

    cOutputPin m_oOutputHeadLight;    // The output pin for head light
    cOutputPin m_oOutputReverseLight; // The output pin for reverse light
    cOutputPin m_oOutputBrakeLight;   // The output pin for brake light
    cOutputPin m_oOutputHazzardLight; // The output pin for hazzard light
    cOutputPin m_oOutputTurnRight;    // The output pin for turn right controller
    cOutputPin m_oOutputTurnLeft;     // The output pin for turn left controller
    cInputPin m_oInputTurnRight;    // The input pin for turn right controller
    cInputPin m_oInputTurnLeft;     // The input pin for turn left controller

    cInputPin m_oInputRoadSignExt;   // Input pin for the road sign Ext data
    tBufferID m_szIDRoadSignExtI16Identifier;
    tBufferID m_szIDRoadSignExtF32Imagesize;
    tBufferID m_szIDRoadSignExtAf32TVec;
    tBufferID m_szIDRoadSignExtAf32RVec;
    tBool     m_bIDsRoadSignExtSet;

    cInputPin m_oInputClassification;  // input pin for the classification result

    cInputPin    m_oInputWheelLeft;
    cInputPin    m_oInputWheelRight;

    //Position Output to backend
    cOutputPin m_OutputPostion;
    tBufferID m_szF32X,m_szF32Y,m_szF32Radius,m_szF32Speed,m_szF32Heading;
    tBool m_PosOutputSet;

    //Trafficsign Output to backend
    cOutputPin m_OutputTrafficSign;
    tBufferID m_tsI16id,m_tsF32X,m_tsF32Y,m_tsF32Angle;
    tBool m_TrafficSignOutputSet;

    //Obstacle Output to backend
    cOutputPin m_OutputObstacle;
    tBufferID m_obstacleF32X,m_obstacleF32Y;
    tBool m_ObstacleOutputSet;

    //Parking Output to backend
    cOutputPin m_OutputParkingSpace;
    tBufferID m_parkingI16Id,m_parkingF32X,m_parkingF32Y,m_parkingUI16Status;
    tBool m_ParkingOutputSet;

    // Input pin for finished maneuvers
    cInputPin  m_oInputManeuverFinished;
    tBufferID  m_szManeuverIDInput;
    tBufferID  m_szFinishedManeuverInput;
    tBool      m_bIDsManeuverFinishedSet;

    // input pin for speed
    cInputPin   m_oInputSpeedController;   //typ tSignalValue
    tBufferID   m_szIdInputspeedControllerValue;
    tBufferID   m_szIdInputspeedControllerTimeStamp;
    tBool       m_szIdInputSpeedSet;

    // output pin for speed
    cOutputPin  m_oOutputSpeedController;  //typ tSignalValue
    tBufferID   m_szIdOutputspeedControllerValue;
    tBufferID   m_szIdOutputspeedControllerTimeStamp;
    tBool       m_szIdOutputSpeedSet;

    cInputPin              m_oInputUsStruct; //tUltrasonicStruct
    std::vector<tBufferID> m_szIdUsStructValues;
    std::vector<tBufferID> m_szIdUsStructTimeStamps;
    tBool                  m_szIdsUsStructSet;

    // Input pin for ticks to line
    cInputPin  m_oInputTicksToLine;

    /*! currently processed road-sign */
    tInt16 m_i16ID;
    tFloat32 m_f32MarkerSize;
    Mat m_Tvec; /*! translation vector */
    Mat m_Rvec; /*! rotation vector */

    tInt m_ui32Cnt;

    // Position input
    cInputPin  m_oInputPinPosition;        // Input pin for the road sign position
    tBool m_PosInputSet;

    //  input pin for the maneuver list
    cInputPin            m_ManeuverListInputPin;
    cString              m_strManeuverFileString; // The maneuver file string
    cFilename            m_maneuverListFile;      // this is the filename of the maneuver list
    std::vector<tSector> m_sectorList;            // this is the list with all the loaded sections(containing the single maneuvers) from the maneuver list

    bool m_hasSentActualManeuver;

    vector<roadSign>     m_roadSigns;  // storage for the roadsign data
    vector<parkingSpace> m_parkingSpaces;

    tFloat32 m_actualSpeedState;
    tFloat32 m_actualSpeedLaneDetection;
    tFloat32 m_actualSpeedCarDetection;
    tFloat32 m_actualSpeedChildDetection;
    tFloat32 m_actualSpeedAdultDetection;
    tFloat32 m_actualSpeedTrafficSignDetection;

    tInt32   m_lastTicksAdultDetected;
    tInt32   m_lastTicksCarDetected;
    tInt32   m_lastTicksChildDetected;
    tInt32   m_lastTicksStopSignDetected;
    tInt32   m_lastTicksGiveWaySignDetected;
    tInt32   m_lastTicksHaveWaySignDetected;
    tInt32   m_lastTicksPedestrianSignDetected;

    tInt64   m_lastTimeStopp;
    tInt64   m_lastTimeCarDetected;
    tInt64   m_EmergencyBreakSince;
    tInt64   m_initTime;

    bool m_emergencyBreakStatus;
    bool m_emergencyBreakStatus_changed;

    int m_HeadLightOn = 0;
    bool m_HeadLightOn_changed = false;
    int m_BrakeLightOn = 0;
    bool m_BrakeLightOn_changed = false;
    int m_HazzardLightOn = 0;
    bool m_HazzardLightOn_changed = false;
    int m_ReverseLightOn = 0;
    bool m_ReverseLightOn_changed = false;
    int m_TurnLeftSignalOn = 0;
    bool m_TurnLeftSignalOn_changed = false;
    int m_TurnRightSignalOn = 0;
    bool m_TurnRightSignalOn_changed = false;

    pos m_ActualPosition;
    bool m_pos_Initialized;
    bool m_Emergency_Stop_Jury;

    tFloat32 m_actualDistances[10];

    tFloat32 m_f32CameraOffsetLat;
    tFloat32 m_f32CameraOffsetLon;
    Mat m_state; /*! filter state {X} */
    Mat m_errorCov; /*! error covariance matrix {P} */
    Mat m_processCov; /*! process covariance matrix {Q} */
    Mat m_transitionMatrix; /*! state transition matrix {F} */

    //mediatype descriptions
    cObjectPtr<IMediaTypeDescription> m_pDescJuryStruct;
    cObjectPtr<IMediaTypeDescription> m_pDescDriverStruct;
    cObjectPtr<IMediaTypeDescription> m_pDescStatesStruct;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyStop;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyBreakSet;
    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionManeuver;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;
    cObjectPtr<IMediaTypeDescription> m_pDescPosition;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionPos;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSign;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionObstacle;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionParkingSpace;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionManeuverFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelLeftData;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelRightData;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTicksToLine;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;

    primaryStates m_primaryState;     // the primary state of this state machince
    runStates     m_runState;         // the run state of this state machince
    int           m_actualSectorID;   // the id of the actual sector
    int           m_actualManeuverID; // the id of the actual maneuver

    int wheelCountLeft;
    int wheelCountRight;
    int m_actualWheelTicks;
    int m_ticksOfNextLine;

    cCriticalSection m_critSecTransmitBool; // The critical section transmit bool
    cCriticalSection m_critSecTransmitManeuver; // The critical section transmit bool
    cCriticalSection m_critSecTransmitControl;
    cCriticalSection m_critSecMinimumUsValue;

public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cStateMachine(const tChar* __info);

    /*! default destructor */
    virtual ~cStateMachine();

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
    *   \return Returns a standard result code.
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

    /*!
     * \brief sends a DriverStruct to the jury module
     * \param stateID  the id of the actual state  (see <stateCar>)
     * \param i16ManeuverEntry  the actual maneuver entry
     * \return  Returns a standard result code.
     */
    tResult SendState(stateCar stateID, tInt16 i16ManeuverEntry);

    tResult LoadRoadSignConfiguration();

    /*! this function loads the maneuver list given in the properties
    * \result Returns a standard result code.
    */
    tResult LoadManeuverList();

    cString getActualManeuver();

    tResult setActualManeuverCompleted();

    tResult TransmitEmergencyBreakSet();

    tResult TransmitBoolValue(cOutputPin* pin, tBool value, tUInt32 timestamp);

    tResult TransmitManeuver(tInt16 maneuverNumber, tInt8 parkingSpaceID);

    tResult TransmitPosition(IMediaSample* pMediaSample);

    tResult TransmitTrafficSign(tInt16 i16Id, tFloat32 f32x, tFloat32 f32y, tFloat32 f32angle);

    tResult TransmitObstacle(tFloat32 f32x, tFloat32 f32y);

    tResult TransmitParkingSpace(tInt16 i16Id, tFloat32 f32x, tFloat32 f32y, tUInt16 ui16Status);

    tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);

    tResult updateSpeed();

    tStatesStruct ChangeState(bool emergencyBreakSet, bool start, bool ready, bool stop, bool maneuverCompleted);

    tResult ChangePrimaryState(primaryStates newPrimaryState);

    tResult ChangeRunState(runStates newRunState);

    tResult ProcessJuryInput(IMediaSample* pMediaSample);

    tResult ProcessManeuverList(IMediaSample* pMediaSample);

    tResult ProcessEmergencyBreakStatus(IMediaSample* pMediaSample);

    tResult ProcessEmergencyStop(IMediaSample* pMediaSample);

    tResult ProcessSpeedController(IMediaSample* pMediaSample);

    tResult ProcessUsValues(IMediaSample* pMediaSample);

    tResult ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime);

    tResult ProcessTicksToLine(IMediaSample* pMediaSample);

    tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);

    tFloat32 mod(tFloat32 x, tFloat32 y);

    tResult ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn);

    tResult ProcessFinishedManeuver(IMediaSample* pMediaSample);

    tResult ProcessClassificationInput(IMediaSample* pMediaSample);

    tResult ProcessWheelSampleLeft(IMediaSample* pMediaSample);

    tResult ProcessWheelSampleRight(IMediaSample* pMediaSample);

    tResult checkActualManeuver();

    tResult checkTickAndTimeStemps();

    tResult ComputeNextStep();

    tResult updateLights();

    tResult SetHazzardLight(bool status);

    tResult SetBrakeLight(bool status);
};

//*************************************************************************************************
#endif // _STATE_MACHINE_H_

/*!
*@}
*/
