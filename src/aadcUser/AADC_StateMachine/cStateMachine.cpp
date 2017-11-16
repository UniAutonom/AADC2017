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

#include "stdafx.h"
#include "cStateMachine.h"
#include "aadc_myclassification_structs.h"
#include "aadc_roadSign_enums.h"
#include "aadc_enums.h"
using namespace Unia;
using namespace roadsignIDs;

#define MP_PROP_CAMERA_OFFSET_LAT "Camera Offset::Lateral"
#define MP_PROP_CAMERA_OFFSET_LON "Camera Offset::Longitudinal"
#define SPEEDUP_FACTOR "SpeedupFactor"
#define OVERTAKING_TRESHOLD "Overtaking_Treshold"

// road sign distance and pose estimation
#define MP_LIMIT_ALPHA    70.0 // [degrees]
#define MP_LIMIT_YAW      15.0 // [degrees]
#define MP_LIMIT_YAW_INIT  8.0 // [degrees]
#define MP_LIMIT_DISTANCE  0.8 // [m]

/// Create filter shell
ADTF_FILTER_PLUGIN("State Machine", OID_ADTF_FILTER_DEF, cStateMachine)

cStateMachine::cStateMachine(const tChar* __info):cFilter(__info)
{
    SetPropertyStr("Configuration","roadSign.xml");
    SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT, 0.05);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_DESCRIPTION, "Camera offset in lateral direction");

    SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON, 0.0);
    SetPropertyBool(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_DESCRIPTION, "Camera offset in longitudinal direction");

    SetPropertyFloat(SPEEDUP_FACTOR, 1.0);
    SetPropertyBool(SPEEDUP_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SPEEDUP_FACTOR NSSUBPROP_DESCRIPTION, "Speedupfactor");

    SetPropertyInt(OVERTAKING_TRESHOLD, 5000000);
    SetPropertyBool(OVERTAKING_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(OVERTAKING_TRESHOLD NSSUBPROP_DESCRIPTION, "OvertakingTreshold");
}

cStateMachine::~cStateMachine()
{
}


tResult cStateMachine::PropertyChanged(const tChar* strName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));

    //associate the properties to the member
    if (cString::IsEqual(strName, SPEEDUP_FACTOR))
    {
        m_maxSpeedUpFactor = GetPropertyFloat(SPEEDUP_FACTOR);
        if(m_actualSpeedUpFactor != 1.0){
            m_actualSpeedUpFactor = m_maxSpeedUpFactor;
            m_actualSpeed_changed = true;
            m_runState_changed = true;
        }
    }
    if (cString::IsEqual(strName, OVERTAKING_TRESHOLD))
    {
        m_overtakingTreshold = GetPropertyInt(OVERTAKING_TRESHOLD);
    }

    RETURN_NOERROR;
}

tResult cStateMachine::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // input jury struct
        tChar const * strDescJuryStruct = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strDescJuryStruct);
        cObjectPtr<IMediaType> pTypeJuryStruct = new cMediaType(0, 0, 0, "tJuryStruct", strDescJuryStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pTypeJuryStruct, this));
        RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
        RETURN_IF_FAILED(pTypeJuryStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionJuryStruct));

        // input maneuver list
        tChar const * strDescManeuverList = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strDescManeuverList);
        cObjectPtr<IMediaType> pTypeManeuverList = new cMediaType(0, 0, 0, "tManeuverList", strDescManeuverList, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pTypeManeuverList, this));
        RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
        RETURN_IF_FAILED(pTypeManeuverList->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuverList));

        // output driver struct
        tChar const * strDescDriverStruct = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDescDriverStruct);
        cObjectPtr<IMediaType> pTypeDriverStruct = new cMediaType(0, 0, 0, "tDriverStruct", strDescDriverStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pTypeDriverStruct, this));
        RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
        RETURN_IF_FAILED(pTypeDriverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDriverStruct));

        //input Emergency stop
        tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
        RETURN_IF_POINTER_NULL(strDescEmergencyStop);
        cObjectPtr<IMediaType> pTypeEmergencyStatus = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_EmergencyStopInputPin.Create("EmergencyStop", pTypeEmergencyStatus, this));
        RETURN_IF_FAILED(RegisterPin(&m_EmergencyStopInputPin));
        RETURN_IF_FAILED(pTypeEmergencyStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

        // input EmergencyBreak status
        RETURN_IF_FAILED(m_EmergencyBreakStatusInputPin.Create("EmergencyBreakStatus", pTypeEmergencyStatus, this));
        RETURN_IF_FAILED(RegisterPin(&m_EmergencyBreakStatusInputPin));
        RETURN_IF_FAILED(pTypeEmergencyStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

        // output EmergencyBreak set
        tChar const * strDescEmergencyBreakSet = pDescManager->GetMediaDescription("tEmergencyBreakSet");
        RETURN_IF_POINTER_NULL(strDescEmergencyBreakSet);
        cObjectPtr<IMediaType> pTypeEmergencyBreakSet = new cMediaType(0, 0, 0, "tEmergencyBreakSet", strDescEmergencyBreakSet, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oOutputEmergencyBreakSet.Create("EmergencyBreakSet", pTypeEmergencyBreakSet, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputEmergencyBreakSet));
        RETURN_IF_FAILED(pTypeEmergencyBreakSet->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyBreakSet));

        // create the road sign pin
        tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");
        RETURN_IF_POINTER_NULL(strDescExt);
        cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputRoadSignExt.Create("RoadSign_ext", pTypeExt, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSignExt));
        RETURN_IF_FAILED(RegisterPin(&m_oInputRoadSignExt));

        // create the the position pin
        tChar const * strDescPosition = pDescManager->GetMediaDescription("tPosition");
        RETURN_IF_POINTER_NULL(strDescPosition);
        cObjectPtr<IMediaType> pTypePosition = new cMediaType(0, 0, 0, "tPosition", strDescPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputPinPosition.Create("Position_in", pTypePosition, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputPinPosition));
        RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));

        //create and register pins for speed in
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
        RETURN_IF_FAILED(m_oInputSpeedController.Create("Speed_in", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        //create and register pins for speed out
        RETURN_IF_FAILED(m_oOutputSpeedController.Create("Speed_out", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

        // triggering the lights
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
        RETURN_IF_FAILED(m_oOutputHeadLight.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputHeadLight));
        RETURN_IF_FAILED(m_oOutputBrakeLight.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));
        RETURN_IF_FAILED(m_oOutputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputReverseLight));
        RETURN_IF_FAILED(m_oOutputHazzardLight.Create("hazzardLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputHazzardLight));
        RETURN_IF_FAILED(m_oOutputTurnLeft.Create("turnSignalLeftEnabled_Out", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft));
        RETURN_IF_FAILED(m_oOutputTurnRight.Create("turnSignalRightEnabled_Out", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight));
        RETURN_IF_FAILED(m_oInputTurnLeft.Create("turnSignalLeftEnabled_In", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTurnLeft));
        RETURN_IF_FAILED(m_oInputTurnRight.Create("turnSignalRightEnabled_In", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTurnRight));

        // create input pin for classfication input
        RETURN_IF_FAILED(m_oInputClassification.Create("classification", new cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputClassification));

        // output position to backend
        tChar const * strDescPos = pDescManager->GetMediaDescription("tPosition");
        RETURN_IF_POINTER_NULL(strDescPos);
        cObjectPtr<IMediaType> pTypePos = new cMediaType(0, 0, 0, "tPosition", strDescPos, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypePos->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionPos));
        RETURN_IF_FAILED(m_OutputPostion.Create("Position", pTypePos, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_OutputPostion));

        // output traffic sign to backend
        tChar const * strDescTrafficSign = pDescManager->GetMediaDescription("tTrafficSign");
        RETURN_IF_POINTER_NULL(strDescTrafficSign);
        cObjectPtr<IMediaType> pTypeTrafficSign = new cMediaType(0, 0, 0, "tTrafficSign", strDescTrafficSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeTrafficSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTrafficSign));
        RETURN_IF_FAILED(m_OutputTrafficSign.Create("TrafficSign", pTypeTrafficSign, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_OutputTrafficSign));

        // output obstacle to backend
        tChar const * strDescObstacle = pDescManager->GetMediaDescription("tObstacle");
        RETURN_IF_POINTER_NULL(strDescObstacle);
        cObjectPtr<IMediaType> pTypeObstacle = new cMediaType(0, 0, 0, "tObstacle", strDescObstacle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeObstacle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionObstacle));
        RETURN_IF_FAILED(m_OutputObstacle.Create("Obstacle", pTypeObstacle, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_OutputObstacle));

        // output parking space to backend
        tChar const * strDescParkingSpace = pDescManager->GetMediaDescription("tParkingSpace");
        RETURN_IF_POINTER_NULL(strDescParkingSpace);
        cObjectPtr<IMediaType> pTypeParkingSpace = new cMediaType(0, 0, 0, "tParkingSpace", strDescParkingSpace, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeParkingSpace->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParkingSpace));
        RETURN_IF_FAILED(m_OutputParkingSpace.Create("ParkingSpace", pTypeParkingSpace, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_OutputParkingSpace));

        // output maneuvers to lane detection
        tChar const * strDescManeuvers = pDescManager->GetMediaDescription("tManeuverValues");
        RETURN_IF_POINTER_NULL(strDescManeuvers);
        cObjectPtr<IMediaType> pTypeManeuvers = new cMediaType(0, 0, 0, "tManeuverValues", strDescManeuvers, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_ManeuverOutputPin.Create("SendManeuvers", pTypeManeuvers, this));
        RETURN_IF_FAILED(RegisterPin(&m_ManeuverOutputPin));
        RETURN_IF_FAILED(pTypeManeuvers->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuver));

        // input finished maneuvers
        tChar const * strDescFinishedManeuvers = pDescManager->GetMediaDescription("tManeuverFinished");
        RETURN_IF_POINTER_NULL(strDescFinishedManeuvers);
        cObjectPtr<IMediaType> pTypeFinishedManeuvers = new cMediaType(0, 0, 0, "tManeuverFinished", strDescFinishedManeuvers, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputManeuverFinished.Create("FinishedManeuvers", pTypeFinishedManeuvers, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputManeuverFinished));
        RETURN_IF_FAILED(pTypeFinishedManeuvers->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuverFinished));

        // input ticks to line
        tChar const * strTicksToLine = pDescManager->GetMediaDescription("tTicksToLine");
        RETURN_IF_POINTER_NULL(strTicksToLine);
        cObjectPtr<IMediaType> pTypeTicksToLine = new cMediaType(0, 0, 0, "tTicksToLine", strTicksToLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputTicksToLine.Create("Ticks_to_line", pTypeTicksToLine, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTicksToLine));
        RETURN_IF_FAILED(pTypeTicksToLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTicksToCrosspoint));

        //Wheel Tick Input
        tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
        RETURN_IF_POINTER_NULL(strDescWheelData);
        cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelLeftData));
        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelRightData));
        RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));
        RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

        // input ultrasonic
        tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
        RETURN_IF_POINTER_NULL(strDescUsStruct);
        cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));
        RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));

        //input parkingspaces status
        tChar const * strParkingspaces = pDescManager->GetMediaDescription("tParkingSpaces");
        RETURN_IF_POINTER_NULL(strParkingspaces);
        cObjectPtr<IMediaType> pTypeParkingSpaces = new cMediaType(0, 0, 0, "tParkingSpaces", strParkingspaces, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputParkingspaces.Create("Parkingspaces_In", pTypeParkingSpaces, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputParkingspaces));
        RETURN_IF_FAILED(pTypeParkingSpaces->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParkingSpacesStatus));
    }
    else if (eStage == StageNormal)
    {
        m_bIDsJuryStructSet       = tFalse;
        m_bIDsDriverStructSet     = tFalse;
        m_bIDsStatesStructSet     = tFalse;
        m_szIdEmergencyStopSet    = tFalse;
        m_szIdEmergencyBreakSet   = tFalse;
        m_bIDsRoadSignExtSet      = tFalse;
        m_bIDManeuverSendSet      = tFalse;
        m_PosOutputSet            = tFalse;
        m_TrafficSignOutputSet    = tFalse;
        m_ParkingOutputSet        = tFalse;
        m_ObstacleOutputSet       = tFalse;
        m_bIDsManeuverFinishedSet = tFalse;
        m_szIdInputSpeedSet       = tFalse;
        m_szIdOutputSpeedSet      = tFalse;
        m_szIdsUsStructSet        = tFalse;

        // default / initial values

        //states
        m_primaryState = primaryState_run;
        m_runState = runState_stop;
        m_primaryState_changed = false;
        m_runState_changed = false;

        // jury maneuver
        m_actualSectorID = -1;
        m_actualManeuverID = -1;

        // speed control
        m_actualSpeedState = 0;
        m_actualSpeedLaneDetection = NO_LD_SPEED;
        m_actualSpeedCarDetection = DEFAULT_SPEED;
        m_actualSpeedChildDetection = DEFAULT_SPEED;
        m_actualSpeedAdultDetection = DEFAULT_SPEED;
        m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = false;
        m_actualSpeedUpFactor = 1.0;
        m_maxSpeedUpFactor = 1.0;

        // emergency break
        m_emergencyBreakStatus = false;
        m_emergencyBreakStatus_changed = false;
        m_Emergency_Stop_Jury = false;
        m_EmergencyBreakSince = INT32_MIN;

        // light control
        m_HeadLightOn = 0;
        m_HeadLightOn_changed = false;
        m_BrakeLightOn = 0;
        m_BrakeLightOn_changed = false;
        m_HazzardLightOn = 0;
        m_HazzardLightOn_changed = false;
        m_ReverseLightOn = 0;
        m_ReverseLightOn_changed = false;
        m_TurnLeftSignalOn = 0;
        m_TurnLeftSignalOn_changed = false;
        m_TurnRightSignalOn = 0;
        m_TurnRightSignalOn_changed = false;

        // tick stamps
        m_actualWheelTicks = 0;
        m_lastTicksRelevantSignDetected = INT32_MAX;
        m_lastTicksAdultDetected = INT32_MAX;
        m_lastTimeFollowCarDetected = INT32_MAX;
        m_lastTicksChildDetected = INT32_MAX;
        m_lastTicksGiveWaySignDetected = INT32_MAX;
        m_lastTicksHaveWaySignDetected = INT32_MAX;
        m_lastTicksPedestrianSignDetected = INT32_MAX;
        m_lastTicksStopSignDetected = INT32_MAX;
        m_ticksOfNextCrosspoint = INT32_MAX;
        m_lastCarCounter = 0;

        // time stamps
        m_lastTimeCarDetected = 0;
        m_lastTimeStopp = INT32_MAX;

        // positioning
        m_ActualPosition.f32X = 0;
        m_ActualPosition.f32Y = 0;
        m_ActualPosition.f32Heading = 0;
        m_ActualPosition.f32Radius = 0;
        m_ActualPosition.f32Speed = 0;
        m_pos_Initialized = false;

        // lane detection maneuver communication
        m_hasSentActualManeuver = true;

        // initialize translation and rotation vectors
        m_state = Mat(6,1,CV_64F,Scalar::all(0));
        m_Tvec = Mat(3,1,CV_32F,Scalar::all(0));
        m_Rvec = Mat(3,1,CV_32F,Scalar::all(0));

        transmitParkingSpaceSearch = true;

        m_maxSpeedUpFactor = GetPropertyFloat(SPEEDUP_FACTOR);
        m_overtakingTreshold = GetPropertyInt(OVERTAKING_TRESHOLD);
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.

        // simulate ready signal from jury
        //ChangeState(false, false, true, false, false);

        // simulate start signal from jury
        //ChangeState(false, true, false, false, false);

        m_actualSpeedUpFactor = 1.0;

        ResetAllLights();

        TransmitEmergencyBreakSet();    // set initial E.B. distances

        LoadRoadSignConfiguration();    // load road sign config
    }
    RETURN_NOERROR;
}

tResult cStateMachine::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cStateMachine::Stop(__exception)
{
    // call the base class implementatistrDescSignalValueon
    return cFilter::Stop(__exception_ptr);
}

tResult cStateMachine::OnPinEvent(IPin* pSource,
                                  tInt nEventCode,
                                  tInt nParam1,
                                  tInt nParam2,
                                  IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {
        if (pSource == &m_JuryStructInputPin && m_pDescriptionJuryStruct != NULL)
        {
            ProcessJuryInput(pMediaSample);
        }
        else if (pSource == &m_ManeuverListInputPin && m_pDescriptionManeuverList != NULL)
        {
            ProcessManeuverList(pMediaSample);
        }
        else if (pSource == &m_EmergencyStopInputPin && m_pDescriptionEmergencyStop != NULL)
        {
            ProcessEmergencyStop(pMediaSample);
        }
        else if (pSource == &m_EmergencyBreakStatusInputPin && m_pDescriptionEmergencyStop != NULL)
        {
            ProcessEmergencyBreakStatus(pMediaSample);
        }
        else if (pSource == &m_oInputClassification)
        {
            ProcessClassificationInput(pMediaSample);
        }
        else if (pSource == &m_oInputRoadSignExt && m_pDescriptionRoadSignExt != NULL)
        {
            if(m_runState != runState_stop)
                ProcessRoadSignStructExt(pMediaSample);
        }
        else if (pSource == &m_oInputManeuverFinished && m_pDescriptionManeuverFinished != NULL)
        {
            ProcessFinishedManeuver(pMediaSample);
        }
        else if (pSource == &m_oInputSpeedController && m_pDescriptionSignalValue != NULL)
        {
            ProcessSpeedController(pMediaSample);
        }
        else if (pSource == &m_oInputPinPosition && m_pDescriptionPos != NULL)
        {
            ProcessInputPosition(pMediaSample, 0);
            //TransmitPosition(pMediaSample);
        }
        else if (pSource == &m_oInputTicksToLine && m_pDescriptionTicksToCrosspoint != NULL)
        {
            ProcessTicksToCrosspoint(pMediaSample);
        }
        else if (pSource == &m_oInputTurnLeft && m_pDescriptionBool != NULL)
        {
            ProcessTurnSignalLeft(pMediaSample);
        }
        else if (pSource == &m_oInputTurnRight && m_pDescriptionBool != NULL)
        {
            ProcessTurnSignalRight(pMediaSample);
        }
        else if (pSource == &m_oInputWheelLeft && m_pDescriptionWheelLeftData != NULL)
        {
            ProcessWheelSampleLeft(pMediaSample);
        }
        else if (pSource == &m_oInputWheelRight && m_pDescriptionWheelRightData != NULL)
        {
            ProcessWheelSampleRight(pMediaSample);
        }
        else if (pSource == &m_oInputUsStruct && m_pDescriptionSignalValue != NULL)
        {
            ProcessUsValues(pMediaSample);
        }
        else if (pSource == &m_oInputParkingspaces && m_pDescriptionParkingSpacesStatus != NULL)
        {
            ProcessParkingSpacesStatus(pMediaSample);
        }
    }
    RETURN_NOERROR;
}

tResult cStateMachine::ProcessJuryInput(IMediaSample* pMediaSample)
{
    tInt8 i8ActionID = -2;
    tInt16 i16entry = -1;
    {
        // focus for sample read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionJuryStruct,pMediaSample,pCoder);

        if(!m_bIDsJuryStructSet)
        {
            pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
            pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
            m_bIDsJuryStructSet = tTrue;
        }

        pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
        pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
    }

    if(i8ActionID == action_GETREADY && m_runState != runState_ready)
    {
        // update actual sector and  maneuver
        m_actualManeuverID = i16entry;
        for(unsigned int i = 0; i < m_sectorList.size(); i++)
        {
            tSector s = m_sectorList[i];
            for(unsigned int k = 0; k < s.maneuverList.size(); k++)
            {
                tAADC_Maneuver m = s.maneuverList[k];
                if(i16entry == m.id)
                {
                    m_actualSectorID = i;       // set new actual sector index
                    i = m_sectorList.size();    // break out of
                    break;                      // both loops
                }
            }
        }

        //LOG_INFO(cString::Format("State Machine: Received Request Ready with maneuver ID %d",i16entry));
        ChangeRunState(runState_ready);
        SendState(stateCar_READY, tInt16(m_actualManeuverID));
        TransmitManeuver(-1, -1);   //Reset maneuver list in lane detection
        m_hasSentActualManeuver = false;
    }
    else if(i8ActionID == action_START && m_runState != runState_running)
    {
        //LOG_INFO(cString::Format("State Machine: Received Run with maneuver ID %d",i16entry));
        ChangeRunState(runState_running);
        SendState(stateCar_RUNNING, tInt16(m_actualManeuverID));
        if((getActualManeuver().Compare("pull_out_left") == 0) || (getActualManeuver().Compare("pull_out_right") == 0) )
            checkActualManeuver();
    }
    else if(i8ActionID == action_STOP && m_runState != runState_stop)
    {
        //LOG_INFO(cString::Format("State Machine: Received Stop with maneuver ID %d",i16entry));
        ChangeRunState(runState_stop);
        ChangePrimaryState(primaryState_run);
        SendState(stateCar_STARTUP, tInt16(m_actualManeuverID));
        TransmitManeuver(-1, -1);   //Reset maneuver list in lane detection
        m_hasSentActualManeuver = true;
    }

//    if((getActualManeuver().Compare("pull_out_left") == 0) || (getActualManeuver().Compare("pull_out_right") == 0) )
//        checkActualManeuver();

    //LOG_INFO(cString::Format("got maneuver %d in sector %d from jury!", m_actualManeuverID, m_actualSectorID));

//    switch (juryActions(i8ActionID))
//    {
//    case action_GETREADY:
//        if(m_runState != runState_ready)
//        {
//            //LOG_INFO(cString::Format("State Machine: Received Request Ready with maneuver ID %d",i16entry));
//            ChangeRunState(runState_ready);
//            SendState(stateCar_READY, tInt16(m_actualManeuverID));
//            TransmitManeuver(-1, -1);   //Reset maneuver list in lane detection
//            m_hasSentActualManeuver = false;
//        }
//        break;

//    case action_START:
//        if(m_runState != runState_ready)
//        {
//            //LOG_INFO(cString::Format("State Machine: Received Run with maneuver ID %d",i16entry));
//            ChangeRunState(runState_running);
//            SendState(stateCar_RUNNING, tInt16(m_actualManeuverID));
//            if((getActualManeuver().Compare("pull_out_left") == 0) || (getActualManeuver().Compare("pull_out_right") == 0) )
//                checkActualManeuver();
//        }
//        break;

//    case action_STOP:
//        //LOG_INFO(cString::Format("State Machine: Received Stop with maneuver ID %d",i16entry));
//        ChangeRunState(runState_stop);
//        ChangePrimaryState(primaryState_run);
//        SendState(stateCar_STARTUP, tInt16(m_actualManeuverID));
//        TransmitManeuver(-1, -1);   //Reset maneuver list in lane detection
//        m_hasSentActualManeuver = true;
//        break;
//    }

    RETURN_NOERROR;
}

tResult cStateMachine::SendState(stateCar stateID, tInt16 i16ManeuverEntry)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionDriverStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    tInt8 value = tInt8(stateID);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDriverStruct,pMediaSample,pCoder);
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

    RETURN_NOERROR;
}


tResult cStateMachine::ProcessManeuverList(IMediaSample* pMediaSample)
{
    std::vector<tSize> vecDynamicIDs;
    {
        // focus for sample read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionManeuverList,pMediaSample,pCoder);

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

    LoadManeuverList();

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessFinishedManeuver(IMediaSample* pMediaSample)
{
    tInt16 maneuver = -555;
    tBool  finished = tFalse;

    {   __adtf_sample_read_lock_mediadescription(m_pDescriptionManeuverFinished,pMediaSample,pCoderInput);
        // get IDs
        if (!m_bIDsManeuverFinishedSet)
        {
            pCoderInput->GetID("i16ManeuverValue", m_szManeuverIDInput);
            pCoderInput->GetID("bFinishedFlag", m_szFinishedManeuverInput);
            m_bIDsManeuverFinishedSet=tTrue;
        }

        pCoderInput->Get(m_szManeuverIDInput, (tVoid*)&maneuver);
        pCoderInput->Get(m_szFinishedManeuverInput, (tVoid*)&finished);
    }

    LOG_INFO(cString::Format("received finsihed maneuver %d    %d", maneuver, finished));

    m_actualSpeedLaneDetection = NO_LD_SPEED;
    m_actualSpeed_changed = true;

    if(finished)
    {
        cString actualManeuver = getActualManeuver();
        switch(maneuver)
        {
        case LEFT:
            if(actualManeuver.Compare("left") == 0)
                setActualManeuverCompleted();
            break;

        case RIGHT:
            if(actualManeuver.Compare("right") == 0)
                setActualManeuverCompleted();
            break;

        case STRAIGHT:
            if(actualManeuver.Compare("straight") == 0)
                setActualManeuverCompleted();
            break;

        case PARKING_IN:
            if(cString::Compare(actualManeuver, ("cross_parking"), 13) == 0)
            {
                SetHazzardLight(true);
                m_actualSpeedLaneDetection = 0;
                m_actualSpeed_changed = true;
                ComputeNextStep();
                parkingSpace p = FindNextParkingSpace();
                TransmitParkingSpace(p.u16Id, p.f32X, p.f32Y, 1);

                sleep(3);

                SetHazzardLight(false);
                setActualManeuverCompleted();
                ChangeRunState(runState_running);
            }
            break;

        case PARKING_OUT_LEFT:
            if(actualManeuver.Compare("pull_out_left") == 0)
                setActualManeuverCompleted();
            break;

        case PARKING_OUT_RIGHT:
            if(actualManeuver.Compare("pull_out_right") == 0)
                setActualManeuverCompleted();
            break;

        case OVERTAKING_CHECK:
            ChangeRunState(runState_overtaking_run);
            break;

        case OVERTAKING_RUN:
            ChangeRunState(runState_running);

            break;
        }
    }
    RETURN_NOERROR;
}

tResult cStateMachine::LoadManeuverList()
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
        //LOG_INFO(cString::Format("State Machine: Loaded Maneuver file successfully with %d sectors.", m_sectorList.size()));
        m_actualSectorID = 0;
        m_actualManeuverID = 0;
    }
    else
    {
        LOG_ERROR("State Machine: no valid Maneuver Data found!");
        m_actualSectorID   = -1;
        m_actualManeuverID = -1;
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

/*!
 * loads road-sign configuration from a file, and stores into memory
 * */
tResult cStateMachine::LoadRoadSignConfiguration()
{
    cFilename fileConfig = GetPropertyStr("Configuration");

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty())
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if(IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                roadSign item;
                item.u16Id = tUInt16((*itElem)->GetAttribute("id","0").AsInt32());
                item.f32X = tFloat32((*itElem)->GetAttribute("x","0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y","0").AsFloat64());
                item.f32Radius = tFloat32((*itElem)->GetAttribute("radius","0").AsFloat64());
                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction","0").AsFloat64());

                item.u16Cnt = 0;
                item.u32ticks = adtf_util::cHighResTimer::GetTime();

                item.f32Direction *= DEG2RAD; // convert to radians

                //LOG_INFO(cString::Format("LoadConfiguration::Road Sign Id %d XY %f %f Radius %f Direction %f", item.u16Id, item.f32X, item.f32Y, item.f32Radius, item.f32Direction));

                m_roadSigns.push_back(item);

                i++;
            }
        }
        if(IS_OK(oDOM.FindNodes("configuration/parkingSpace", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                parkingSpace item;
                item.u16Id = tUInt16((*itElem)->GetAttribute("id","0").AsInt32());
                item.f32X = tFloat32((*itElem)->GetAttribute("x","0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y","0").AsFloat64());
                item.f32Status = tFloat32((*itElem)->GetAttribute("status","0").AsInt32());
                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction","0").AsFloat64());

                item.f32Direction *= DEG2RAD; // convert to radians

                //LOG_INFO(cString::Format("LoadConfiguration::Parking Space Id %d XY %f %f Radius %f Direction %f", item.u16Id, item.f32X, item.f32Y, item.f32Status, item.f32Direction));

                m_parkingSpaces.push_back(item);

                i++;
            }
        }
    }
    else
    {
        LOG_ERROR("Configuration file does not exist");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

cString cStateMachine::getActualManeuver()
{
    if((m_actualSectorID == -1) || (m_actualManeuverID == -1))
        return "";

    tSector s = m_sectorList[m_actualSectorID];
    for(unsigned int k = 0; k < s.maneuverList.size(); k++)
    {
        tAADC_Maneuver m = s.maneuverList[k];
        if(m_actualManeuverID == m.id)
            return m_sectorList[m_actualSectorID].maneuverList[k].action;
    }

    return "NO MANEUVER FOUND!";
}

tResult cStateMachine::setActualManeuverCompleted()
{
    if((m_actualSectorID) == -1 || (m_actualManeuverID == -1))
        RETURN_ERROR(-1);

    m_hasSentActualManeuver = false;
    tSector actualSector = m_sectorList[m_actualSectorID];
    if(actualSector.maneuverList[actualSector.maneuverList.size() - 1].id == m_actualManeuverID) // this was the last maneuver of the current sector
    {
        if((int) m_sectorList.size() - 1 == m_actualSectorID) // this was the last sector => finished
        {
            ChangeRunState(runState_stop);
            SendState(stateCar_COMPLETE, tInt16(m_actualManeuverID));
            RETURN_NOERROR;
        }
        else
            m_actualSectorID++;
    }

    m_actualManeuverID++;

    //LOG_INFO(cString::Format("actual maneuver finished, GO on to next Maneuver %d in sector %d: ", m_actualManeuverID, m_actualSectorID).Append(getActualManeuver()));

    SendState(stateCar_RUNNING, tInt16(m_actualManeuverID));

    if((getActualManeuver().Compare("pull_out_left") == 0) || (getActualManeuver().Compare("pull_out_right") == 0) )
        checkActualManeuver();

    RETURN_NOERROR;
}

tResult cStateMachine::checkActualManeuver()
{
    if(!m_hasSentActualManeuver)
    {
        cString actualManeuver = getActualManeuver();
        if(actualManeuver.Compare("left") == 0)
        {
            TransmitManeuver(LEFT, -1);
        }
        else if(actualManeuver.Compare("right") == 0)
        {
            TransmitManeuver(RIGHT, -1);
        }
        else if(actualManeuver.Compare("straight") == 0)
        {
            TransmitManeuver(STRAIGHT, -1);
        }
        else if(actualManeuver.Compare("pull_out_left") == 0)
        {
            TransmitManeuver(PARKING_OUT_LEFT, -1);
            parkingSpace p = FindNextParkingSpace();
            TransmitParkingSpace(p.u16Id, p.f32X, p.f32Y, 0);
        }
        else if(actualManeuver.Compare("pull_out_right") == 0)
        {
            TransmitManeuver(PARKING_OUT_RIGHT, -1);
            parkingSpace p = FindNextParkingSpace();
            TransmitParkingSpace(p.u16Id, p.f32X, p.f32Y, 0);
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessEmergencyStop(IMediaSample* pMediaSample)
{
    static tBufferID szIdEmergencyStopValue = 0;
    static bool szIdEmergencyStopSet = false;
    {
        // focus for sample read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyStop,pMediaSample,pCoder);

        if(!szIdEmergencyStopSet)
        {
            pCoder->GetID("bEmergencyStop", szIdEmergencyStopValue);
            szIdEmergencyStopSet = tTrue;
        }

        pCoder->Get(szIdEmergencyStopValue, (tVoid*)&m_Emergency_Stop_Jury);

    }
    if(m_Emergency_Stop_Jury)
        ChangeRunState(runState_stop);

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessEmergencyBreakStatus(IMediaSample* pMediaSample)
{
    bool ebStatus = false;
    {
        // focus for sample read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyStop,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_szIdEmergencyStopSet)
        {
            pCoder->GetID("bEmergencyStop", m_szIdEmergencyStopValue);
            m_szIdEmergencyStopSet = tTrue;
        }

        pCoder->Get(m_szIdEmergencyStopValue, (tVoid*)&ebStatus);
    }

    if(ebStatus != m_emergencyBreakStatus)
    {
        m_emergencyBreakStatus_changed = true;
        m_emergencyBreakStatus = ebStatus;
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessSpeedController(IMediaSample* pMediaSample)
{
    tFloat32 speed;
    tUInt32 timestamp;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdInputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdInputspeedControllerValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputspeedControllerTimeStamp);
            m_szIdInputSpeedSet = tTrue;
        }

        pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speed);
        pCoderInput->Get(m_szIdInputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    m_actualSpeedLaneDetection = speed;
    m_actualSpeed_changed = true;

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessTicksToCrosspoint(IMediaSample* pMediaSample)
{
    int ticksToNextCrosspoint;
    static bool hasID = false;
    static tBufferID szIDTicksToCrosspoint;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionTicksToCrosspoint, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("i32ticksToLine", szIDTicksToCrosspoint);
            hasID = tTrue;
        }

        pCoderInput->Get(szIDTicksToCrosspoint, (tVoid*)&ticksToNextCrosspoint);
    }
    //LOG_INFO(cString::Format("%d ticks to next line received!", ticksToNextCrosspoint));

    ticksToNextCrosspoint -= 120;

    m_ticksOfNextCrosspoint = ticksToNextCrosspoint + m_actualWheelTicks;

    //LOG_INFO(cString::Format("%d ticks of next crosspoint!", m_ticksOfNextCrosspoint));

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessTurnSignalLeft(IMediaSample* pMediaSample)
{
    tBool bValue = tFalse;
    static bool hasID = false;
    static tBufferID szIDBoolSignalInputF32Value;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("bValue", szIDBoolSignalInputF32Value);
            hasID = true;
        }

        pCoder->Get(szIDBoolSignalInputF32Value, (tVoid*)&bValue);
    }

    //LOG_INFO(cString::Format("Turn signal left %s received!", bValue? "true" : "false"));

    SetTurnLeftSignal(bValue);

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessTurnSignalRight(IMediaSample* pMediaSample)
{
    tBool bValue = tFalse;
    static bool hasID = false;
    static tBufferID szIDBoolSignalInputF32Value;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoder);

        if (!hasID)
        {
            pCoder->GetID("bValue", szIDBoolSignalInputF32Value);
            hasID = true;
        }

        pCoder->Get(szIDBoolSignalInputF32Value, (tVoid*)&bValue);
    }

    //LOG_INFO(cString::Format("Turn signal right %s received!", bValue? "true" : "false"));

    SetTurnRightSignal(bValue);

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessUsValues(IMediaSample* pMediaSample)
{
    //use mutex
    __synchronized_obj(m_critSecMinimumUsValue);

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

    //iterate through all values
    tFloat32 buf_UsSignal;
    for(int i=0; i<(int)m_szIdUsStructValues.size(); ++i)
    {
        pCoderInput->Get(m_szIdUsStructValues[i], (tVoid*)&buf_UsSignal);
        //save values
        m_actualDistances[i] = buf_UsSignal;
    }

    RETURN_NOERROR;
}

tFloat32 cStateMachine::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha-center+ static_cast<tFloat32>(M_PI), 2.0*static_cast<tFloat32>(M_PI)) + center- static_cast<tFloat32>(M_PI);
}

tFloat32 cStateMachine::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
        return x - floor(x / y) * y;
    else
    {
        r = x / y;
        if (r < 0.0f)
            b_x = ceil(r - 0.5f);
        else
            b_x = floor(r + 0.5f);

        if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r))
            return 0.0f;
        else
            return (r - floor(r)) * y;
    }
}

/*! workaround for heading update due to pose estimation
    issues with current Aruco version
*/
#define MP_ARUCO_WORKAROUND
/*! Calculates orientation, distance and pose of the given road sign,
 * and updates the positioning filter accordingly */
tResult cStateMachine::ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn)
{
    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSignExt,pMediaSampleIn,pCoderInput);

        // get IDs
        if(!m_bIDsRoadSignExtSet)
        {
            pCoderInput->GetID("i16Identifier",m_szIDRoadSignExtI16Identifier);
            pCoderInput->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoderInput->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoderInput->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }

        pCoderInput->Get(m_szIDRoadSignExtI16Identifier, (tVoid*)&m_i16ID);
        pCoderInput->Get(m_szIDRoadSignExtF32Imagesize, (tVoid*)&m_f32MarkerSize);
        pCoderInput->Get("af32TVec", (tVoid*)m_Tvec.data);
        pCoderInput->Get("af32RVec", (tVoid*)m_Rvec.data);

    }

    // ignore initial noisy markers
    if (m_ui32Cnt < 5)
    {
        m_ui32Cnt++;
        RETURN_NOERROR;
    }

    cv::Mat R;
    cv::Rodrigues(m_Rvec, R); // rot is 3x3

    // calculate translation
    tFloat32 lateral = m_Tvec.at<float>(0);
    tFloat32 longitudinal = m_Tvec.at<float>(2);

    // add camera offset
    lateral += m_f32CameraOffsetLat;
    longitudinal += m_f32CameraOffsetLon;

    tFloat32 d0 = sqrt(lateral*lateral+longitudinal*longitudinal);

    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = (tFloat32)normalizeAngle(a0,0.0f)*RAD2DEG; // normalize angle -pi:pi

    a0 *= -1.0; // and change direction

    // calculate pose of the road sign
    //tFloat32 yawE;

    //    tFloat32 pitch1  = -asin(R.at<float>(2,0));
    //    tFloat32 pitch2  = (tFloat32)normalizeAngle(static_cast<tFloat32>(CV_PI) - pitch1, 0.0f);

    //    tFloat32 yaw1  = atan2(R.at<float>(2,1) / cos(pitch1), R.at<float>(2,2) / cos(pitch1));
    //    tFloat32 yaw2  = atan2(R.at<float>(2,1) / cos(pitch2), R.at<float>(2,2) / cos(pitch2));

    // select shortest rotation for yaw
    //    if (abs(yaw1) <= abs(yaw2))
    //    {
    //        yawE   = yaw1*RAD2DEG;
    //    }
    //    else
    //    {
    //        yawE   = yaw2*RAD2DEG;
    //    }

    if(m_f32MarkerSize > 250 && m_i16ID != MARKER_ID_TESTCOURSEA9 && m_i16ID < 22)
    {
        m_lastTicksRelevantSignDetected = m_actualWheelTicks;
        if(m_actualSpeedUpFactor != 1.0)
        {
            m_actualSpeed_changed = true;
            m_actualSpeedUpFactor = 1.0;
            m_runState_changed = true;
        }
    }

    if(m_f32MarkerSize < 1250 && m_i16ID != MARKER_ID_TESTCOURSEA9 && m_i16ID < 22)  // check distance limit
    {
        //LOG_INFO(cString::Format("Roadsign %d rejected with marker size %f", m_i16ID, m_f32MarkerSize));
        RETURN_NOERROR;
    }

    //LOG_INFO(cString::Format("Roadsign %d accepted with marker size %f", m_i16ID, m_f32MarkerSize));

    switch(m_i16ID)
    {
    case MARKER_ID_PARKINGAREA:
        if(cString::Compare("cross_parking", getActualManeuver(), 13) == 0)
        {
            if(!m_hasSentActualManeuver)
            {
                int parkingspaceID = getActualManeuver().GetAt(14) - '0';
                while(parkingspaceID > 4)
                {
                    parkingspaceID -= 4;
                }

                parkingspaceID = 4 - parkingspaceID + 1;

                //LOG_INFO(cString::Format("parking sign detected and maneuver, start parking at id %d!", parkingspaceID));
                TransmitManeuver(PARKING_IN, parkingspaceID);
                ChangeRunState(runState_parking);
            }
        }
        else{
            if(transmitParkingSpaceSearch){
                transmitParkingSpaceSearch = false;
                TransmitManeuver(PARKING_SPACE_SEARCH, -1);
            }
        }
        break;

    case MARKER_ID_GIVEWAY:
        //LOG_INFO(cString::Format("giveway sign detected at ticks: %d", m_lastTicksGiveWaySignDetected));
        m_lastTicksGiveWaySignDetected = m_actualWheelTicks;
        checkActualManeuver();
        break;

    case MARKER_ID_STOPANDGIVEWAY:
    case MARKER_ID_UNMARKEDINTERSECTION:
        //LOG_INFO(cString::Format("Stop sign detected at ticks: %d", m_lastTicksStopSignDetected));
        m_lastTicksStopSignDetected = m_actualWheelTicks;
        checkActualManeuver();
        break;

    case MARKER_ID_PEDESTRIANCROSSING:
        //LOG_INFO(cString::Format("pedestrian sign detected at ticks: %d", m_lastTicksPedestrianSignDetected));
        m_lastTicksPedestrianSignDetected = m_actualWheelTicks;
        TransmitManeuver(PEDESTRIAN_CROSSING, -1);
        break;

    case MARKER_ID_HAVEWAY:
        //LOG_INFO(cString::Format("have way sign detected at ticks: %d", m_lastTicksHaveWaySignDetected));
        m_lastTicksHaveWaySignDetected = m_actualWheelTicks;
        checkActualManeuver();
        break;

    case MARKER_ID_AHEADONLY:
        checkActualManeuver();
        break;
    }

    if(!m_pos_Initialized)
        RETURN_NOERROR;

    // find a matching road sign
    tFloat64 dt = 0;
    bool found = false;
    for (unsigned int i=0;i<m_roadSigns.size();i++)
    {
        if (m_roadSigns[i].u16Id == m_i16ID)
        {
            // calculate heading wrt marker
            tFloat32 heading = static_cast<tFloat32>(m_ActualPosition.f32Heading + a0*DEG2RAD);
            heading = normalizeAngle(heading,0);

            // estimate marker location based on current vehicle location
            // and marker measurement
            tFloat32 x0 = static_cast<tFloat32>(m_ActualPosition.f32X+cos(heading)*d0);//m_ActualPosition.f32X;//static_cast<tFloat32>(m_state.at<double>(0)+cos(heading)*d0);
            tFloat32 y0 = static_cast<tFloat32>(m_ActualPosition.f32Y+sin(heading)*d0);//m_ActualPosition.f32Y;//static_cast<tFloat32>(m_state.at<double>(1)+sin(heading)*d0);

            // calculate error distance
            tFloat32 dx = x0-m_roadSigns[i].f32X;
            tFloat32 dy = y0-m_roadSigns[i].f32Y;

            tFloat32 distance = sqrt(dx*dx + dy*dy);

            //if (m_log) LOG_INFO(cString::Format("markerSearch :: dx %.3f distance %.3f", distance, d0));
            //LOG_INFO(cString::Format("estimated distance to Roadsign %f", distance));
            // marker found within the radius
            if(distance < 2.0)
                found = true;

            if (distance < m_roadSigns[i].f32Radius)
            {
                // calculate time from previous marker measurement
                dt = (cSystem::GetTime() - m_roadSigns[i].u32ticks)*1e-6;
                m_roadSigns[i].u32ticks = cSystem::GetTime();

                // reset sample counter when marker reappears
                if (dt > 1.0)
                {
                    m_roadSigns[i].u16Cnt = 0;
                }

                //TransmitTrafficSign(m_i16ID, m_roadSigns[i].f32X, m_roadSigns[i].f32Y, m_roadSigns[i].f32Direction);

                break;
            }
        }
    }

//    if(!found)
//    {
//        tFloat32 angle = m_ActualPosition.f32Heading  / -1.5 * 270;
//        while(angle < 0)
//            angle += 360;

//        while(angle >= 360)
//            angle -= 360;

//        tFloat32 heading = static_cast<tFloat32>(m_ActualPosition.f32Heading + a0*DEG2RAD);
//        heading = normalizeAngle(heading,0);
//        tFloat32 x0 = static_cast<tFloat32>(m_ActualPosition.f32X+cos(heading)*d0);
//        tFloat32 y0 = static_cast<tFloat32>(m_ActualPosition.f32Y+sin(heading)*d0);
//        LOG_INFO(cString::Format("Found NEW traffic sign %d at x %f and y %f !", m_i16ID, x0, y0));

//        TransmitTrafficSign(m_i16ID, x0, y0, heading);
//    }



    //LOG_INFO(cString::Format("Found Sign with id %d and x %f and y %f", m_i16ID, x0, y0));
    RETURN_NOERROR;
}

tResult cStateMachine::CheckMissingTrafficSign()
{
    tFloat32 distance = 99999999;
    tFloat32 angleVariance = 10.0f;
    roadSign rs;

    for(int i = 0; i < (int) m_roadSigns.size(); i++)
    {
        roadSign r = m_roadSigns.at(i);

        if(rs.u16Id == MARKER_ID_TESTCOURSEA9 || rs.u16Id > 22)
        {
            continue;
        }

        //########################################
        ///////// MADE LAST NIGHT!!!!
        //########################################

        tFloat32 dif = abs(r.f32Direction - m_ActualPosition.f32Heading * RAD2DEG);

        if((dif < 180 + angleVariance) && (dif > 180 - angleVariance))
        {
            tFloat32 dx = m_ActualPosition.f32X - r.f32X;
            tFloat32 dy = m_ActualPosition.f32Y - r.f32Y;
            tFloat32 d = sqrt(dx*dx + dy*dy);

            if(d < distance)
            {
                distance = d;
                rs = r;
            }
        }
    }



    if((cSystem::GetTime() - rs.u32ticks) > 20000000)
    {
        if(distance < 0.5)
        {
            //LOG_INFO(cString::Format("Missing traffic sign %d detected with distance %f!", rs.u16Id, distance));
            rs.u32ticks = m_actualWheelTicks;
            TransmitTrafficSign(-1, rs.f32X, rs.f32Y, rs.f32Direction);
            if(rs.u16Id == MARKER_ID_GIVEWAY || rs.u16Id == MARKER_ID_HAVEWAY || rs.u16Id == MARKER_ID_UNMARKEDINTERSECTION || rs.u16Id == MARKER_ID_AHEADONLY)
                m_lastTicksStopSignDetected = m_actualWheelTicks;

        }

        if(distance < 0.97 && (rs.u16Id == MARKER_ID_GIVEWAY || rs.u16Id == MARKER_ID_HAVEWAY || rs.u16Id == MARKER_ID_UNMARKEDINTERSECTION || rs.u16Id == MARKER_ID_AHEADONLY))
        {
            checkActualManeuver();
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionPos,pMediaSampleIn,pCoderInput);

        if (!m_PosInputSet)
        {
            pCoderInput->GetID("f32x", m_szF32X);
            pCoderInput->GetID("f32y", m_szF32Y);
            pCoderInput->GetID("f32radius", m_szF32Radius);
            pCoderInput->GetID("f32speed", m_szF32Speed);
            pCoderInput->GetID("f32heading", m_szF32Heading);
            m_PosInputSet=tTrue;
        }

        pCoderInput->Get(m_szF32X, (tVoid*)&m_ActualPosition.f32X);
        pCoderInput->Get(m_szF32Y, (tVoid*)&m_ActualPosition.f32Y);
        pCoderInput->Get(m_szF32Radius, (tVoid*)&m_ActualPosition.f32Radius);
        pCoderInput->Get(m_szF32Speed, (tVoid*)&m_ActualPosition.f32Speed);
        pCoderInput->Get(m_szF32Heading, (tVoid*)&m_ActualPosition.f32Heading);
    }

    if(!m_pos_Initialized)
    {
        m_pos_Initialized = true;
        //LOG_INFO(cString::Format("Pos initialized with x %f and y %f ",m_ActualPosition.f32X, m_ActualPosition.f32Y));
    }

    //    static uint64 startTime = cSystem::GetTime();
    //    uint64 actualtime = cSystem::GetTime();
    //    if(actualtime - startTime > 300000){
    //        LOG_INFO(cString::Format("car position: x: %f, y: %f dir %f", m_ActualPosition.f32X, m_ActualPosition.f32Y, m_ActualPosition.f32Heading));
    //        startTime = actualtime;
    //    }
    CheckMissingTrafficSign();

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitPosition(IMediaSample* pMediaSample)
{
    m_OutputPostion.Transmit(pMediaSample);
    RETURN_NOERROR;
}

tResult cStateMachine::TransmitManeuver(tInt16 maneuverNumber, tInt8 parkingSpaceID)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitManeuver);

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionManeuver->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionManeuver, pMediaSample, pCoderOutput);

        if(!m_bIDManeuverSendSet)
        {
            pCoderOutput->GetID("i16ManeuverValue", m_szManeuverIDOutput);
            pCoderOutput->GetID("i8ParkingSpaceID", m_szParkingSpaceIDOutput);
            m_bIDManeuverSendSet = tTrue;
        }

        pCoderOutput->Set(m_szManeuverIDOutput, (tVoid*)&maneuverNumber);
        pCoderOutput->Set(m_szParkingSpaceIDOutput, (tVoid*)&parkingSpaceID);
    }
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_ManeuverOutputPin.Transmit(pMediaSample);

    if((maneuverNumber != PEDESTRIAN_CROSSING ) && (maneuverNumber != -1) && (maneuverNumber != PARKING_SPACE_SEARCH))
    {
        m_hasSentActualManeuver = true;
    }

    LOG_INFO(cString::Format("State Machine transmitted maneuver %d to lane detection", maneuverNumber));
    RETURN_NOERROR;
}

tResult cStateMachine::ProcessClassificationInput(IMediaSample* pMediaSample)
{
    std::vector<myClassificationResult> classificationResults;
    classificationResults.resize(pMediaSample->GetSize() / sizeof(myClassificationResult));
    //get the date from the media sample
    tVoid* pIncomingData;
    if (IS_OK(pMediaSample->Lock((const tVoid**)&pIncomingData)))
    {
        //make copy
        memcpy(classificationResults.data(), pIncomingData, pMediaSample->GetSize());
        pMediaSample->Unlock(pIncomingData);
    }

    if (!classificationResults.empty())
    {
        for (std::vector<myClassificationResult>::iterator it = classificationResults.begin(); it != classificationResults.end(); it++)
        {
            if(it->probability >= DETECTION_THRESHOLD)
            {
                //LOG_INFO(cString::Format("Classification %s with prop %f %% received bbx %f bby %f widht %f height %f!", it->classificationDesc, it->probability * 100, it->boundingBox.x, it->boundingBox.y, it->boundingBox.width, it->boundingBox.height));
                cString name(it->classificationDesc);
                if(name.Compare("car_back") == 0)
                {
                    m_lastTimeFollowCarDetected = cSystem::GetTime();

                    int heightPixel = 750;
                    int widthPixelLeft = 400;
                    int widthPixelRight = 1500;
                    m_lastCarCounter = 0;

                    //LOG_INFO(cString::Format("ignolwjngvowng %f ", (heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f)));
                    if(m_actualSpeedCarDetection <= -2){
                        if((heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f) <= 0){
                            // slower
                            m_actualSpeedCarDetection -= heightPixel/500.0f - (it->boundingBox.y + it->boundingBox.height)/500.0f;

                            if(m_actualSpeedCarDetection >= -2){
                                if(m_actualSpeedCarDetection != 0)
                                    m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                                m_actualSpeedCarDetection = 0;
                            }
                            m_actualSpeed_changed = true;
                        }
                        else{
                            // faster
                            m_actualSpeedCarDetection -= ceil(heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f);
                            if(m_actualSpeedCarDetection <= DEFAULT_SPEED){
                                //LOG_INFO("FASTER!");
                                m_actualSpeedCarDetection = DEFAULT_SPEED;
                            }
                            m_actualSpeed_changed = true;
                        }
                    }
                    else{
                        if((it->boundingBox.y + it->boundingBox.height) < heightPixel)
                        {
                            m_actualSpeedCarDetection = DEFAULT_SPEED;
                            //LOG_INFO(cString::Format("y + height: %d,  heightpixel %d", it->boundingBox.y + it->boundingBox.height, heightPixel));
                            m_actualSpeed_changed = true;
                        }
                    }

                    //LOG_INFO(cString::Format("folge fahrt speed %f", m_actualSpeedCarDetection));
                }
                else if(name.Compare("car_left") == 0 || name.Compare("car_right") == 0)
                {
                    // check if there was a GiveWay oder Stop sign at the last 200 ticks, then stop!
                    if((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 200)
                            || (m_actualWheelTicks < m_lastTicksStopSignDetected + 200))
                    {
                        m_actualSpeedCarDetection = 0;
                        m_actualSpeed_changed = true;
                        m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                    }

                    //LOG_INFO(cString::Format("Classification %s with prop %f %% received bbx %f bby %f widht %f height %f!", it->classificationDesc, it->probability * 100, it->boundingBox.x, it->boundingBox.y, it->boundingBox.width, it->boundingBox.height));


                    m_lastTimeCarDetected = adtf_util::cHighResTimer::GetTime();
                }
                else if(name.Compare("car_front") == 0)
                {
                    // check if there was a GiveWay oder Stop sign at the last 200 ticks, then stop!
                    if(((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 200)
                        || (m_actualWheelTicks < m_lastTicksStopSignDetected + 200)
                        || (m_actualWheelTicks < m_lastTicksHaveWaySignDetected + 200))
                            && (getActualManeuver().Compare("left") == 0))
                    {
                        m_actualSpeedCarDetection = 0;
                        m_actualSpeed_changed = true;
                        m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                    }
                }
                else if(name.Compare("adult") == 0)
                {
                    if((it->boundingBox.x) > 0.3f*1920)// && (it->boundingBox.x + it->boundingBox.width) < 0.6f*1920) // adult is at right side of the street
                    {
                        if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                        {
                            m_actualSpeedAdultDetection = 0;  // stop, if there is a adult_right at the left side of a pedestrian crossing
                            m_actualSpeed_changed = true;
                            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                        }
                        m_lastTicksAdultDetected = m_actualWheelTicks;
                    }
                }
                else if(name.Compare("adult_left") == 0)
                {
                    if((it->boundingBox.x) > 0.3f*1920) // adult_left is at right side of the street
                    {
                        if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                        {
                            m_actualSpeedAdultDetection = 0;  // stop, if there is a adult_right at the left side of a pedestrian crossing
                            m_actualSpeed_changed = true;
                            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                        }
                        m_lastTicksAdultDetected = m_actualWheelTicks;
                    }
                }
                else if(name.Compare("adult_right") == 0)
                {
                    if((it->boundingBox.x) > 0.3f*1920) //(it->boundingBox.x + it->boundingBox.width) < 0.6f*1920) // adult_right is at left side of the street
                    {
                        if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                        {
                            m_actualSpeedAdultDetection = 0;    // stop, if there is a adult_right at the left side of a pedestrian crossing
                            m_actualSpeed_changed = true;
                            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                        }
                        m_lastTicksAdultDetected = m_actualWheelTicks;
                    }
                }
                else if(name.Compare("child") == 0)
                {
                    //int heightPixel = 650;
                    //int widthPixelRight = 1500;
                    //                    if(it->boundingBox.y > heightPixel && (it->boundingBox.x + it->boundingBox.width) < widthPixelRight) // child is near car
                    //                    {
                    if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                    {
                        m_actualSpeedChildDetection = 0;    // stop, if there is a child at the pedestrian crossing
                        m_actualSpeed_changed = true;
                        m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                    }
                    else
                    {
                        m_actualSpeedChildDetection = DEFAULT_SPEED  * 0.65f;
                        m_actualSpeed_changed = true;
                    }

                    m_lastTicksChildDetected = m_actualWheelTicks;
                    //                    }
                }
            }
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessParkingSpacesStatus(IMediaSample* pMediaSample)
{
    bool status[4];
    static bool status_set = false;
    static tBufferID szBool1, szBool2, szBool3, szBool4;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionParkingSpacesStatus,pMediaSample,pCoderInput);

        if (!status_set)
        {
            pCoderInput->GetID("parkingSpace1", szBool1);
            pCoderInput->GetID("parkingSpace2", szBool2);
            pCoderInput->GetID("parkingSpace3", szBool3);
            pCoderInput->GetID("parkingSpace4", szBool4);
            status_set = tTrue;
        }

        pCoderInput->Get(szBool1, (tVoid*)&status[0]);
        pCoderInput->Get(szBool2, (tVoid*)&status[1]);
        pCoderInput->Get(szBool3, (tVoid*)&status[2]);
        pCoderInput->Get(szBool4, (tVoid*)&status[3]);
    }

    //LOG_INFO(cString::Format("State Machine received Parking status: %s %s %s %s", status[0] ? "true" : "false", status[1] ? "true" : "false", status[2] ? "true" : "false", status[3] ? "true" : "false"));

    transmitParkingSpaceSearch = true;

    std::vector<parkingSpace> parkingSpaces = FindNextParkingArea();
    if(!parkingSpaces.empty())
    {
        for(int i = 0; i < (int) parkingSpaces.size(); i++)
        {
            parkingSpace p = parkingSpaces[i];
            TransmitParkingSpace(p.u16Id, p.f32X, p.f32Y, status[i]);
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitEmergencyBreakSet()
{
    // default: no sensor is set
    tInt16 newDistances[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    switch(m_runState)
    {
    case runState_stop:
    case runState_ready:
    case runState_running:
    {
        // set only the front sensors
        if(m_actualSpeedUpFactor > 1.0)
        {
            newDistances[1] = 15;
            newDistances[2] = 40;
            newDistances[3] = 15;
        }
        else
        {
            newDistances[1] = 10;
            newDistances[2] = 30;
            newDistances[3] = 10;
        }
        break;
    }

    case runState_parking:
    {
        // set all sensors
        newDistances[1] = 5;
        newDistances[2] = 10;
        newDistances[3] = 5;
        newDistances[7] = 5;
        newDistances[8] = 5;
        newDistances[9] = 5;
        break;
    }

    case runState_overtaking_check:
        break;

    case runState_overtaking_run:
    {
        newDistances[1] = 10;
        newDistances[2] = 30;
        newDistances[3] = 10;
        break;
    }
    }

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionEmergencyBreakSet->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionEmergencyBreakSet,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_szIdEmergencyBreakSet)
        {
            pCoder->GetID("i16EmergencyBreakFrontLeft",        m_szIdEmergencyBreakSetFrontLeftValue);
            pCoder->GetID("i16EmergencyBreakFrontCenterLeft",  m_szIdEmergencyBreakSetFrontHalfLeftValue);
            pCoder->GetID("i16EmergencyBreakFrontMiddle",      m_szIdEmergencyBreakSetFrontMiddleValue);
            pCoder->GetID("i16EmergencyBreakFrontCenterRight", m_szIdEmergencyBreakSetFrontHalfRightValue);
            pCoder->GetID("i16EmergencyBreakFrontRight",       m_szIdEmergencyBreakSetFrontRightValue);
            pCoder->GetID("i16EmergencyBreakSideLeft",         m_szIdEmergencyBreakSetSideLeftValue);
            pCoder->GetID("i16EmergencyBreakSideRight",        m_szIdEmergencyBreakSetSideRightValue);
            pCoder->GetID("i16EmergencyBreakBackLeft",         m_szIdEmergencyBreakSetBackLeftValue);
            pCoder->GetID("i16EmergencyBreakBackMiddle",       m_szIdEmergencyBreakSetBackMiddleValue);
            pCoder->GetID("i16EmergencyBreakBackRight",        m_szIdEmergencyBreakSetBackRightValue);
            m_szIdEmergencyBreakSet = tTrue;
        }

        pCoder->Set(m_szIdEmergencyBreakSetFrontLeftValue,      (tVoid*)&newDistances[0]);
        pCoder->Set(m_szIdEmergencyBreakSetFrontHalfLeftValue,  (tVoid*)&newDistances[1]);
        pCoder->Set(m_szIdEmergencyBreakSetFrontMiddleValue,    (tVoid*)&newDistances[2]);
        pCoder->Set(m_szIdEmergencyBreakSetFrontHalfRightValue, (tVoid*)&newDistances[3]);
        pCoder->Set(m_szIdEmergencyBreakSetFrontRightValue,     (tVoid*)&newDistances[4]);
        pCoder->Set(m_szIdEmergencyBreakSetSideLeftValue,       (tVoid*)&newDistances[5]);
        pCoder->Set(m_szIdEmergencyBreakSetSideRightValue,      (tVoid*)&newDistances[6]);
        pCoder->Set(m_szIdEmergencyBreakSetBackLeftValue,       (tVoid*)&newDistances[7]);
        pCoder->Set(m_szIdEmergencyBreakSetBackMiddleValue,     (tVoid*)&newDistances[8]);
        pCoder->Set(m_szIdEmergencyBreakSetBackRightValue,      (tVoid*)&newDistances[9]);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputEmergencyBreakSet.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitTrafficSign(tInt16 i16Id, tFloat32 f32x, tFloat32 f32y, tFloat32 f32angle)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionTrafficSign->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionTrafficSign,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_TrafficSignOutputSet)
        {
            pCoder->GetID("i16Identifier", m_tsI16id);
            pCoder->GetID("f32x", m_tsF32X);
            pCoder->GetID("f32y", m_tsF32Y);
            pCoder->GetID("f32angle", m_tsF32Angle);
            m_TrafficSignOutputSet = tTrue;
        }

        pCoder->Set(m_tsI16id, (tVoid*)&i16Id);
        pCoder->Set(m_tsF32X, (tVoid*)&f32x);
        pCoder->Set(m_tsF32Y, (tVoid*)&f32y);
        pCoder->Set(m_tsF32Angle, (tVoid*)&f32angle);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_OutputTrafficSign.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitObstacle(tFloat32 f32x, tFloat32 f32y)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionObstacle->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionObstacle,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_ObstacleOutputSet)
        {
            pCoder->GetID("f32x", m_obstacleF32X);
            pCoder->GetID("f32y", m_obstacleF32Y);
            m_ObstacleOutputSet = tTrue;
        }

        pCoder->Set(m_obstacleF32X, (tVoid*)&f32x);
        pCoder->Set(m_obstacleF32Y, (tVoid*)&f32y);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_OutputObstacle.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitParkingSpace(tInt16 i16Id, tFloat32 f32x, tFloat32 f32y, tUInt16 ui16Status)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionParkingSpace->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {
        // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionParkingSpace,pMediaSample,pCoder);
        // get the IDs for the items in the media sample
        if(!m_ParkingOutputSet)
        {
            pCoder->GetID("i16Identifier", m_parkingI16Id);
            pCoder->GetID("f32x", m_parkingF32X);
            pCoder->GetID("f32y", m_parkingF32Y);
            pCoder->GetID("ui16Status", m_parkingUI16Status);
            m_ParkingOutputSet = tTrue;
        }

        pCoder->Set(m_parkingI16Id, (tVoid*)&i16Id);
        pCoder->Set(m_parkingF32X, (tVoid*)&f32x);
        pCoder->Set(m_parkingF32Y, (tVoid*)&f32y);
        pCoder->Set(m_parkingUI16Status, (tVoid*)&ui16Status);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_OutputParkingSpace.Transmit(pMediaSample);

    RETURN_NOERROR;
}

parkingSpace cStateMachine::FindNextParkingSpace()
{
    tFloat32     distance = 99999999;
    parkingSpace ps;
    for(int i = 0; i < (int) m_parkingSpaces.size(); i++)
    {
        parkingSpace p = m_parkingSpaces.at(i);
        tFloat32 d = sqrt(pow(abs(p.f32X - m_ActualPosition.f32X), 2) + pow(abs(p.f32Y - m_ActualPosition.f32Y), 2));
        if(d < distance)
        {
            distance = d;
            ps = p;
        }
    }

    return ps;
}

std::vector<parkingSpace> cStateMachine::FindNextParkingArea()
{
    tFloat32     distance = 99999999;
    std::vector<parkingSpace> ps;
    ps.reserve(4);

    for(int i = 3; i < (int) m_parkingSpaces.size(); i+=4)
    {
        parkingSpace p = m_parkingSpaces.at(i);
        tFloat32 d = sqrt(pow(abs(p.f32X - m_ActualPosition.f32X), 2) + pow(abs(p.f32Y - m_ActualPosition.f32Y), 2));
        if(d < distance && m_parkingSpaces.size() > i)
        {
            distance = d;
            ps[0] = m_parkingSpaces.at(i - 3);
            ps[1] = m_parkingSpaces.at(i - 2);
            ps[2] = m_parkingSpaces.at(i - 1);
            ps[3] = m_parkingSpaces.at(i);
        }
    }

    return ps;
}

tResult cStateMachine::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
{
    static bool hasID = false;
    static tBufferID szIDWheelDataUi32WheelTach;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelLeftData, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
            hasID = tTrue;
        }

        pCoderInput->Get(szIDWheelDataUi32WheelTach, (tVoid*)&wheelCountLeft);
    }

    m_actualWheelTicks = ((wheelCountLeft + wheelCountRight) / 2);

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessWheelSampleRight(IMediaSample* pMediaSample)
{
    static bool hasID = false;
    static tBufferID szIDWheelDataUi32WheelTach;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelRightData, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
            hasID = tTrue;
        }

        pCoderInput->Get(szIDWheelDataUi32WheelTach, (tVoid*)&wheelCountRight);

    }

    if(m_actualWheelTicks == 0)
        m_lastTicksRelevantSignDetected = m_actualWheelTicks; // to prevent speedup at startup!

    m_actualWheelTicks = ((wheelCountLeft + wheelCountRight) / 2);

    ComputeNextStep();

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitBool);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDBoolValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("bValue", szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDBoolValueOutput, (tVoid*)&value);
        pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cStateMachine::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitControl);

    //init mediasample
    cObjectPtr<IMediaSample> pMediaSample;
    //allocate memory to mediasample
    AllocMediaSample((tVoid**)&pMediaSample);

    //create interaction with ddl
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);

    //allocate buffer to write in mediasample
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
    {
        //write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdOutputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdOutputspeedControllerValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdOutputspeedControllerTimeStamp);
            m_szIdOutputSpeedSet = tTrue;
        }

        pCoderInput->Set(m_szIdOutputspeedControllerValue, (tVoid*)&speed);
        pCoderInput->Set(m_szIdOutputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oOutputSpeedController.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cStateMachine::updateLights()
{
    if(m_BrakeLightOn_changed)
    {
        m_BrakeLightOn_changed = false;
        if(m_BrakeLightOn > 0){
            TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
        }
        else
            TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
    }

    if(m_HeadLightOn_changed)
    {
        m_HeadLightOn_changed = false;
        if(m_HeadLightOn > 0){
            TransmitBoolValue(&m_oOutputHeadLight, true, 0);
        }
        else
            TransmitBoolValue(&m_oOutputHeadLight, false, 0);
    }

    if(m_ReverseLightOn_changed)
    {
        m_ReverseLightOn_changed = false;
        if(m_ReverseLightOn > 0){
            TransmitBoolValue(&m_oOutputReverseLight, true, 0);
        }
        else if(m_HazzardLightOn == 0)
            TransmitBoolValue(&m_oOutputReverseLight, false, 0);
    }

    if(m_TurnLeftSignalOn_changed)
    {
        m_TurnLeftSignalOn_changed = false;
        if(m_TurnLeftSignalOn > 0){
            if(m_HazzardLightOn == 0){
                if(m_TurnRightSignalOn > 0){
                    m_TurnRightSignalOn = 0;
                }
                TransmitBoolValue(&m_oOutputTurnLeft, true, 0);
            }
        }
        else if(m_HazzardLightOn == 0){
            TransmitBoolValue(&m_oOutputTurnLeft, false, 0);
        }
    }

    if(m_TurnRightSignalOn_changed)
    {
        m_TurnRightSignalOn_changed = false;
        if(m_TurnRightSignalOn > 0){
            if(m_HazzardLightOn == 0){
                if(m_TurnLeftSignalOn > 0){
                    m_TurnLeftSignalOn = 0;
                }
                TransmitBoolValue(&m_oOutputTurnRight, true, 0);
            }
        }
        else
            TransmitBoolValue(&m_oOutputTurnRight, false, 0);
    }

    if(m_HazzardLightOn_changed)
    {
        m_HazzardLightOn_changed = false;
        if(m_HazzardLightOn > 0){
            TransmitBoolValue(&m_oOutputHazzardLight, true, 0);
        }
        else{
            TransmitBoolValue(&m_oOutputHazzardLight, false, 0);
            if(m_TurnLeftSignalOn > 0)
                m_TurnLeftSignalOn_changed = true;
            if(m_TurnRightSignalOn > 0)
                m_TurnRightSignalOn_changed = true;
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::updateSpeed()
{
    if(m_actualSpeed_changed)
    {
        m_actualSpeed_changed = false;

        tFloat32 newSpeed = DEFAULT_SPEED;
        if(abs(m_actualSpeedState) < abs(newSpeed))
            newSpeed = m_actualSpeedState;

        if(abs(m_actualSpeedAdultDetection) < abs(newSpeed))
            newSpeed = m_actualSpeedAdultDetection;

        if(abs(m_actualSpeedCarDetection) < abs(newSpeed) && m_lastCarCounter < 400)
            newSpeed = m_actualSpeedCarDetection;

        if(abs(m_actualSpeedChildDetection) < abs(newSpeed))
            newSpeed = m_actualSpeedChildDetection;

        if(abs(m_actualSpeedTrafficSignDetection) < abs(newSpeed))
            newSpeed = m_actualSpeedTrafficSignDetection;

        if(newSpeed == DEFAULT_SPEED)
        {
            newSpeed *= m_actualSpeedUpFactor;
            //LOG_INFO(cString::Format("SPEED UP! %d > %d", m_actualWheelTicks, (m_lastTicksRelevantSignDetected + 500)));
        }


        if(m_actualSpeedLaneDetection != NO_LD_SPEED && m_actualSpeedState != 0)
        {
            newSpeed = m_actualSpeedLaneDetection;
        }

        if(abs(newSpeed) < abs(DEFAULT_SPEED) && (m_BrakeLightOn == 0)
                && (m_runState != runState_stop) && (m_runState != runState_ready))
        {
            SetBrakeLight(true);
        }
        else if(abs(newSpeed) >= abs(DEFAULT_SPEED) && (m_BrakeLightOn > 0)
                && m_runState != runState_stop && m_runState != runState_ready)
        {
            SetBrakeLight(false);
        }

        if(newSpeed > 0 && m_ReverseLightOn == 0)
        {
            SetReverseLight(true);
        }
        else if(newSpeed <= 0 && m_ReverseLightOn > 0)
        {
            SetReverseLight(false);
        }

        //LOG_INFO(cString::Format("speed %f", newSpeed));
        TransmitSpeed(newSpeed, 0);
    }

    m_lastCarCounter++;

    RETURN_NOERROR;
}

tResult cStateMachine::SetLight(int* lightCounter, bool* changedFlag, bool status)
{
    if(status)
    {
        if(*lightCounter == 0)
            *changedFlag = true;
        (*lightCounter)++;
    }
    else
    {
        (*lightCounter)--;
        *lightCounter = max(*lightCounter, 0);
        if(*lightCounter == 0)
            *changedFlag = true;
    }

    RETURN_NOERROR;
}

tResult cStateMachine::SetHazzardLight(bool status)
{
    SetLight(&m_HazzardLightOn, &m_HazzardLightOn_changed, status);

    //LOG_INFO(cString::Format("set hazzard light to %d , changed [%s]  ( hazzard: %d)", m_HazzardLightOn, m_HazzardLightOn_changed ? "true" : "false", m_HazzardLightOn));

    RETURN_NOERROR;
}

tResult cStateMachine::SetBrakeLight(bool status)
{
    SetLight(&m_BrakeLightOn, &m_BrakeLightOn_changed, status);

    RETURN_NOERROR;
}

tResult cStateMachine::SetHeadLight(bool status)
{
    SetLight(&m_HeadLightOn, &m_HeadLightOn_changed, status);

    RETURN_NOERROR;
}

tResult cStateMachine::SetReverseLight(bool status)
{
    SetLight(&m_ReverseLightOn, &m_ReverseLightOn_changed, status);

    RETURN_NOERROR;
}

tResult cStateMachine::SetTurnLeftSignal(bool status)
{
    SetLight(&m_TurnLeftSignalOn, &m_TurnLeftSignalOn_changed, status);
    //LOG_INFO(cString::Format("set turn light left to %d , changed [%s]  ( hazzard: %d)", m_TurnLeftSignalOn, m_TurnLeftSignalOn_changed ? "true" : "false", m_HazzardLightOn));

    RETURN_NOERROR;
}

tResult cStateMachine::SetTurnRightSignal(bool status)
{
    SetLight(&m_TurnRightSignalOn, &m_TurnRightSignalOn_changed, status);
    //LOG_INFO(cString::Format("set turn light right to %d , changed [%s]  ( hazzard: %d)", m_TurnRightSignalOn, m_TurnRightSignalOn_changed ? "true" : "false", m_HazzardLightOn));

    RETURN_NOERROR;
}

tResult cStateMachine::ResetAllLights()
{
    m_HazzardLightOn    = 0;
    m_BrakeLightOn      = 0;
    m_HeadLightOn       = 0;
    m_ReverseLightOn    = 0;
    m_TurnLeftSignalOn  = 0;
    m_TurnRightSignalOn = 0;

    m_HazzardLightOn_changed    = true;
    m_BrakeLightOn_changed      = true;
    m_HeadLightOn_changed       = true;
    m_ReverseLightOn_changed    = true;
    m_TurnLeftSignalOn_changed  = true;
    m_TurnRightSignalOn_changed = true;

    RETURN_NOERROR;
}

tResult cStateMachine::checkTickAndTimeStemps()
{
    // pedestrianSign detected less than 300 ticks ago
    if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 300)
    {
        m_actualSpeedTrafficSignDetection = DEFAULT_SPEED * 0.7;
        m_actualSpeed_changed = true;
    }
    else if(m_actualSpeedTrafficSignDetection != DEFAULT_SPEED && m_actualSpeedTrafficSignDetection != 0)
    {
        m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
    }

    // GiveWaySign detected less than 300 ticks ago && car passed the stopline
    if((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 300)
            && (m_actualWheelTicks > m_ticksOfNextCrosspoint))
    {
        // No car detected for at least 3 seconds
        if((adtf_util::cHighResTimer::GetTime() - m_lastTimeCarDetected) > 3000000)
        {
            m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
            m_actualSpeed_changed = true;
            m_ticksOfNextCrosspoint = INT32_MAX;
        }
        else  // car detected less than 3 seconds ago
        {
            m_actualSpeedCarDetection = 0;
            m_actualSpeed_changed = true;
            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
        }
    }
    // GiveWaySign detected less than 300 ticks ago && car did not passed the stopline yet
    else if((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 300)
            && (m_actualWheelTicks > m_ticksOfNextCrosspoint - 100))
    {
        m_actualSpeedTrafficSignDetection = DEFAULT_SPEED * 0.7;
        m_actualSpeed_changed = true;
    }

    // StopSign detected less than 300 ticks ago && car is at the stopline
    if((m_actualWheelTicks < m_lastTicksStopSignDetected + 300)
            && (m_actualWheelTicks > m_ticksOfNextCrosspoint))
    {
        //LOG_INFO(cString::Format("STOP!"));
        m_actualSpeedTrafficSignDetection = 0;
        m_actualSpeed_changed = true;
        m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
        m_ticksOfNextCrosspoint = INT32_MAX - 1000;
    }

    //    else if(m_actualWheelTicks < m_lastTicksStopSignDetected + 300)
    //    {
    //        LOG_INFO(cString::Format("Stop sign last 300 ticks, but no stop because %d < %d", m_actualWheelTicks, m_ticksOfNextCrosspoint));
    //    }
    //    else if(m_actualWheelTicks > m_ticksOfNextCrosspoint)
    //    {
    //        LOG_INFO(cString::Format("crosspoint reached, but no stop because last stopsign %d < %d", m_lastTicksStopSignDetected, m_actualWheelTicks));
    //    }

    // no child  detected for at least  200 ticks
    if(m_actualWheelTicks > (m_lastTicksChildDetected) + 200)
    {
        m_actualSpeedChildDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
        m_lastTicksChildDetected = INT32_MAX - 1000;
    }

    // no car detected for at least  200 ticks
    //    if(m_actualWheelTicks > (m_lastTicksCarDetected) + 200)
    //    {
    //        m_actualSpeedCarDetection = DEFAULT_SPEED;
    //        m_actualSpeed_changed = true;
    //        m_lastTicksCarDetected = INT32_MAX - 1000;
    //    }

    // no adult  detected for at least  200 ticks
    if(m_actualWheelTicks > (m_lastTicksAdultDetected) + 200)
    {
        m_actualSpeedAdultDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
        m_lastTicksAdultDetected = INT32_MAX - 1000;
    }

    // car stands still for at least 3 seconds, because another car was detected
    if((m_actualSpeedCarDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 3000000))
    {
        m_actualSpeedCarDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
    }

    if((m_lastTimeFollowCarDetected != INT32_MAX) && (cSystem::GetTime() - m_lastTimeFollowCarDetected) > 2000000)
    {
        //LOG_INFO(cString::Format("Folgefahrt beendet, last time follo w%d, dif %d", m_lastTimeFollowCarDetected, cSystem::GetTime() - m_lastTimeFollowCarDetected));
        m_actualSpeedCarDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
        m_lastTimeFollowCarDetected = INT32_MAX;
    }


    // car stands still for at least 3 seconds, because a child at a pedestrian crossing was detected
    if((m_actualSpeedChildDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 6000000))
    {
        m_actualSpeedChildDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
    }

    // car stands still for at least 3 seconds, because a adult at a pedestrian crossingS was detected
    if((m_actualSpeedAdultDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 6000000))
    {
        m_actualSpeedAdultDetection = DEFAULT_SPEED;
        m_actualSpeed_changed = true;
    }

    // car stands still for at least 3 seconds, because a stopSign was detected
    if((m_actualSpeedTrafficSignDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 3000000))
    {
        if((adtf_util::cHighResTimer::GetTime() - m_lastTimeCarDetected) > 3000000)
        {
            m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
            m_actualSpeed_changed = true;
        }
//        else
//        {
//            LOG_INFO(cString::Format("stop not released, car was in sight %d ago!  time since last stopp %d", adtf_util::cHighResTimer::GetTime() - m_lastTimeCarDetected, adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp));
//        }
    }
//    else if(m_actualSpeedTrafficSignDetection == 0)
//    {
//        LOG_INFO(cString::Format("stop not released, last time stop not more than 3 sec!  time since last stopp %d", adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp));
//    }

    // emergencyBreak is activated still at least x seconds
    //LOG_INFO(cString::Format("runstate: %i   primarystate %i", m_runState, m_primaryState));
    //LOG_INFO(cString::Format("cSystem::GetTime() - m_EmergencyBreakSince > m_overtakingTreshold    %i > %i", cSystem::GetTime()-m_EmergencyBreakSince ,m_overtakingTreshold));
    if((m_runState == runState_running) && (m_primaryState == primaryState_emergencyBreak)
            && (cSystem::GetTime() - m_EmergencyBreakSince > m_overtakingTreshold))
    {
        //if((m_actualDistances[0] > 50 || m_actualDistances[0] == -1) /*){&&(m_actualDistances[1] > 20 || m_actualDistances[1] == -1)*/){
            //LOG_INFO("OVERTAKING!!!!");
            ChangeRunState(runState_overtaking_check);
        //}
    }

    // there was no relevant traffic sign last 500 ticks => speed up!
    if(m_actualWheelTicks > (m_lastTicksRelevantSignDetected + 500))
    {
        if(m_actualSpeedUpFactor < m_maxSpeedUpFactor)
        {
            m_actualSpeedUpFactor = m_maxSpeedUpFactor;
            m_actualSpeed_changed = true;
            m_runState_changed = true;
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ChangePrimaryState(primaryStates newPrimaryState)
{
    if(m_primaryState != newPrimaryState)
    {
        m_primaryState = newPrimaryState;
        m_primaryState_changed = true;

        switch(newPrimaryState)
        {
        case primaryState_emergencyBreak:
        {
            if(m_runState != runState_ready && m_runState != runState_stop)
                m_EmergencyBreakSince = cSystem::GetTime();
            SetHazzardLight(true);
            SetBrakeLight(true);
            break;
        }

        case primaryState_run:
        {
            m_EmergencyBreakSince = INT32_MIN;
            SetHazzardLight(false);
            SetBrakeLight(false);
            break;
        }
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ChangeRunState(runStates newRunState)
{
    if(m_runState != newRunState)
    {
        m_runState = newRunState;
        m_runState_changed = true;

        switch(newRunState)
        {
        case runState_stop:
        {
            ResetAllLights();
            m_actualSpeedState = 0;
            m_actualSpeed_changed = true;
            m_EmergencyBreakSince = INT32_MIN;
            break;
        }

        case runState_ready:
        {
            ResetAllLights();
            SetHeadLight(true);
            SetBrakeLight(true);
            m_actualSpeedState = 0;
            m_actualSpeed_changed = true;
            m_EmergencyBreakSince = INT32_MIN;
            break;
        }

        case runState_running:
        {
            if(m_HeadLightOn == 0)
                SetHeadLight(true);
            SetBrakeLight(false);
            m_actualSpeedState = DEFAULT_SPEED;
            m_actualSpeed_changed = true;
            m_EmergencyBreakSince = INT32_MIN;
            break;
        }

        case runState_parking:
        {
            break;
        }

        case runState_overtaking_check:
        {
            TransmitManeuver(OVERTAKING_CHECK, -1);
            break;
        }

        case runState_overtaking_run:
        {

            m_actualSpeedLaneDetection = 0;
            m_actualSpeed_changed = true;
            updateSpeed();
            sleep(1);
            TransmitManeuver(OVERTAKING_RUN, -1);
            m_actualSpeedLaneDetection = 999;
            m_actualSpeedCarDetection = DEFAULT_SPEED;
            m_actualSpeed_changed = true;
            updateSpeed();
            //            if((m_actualDistances[0] > 50 || m_actualDistances[0] == -1 )&&(m_actualDistances[1] > 20 || m_actualDistances[1] == -1)){
            //                TransmitManeuver(OVERTAKING_RUN, -1);
            //            }
            //            else{
            //                LOG_INFO(cString::Format("overtaking check failed 0: %f, 1: %f", m_actualDistances[0], m_actualDistances[1]));
            //                TransmitManeuver(ResumeDefault, -1);
            //                ChangeRunState(runState_running);
            //            }
            break;
        }
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ComputeNextStep()
{
    if(m_emergencyBreakStatus_changed)
    {
        m_emergencyBreakStatus_changed = false;
        if(m_emergencyBreakStatus)
            ChangePrimaryState(primaryState_emergencyBreak);
        else
            ChangePrimaryState(primaryState_run);
    }

    if(m_runState_changed)
    {
        m_runState_changed = false;
        TransmitEmergencyBreakSet();
    }

    if(m_primaryState_changed)
    {
        m_primaryState_changed = false;
        if(m_primaryState == primaryState_emergencyBreak)
        {
            tFloat32 angle = m_ActualPosition.f32Heading  / -1.5 * 270;
            while(angle < 0)
                angle += 360;

            while(angle >= 360)
                angle -= 360;

            //            LOG_INFO(cString::Format("car angle %f, car :x %f , y %f, obstacle: x %f y %f", angle,  m_ActualPosition.f32X, m_ActualPosition.f32Y, m_ActualPosition.f32X + cos(angle) * 0.5, m_ActualPosition.f32Y + sin(angle) * 0.5));
            //            LOG_INFO(cString::Format("cos angle %f, sin angle %f", cos(radAngle), sin(radAngle)));

            float radAngle = DEG2RAD*angle;
            TransmitObstacle(m_ActualPosition.f32X + cos(radAngle) * 0.2, m_ActualPosition.f32Y + sin(radAngle) * 0.2);

        }
    }

    checkTickAndTimeStemps();

    updateSpeed();

    updateLights();

    RETURN_NOERROR;
}
