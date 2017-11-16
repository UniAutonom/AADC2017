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
// road sign distance and pose estimation
#define MP_LIMIT_ALPHA    70.0 // [degrees]
#define MP_LIMIT_YAW      15.0 // [degrees]
#define MP_LIMIT_YAW_INIT  8.0 // [degrees]
#define MP_LIMIT_DISTANCE  0.8 // [m]

/// Create filter shell
ADTF_FILTER_PLUGIN("State Machine", OID_ADTF_TEMPLATE_FILTER, cStateMachine)


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
}

cStateMachine::~cStateMachine()
{
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
        RETURN_IF_FAILED(pTypeJuryStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescJuryStruct));

        // input maneuver list
        tChar const * strDescManeuverList = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strDescManeuverList);
        cObjectPtr<IMediaType> pTypeManeuverList = new cMediaType(0, 0, 0, "tManeuverList", strDescManeuverList, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pTypeManeuverList, this));
        RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
        RETURN_IF_FAILED(pTypeManeuverList->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescManeuverList));

        // input EmergencyBreak status
        tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
        RETURN_IF_POINTER_NULL(strDescEmergencyStop);
        cObjectPtr<IMediaType> pTypeEmergencyStatus = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_EmergencyBreakStatusInputPin.Create("EmergencyBreakStatus", pTypeEmergencyStatus, this));
        RETURN_IF_FAILED(RegisterPin(&m_EmergencyBreakStatusInputPin));
        RETURN_IF_FAILED(pTypeEmergencyStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

        //input Emergency stop
        RETURN_IF_FAILED(m_EmergencyStopInputPin.Create("EmergencyStop", pTypeEmergencyStatus, this));
        RETURN_IF_FAILED(RegisterPin(&m_EmergencyStopInputPin));
        RETURN_IF_FAILED(pTypeEmergencyStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

        // output EmergencyBreak set
        tChar const * strDescEmergencyBreakSet = pDescManager->GetMediaDescription("tEmergencyBreakSet");
        RETURN_IF_POINTER_NULL(strDescEmergencyBreakSet);
        cObjectPtr<IMediaType> pTypeEmergencyBreakSet = new cMediaType(0, 0, 0, "tEmergencyBreakSet", strDescEmergencyBreakSet, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oOutputEmergencyBreakSet.Create("EmergencyBreakSet", pTypeEmergencyBreakSet, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputEmergencyBreakSet));
        RETURN_IF_FAILED(pTypeEmergencyBreakSet->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyBreakSet));

        // output driver struct
        tChar const * strDescDriverStruct = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDescDriverStruct);
        cObjectPtr<IMediaType> pTypeDriverStruct = new cMediaType(0, 0, 0, "tDriverStruct", strDescDriverStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pTypeDriverStruct, this));
        RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
        RETURN_IF_FAILED(pTypeDriverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescDriverStruct));

        // output maneuvers to lane detection
        tChar const * strDescManeuvers = pDescManager->GetMediaDescription("tManeuverValues");
        RETURN_IF_POINTER_NULL(strDescManeuvers);
        cObjectPtr<IMediaType> pTypeManeuvers = new cMediaType(0, 0, 0, "tManeuverValues", strDescManeuvers, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_ManeuverOutputPin.Create("SendManeuvers", pTypeManeuvers, this));
        RETURN_IF_FAILED(RegisterPin(&m_ManeuverOutputPin));
        RETURN_IF_FAILED(pTypeManeuvers->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuver));

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
        RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescPosition));

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

        // input finished maneuvers
        tChar const * strDescFinishedManeuvers = pDescManager->GetMediaDescription("tManeuverFinished");
        RETURN_IF_POINTER_NULL(strDescFinishedManeuvers);
        cObjectPtr<IMediaType> pTypeFinishedManeuvers = new cMediaType(0, 0, 0, "tManeuverFinished", strDescFinishedManeuvers, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputManeuverFinished.Create("FinishedManeuvers", pTypeFinishedManeuvers, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputManeuverFinished));
        RETURN_IF_FAILED(pTypeFinishedManeuvers->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuverFinished));

        //create and register pins for speed in
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputSpeedController.Create("Speed_in", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        //create and register pins for speed out
        RETURN_IF_FAILED(m_oOutputSpeedController.Create("Speed_out", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

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

        // input ticks to line
        tChar const * strTicksToLine = pDescManager->GetMediaDescription("tTicksToLine");
        RETURN_IF_POINTER_NULL(strTicksToLine);
        cObjectPtr<IMediaType> pTypeTicksToLine = new cMediaType(0, 0, 0, "tTicksToLine", strTicksToLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputTicksToLine.Create("Ticks_to_line", pTypeTicksToLine, this));
        RETURN_IF_FAILED(RegisterPin(&m_oInputTicksToLine));
        RETURN_IF_FAILED(pTypeTicksToLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTicksToLine));

        // input ultrasonic
        tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
        RETURN_IF_POINTER_NULL(strDescUsStruct);
        cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));
        RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
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
        m_primaryState = primaryState_run;
        m_runState     = runState_stop;
        m_actualSectorID = -1;
        m_actualManeuverID = -1;
        m_Emergency_Stop_Jury = false;
        m_actualSpeedState = 0;
        m_actualSpeedLaneDetection = 999;
        m_actualSpeedCarDetection = DEFAULT_SPEED;
        m_actualSpeedChildDetection = DEFAULT_SPEED;
        m_actualSpeedAdultDetection = DEFAULT_SPEED;
        m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
        m_lastTicksAdultDetected = INT32_MAX;
        m_lastTicksCarDetected = INT32_MAX;
        m_lastTicksChildDetected = INT32_MAX;
        m_lastTicksGiveWaySignDetected = INT32_MAX;
        m_lastTicksHaveWaySignDetected = INT32_MAX;
        m_lastTicksPedestrianSignDetected = INT32_MAX;
        m_lastTicksStopSignDetected = INT32_MAX;
        m_lastTimeStopp = 0;
        m_ticksOfNextLine = INT32_MAX;
        m_initTime = INT32_MAX;
        m_hasSentActualManeuver = false;
        m_emergencyBreakStatus = false;

        m_ActualPosition.f32X = 0;
        m_ActualPosition.f32Y = 0;
        m_ActualPosition.f32Heading = 0;
        m_ActualPosition.f32Radius = 0;
        m_ActualPosition.f32Speed = 0;
        m_pos_Initialized = false;

        // initialize translation and rotation vectors
        m_state = Mat(6,1,CV_64F,Scalar::all(0));
        m_Tvec = Mat(3,1,CV_32F,Scalar::all(0));
        m_Rvec = Mat(3,1,CV_32F,Scalar::all(0));
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

        TransmitEmergencyBreakSet();

        LoadRoadSignConfiguration();
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
        if (pSource == &m_JuryStructInputPin && m_pDescJuryStruct != NULL)
        {
            ProcessJuryInput(pMediaSample);
        }
        else if (pSource == &m_ManeuverListInputPin && m_pDescManeuverList != NULL)
        {
            ProcessManeuverList(pMediaSample);
        }
        else if (pSource == &m_EmergencyBreakStatusInputPin && m_pDescriptionEmergencyStop != NULL)
        {
            ProcessEmergencyBreakStatus(pMediaSample);
        }
        else if (pSource == &m_EmergencyStopInputPin && m_pDescriptionEmergencyStop != NULL)
        {
            ProcessEmergencyStop(pMediaSample);
        }
        else if (pSource == &m_oInputRoadSignExt && m_pDescriptionRoadSignExt != NULL)
        {
            ProcessRoadSignStructExt(pMediaSample);
        }
        else if (pSource == &m_oInputClassification)
        {
            ProcessClassificationInput(pMediaSample);
        }
        else if (pSource == &m_oInputPinPosition)
        {
            ProcessInputPosition(pMediaSample, 0);
            TransmitPosition(pMediaSample);
        }
        else if (pSource == &m_oInputManeuverFinished && m_pDescriptionManeuverFinished != NULL)
        {
            ProcessFinishedManeuver(pMediaSample);
        }
        else if(pSource == &m_oInputSpeedController && m_pDescriptionSignalValue != NULL)
        {
            ProcessSpeedController(pMediaSample);
        }
        else if (pSource == &m_oInputWheelLeft)
        {
            ProcessWheelSampleLeft(pMediaSample);
        }
        else if (pSource == &m_oInputWheelRight)
        {
            ProcessWheelSampleRight(pMediaSample);
        }
        else if (pSource == &m_oInputTicksToLine)
        {
            ProcessTicksToLine(pMediaSample);
        }
        else if (pSource == &m_oInputUsStruct)
        {
            ProcessUsValues(pMediaSample);
        }
    }

    //ComputeNextStep();

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessJuryInput(IMediaSample* pMediaSample)
{
    tInt8 i8ActionID = -2;
    tInt16 i16entry = -1;
    {
        // focus for sample read lock
        __adtf_sample_read_lock_mediadescription(m_pDescJuryStruct,pMediaSample,pCoder);

        if(!m_bIDsJuryStructSet)
        {
            pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
            pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
            m_bIDsJuryStructSet = tTrue;
        }

        pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
        pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
    }

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
    if((getActualManeuver().Compare("pull_out_left") == 0) || (getActualManeuver().Compare("pull_out_right") == 0) )
        checkActualManeuver();

    LOG_INFO(cString::Format("get maneuver %d in sector %d from jury!", m_actualManeuverID, m_actualSectorID));

    switch (juryActions(i8ActionID))
    {
        case action_GETREADY:
            //LOG_INFO(cString::Format("State Machine: Received Request Ready with maneuver ID %d",i16entry));
            ChangeState(false, false, true, false, false);
            SendState(stateCar_READY, tInt16(m_actualManeuverID));
            break;
        case action_START:
            //LOG_INFO(cString::Format("State Machine: Received Run with maneuver ID %d",i16entry));
            ChangeState(false, true, false, false, false);
            SendState(stateCar_RUNNING, tInt16(m_actualManeuverID));
            break;
        case action_STOP:
            //LOG_INFO(cString::Format("State Machine: Received Stop with maneuver ID %d",i16entry));
            ChangeState(false, false, false, true, false);
            SendState(stateCar_STARTUP, tInt16(m_actualManeuverID));
            break;
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessManeuverList(IMediaSample* pMediaSample)
{
    {
        std::vector<tSize> vecDynamicIDs;

        // focus for sample read lock
        __adtf_sample_read_lock_mediadescription(m_pDescManeuverList,pMediaSample,pCoder);

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

    // load maneuver list
    LoadManeuverList();

    RETURN_NOERROR;
}

tResult cStateMachine::SendState(stateCar stateID, tInt16 i16ManeuverEntry)
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

    switch (stateID)
    {
    case stateCar_READY:
        //LOG_INFO(cString::Format("State Machine: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
        break;
    case stateCar_RUNNING:
        //LOG_INFO(cString::Format("State Machine: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
        break;
    case stateCar_COMPLETE:
        //LOG_INFO(cString::Format("State Machine: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
        break;
    case stateCar_ERROR:
        //LOG_INFO(cString::Format("State Machine: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
        break;
    case stateCar_STARTUP:
        //LOG_INFO(cString::Format("State Machine: Send state: STARTUP, Maneuver ID %d",i16ManeuverEntry));
        break;
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

    LOG_INFO(cString::Format("State Machine: Loaded Maneuver file successfully with %d sectors.", m_sectorList.size()));

    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        tSector s = m_sectorList[i];
        LOG_INFO(cString::Format("sector %d with %d maneuvers.", s.id, s.maneuverList.size()));
        for(unsigned int k = 0; k < s.maneuverList.size(); k++)
        {
            tAADC_Maneuver m = s.maneuverList[k];
            LOG_INFO(cString::Format("Maneuver %d in Sektor %d with id %d: ", k, i, m.id).Append(m.action));
        }
    }


    if (oSectorElems.size() > 0)
    {
        //LOG_INFO(cString::Format("State Machine: Loaded Maneuver file successfully with %d sectors.", m_sectorList.size()));
        m_actualSectorID = 0;
        m_actualManeuverID = 0;
        //checkActualManeuver();
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

cString cStateMachine::getActualManeuver()
{
    if((m_actualSectorID == -1) || (m_actualManeuverID == -1)){
        LOG_INFO(cString::Format("get actual Maneuver  %d and sector %d", m_actualManeuverID, m_actualSectorID));
        return "";
    }

    tSector s = m_sectorList[m_actualSectorID];
    for(unsigned int k = 0; k < s.maneuverList.size(); k++)
    {
        tAADC_Maneuver m = s.maneuverList[k];
        if(m_actualManeuverID == m.id)
        {

            return m_sectorList[m_actualSectorID].maneuverList[k].action;
        }
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
            ChangeState(false, false, false, true, false);
            SendState(stateCar_COMPLETE, tInt16(m_actualManeuverID));
            RETURN_NOERROR;
        }
        else
        {
            m_actualSectorID++;
        }
    }

    ChangeState(false, false, false, false, true);

    m_actualManeuverID++;

    LOG_INFO(cString::Format("actual maneuver finished, GO on to next Maneuver %d in sector %d: ", m_actualManeuverID, m_actualSectorID).Append(getActualManeuver()));

    SendState(stateCar_RUNNING, tInt16(m_actualManeuverID));

    ChangeState(false, false, false, false, false);

    if((getActualManeuver().Compare("pull_out_left") == 0) || (getActualManeuver().Compare("pull_out_right") == 0) )
        checkActualManeuver();

    RETURN_NOERROR;
}

tResult cStateMachine::checkActualManeuver()
{
    if( m_hasSentActualManeuver != true)
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
        else if(cString::Compare("cross_parking", getActualManeuver(), 13) == 0)
        {
            int parkingspaceID = getActualManeuver().GetAt(14) - '0';
            //LOG_INFO(cString::Format("parking sign detected and maneuver, start parking at id %d!", parkingspaceID));
            while(parkingspaceID > 4)
                parkingspaceID -= 4;
            TransmitManeuver(PARKING_IN, parkingspaceID);
        }
        else if(actualManeuver.Compare("pull_out_left") == 0)
        {
            TransmitManeuver(PARKING_OUT_LEFT, -1);
        }
        else if(actualManeuver.Compare("pull_out_right") == 0)
        {
            TransmitManeuver(PARKING_OUT_RIGHT, -1);
        }
    }

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
        ChangeState(false, false, false, true, false);

    RETURN_NOERROR;
}


tResult cStateMachine::updateSpeed()
{
    tFloat32 newSpeed = DEFAULT_SPEED;
    if(abs(m_actualSpeedState) < abs(newSpeed))
        newSpeed = m_actualSpeedState;

    if(abs(m_actualSpeedAdultDetection) < abs(newSpeed))
        newSpeed = m_actualSpeedAdultDetection;

    if(abs(m_actualSpeedCarDetection) < abs(newSpeed))
        newSpeed = m_actualSpeedCarDetection;

    if(abs(m_actualSpeedChildDetection) < abs(newSpeed))
        newSpeed = m_actualSpeedChildDetection;

    if(abs(m_actualSpeedTrafficSignDetection) < abs(newSpeed))
        newSpeed = m_actualSpeedTrafficSignDetection;


    if(m_actualSpeedLaneDetection != 999 && m_actualSpeedState != 0)
    {
        newSpeed = m_actualSpeedLaneDetection;
    }

//    if(abs(newSpeed) < abs(DEFAULT_SPEED) && !m_BrakeLightSpeedOn)
//    {
//        m_BrakeLightSpeedOn = true;
//        if(!m_BrakeLightStateOn)
//            TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
//    }
//    else if(abs(newSpeed) >= abs(DEFAULT_SPEED) && m_BrakeLightSpeedOn)
//    {
//        m_BrakeLightSpeedOn = false;
//        if(!m_BrakeLightStateOn)
//            TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//    }

    if(newSpeed > 0 && !m_ReverseLightOn)
    {
        TransmitBoolValue(&m_oOutputReverseLight, true, 0);
        m_ReverseLightOn = true;
    }
    else if(newSpeed <= 0 && m_ReverseLightOn)
    {
        TransmitBoolValue(&m_oOutputReverseLight, false, 0);
        m_ReverseLightOn = false;
    }

    //LOG_INFO(cString::Format("speed %f", newSpeed));
    TransmitSpeed(newSpeed, 0);

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
    case runState_follow:
    {
        // set only the front sensors
//        newDistances[1] = 15;
        newDistances[2] = 30;
//        newDistances[3] = 15;
        //            newDistances[8] = 15;
        break;
    }

    case runState_turning_left:
    case runState_turning_right:
    {
        // set front and side sensors
        newDistances[0] = 10;
        newDistances[1] = 20;
        newDistances[2] = 20;
        newDistances[3] = 20;
        newDistances[4] = 10;
        //            newDistances[5] = 5;
        //            newDistances[6] = 5;
        break;
    }

    case runState_parking:
    {
        // set all sensors
        newDistances[0] = 10;
        newDistances[1] = 10;
        newDistances[2] = 10;
        newDistances[3] = 10;
        newDistances[4] = 10;
        //            newDistances[5] = 3;
        //            newDistances[6] = 3;
        newDistances[7] = 10;
        newDistances[8] = 10;
        newDistances[9] = 10;
        break;
    }

    case runState_overtaking_check:
    {
        break;
    }
    case runState_overtaking_run:
    {
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

tResult cStateMachine::ProcessInputPosition(IMediaSample* pMediaSampleIn, tTimeStamp tsInputTime)
{
    if(!m_pos_Initialized)
    {
        m_pos_Initialized = true;
    }

    tFloat32 f32x = 0;
    tFloat32 f32y = 0;
    tFloat32 f32radius = 0;
    tFloat32 f32speed = 0;
    tFloat32 f32heading = 0;

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

        pCoderInput->Get(m_szF32X, (tVoid*)&f32x);
        pCoderInput->Get(m_szF32Y, (tVoid*)&f32y);
        pCoderInput->Get(m_szF32Radius, (tVoid*)&f32radius);
        pCoderInput->Get(m_szF32Speed, (tVoid*)&f32speed);
        pCoderInput->Get(m_szF32Heading, (tVoid*)&f32heading);
    }

//    //LOG_INFO(cString::Format("x: %f, y: %f, radius: %f, speed: %f, heading: %f", f32x, f32y, f32radius, f32speed, f32heading));
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

tResult cStateMachine::updateLights()
{
    if(m_HazzardLightOn_changed)
    {
        if(m_HazzardLightOn > 0)
            TransmitBoolValue(&m_oOutputHazzardLight, true, 0);
        else
            TransmitBoolValue(&m_oOutputHazzardLight, false, 0);

        m_HazzardLightOn_changed = false;
    }

    if(m_BrakeLightOn_changed)
    {
        if(m_BrakeLightOn > 0)
            TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
        else
            TransmitBoolValue(&m_oOutputBrakeLight, false, 0);

        m_BrakeLightOn_changed = false;
    }
}

tResult cStateMachine::SetHazzardLight(bool status)
{
    if(status)
    {
        if(m_HazzardLightOn == 0)
            m_HazzardLightOn_changed = true;
        m_HazzardLightOn++;
    }
    else
    {
        if(m_HazzardLightOn == 1)
            m_HazzardLightOn_changed = true;
        m_HazzardLightOn--;
    }
}

tResult cStateMachine::SetBrakeLight(bool status)
{
    if(status)
    {
        if(m_BrakeLightOn == 0)
            m_BrakeLightOn_changed = true;
        m_BrakeLightOn++;
    }
    else
    {
        if(m_BrakeLightOn == 1)
            m_BrakeLightOn_changed = true;
        m_BrakeLightOn--;
    }
}

tResult cStateMachine::ChangePrimaryState(primaryStates newPrimaryState)
{
    if(m_primaryState != newPrimaryState)
    {
        switch(newPrimaryState)
        {
            case primaryState_emergencyBreak:
            {
                m_EmergencyBreakSince = cSystem::GetTime();
                SetHazzardLight(true);
                SetBrakeLight(true);
                break;
            }

            case primaryState_run:
            {
                m_EmergencyBreakSince = 0;
                SetHazzardLight(false);
                SetBrakeLight(false);
                break;
            }
        }

        m_primaryState = newPrimaryState;
    }
}

tResult cStateMachine::ChangeRunState(runStates newRunState)
{
    if(m_runState != newRunState)
    {
        switch(newRunState)
        {

        }
    }

}


tStatesStruct cStateMachine::ChangeState(bool emergencyBreakSet, bool start, bool ready, bool stop, bool maneuverCompleted){
//    cString actualManeuver = getActualManeuver();
//    bool x = false;
//    switch(m_primaryState)
//    {
//    case primaryState_emergencyBreak:
//    {
//        if(!emergencyBreakSet){
//            m_primaryState = primaryState_run;
//            if(m_HazzardLightStateOn)
//            {
//                m_HazzardLightStateOn = false;
//                if(!m_HazzardLightChildOn && !m_HazzardLightParkingOn){
//                    TransmitBoolValue(&m_oOutputHazzardLight, false, 0);
//                }
//            }
//            if(m_BrakeLightStateOn)
//            {
//                m_BrakeLightStateOn = false;
//                if(!m_BrakeLightSpeedOn)
//                    TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//            }
//        }

//        if(stop){
//            m_runState = runState_stop;
//            if(m_HeadLightOn)
//            {
//                TransmitBoolValue(&m_oOutputHeadLight, false, 0);
//                m_HeadLightOn = false;
//            }
//            if(m_BrakeLightStateOn)
//            {
//                m_BrakeLightStateOn = false;
//                if(!m_BrakeLightSpeedOn)
//                    TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//            }
//            m_actualSpeedState = 0;
//            break;
//        }

//        if(ready)
//        {
//            m_runState = runState_ready;
//            if(!m_HeadLightOn)
//            {
//                TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                m_HeadLightOn = true;
//            }
//            if(!m_BrakeLightStateOn)
//            {
//                m_BrakeLightStateOn = true;
//                if(!m_BrakeLightSpeedOn)
//                    TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
//            }
//            m_actualSpeedState = 0;
//        }

//        if(start)
//        {
//            m_runState = runState_follow;
//            if(!m_HeadLightOn)
//            {
//                TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                m_HeadLightOn = true;
//            }
//            if(m_BrakeLightStateOn)
//            {
//                m_BrakeLightStateOn = false;
//                if(!m_BrakeLightSpeedOn)
//                    TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//            }
//            m_actualSpeedState = DEFAULT_SPEED;
//        }
//        break;
//    }

//    case primaryState_run:
//    {
//        if(stop){
//            m_runState = runState_stop;
//            if(m_HeadLightOn)
//            {
//                TransmitBoolValue(&m_oOutputHeadLight, false, 0);
//                m_HeadLightOn = false;
//            }
//            if(m_BrakeLightStateOn)
//            {
//                m_BrakeLightStateOn = false;
//                if(!m_BrakeLightSpeedOn)
//                    TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//            }
//            m_actualSpeedState = 0;

//            break;
//        }

//        if(emergencyBreakSet){
//            m_primaryState = primaryState_emergencyBreak;
//            if(!m_HazzardLightStateOn)
//            {
//                m_HazzardLightStateOn = true;
//                if(!m_HazzardLightChildOn && !m_HazzardLightParkingOn){
//                    TransmitBoolValue(&m_oOutputHazzardLight, true, 0);
//                }
//            }
//            if(!m_BrakeLightStateOn)
//            {
//                m_BrakeLightStateOn = true;
//                if(!m_BrakeLightSpeedOn)
//                    TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
//            }
//            break;
//        }

//        switch(m_runState)
//        {
//        case runState_stop:
//        {
//            if(ready)
//            {
//                m_runState = runState_ready;
//                if(!m_HeadLightOn)
//                {
//                    TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                    m_HeadLightOn = true;
//                }
//                if(!m_BrakeLightStateOn)
//                {
//                    m_BrakeLightStateOn = true;
//                    if(!m_BrakeLightSpeedOn)
//                        TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
//                }
//                m_actualSpeedState = 0;
//            }
//            break;
//        }

//        case runState_ready:
//        {
//            if(start)
//            {
//                m_runState = runState_follow;
//                if(!m_HeadLightOn)
//                {
//                    TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                    m_HeadLightOn = true;
//                }
//                if(m_BrakeLightStateOn)
//                {
//                    m_BrakeLightStateOn = false;
//                    if(!m_BrakeLightSpeedOn)
//                        TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//                }
//                m_actualSpeedState = DEFAULT_SPEED;
//            }
//            break;
//        }

//        case runState_follow:
//        {
//            if(actualManeuver.Compare("left") == 0){
//                m_runState = runState_turning_left;
//            }
//            else if(actualManeuver.Compare("right") == 0){
//                m_runState = runState_turning_right;
//            }
//            else if(actualManeuver.Compare("cross_parking") == 0){
//                m_runState = runState_parking;
//            }
//            break;
//        }

//        case runState_turning_left:
//        {
//            if(maneuverCompleted == true){
//                m_runState = runState_follow;
//                if(!m_HeadLightOn)
//                {
//                    TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                    m_HeadLightOn = true;
//                }
//                if(m_BrakeLightStateOn)
//                {
//                    m_BrakeLightStateOn = false;
//                    if(!m_BrakeLightSpeedOn)
//                        TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//                }
//                m_actualSpeedState = DEFAULT_SPEED;
//            }
//            break;
//        }

//        case runState_turning_right:
//        {
//            if(maneuverCompleted == true){
//                m_runState = runState_follow;
//                if(!m_HeadLightOn)
//                {
//                    TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                    m_HeadLightOn = true;
//                }
//                if(m_BrakeLightStateOn)
//                {
//                    m_BrakeLightStateOn = false;
//                    if(!m_BrakeLightSpeedOn)
//                        TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//                }
//                m_actualSpeedState = DEFAULT_SPEED;
//            }
//            break;
//        }

//        case runState_parking:
//        {
//            if(actualManeuver.Compare("pull_out_left") == 0){
//                m_runState = runState_turning_left;
//            }
//            else if(actualManeuver.Compare("pull_out_right") == 0){
//                m_runState = runState_turning_right;
//            }
//            break;
//        }

//        case runState_overtaking_check:
//        {
//            if(maneuverCompleted == true){
//                m_runState = runState_overtaking_run;
//                if((m_actualDistances[0] > 60 || m_actualDistances[0] == -1 )&&(m_actualDistances[1] > 20 || m_actualDistances[1] == -1)){
//                    TransmitManeuver(OVERTAKING_RUN, -1);
//                    TransmitEmergencyBreakSet();
//                }
//                else{
//                    x = true;
//                    TransmitManeuver(ResumeDefault, -1);
//                    m_runState = runState_follow;
//                    if(!m_HeadLightOn)
//                    {
//                        TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                        m_HeadLightOn = true;
//                    }
//                    if(m_BrakeLightStateOn)
//                    {
//                        m_BrakeLightStateOn = false;
//                        if(!m_BrakeLightSpeedOn)
//                            TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//                    }
//                    m_actualSpeedState = DEFAULT_SPEED;
//                }
//            }
//            break;
//        }

//        case runState_overtaking_run:
//        {
//            if(maneuverCompleted == true){
//                m_runState = runState_follow;
//                if(!m_HeadLightOn)
//                {
//                    TransmitBoolValue(&m_oOutputHeadLight, true, 0);
//                    m_HeadLightOn = true;
//                }
//                if(m_BrakeLightStateOn)
//                {
//                    m_BrakeLightStateOn = false;
//                    if(!m_BrakeLightSpeedOn)
//                        TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
//                }
//                m_actualSpeedState = DEFAULT_SPEED;
//            }
//            break;
//        }
//        }
//        if(!x)
//            TransmitEmergencyBreakSet();
//        break;
//    }
//    }

//    //LOG_INFO(cString::Format("switched to state %d-%d", m_primaryState, m_runState));
//    updateSpeed();

    tStatesStruct actualState;
    actualState.i8PrimaryState = m_primaryState;
    actualState.i8RunState = m_runState;
    return actualState;
}

/*! calculates normalized angle */
tFloat32 cStateMachine::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha-center+ static_cast<tFloat32>(M_PI), 2.0*static_cast<tFloat32>(M_PI)) + center- static_cast<tFloat32>(M_PI);
}

tFloat32 cStateMachine::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0f)
        {
            b_x = ceil(r - 0.5f);
        }
        else
        {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r))
        {
            return 0.0f;
        }
        else
        {
            return (r - floor(r)) * y;
        }
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
    if (m_ui32Cnt<5)
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
    tFloat32 yawE;

    tFloat32 pitch1  = -asin(R.at<float>(2,0));
    tFloat32 pitch2  = (tFloat32)normalizeAngle(static_cast<tFloat32>(CV_PI) - pitch1, 0.0f);

    tFloat32 yaw1  = atan2(R.at<float>(2,1) / cos(pitch1), R.at<float>(2,2) / cos(pitch1));
    tFloat32 yaw2  = atan2(R.at<float>(2,1) / cos(pitch2), R.at<float>(2,2) / cos(pitch2));

    // select shortest rotation for yaw
    if (abs(yaw1) <= abs(yaw2))
    {
        yawE   = yaw1*RAD2DEG;
    }
    else
    {
        yawE   = yaw2*RAD2DEG;
    }
    //LOG_INFO(cString::Format("Roadsign %d ANGLE %f", m_i16ID, a0));


    // check angle and distance limit
    if(m_f32MarkerSize < 1500)
    {
        //LOG_INFO(cString::Format("Roadsign %d rejected with marker size %f", m_i16ID, m_f32MarkerSize));
        RETURN_NOERROR;
    }

    //LOG_INFO(cString::Format("Roadsign %d accepted with marker size %f", m_i16ID, m_f32MarkerSize));
    //LOG_INFO(cString::Format("a0 %f, yaw1 %f, yaw2 %f, yawE %f, pitch1 %f, pitch2 %f", a0, yaw1, yaw2, yawE, pitch1, pitch2));

    switch(m_i16ID)
    {
        case MARKER_ID_PARKINGAREA:
            checkActualManeuver();
            break;

        case MARKER_ID_GIVEWAY:
            m_lastTicksGiveWaySignDetected = (wheelCountLeft + wheelCountRight) / 2;
            //LOG_INFO(cString::Format("giveway sign detected at ticks: %d", m_lastTicksGiveWaySignDetected));
            checkActualManeuver();
            break;

        case MARKER_ID_STOPANDGIVEWAY:
            m_lastTicksStopSignDetected = (wheelCountLeft + wheelCountRight) / 2;
            //LOG_INFO(cString::Format("Stop sign detected at ticks: %d", m_lastTicksStopSignDetected));
            checkActualManeuver();
            break;

        case MARKER_ID_PEDESTRIANCROSSING:
            m_lastTicksPedestrianSignDetected = (wheelCountLeft + wheelCountRight) / 2;
            //LOG_INFO(cString::Format("pedestrian sign detected at ticks: %d", m_lastTicksPedestrianSignDetected));
            TransmitManeuver(PEDESTRIAN_CROSSING, -1);
            break;

        case MARKER_ID_HAVEWAY:
            m_lastTicksHaveWaySignDetected = (wheelCountLeft + wheelCountRight) / 2;
            //LOG_INFO(cString::Format("have way sign detected at ticks: %d", m_lastTicksHaveWaySignDetected));
            checkActualManeuver();
            break;
    }

    // calculate heading wrt marker
    tFloat32 heading = static_cast<tFloat32>(m_state.at<double>(2) + a0*DEG2RAD);
    heading = normalizeAngle(heading,0);

    // estimate marker location based on current vehicle location
    // and marker measurement
    tFloat32 x0 = static_cast<tFloat32>(m_state.at<double>(0)+cos(heading)*d0);
    tFloat32 y0 = static_cast<tFloat32>(m_state.at<double>(1)+sin(heading)*d0);
    TransmitTrafficSign(m_i16ID, x0, y0, heading);

    //LOG_INFO(cString::Format("Found Sign with id %d and x %f and y %f", m_i16ID, x0, y0));
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

                if(name.Compare("car") == 0)
                {
                    int heightPixel = 800;
                    int widthPixelLeft = 400;
                    int widthPixelRight = 1500;

                    //LOG_INFO(cString::Format("x %i y%i height %i width %i", it->boundingBox.x, it->boundingBox.y, it->boundingBox.height, it->boundingBox.width));

                    if(it->boundingBox.x > widthPixelLeft && (it->boundingBox.x + it->boundingBox.width) < widthPixelRight && m_actualSpeedCarDetection < 0) // car is in center
                    {

                        //LOG_INFO(cString::Format("ignolwjngvowng %f ", (heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f)));
                        if(m_actualSpeedCarDetection<0){
                            if((heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f) <= 0){
                                //LOG_INFO("IF");
                                //langsamer

                                m_actualSpeedCarDetection -= floor(heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f);
                                if(m_actualSpeedCarDetection < -1){
                                    m_actualSpeedCarDetection = -1;
                                }
                            }
                            else{
                                //LOG_INFO("ELSE");
                                //schneller
                                m_actualSpeedCarDetection -= ceil(heightPixel/1000.0f - (it->boundingBox.y + it->boundingBox.height)/1000.0f);
                                if(m_actualSpeedCarDetection <= DEFAULT_SPEED){
                                    m_actualSpeedCarDetection = DEFAULT_SPEED;
                                }
                            }
                        }else if(m_actualSpeedCarDetection == 0){
                            if((it->boundingBox.y + it->boundingBox.height) < heightPixel){
                                //LOG_INFO("HAAAALLLLOOOOO");
                                m_actualSpeedCarDetection = DEFAULT_SPEED;
                            }
                        }else{
                            m_actualSpeedCarDetection = DEFAULT_SPEED;
                        }

                        m_lastTicksCarDetected = (wheelCountLeft + wheelCountRight) / 2;
                        //LOG_INFO(cString::Format("Folgefahrt Speed: %f", m_actualSpeedCarDetection));


                        // check if there was a GiveWay oder Stop sign at the last 200 ticks, then stop!
                        if((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 200)
                                || (m_actualWheelTicks < m_lastTicksStopSignDetected + 200))
                        {
                            m_actualSpeedCarDetection = 0;
                            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                        }
                        else if(m_actualWheelTicks < m_lastTicksHaveWaySignDetected + 200){
                            if(getActualManeuver().Compare("left") == 0){
                                m_actualSpeedCarDetection = 0;
                                m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                            }
                        }
                    }
                }
                else if(name.Compare("adult") == 0)
                {
                    //nothing to do
                }
                else if(name.Compare("adult_left") == 0)
                {
                    // Auskommentiert weil wir das nur brauchen wenn der adult an einem zebrastreifen steht
                    if((it->boundingBox.x) > 0.7f*1920) // adult_left is at right side of the street
                    {
                        if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                        {
                            m_actualSpeedAdultDetection = 0;  // stop, if there is a adult_right at the left side of a pedestrian crossing
                            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                        }
                        m_lastTicksAdultDetected = (wheelCountLeft + wheelCountRight) / 2;
                    }
                }
                else if(name.Compare("adult_right") == 0)
                {
                    // Auskommentiert weil wir das nur brauchen wenn der adult an einem zebrastreifen steht
                    if((it->boundingBox.x + it->boundingBox.width) < 0.3f*1920) // adult_right is at left side of the street
                    {
                        if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                        {
                            m_actualSpeedAdultDetection = 0;    // stop, if there is a adult_right at the left side of a pedestrian crossing
                            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                        }
                        m_lastTicksAdultDetected = (wheelCountLeft + wheelCountRight) / 2;
                    }
                }
                else if(name.Compare("child") == 0)
                {
                    int heightPixel = 650;
                    //int widthPixelLeft = 400;
                    int widthPixelRight = 1500;
                    if(it->boundingBox.y > heightPixel && /*it->boundingBox.x > widthPixelLeft && */ (it->boundingBox.x + it->boundingBox.width) < widthPixelRight) // child is near car
                    {
                        //TODO
                    }

                    if(m_actualWheelTicks < m_lastTicksPedestrianSignDetected + 200)
                    {
                        m_actualSpeedChildDetection = 0;    // stop, if there is a child at the pedestrian crossing
                        m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
                    }
                    else
                    {
                        m_actualSpeedChildDetection = DEFAULT_SPEED  * 0.65f;
                    }

//                    if(!m_HazzardLightChildOn)
//                    {
//                        m_HazzardLightChildOn = true;
//                        if(!m_HazzardLightStateOn && !m_HazzardLightParkingOn){
//                            TransmitBoolValue(&m_oOutputHazzardLight, true, 0);
//                            LOG_INFO("HAZZARD LIGHT ENABLED");
//                        }
//                    }
                    m_lastTicksChildDetected = (wheelCountLeft + wheelCountRight) / 2;
                }
            }
        }
        //updateSpeed();
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessSpeedController(IMediaSample* pMediaSample)
{

    tFloat32 speed;
    tUInt32 timestamp;

    //sobald der block verlassen wird, wird das lock aufgehoben
    {
        //read lock
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
    updateSpeed();

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

    LOG_INFO(cString::Format(" received finsihed maneuver %d    %d", maneuver, finished));

    m_actualSpeedLaneDetection = 999;

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
//                if(!m_HazzardLightParkingOn)
//                {
//                    m_HazzardLightParkingOn = true;
//                    if(!m_HazzardLightStateOn && !m_HazzardLightChildOn){
//                        TransmitBoolValue(&m_oOutputHazzardLight, true, 0);
//                        LOG_INFO("HAZZARD LIGHT ENABLED");
//                    }
//                }
                m_actualSpeedLaneDetection = 0;
                updateSpeed();
                // TODO TransmitParkingSpace()
                sleep(3);

//                if(m_HazzardLightParkingOn)
//                {
//                    m_HazzardLightParkingOn = false;
//                    if(!m_HazzardLightStateOn && !m_HazzardLightChildOn){
//                        TransmitBoolValue(&m_oOutputHazzardLight, false, 0);
//                        LOG_INFO("HAZZARD LIGHT ENABLED");
//                    }
//                }
                // TODO TransmitParkingSpace()
                setActualManeuverCompleted();
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
            ChangeState(false, false, false, false, true);
        case OVERTAKING_RUN:
            ChangeState(false, false, false, false, true);
        }
    }
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

    if(maneuverNumber != PEDESTRIAN_CROSSING)
    {
        m_hasSentActualManeuver = true;
    }

    LOG_INFO(cString::Format("State Machine transmitted maneuver %d to lane detection", maneuverNumber));
    RETURN_NOERROR;
}

tResult cStateMachine::TransmitPosition(IMediaSample* pMediaSample)
{
    m_OutputPostion.Transmit(pMediaSample);
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

    //ComputeNextStep();

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

    m_actualWheelTicks = ((wheelCountLeft + wheelCountRight) / 2);

    //ComputeNextStep();

    RETURN_NOERROR;
}

tResult cStateMachine::ProcessTicksToLine(IMediaSample* pMediaSample)
{
    int ticksToNextLine;
    static bool hasID = false;
    static tBufferID szIDTicksToLine;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionTicksToLine, pMediaSample, pCoderInput);

        if (!hasID)
        {
            pCoderInput->GetID("i32ticksToLine", szIDTicksToLine);
            hasID = tTrue;
        }

        pCoderInput->Get(szIDTicksToLine, (tVoid*)&ticksToNextLine);
    }
    //LOG_INFO(cString::Format("%d ticks to next line received!", ticksToNextLine));

    m_ticksOfNextLine = ticksToNextLine + m_actualWheelTicks;

    //LOG_INFO(cString::Format("%d ticks of next line!", m_ticksOfNextLine));

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

    static int count = 0;
    count++;
    if(count < 10)  // ignore first 10 samples
        RETURN_NOERROR;

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

tResult cStateMachine::checkTickAndTimeStemps()
{
    if((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 300)
            && (m_actualWheelTicks > m_ticksOfNextLine))
    {
        if(adtf_util::cHighResTimer::GetTime() - m_lastTimeCarDetected > 3000000)
        {
            //LOG_INFO(cString::Format("giveway limit rejected!"));
            m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
            m_ticksOfNextLine = INT32_MAX;
        }
        else
        {
            m_actualSpeedCarDetection = 0;
            m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
        }
    }
    else if((m_actualWheelTicks < m_lastTicksGiveWaySignDetected + 300)
            && (m_actualWheelTicks > m_ticksOfNextLine - 100))
    {
        //LOG_INFO(cString::Format("giveway detected and ticks under 100!"));
        m_actualSpeedTrafficSignDetection = DEFAULT_SPEED * 0.7;
    }

    if((m_actualWheelTicks < m_lastTicksStopSignDetected + 300)
            && (m_actualWheelTicks > m_ticksOfNextLine))
    {
        //LOG_INFO(cString::Format("stopsign detected , stop!!!!"));
        m_actualSpeedTrafficSignDetection = 0;
        m_lastTimeStopp = adtf_util::cHighResTimer::GetTime();
        m_ticksOfNextLine = INT32_MAX - 1000;
    }

    if((wheelCountLeft + wheelCountRight) / 2 > (m_lastTicksChildDetected) + 200)
    {
//        if(m_HazzardLightChildOn)
//        {
//            m_HazzardLightChildOn = false;
//            if(!m_HazzardLightParkingOn && !m_HazzardLightStateOn){
//                TransmitBoolValue(&m_oOutputHazzardLight, false, 0);
//            }
//        }
        m_actualSpeedChildDetection = DEFAULT_SPEED;
        m_lastTicksChildDetected = INT32_MAX - 1000;
    }

    if((wheelCountLeft + wheelCountRight) / 2 > (m_lastTicksCarDetected) + 200)
    {
        m_actualSpeedCarDetection = DEFAULT_SPEED;
        m_lastTicksCarDetected = INT32_MAX - 1000;
    }

    if((wheelCountLeft + wheelCountRight) / 2 > (m_lastTicksAdultDetected) + 200)
    {
        m_actualSpeedAdultDetection = DEFAULT_SPEED;
        m_lastTicksAdultDetected = INT32_MAX - 1000;
    }

    if((m_actualSpeedCarDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 3000000))
    {
        m_actualSpeedCarDetection = DEFAULT_SPEED;
    }

    if((m_actualSpeedChildDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 3000000))
    {
        m_actualSpeedChildDetection = DEFAULT_SPEED;
    }

    if((m_actualSpeedAdultDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 3000000))
    {
        m_actualSpeedAdultDetection = DEFAULT_SPEED;
    }

    if((m_actualSpeedTrafficSignDetection == 0) && (adtf_util::cHighResTimer::GetTime() - m_lastTimeStopp > 3000000))
    {
        if(adtf_util::cHighResTimer::GetTime() - m_lastTimeCarDetected > 3000000)
        {
            //LOG_INFO(cString::Format("stop released!"));
            m_actualSpeedTrafficSignDetection = DEFAULT_SPEED;
        }
    }

    if(m_runState != runState_overtaking_check && m_EmergencyBreakSince != 0 && (cSystem::GetTime() - m_EmergencyBreakSince > 15000000))
    {
        if((m_actualDistances[0] > 50 || m_actualDistances[0] == -1) /*){&&(m_actualDistances[1] > 20 || m_actualDistances[1] == -1)*/){
            LOG_INFO("OVERTAKING!!!!");
            m_runState = runState_overtaking_check;
            TransmitEmergencyBreakSet();
            TransmitManeuver(OVERTAKING_CHECK, -1);
        }
    }

    RETURN_NOERROR;
}

tResult cStateMachine::ComputeNextStep()
{
    if(m_emergencyBreakStatus_changed)
    {
        if(m_emergencyBreakStatus)
            ChangePrimaryState(primaryState_emergencyBreak);
        else
            ChangePrimaryState(primaryState_run);

        m_emergencyBreakStatus_changed = false;
    }

    updateLights();


//    checkTickAndTimeStemps();
//    updateSpeed();

    RETURN_NOERROR;
}

