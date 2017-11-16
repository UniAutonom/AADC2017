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
* $Author:: spiesra $  $Date:: 2017-04-26 13:55:11#$ $Rev:: 62570   $
**********************************************************************/
#include "stdafx.h"
#include "AADC_LaneTracking.h"

#define DISTANCE_LENGTH 4
#define MIN_FREE_DISTANCE 1.5f


ADTF_FILTER_PLUGIN("AADC LaneTracking Jury", OID_ADTF_LaneTracking_jury, cLaneTracking)

	// define the ADTF property names to avoid errors 
#define LT_PROP_NEAR_LINE "LaneDetection::Near area::Near Line"
#define LT_PROP_NEAR_LINE_MAX_OFFSET "LaneDetection::Near area::Max offset for near line"
#define LT_PROP_LANE_WIDTH_MIN_NEAR "LaneDetection::Near area::Lane width min near"
#define LT_PROP_LANE_WIDTH_MAX_NEAR "LaneDetection::Near area::Lane width max near"

#define LT_PROP_FAR_LINE "LaneDetection::Far area::Far Line"
#define LT_PROP_LANE_WIDTH_MIN_FAR "LaneDetection::Far area::Lane width min far"
#define LT_PROP_LANE_WIDTH_MAX_FAR "LaneDetection::Far area::Lane width max far"

#define LT_PROP_CAMERA_OFFSET "LaneDetection::Camera Offset"
#define LT_PROP_TRESHOLD "LaneDetection::ThresholdValue"

#define LT_PROP_US_ENABLE "US::Enable US Detection"
#define LT_PROP_US_MINIMUM_FREE_DISTANCE "US::Minimum free distance"

#define LT_PROP_MAX_ACCELERATION "LongitudinalControl::Max Acceleration"
#define LT_PROP_MIN_ACCELERATION "LongitudinalControl::Min Acceleration"
#define LT_PROP_ACCELERATION_FAR_NEAR_DIFF "LongitudinalControl::Far Near difference"

#define LT_PROP_SHOW_DEBUG "Common::Show Debug"
#define LT_PROP_ENABLE_LIGHTBEAM_TRIGGER "Common::Enable Lightbeam Trigger"
#define LT_PROP_DRIVE_TIME "Common::Drive Time in milliseconds"
#define LT_PROP_DRIVE_DISTANCE "Common::Drive Distance in meters"
#define LT_PROP_EMERGENCY_STOP_TIME "Common::Emergency Stop Time in milliseconds"	
#define LT_PROP_STARTUP_STATE "Common::Startup State"
#define LT_PROP_STOP_MODE "Common::Stop on Distance or Time"

#define LT_PROP_CONTR_PROPORTIONAL_GAIN "PID::Controller Proportional Gain"
#define LT_PROP_CONTR_INTEGRAL_GAIN "PID::Controller Integral Gain"
#define LT_PROP_CONTR_DIFFERENTIAL_GAIN "PID::Controller Differential Gain"

#define LT_PROP_PT1_TAU "PT1::Tau"
#define LT_PROP_PT1_SAMPLE_TIME "PT1::Sample_Time"
#define LT_PROP_PT1_INPUT_FACTOR "PT1::InputFactor"

#define LT_PROP_LIGHTBEAM_ONLY_ONCE "Common::Lightbeam Trigger only once"

	// id and name definitions for signal registry (increase the id for new signals)
#define LT_SIGREG_ID_SCALED_PT1_INPUT 0
#define LT_SIGREG_NAME_SCALED_PT1_INPUT "scaled pt1 input"
#define LT_SIGREG_UNIT_SCALED_PT1_INPUT "pixel"

#define LT_SIGREG_ID_CONTROLLER_INPUT 1
#define LT_SIGREG_NAME_CONTROLLER_INPUT "controller input"
#define LT_SIGREG_UNIT_CONTROLLER_INPUT "pixel"



#define MAX_DEVIATION 150

	// used to identify the braking timer
	const tUInt8 g_ui8StopTimerIdNegative = 77;
const tUInt8 g_ui8StopTimerIdZero = 78;
const tUInt8 g_ui8EmergencyStopTimerIdNegative = 87;
const tUInt8 g_ui8EmergencyStopTimerIdZero = 88;
const tUInt8 g_ui8EmergencyStopTimerIdResume = 89;

//#define LT_ENABLE_CANNY_WINDOWS


cLaneTracking::cLaneTracking(const tChar* __info) : cFilter(__info), 
	m_hStopTimerNegative(NULL),
	m_hStopTimerZero(NULL),
	m_hEmergencyStopTimerNegative(NULL),
	m_hEmergencyStopTimerZero(NULL),
	m_hEmergencyStopTimerResume(NULL),
	m_usCurrentDistance(0),
	m_usMinFreeThreshold(MIN_FREE_DISTANCE),
	m_usDetectionEnabled(tFalse),
	m_bLightBeamOnlyOnce(tFalse),
	m_bWasLightbeamTriggered(tFalse),
	m_iStopMode(1),
	m_f32DistanceStartValue(0),
	m_f32CurrentDistance(0)
{
	    // create the filter properties
    SetPropertyInt(LT_PROP_STOP_MODE, 1);
    SetPropertyStr(LT_PROP_STOP_MODE NSSUBPROP_VALUELISTNOEDIT, "1@Distance|2@Time");
    SetPropertyStr(LT_PROP_STOP_MODE NSSUBPROP_DESCRIPTION, "Change when to stop again after time or distance"); 

	
	SetPropertyInt(LT_PROP_NEAR_LINE, 350);
	SetPropertyBool(LT_PROP_NEAR_LINE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_NEAR_LINE NSSUBPROP_DESCRIPTION, "The y value of the near line used for lateral control.");

	SetPropertyInt(LT_PROP_NEAR_LINE_MAX_OFFSET, -25);
	SetPropertyBool(LT_PROP_NEAR_LINE_MAX_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_NEAR_LINE_MAX_OFFSET NSSUBPROP_DESCRIPTION, "The maximum offset to adjust near line to the current speed.");

	SetPropertyInt(LT_PROP_FAR_LINE, 250);
	SetPropertyBool(LT_PROP_FAR_LINE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_FAR_LINE NSSUBPROP_DESCRIPTION, "The y value of the far line used for longitudinal control.");

	SetPropertyBool(LT_PROP_SHOW_DEBUG, tFalse);
	SetPropertyStr(LT_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");

	SetPropertyBool(LT_PROP_ENABLE_LIGHTBEAM_TRIGGER, tFalse);
	SetPropertyStr(LT_PROP_ENABLE_LIGHTBEAM_TRIGGER NSSUBPROP_DESCRIPTION, "If true, start_trigger pin will be used to start driving.");

	SetPropertyInt(LT_PROP_DRIVE_TIME, 25000);
	SetPropertyInt(LT_PROP_DRIVE_TIME NSSUBPROP_MIN, 0);
	SetPropertyStr(LT_PROP_DRIVE_TIME NSSUBPROP_DESCRIPTION, "If enable lightbeam trigger is set to true, this value will be used to stop driving after the given time." \
		"If the value is 0, the stop trigger is disabled.");
	
	SetPropertyFloat(LT_PROP_DRIVE_DISTANCE, 8);
	SetPropertyInt(LT_PROP_DRIVE_DISTANCE NSSUBPROP_MIN, 0);
	SetPropertyStr(LT_PROP_DRIVE_DISTANCE NSSUBPROP_DESCRIPTION, "The distance in meter after that the car stops");

	SetPropertyInt(LT_PROP_EMERGENCY_STOP_TIME, 0);
	SetPropertyInt(LT_PROP_EMERGENCY_STOP_TIME NSSUBPROP_MIN, 0);
	SetPropertyStr(LT_PROP_EMERGENCY_STOP_TIME NSSUBPROP_DESCRIPTION, "If enable lightbeam trigger is set to true, this value will be used to perform an emergency stop after the given time." \
		"If the value is 0, the emergency stop trigger is disabled.");

	SetPropertyFloat(LT_PROP_MAX_ACCELERATION, 2);
	SetPropertyBool(LT_PROP_MAX_ACCELERATION NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_MAX_ACCELERATION NSSUBPROP_DESCRIPTION, "Acceleration value used on a straight.");

	SetPropertyFloat(LT_PROP_US_MINIMUM_FREE_DISTANCE,MIN_FREE_DISTANCE);
	SetPropertyBool(LT_PROP_US_MINIMUM_FREE_DISTANCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_US_MINIMUM_FREE_DISTANCE NSSUBPROP_DESCRIPTION, "Free Distance");

	SetPropertyBool(LT_PROP_US_ENABLE,tFalse);
	SetPropertyBool(LT_PROP_US_ENABLE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_US_ENABLE NSSUBPROP_DESCRIPTION, "If us free space detection is enabled");
	
	SetPropertyBool(LT_PROP_STARTUP_STATE,tFalse);
	SetPropertyBool(LT_PROP_STARTUP_STATE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_STARTUP_STATE NSSUBPROP_DESCRIPTION, "If is enabled after startup or not");
    
	SetPropertyBool(LT_PROP_LIGHTBEAM_ONLY_ONCE,tTrue);
	SetPropertyBool(LT_PROP_LIGHTBEAM_ONLY_ONCE NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_LIGHTBEAM_ONLY_ONCE NSSUBPROP_DESCRIPTION, "If lightbeam can be triggered only once");

	SetPropertyFloat(LT_PROP_MIN_ACCELERATION, 0.5);
	SetPropertyBool(LT_PROP_MIN_ACCELERATION NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_MIN_ACCELERATION NSSUBPROP_DESCRIPTION, "Acceleration value used in a turn.");

	SetPropertyInt(LT_PROP_ACCELERATION_FAR_NEAR_DIFF, 40);
	SetPropertyBool(LT_PROP_ACCELERATION_FAR_NEAR_DIFF NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_ACCELERATION_FAR_NEAR_DIFF NSSUBPROP_DESCRIPTION, "The difference between far and near point in pixel.");

	SetPropertyFloat(LT_PROP_CAMERA_OFFSET, 15);
	SetPropertyBool(LT_PROP_CAMERA_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_CAMERA_OFFSET NSSUBPROP_DESCRIPTION, "The offset of the camera in relation to the center of the car.");

	SetPropertyFloat(LT_PROP_CONTR_PROPORTIONAL_GAIN, 0.08);
	SetPropertyBool(LT_PROP_CONTR_PROPORTIONAL_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_CONTR_PROPORTIONAL_GAIN NSSUBPROP_DESCRIPTION, "The proportional gain of the PID controller.");

	SetPropertyFloat(LT_PROP_CONTR_INTEGRAL_GAIN, 0.1445);
	SetPropertyBool(LT_PROP_CONTR_INTEGRAL_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_CONTR_INTEGRAL_GAIN NSSUBPROP_DESCRIPTION, "The integral gain of the PID controller.");

	SetPropertyFloat(LT_PROP_CONTR_DIFFERENTIAL_GAIN, 0.01238);
	SetPropertyBool(LT_PROP_CONTR_DIFFERENTIAL_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_CONTR_DIFFERENTIAL_GAIN NSSUBPROP_DESCRIPTION, "The differential gain of the PID controller.");

	SetPropertyInt(LT_PROP_TRESHOLD, 150);
	SetPropertyBool(LT_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");

	SetPropertyFloat(LT_PROP_PT1_TAU, 0.1);
	SetPropertyBool(LT_PROP_PT1_TAU NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_PT1_TAU NSSUBPROP_DESCRIPTION, "The tau value of the PT1 controller.");

	SetPropertyFloat(LT_PROP_PT1_SAMPLE_TIME, 0.9);
	SetPropertyBool(LT_PROP_PT1_SAMPLE_TIME NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_PT1_SAMPLE_TIME NSSUBPROP_DESCRIPTION, "The sample time of the PT1 controller.");

	SetPropertyFloat(LT_PROP_PT1_INPUT_FACTOR, 0.1);
	SetPropertyBool(LT_PROP_PT1_INPUT_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_PT1_INPUT_FACTOR NSSUBPROP_DESCRIPTION, "The factor to normalize the input value (difference of the lane center and the place to be) to the range of the output values.");

	SetPropertyInt(LT_PROP_LANE_WIDTH_MIN_NEAR, 320);
	SetPropertyBool(LT_PROP_LANE_WIDTH_MIN_NEAR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_LANE_WIDTH_MIN_NEAR NSSUBPROP_DESCRIPTION, "The Minimum value for the near lane width. If the calculated lane width is smaller than this value the blind count will be increased.");

	SetPropertyInt(LT_PROP_LANE_WIDTH_MAX_NEAR, 430);
	SetPropertyBool(LT_PROP_LANE_WIDTH_MAX_NEAR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_LANE_WIDTH_MAX_NEAR NSSUBPROP_DESCRIPTION, "The Maximum value for the near lane width. If the calculated lane width is greater than this value the blind count will be increased.");


	SetPropertyInt(LT_PROP_LANE_WIDTH_MIN_FAR, 120);
	SetPropertyBool(LT_PROP_LANE_WIDTH_MIN_FAR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_LANE_WIDTH_MIN_FAR NSSUBPROP_DESCRIPTION, "The Minimum value for the far lane width. If the calculated lane width is smaller than this value the blind count will be increased.");

	SetPropertyInt(LT_PROP_LANE_WIDTH_MAX_FAR, 290);
	SetPropertyBool(LT_PROP_LANE_WIDTH_MAX_FAR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(LT_PROP_LANE_WIDTH_MAX_FAR NSSUBPROP_DESCRIPTION, "The Maximum value for the far lane width. If the calculated lane width is greater than this value the blind count will be increased.");

	m_pISignalRegistry = NULL;
}

cLaneTracking::~cLaneTracking()
{
}

tResult cLaneTracking::GetInterface(const tChar* idInterface,
	tVoid** ppvObject)
{
	if (idmatch(idInterface, IID_ADTF_SIGNAL_PROVIDER))
	{
		*ppvObject = static_cast<ISignalProvider*> (this);
	}
	else
	{
		return cFilter::GetInterface(idInterface, ppvObject);
	}

	Ref();

	RETURN_NOERROR;
}

tUInt cLaneTracking::Ref()
{
	return cFilter::Ref();
}

tUInt cLaneTracking::Unref()
{
	return cFilter::Unref();
}

tVoid cLaneTracking::Destroy()
{
	delete this;
}

tResult cLaneTracking::Start(__exception)
{
	m_usDistanceVector.clear();
	m_bWasLightbeamTriggered = tFalse;
	return cFilter::Start(__exception_ptr);
}

tResult cLaneTracking::Stop(__exception)
{

#ifdef LT_ENABLE_CANNY_WINDOWS
	if(m_bShowDebug)
	{
		destroyWindow("Canny Near");
		destroyWindow("Canny Far");
		//destroyWindow("RGB Image");
		//destroyWindow("GaussianBlur");
		//destroyWindow("GreyScale Image");
		//destroyWindow("Binary Image");
		//destroyWindow("Canny Image");
		//destroyWindow("LaneTracking");  
	}  
#endif    

	return cFilter::Stop(__exception_ptr);
}

tResult cLaneTracking::Init(tInitStage eStage, __exception )
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		// Media Description Signal
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);        
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
		
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 

		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionInputOverallDistance));

		// Media Description Bool
		tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
		cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
		
		RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBool));

		RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEnablePin));

		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsData)); 


		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

		RETURN_IF_FAILED(m_oInputStartLightbeam.Create("LightbeamInput", new cMediaType(0, 0, 0, "tBoolSignalValue"), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputStartLightbeam));
		
		//ultrasonic input pin
		RETURN_IF_FAILED(m_oInputUsFrontCenter.Create("Ultrasonic_Front_Center", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oInputUsFrontCenter));

		//enable input pin
		RETURN_IF_FAILED(m_oInputEnablePin.Create("Enable", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oInputEnablePin));

		//create pin for overall distance
		RETURN_IF_FAILED(m_oInputDistanceOverall.Create("distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oInputDistanceOverall));

		//Acceleration Output
		RETURN_IF_FAILED(m_oAccelerateOutput.Create("Acceleration", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oAccelerateOutput));

		//Steering Angle Output
		RETURN_IF_FAILED(m_oSteeringAngleOutput.Create("Steering_Angle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngleOutput));

		//Steering Angle PT1 Output
		RETURN_IF_FAILED(m_oSteeringAnglePT1Output.Create("Steering_Angle_PT1", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSteeringAnglePT1Output));


        //m_oLaneCenterOffsetOutput Output
		RETURN_IF_FAILED(m_oLaneCenterOffsetOutput.Create("LaneCenterOffset", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oLaneCenterOffsetOutput));
        


		//HeadLights Output
		RETURN_IF_FAILED(m_oHeadLightsOutput.Create("HeadLights", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oHeadLightsOutput));

		//GLC Output
		cObjectPtr<IMediaType> pCmdType = NULL;
		RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
		RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));



	}
	else if (eStage == StageNormal)
	{
		m_bFirstFrame = true;
		m_ui8Imagecount = 0;
		m_bActive = tFalse;

		ReadProperties(NULL);

		if(m_bLightBeamTriggerEnabled == tFalse && GetPropertyBool(LT_PROP_STARTUP_STATE))
		{
			m_bActive = tTrue;
		}

		m_nCenterFromLeft = 0;
		m_nCenterFromRight = 0;
		m_i16ErrorSum = 0;
		m_i16ErrorOld = 0; 
		m_f32Ts = 0.033f;

		m_i16FarLaneCenter = 320;
		m_i16FarLaneWidth = 200;
		m_f32AccelerateOut = 0.0f;
		m_i16FarLaneWidthCalc = 0;

		m_ui8InitCtrl = 0;

		m_f32SteeringAngle = 0.0f;
		m_i16LaneWidth = 390;
		m_nBlindCounter = 0;
		m_nBlindCounterFar = 0;

		m_nCurrentNearLine = m_nNearLine;

		if (m_bShowDebug)
		{
			// create a kernel mutex 
			THROW_IF_FAILED(m_oLock.Create(adtf_util::cString(OIGetInstanceName()) + ".active_signals"));

			// get the signal registry object
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_SIGNAL_REGISTRY,
				IID_ADTF_SIGNAL_REGISTRY_EXTENDED,
				(tVoid**) &m_pISignalRegistry,
				__exception_ptr));

			// register the provider at the registry
			RETURN_IF_FAILED(m_pISignalRegistry->RegisterProvider(this, OIGetInstanceName(), __exception_ptr));

			RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(this,
				LT_SIGREG_ID_SCALED_PT1_INPUT,
				LT_SIGREG_NAME_SCALED_PT1_INPUT,
				LT_SIGREG_UNIT_SCALED_PT1_INPUT,
				"The variation of the 'lane center' and 'place to be' with the factor given by the PT1 input factor property.",
				-35,
				+35.0,
				__exception_ptr));

			RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(this,
				LT_SIGREG_ID_CONTROLLER_INPUT,
				LT_SIGREG_NAME_CONTROLLER_INPUT,
				LT_SIGREG_UNIT_CONTROLLER_INPUT,
				"The variation of the 'lane center' and 'place to be' (simple difference)",
				-50.0,
				+50.0,
				__exception_ptr));
		}

	}
	else if (eStage == StageGraphReady)
	{
		// the ids for the items in the structs of the mediasamples still have to be set

		m_bIDsUltrasonicSet = tFalse;
		m_bIDsEnablePinSet = tFalse;
		m_bIDsOverallDistanceSet = tFalse;
	}
	RETURN_NOERROR;
}

tResult cLaneTracking::PropertyChanged(const char* strProperty)
{
	ReadProperties(strProperty);

	RETURN_NOERROR;
}

tResult cLaneTracking::ReadProperties(const tChar* strPropertyName)
{
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_NEAR_LINE))
	{
		m_nNearLine = GetPropertyInt(LT_PROP_NEAR_LINE);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_NEAR_LINE_MAX_OFFSET))
	{
		m_nNearLineMaxOffset = GetPropertyInt(LT_PROP_NEAR_LINE_MAX_OFFSET);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_FAR_LINE))
	{
		m_nFarLine = GetPropertyInt(LT_PROP_FAR_LINE);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_MAX_ACCELERATION))
	{
		m_f32AccelerationMax = static_cast<tFloat32> (GetPropertyFloat(LT_PROP_MAX_ACCELERATION));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_MIN_ACCELERATION))
	{
		m_f32AccelerationMin = static_cast<tFloat32> (GetPropertyFloat(LT_PROP_MIN_ACCELERATION));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ACCELERATION_FAR_NEAR_DIFF))
	{
		m_nAccelerationFarNearDiff = GetPropertyInt(LT_PROP_ACCELERATION_FAR_NEAR_DIFF);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CAMERA_OFFSET))
	{
		m_f64CamOffset = GetPropertyFloat(LT_PROP_CAMERA_OFFSET);
		m_sLaneCenterNear.x = 320 + static_cast<tInt16> (m_f64CamOffset);
		m_sPlaceToBe.x = 320 + static_cast<tInt16> (m_f64CamOffset);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CONTR_PROPORTIONAL_GAIN))
	{
		m_f64Kp = GetPropertyFloat(LT_PROP_CONTR_PROPORTIONAL_GAIN);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CONTR_INTEGRAL_GAIN))
	{
		m_f64Ki = GetPropertyFloat(LT_PROP_CONTR_INTEGRAL_GAIN);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CONTR_DIFFERENTIAL_GAIN))
	{
		m_f64Kd = GetPropertyFloat(LT_PROP_CONTR_DIFFERENTIAL_GAIN);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_TRESHOLD))
	{
		m_nThresholdValue = GetPropertyInt(LT_PROP_TRESHOLD);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MIN_NEAR))
	{
		m_i16LaneWidthMinNear = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MIN_NEAR));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MAX_NEAR))
	{
		m_i16LaneWidthMaxNear = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MAX_NEAR));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MIN_FAR))
	{
		m_i16LaneWidthMinFar = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MIN_FAR));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MAX_FAR))
	{
		m_i16LaneWidthMaxFar = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MAX_FAR));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_TAU))
	{
		m_f64PT1Tau = GetPropertyFloat(LT_PROP_PT1_TAU);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_SAMPLE_TIME))
	{
		m_f64PT1Sample = GetPropertyFloat(LT_PROP_PT1_SAMPLE_TIME);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_INPUT_FACTOR))
	{
		m_f64PT1InputFactor = GetPropertyFloat(LT_PROP_PT1_INPUT_FACTOR);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_TAU) || cString::IsEqual(strPropertyName, LT_PROP_PT1_SAMPLE_TIME))
	{
		m_f64PT1Gain = m_f64PT1Tau / m_f64PT1Sample;
		m_f32PT1LastSteeringOut = 0.0f;
		m_f32PT1SteeringOut = 0.0f;
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ENABLE_LIGHTBEAM_TRIGGER))
	{
		m_bLightBeamTriggerEnabled = GetPropertyBool(LT_PROP_ENABLE_LIGHTBEAM_TRIGGER);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_SHOW_DEBUG))
	{
		m_bShowDebug = GetPropertyBool(LT_PROP_SHOW_DEBUG);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_DRIVE_TIME))
	{
		m_nDriveTime = GetPropertyInt(LT_PROP_DRIVE_TIME);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_DRIVE_DISTANCE))
	{
		m_f32OverallDistanceThreshold = GetPropertyFloat(LT_PROP_DRIVE_DISTANCE);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_EMERGENCY_STOP_TIME))
	{
		m_nEmergencyStopTime = GetPropertyInt(LT_PROP_EMERGENCY_STOP_TIME);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_US_MINIMUM_FREE_DISTANCE))
	{
		m_usMinFreeThreshold = GetPropertyFloat(LT_PROP_US_MINIMUM_FREE_DISTANCE);
	}
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_US_ENABLE))
	{
		m_usDetectionEnabled = GetPropertyBool(LT_PROP_US_ENABLE);
	}
	
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_STARTUP_STATE))
	{
		m_bActive = GetPropertyBool(LT_PROP_STARTUP_STATE);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LIGHTBEAM_ONLY_ONCE))
	{
		m_bLightBeamOnlyOnce = GetPropertyInt(LT_PROP_LIGHTBEAM_ONLY_ONCE);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_STOP_MODE))
	{
		m_iStopMode = GetPropertyInt(LT_PROP_STOP_MODE);
	}
	RETURN_NOERROR;
}

tResult cLaneTracking::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{
	__synchronized_obj(m_oRunCritSection);
	if(IRunnable::RUN_TIMER == nActivationCode)
	{
		LOG_INFO(cString::Format("Run timer called with user data size: %d", szUserDataSize).GetPtr());
		if(szUserDataSize == sizeof(tUInt8))
		{
			tUInt8* nUserData = (tUInt8*) pvUserData;
			LOG_INFO(cString::Format("Run timer called with user data value: %u", *nUserData));
			if(*nUserData == g_ui8StopTimerIdNegative)
			{
				LOG_INFO("Set active to false and break (negative acceleration).");
				m_bActive = tFalse;

				// transmit negative acceleration for breaking
				TransmitAcceleration(0.0f, _clock->GetStreamTime());

				// destroy this timer
				_kernel->TimerDestroy(m_hStopTimerNegative);
				m_hStopTimerNegative = NULL;

				m_hStopTimerZero = _kernel->TimerCreate(-1, 500000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8StopTimerIdZero, sizeof(tUInt8), 0, cString::Format("%s.stoptimerZero", OIGetInstanceName()));
			}
			else if(*nUserData == g_ui8StopTimerIdZero)
			{
				LOG_INFO("Set active to false, set acceleration to zero and destroy timer.");
				m_bActive = tFalse;

				// transmit negative acceleration for breaking
				TransmitAcceleration(0.0f, _clock->GetStreamTime());

				// destroy this timer
				_kernel->TimerDestroy(m_hStopTimerZero);
				m_hStopTimerZero = NULL;
			}
			else if(*nUserData == g_ui8EmergencyStopTimerIdNegative)
			{
				LOG_INFO("Set active to false, performing an emergency break (acceleration negative) and destroy timer.");
				m_bActive = tFalse;

				// transmit negative acceleration for breaking
				// break hard for 1 second
				TransmitAcceleration(0.0f, _clock->GetStreamTime());

				// cleanup this timer
				_kernel->TimerDestroy(m_hEmergencyStopTimerNegative);
				m_hEmergencyStopTimerNegative = NULL;

				m_hEmergencyStopTimerZero = _kernel->TimerCreate(-1, 1000000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8EmergencyStopTimerIdZero, sizeof(tUInt8), 0, cString::Format("%s.emergencystoptimerZero", OIGetInstanceName()));
			}
			else if(*nUserData == g_ui8EmergencyStopTimerIdZero)
			{
				LOG_INFO("Set active to false, performing an emergency break, set active to true and destroy timer.");
				m_bActive = tFalse;

				// stay on that position for 3 seconds
				TransmitAcceleration(0.0f, _clock->GetStreamTime());

				// cleanup this timer
				_kernel->TimerDestroy(m_hEmergencyStopTimerZero);
				m_hEmergencyStopTimerZero = NULL;

				m_hEmergencyStopTimerResume = _kernel->TimerCreate(-1, 3000000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8EmergencyStopTimerIdResume, sizeof(tUInt8), 0, cString::Format("%s.emergencystoptimerResume", OIGetInstanceName()));
			}

			else if(*nUserData == g_ui8EmergencyStopTimerIdResume)
			{
				LOG_INFO("Set active to true after performing an emergency break and destroy timer.");

				// reenable the longitudinal controller
				m_bActive = tTrue;

				// cleanup this timer
				_kernel->TimerDestroy(m_hEmergencyStopTimerResume);
				m_hEmergencyStopTimerResume = NULL;

			}
		}
	}

	RETURN_NOERROR;
}

tResult cLaneTracking::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if(m_bShowDebug && eStage == cFilter::StageNormal)
	{        
		ucom::cObjectPtr<ISignalRegistry> pSignalRegistry;
		if (IS_OK(_runtime->GetObject(OID_ADTF_SIGNAL_REGISTRY,
			IID_ADTF_SIGNAL_REGISTRY,
			(tVoid**)&pSignalRegistry)))
		{
			// Unregister the provider
			pSignalRegistry->UnregisterSignalProvider(this);
		}
		m_oActive.clear();

		m_oLock.Release();
	}

	if(m_hStopTimerZero != NULL)
	{
		_kernel->TimerDestroy(m_hStopTimerZero);
		m_hStopTimerZero = NULL;
	}
	if(m_hStopTimerNegative != NULL)
	{
		_kernel->TimerDestroy(m_hStopTimerNegative);
		m_hStopTimerNegative = NULL;
	}

	if(m_hEmergencyStopTimerNegative != NULL)
	{
		_kernel->TimerDestroy(m_hEmergencyStopTimerNegative);
		m_hEmergencyStopTimerNegative = NULL;
	}

	if(m_hEmergencyStopTimerZero != NULL)
	{
		_kernel->TimerDestroy(m_hEmergencyStopTimerZero);
		m_hEmergencyStopTimerZero = NULL;
	}

	if(m_hEmergencyStopTimerResume != NULL)
	{
		_kernel->TimerDestroy(m_hEmergencyStopTimerResume);
		m_hEmergencyStopTimerResume = NULL;
	}

	return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cLaneTracking::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tTimeStamp InputTimeStamp;
		InputTimeStamp = pMediaSample->GetTime();


		if(pSource == &m_oVideoInputPin)
		{
			//Videoformat
			if (m_bFirstFrame)
			{        
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
				const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
				if (pFormat == NULL)
				{
					LOG_ERROR("No Bitmap information found on pin \"input\"");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
				m_sInputFormat.nWidth = pFormat->nWidth;
				m_sInputFormat.nHeight =  pFormat->nHeight;
				m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
				m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
				m_sInputFormat.nSize = pFormat->nSize;
				m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
				m_bFirstFrame = false;
			}

			ProcessInput(pMediaSample, InputTimeStamp);
		}
		else if(pSource == &m_oInputStartLightbeam)
		{
			tBool bValue = tFalse;
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pCoderDescBool,pMediaSample,pCoder);

                pCoder->Get("bValue", (tVoid*)&bValue);            
            }
		
			if ((bValue && (m_bLightBeamOnlyOnce && !(m_bWasLightbeamTriggered)))
				|| (bValue && !m_bLightBeamOnlyOnce))
            {
                m_bWasLightbeamTriggered = tTrue;
				LOG_INFO("Receiving positive trigger.");
                if (m_iStopMode == 1)
				{
					m_bActive = tTrue;
					LOG_INFO("Starting Lanetracking... waiting for overall distance.");
					m_f32DistanceStartValue = m_f32CurrentDistance;

				}
				else if(m_iStopMode == 2)
				{
					if(m_hStopTimerNegative == NULL && m_hStopTimerZero == NULL && m_nDriveTime > 0)
					{
						m_bActive = tTrue;
						m_hStopTimerNegative = _kernel->TimerCreate(-1, m_nDriveTime * 1000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8StopTimerIdNegative, sizeof(tUInt8), 0, cString::Format("%s.stoptimerNegative", OIGetInstanceName()));
						LOG_INFO(cString::Format("Starting stopping timer with: %d us and %u", m_nDriveTime * 1000, g_ui8StopTimerIdNegative).GetPtr());
					}
					else if(m_hStopTimerNegative != NULL || m_hStopTimerZero != NULL)
					{
						LOG_WARNING("There is already a stopping timer running. Will ignore this trigger.");
					}
					else if (m_nDriveTime <= 0)
					{
						m_bActive = tTrue;
						LOG_INFO("There is no Drive time set. The car will not stop unless stopping ADTF.");
					}

					if(m_hEmergencyStopTimerNegative == NULL && m_hEmergencyStopTimerZero == NULL && m_hEmergencyStopTimerResume == NULL && m_nEmergencyStopTime > 0)
					{
						m_hEmergencyStopTimerNegative = _kernel->TimerCreate(-1, m_nEmergencyStopTime * 1000, static_cast<IRunnable*>(this), NULL,(tVoid*) &g_ui8EmergencyStopTimerIdNegative, sizeof(tUInt8), 0, cString::Format("%s.emergencytimerNegative", OIGetInstanceName()));
						LOG_INFO(cString::Format("Starting emergency timer with: %d us and %u", m_nEmergencyStopTime * 1000, g_ui8EmergencyStopTimerIdNegative).GetPtr());
					}
					else if(m_hEmergencyStopTimerNegative != NULL || m_hEmergencyStopTimerZero != NULL || m_hEmergencyStopTimerResume != NULL)
					{
						LOG_WARNING("There is already a braking timer running. Will ignore this trigger.");
					}
					else if (m_nEmergencyStopTime <= 0)
					{
						LOG_INFO("There is no emergency stop time set. This action will not be performed.");
					}
				}
            }
			else if(bValue && m_bLightBeamOnlyOnce && m_bWasLightbeamTriggered)
			{
				LOG_INFO("Received positive Trigger, but was already triggered...doing nothing");
			}
            else if(bValue == tFalse)
            {
                LOG_INFO("Receiving negative trigger. Will stop the car.");
                m_bActive = tFalse;
            }
		}
		else if (pSource == &m_oInputUsFrontCenter)
		{
			RETURN_IF_FAILED(ProcessUltrasonicSample(pMediaSample, SensorDefinition::usFrontCenter));
		}
		else if (pSource == &m_oInputEnablePin)
		{
			RETURN_IF_FAILED(ProcessEnableSample(pMediaSample));
		}
		else if (pSource == &m_oInputDistanceOverall)
		{
			RETURN_IF_FAILED(ProcessOverallDistanceSample(pMediaSample));
		}


		RETURN_NOERROR;
	}

	RETURN_NOERROR;
}

tResult cLaneTracking::ProcessUltrasonicSample(IMediaSample* pMediaSample, SensorDefinition::ultrasonicSensor usSensor)
{
	//write values with zero
	tFloat32 f32value = 0;
	{   // focus for sample read lock
		// read-out the incoming Media Sample
		__adtf_sample_read_lock_mediadescription(m_pDescriptionUsData,pMediaSample,pCoderInput);

		// get the IDs for the items in the media sample 
		if(!m_bIDsUltrasonicSet)
		{
			pCoderInput->GetID("f32Value", m_szIDUltrasonicF32Value);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDUltrasonicArduinoTimestamp);
			m_bIDsUltrasonicSet = tTrue;
		}     

		//get values from media sample        
		pCoderInput->Get(m_szIDUltrasonicF32Value, (tVoid*)&f32value);

		m_usDistanceVector.push_back(f32value);

		if (m_usDistanceVector.size()>DISTANCE_LENGTH)
		{
			m_usDistanceVector.pop_front();
		}

		tFloat32 tmpDistance = 0;
		for (std::list<tFloat32>::iterator it= m_usDistanceVector.begin(); it != m_usDistanceVector.end(); it++)
		{
			tmpDistance = tmpDistance + *it;
		}
		m_usCurrentDistance = tmpDistance/DISTANCE_LENGTH;

	}
	RETURN_NOERROR;
}

tResult cLaneTracking::ProcessEnableSample(IMediaSample* pMediaSample)
{
	//write values with zero
	tBool bValue = 0;
	{   // focus for sample read lock
		// read-out the incoming Media Sample
		__adtf_sample_read_lock_mediadescription(m_pDescriptionEnablePin,pMediaSample,pCoderInput);

		// get the IDs for the items in the media sample 
		if(!m_bIDsEnablePinSet)
		{
			pCoderInput->GetID("bValue", m_szIDEnablePinbValue);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDEnablePinArduinoTimestamp);
			m_bIDsEnablePinSet = tTrue;
		}     

		//get values from media sample        
		pCoderInput->Get(m_szIDEnablePinbValue, (tVoid*)&bValue);
	}

	if (bValue)
	{
		LOG_INFO("Lane Tracking was enabled by user");
		m_bActive = tTrue;
		m_bWasLightbeamTriggered = tFalse;
	}
	else
	{
		LOG_INFO("Lane Tracking was disabled by user");
		m_bActive = tFalse;
	}

	RETURN_NOERROR;
}

tResult cLaneTracking::ProcessOverallDistanceSample(IMediaSample* pMediaSample)
{
	//write values with zero
	tFloat32 f32Value = 0;
	{   // focus for sample read lock
		// read-out the incoming Media Sample
		__adtf_sample_read_lock_mediadescription(m_pDescriptionInputOverallDistance,pMediaSample,pCoderInput);

		// get the IDs for the items in the media sample 
		if(!m_bIDsOverallDistanceSet)
		{
			pCoderInput->GetID("f32Value", m_szIDOverallDistanceF32Value);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDOverallDistanceArduinoTimestamp);
			m_bIDsOverallDistanceSet = tTrue;
		}     

		//get values from media sample        
		pCoderInput->Get(m_szIDOverallDistanceF32Value, (tVoid*)&f32Value);
		
		m_f32CurrentDistance = f32Value;

		if (((f32Value - m_f32DistanceStartValue) > m_f32OverallDistanceThreshold) && m_bWasLightbeamTriggered)
		{
			m_bActive = tFalse;
			//LOG_INFO("Lane tracking was stopped because distance threshold was reached");
		};
	}

	RETURN_NOERROR;
}

tResult cLaneTracking::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	if(m_ui8InitCtrl < 150)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
	{
		TransmitAcceleration(0.0f, tsInputTime);
		TransmitSteeringAngle(0.0f, tsInputTime);
		TransmitSteeringAnglePT1(0.0f, tsInputTime);
		TransmitHeadLights(tTrue, tsInputTime);    // Switch on Headlights
		m_ui8InitCtrl++;
	}
	else
	{
		// VideoInput
		RETURN_IF_POINTER_NULL(pSample);

		const tVoid* l_pSrcBuffer;

		/*IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
		RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));

		cvSetData(img, (tVoid*)l_pSrcBuffer, img->widthStep);
		Mat image(cvarrToMat(img));
		cvReleaseImage(&img);
		pSample->Unlock(l_pSrcBuffer);*/

		IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
		RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
		oImg->imageData = (char*)l_pSrcBuffer;
		Mat image(cvarrToMat(oImg));
		cvReleaseImage(&oImg);
		pSample->Unlock(l_pSrcBuffer);

     


		// Transform nearfield (Nearfield is used for lateral control)
		Mat matImageCutNear= image(cv::Range(m_nCurrentNearLine - 20, m_nCurrentNearLine + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image    
		GaussianBlur(matImageCutNear, matImageCutNear, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
		cvtColor(matImageCutNear, m_matGreyNear ,CV_RGB2GRAY);// Grey Image

             
        tInt blocksize = 5;
        tInt param1=2;
        Mat adaptTresh, cannyAdapt;
		
     

          threshold(m_matGreyNear, adaptTresh, m_nThresholdValue, 255,THRESH_BINARY);// Generate Binary Image
            Canny(adaptTresh, cannyAdapt, 0, 2, 3, false);// Detect Edges
           
            adaptiveThreshold(m_matGreyNear, m_matGreyThreshNear, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, blocksize, param1);
           
            Canny(m_matGreyThreshNear, m_matLineCannyNear, 0, 2, 3, false);
            dilate(m_matGreyThreshNear, m_matLineCannyNear, Mat());
           
        
        
        

        /**
        threshold(m_matGreyNear, m_matGreyThreshNear, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
		Canny(m_matGreyThreshNear, m_matLineCannyNear, 0, 2, 3, false);// Detect Edges
        **/
		// search points within the canny near
		Search(m_asAllpointsNear, &m_ui8NearPointsCount, 40, m_matLineCannyNear);



		// Transform farfield (farfield is used for longitudinal control)
		Mat matImageCutFar= image(cv::Range(m_nFarLine - 20, m_nFarLine + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image        
		GaussianBlur(matImageCutFar, matImageCutFar, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
		cvtColor(matImageCutFar, m_matGreyFar ,CV_RGB2GRAY);// Grey Image
		threshold(m_matGreyFar, m_matGreyThreshFar, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
		Canny(m_matGreyThreshFar, m_matLineCannyFar, 0, 2, 3, false);// Detect Edges

		// search points within the canny far
		Search(m_asAllpointsFar, &m_ui8FarPointsCount, 40, m_matLineCannyFar);    

//#define LT_ENABLE_CANNY_WINDOWS
#ifdef LT_ENABLE_CANNY_WINDOWS
		//**************** Show Image *************************
		if(m_bShowDebug)
		{
			if(m_ui8Imagecount > 2)
			{
				m_ui8Imagecount=0;
				//imshow("RGB Image", image);
				//imshow("GaussianBlur", imagegauss);
				//imshow("GreyScale Image", greyNear);
				//imshow("Binary Image", greythreshNear);
				//imshow("Canny Image", linecanny);*/
				imshow("Canny Near", m_matLineCannyNear);
				//imshow("Canny Far", m_matLineCannyFar);
				waitKey(1);
			}
			m_ui8Imagecount++;
		}
		//********************************************************
#endif

		//********** Find horizontal line (for calibration only)**************
		//tInt edgerow = 0;
		//for(tInt row=cannysize.height;row>=1;row--)                                                                                                                                                                                  
		//{            
		//    if(linecanny.at<uchar>(row,cannysize.width/2)!=0) 
		//    {    
		//        edgerow = row;
		//        row = 0;
		//    }
		//}
		//LOG_INFO(cString::Format("Found edge at row: %i", edgerow));
		//********************************************************************    

		//LOG_INFO(cString::Format("lfdnr1: %i",lfdnr1));

		LateralControl(m_asAllpointsNear, &m_ui8NearPointsCount, tsInputTime);
		LongitudinalControl(m_asAllpointsFar, &m_ui8FarPointsCount, tsInputTime);
	}

	RETURN_NOERROR;            
}

tResult cLaneTracking::ProcessFound()
{        

	RETURN_NOERROR;
}

tResult cLaneTracking::ProcessOutput()
{

	RETURN_NOERROR;
}

tResult cLaneTracking::Search(sPoint *psPoints, tUInt8 *pui8PointsCount, tUInt8 ui8Limit, cv::Mat matCannyImg)
{
	A_UTIL_ASSERT(NULL != psPoints);
	A_UTIL_ASSERT(NULL != pui8PointsCount);

	*pui8PointsCount    = 0;
	tInt nColumnLast    = -1;
	Size szCannySize      = matCannyImg.size();

	for(tInt nColumn=0; nColumn < szCannySize.width; nColumn++)
	{                    
		if(matCannyImg.at<uchar>(szCannySize.height/2, nColumn)!=0) 
		{    
			if(abs(nColumnLast-nColumn)>=5 && abs(nColumnLast-nColumn) < ui8Limit && nColumnLast!=-1)
			{
				psPoints[*pui8PointsCount].x = static_cast<tInt>(nColumn-(abs(nColumnLast-nColumn)/2));
				(*pui8PointsCount)++;            
				nColumnLast=-1;
			}
			else
			{
				nColumnLast=nColumn;
			}
		}
	}

	RETURN_NOERROR;
}

tResult cLaneTracking::LateralControl(sPoint *psPoints, tUInt8 *pui8PointsCount, tTimeStamp tsInputTime)
{

	if(*pui8PointsCount == 2)
	{
		tInt16 i16LaneWidthCalc = psPoints[1].x - psPoints[0].x;
		//LOG_INFO(cString::Format("Point0: %i, Point1: %i, LaneWidth calc: %i", points[0].x, points[1].x, LaneWidth_calc));
		tInt nLaneCenterCalc = psPoints[0].x + (static_cast<tInt>(i16LaneWidthCalc/2));

		if ((i16LaneWidthCalc > m_i16LaneWidthMinNear) && (i16LaneWidthCalc < m_i16LaneWidthMaxNear) && abs(nLaneCenterCalc - m_sLaneCenterNear.x) < MAX_DEVIATION)
		{
			m_i16LaneWidth = i16LaneWidthCalc;
			m_sLaneCenterNear.x = nLaneCenterCalc;
			m_nBlindCounter = 0;
		}
		else
		{
			m_nBlindCounter++;
		}        
	}
	else if(*pui8PointsCount == 3)
	{
		// If three points are found the correct lane is chosen according to the calculated lanewidth
		tInt16 i16Width1 = psPoints[2].x - psPoints[1].x;
		tInt16 i16Width2 = psPoints[1].x - psPoints[0].x;
		tInt nLaneCenterCalc1 = psPoints[1].x + (static_cast<tInt>(i16Width1/2));
		tInt nLaneCenterCalc2 = psPoints[0].x + (static_cast<tInt>(i16Width2/2));

		if((i16Width1 > m_i16LaneWidthMinNear) && (i16Width1 < m_i16LaneWidthMaxNear) && abs(nLaneCenterCalc1 - m_sLaneCenterNear.x) < MAX_DEVIATION)
		{
			m_i16LaneWidth = i16Width1;
			m_sLaneCenterNear.x = nLaneCenterCalc1;
			m_nBlindCounter = 0;
		}
		else if((i16Width2 > m_i16LaneWidthMinNear) && (i16Width2 < m_i16LaneWidthMaxNear) && abs(nLaneCenterCalc2 - m_sLaneCenterNear.x) < MAX_DEVIATION)
		{
			m_i16LaneWidth = i16Width2;
			m_sLaneCenterNear.x = nLaneCenterCalc2;
			m_nBlindCounter = 0;
		}
		else
		{
			m_nBlindCounter++;
		}        
	}
	else if(*pui8PointsCount == 1)
	{
		// If just one point is found the lane center shall be calculated based on this single point and the half of the previously calculated lane width
		// If the right point of the lane was found LaneWidth/2 must be subtracted from the position of the point, if the left point was found LaneWidth/2
		// must be added. Whether to add or to subtract LaneWidth/2 is determined by comparing the calculated new LaneCenter with the old LaneCenter.

		m_nCenterFromLeft = psPoints[0].x + (static_cast<tInt>(m_i16LaneWidth/2));

		m_nCenterFromRight = psPoints[0].x - (static_cast<tInt>(m_i16LaneWidth/2));

		if(abs(m_nCenterFromLeft - m_sLaneCenterNear.x) < MAX_DEVIATION)
		{
			m_sLaneCenterNear.x = m_nCenterFromLeft;
			m_nBlindCounter = 0;
		}
		else if(abs(m_nCenterFromRight - m_sLaneCenterNear.x) < MAX_DEVIATION)
		{
			m_sLaneCenterNear.x = m_nCenterFromRight;
			m_nBlindCounter = 0;
		}
		else
		{
			m_nBlindCounter++;
		}                
	}
	else
	{    
		// If more than three or no point (=lane) is detected, continue with the previously calculated values
		// Due to safety reasons a counter is implemented, that will cause the car to stop, if the car did not find any lane for more than X times consecutively (see parameter in logitudinal control)

		m_nBlindCounter++;
	}

   

    if(m_sLaneCenterNear.x > 630 ||  m_sLaneCenterNear.x < 10)
    {
        	m_sLaneCenterNear.x = 320 + static_cast<tInt16> (m_f64CamOffset);
		m_sPlaceToBe.x = 320 + static_cast<tInt16> (m_f64CamOffset);
      // m_sLaneCenterNear.x = 320; 

    }

    tFloat32 scaledValue = m_sLaneCenterNear.x - (320 + static_cast<tInt16> (m_f64CamOffset)) ;
    scaledValue /=   static_cast<tFloat32>((m_i16LaneWidthMinNear + m_i16LaneWidthMaxNear)/2);
    scaledValue *= 0.2;
    
     if(m_nBlindCounter == 0)    LaneOffsetAcceleration(scaledValue,tsInputTime);

   // if(m_nBlindCounter >)

	//if (m_sLaneCenterNear.x < 0 || m_sLaneCenterNear.x > m_sInputFormat.nWidth)
	//{
	//    m_sLaneCenterNear.x = static_cast<tInt16> (m_sInputFormat.nWidth / 2);
	//}


	//PID - Controller for Steering Angle *
	m_i16Error = m_sLaneCenterNear.x - m_sPlaceToBe.x;
	m_i16ErrorSum = m_i16ErrorSum + m_i16Error;    
	m_f32SteeringAngle = static_cast<tFloat32>(m_f64Kp*m_i16Error + m_f64Ki*m_f32Ts*m_i16ErrorSum + (m_f64Kd*(m_i16Error-m_i16ErrorOld))/m_f32Ts);
	m_i16ErrorOld = m_i16Error;

	//  LOG_INFO(cString::Format("Kp: %f, Ki: %f, Kd: %f", m_Kp, m_Ki, m_Kd).GetPtr());

	// Limitation of Steering Angle
	if (m_f32SteeringAngle > 30)
	{
		//      LOG_INFO(cString::Format("Error greater: LaneCenter %d Place2Be %d Steering %f error_val: %d error_sum: %d", m_LaneCenter.x, m_PlaceToBe.x, m_SteeringAngle, error, e_sum).GetPtr());
		m_f32SteeringAngle = 30.0;
	}
	else if (m_f32SteeringAngle < -30)
	{
		//      LOG_INFO(cString::Format("Error smaller: LaneCenter %d Place2Be %d Steering %f error_val: %d error_sum: %d", m_LaneCenter.x, m_PlaceToBe.x, m_SteeringAngle, error, e_sum).GetPtr());
		m_f32SteeringAngle = -30.0;
	}


	/*********************************
	*
	* PT 1 discrete algorithm
	*
	*               Tau
	*       In +    ---  * LastOut
	*             Tsample
	* Out = ---------------------
	*               Tau
	*       1 +     ---
	*             Tsample
	*
	*                           Tau
	* here with     Factor =    ---
	*                         Tsample
	*
	********************************************/
	m_f64PT1ScaledError = m_i16Error * m_f64PT1InputFactor/*/ 5*/;
	m_f32PT1SteeringOut = static_cast<tFloat32> ((m_f64PT1ScaledError + m_f64PT1Gain * m_f32PT1LastSteeringOut) / (1 + m_f64PT1Gain));
	m_f32PT1LastSteeringOut = m_f32PT1SteeringOut;

	if (m_f32PT1SteeringOut > 30.0f)
	{
		m_f32PT1SteeringOut = 30.0f;
	}
	else if (m_f32PT1SteeringOut < -30.0f)
	{
		m_f32PT1SteeringOut = -30.0f;
	}
	//forchher: start immediately

	if(m_nBlindCounter > 30){
		/**
		m_i16Error = 0;
		m_i16ErrorSum = 0;
		m_f32SteeringAngle = 0;
		m_f32PT1SteeringOut = 0;
		m_i16ErrorOld = 0;
		m_f64PT1Gain = m_f64PT1Tau / m_f64PT1Sample;

		m_f32PT1LastSteeringOut = 0.0f;
		**/
		m_f32PT1SteeringOut = 0.0f;


	}    

	TransmitSteeringAngle(m_f32SteeringAngle, tsInputTime);        // Send data to output pin
	TransmitSteeringAnglePT1(m_f32PT1SteeringOut, tsInputTime);

	CreateAndTransmitGCL();

	if (m_bShowDebug)
	{
		__synchronized_kernel(m_oLock);
		tTimeStamp tsStreamTime = _clock->GetStreamTime();

		for (tActiveSignals::iterator oSignal = m_oActive.begin();
			oSignal != m_oActive.end();
			++oSignal)
		{
			if (*oSignal == LT_SIGREG_ID_CONTROLLER_INPUT)
			{
				tSignalValue sValue;
				sValue.nRawValue = 0;
				sValue.nTimeStamp = tsStreamTime;
				sValue.strTextValue = 0;
				sValue.f64Value = m_i16Error;

				m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
			}
			else if (*oSignal == LT_SIGREG_ID_SCALED_PT1_INPUT)
			{
				tSignalValue sValue;
				sValue.nRawValue = 0;
				sValue.nTimeStamp = tsStreamTime;
				sValue.strTextValue = 0;
				sValue.f64Value = m_f64PT1ScaledError;

				m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
			}
		}
	}

	RETURN_NOERROR;
}

/**
*   Returns the current value of a Signal.
*/
tResult cLaneTracking::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
	if (nSignalID == LT_SIGREG_ID_CONTROLLER_INPUT)
	{
		pValue->f64Value = m_i16Error;
		tTimeStamp nStreamTime = _clock->GetStreamTime();
		pValue->nRawValue = 0;
		pValue-> nTimeStamp = nStreamTime;
		pValue->strTextValue = 0;
	}
	else if (nSignalID == LT_SIGREG_ID_SCALED_PT1_INPUT)
	{
		pValue->f64Value = m_f64PT1ScaledError;
		tTimeStamp nStreamTime = _clock->GetStreamTime();
		pValue->nRawValue = 0;
		pValue-> nTimeStamp = nStreamTime;
		pValue->strTextValue = 0;
	}
	else
	{
		RETURN_ERROR(ERR_NOT_FOUND);
	}


	RETURN_NOERROR;
}

/**
*   Activates a signal.
*   Activated signals send their values to the Signal Registry Service.
*/
tResult cLaneTracking::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{
	if (nSignalID == LT_SIGREG_ID_CONTROLLER_INPUT || nSignalID == LT_SIGREG_ID_SCALED_PT1_INPUT)
	{
		__synchronized_kernel(m_oLock);
		m_oActive.insert(nSignalID);
	}
	else
	{
		RETURN_ERROR(ERR_NOT_FOUND);
	}

	RETURN_NOERROR;
}

/**
*   Deactivates a signal.
*/
tResult cLaneTracking::DeactivateSignalEvents(tSignalID nSignalID)
{
	__synchronized_kernel(m_oLock);
	m_oActive.erase(nSignalID);
	RETURN_NOERROR;
}

tResult cLaneTracking::LongitudinalControl(sPoint *psPointsFar, tUInt8 *pui8PointsCountFar, tTimeStamp tsInputTime)
{

	if(*pui8PointsCountFar == 2)
	{
		m_i16FarLaneWidthCalc = psPointsFar[1].x - psPointsFar[0].x;

		//LOG_INFO(cString::Format("FarLaneWidth calc: %i", FarLaneWidth_calc));

		if ((m_i16FarLaneWidthCalc > m_i16LaneWidthMinFar) && (m_i16FarLaneWidthCalc < m_i16LaneWidthMaxFar))
		{
			m_i16FarLaneWidth = m_i16FarLaneWidthCalc;
			m_i16FarLaneCenter = psPointsFar[0].x + (static_cast<tInt>(m_i16FarLaneWidth/2));
			m_nBlindCounterFar = 0;
		} 

		//LOG_INFO(cString::Format("FarLaneCenter: %i", FarLaneCenter));
		//LOG_INFO(cString::Format("FarLaneWidth: %i", FarLaneWidth));
	}
	else if(*pui8PointsCountFar == 1)
	{
		tInt nFarCenterLeft = psPointsFar[0].x + (static_cast<tInt>(m_i16FarLaneWidth/2));

		tInt nFarCenterRight = psPointsFar[0].x - (static_cast<tInt>(m_i16FarLaneWidth/2));

		if(abs(nFarCenterRight - m_i16FarLaneCenter) < MAX_DEVIATION)
		{
			m_i16FarLaneCenter = nFarCenterRight;
			m_nBlindCounterFar = 0;
		} 
		else if(abs(nFarCenterLeft - m_i16FarLaneCenter) < MAX_DEVIATION)
		{
			m_i16FarLaneCenter = nFarCenterLeft;
			m_nBlindCounterFar = 0;
		}

	} 
	else if(*pui8PointsCountFar == 3)
	{
		tInt16 i16FarWidth1 = psPointsFar[1].x - psPointsFar[0].x;
		tInt16 i16FarWidth2 = psPointsFar[2].x - psPointsFar[1].x;

		if((i16FarWidth2 > m_i16LaneWidthMinFar) && (i16FarWidth2 < m_i16LaneWidthMaxFar))
		{
			m_i16FarLaneCenter = psPointsFar[1].x + (static_cast<tInt>(i16FarWidth2/2)); 
			m_nBlindCounterFar = 0;
		}
		else if((i16FarWidth1 > m_i16LaneWidthMinFar) && (i16FarWidth1 < m_i16LaneWidthMaxFar))
		{
			m_i16FarLaneCenter = psPointsFar[0].x + (static_cast<tInt>(i16FarWidth1/2));
			m_nBlindCounterFar = 0;
		}
	}
	else
	{    // If no or more than 3 points are found, it is supposed that the car is located in a curve, looking anywhere. 
		// increase the blind counter
		m_nBlindCounterFar++; 
	}

	if (m_nBlindCounterFar > 5)
	{
		// For this reason the variable Lanecenter is set to 0, since this will 
		// cause the car to continue driving at m_AccelerationMin.
		m_i16FarLaneCenter = 0;
	}


	if (m_nBlindCounter > 30 )
	{
		//forchher: start immediately
		m_f32AccelerateOut = m_f32AccelerationMin;
		//m_f32AccelerateOut = 0.0;        // If the car can't find lanes anymore --> stop
	}
	else
	{
		//        tInt16 i16Difference = abs(m_i16FarLaneCenter - m_sPlaceToBe.x);
		tInt16 i16Difference = abs(m_i16FarLaneCenter - m_sLaneCenterNear.x);
		tFloat32 f32AccelerationDifference = m_f32AccelerationMax - m_f32AccelerationMin;
		tFloat32 f32Factor = (m_nAccelerationFarNearDiff - i16Difference) > 0 ? (m_nAccelerationFarNearDiff - i16Difference) / (tFloat32) m_nAccelerationFarNearDiff : 0.0f;
		m_f32AccelerateOut = m_f32AccelerationMin + (f32Factor * f32AccelerationDifference);
		// Adjust current near line according to absolute output acceleration (speed) 
		// higher speed results in farther distance => look ahead!
		f32Factor = m_f32AccelerateOut / 100;
		m_nCurrentNearLine = (tInt) (f32Factor * m_nNearLineMaxOffset) + m_nNearLine;
	}
	/*    else if(abs(m_i16FarLaneCenter - m_sLaneCenterNear.x) <= m_nAccelerationFarNearDiff)    // This parameter determines how early the car brakes before and accelerates after a curve
	{
	m_f32AccelerateOut = m_f32AccelerationMax;
	}
	else if(abs(m_i16FarLaneCenter - m_sLaneCenterNear.x) > m_nAccelerationFarNearDiff)
	{
	m_f32AccelerateOut = m_f32AccelerationMin;
	}
	*/
	//LOG_INFO(cString::Format("Acceleration: %f", AccelerateOut));
	if (m_bActive)
	{
		// transmit acceleration out only if the "global" active flag is set to true
		TransmitAcceleration(m_f32AccelerateOut, tsInputTime);        // Send data to output pin
	}
	else if(m_bActive==tFalse)
	{
		TransmitAcceleration(0.0, tsInputTime);
	}
	else if (m_hEmergencyStopTimerNegative == NULL && 
		m_hEmergencyStopTimerZero == NULL && 
		//m_hEmergencyStopTimerResume == NULL && 
		m_hStopTimerNegative == NULL &&
		m_hStopTimerZero == NULL)
	{
		TransmitAcceleration(0.0f, tsInputTime);
	}

	RETURN_NOERROR;
}

tResult cLaneTracking::TransmitAcceleration(tFloat32 f32Acceleration, tTimeStamp tsInputTime)
{
	__synchronized_obj(m_oTransmitAccelCritSection);



	/*static tFloat32 AccelLastTransmitted = 1000.0;

	if (fabs(Acceleration - AccelLastTransmitted)< 0.001f)
	{
	RETURN_NOERROR;
	} 

	AccelLastTransmitted = Acceleration;*/

	//create new media sample
	cObjectPtr<IMediaSample> pSampleAccel;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleAccel));

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pSampleAccel->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pSampleAccel, pCoderOutput);
		if (m_usCurrentDistance < MIN_FREE_DISTANCE && m_usDetectionEnabled)         
		{
			f32Acceleration = 0;  
		}
		pCoderOutput->Set("f32Value", (tVoid*)&f32Acceleration);    


		pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
	}

	pSampleAccel->SetTime(tsInputTime);
	RETURN_IF_FAILED(m_oAccelerateOutput.Transmit(pSampleAccel));

	RETURN_NOERROR;
}

tResult cLaneTracking::LaneOffsetAcceleration(tFloat32 offset, tTimeStamp ts)
{
    __synchronized_obj(m_oTransmitLaneOffsetCritSection);

    //create new media sample
	cObjectPtr<IMediaSample> pSampleLaneOffset;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleLaneOffset));

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pSampleLaneOffset->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pSampleLaneOffset, pCoderOutput);
		
		pCoderOutput->Set("f32Value", (tVoid*)&offset);    


		pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
	}

	pSampleLaneOffset->SetTime(ts);
	RETURN_IF_FAILED(m_oLaneCenterOffsetOutput.Transmit(pSampleLaneOffset));


    RETURN_NOERROR;
}

tResult cLaneTracking::TransmitSteeringAngle(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
{
	__synchronized_obj(m_oTransmitSteerCritSection);

	/*static tFloat32 SteerLastTransmitted = 100.0;

	if (fabs(SteeringAngle - SteerLastTransmitted)< 0.00001f)
	{
	RETURN_NOERROR;
	}

	SteerLastTransmitted = SteeringAngle;*/

	//create new media sample
	cObjectPtr<IMediaSample> pNewMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pNewMediaSample, pCoderOutput);    

		pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle);    
		pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
	}

	pNewMediaSample->SetTime(tsInputTime);
	RETURN_IF_FAILED(m_oSteeringAngleOutput.Transmit(pNewMediaSample));

	//LOG_INFO(cString::Format("Sending SteeringAngle: %f", SteeringAngle).GetPtr());

	RETURN_NOERROR;

}

tResult cLaneTracking::TransmitSteeringAnglePT1(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
{
	__synchronized_obj(m_oTransmitSteerCritSection);

	/*static tFloat32 SteerLastTransmitted = 100.0;

	if (fabs(SteeringAngle - SteerLastTransmitted)< 0.00001f)
	{
	RETURN_NOERROR;
	}

	SteerLastTransmitted = SteeringAngle;*/
	tFloat32 f32SteeringAngle1 = f32SteeringAngle+90.f;
	//create new media sample
	cObjectPtr<IMediaSample> pNewMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pNewMediaSample, pCoderOutput);    

		pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle1);    
		pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
	}

	pNewMediaSample->SetTime(tsInputTime);
	RETURN_IF_FAILED(m_oSteeringAnglePT1Output.Transmit(pNewMediaSample));

	//LOG_INFO(cString::Format("Sending SteeringAnglePT1: %f", SteeringAngle).GetPtr());

	RETURN_NOERROR;

}

tResult cLaneTracking::TransmitHeadLights(const tBool bHeadLights, tTimeStamp tsInputTime)
{
	__synchronized_obj(m_oTransmitLightCritSection);

	//create new media sample
	cObjectPtr<IMediaSample> pNewMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescBool->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

	{
		__adtf_sample_write_lock_mediadescription(m_pCoderDescBool, pNewMediaSample, pCoderOutput);    

		pCoderOutput->Set("bValue", (tVoid*)&bHeadLights);    
		pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
	}

	pNewMediaSample->SetTime(tsInputTime);
	RETURN_IF_FAILED(m_oHeadLightsOutput.Transmit(pNewMediaSample));

	RETURN_NOERROR;

}

tResult cLaneTracking::CreateAndTransmitGCL()
{
	// just draw gcl if the pin is connected and debug mode is enabled
	if (!m_oGCLOutput.IsConnected() || !m_bShowDebug)
	{
		RETURN_NOERROR;
	}

	// create a mediasample
	cObjectPtr<IMediaSample> pSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

	RETURN_IF_FAILED(pSample->AllocBuffer(8192));

	pSample->SetTime(_clock->GetStreamTime());

	tUInt32* aGCLProc;
	RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

	tUInt32* pc = aGCLProc;

	// draw rectangle to scale the video display correctly
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, m_sInputFormat.nWidth, m_sInputFormat.nHeight);

	// draw near and far area
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 255).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nCurrentNearLine - 20, m_sInputFormat.nWidth, m_nCurrentNearLine + 20);
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nFarLine - 20, m_sInputFormat.nWidth, m_nFarLine + 20);

	// draw the min and max lane width
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(150,150,150).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMaxFar/2, m_nFarLine, m_i16FarLaneCenter + m_i16LaneWidthMaxFar/2, m_nFarLine + 5);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMinFar/2, m_nFarLine - 5, m_i16FarLaneCenter + m_i16LaneWidthMinFar/2, m_nFarLine);

	cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_sLaneCenterNear.x - m_i16LaneWidthMaxNear/2, m_nCurrentNearLine, m_sLaneCenterNear.x + m_i16LaneWidthMaxNear/2, m_nCurrentNearLine + 5);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_sLaneCenterNear.x - m_i16LaneWidthMinNear/2, m_nCurrentNearLine - 5, m_sLaneCenterNear.x + m_i16LaneWidthMinNear/2, m_nCurrentNearLine);

	// draw near and far line
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nCurrentNearLine, m_sInputFormat.nWidth, m_nCurrentNearLine);
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nFarLine, m_sInputFormat.nWidth, m_nFarLine);	

	// draw the lines for place to be and the detected lanecenter
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,150,0).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_sLaneCenterNear.x, m_nCurrentNearLine - 20, m_sLaneCenterNear.x, m_nCurrentNearLine + 20);
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_sPlaceToBe.x, m_nCurrentNearLine - 10, m_sPlaceToBe.x, m_nCurrentNearLine + 10);
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_i16FarLaneCenter, m_nFarLine - 20, m_i16FarLaneCenter, m_nFarLine + 20);

	// draw a rect for longitudinal control
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_i16FarLaneCenter - m_nAccelerationFarNearDiff, m_nFarLine - 20, m_i16FarLaneCenter + m_nAccelerationFarNearDiff, m_nCurrentNearLine + 20);


	// draw the found near lines
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,50,100).GetRGBA());
	for (tInt nIdx = 0; nIdx < m_ui8NearPointsCount; ++nIdx)
	{
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsNear[nIdx].x, m_nCurrentNearLine - 20, m_asAllpointsNear[nIdx].x, m_nCurrentNearLine + 20);    
	}

	// draw the found far lines
	for (tInt nIdx = 0; nIdx < m_ui8FarPointsCount; ++nIdx)
	{
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsFar[nIdx].x, m_nFarLine - 20, m_asAllpointsFar[nIdx].x, m_nFarLine + 20);    
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_END);

	pSample->Unlock(aGCLProc);

	RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
	RETURN_NOERROR;

}
