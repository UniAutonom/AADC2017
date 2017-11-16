/*****************************************************************************
* Copyright (c) 2017 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.             *
* All rights reserved.                                                       *

Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hartlan $  $Date:: 2017-04-28 13:31:45#$ $Rev:: 62678   $
**********************************************************************/
#ifndef _LaneTracking_FILTER_HEADER_
#define _LaneTracking_FILTER_HEADER_

#define OID_ADTF_LaneTracking_jury  "adtf.aadc.aadc_laneTracking_jury"

#include "aadc_enums.h"

#define LT_POINTS_ARRAY_SIZE 640

/*! @defgroup LaneTracking Lane Tracking
*  @{
*  
* Text here
*  
*  \image html .PNG "Plugin Fisheye Undistortion"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li OpenCV  v.3.2.0
*
* <b> ADTF Versions </b> \n
* This plugin was built and tested with the following ADTF Versions:
* \li ADTF v.2.13.3
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td><td><td>
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType<th>Details* 
* <tr><td><td><td><td>
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td><td><td><td>
* </table>
*    
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>
* <tr><td>Filename<td>aadc_.plb
* <tr><td>Version<td>1.0.0
* </table>
*    
*/

class cLaneTracking : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LaneTracking_jury, "AADC LaneTracking Jury", OBJCAT_Tool, "AADC Lane Tracking Jury", 1, 0, 1, "BFFT GmbH");    
    
protected:
    //Eingang für RGB Bild
    cVideoPin           m_oVideoInputPin;
    cInputPin           m_oInputStartLightbeam; 
        
    cOutputPin          m_oGCLOutput;
    cOutputPin          m_oAccelerateOutput;
    cOutputPin          m_oSteeringAngleOutput;
    cOutputPin          m_oSteeringAnglePT1Output;
    cOutputPin          m_oLaneCenterOffsetOutput;
    cOutputPin          m_oHeadLightsOutput;

    
    /*! Input pin for the ultrasonic front center  data */
    cInputPin       m_oInputUsFrontCenter;

    /*! processes the mediasample with ultrasonic data to the gui
    @param pMediaSample the incoming mediasample
    @param usSensor the enum for the data of the mediasample
    */
    tResult ProcessUltrasonicSample(IMediaSample* pMediaSample, SensorDefinition::ultrasonicSensor usSensor);   

    
    /*! descriptor for ultrasonic sensor data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData; 
    /*! the id for the f32value of the media description for input pin of the ultrasoncic data */
    tBufferID m_szIDUltrasonicF32Value; 
    /*! the id for the arduino time stamp of the media description for input pin of the ultrasoncic data */
    tBufferID m_szIDUltrasonicArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsUltrasonicSet;

	/*! the vector with the last distances*/
    std::list<tFloat32> m_usDistanceVector;

	/*! the current free distance */
    tFloat32 m_usCurrentDistance;

	/*! the set free us distance threshold*/ 
    tFloat32 m_usMinFreeThreshold;

	/*! if us free detection is enabled */
	tBool m_usDetectionEnabled;

	/*! the pin for enable*/ 
	cInputPin m_oInputEnablePin;

	/*! Coder Descriptor for input set state error */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionEnablePin;
    /*! the id for the bool value output of the media description */
    tBufferID m_szIDEnablePinbValue;     
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDEnablePinArduinoTimestamp; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsEnablePinSet;

	/*! processes the mediasample with enable data 
		@param pMediaSample the incoming mediasample
    */
	tResult ProcessEnableSample(IMediaSample* pMediaSample);

	/*! output pin for the the distance of distance since adtf start config */
    cInputPin m_oInputDistanceOverall;

	/*! descriptor for sample distance data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInputOverallDistance;
    /*! the id for the f32value of the media description for the actuator output pins */
    tBufferID m_szIDOverallDistanceF32Value; 
    /*! the id for the arduino time stamp of the media description for the actuator output input pins */
    tBufferID m_szIDOverallDistanceArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsOverallDistanceSet;

	/*! the overall distance */
	tFloat32 m_f32OverallDistanceThreshold;

	/*! the distance when the start trigger was received*/
	tFloat32 m_f32DistanceStartValue;

	/*! the most current distance */
	tFloat32 m_f32CurrentDistance;

	/*! processes the mediasample with overall distance data 
		@param pMediaSample the incoming mediasample
    */
	tResult ProcessOverallDistanceSample(IMediaSample* pMediaSample);

public:

    struct sPoint 
    {
        tInt16 x;
        tInt16 y;
    };

                
    cLaneTracking(const tChar*);
    virtual ~cLaneTracking();

    // implements cFilter
    tResult Init(tInitStage eStage, __exception=NULL);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult PropertyChanged(const char* strProperty);

public: // implements ISignalProvider
    virtual tResult GetSignalValue(tSignalID nSignalID, tSignalValue* pValue);
    virtual tResult ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate = 0);
    virtual tResult DeactivateSignalEvents(tSignalID nSignalID);

public: // implements IObject
    tResult GetInterface(const tChar* idInterface, tVoid** ppvObject);
    tUInt Ref();
    tUInt Unref();
    tVoid Destroy();

private: // private methods
    tResult Search(sPoint *points,tUInt8 *lfdnr,tUInt8 limit,cv::Mat cannyimage);
    tResult TransmitAcceleration(tFloat32, tTimeStamp);
    tResult LaneOffsetAcceleration(tFloat32, tTimeStamp);
    tResult TransmitSteeringAngle(const tFloat32, tTimeStamp);
    tResult TransmitSteeringAnglePT1(const tFloat32, tTimeStamp);
    tResult TransmitHeadLights(const tBool, tTimeStamp);
    tResult LateralControl(sPoint*, tUInt8*, tTimeStamp);
    tResult LongitudinalControl(sPoint*, tUInt8*, tTimeStamp);
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
    tResult ProcessFound();
    tResult ProcessOutput();
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();


protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

private:
    //für Search
    sPoint      m_asAllpointsNear[LT_POINTS_ARRAY_SIZE];
    tUInt8      m_ui8NearPointsCount;
    sPoint      m_asAllpointsFar[LT_POINTS_ARRAY_SIZE];
    tUInt8      m_ui8FarPointsCount;

    // points and values for lane detection
    sPoint      m_sLaneCenterNear;
    tInt16      m_i16LaneWidth;
    sPoint      m_sPlaceToBe;
    tInt16      m_i16LaneWidthMinNear;
    tInt16      m_i16LaneWidthMaxNear;  
    tInt16      m_i16LaneWidthMinFar;
    tInt16      m_i16LaneWidthMaxFar;  

    // offset of the camera to the mid of the vehicle
    tFloat64    m_f64CamOffset;
    
    tInt        m_nCurrentNearLine;

    // PID-Controller values
    tFloat64    m_f64Kp;
    tFloat64    m_f64Ki;
    tFloat64    m_f64Kd;
    tInt        m_nCenterFromLeft;
    tInt        m_nCenterFromRight;
    tInt16      m_i16ErrorSum;
    tInt16      m_i16ErrorOld; 
    tFloat32    m_f32Ts;
    tInt16      m_i16FarLaneCenter;
    tInt16      m_i16FarLaneWidth;
    tFloat32    m_f32AccelerateOut;
    tInt16      m_i16FarLaneWidthCalc;

    // PT1-Controller values
    tFloat64    m_f64PT1Tau;
    tFloat64    m_f64PT1Sample;
    tFloat64    m_f64PT1Gain;
    tFloat32    m_f32PT1LastSteeringOut;
    tFloat32    m_f32PT1SteeringOut;
    tFloat64    m_f64PT1InputFactor;



    // active flag to enable driving
    tBool       m_bActive;

	tInt		m_iStopMode;

	tBool		m_bLightBeamOnlyOnce;

	tBool		m_bWasLightbeamTriggered;
        
    // opencv members for line detection
    cv::Mat     m_matLine;    
    cv::Mat     m_matGreyNear;
    cv::Mat     m_matGreyFar;
    cv::Mat     m_matGreyThreshNear;
    cv::Mat     m_matGreyThreshFar;
    cv::Size    m_szCannySize;
    //cv::Size  m_szCannySizeFar;
    cv::Mat     m_matLineCannyNear;
    cv::Mat     m_matLineCannyFar;

    // DDL descriptions
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;

    // flag to check the first frame
    bool        m_bFirstFrame;
    // image counter 
    tUInt8      m_ui8Imagecount;
    // arduino timestamp value
    tUInt32     m_tsArduinoTime;
    // bitmap format of input pin
    tBitmapFormat m_sInputFormat;
        
    //Properties:
    
    tUInt8      m_ui8InitCtrl;
    tInt        m_nNearLine;
    tInt        m_nNearLineMaxOffset;
    tInt        m_nFarLine;
    tFloat32    m_f32AccelerationMax;
    tFloat32    m_f32AccelerationMin;
    tInt        m_nAccelerationFarNearDiff;
    tFloat32    m_f32SteeringAngle;
    tInt        m_nThresholdValue;
    tInt        m_nBlindCounter;
    tInt        m_nBlindCounterFar;
    tInt        m_nDriveTime;
    tInt        m_nEmergencyStopTime;
        
    tBool       m_bShowDebug;
    tBool       m_bLightBeamTriggerEnabled;
        
    tHandle     m_hStopTimerNegative;
    tHandle     m_hStopTimerZero;
    tHandle     m_hEmergencyStopTimerNegative;
    tHandle     m_hEmergencyStopTimerZero;
    tHandle     m_hEmergencyStopTimerResume;
        
    // Critical Sections
    cCriticalSection    m_oTransmitSteerCritSection;
      cCriticalSection    m_oTransmitLaneOffsetCritSection;
    cCriticalSection    m_oTransmitAccelCritSection;
    cCriticalSection    m_oTransmitLightCritSection;
    cCriticalSection    m_oRunCritSection;

    // members for signal registry
    typedef std::set<tSignalID> tActiveSignals;

    ucom::cObjectPtr<ISignalRegistryExtended> m_pISignalRegistry;
    cKernelMutex                              m_oLock;
    tActiveSignals                            m_oActive;

    // caching the error values for signal registry
    tFloat64            m_f64PT1ScaledError;
    tInt16              m_i16Error;
        
};

#endif 
