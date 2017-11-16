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
* $Author:: spie#$  $Date:: 2017-05-22 18:08:00#$ $Rev:: 63774   $
**********************************************************************/
#ifndef _LANEDETECTION_FILTER_HEADER_
#define _LANEDETECTION_FILTER_HEADER_

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"
#include "aadc_roadSign_enums.h"
#include "aadc_enums.h"


#define OID_ADTF_FILTER_DEF "adtf.aadc_LaneDetection" //unique for a filter
#define ADTF_FILTER_DESC "AADC Lane Detection"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "OpenCVLaneDetection"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
#define NO_LD_SPEED 999
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A small Lane DetectionFilter \n$Rev:: 62948"

/*! @defgroup LaneDetection Lane Detection
*  @{
*
*  \image html LaneDetction.PNG "Plugin Lane Detection Filter"
*
* This filter does a very simple detection of lanes in the input image and can be used by the teams as a startup for their own implementation.
* It does the following subsequent steps:
* \li binarize input image with threshold set in the properties "Threshold for image binarization"
* \li calculate the horizontal lines where to search for the Lanes. This is defined by the ROI and the number "Algorithm::Detection Lines" set in the filter properties
* \li iterate through the detection lines and find transitions with high contrasts. For each detection line we iterate from left to right and search for lines within the maximum and minimum width defined in "Algorithm::Minimum Line Width" and "Algorithm::Maximum Line Width"
* \li# all the found linepoints are added to one vector
* # suggested: do a classification which point is left line, middle line, right line
* \li# suggested: calculate a clothoide for each line
* \li# suggested: calculate a clothoide for the car
*
* This plugin needs the following libraries:
* \li OpenCV  v.3.2.0
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>ROI::XOffset <td>X Offset for Region of Interest Rectangular<td>0
* <tr><td>ROI::YOffset <td>Y Offset for Region of Interest Rectangular<td>500
* <tr><td>ROI::Width <td>Width of the Region of Interest Rectangular<td>1280
* <tr><td>ROI::Height <td>Height of the Region of Interest Rectangular<td>200
* <tr><td>Algorithm::Detection Lines<td>number of detection lines searched in ROI<td>10
* <tr><td>Algorithm::Minimum Line Width<td>Minimum Line Width in Pixel<td>10
* <tr><td>Algorithm::Maximum Line Width<td>Maximum Line Width in Pixel<td>30
* <tr><td>Algorithm::Minimum Line Contrast<td>Mimimum line contrast in gray Values<td>50
* <tr><td>Algorithm::Image Binarization Threshold<td>Threshold for image binarization<td>180
* </table>
*
* <b>Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Output_Debug<td>Video Output for debugging<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>GCL<td>GCL with debug information<td>MEDIA_TYPE_COMMAND<td>MEDIA_SUBTYPE_COMMAND_GCL
*</table>
*
* <b>Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Input<td>Video Pin for data from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/algorithms/AADC_LaneDetection
* <tr><td>Filename<td>aadc_LaneDetection.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/

//!  Template filter for OpenCV Image Processing
/*!
* This class is the main class of the OpenCV Template Filter and can be used as template for user specific image processing filters
*/

#define SENSOR_COUNT 10
class cLaneDetection : public adtf::cFilter
{

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_Auxiliary,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                        );

protected:

    cInputPin       m_oInputManeuver;
    cInputPin       m_oInputWheelLeft;
    cInputPin       m_oInputWheelRight;
    cInputPin   m_oInputSpeedController;

    cInputPin              m_oInputUsStruct; //tUltrasonicStruct
    std::vector<tBufferID> m_szIdUsStructValues;
    std::vector<tBufferID> m_szIdUsStructTimeStamps;
    tBool                  m_szIdsUsStructSet;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;

    /*! The critical section Minimum Us Value */
    cCriticalSection m_critSecMinimumUsValue;
    tFloat32 m_actualDistances[SENSOR_COUNT];

    tResult ProcessMinimumValueUs(IMediaSample* pMediaSample);

    cVideoPin           m_oVideoInputPin;
    cVideoPin           m_oVideoOutputPin;
    cOutputPin           m_oGCLOutputPin;
    cOutputPin           m_oSteeringAngleOutputPin;
    cOutputPin           m_oManeuverFinishedOutputPin;
    cOutputPin  m_oOutputTicksToLine; // output pin for ticks to line
    cOutputPin  m_oOutputParkingspaces;
    tBufferID   m_szIdSteeringAngleValue;
    tBufferID   m_szIdSteeringAngleTimestamp;
    tBufferID m_szIdInputspeedControllerValue;

    // output pin for speed
    cOutputPin  m_oOutputSpeedController;  //typ tSignalValue
    tBufferID   m_szIdOutputspeedControllerValue;
    tBufferID   m_szIdOutputspeedControllerTimeStamp;
    tBool       m_szIdOutputSpeedSet;

    cOutputPin m_oOutputReverseLight; // The output pin for reverse light
    cOutputPin m_oOutputTurnRight;    // The output pin for turn right controller
    cOutputPin m_oOutputTurnLeft;     // The output pin for turn left controller
    cOutputPin m_oOutputHazzardLight; // The output pin for hazzard light

    cObjectPtr<IMediaTypeDescription> m_pDescriptionSteeringAngle;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionManeuverFinished;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelLeftData;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelRightData;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionManeuverData;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionTicksToLine;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionParkingspaces;

    cCriticalSection m_critSecTransmitControl;
    cCriticalSection m_critSecTransmitBool; // The critical section transmit bool
    cCriticalSection m_critSecTransmitParkingSpaces;

public:
    /*! default constructor for template class
        \param __info   [in] This is the name of the filter instance.
    */
    cLaneDetection(const tChar* __info);

    /*! default destructor */
    virtual ~cLaneDetection();

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




    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by CurvatureOffsetthe special filter.
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

    /*! This Function is always called when any property has changed. This should be the only place
    *    to read from the properties itself and store their values in a member.
    *
    *    \param [in] strName the name of the property that has changed.
    *    \
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* strName);

private: // private methods

    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    *   \param pFormat the new format for the input pin
    *   \return Standard Result Code.
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);

    tResult TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp);

    /*! function to set the output image format
    *   \param outputImage the new format for the input pin
    *   \return Standard Result Code.
    */
    tResult UpdateOutputImageFormat(const cv::Mat& outputImage);

    tResult TransmitTicksToCrosspoint(int ticksToLine);

    /*! function to process the mediasample
    *   \param pSample the new media sample
    *   \return Standard Result Code.
    */
    tResult ProcessVideo(IMediaSample* pSample);

    /*! bitmap format of input pin */
    tBitmapFormat m_sInputFormat;

    /*! bitmap format of output pin */
    tBitmapFormat m_sOutputFormat;

    /*! tha last received input image*/
    Mat m_inputImage;

    struct line {
        cPoint startPoint;
        cPoint endPoint;
        line(cPoint s, cPoint e){
            startPoint = s;
            endPoint = e;
        }
    };
    struct stopLine {
        Vec4i firstLine;
        Vec4i secondLine;
    };
    struct maneuver {
        int ticks;
        float steeringAngle;
        float speed;   // 999 => no speed change
        bool turnLeft;  // false = light off, true = light on
        bool turnRight; // false = light off, true = light on
    };

    bool turnLeftOn = false;
    bool turnRightOn = false;

    bool DEBUG = false;

    bool EMERGENCY_SEARCH;
    int EME_SEARCH_NO_STREET_COUNTER;
    Vec4i last_right_line;
    Vec4i last_middle_line;
    Vec4i last_left_line;

    int lastPedestianCrossingDetected;
    bool nextManeuverRight;
    bool nextManeuverLeft;
    bool nextManeuverStraight;
    bool nextManeuverParking;
    bool nextManeuverParkingSpaceSearch;
    bool nextManeuverParkingOutLeft;
    bool nextManeuverParkingOutRight;
    bool parkingTestRunning;
    tInt16 maneuverID;


    vector<Vec4i> leftLineBuffer;
    vector<Vec4i> middleLineBuffer;
    vector<Vec4i> rightLineBuffer;
    vector<tFloat32> steeringAngleBuffer;
    vector<tFloat32> laneBufferRight;
    vector<tFloat32> laneBufferMiddle;
    vector<tFloat32> laneBufferLeft;
    vector<tFloat32> angleBufferRight;
    vector<tFloat32> angleBufferMiddle;
    vector<tFloat32> angleBufferLeft;
    vector<maneuver> maneuverList;
    bool doManeuver;
    bool maneuverNotSet;
    bool m_szIdInputSpeedSet;
    bool m_szIdOutputSteeringAngleSet;
    bool m_szIdOutputManeuverFinishedSet;
    bool m_bIDManeuverSendSet;
    bool firstGoThrough;
    int faultyLineCounterRight;
    int faultyLineCounterMiddle;
    int faultyLineCounterLeft;
    int oldAngleMiddle;
    int oldAngleRight;
    int oldAngleLeft;
    int oldXPosRight;
    int oldXPosMiddle;
    int oldXPosLeft;
    int goThroughCounter;
    int rightXPos;
    int middleXPos;
    int leftXPos;
    int wheelCountLeft;
    int wheelCountRight;
    int oldTicks;
    int m_manoeverCount;
    int wheelTickSave;
    int walkThroughCounter_steeringAngle;
    float deviationSum;
    float xPosDeviationSum;
    float oldxPosDeviation;
    float oldAngle;
    tUInt64 oldTime;
    tFloat32 carSpeed;
    Mat lambda;
    int parkingSpaceBeginTicks;
    tFloat32 lastSpeedSend;

    tInt8 parkingSpaceID;

    int m_stManTicks;
    float lastSteeringangle;

    //parking
    int parkingEndCounter;
    int isItEmptyCounter;

    bool isOnLeftLane;
    int overtakingTicks;

    bool lastTurnRightOnSend;
    bool lastTurnLeftOnSend;


    /*! the struct with all the properties*/
    struct filterProperties
    {
        //Canny
        int thresholdImageBinarization;
        bool testBool;
        //HoughLines
        float HLrho;
        float HLtheta;
        int HLthreshold;
        float HLminLineLenght;
        float HLmaxLineGap;
        //StreetDetectionROI
        int LeftLowerLeftX;
        int LeftLowerLeftY;
        int LeftLowerRightX;
        int LeftLowerRightY;
        int LeftUpperLeftX;
        int LeftUpperLeftY;
        int LeftUpperRightX;
        int LeftUpperRightY;
        int MidLowerLeftX;
        int MidLowerLeftY;
        int MidLowerRightX;
        int MidLowerRightY;
        int MidUpperLeftX;
        int MidUpperLeftY;
        int MidUpperRightX;
        int MidUpperRightY;
        int RightLowerLeftX;
        int RightLowerLeftY;
        int RightLowerRightX;
        int RightLowerRightY;
        int RightUpperLeftX;
        int RightUpperLeftY;
        int RightUpperRightX;
        int RightUpperRightY;
        float maxAngle;
        float minAngle;
        int curvatureOffsetRight;
        int curvatureOffsetLeft;
        int curvatureOffsetMid;
        //Laneholding
        int faultyCounterThreshold;
        int emergencyThreshold;
        float FovCurveRight;
        float FovCurveLeft;
        //WarpPerspective
        int warpLeft;
        int warpRight;
        int warpLeftL;
        int warpRightL;
        //Regler
        float Kp;
        float Ki;
        float Kd;
        float K1;
        float K2;
        float K3;
        float K4;
        //StoplineDetection
        float distanceOffset;
        float stopLineDistance;
        float stopLineLength;
        float lengthOffset;
        //CrossDetectionROI
        int fourCrossLowerLeftX;
        int fourCrossLowerLeftY;
        int fourCrossUpperRightX;
        int fourCrossUpperRightY;
        //Maneuver
        int verticalTicks;
        int horizontalTicks;
        int pixelToTicks;
    }
    /*! the filter properties of this class */
    m_filterProperties;

    /*!
     * Transmit gcl.
     *
     * \param   detectionLines  The detection lines.
     * \param   detectedLinePoints  The left lane pixels.
     *
     * \return  A tResult.
     */
    tResult transmitGCL(const vector<tInt>& detectionLines, const vector<cPoint>& detectedLinePoints, const vector<line> detectedLines);
    /*!
     * Transmit gcl.
     *
     * \param   detectionLines  The detection lines.
     * \param   detectedLinePoints  The left lane pixels.
     *
     * \return  A tResult.
     */

    tResult transmitSteeringAngle(tFloat32 SteeringAngle);
    tFloat32 computeSteeringAngle(Vec4i leftLane, Vec4i middleLane, Vec4i rightLane, Mat &outputImage);
    void calculateROI(cv::Mat& outputImage, Point** poly);
    tFloat32 computeAngleFromVec4i(const Vec4i vector);
    tFloat32 computeRadianFromVec4i(const Vec4i vector);
    Vec4i computeNearestLineToCar(vector<Vec4i> &lineVector, string lanePosition);
    tResult detectLines(Mat &outputImage, vector<Vec4i> &detectedLines);
    tResult detectLane(Mat &outputImage, vector<Vec4i> &lines, Vec4i &nearestLineToCarRight, Vec4i &nearestLineToCarMiddle, Vec4i &nearestLineToCarLeft);
    tResult adaptROI(Vec4i &lineRight, Vec4i &lineMiddle, Vec4i &lineLeft);
    tResult setStartEndPoint(vector<Vec4i> &lines);
    tResult bufferSteeringangle(int &steeringAngle, int countRight, int countLeft);
    tResult ProcessWheelSampleLeft(IMediaSample* pMediaSample);
    tResult ProcessWheelSampleRight(IMediaSample* pMediaSample);
    tResult GetSpeed(IMediaSample* pMediaSample);
    tResult detectStopLines(vector<Vec4i> &lines, vector<stopLine>&stopLines, vector<stopLine>&stopLinesHorizontal, Mat &outputImage);
    tResult setManeuvers(vector<maneuver> man);
    tResult processManeuvers(IMediaSample* pMediaSample);
    tResult perspective_to_maps(const cv::Mat &perspective_mat, const cv::Size &img_size,
                              cv::Mat &map1, cv::Mat &map2);
    tResult checkCrossManeuver(vector<Point> &rightCrossPoint, vector<Point> &leftCrossPoint, Vec4i gapVectorVertical, Vec4i gapVectorHorizontal);
    tResult transmitManeuverFinished(tInt16 maneuverID);
    tResult detectParkingSpot(vector<Vec4i> &lines, bool& successful, int& y_offset, Mat& outputImage);
    tResult detectEmergencyHorizontalStoplines(vector<Vec4i> lines, vector<stopLine> &stopLinesHorizontal);
    tResult SwitchToLaneLeft();
    tResult SwitchToLaneRight();
    tResult checkParkingManeuver(vector<Vec4i> &lines, Mat& outputImage);
    tResult TransmitParkingspaces(cOutputPin* oPin, vector<bool> parkingSpaces);
    tResult parkingSpaceSearch(vector<Vec4i> &lines, vector<bool> &parkingSpaces, Mat& outputImage);
    tBool Intersection(Vec4i Vec1, Vec4i Vec2);
    tBool IntersectionUL(Vec4i line, int upperX,int lowerX,int upperY,int lowerY);
    tResult detectCrossPoints(Mat& outputImage, vector<Vec4i> &lines, vector<Point> &rightCrossPoints, vector<Point> &leftCrossPoints, Vec4i &gapVectorVertical, Vec4i &gapVectorHorizontal, Vec4i &nearestLineToCarRight);
};

/** @} */ // end of group

#endif  //_OPENCVTEMPLATE_FILTER_HEADER_
