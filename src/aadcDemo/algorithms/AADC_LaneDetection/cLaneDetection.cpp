
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
* $Author:: spiesra $  $Date:: 2017-05-22 18:08:00#$ $Rev:: 63774   $
**********************************************************************/
#include "stdafx.h"
#include "cLaneDetection.h"
using namespace roadsignIDs;
using namespace Unia;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   cLaneDetection)

cLaneDetection::cLaneDetection(const tChar* __info) : cFilter(__info)
{
    SetPropertyInt("Algorithm::Image Binarization Threshold", 220);
    SetPropertyStr("Algorithm::Image Binarization Threshold" NSSUBPROP_DESCRIPTION, "Threshold for image binarization");
    SetPropertyBool("Algorithm::Image Binarization Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MIN, 1);
    SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MAX, 255);

    SetPropertyBool("Algorithm::TestBool", false);
    SetPropertyStr("Algorithm::TestBool" NSSUBPROP_DESCRIPTION, "Bool to turn on/off things");
    SetPropertyBool("Algorithm::TestBool" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("ROI::LowerRightY", 5);
    SetPropertyStr("ROI::LowerRightY" NSSUBPROP_DESCRIPTION, "Polygon Lower Right y offset");
    SetPropertyBool("ROI::LowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::CurvatureOffsetRight", 1);
    SetPropertyStr("StreetDetection::CurvatureOffsetRight" NSSUBPROP_DESCRIPTION, "Offset for curves");
    SetPropertyBool("StreetDetection::CurvatureOffsetRight" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::CurvatureOffsetLeft", 1);
    SetPropertyStr("StreetDetection::CurvatureOffsetLeft" NSSUBPROP_DESCRIPTION, "Offset for curves");
    SetPropertyBool("StreetDetection::CurvatureOffsetLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::CurvatureOffsetMid", 1);
    SetPropertyStr("StreetDetection::CurvatureOffsetMid" NSSUBPROP_DESCRIPTION, "Offset for curves");
    SetPropertyBool("StreetDetection::CurvatureOffsetMid" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("HoughLines::rho", 2);
    SetPropertyStr("HoughLines::rho" NSSUBPROP_DESCRIPTION, "Hough Lines");
    SetPropertyBool("HoughLines::rho" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("HoughLines::theta", CV_PI/180);
    SetPropertyStr("HoughLines::theta" NSSUBPROP_DESCRIPTION, "Hough Lines");
    SetPropertyBool("HoughLines::theta" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("HoughLines::threshold", 50);
    SetPropertyStr("HoughLines::threshold" NSSUBPROP_DESCRIPTION, "Hough Lines");
    SetPropertyBool("HoughLines::threshold" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("HoughLines::minLineLenght", 30);
    SetPropertyStr("HoughLines::minLineLenght" NSSUBPROP_DESCRIPTION, "Hough Lines");
    SetPropertyBool("HoughLines::minLineLenght" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("HoughLines::maxLineGap", 30);
    SetPropertyStr("HoughLines::maxLineGap" NSSUBPROP_DESCRIPTION, "Hough Lines");
    SetPropertyBool("HoughLines::maxLineGap" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftLowerLeftX", 100);
    SetPropertyStr("StreetDetection::LeftLowerLeftX" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
    SetPropertyBool("StreetDetection::LeftLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftLowerLeftY", 100);
    SetPropertyStr("StreetDetection::LeftLowerLeftY" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
    SetPropertyBool("StreetDetection::LeftLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftLowerRightX", 100);
    SetPropertyStr("StreetDetection::LeftLowerRightX" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
    SetPropertyBool("StreetDetection::LeftLowerRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftLowerRightY", 100);
    SetPropertyStr("StreetDetection::LeftLowerRightY" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
    SetPropertyBool("StreetDetection::LeftLowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftUpperLeftX", 100);
    SetPropertyStr("StreetDetection::LeftUpperLeftX" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
    SetPropertyBool("StreetDetection::LeftUpperLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftUpperLeftY", 100);
    SetPropertyStr("StreetDetection::LeftUpperLeftY" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
    SetPropertyBool("StreetDetection::LeftUpperLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftUpperRightX", 100);
    SetPropertyStr("StreetDetection::LeftUpperRightX" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
    SetPropertyBool("StreetDetection::LeftUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::LeftUpperRightY", 100);
    SetPropertyStr("StreetDetection::LeftUpperRightY" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
    SetPropertyBool("StreetDetection::LeftUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidLowerLeftX", 100);
    SetPropertyStr("StreetDetection::MidLowerLeftX" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
    SetPropertyBool("StreetDetection::MidLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidLowerLeftY", 100);
    SetPropertyStr("StreetDetection::MidLowerLeftY" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
    SetPropertyBool("StreetDetection::MidLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidLowerRightX", 100);
    SetPropertyStr("StreetDetection::MidLowerRightX" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
    SetPropertyBool("StreetDetection::MidLowerRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidLowerRightY", 100);
    SetPropertyStr("StreetDetection::MidLowerRightY" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
    SetPropertyBool("StreetDetection::MidLowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidUpperLeftX", 100);
    SetPropertyStr("StreetDetection::MidUpperLeftX" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
    SetPropertyBool("StreetDetection::MidUpperLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidUpperLeftY", 100);
    SetPropertyStr("StreetDetection::MidUpperLeftY" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
    SetPropertyBool("StreetDetection::MidUpperLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidUpperRightX", 100);
    SetPropertyStr("StreetDetection::MidUpperRightX" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
    SetPropertyBool("StreetDetection::MidUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::MidUpperRightY", 100);
    SetPropertyStr("StreetDetection::MidUpperRightY" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
    SetPropertyBool("StreetDetection::MidUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightLowerLeftX", 100);
    SetPropertyStr("StreetDetection::RightLowerLeftX" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
    SetPropertyBool("StreetDetection::RightLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightLowerLeftY", 100);
    SetPropertyStr("StreetDetection::RightLowerLeftY" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
    SetPropertyBool("StreetDetection::RightLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightLowerRightX", 100);
    SetPropertyStr("StreetDetection::RightLowerRightX" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
    SetPropertyBool("StreetDetection::RightLowerRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightLowerRightY", 100);
    SetPropertyStr("StreetDetection::RightLowerRightY" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
    SetPropertyBool("StreetDetection::RightLowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightUpperLeftX", 100);
    SetPropertyStr("StreetDetection::RightUpperLeftX" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
    SetPropertyBool("StreetDetection::RightUpperLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightUpperLeftY", 100);
    SetPropertyStr("StreetDetection::RightUpperLeftY" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
    SetPropertyBool("StreetDetection::RightUpperLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightUpperRightX", 100);
    SetPropertyStr("StreetDetection::RightUpperRightX" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
    SetPropertyBool("StreetDetection::RightUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("StreetDetection::RightUpperRightY", 100);
    SetPropertyStr("StreetDetection::RightUpperRightY" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
    SetPropertyBool("StreetDetection::RightUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("StreetDetection::minAngle", 10);
    SetPropertyStr("StreetDetection::minAngle" NSSUBPROP_DESCRIPTION, "Minimum angle of Line");
    SetPropertyBool("StreetDetection::minAngle" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("StreetDetection::maxAngle", 10);
    SetPropertyStr("StreetDetection::maxAngle" NSSUBPROP_DESCRIPTION, "Maximum angle of line");
    SetPropertyBool("StreetDetection::maxAngle" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("LaneHolding::FaultyLaneThreshold", 10);
    SetPropertyStr("LaneHolding::FaultyLaneThreshold" NSSUBPROP_DESCRIPTION, "Counter for faulty lanes");
    SetPropertyBool("LaneHolding::FaultyLaneThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("LaneHolding::emergencyThreshold", 10);
    SetPropertyStr("LaneHolding::emergencyThreshold" NSSUBPROP_DESCRIPTION, "Counter for ticks until emergency search");
    SetPropertyBool("LaneHolding::emergencyThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("WarpPerspective::warpLeft", 300);
    SetPropertyStr("WarpPerspective::warpLeft" NSSUBPROP_DESCRIPTION, "X value for input[0]");
    SetPropertyBool("WarpPerspective::warpLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("WarpPerspective::warpRight", 980);
    SetPropertyStr("WarpPerspective::warpRight" NSSUBPROP_DESCRIPTION, "X value for input[1]");
    SetPropertyBool("WarpPerspective::warpRight" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("LaneHolding::FovCurveRight", 50);
    SetPropertyStr("LaneHolding::FovCurveRight" NSSUBPROP_DESCRIPTION, "Field of View distance to car in right curves");
    SetPropertyBool("LaneHolding::FovCurveRight" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("LaneHolding::FovCurveLeft", 50);
    SetPropertyStr("LaneHolding::FovCurveLeft" NSSUBPROP_DESCRIPTION, "Field of View distance to car in left curves");
    SetPropertyBool("LaneHolding::FovCurveLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::Kp", 0);
    SetPropertyStr("PID::Kp" NSSUBPROP_DESCRIPTION, "P-Wert des PID-Reglers");
    SetPropertyBool("PID::Kp" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::Ki", 0);
    SetPropertyStr("PID::Ki" NSSUBPROP_DESCRIPTION, "I-Wert des PID-Reglers");
    SetPropertyBool("PID::Ki" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::Kd", 0);
    SetPropertyStr("PID::Kd" NSSUBPROP_DESCRIPTION, "D-Wert des PID-Reglers");
    SetPropertyBool("PID::Kd" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::K1", 0);
    SetPropertyStr("PID::K1" NSSUBPROP_DESCRIPTION, "K1-Wert des Reglers");
    SetPropertyBool("PID::K1" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::K2", 0);
    SetPropertyStr("PID::K2" NSSUBPROP_DESCRIPTION, "K2-Wert des Reglers");
    SetPropertyBool("PID::K2" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::K3", 0);
    SetPropertyStr("PID::K3" NSSUBPROP_DESCRIPTION, "K3-Wert des Reglers");
    SetPropertyBool("PID::K3" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("PID::K4", 0);
    SetPropertyStr("PID::K4" NSSUBPROP_DESCRIPTION, "K4-Wert des Reglers");
    SetPropertyBool("PID::K4" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("StopLineDetection::stoplineDistance", 10);
    SetPropertyStr("StopLineDetection::stoplineDistance" NSSUBPROP_DESCRIPTION, "Distance zwischen den zwei kanten der Haltelinie");
    SetPropertyBool("StopLineDetection::stoplineDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("StopLineDetection::distanceOffset", 1);
    SetPropertyStr("StopLineDetection::distanceOffset" NSSUBPROP_DESCRIPTION, "Varianz den der Abstand der Haltelinienkanten haben darf");
    SetPropertyBool("StopLineDetection::distanceOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("StopLineDetection::lineLength", 25);
    SetPropertyStr("StopLineDetection::lineLength" NSSUBPROP_DESCRIPTION, "Länge der Stoplinie");
    SetPropertyBool("StopLineDetection::lineLength" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("StopLineDetection::lengthOffset", 5);
    SetPropertyStr("StopLineDetection::lengthOffset" NSSUBPROP_DESCRIPTION, "Offset der LinienLänge");
    SetPropertyBool("StopLineDetection::lengthOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("CrossDetection::fourCrossLowerLeftX", 5);
    SetPropertyStr("CrossDetection::fourCrossLowerLeftX" NSSUBPROP_DESCRIPTION, "ROI Koordinaten für vierer Kreuzung");
    SetPropertyBool("CrossDetection::fourCrossLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("CrossDetection::fourCrossLowerLeftY", 5);
    SetPropertyStr("CrossDetection::fourCrossLowerLeftY" NSSUBPROP_DESCRIPTION, "ROI Koordinaten für vierer Kreuzung");
    SetPropertyBool("CrossDetection::fourCrossLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("CrossDetection::fourCrossUpperRightX", 5);
    SetPropertyStr("CrossDetection::fourCrossUpperRightX" NSSUBPROP_DESCRIPTION, "ROI Koordinaten für vierer Kreuzung");
    SetPropertyBool("CrossDetection::fourCrossUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("CrossDetection::fourCrossUpperRightY", 5);
    SetPropertyStr("CrossDetection::fourCrossUpperRightY" NSSUBPROP_DESCRIPTION, "ROI Koordinaten für vierer Kreuzung");
    SetPropertyBool("CrossDetection::fourCrossUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("Maneuver::VerticalTicks", 20);
    SetPropertyStr("Maneuver::VerticalTicks" NSSUBPROP_DESCRIPTION, "Ticks bis parallel");
    SetPropertyBool("Maneuver::VerticalTicks" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("Maneuver::HorizontalTicks", 130);
    SetPropertyStr("Maneuver::HorizontalTicks" NSSUBPROP_DESCRIPTION, "Ticks bis parallel");
    SetPropertyBool("Maneuver::HorizontalTicks" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("Maneuver::PixelToTicks", 1);
    SetPropertyStr("Maneuver::PixelToTicks" NSSUBPROP_DESCRIPTION, "Unrechnungsfaktor Pixel zu Ticks");
    SetPropertyBool("Maneuver::PixelToTicks" NSSUBPROP_ISCHANGEABLE, tTrue);

    m_szIdOutputSteeringAngleSet = tFalse;
    m_szIdOutputManeuverFinishedSet = tFalse;
    m_szIdInputSpeedSet = tFalse;
    m_bIDManeuverSendSet = tFalse;
    m_szIdOutputSpeedSet = tFalse;
}

tResult cLaneDetection::PropertyChanged(const tChar* strName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    //associate the properties to the member
    if (cString::IsEqual(strName, "Algorithm::Image Binarization Threshold"))
        m_filterProperties.thresholdImageBinarization = GetPropertyInt("Algorithm::Image Binarization Threshold");
    else if (cString::IsEqual(strName, "Algorithm::TestBool"))
        m_filterProperties.testBool = GetPropertyBool("Algorithm::TestBool");

    else if (cString::IsEqual(strName, "HoughLines::rho"))
        m_filterProperties.HLrho = GetPropertyFloat("HoughLines::rho");
    else if (cString::IsEqual(strName, "HoughLines::theta"))
        m_filterProperties.HLtheta = GetPropertyFloat("HoughLines::theta");
    else if (cString::IsEqual(strName, "HoughLines::threshold"))
        m_filterProperties.HLthreshold = GetPropertyInt("HoughLines::threshold");
    else if (cString::IsEqual(strName, "HoughLines::minLineLenght"))
        m_filterProperties.HLminLineLenght = GetPropertyFloat("HoughLines::minLineLenght");
    else if (cString::IsEqual(strName, "HoughLines::maxLineGap"))
        m_filterProperties.HLmaxLineGap = GetPropertyFloat("HoughLines::maxLineGap");

    else if (cString::IsEqual(strName, "StreetDetection::LeftLowerLeftX"))
        m_filterProperties.LeftLowerLeftX = GetPropertyFloat("StreetDetection::LeftLowerLeftX");
    else if (cString::IsEqual(strName, "StreetDetection::LeftLowerLeftY"))
        m_filterProperties.LeftLowerLeftY = GetPropertyFloat("StreetDetection::LeftLowerLeftY");
    else if (cString::IsEqual(strName, "StreetDetection::LeftLowerRightX"))
        m_filterProperties.LeftLowerRightX = GetPropertyFloat("StreetDetection::LeftLowerRightX");
    else if (cString::IsEqual(strName, "StreetDetection::LeftLowerRightY"))
        m_filterProperties.LeftLowerRightY = GetPropertyFloat("StreetDetection::LeftLowerRightY");
    else if (cString::IsEqual(strName, "StreetDetection::LeftUpperLeftX"))
        m_filterProperties.LeftUpperLeftX = GetPropertyFloat("StreetDetection::LeftUpperLeftX");
    else if (cString::IsEqual(strName, "StreetDetection::LeftUpperLeftY"))
        m_filterProperties.LeftUpperLeftY = GetPropertyFloat("StreetDetection::LeftUpperLeftY");
    else if (cString::IsEqual(strName, "StreetDetection::LeftUpperRightX"))
        m_filterProperties.LeftUpperRightX = GetPropertyFloat("StreetDetection::LeftUpperRightX");
    else if (cString::IsEqual(strName, "StreetDetection::LeftUpperRightY"))
        m_filterProperties.LeftUpperRightY = GetPropertyFloat("StreetDetection::LeftUpperRightY");
    else if (cString::IsEqual(strName, "StreetDetection::MidLowerLeftX"))
        m_filterProperties.MidLowerLeftX = GetPropertyFloat("StreetDetection::MidLowerLeftX");
    else if (cString::IsEqual(strName, "StreetDetection::MidLowerLeftY"))
        m_filterProperties.MidLowerLeftY = GetPropertyFloat("StreetDetection::MidLowerLeftY");
    else if (cString::IsEqual(strName, "StreetDetection::MidLowerRightX"))
        m_filterProperties.MidLowerRightX = GetPropertyFloat("StreetDetection::MidLowerRightX");
    else if (cString::IsEqual(strName, "StreetDetection::MidLowerRightY"))
        m_filterProperties.MidLowerRightY = GetPropertyFloat("StreetDetection::MidLowerRightY");
    else if (cString::IsEqual(strName, "StreetDetection::MidUpperLeftX"))
        m_filterProperties.MidUpperLeftX = GetPropertyFloat("StreetDetection::MidUpperLeftX");
    else if (cString::IsEqual(strName, "StreetDetection::MidUpperLeftY"))
        m_filterProperties.MidUpperLeftY = GetPropertyFloat("StreetDetection::MidUpperLeftY");
    else if (cString::IsEqual(strName, "StreetDetection::MidUpperRightX"))
        m_filterProperties.MidUpperRightX = GetPropertyFloat("StreetDetection::MidUpperRightX");
    else if (cString::IsEqual(strName, "StreetDetection::MidUpperRightY"))
        m_filterProperties.MidUpperRightY = GetPropertyFloat("StreetDetection::MidUpperRightY");
    else if (cString::IsEqual(strName, "StreetDetection::RightLowerLeftX"))
        m_filterProperties.RightLowerLeftX = GetPropertyFloat("StreetDetection::RightLowerLeftX");
    else if (cString::IsEqual(strName, "StreetDetection::RightLowerLeftY"))
        m_filterProperties.RightLowerLeftY = GetPropertyFloat("StreetDetection::RightLowerLeftY");
    else if (cString::IsEqual(strName, "StreetDetection::RightLowerRightX"))
        m_filterProperties.RightLowerRightX = GetPropertyFloat("StreetDetection::RightLowerRightX");
    else if (cString::IsEqual(strName, "StreetDetection::RightLowerRightY"))
        m_filterProperties.RightLowerRightY = GetPropertyFloat("StreetDetection::RightLowerRightY");
    else if (cString::IsEqual(strName, "StreetDetection::RightUpperLeftX"))
        m_filterProperties.RightUpperLeftX = GetPropertyFloat("StreetDetection::RightUpperLeftX");
    else if (cString::IsEqual(strName, "StreetDetection::RightUpperLeftY"))
        m_filterProperties.RightUpperLeftY = GetPropertyFloat("StreetDetection::RightUpperLeftY");
    else if (cString::IsEqual(strName, "StreetDetection::RightUpperRightX"))
        m_filterProperties.RightUpperRightX = GetPropertyFloat("StreetDetection::RightUpperRightX");
    else if (cString::IsEqual(strName, "StreetDetection::RightUpperRightY"))
        m_filterProperties.RightUpperRightY = GetPropertyFloat("StreetDetection::RightUpperRightY");
    else if (cString::IsEqual(strName, "StreetDetection::minAngle"))
        m_filterProperties.minAngle = GetPropertyFloat("StreetDetection::minAngle");
    else if (cString::IsEqual(strName, "StreetDetection::maxAngle"))
        m_filterProperties.maxAngle = GetPropertyFloat("StreetDetection::maxAngle");
    else if (cString::IsEqual(strName, "StreetDetection::CurvatureOffsetRight"))
        m_filterProperties.curvatureOffsetRight = GetPropertyInt("StreetDetection::CurvatureOffsetRight");
    else if (cString::IsEqual(strName, "StreetDetection::CurvatureOffsetLeft"))
        m_filterProperties.curvatureOffsetLeft = GetPropertyInt("StreetDetection::CurvatureOffsetLeft");
    else if (cString::IsEqual(strName, "StreetDetection::CurvatureOffsetMid"))
        m_filterProperties.curvatureOffsetMid = GetPropertyInt("StreetDetection::CurvatureOffsetMid");

    else if (cString::IsEqual(strName, "LaneHolding::FaultyLaneThreshold"))
        m_filterProperties.faultyCounterThreshold = GetPropertyInt("LaneHolding::FaultyLaneThreshold");
    else if (cString::IsEqual(strName, "LaneHolding::emergencyThreshold"))
        m_filterProperties.emergencyThreshold = GetPropertyInt("LaneHolding::emergencyThreshold");
    else if (cString::IsEqual(strName, "LaneHolding::FovCurveRight"))
        m_filterProperties.FovCurveRight = GetPropertyFloat("LaneHolding::FovCurveRight");
    else if (cString::IsEqual(strName, "LaneHolding::FovCurveLeft"))
        m_filterProperties.FovCurveLeft = GetPropertyFloat("LaneHolding::FovCurveLeft");

    else if (cString::IsEqual(strName, "WarpPerspective::warpLeft"))
        m_filterProperties.warpLeft = GetPropertyInt("WarpPerspective::warpLeft");
    else if (cString::IsEqual(strName, "WarpPerspective::warpRight"))
        m_filterProperties.warpRight = GetPropertyInt("WarpPerspective::warpRight");

    else if (cString::IsEqual(strName, "PID::Kp"))
        m_filterProperties.Kp = GetPropertyFloat("PID::Kp");
    else if (cString::IsEqual(strName, "PID::Ki"))
        m_filterProperties.Ki = GetPropertyFloat("PID::Ki");
    else if (cString::IsEqual(strName, "PID::Kd"))
        m_filterProperties.Kd = GetPropertyFloat("PID::Kd");
    else if (cString::IsEqual(strName, "PID::K1"))
        m_filterProperties.K1 = GetPropertyFloat("PID::K1");
    else if (cString::IsEqual(strName, "PID::K2"))
        m_filterProperties.K2 = GetPropertyFloat("PID::K2");
    else if (cString::IsEqual(strName, "PID::K3"))
        m_filterProperties.K3 = GetPropertyFloat("PID::K3");
    else if (cString::IsEqual(strName, "PID::K4"))
        m_filterProperties.K4 = GetPropertyFloat("PID::K4");

    else if (cString::IsEqual(strName, "StopLineDetection::stoplineDistance"))
        m_filterProperties.stopLineDistance = GetPropertyFloat("StopLineDetection::stoplineDistance");
    else if (cString::IsEqual(strName, "StopLineDetection::distanceOffset"))
        m_filterProperties.distanceOffset = GetPropertyFloat("StopLineDetection::distanceOffset");
    else if (cString::IsEqual(strName, "StopLineDetection::lineLength"))
        m_filterProperties.stopLineLength = GetPropertyFloat("StopLineDetection::lineLength");
    else if (cString::IsEqual(strName, "StopLineDetection::lengthOffset"))
        m_filterProperties.lengthOffset = GetPropertyFloat("StopLineDetection::lengthOffset");

    else if (cString::IsEqual(strName, "CrossDetection::fourCrossLowerLeftX"))
        m_filterProperties.fourCrossLowerLeftX = GetPropertyInt("CrossDetection::fourCrossLowerLeftX");
    else if (cString::IsEqual(strName, "CrossDetection::fourCrossLowerLeftY"))
        m_filterProperties.fourCrossLowerLeftY = GetPropertyInt("CrossDetection::fourCrossLowerLeftY");
    else if (cString::IsEqual(strName, "CrossDetection::fourCrossUpperRightX"))
        m_filterProperties.fourCrossUpperRightX = GetPropertyInt("CrossDetection::fourCrossUpperRightX");
    else if (cString::IsEqual(strName, "CrossDetection::fourCrossUpperRightY"))
        m_filterProperties.fourCrossUpperRightY = GetPropertyInt("CrossDetection::fourCrossUpperRightY");

    else if (cString::IsEqual(strName, "Maneuver::VerticalTicks"))
        m_filterProperties.verticalTicks = GetPropertyInt("Maneuver::VerticalTicks");
    else if (cString::IsEqual(strName, "Maneuver::HorizontalTicks"))
        m_filterProperties.horizontalTicks = GetPropertyInt("Maneuver::HorizontalTicks");
    else if (cString::IsEqual(strName, "Maneuver::PixelToTicks"))
        m_filterProperties.pixelToTicks = GetPropertyInt("Maneuver::PixelToTicks");

    RETURN_NOERROR;
}

cLaneDetection::~cLaneDetection()
{
}

tResult cLaneDetection::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cLaneDetection::Stop(__exception)
{
    //destroyWindow("Debug");
    return cFilter::Stop(__exception_ptr);
}
tResult cLaneDetection::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        //Create Wheeldata datatype
        tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
        RETURN_IF_POINTER_NULL(strDescWheelData);
        cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //Create Maneuver datatype
        tChar const * strDescManeuverData = pDescManager->GetMediaDescription("tManeuverValues");
        RETURN_IF_POINTER_NULL(strDescManeuverData);
        cObjectPtr<IMediaType> pTypeManeuverData = new cMediaType(0, 0, 0, "tManeuverValues", strDescManeuverData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //init mediatype for US struct pins
        tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
        RETURN_IF_POINTER_NULL(strDescUsStruct);
        cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelLeftData));
        RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelRightData));
        RETURN_IF_FAILED(pTypeManeuverData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuverData));

        // Video Input
        m_oGCLOutputPin.Create("GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this));
        RegisterPin(&m_oGCLOutputPin);

        //Create Wheeldata Pins
        RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

        RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

        //Create Maneuver Pins
        RETURN_IF_FAILED(m_oInputManeuver.Create("ManeuverInput", pTypeManeuverData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputManeuver));

        //Create UltraSonic
        RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));

        //init mediatype for signal values
        tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");

        RETURN_IF_POINTER_NULL(strDescSteeringAngle);
        cObjectPtr<IMediaType> pTypeSteeringangle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        //get mediatype description interfaces
        RETURN_IF_FAILED(pTypeSteeringangle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSteeringAngle));
        RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));

        // steering angle outputint
        m_oSteeringAngleOutputPin.Create("Steeringangle", pTypeSteeringangle, static_cast<IPinEventSink*>(this));
        RegisterPin(&m_oSteeringAngleOutputPin);


        //Maneuver finish output
        tChar const * strDescManeuverFinished = pDescManager->GetMediaDescription("tManeuverFinished");
        RETURN_IF_POINTER_NULL(strDescManeuverFinished);
        cObjectPtr<IMediaType> pTypeManeuverFinished = new cMediaType(0, 0, 0, "tManeuverFinished", strDescManeuverFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeManeuverFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuverFinished));
        m_oManeuverFinishedOutputPin.Create("ManeuverFinished_out", pTypeManeuverFinished, static_cast<IPinEventSink*>(this));
        RegisterPin(&m_oManeuverFinishedOutputPin);

        //Speed Input
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

        RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedcontrollerIn", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        RETURN_IF_FAILED(m_oOutputSpeedController.Create("SpeedcontrollerOut", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

        // triggering the lights
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
        //        RETURN_IF_FAILED(m_oOutputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        //        RETURN_IF_FAILED(RegisterPin(&m_oOutputReverseLight));
        RETURN_IF_FAILED(m_oOutputTurnLeft.Create("turnSignalLeftEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft));
        RETURN_IF_FAILED(m_oOutputTurnRight.Create("turnSignalRightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight));
        //        RETURN_IF_FAILED(m_oOutputHazzardLight.Create("hazzardLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        //        RETURN_IF_FAILED(RegisterPin(&m_oOutputHazzardLight));

        // output ticks to line
        tChar const * strTicksToLine = pDescManager->GetMediaDescription("tTicksToLine");
        RETURN_IF_POINTER_NULL(strTicksToLine);
        cObjectPtr<IMediaType> pTypeTicksToLine = new cMediaType(0, 0, 0, "tTicksToLine", strTicksToLine, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oOutputTicksToLine.Create("Ticks_to_line", pTypeTicksToLine, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputTicksToLine));
        RETURN_IF_FAILED(pTypeTicksToLine->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionTicksToLine));

        //output parkingspaces
        tChar const * strParkingspaces = pDescManager->GetMediaDescription("tParkingSpaces");
        RETURN_IF_POINTER_NULL(strParkingspaces);
        cObjectPtr<IMediaType> pTypeParkingSpaces = new cMediaType(0, 0, 0, "tParkingSpaces", strParkingspaces, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oOutputParkingspaces.Create("Parkingspaces_Out", pTypeParkingSpaces, this));
        RETURN_IF_FAILED(RegisterPin(&m_oOutputParkingspaces));
        RETURN_IF_FAILED(pTypeParkingSpaces->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionParkingspaces));
    }
    else if (eStage == StageNormal)
    {
        //Counter of not detected lines
        faultyLineCounterRight = -1;
        faultyLineCounterMiddle = -1;
        faultyLineCounterLeft = -1;
        oldAngleRight = 0;
        oldAngleMiddle = 0;
        oldAngleLeft = 0;
        goThroughCounter = 0;
        rightXPos = 685;
        middleXPos = 572;//567
        leftXPos = 441;
        oldXPosRight = rightXPos;
        oldXPosMiddle = middleXPos;
        oldXPosLeft = leftXPos;
        walkThroughCounter_steeringAngle = 0;
        wheelCountLeft = 0;
        wheelCountRight = 0;
        wheelTickSave = 1000000;
        doManeuver = true;
        maneuverNotSet = true;
        deviationSum = 0;
        xPosDeviationSum = 0;
        oldxPosDeviation = 0;
        oldTime = 0;
        oldAngle = 0;
        firstGoThrough = true;
        lambda.create( 2, 4, CV_32FC1 );

        EMERGENCY_SEARCH = false;
        EME_SEARCH_NO_STREET_COUNTER = -1;
        last_right_line = Vec4i(0,0,0,0);
        last_middle_line = Vec4i(0,0,0,0);
        last_left_line = Vec4i(0,0,0,0);

        parkingSpaceBeginTicks = 0;
        parkingEndCounter = 0;
        m_szIdsUsStructSet          = tFalse;
        isItEmptyCounter = 0;
        lastPedestianCrossingDetected = 0;
        m_stManTicks = INT32_MAX;
        lastSpeedSend = NO_LD_SPEED;
        lastSteeringangle = 0;

        nextManeuverRight = false;
        nextManeuverLeft = false;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;

        lastTurnLeftOnSend = false;
        lastTurnRightOnSend = false;

        isOnLeftLane = false;
        overtakingTicks = 100000000000;

        parkingTestRunning = false;
    }
    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input videinto pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
        {
            //LOG_ERROR("Invalid Input Format for this filter");
        }
    }

    RETURN_NOERROR;
}



tResult cLaneDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}
tResult cLaneDetection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oVideoInputPin)
        {
            if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
            }

            //Do Maneuver or Follow Street
            if(m_manoeverCount>0){
                maneuver actualManeuver = maneuverList.at(m_manoeverCount-1);
                //                LOG_INFO(cString::Format("wheelTickSave: %i",wheelTickSave));
                //                LOG_INFO(cString::Format("maneuverticks: %i",wheelTickSave+actualManeuver.ticks));
                //                LOG_INFO(cString::Format("actual ticks: %i",(wheelCountLeft + wheelCountRight)/2));
                //                LOG_INFO(cString::Format("m_manoeverCount: %i",m_manoeverCount));
                //LOG_INFO(cString::Format("%i vs %i ", (wheelCountLeft + wheelCountRight)/2, wheelTickSave+actualManeuver.ticks));

                if(!actualManeuver.turnLeft && lastTurnLeftOnSend){
                    lastTurnLeftOnSend = false;
                    TransmitBoolValue(&m_oOutputTurnLeft, false, 0);
                    //LOG_INFO("LEFT TURN DISBALED");
                }
                else if(actualManeuver.turnLeft && !lastTurnLeftOnSend){
                    lastTurnLeftOnSend = true;
                    TransmitBoolValue(&m_oOutputTurnLeft, true, 0);
                    //LOG_INFO("LEFT TURN ENABLED");
                }

                if(!actualManeuver.turnRight && lastTurnRightOnSend){
                    lastTurnRightOnSend = false;
                    TransmitBoolValue(&m_oOutputTurnRight, false, 0);
                    //LOG_INFO("RIGHT TURN DISABLED");
                }
                else if(actualManeuver.turnRight && !lastTurnRightOnSend){
                    lastTurnRightOnSend = true;
                    TransmitBoolValue(&m_oOutputTurnRight, true, 0);
                    //LOG_INFO("RIGHT TURN ENABLED");
                }

                if(actualManeuver.speed != NO_LD_SPEED && lastSpeedSend != actualManeuver.speed)
                {
                    lastSpeedSend = actualManeuver.speed;
                    TransmitSpeed(actualManeuver.speed, 0);
                }

                if(((wheelCountLeft + wheelCountRight)/2) > wheelTickSave+actualManeuver.ticks){
                    m_manoeverCount--;
                    if(m_manoeverCount>0){
                        doManeuver = false;
                    }
                    else{
                        TransmitBoolValue(&m_oOutputTurnRight, false, 0);
                        TransmitBoolValue(&m_oOutputTurnLeft, false, 0);
                        lastTurnLeftOnSend = false;
                        lastTurnRightOnSend = false;
                        TransmitSpeed(NO_LD_SPEED, 0);
                        if(!isOnLeftLane)
                            transmitManeuverFinished(maneuverID);
                        lastSpeedSend = NO_LD_SPEED;
                    }
                    wheelTickSave = (wheelCountLeft + wheelCountRight)/2;
                }
                transmitSteeringAngle(actualManeuver.steeringAngle);
            }else{
                if(isOnLeftLane && m_actualDistances[6] >= 50 && (wheelCountLeft + wheelCountRight)/2 >= overtakingTicks){
                    SwitchToLaneRight();
                }
                ProcessVideo(pMediaSample);
            }
        }
        else if (pSource == &m_oInputWheelLeft)
        {
            RETURN_IF_FAILED(ProcessWheelSampleLeft(pMediaSample));
        }
        else if (pSource == &m_oInputWheelRight)
        {
            RETURN_IF_FAILED(ProcessWheelSampleRight(pMediaSample));
        }
        else if (pSource == &m_oInputSpeedController)
        {
            RETURN_IF_FAILED(GetSpeed(pMediaSample));
        }
        else if (pSource == &m_oInputManeuver)
        {
            processManeuvers(pMediaSample);
        }
        else if (pSource == &m_oInputUsStruct)
        {
            //if mediasample is from type ultrasonic detect the minimum ultrasonic value
            ProcessMinimumValueUs(pMediaSample);
        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_oVideoInputPin)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
        }
    }

    RETURN_NOERROR;
}

tResult cLaneDetection::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
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

tResult cLaneDetection::setManeuvers(vector<maneuver> man){
    wheelTickSave = (wheelCountLeft + wheelCountRight)/2;
    m_manoeverCount = man.size();
    maneuverList = man;
    RETURN_NOERROR;
}

tResult cLaneDetection::processManeuvers(IMediaSample* pMediaSample){

    static tBufferID szManeuverIDInput;
    static tBufferID szParkingSpaceIDInput;
    int temp;
    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionManeuverData, pMediaSample, pCoderInput);

        if (!m_bIDManeuverSendSet)
        {
            pCoderInput->GetID("i16ManeuverValue", szManeuverIDInput);
            pCoderInput->GetID("i8ParkingSpaceID", szParkingSpaceIDInput);
            m_bIDManeuverSendSet = tTrue;
        }
        pCoderInput->Get(szManeuverIDInput, (tVoid*)&temp);
        pCoderInput->Get(szParkingSpaceIDInput, (tVoid*)&parkingSpaceID);
    }

    switch(temp){
    case -1:
        nextManeuverLeft = false;
        nextManeuverRight = false;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;
        maneuverList.clear();
        m_manoeverCount = 0;
        lastSpeedSend = NO_LD_SPEED;
        turnLeftOn = false;
        turnRightOn = false;
        TransmitSpeed(NO_LD_SPEED, 0);
        break;

    case LEFT:
        nextManeuverLeft = true;
        nextManeuverRight = false;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;
        break;

    case RIGHT:
        nextManeuverLeft = false;
        nextManeuverRight = true;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;
        break;

    case STRAIGHT:
        nextManeuverLeft = false;
        nextManeuverRight = false;
        nextManeuverStraight = true;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;
        break;

    case PARKING_IN:
        nextManeuverLeft = false;
        nextManeuverRight = false;
        nextManeuverStraight = false;
        nextManeuverParking = true;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;
        break;

    case PARKING_SPACE_SEARCH:
        nextManeuverLeft = false;
        nextManeuverRight = false;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = true;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = false;
        break;

    case PARKING_OUT_LEFT:
        nextManeuverLeft = false;
        nextManeuverRight = false;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = true;
        nextManeuverParkingOutRight = false;
        break;

    case PARKING_OUT_RIGHT:
        nextManeuverLeft = false;
        nextManeuverRight = false;
        nextManeuverStraight = false;
        nextManeuverParking = false;
        nextManeuverParkingSpaceSearch = false;
        nextManeuverParkingOutLeft = false;
        nextManeuverParkingOutRight = true;
        break;

    case PEDESTRIAN_CROSSING:
        EMERGENCY_SEARCH = true;
        //LOG_INFO("ALARM pedestrian!");
        lastPedestianCrossingDetected = ((wheelCountRight + wheelCountLeft) / 2);
        break;

    case OVERTAKING_CHECK:
    {
        vector<maneuver> execs;

        maneuver e;
        e.ticks = 80;
        e.steeringAngle = 0; //oldAngleRight ;// 2;
        e.speed = 8;
        e.turnLeft = false;
        e.turnRight = false;
        execs.push_back(e);
        setManeuvers(execs);

        break;
    }

    case OVERTAKING_RUN:
    {
        SwitchToLaneLeft();
//        vector<maneuver> execs;

//        maneuver h;
//        h.ticks = 10;
//        h.steeringAngle = 0.0f;
//        h.speed = -8;
//        h.turnLeft = false;
//        h.turnRight = false;
//        execs.push_back(h);
//        setManeuvers(execs);

//        maneuver c;
//        c.ticks = 20;
//        c.steeringAngle = 0.0f;
//        c.speed = -12;
//        c.turnLeft = false;
//        c.turnRight = false;
//        execs.push_back(c);

//        maneuver b;
//        b.ticks = 50;
//        b.steeringAngle = -100.0f;
//        b.speed = -8;
//        b.turnLeft = false;
//        b.turnRight = true;
//        execs.push_back(b);

//        maneuver a;
//        a.ticks = 80;
//        a.steeringAngle = 80.0f;
//        a.speed = -12;
//        a.turnLeft = false;
//        a.turnRight = true;
//        execs.push_back(a);

//        maneuver g;
//        g.ticks = 90;
//        g.steeringAngle = 0.0f;
//        g.speed = -10;
//        g.turnLeft = true;
//        g.turnRight = false;
//        execs.push_back(g);

//        maneuver f;
//        f.ticks = 90;
//        f.steeringAngle = 80.0f;
//        f.speed = -12;
//        f.turnLeft = true;
//        f.turnRight = false;
//        execs.push_back(f);

//        maneuver e;
//        e.ticks = 120;
//        e.steeringAngle = -100.0f;
//        e.speed = -12;
//        e.turnLeft = true;
//        e.turnRight = false;
//        execs.push_back(e);


//        setManeuvers(execs);
        break;
    }

    case ResumeDefault:
    {
        vector<maneuver> execs;maneuver c;
        c.ticks = 10;
        c.steeringAngle = 0.0f;
        c.speed = -8;
        c.turnLeft = false;
        c.turnRight = false;
        execs.push_back(c);
        setManeuvers(execs);
        break;}
    }


    if(temp != PEDESTRIAN_CROSSING)
        maneuverID = temp;
    RETURN_NOERROR;
}

tResult cLaneDetection::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
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
    RETURN_NOERROR;
}



tResult cLaneDetection::ProcessWheelSampleRight(IMediaSample* pMediaSample)
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
    RETURN_NOERROR;
}

tResult cLaneDetection::ProcessMinimumValueUs(IMediaSample* pMediaSample)
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

tResult cLaneDetection::ProcessVideo(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    const tVoid* l_pSrcBuffer;
    cv::Mat outputImage;
    vector<Vec4i> lines;
    vector<Point> rightCrossPoints;
    vector<Point> leftCrossPoints;
    Vec4i nearestLineToCarRight = Vec4i (0,0,0,0);
    Vec4i nearestLineToCarMiddle = Vec4i (0,0,0,0);
    Vec4i nearestLineToCarLeft = Vec4i (0,0,0,0);
    Vec4i gapVectorVertical = Vec4i (0,0,0,0);
    Vec4i gapVectorHorizontal = Vec4i (0,0,0,0);
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {

        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
        {
            m_inputImage.data = (uchar*)(l_pSrcBuffer);
            detectLines(outputImage, lines);
        }
        }
        //detect all Lines in the image
        pSample->Unlock(l_pSrcBuffer);

    //check if parking maneuver
    checkParkingManeuver(lines, outputImage);
    //compute left, middle and right lane from the "lines"-vector of detectLines()
    detectLane(outputImage, lines, nearestLineToCarRight, nearestLineToCarMiddle, nearestLineToCarLeft);
    // detect crossPoints

    detectCrossPoints(outputImage,lines, rightCrossPoints, leftCrossPoints, gapVectorVertical, gapVectorHorizontal, nearestLineToCarRight);
    // check cross maneuver
    checkCrossManeuver(rightCrossPoints, leftCrossPoints, gapVectorVertical, gapVectorHorizontal);
    //forparking purposes
    //comput the steeringangle from the street lanes
     tFloat32 sAngle = 0;

    //Wir haben keine Straße
    if(EME_SEARCH_NO_STREET_COUNTER > 5){
        // Wenn wir zu weit links sind
        if(last_right_line[0] > m_filterProperties.RightLowerRightX || computeAngleFromVec4i(last_right_line) >= 10.0f){
            // fahren wir nach rechts
            sAngle = 100.0f;
        }
        // Wenn wir zu weit rechts sind
        else{
            // fahren wir nach links
            sAngle = -100.0f ;
        }
    }else{
        // Wir haben eine Straße alles ist super
        sAngle  = computeSteeringAngle(nearestLineToCarLeft, nearestLineToCarMiddle, nearestLineToCarRight, outputImage);
    }
    transmitSteeringAngle(sAngle);

    if (!outputImage.empty() && m_oVideoOutputPin.IsConnected() /*&& DEBUG*/)
    {
        UpdateOutputImageFormat(outputImage);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImage;
        newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

        outputImage.release();
    }

    //if (m_oGCLOutputPin.IsConnected()) transmitGCL(detectionLines, detectedLinePoints, detectedLines);


    RETURN_NOERROR;
}

tResult cLaneDetection::detectLines(Mat &outputImage, vector<Vec4i> &detectedLines){

    //    Mat outputImage_hsv;

    //    Mat lower_yellow(3, 1, m_inputImage.type());
    //    lower_yellow.at<uchar>(0,0) = 20;
    //    lower_yellow.at<uchar>(1,0) = 100;
    //    lower_yellow.at<uchar>(2,0) = 100;

    //    Mat upper_yellow(3, 1, m_inputImage.type());
    //    upper_yellow.at<uchar>(0,0) = 30;
    //    upper_yellow.at<uchar>(1,0) = 255;
    //    upper_yellow.at<uchar>(2,0) = 255;

    //    Mat mask_yellow, mask_white, mask_yw, mask_roi;
    //    mask_yellow.create(m_inputImage.size(), m_inputImage.type());
    //    mask_white.create(m_inputImage.size(), m_inputImage.type());
    //    mask_yw.create(m_inputImage.size(), m_inputImage.type());
    cvtColor(m_inputImage, outputImage, CV_RGB2GRAY);

    static Mat transformation_x, transformation_y;
    if(firstGoThrough){
        Point2f inputQuad[4];
        Point2f outputQuad[4];
        lambda = Mat::zeros( outputImage.rows, outputImage.cols, outputImage.type());
        inputQuad[0] = Point2f( m_filterProperties.warpLeft, 500);
        inputQuad[1] = Point2f( m_filterProperties.warpRight,500);
        inputQuad[2] = Point2f( outputImage.cols,590);
        inputQuad[3] = Point2f( 0, 590);              //
        outputQuad[0] = Point2f( 0, 0 );
        outputQuad[1] = Point2f( outputImage.cols,0);
        outputQuad[2] = Point2f( outputImage.cols-280,outputImage.rows);
        outputQuad[3] = Point2f( 280, outputImage.rows);
        lambda = getPerspectiveTransform(inputQuad, outputQuad);
        perspective_to_maps(lambda,outputImage.size(),transformation_x,transformation_y);
        Mat inverseLambda;
        invert(lambda,inverseLambda);
        Mat map_x, map_y, srcTM;
        srcTM = inverseLambda.clone();
        map_x.create(outputImage.size(), CV_32FC1);
        map_y.create(outputImage.size(), CV_32FC1);
        double M11, M12, M13, M21, M22, M23, M31, M32, M33;
        M11 = srcTM.at<double>(0,0);
        M12 = srcTM.at<double>(0,1);
        M13 = srcTM.at<double>(0,2);
        M21 = srcTM.at<double>(1,0);
        M22 = srcTM.at<double>(1,1);
        M23 = srcTM.at<double>(1,2);
        M31 = srcTM.at<double>(2,0);
        M32 = srcTM.at<double>(2,1);
        M33 = srcTM.at<double>(2,2);
        for (int y = 0; y < outputImage.rows; y++) {
            double fy = (double)y;
            for (int x = 0; x < outputImage.cols; x++) {
                double fx = (double)x;
                double w = ((M31 * fx) + (M32 * fy) + M33);
                w = w != 0.0f ? 1.f / w : 0.0f;
                float new_x = (float)((M11 * fx) + (M12 * fy) + M13) * w;
                float new_y = (float)((M21 * fx) + (M22 * fy) + M23) * w;
                map_x.at<float>(y,x) = new_x;
                map_y.at<float>(y,x) = new_y;
            }
        }
        transformation_x.create(outputImage.size(), CV_16SC2);
        transformation_y.create(outputImage.size(), CV_16UC1);
        cv::convertMaps(map_x, map_y, transformation_x, transformation_y, false);
        firstGoThrough = false;
    }

    //inputImage = Mat(m_sInputFormat.nHeight, m_sInputFormat.nWidth, CV_8UC3, (tVoid*)l_pSrcBuffer, m_sInputFormat.nBytesPerLine);

    //warpPerspective(outputImage,outputImage,lambda,outputImage.size());
    remap(outputImage, outputImage, transformation_x, transformation_y, CV_INTER_LINEAR);
    //resize(outputImage, outputImage, Size2d(outputImage.cols/2, outputImage.rows/2));

    //cvtColor(outputImage, outputImage_hsv, CV_RGB2HSV);
    //inRange(outputImage_hsv, lower_yellow, upper_yellow, mask_yellow); //TODO yellow reparieren
    //inRange(outputImage, 230, 255, mask_white);
    //bitwise_or(mask_yellow, mask_white, mask_yw);
    //bitwise_and(outputImage, mask_white, outputImage);

    if(EMERGENCY_SEARCH){
        //GaussianBlur(outputImage, outputImage, Size(3, 3), 0, 0);
        Canny(outputImage, outputImage, m_filterProperties.thresholdImageBinarization, m_filterProperties.thresholdImageBinarization*2, 3);
        HoughLinesP(outputImage, detectedLines, m_filterProperties.HLrho, m_filterProperties.HLtheta, m_filterProperties.HLthreshold, m_filterProperties.HLminLineLenght, m_filterProperties.HLmaxLineGap);
    }else{
        Rect houghLinesRect(Point(m_filterProperties.fourCrossUpperRightX, m_filterProperties.fourCrossUpperRightY),(Point(m_filterProperties.fourCrossLowerLeftX, m_filterProperties.fourCrossLowerLeftY)));
        Mat out = outputImage(houghLinesRect);

        //GaussianBlur(out, out, Size(3, 3), 0, 0);

        //        if(m_filterProperties.testBool)
        //            threshold(out,out,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU);

        Canny(out, out, m_filterProperties.thresholdImageBinarization, m_filterProperties.thresholdImageBinarization*2, 3);
        HoughLinesP(out, detectedLines, m_filterProperties.HLrho, m_filterProperties.HLtheta, m_filterProperties.HLthreshold, m_filterProperties.HLminLineLenght, m_filterProperties.HLmaxLineGap);

        for(unsigned int i=0; i<detectedLines.size();i++){
            Vec4i tmp = detectedLines[i];
            tmp[0] += m_filterProperties.fourCrossLowerLeftX;
            tmp[1] += m_filterProperties.fourCrossUpperRightY;
            tmp[2] += m_filterProperties.fourCrossLowerLeftX;
            tmp[3] += m_filterProperties.fourCrossUpperRightY;

            detectedLines[i] = tmp;
        }
    }

    setStartEndPoint(detectedLines);


    RETURN_NOERROR;
}

tResult cLaneDetection::perspective_to_maps(const cv::Mat &perspective_mat, const cv::Size &img_size,
                                            cv::Mat &map1, cv::Mat &map2)
{
    // invert the matrix because the transformation maps must be
    // bird's view -> original
    cv::Mat inv_perspective(perspective_mat.inv());
    inv_perspective.convertTo(inv_perspective, CV_32FC1);

    // create XY 2D array
    // (((0, 0), (1, 0), (2, 0), ...),
    //  ((0, 1), (1, 1), (2, 1), ...),
    // ...)
    cv::Mat xy(img_size, CV_32FC2);
    float *pxy = (float*)xy.data;
    for (int y = 0; y < img_size.height; y++)
        for (int x = 0; x < img_size.width; x++)
        {
            *pxy++ = x;
            *pxy++ = y;
        }

    // perspective transformation of the points
    cv::Mat xy_transformed;
    cv::perspectiveTransform(xy, xy_transformed, inv_perspective);

    // split x/y to extra maps
    assert(xy_transformed.channels() == 2);
    cv::Mat maps[2]; // map_x, map_y
    cv::split(xy_transformed, maps);

    // remap() with integer maps is faster
    cv::convertMaps(maps[0], maps[1], map1, map2, CV_16SC2);
    RETURN_NOERROR;
}

tResult cLaneDetection::detectStopLines(vector<Vec4i> &lines, vector<stopLine>&stopLinesVertical, vector<stopLine>&stopLinesHorizontal, Mat &outputImage){
    int lineAngleOffset = 4;
    int maxOffset = 25;

    for(unsigned int i = 0; i < lines.size(); i++){
        int lineAngle = computeAngleFromVec4i(lines[i]);
        for(unsigned int j = 0; j < lines.size(); j++){
            if((lineAngle-lineAngleOffset < computeAngleFromVec4i(lines[j])) && (computeAngleFromVec4i(lines[j]) < lineAngle+lineAngleOffset)){
                int startPointDistance = sqrt(pow(lines[i][0] - lines[j][0],2) + pow(lines[i][1] - lines[j][1],2));
                int endPointDistance = sqrt(pow(lines[i][2] - lines[j][2],2) + pow(lines[i][3] - lines[j][3],2));
                int startEndDistance = sqrt(pow(lines[i][0] - lines[j][2],2) + pow(lines[i][1] - lines[j][3],2));
                int endStartDistance = sqrt(pow(lines[i][2] - lines[j][0],2) + pow(lines[i][3] - lines[j][1],2));

                Vec3i firstLineMiddlePoint = Vec3i((lines[i][0]+lines[i][2])/2, (lines[i][1]+lines[i][3])/2, 0);
                Vec3i secondLineUpVector = Vec3i(lines[j][0], lines[j][1], 0);
                Vec3i secondLineSteigungsVector = Vec3i((lines[j][2]-lines[j][0]), (lines[j][3]-lines[j][1]), 0);
                Vec3i tempDif = firstLineMiddlePoint-secondLineUpVector;
                Vec3i crossProduct = Vec3i(tempDif[1]*secondLineSteigungsVector[2]-tempDif[2]*secondLineSteigungsVector[1],
                        tempDif[2]*secondLineSteigungsVector[0]-tempDif[0]*secondLineSteigungsVector[2],
                        tempDif[0]*secondLineSteigungsVector[1]-tempDif[1]*secondLineSteigungsVector[0]);

                float distance = abs(sqrt(pow(crossProduct[0],2)+pow(crossProduct[1],2)+pow(crossProduct[2],2)))/sqrt(pow(secondLineSteigungsVector[0],2)+pow(secondLineSteigungsVector[1],2));

                if(((m_filterProperties.stopLineDistance-m_filterProperties.distanceOffset) < distance)
                        && ((m_filterProperties.stopLineDistance+m_filterProperties.distanceOffset) > distance)
                        && (startPointDistance<maxOffset || endPointDistance<maxOffset || startEndDistance<maxOffset || endStartDistance<maxOffset)){
                    int firstLineLength = sqrt(pow(lines[i][0] - lines[i][2],2) + pow(lines[i][1] - lines[i][3],2));
                    int secondLineLength = sqrt(pow(lines[j][0] - lines[j][2],2) + pow(lines[j][1] - lines[j][3],2));


                    if((m_filterProperties.stopLineLength-m_filterProperties.lengthOffset <= firstLineLength) && (m_filterProperties.stopLineLength+m_filterProperties.lengthOffset >= firstLineLength)
                            && (m_filterProperties.stopLineLength-m_filterProperties.lengthOffset <= secondLineLength) && (m_filterProperties.stopLineLength+m_filterProperties.lengthOffset >= secondLineLength)){
                        stopLine foundStopline;
                        if(abs(lines[i][0]-lines[i][2]) < abs(lines[i][1]-lines[i][3])){
                            //vertikal
                            if(oldAngle < (lineAngle + lineAngleOffset) && oldAngle > (lineAngle - lineAngleOffset)){
                                if(lines[i][0] > lines[j][0]){
                                    foundStopline.firstLine = lines[j];
                                    foundStopline.secondLine = lines[i];
                                    stopLinesVertical.push_back(foundStopline);
                                }else{
                                    foundStopline.firstLine = lines[i];
                                    foundStopline.secondLine = lines[j];
                                    stopLinesVertical.push_back(foundStopline);
                                }
                            }
                        }else{
                            //horizontal
                            if(lines[i][1] > lines[j][1]){
                                foundStopline.firstLine = lines[i];
                                foundStopline.secondLine = lines[j];
                                stopLinesHorizontal.push_back(foundStopline);
                            }else{
                                foundStopline.firstLine = lines[j];
                                foundStopline.secondLine = lines[i];
                                stopLinesHorizontal.push_back(foundStopline);
                            }
                        }
                        if(DEBUG){
                            cv::line(outputImage, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), 255, 3, CV_AA);
                            cv::line(outputImage, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), 255, 3, CV_AA);
                        }

                        //delete stoplines from lines vector
                        //                        lines.erase(lines.begin()+i);
                        //                        lines.erase(lines.begin()+j-1);
                        //                        i--;
                        break;
                    }
                }
            }
        }
    }
    RETURN_NOERROR;
}

tResult cLaneDetection::parkingSpaceSearch(vector<Vec4i> &lines, vector<bool> &parkingSpaces, Mat& outputImage)
{
    int lineAngleOffset = 4;
    Point crossing;
    if(parkingEndCounter >= 5 && (((wheelCountLeft + wheelCountRight) /2 - parkingSpaceBeginTicks) > 90))
    {
        if(m_actualDistances[6] > 50 || m_actualDistances[6] == -1)
        {
            parkingSpaces.push_back(false);
            parkingEndCounter = 0;
        }
        else
        {
            parkingSpaces.push_back(true);
            parkingEndCounter = 0;
        }
        //LOG_INFO(cString::Format("%d Parkingspaces %s %s %s %s", parkingSpaces.size(), parkingSpaces[0] ? "true" : "false", parkingSpaces[1] ? "true" : "false",parkingSpaces[2] ? "true" : "false",parkingSpaces[3] ? "true" : "false"));
        TransmitParkingspaces(&m_oOutputParkingspaces, parkingSpaces);
        parkingSpaces.clear();
        nextManeuverParkingSpaceSearch = false;
        RETURN_NOERROR;
    }
    for(unsigned int i = 0; i < lines.size(); i++)
    {
        int lineAngle1 = computeAngleFromVec4i((lines[i]));
        for(unsigned int j = 0; j < lines.size(); j++)
        {
            int lineAngle2 = computeAngleFromVec4i((lines[j]));

            if(abs(lineAngle1) + abs(lineAngle2) + lineAngleOffset > 90 && 90 > abs(lineAngle1) + abs(lineAngle2) - lineAngleOffset)
            {
                int x,y;
                // Horizontal
                if(abs(lines[i][0] - lines[i][2]) > abs(lines[i][1] - lines[i][3])){
                    x = (lines[i][0] < lines[i][2]) ? lines[i][0] : lines[i][2];
                    y = lines[i][1];
                }else{
                    x = (lines[j][0] < lines[j][2]) ? lines[j][0] : lines[j][2];
                    y = lines[j][1];
                }
                if(x > m_filterProperties.RightUpperLeftX && x < m_filterProperties.RightUpperRightX && y > m_filterProperties.RightUpperLeftY)
                {
                    crossing = Point(x,y);
                    if((wheelCountLeft + wheelCountRight) /2 - parkingSpaceBeginTicks > 50)
                    {
                        parkingSpaceBeginTicks = (wheelCountLeft + wheelCountRight) /2;
                        parkingEndCounter++;
                        //check if space is empty
                        if(parkingEndCounter >= 3)
                        {
                            //LOG_INFO(cString::Format("ABS %f", m_actualDistances[6]));
                            if((m_actualDistances[6] > 50 || m_actualDistances[6] == -1))
                            {
                                parkingSpaces.push_back(false);
                            }
                            else{
                                parkingSpaces.push_back(true);
                            }

                        }
                    }
                }
            }
        }
    }
    cv::circle(outputImage, crossing, 10, 255, 5);
    RETURN_NOERROR;
}


tResult cLaneDetection::detectParkingSpot(vector<Vec4i> &lines, bool& successful, int& y_offset, Mat& outputImage)
{
    int lineAngleOffset = 4;
    Point crossing;
    // LOG_INFO(cString::Format("%i %i", parkingEndCounter, ((wheelCountLeft + wheelCountRight) /2 - parkingSpaceBeginTicks)));
    if(parkingEndCounter >= 5 && (((wheelCountLeft + wheelCountRight) /2 - parkingSpaceBeginTicks) > 90))
    {
        if(m_actualDistances[6] > 50 || m_actualDistances[6] == -1)
        {
            successful = true;
            parkingEndCounter = 0;
        }
        else
        {
            nextManeuverParking = false;
        }
    }
    for(unsigned int i = 0; i < lines.size(); i++)
    {
        int lineAngle1 = computeAngleFromVec4i((lines[i]));
        for(unsigned int j = 0; j < lines.size(); j++)
        {
            int lineAngle2 = computeAngleFromVec4i((lines[j]));

            if(abs(lineAngle1) + abs(lineAngle2) + lineAngleOffset > 90 && 90 > abs(lineAngle1) + abs(lineAngle2) - lineAngleOffset)
            {
                //LOG_INFO(cString::Format("Found %i %i %i %i vs %i %i %i %i Winkel %i %i", lines[i][0], lines[i][1], lines[i][2], lines[i][3], lines[j][0], lines[j][1], lines[j][2], lines[j][3], lineAngle1, lineAngle2));

                int x,y;
                // Horizontal
                if(abs(lines[i][0] - lines[i][2]) > abs(lines[i][1] - lines[i][3])){
                    x = (lines[i][0] < lines[i][2]) ? lines[i][0] : lines[i][2];
                    y = lines[i][1];
                }else{
                    x = (lines[j][0] < lines[j][2]) ? lines[j][0] : lines[j][2];
                    y = lines[j][1];
                }
                if(x > m_filterProperties.RightUpperLeftX && x < m_filterProperties.RightUpperRightX && y > m_filterProperties.RightUpperLeftY)
                {
                    crossing = Point(x,y);
                    y_offset = outputImage.rows - crossing.y;
                    if((wheelCountLeft + wheelCountRight) /2 - parkingSpaceBeginTicks > 50)
                    {
                        parkingSpaceBeginTicks = (wheelCountLeft + wheelCountRight) /2;
                        parkingEndCounter++;
                        //LOG_INFO(cString::Format("ABS %f", m_actualDistances[6]));
                        //check if space is empty
                        if(parkingEndCounter >= 3)
                        {
                            if(parkingEndCounter == (parkingSpaceID + 2))
                            {
                                if((m_actualDistances[6] > 50 || m_actualDistances[6] == -1))
                                {

                                    successful = true;
                                    parkingEndCounter = 0;
                                }
                                else{
                                    parkingSpaceID++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    cv::circle(outputImage, crossing, 10, 255, 5);
    //LOG_INFO(cString::Format("Found %i %i %i %i", nearestLine[0], nearestLine[1], nearestLine[2], nearestLine[3]));
    RETURN_NOERROR;
}

tResult cLaneDetection::checkParkingManeuver(vector<Vec4i> &lines, Mat& outputImage){

    if(nextManeuverParking){
        bool success = false;
        int y_offset;
        int i = 0;
        detectParkingSpot(lines, success, y_offset, outputImage);

        if(success){

            nextManeuverParking = false;

            //LOG_INFO("Starting Parking");
            i=0;
            vector<maneuver> execs;

            maneuver d;
            d.ticks = 80;
            d.steeringAngle = 0.0f;
            d.speed = 14;
            d.turnLeft = false;
            d.turnRight = false;
            execs.push_back(d);

            maneuver c;
            c.ticks = 110;
            c.steeringAngle = 100.0f;
            c.speed = 14;
            c.turnLeft = false;
            c.turnRight = true;
            execs.push_back(c);

            maneuver b;
            b.ticks = 90;
            b.steeringAngle = -100.0f;
            b.speed = -14;
            b.turnLeft = true;
            b.turnRight = false;
            execs.push_back(b);

            maneuver a;
            a.ticks = 5;//y_offset;
            a.steeringAngle = 0.0f;
            a.speed = -14;
            a.turnLeft = false;
            a.turnRight = false;
            execs.push_back(a);

            setManeuvers(execs);

        }
    }else if(nextManeuverParkingOutLeft){

        nextManeuverParkingOutLeft = false;

        // links ausparken
        vector<maneuver> execs;

        maneuver g;
        g.ticks = 15;
        g.steeringAngle = 0.0f;
        g.speed = -8;
        g.turnLeft = false;
        g.turnRight = false;
        execs.push_back(g);

        maneuver f;
        f.ticks = 250;
        f.steeringAngle = -100.0f;
        f.speed = -14;
        f.turnLeft = true;
        f.turnRight = false;
        execs.push_back(f);

        maneuver e;
        e.ticks = 50;
        e.steeringAngle = 0.0f;
        e.speed = -14;
        e.turnLeft = true;
        e.turnRight = false;
        execs.push_back(e);

        setManeuvers(execs);

    }else if(nextManeuverParkingOutRight){

        nextManeuverParkingOutRight = false;

        //LOG_INFO("AUSPARKEN!");
        // rechts ausparken
        vector<maneuver> execs;

        maneuver g;
        g.ticks = 10;
        g.steeringAngle = 0.0f;
        g.speed = -8;
        g.turnLeft = false;
        g.turnRight = false;
        execs.push_back(g);

        maneuver f;
        f.ticks = 170;
        f.steeringAngle = 100.0f;
        f.speed = -14;
        f.turnLeft = false;
        f.turnRight = true;
        execs.push_back(f);

        maneuver e;
        e.ticks = 50;
        e.steeringAngle = 0.0f;
        e.speed = -14;
        e.turnLeft = false;
        e.turnRight = true;
        execs.push_back(e);
        setManeuvers(execs);

    }else if(nextManeuverParkingSpaceSearch){
        //TODO
        static vector<bool> parkingspaces;
        parkingSpaceSearch(lines, parkingspaces, outputImage);
    }
    RETURN_NOERROR;
}


tResult cLaneDetection::detectCrossPoints(Mat& outputImage, vector<Vec4i> &lines, vector<Point> &rightCrossPoints, vector<Point> &leftCrossPoints, Vec4i &gapVectorVertical, Vec4i &gapVectorHorizontal, Vec4i &nearestLineToCarRight){

    if(nextManeuverStraight){
        //LOG_INFO("SEARCH FOR CROSSPOINTS");
        //muss man was machen?? grusel grusel
    }

    if(nextManeuverRight || nextManeuverLeft || nextManeuverStraight){
        //LOG_INFO("SEARCH FOR CROSSPOINTS");
        vector<Vec4i> potentialLowerLinesRight;
        Vec4i bestLowerLineRight;
        Vec4i extendedbestLowerLineRight;
        Vec2i bestLowerLineRightVector;
        int bestLowerLineRightVectorLength;
        int gapDistanceRight = m_filterProperties.stopLineDistance;
        int distanceOffsetRight = 25;


        ///right crosspoint detection (vertical gap)
        //linie im rechten unteren ROI finden -> potentialLowerLinesRight
        for(unsigned int i = 0; i < lines.size(); i++){

            int lineAngleRighti = computeAngleFromVec4i((lines[i]));
            int xVali = lines[i][0];
            int yVali = lines[i][1];
            int xValEndi = lines[i][2];
            int yValEndi = lines[i][3];

            if((xVali > m_filterProperties.RightUpperLeftX) && (xVali < m_filterProperties.RightUpperRightX) &&
                    (yVali < m_filterProperties.RightLowerLeftY) && (yVali > m_filterProperties.RightUpperRightY) &&
                    (xValEndi > m_filterProperties.RightUpperLeftX) && (xValEndi < m_filterProperties.RightUpperRightX) &&
                    (yValEndi < m_filterProperties.RightLowerLeftY) && (yValEndi > m_filterProperties.RightUpperRightY-20) &&
                    lineAngleRighti > -20 && lineAngleRighti < 20){
                potentialLowerLinesRight.push_back(lines[i]);
            }
        }
        //aus potentialLowerLinesRight die linie finden mit dem kleinsten yEnd wert (Damit die linie genommen wird die ihren endpunkt auf dem kreuzungspunkt hat)
        for(unsigned int i = 0; i < potentialLowerLinesRight.size(); i++){
            if(i==0){
                bestLowerLineRight = potentialLowerLinesRight[i];
            }else if(potentialLowerLinesRight[i][3] < bestLowerLineRight[3]){
                bestLowerLineRight = potentialLowerLinesRight[i];
            }
        }
        //gefundene linie um die gapdistance verlängern damit man anschließend im bereich des endpunktes der verlängerten linie nach dem zweiten kreuzungspunkt suchen kann
        bestLowerLineRightVector[0] = bestLowerLineRight[2] - bestLowerLineRight[0];
        bestLowerLineRightVector[1] = bestLowerLineRight[3] - bestLowerLineRight[1];
        bestLowerLineRightVectorLength = sqrt(pow(bestLowerLineRightVector[0],2) + pow(bestLowerLineRightVector[1],2));

        extendedbestLowerLineRight[0] = bestLowerLineRight[0];
        extendedbestLowerLineRight[1] = bestLowerLineRight[1];
        extendedbestLowerLineRight[2] = bestLowerLineRightVector[0] * ((bestLowerLineRightVectorLength+gapDistanceRight)/bestLowerLineRightVectorLength);
        extendedbestLowerLineRight[3] = bestLowerLineRightVector[1] * ((bestLowerLineRightVectorLength+gapDistanceRight)/bestLowerLineRightVectorLength);

        extendedbestLowerLineRight[2] = bestLowerLineRight[0] + extendedbestLowerLineRight[2];
        extendedbestLowerLineRight[3] = (extendedbestLowerLineRight[3] < 0) ? bestLowerLineRight[1] + extendedbestLowerLineRight[3] : bestLowerLineRight[1] - extendedbestLowerLineRight[3];

        //LOG_INFO("right vertical gap: First Line Detected");
        //cv::line(outputImage,Point(bestLowerLineRight[0], bestLowerLineRight[1]),Point(bestLowerLineRight[2],bestLowerLineRight[3]), 200, 3);
        cv::line(outputImage,Point(extendedbestLowerLineRight[0], extendedbestLowerLineRight[1]),Point(extendedbestLowerLineRight[2],extendedbestLowerLineRight[3]), 200, 3);


        //linie im bereich des endpunktes der verlängerten linie finden die einen ähnlichen winkel hat wie die erste linie und den richtiigen abstand zur ersten linie besitzt
        for(unsigned int j = 0; j < lines.size(); j++){
            int lineAngleRightj = computeAngleFromVec4i((lines[j]));
            int xValj = lines[j][0];
            int yValj = lines[j][1];

            if((xValj > extendedbestLowerLineRight[2] - 25) && (xValj < extendedbestLowerLineRight[2] + 6) &&
                    (yValj < extendedbestLowerLineRight[3] + 30) &&
                    lineAngleRightj > -20 && lineAngleRightj < 20){


                int lowerLineXEnd = bestLowerLineRight[2];
                int lowerLineYEnd = bestLowerLineRight[3];
                int upperLineXStart = lines[j][0];
                int upperLineYStart = lines[j][1];

                Vec4i Vec1 = Vec4i(lowerLineXEnd-15, lowerLineYEnd-10, upperLineXStart-15, upperLineYStart+10);
                Vec4i Vec2 = Vec4i(lowerLineXEnd+15, lowerLineYEnd-10, upperLineXStart+15, upperLineYStart);
                Vec4i Vec3 = Vec4i(lowerLineXEnd-15, lowerLineYEnd, lowerLineXEnd+15, lowerLineYEnd);
                Vec4i Vec4 = Vec4i(upperLineXStart-15, upperLineYStart, upperLineXStart+15, upperLineYStart);

//                LOG_INFO(cString::Format("Vec1 %i %i %i %i",Vec1[0], Vec1[1], Vec1[2], Vec1[3]));
//                LOG_INFO(cString ::Format("Vec2 %i %i %i %i",Vec2[0], Vec2[1], Vec2[2], Vec2[3]));
//                LOG_INFO(cString::Format("Vec3 %i %i %i %i",Vec3[0], Vec3[1], Vec3[2], Vec3[3]));
//                LOG_INFO(cString::Format("Vec4 %i %i %i %i",Vec4[0], Vec4[1], Vec4[2], Vec4[3]));

//                cv::line(outputImage, Point(Vec1[0], Vec1[1]), Point(Vec1[2], Vec1[3]) ,255,3);
//                cv::line(outputImage, Point(Vec2[0], Vec2[1]), Point(Vec2[2], Vec2[3]) ,255,3);
//                cv::line(outputImage, Point(Vec3[0], Vec3[1]), Point(Vec3[2], Vec3[3]) ,255,3);
//                cv::line(outputImage, Point(Vec4[0], Vec4[1]), Point(Vec4[2], Vec4[3]) ,255,3);

                static int counter = 0;
                int counterThresh = 2;
                for(unsigned int k=0; k<lines.size();k++){
                    if(Intersection(Vec1,lines[k]) || Intersection(Vec2,lines[k]) || Intersection(Vec3,lines[k]) || Intersection(Vec4,lines[k])){
                        counter++;

//                        cv::line(outputImage,Point(lines[k][0], lines[k][1]), Point(lines[k][2], lines[k][3]),255,2);
//                        vector<maneuver> mans;
//                        maneuver man;
//                        man.ticks = 100;
//                        man.speed = 0;
//                        mans.push_back(man);
//                        setManeuvers(mans);
                    }
                    if(counter > counterThresh)
                        break;
                }
                //LOG_INFO(cString::Format("counter %i",counter));
                if(counter > counterThresh){
                    //LOG_INFO("NOT REALLY A RIGHT VERTICAL GAP");
                    counter = 0;
                    continue;
                }
                counter = 0;

                //LOG_INFO("right vertical gap: Second Line Detected");
                cv::line(outputImage,Point(lines[j][0], lines[j][1]),Point(lines[j][2],lines[j][3]), 200, 3);
                int distance = sqrt(pow(lowerLineXEnd - upperLineXStart,2) + pow(lowerLineYEnd - upperLineYStart,2));
                if(distance > gapDistanceRight - distanceOffsetRight && distance < gapDistanceRight + distanceOffsetRight && upperLineYStart < lowerLineYEnd){
                    gapVectorVertical = Vec4i(lowerLineXEnd, lowerLineYEnd, upperLineXStart, upperLineYStart);
                    cv::circle(outputImage, Point(lowerLineXEnd,lowerLineYEnd), 10, 255, 5);
                    cv::circle(outputImage, Point(upperLineXStart,upperLineYStart), 10, 255, 5);
                    rightCrossPoints.push_back(Point(lowerLineXEnd,lowerLineYEnd));
                    TransmitTicksToCrosspoint(m_filterProperties.verticalTicks + (m_filterProperties.RightLowerRightY - lowerLineYEnd));
                    RETURN_NOERROR;
                }
            }
        }


        ///right crosspoint detection (horizontal gap)
        vector<Vec4i> potentialRightHorizontalLine;
        Vec4i bestRightHorizontalLine;
        int gapDistanceHorizontal = 220;
        int distanceOffsetHorizontal = 25;

        for(unsigned int i = 0; i < lines.size(); i++){
            int lineAngleRighti = computeAngleFromVec4i((lines[i]));
            //startpoint should be the left point
            int xVali = (lines[i][0] < lines[i][2]) ? lines[i][0] : lines[i][2];
            int yVali = (lines[i][0] < lines[i][2]) ? lines[i][1] : lines[i][3];
            int xValEndi = (lines[i][0] < lines[i][2]) ? lines[i][2] : lines[i][0];
            int yValEndi = (lines[i][0] < lines[i][2]) ? lines[i][3] : lines[i][1];

            if((xVali > nearestLineToCarRight[0] - 10) && (xVali < nearestLineToCarRight[0] + 25) &&
                    (yVali < m_filterProperties.RightLowerLeftY) && (yVali > m_filterProperties.RightUpperRightY - 20) ){
                if(lineAngleRighti > 85 && lineAngleRighti <= 90 || lineAngleRighti < -85 && lineAngleRighti > -90){
                    potentialRightHorizontalLine.push_back(Vec4i(xVali,yVali,xValEndi,yValEndi));
                    //LOG_INFO("right horizontal gap: First Line Detected");
                    cv::line(outputImage,Point(xVali, yVali),Point(xValEndi,yValEndi), 200, 3);
                }
            }
        }
        if(!potentialRightHorizontalLine.empty()){
            for(unsigned int i = 0; i < potentialRightHorizontalLine.size(); i++){
                if(i==0){
                    bestRightHorizontalLine = potentialRightHorizontalLine[i];
                }else if(potentialRightHorizontalLine[i][0] < bestRightHorizontalLine[0]){
                    bestRightHorizontalLine = potentialRightHorizontalLine[i];
                }
            }
            for(unsigned int j = 0; j < lines.size(); j++){
                int lineAngleRightj = computeAngleFromVec4i((lines[j]));
                //startpoint should be the right point
                int xValj = (lines[j][0] > lines[j][2]) ? lines[j][0] : lines[j][2];
                int yValj = (lines[j][0] > lines[j][2]) ? lines[j][1] : lines[j][3];
                int xValEndj = (lines[j][0] > lines[j][2]) ? lines[j][2] : lines[j][0];
                int yValEndj = (lines[j][0] > lines[j][2]) ? lines[j][3] : lines[j][1];

                if((xValj > m_filterProperties.LeftUpperLeftX) && (xValj < m_filterProperties.LeftUpperRightX) &&
                        (yValj < m_filterProperties.LeftLowerLeftY) && (yValj > m_filterProperties.LeftUpperRightY - 20) /*||
                                                                                                                            (xValEndj > m_filterProperties.LeftUpperLeftX) && (xValEndj < m_filterProperties.LeftUpperRightX) &&
                                                                                                                            (yValEndj < m_filterProperties.LeftLowerLeftY) && (yValEndj > m_filterProperties.LeftUpperRightY)*/){
                    if(lineAngleRightj > 80 && lineAngleRightj <= 90 || lineAngleRightj < -80 && lineAngleRightj > -90){
                        int rightLineStartX = bestRightHorizontalLine[0];
                        int rightLineStartY = bestRightHorizontalLine[1];
                        int leftLineStartX = xValj;
                        int leftLineStartY = yValj;

                        Vec4i Vec1 = Vec4i(leftLineStartX+15, leftLineStartY+2, rightLineStartX-15, rightLineStartY+2);
                        Vec4i Vec2 = Vec4i(leftLineStartX+15, leftLineStartY-10, rightLineStartX-15, rightLineStartY-10);
                        Vec4i Vec3 = Vec4i(leftLineStartX+15, leftLineStartY+2, leftLineStartX+15, leftLineStartY-10);
                        Vec4i Vec4 = Vec4i(rightLineStartX-15, rightLineStartY+2, rightLineStartX-15, rightLineStartY-10);

                        if(DEBUG)
                        {
                            cv::line(outputImage, Point(Vec1[0], Vec1[1]), Point(Vec1[2], Vec1[3]) ,100,3);
                            cv::line(outputImage, Point(Vec2[0], Vec2[1]), Point(Vec2[2], Vec2[3]) ,100,3);
                            cv::line(outputImage, Point(Vec3[0], Vec3[1]), Point(Vec3[2], Vec3[3]) ,100,3);
                            cv::line(outputImage, Point(Vec4[0], Vec4[1]), Point(Vec4[2], Vec4[3]) ,100,3);
                        }

                        static int counter = 0;
                        int counterThresh = 2;
                        for(unsigned int k=0; k<lines.size();k++){
                            if(Intersection(Vec1,lines[k]) || Intersection(Vec2,lines[k]) || Intersection(Vec3,lines[k]) || Intersection(Vec4,lines[k])){
                            //if(IntersectionUL(lines[k],Vec1[2],Vec1[0],Vec2[1],Vec2[3])){
                                counter++;
//                                cv::line(outputImage,Point(lines[k][0], lines[k][1]), Point(lines[k][2], lines[k][3]),255,5);
//                                vector<maneuver> mans;
//                                maneuver man;
//                                man.ticks = 100;
//                                man.speed = 0;
//                                mans.push_back(man);
//                                setManeuvers(mans);
                            }
                            if(counter > counterThresh)
                                break;
                        }
                        //LOG_INFO(cString::Format("counter %i",counter));
                        //LOG_INFO("right horizontal gap: Second Line Detected");
                        if(DEBUG)
                        {
                            cv::line(outputImage,Point(xValj, yValj),Point(xValEndj,yValEndj), 200, 3);
                        }
                        if(counter > counterThresh){
                            //LOG_INFO("NOT REALLY A HORIZONTAL GAP");
                            counter = 0;
                            continue;
                        }
                        counter = 0;


                        int distance = sqrt(pow(rightLineStartX - leftLineStartX,2) + pow(rightLineStartY - leftLineStartY,2));

                        if(distance > gapDistanceHorizontal - distanceOffsetHorizontal && distance < gapDistanceHorizontal + distanceOffsetHorizontal){
                            gapVectorHorizontal = Vec4i(rightLineStartX, rightLineStartY, leftLineStartX, leftLineStartY);
                            if(DEBUG)
                            {
                                cv::circle(outputImage, Point(rightLineStartX,rightLineStartY), 10, 255, 5);
                                cv::circle(outputImage, Point(leftLineStartX,leftLineStartY), 10, 255, 5);
                            }
                            //LOG_INFO(cString::Format("gapVectorHorizontal %i %i %i %i",gapVectorHorizontal[0], gapVectorHorizontal[1], gapVectorHorizontal[2], gapVectorHorizontal[3]));
                            rightCrossPoints.push_back(Point(rightLineStartX,rightLineStartY));
                            TransmitTicksToCrosspoint(m_filterProperties.horizontalTicks + (m_filterProperties.RightLowerRightY - rightLineStartY));
                            RETURN_NOERROR;
                        }
                    }
                }
            }
        }

        ///Left crosspoint detection (vertical gap)
        vector<Vec4i> potentialLowerLinesLeft;
        Vec4i bestLowerLineLeft;
        Vec4i extendedbestLowerLineLeft;
        Vec2i bestLowerLineLeftVector;
        int bestLowerLineLeftVectorLength;
        int gapDistanceLeft = 220;
        int distanceOffsetLeft = 25;

        for(unsigned int i = 0; i < lines.size(); i++){

            int lineAngleLefti = computeAngleFromVec4i((lines[i]));
            int xVali = lines[i][0];
            int yVali = lines[i][1];

            if((xVali > m_filterProperties.LeftLowerLeftX) && (xVali < m_filterProperties.LeftLowerRightX) &&
                    (yVali < m_filterProperties.LeftLowerLeftY) && (yVali > m_filterProperties.LeftUpperRightY-20) &&
                    lineAngleLefti > -20 && lineAngleLefti < 20){

                potentialLowerLinesLeft.push_back(lines[i]);
            }
        }
        for(unsigned int i = 0; i < potentialLowerLinesLeft.size(); i++){
            if(i==0){
                bestLowerLineLeft = potentialLowerLinesLeft[i];
            }else if(potentialLowerLinesLeft[i][3] < bestLowerLineLeft[3]){
                bestLowerLineLeft = potentialLowerLinesLeft[i];
            }
        }

        bestLowerLineLeftVector[0] = bestLowerLineLeft[2] - bestLowerLineLeft[0];
        bestLowerLineLeftVector[1] = bestLowerLineLeft[3] - bestLowerLineLeft[1];
        bestLowerLineLeftVectorLength = sqrt(pow(bestLowerLineLeftVector[0],2) + pow(bestLowerLineLeftVector[1],2));

        extendedbestLowerLineLeft[0] = bestLowerLineLeft[0];
        extendedbestLowerLineLeft[1] = bestLowerLineLeft[1];
        extendedbestLowerLineLeft[2] = bestLowerLineLeftVector[0] * ((bestLowerLineLeftVectorLength+gapDistanceLeft)/bestLowerLineLeftVectorLength);
        extendedbestLowerLineLeft[3] = bestLowerLineLeftVector[1] * ((bestLowerLineLeftVectorLength+gapDistanceLeft)/bestLowerLineLeftVectorLength);

        extendedbestLowerLineLeft[2] = bestLowerLineLeft[0] + extendedbestLowerLineLeft[2];
        extendedbestLowerLineLeft[3] = (extendedbestLowerLineLeft[3] < 0) ? bestLowerLineLeft[1] + extendedbestLowerLineLeft[3] : bestLowerLineLeft[1] - extendedbestLowerLineLeft[3];
        //LOG_INFO("left vertical gap: First Line Detected");
        //cv::line(outputImage,Point(bestLowerLineLeft[0], bestLowerLineLeft[1]),Point(bestLowerLineLeft[2],bestLowerLineLeft[3]), 200, 3);

        if(DEBUG)
        {
            cv::line(outputImage,Point(extendedbestLowerLineLeft[0], extendedbestLowerLineLeft[1]),Point(extendedbestLowerLineLeft[2],extendedbestLowerLineLeft[3]), 200, 3);
            cv::line(outputImage,Point(bestLowerLineLeft[2]-100, bestLowerLineLeft[3]),Point(bestLowerLineLeft[2]-100,bestLowerLineLeft[3]-gapDistanceLeft), 200, 3);
        }
        //cv::rectangle(outputImage,Point(extendedbestLowerLineLeft[2]-25,extendedbestLowerLineLeft[3]+30), Point(extendedbestLowerLineLeft[2]+25,m_filterProperties.fourCrossUpperRightY), 255, 3);

        for(unsigned int j = 0; j < lines.size(); j++){
            int lineAngleLeftj = computeAngleFromVec4i((lines[j]));
            int xValj = lines[j][0];
            int yValj = lines[j][1];

            if((xValj > extendedbestLowerLineLeft[2] - 25) && (xValj < extendedbestLowerLineLeft[2] + 25) &&
                    (yValj < extendedbestLowerLineLeft[3] + 30) &&
                    lineAngleLeftj > -20 && lineAngleLeftj < 20){


                int lowerLineXEnd = bestLowerLineLeft[2];
                int lowerLineYEnd = bestLowerLineLeft[3];
                int upperLineXStart = lines[j][0];
                int upperLineYStart = lines[j][1];

                Vec4i Vec1 = Vec4i(lowerLineXEnd-15, lowerLineYEnd, upperLineXStart-15, upperLineYStart);
                Vec4i Vec2 = Vec4i(lowerLineXEnd+15, lowerLineYEnd, upperLineXStart+15, upperLineYStart);
                Vec4i Vec3 = Vec4i(lowerLineXEnd-15, lowerLineYEnd, lowerLineXEnd+15, lowerLineYEnd);
                Vec4i Vec4 = Vec4i(upperLineXStart-15, upperLineYStart, upperLineXStart+15, upperLineYStart);

                if(DEBUG)
                {
                    cv::line(outputImage, Point(Vec1[0], Vec1[1]), Point(Vec1[2], Vec1[3]) ,100,3);
                    cv::line(outputImage, Point(Vec2[0], Vec2[1]), Point(Vec2[2], Vec2[3]) ,100,3);
                    cv::line(outputImage, Point(Vec3[0], Vec3[1]), Point(Vec3[2], Vec3[3]) ,100,3);
                    cv::line(outputImage, Point(Vec4[0], Vec4[1]), Point(Vec4[2], Vec4[3]) ,100,3);
                }

                static int counter = 0;
                int counterThresh = 2;
                for(unsigned int k=0; k<lines.size();k++){
                    if(Intersection(Vec1,lines[k]) || Intersection(Vec2,lines[k]) || Intersection(Vec3,lines[k]) || Intersection(Vec4,lines[k])){
                        counter++;
                    if(DEBUG)
                        cv::line(outputImage,Point(lines[k][0], lines[k][1]), Point(lines[k][2], lines[k][3]),255,2);
//                    vector<maneuver> mans;
//                    maneuver man;
//                    man.ticks = 100;
//                    man.speed = 0;
//                    mans.push_back(man);
//                    setManeuvers(mans);
                    }
                    if(counter > counterThresh)
                        break;
                }
                //LOG_INFO(cString::Format("counter %i",counter));
                if(counter > counterThresh){
                    //LOG_INFO("NOT REALLY A LEFT VERTICAL GAP");
                    counter = 0;
                    continue;
                }
                counter = 0;

                //LOG_INFO("left vertical gap: Second Line Detected");

                if(DEBUG)
                    cv::line(outputImage,Point(lines[j][0], lines[j][1]),Point(lines[j][2],lines[j][3]), 200, 3);
                int distance = sqrt(pow(lowerLineXEnd - upperLineXStart,2) + pow(lowerLineYEnd - upperLineYStart,2));

                if(distance > gapDistanceLeft - distanceOffsetLeft && distance < gapDistanceLeft + distanceOffsetLeft && upperLineYStart < lowerLineYEnd){
                    gapVectorVertical = Vec4i(lowerLineXEnd, lowerLineYEnd, upperLineXStart, upperLineYStart);
                    if(DEBUG)
                    {
                        cv::circle(outputImage, Point(lowerLineXEnd,lowerLineYEnd), 10, 255, 5);
                        cv::circle(outputImage, Point(upperLineXStart,upperLineYStart), 10, 255, 5);
                    }
                    leftCrossPoints.push_back(Point(lowerLineXEnd, lowerLineYEnd));
                    TransmitTicksToCrosspoint(m_filterProperties.verticalTicks + (m_filterProperties.RightLowerRightY - lowerLineYEnd));
                    RETURN_NOERROR;
                }
            }
        }
    }
    RETURN_NOERROR;
}



tResult cLaneDetection::detectLane(Mat & outputImage, vector<Vec4i> &lines, Vec4i &nearestLineToCarRight, Vec4i &nearestLineToCarMiddle, Vec4i &nearestLineToCarLeft){
    vector<Vec4i> leftLine;
    vector<Vec4i> middleLine;
    vector<Vec4i> rightLine;

    int oldAngleOffset = 25;
    int oldXPosOffset = 25;
    int angleOffsetEMER = 5;
    int leftToMid = 117;
    int MidToRight = 113;
    int distanceOffset = 10;
    int distanceOffsetY = 50;

    if(EMERGENCY_SEARCH){
        for(unsigned int i = 0; i < lines.size(); i++){
            int xVal = lines[i][0];
            int yVal = lines[i][1];
            float angleLeft = computeAngleFromVec4i(lines[i]);
            for(unsigned int j = 0; j < lines.size(); j++){
                float angleMiddle = computeAngleFromVec4i(lines[j]);
                if(angleLeft < angleMiddle+angleOffsetEMER && angleLeft > angleMiddle-angleOffsetEMER &&
                        (lines[j][0]-lines[i][0])>leftToMid-distanceOffset && (lines[j][0]-lines[i][0])<leftToMid+distanceOffset &&
                        (lines[j][1] < lines[i][1] + distanceOffsetY && lines[j][1] > lines[i][1] - distanceOffsetY)){
                    for(unsigned int k = 0; k < lines.size(); k++){
                        float angleRight = computeAngleFromVec4i(lines[k]);
                        if(angleMiddle < angleRight+angleOffsetEMER && angleMiddle > angleRight-angleOffsetEMER &&
                                (lines[k][0]-lines[j][0])>MidToRight-distanceOffset && (lines[k][0]-lines[j][0])<MidToRight+distanceOffset &&
                                (lines[k][1] < lines[j][1] + distanceOffsetY && lines[k][1] > lines[j][1] - distanceOffsetY)){
                            nearestLineToCarLeft = lines[i];
                            nearestLineToCarMiddle = lines[j];
                            nearestLineToCarRight = lines[k];
                            last_right_line = lines[k];
                            last_middle_line = lines[j];
                            last_left_line = lines[i];
                             // LOG_INFO(cString::Format("EME ANGLES: %f %f %f", angleLeft, angleMiddle, angleLeft));
                            break;
                        }
                    }
                }
            }
            if((xVal > m_filterProperties.RightUpperLeftX) && (xVal < m_filterProperties.RightUpperRightX) &&
                    (yVal < m_filterProperties.RightLowerLeftY) && (yVal > m_filterProperties.RightUpperRightY)){
                rightLine.push_back(lines[i]);
            }
            else if((xVal > m_filterProperties.MidUpperLeftX) && (xVal < m_filterProperties.MidUpperRightX) &&
                    (yVal < m_filterProperties.MidLowerLeftY) && (yVal > m_filterProperties.MidUpperRightY)){
                middleLine.push_back(lines[i]);
            }
            else if((xVal > m_filterProperties.LeftUpperLeftX) && (xVal < m_filterProperties.LeftUpperRightX) &&
                    (yVal < m_filterProperties.LeftLowerLeftY) && (yVal > m_filterProperties.LeftUpperRightY)){
                leftLine.push_back(lines[i]);
            }
        }
        // Keine Straße gefunden
        if(nearestLineToCarRight == Vec4i (0,0,0,0)){
            EME_SEARCH_NO_STREET_COUNTER++;
        }else{
            EME_SEARCH_NO_STREET_COUNTER = -1;
        }
        if(!rightLine.empty() && !middleLine.empty() && !leftLine.empty() && (((wheelCountLeft + wheelCountRight) / 2) > (lastPedestianCrossingDetected + 200))){
            EMERGENCY_SEARCH = false;
            EME_SEARCH_NO_STREET_COUNTER = -1;
            last_right_line = Vec4i(0,0,0,0);
            last_middle_line = Vec4i(0,0,0,0);
            last_left_line = Vec4i(0,0,0,0);
        }
    }else{
        if(m_filterProperties.testBool){
            if(!isOnLeftLane){
                SwitchToLaneLeft();

                //LOG_INFO("SWITCHING TO LEFT LANE");
                m_filterProperties.testBool = false;
            }
            else if(isOnLeftLane){
                SwitchToLaneRight();

                //LOG_INFO("SWITCHING TO RIGHT LANE");
                m_filterProperties.testBool = false;
            }
        }

        for(unsigned int i = 0; i < lines.size(); i++){
            int xVal = lines[i][0];
            int yVal = lines[i][1];
            float angle = computeAngleFromVec4i(lines[i]);

            if(angle < m_filterProperties.maxAngle && angle > m_filterProperties.minAngle){
                //Offset if poly ROI
                float offsetRight = 0;
                if(m_filterProperties.RightLowerLeftX - m_filterProperties.RightUpperLeftX != 0){
                    offsetRight = float(m_filterProperties.RightUpperLeftX - m_filterProperties.RightLowerLeftX) / (m_filterProperties.RightLowerLeftY - m_filterProperties.RightUpperLeftY) * (yVal -m_filterProperties.RightUpperLeftY);
                }
                float offsetMid = 0;
                if(m_filterProperties.MidLowerLeftX - m_filterProperties.MidUpperLeftX != 0){
                    offsetMid = float(m_filterProperties.MidUpperLeftX - m_filterProperties.MidLowerLeftX) / (m_filterProperties.MidLowerLeftY - m_filterProperties.MidUpperLeftY) * (yVal - m_filterProperties.MidUpperLeftY);
                }
                float offsetLeft = 0;
                if(m_filterProperties.LeftLowerLeftX - m_filterProperties.LeftUpperLeftX != 0){
                    offsetLeft = float(m_filterProperties.LeftUpperLeftX - m_filterProperties.LeftLowerLeftX) / (m_filterProperties.LeftLowerLeftY - m_filterProperties.LeftUpperLeftY) * (yVal - m_filterProperties.LeftUpperLeftY);
                }

                //!!RECHTE LINIE
                //Betrachte die Linie wenn sie im ROI ist
                if((xVal > m_filterProperties.RightUpperLeftX - offsetRight) && (xVal < m_filterProperties.RightUpperRightX - offsetRight) &&
                        (yVal < m_filterProperties.RightLowerLeftY) && (yVal > m_filterProperties.RightUpperRightY)){
                    //LOG_INFO(adtf_util::cString::Format("OffsetRight: %i %i %f %i %i %i", xVal, yVal, offsetRight,m_filterProperties.RightLowerLeftY - m_filterProperties.RightUpperLeftY, m_filterProperties.RightUpperLeftX - m_filterProperties.RightLowerLeftX, yVal -m_filterProperties.RightUpperLeftY));
                    //Nehme die Linie whärend der ersten 5 frames oder wenn seit N frames keine Linie mehr gefunden wurde
                    if(goThroughCounter < 5 || faultyLineCounterRight > m_filterProperties.faultyCounterThreshold){
                        rightLine.push_back(lines[i]);
                    }
                    else{
                        if(faultyLineCounterRight > 0){
                            if(faultyLineCounterMiddle == -1){
                                if((oldAngleMiddle-oldAngleOffset < angle) && (angle < oldAngleMiddle+oldAngleOffset)){
                                    rightLine.push_back(lines[i]);
                                }
                            }
                            else if(faultyLineCounterLeft == -1){
                                if((oldAngleLeft-oldAngleOffset < angle) && (angle < oldAngleLeft+oldAngleOffset)){
                                    rightLine.push_back(lines[i]);
                                }
                            }
                        }
                        else{
                            //nehme Linie wenn der Winkel und die Xposition nicht zu sehr von der alten Linie abweichen
                            if((oldAngleRight-oldAngleOffset < angle) && (angle < oldAngleRight+oldAngleOffset) &&
                                    (((oldXPosRight-oldXPosOffset < xVal) && (xVal < oldXPosRight+oldXPosOffset)) || oldXPosRight == 0)){
                                rightLine.push_back(lines[i]);
                            }
                        }
                    }
                }
                //!!MITTLERE LINIE
                else if((xVal > m_filterProperties.MidUpperLeftX - offsetMid) && (xVal < m_filterProperties.MidUpperRightX - offsetMid) &&
                        (yVal < m_filterProperties.MidLowerLeftY) && (yVal > m_filterProperties.MidUpperRightY)){
                    if(goThroughCounter < 5 || faultyLineCounterMiddle > m_filterProperties.faultyCounterThreshold){
                        middleLine.push_back(lines[i]);
                    }
                    else{
                        if(faultyLineCounterMiddle > 0){
                            if(faultyLineCounterRight == -1){
                                if((oldAngleRight-oldAngleOffset < angle) && (angle < oldAngleRight+oldAngleOffset)){
                                    middleLine.push_back(lines[i]);
                                }
                            }
                            else if(faultyLineCounterLeft == -1){
                                if((oldAngleLeft-oldAngleOffset < angle) && (angle < oldAngleLeft+oldAngleOffset)){
                                    middleLine.push_back(lines[i]);
                                }
                            }
                        }
                        else{
                            if((oldAngleMiddle-oldAngleOffset < angle) && (angle < oldAngleMiddle+oldAngleOffset) &&
                                    (((oldXPosMiddle-oldXPosOffset < xVal) && (xVal < oldXPosMiddle+oldXPosOffset)) || oldXPosMiddle == 0)){
                                middleLine.push_back(lines[i]);
                            }
                        }
                    }
                }
                //!!LINKE LINIE
                else if((xVal > m_filterProperties.LeftUpperLeftX - offsetLeft) && (xVal < m_filterProperties.LeftUpperRightX - offsetLeft) &&
                        (yVal < m_filterProperties.LeftLowerLeftY) && (yVal > m_filterProperties.LeftUpperRightY)){
                    if(goThroughCounter < 5 || faultyLineCounterLeft > m_filterProperties.faultyCounterThreshold){
                        leftLine.push_back(lines[i]);
                    }
                    else{
                        if(faultyLineCounterLeft > 0){
                            if(faultyLineCounterRight == -1){
                                if((oldAngleRight-oldAngleOffset < angle) && (angle < oldAngleRight+oldAngleOffset)){
                                    leftLine.push_back(lines[i]);
                                }
                            }
                            else if(faultyLineCounterMiddle == -1){
                                if((oldAngleMiddle-oldAngleOffset < angle) && (angle < oldAngleMiddle+oldAngleOffset)){
                                    leftLine.push_back(lines[i]);
                                }
                            }
                        }
                        else{
                            if((oldAngleLeft-oldAngleOffset < angle) && (angle < oldAngleLeft+oldAngleOffset) &&
                                    (((oldXPosLeft-oldXPosOffset < xVal) && (xVal < oldXPosLeft+oldXPosOffset)) || oldXPosLeft == 0)){
                                leftLine.push_back(lines[i]);
                            }
                        }
                    }
                }
            }
        }



        if(DEBUG){


            for(unsigned int i = 0; i < leftLine.size();i++){
                cv::line(outputImage, Point(leftLine[i][0],leftLine[i][1]), Point(leftLine[i][2], leftLine[i][3]), Scalar(100,150,100), 3, CV_AA);
            }
            for(unsigned int i = 0; i < middleLine.size();i++){
                cv::line(outputImage, Point(middleLine[i][0],middleLine[i][1]), Point(middleLine[i][2], middleLine[i][3]), Scalar(100,150,100), 3, CV_AA);
            }
            for(unsigned int i = 0; i < rightLine.size();i++){
                cv::line(outputImage, Point(rightLine[i][0],rightLine[i][1]), Point(rightLine[i][2], rightLine[i][3]), Scalar(100,150,100), 3, CV_AA);
            }

            // Draw Left ROI
            cv::line(outputImage,Point(m_filterProperties.LeftUpperLeftX,m_filterProperties.LeftUpperLeftY), Point(m_filterProperties.LeftUpperRightX,m_filterProperties.LeftUpperRightY),100,2);
            cv::line(outputImage,Point(m_filterProperties.LeftUpperRightX,m_filterProperties.LeftUpperRightY),Point(m_filterProperties.LeftLowerRightX,m_filterProperties.LeftLowerRightY),100,2);
            cv::line(outputImage,Point(m_filterProperties.LeftLowerRightX,m_filterProperties.LeftLowerRightY),Point(m_filterProperties.LeftLowerLeftX,m_filterProperties.LeftLowerLeftY),100,2);
            cv::line(outputImage,Point(m_filterProperties.LeftLowerLeftX,m_filterProperties.LeftLowerLeftY),Point(m_filterProperties.LeftUpperLeftX,m_filterProperties.LeftUpperLeftY),100,2);

            // Draw Middle ROI
            cv::line(outputImage,Point(m_filterProperties.MidUpperLeftX,m_filterProperties.MidUpperLeftY), Point(m_filterProperties.MidUpperRightX,m_filterProperties.MidUpperRightY),100,2);
            cv::line(outputImage,Point(m_filterProperties.MidUpperRightX,m_filterProperties.MidUpperRightY),Point(m_filterProperties.MidLowerRightX,m_filterProperties.MidLowerRightY),100,2);
            cv::line(outputImage,Point(m_filterProperties.MidLowerRightX,m_filterProperties.MidLowerRightY),Point(m_filterProperties.MidLowerLeftX,m_filterProperties.MidLowerLeftY),100,2);
            cv::line(outputImage,Point(m_filterProperties.MidLowerLeftX,m_filterProperties.MidLowerLeftY),Point(m_filterProperties.MidUpperLeftX,m_filterProperties.MidUpperLeftY),100,2);

            //Draw Right ROI
            cv::line(outputImage,Point(m_filterProperties.RightUpperLeftX,m_filterProperties.RightUpperLeftY), Point(m_filterProperties.RightUpperRightX,m_filterProperties.RightUpperRightY),100,2);
            cv::line(outputImage,Point(m_filterProperties.RightUpperRightX,m_filterProperties.RightUpperRightY),Point(m_filterProperties.RightLowerRightX,m_filterProperties.RightLowerRightY),100,2);
            cv::line(outputImage,Point(m_filterProperties.RightLowerRightX,m_filterProperties.RightLowerRightY),Point(m_filterProperties.RightLowerLeftX,m_filterProperties.RightLowerLeftY),100,2);
            cv::line(outputImage,Point(m_filterProperties.RightLowerLeftX,m_filterProperties.RightLowerLeftY),Point(m_filterProperties.RightUpperLeftX,m_filterProperties.RightUpperLeftY),100,2);

        }

        unsigned int maxLinesInROI = 100;
        if(!leftLine.empty() && leftLine.size() < maxLinesInROI){
            leftLineBuffer = leftLine;
            faultyLineCounterLeft = -1;
        }
        else if(faultyLineCounterLeft == -1){
            faultyLineCounterLeft = (wheelCountLeft+wheelCountRight)/2;
        }

        if(!middleLine.empty() && middleLine.size() < maxLinesInROI){
            middleLineBuffer = middleLine;
            faultyLineCounterMiddle = -1;
        }
        else if(faultyLineCounterMiddle == -1){
            faultyLineCounterMiddle = (wheelCountLeft+wheelCountRight)/2;
        }

        if(!rightLine.empty() && rightLine.size() < maxLinesInROI){
            rightLineBuffer = rightLine;
            faultyLineCounterRight = -1;
        }
        else if(faultyLineCounterRight == -1){
            faultyLineCounterRight = (wheelCountLeft+wheelCountRight)/2;
        }

        if(faultyLineCounterRight == -1 && !rightLineBuffer.empty()){
            nearestLineToCarRight = computeNearestLineToCar(rightLine, "right");
        }
        else{
            nearestLineToCarRight = computeNearestLineToCar(rightLineBuffer, "right");
        }
        if(faultyLineCounterLeft == -1 && !leftLineBuffer.empty()){
            nearestLineToCarLeft = computeNearestLineToCar(leftLine, "left");

        }
        else{
            nearestLineToCarLeft = computeNearestLineToCar(leftLineBuffer, "left");
        }
        if(faultyLineCounterMiddle == -1 && !middleLineBuffer.empty()){
            nearestLineToCarMiddle = computeNearestLineToCar(middleLine, "middle");
        }
        else{
            nearestLineToCarMiddle = computeNearestLineToCar(middleLineBuffer, "middle");
        }

        // Wir haben eine straße
        if(faultyLineCounterLeft == -1 && faultyLineCounterMiddle == -1 && faultyLineCounterRight == -1){
            last_left_line = nearestLineToCarLeft;
            last_middle_line = nearestLineToCarMiddle;
            last_right_line = nearestLineToCarRight;
            //LOG_INFO(cString::Format("LAST_LANE %f %f %f", computeAngleFromVec4i(last_left_line), computeAngleFromVec4i(last_middle_line), computeAngleFromVec4i(last_right_line)));
        }

        if((wheelCountLeft+wheelCountRight)/2 - faultyLineCounterLeft > m_filterProperties.emergencyThreshold && faultyLineCounterLeft != -1 &&
                (wheelCountLeft+wheelCountRight)/2- faultyLineCounterMiddle > m_filterProperties.emergencyThreshold && faultyLineCounterMiddle != -1 &&
                (wheelCountLeft+wheelCountRight)/2 -faultyLineCounterRight > m_filterProperties.emergencyThreshold && faultyLineCounterRight != -1){
            //LOG_INFO("Obacht!");
            EMERGENCY_SEARCH = true;
        }

        adaptROI(nearestLineToCarRight, nearestLineToCarMiddle, nearestLineToCarLeft);
    }
    if(DEBUG){
        cv::line(outputImage, Point(nearestLineToCarRight[0],nearestLineToCarRight[1]), Point(nearestLineToCarRight[2], nearestLineToCarRight[3]), 255, 3, CV_AA);
        cv::line(outputImage, Point(nearestLineToCarLeft[0],nearestLineToCarLeft[1]), Point(nearestLineToCarLeft[2], nearestLineToCarLeft[3]), 255, 3, CV_AA);
        cv::line(outputImage, Point(nearestLineToCarMiddle[0],nearestLineToCarMiddle[1]), Point(nearestLineToCarMiddle[2], nearestLineToCarMiddle[3]), 255, 3, CV_AA);
    }

    //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /// Beim fünften Durchlauf werden die Werte der aktuellen nearestLineToCar gespeichert
    Vec4i Test;
    Test.zeros();
    if(goThroughCounter >= 5){
        oldAngleRight = computeAngleFromVec4i(nearestLineToCarRight);
        oldAngleMiddle = computeAngleFromVec4i(nearestLineToCarMiddle);
        oldAngleLeft = computeAngleFromVec4i(nearestLineToCarLeft);

        oldXPosRight = nearestLineToCarRight[0];
        oldXPosMiddle = nearestLineToCarMiddle[0];
        oldXPosLeft = nearestLineToCarLeft[0];
    }

    goThroughCounter++;

    RETURN_NOERROR;
}

tResult cLaneDetection::detectEmergencyHorizontalStoplines(vector<Vec4i> lines, vector<stopLine> &stopLinesHorizontal){
    for(unsigned int i = 0; i < lines.size(); i++){
        Vec4i Tmp = lines[i];
        int xVal = Tmp[0];
        int yVal = Tmp[1];
        int xValEnd = Tmp[2];
        int yValEnd = Tmp[3];
        float angle = computeAngleFromVec4i(Tmp);

        if((xVal > m_filterProperties.LeftUpperLeftX) && (xVal < m_filterProperties.RightUpperRightX) &&
                (yVal < m_filterProperties.LeftLowerLeftY) && (yVal > m_filterProperties.RightUpperRightY) &&
                (xValEnd > m_filterProperties.LeftUpperLeftX) && (xValEnd < m_filterProperties.RightUpperRightX) &&
                (yValEnd < m_filterProperties.LeftLowerLeftY) && (yValEnd > m_filterProperties.RightUpperRightY)){
            if(angle > 85 && angle <= 90 || angle < -85 && angle > -90){
                stopLine TmpStopline;
                TmpStopline.firstLine = Tmp;
                TmpStopline.secondLine = Tmp;
                stopLinesHorizontal.push_back(TmpStopline);
            }
        }
    }
}


tResult cLaneDetection::checkCrossManeuver(vector<Point> &rightCrossPoint, vector<Point> &leftCrossPoint, Vec4i gapVectorVertical, Vec4i gapVectorHorizontal){

    /// STRIAGHT
    if(nextManeuverStraight && (!rightCrossPoint.empty() || !leftCrossPoint.empty())){
        nextManeuverStraight = false;
        m_stManTicks = 500 + (wheelCountLeft + wheelCountRight) / 2;
        // LOG_INFO(cString::Format("Ticks STraight %i", m_stManTicks));
        RETURN_NOERROR;
    }
    if(m_stManTicks < (wheelCountLeft + wheelCountRight) /2 ){
        // LOG_INFO("STRAIGHT FINISHED");
        m_stManTicks = INT32_MAX;
        transmitManeuverFinished(maneuverID);
        RETURN_NOERROR;
    }

    //            if(nextManeuverLeft || nextManeuverRight || m_stManTicks - 300 > (wheelCountLeft + wheelCountRight) /2 ){
    //                TransmitTicksToLine(m_filterProperties.horizontalTicks + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yValEnd));
    //            }
    float gapVectorAngle;
    float steeringAngle;
    bool isVerticalGap;
    if(gapVectorVertical != Vec4i(0,0,0,0)){
        gapVectorAngle = computeAngleFromVec4i(gapVectorVertical);
        steeringAngle = gapVectorAngle*60/100;
        isVerticalGap = true;
    }else if(gapVectorHorizontal != Vec4i(0,0,0,0)){
        gapVectorAngle = computeAngleFromVec4i(gapVectorHorizontal);
        steeringAngle = (gapVectorAngle<0) ? (gapVectorAngle+90)*60/100 :  (gapVectorAngle-90)*60/100;
        isVerticalGap = false;
    }else{
        if(!rightCrossPoint.empty() || !leftCrossPoint.empty()){
            //LOG_INFO("Crosspoint but no Gapvector!!!!!!!");
            RETURN_NOERROR;
        }
    }

    if(nextManeuverRight){
        if(!rightCrossPoint.empty()){
            nextManeuverRight = false;

            Point test = rightCrossPoint[0];
            int yValEnd = test.y;

            vector<maneuver> gutesManeuver;
            maneuver a;
            a.ticks = (isVerticalGap) ? m_filterProperties.verticalTicks+(m_filterProperties.RightUpperLeftY - yValEnd) : m_filterProperties.horizontalTicks+(m_filterProperties.RightUpperLeftY - yValEnd);
            a.steeringAngle = 2*steeringAngle;
            a.speed = NO_LD_SPEED;
            a.turnLeft = false;
            a.turnRight = true;
            maneuver b;
            b.ticks = 1;
            b.steeringAngle = 0;
            b.speed = NO_LD_SPEED;
            b.turnLeft = false;
            b.turnRight = true;
            maneuver c;
            c.ticks = 150;
            c.steeringAngle = 100;
            c.speed = NO_LD_SPEED;
            c.turnLeft = false;
            c.turnRight = true;
            //LOG_INFO(cString::Format("Ausrichteticks: %i", a.ticks));
            //LOG_INFO(cString::Format("Ausrichtewinkel: %f", a.steeringAngle));
            gutesManeuver.push_back(c);
            gutesManeuver.push_back(b);
            gutesManeuver.push_back(a);
            setManeuvers(gutesManeuver);

            //LOG_INFO("Abbiegen Rechts (rightcrosspoint)");
            RETURN_NOERROR;
        }
        else if(!leftCrossPoint.empty()){
            nextManeuverRight = false;

            Point test = leftCrossPoint[0];
            int yValEnd = test.y;

            vector<maneuver> gutesManeuver;
            maneuver a;
            a.ticks = m_filterProperties.verticalTicks + (m_filterProperties.RightUpperLeftY - yValEnd);
            a.steeringAngle = steeringAngle;
            a.speed = NO_LD_SPEED;
            a.turnLeft = false;
            a.turnRight = true;
            maneuver b;
            b.ticks = 1;
            b.steeringAngle = 0;
            b.speed = NO_LD_SPEED;
            b.turnLeft = false;
            b.turnRight = true;
            maneuver c;
            c.ticks = 150;
            c.steeringAngle = 100;
            c.speed = NO_LD_SPEED;
            c.turnLeft = false;
            c.turnRight = true;
            //LOG_INFO(cString::Format("Ausrichteticks: %i", a.ticks));
            //LOG_INFO(cString::Format("Ausrichtewinkel: %f", a.steeringAngle));
            gutesManeuver.push_back(c);
            gutesManeuver.push_back(b);
            gutesManeuver.push_back(a);
            setManeuvers(gutesManeuver);

            //LOG_INFO("Abbiegen Rechts (leftcrosspoint)");
            RETURN_NOERROR;
        }
    }
    else if(nextManeuverLeft){
        if(!rightCrossPoint.empty()){
            nextManeuverLeft = false;

            Point test = rightCrossPoint[0];
            int yValEnd = test.y;

            vector<maneuver> gutesManeuver;
            maneuver a;
            a.ticks = (isVerticalGap) ? m_filterProperties.verticalTicks+(m_filterProperties.RightUpperLeftY - yValEnd) : m_filterProperties.horizontalTicks+(m_filterProperties.RightUpperLeftY - yValEnd);
            a.steeringAngle = steeringAngle;
            a.speed = NO_LD_SPEED;
            a.turnLeft = true;
            a.turnRight = false;
            maneuver b;
            b.ticks = (isVerticalGap) ? 1 : 30;
            b.steeringAngle = 0;
            b.speed = NO_LD_SPEED;
            b.turnLeft = true;
            b.turnRight = false;
            maneuver c;
            c.ticks = 240;
            c.steeringAngle = -100;
            c.speed = NO_LD_SPEED;
            c.turnLeft = true;
            c.turnRight = false;
            //LOG_INFO(cString::Format("Ausrichteticks: %i", a.ticks));
            //LOG_INFO(cString::Format("Ausrichtewinkel: %f", a.steeringAngle));
            gutesManeuver.push_back(c);
            gutesManeuver.push_back(b);
            gutesManeuver.push_back(a);
            setManeuvers(gutesManeuver);

            //LOG_INFO("Abbiegen Links (rightcrosspoint)");
            RETURN_NOERROR;
        }
        if(!leftCrossPoint.empty()){
            nextManeuverLeft = false;

            Point test = leftCrossPoint[0];
            int yValEnd = test.y;

            vector<maneuver> gutesManeuver;
            maneuver a;
            a.ticks = m_filterProperties.verticalTicks + (m_filterProperties.RightUpperLeftY - yValEnd);
            a.steeringAngle = steeringAngle;
            a.speed = NO_LD_SPEED;
            a.turnLeft = true;
            a.turnRight = false;
            maneuver b;
            b.ticks = 10;
            b.steeringAngle = 0;
            b.speed = NO_LD_SPEED;
            b.turnLeft = true;
            b.turnRight = false;
            maneuver c;
            c.ticks = 235;
            c.steeringAngle = -100;
            c.speed = NO_LD_SPEED;
            c.turnLeft = true;
            c.turnRight = false;
            //LOG_INFO(cString::Format("Ausrichteticks: %i", a.ticks));
            //LOG_INFO(cString::Format("Ausrichtewinkel: %f", a.steeringAngle));
            gutesManeuver.push_back(c);
            gutesManeuver.push_back(b);
            gutesManeuver.push_back(a);
            setManeuvers(gutesManeuver);

            //LOG_INFO("Abbiegen Links (leftcrosspoint)");
            RETURN_NOERROR;
        }
    }
    //LOG_INFO(cString::Format("Ticks %i", (wheelCountLeft + wheelCountRight) /2));

    //    /// LEFT OR RIGHT
    //    /// HORIZONTAL
    //    for(unsigned int i = 0; i < stopLinesHorizontal.size(); i++){
    //        stopLine Tmp = stopLinesHorizontal[i];
    //        int xVal = Tmp.firstLine[0];
    //        int yVal = Tmp.firstLine[1];
    //        int xValEnd = Tmp.firstLine[2];
    //        int yValEnd = Tmp.firstLine[3];
    //        float angle = computeAngleFromVec4i(Tmp.firstLine);

    //        if((xVal > m_filterProperties.LeftUpperLeftX) && (xVal < m_filterProperties.RightUpperRightX) &&
    //                (yVal < m_filterProperties.LeftLowerLeftY) && (yVal > m_filterProperties.RightUpperRightY) &&
    //                (xValEnd > m_filterProperties.LeftUpperLeftX) && (xValEnd < m_filterProperties.RightUpperRightX) &&
    //                (yValEnd < m_filterProperties.LeftLowerLeftY) && (yValEnd > m_filterProperties.RightUpperRightY)){

    //            if(nextManeuverLeft || nextManeuverRight || m_stManTicks - 300 > (wheelCountLeft + wheelCountRight) /2 ){
    //                TransmitTicksToLine(m_filterProperties.horizontalTicks + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yValEnd));
    //            }
    //            if(nextManeuverRight){
    //                //rechts abbiegen horizontal
    //                vector<maneuver> gutesManeuver;
    //                maneuver a;
    //                a.ticks = m_filterProperties.horizontalTicks + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yValEnd);
    //                a.steeringAngle =(angle<0) ? (angle+90)*60/100 :  (angle-90)*60/100;
    //                a.speed = NO_LD_SPEED;
    //                a.turnLeft = false;
    //                a.turnRight = true;
    //                maneuver b;
    //                b.ticks = 40;
    //                b.steeringAngle = 0;
    //                b.speed = NO_LD_SPEED;
    //                b.turnLeft = false;
    //                b.turnRight = true;
    //                maneuver c;
    //                c.ticks = 150;
    //                c.steeringAngle = 100;
    //                c.speed = NO_LD_SPEED;
    //                c.turnLeft = false;
    //                c.turnRight = true;
    //                LOG_INFO(cString::Format("angle %f", angle));
    //                LOG_INFO(cString::Format("%f", a.steeringAngle));
    //                gutesManeuver.push_back(c);
    //                gutesManeuver.push_back(b);
    //                gutesManeuver.push_back(a);
    //                setManeuvers(gutesManeuver);

    //                nextManeuverRight = false;

    //                LOG_INFO("Horizontal Rechts");
    //                RETURN_NOERROR;
    //            }else if(nextManeuverLeft){
    //                //links abbiegen horizontal
    //                vector<maneuver> gutesManeuver;
    //                maneuver a;
    //                a.ticks = m_filterProperties.horizontalTicks;
    //                a.steeringAngle =(angle<0) ? (angle+90)*60/100 :  (angle-90)*60/100;
    //                a.speed = NO_LD_SPEED;
    //                a.turnLeft = true;
    //                a.turnRight = false;
    //                maneuver b;
    //                b.ticks = 80 + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yValEnd);
    //                b.steeringAngle = 0;
    //                b.speed = NO_LD_SPEED;
    //                b.turnLeft = true;
    //                b.turnRight = false;
    //                maneuver c;
    //                c.ticks = 220;
    //                c.steeringAngle = -100;
    //                c.speed = NO_LD_SPEED;
    //                c.turnLeft = true;
    //                c.turnRight = false;
    //                LOG_INFO(cString::Format("angle %f", angle));
    //                LOG_INFO(cString::Format("%f", a.steeringAngle));
    //                gutesManeuver.push_back(c);
    //                gutesManeuver.push_back(b);
    //                gutesManeuver.push_back(a);
    //                setManeuvers(gutesManeuver);

    //                nextManeuverLeft = false;

    //                LOG_INFO("Horizontal Links");
    //                RETURN_NOERROR;
    //            }
    //        }
    //    }

    //    /// VERTICAL
    //    for(unsigned int i = 0; i < stopLinesVertical.size(); i++){
    //        stopLine Tmp = stopLinesVertical[i];
    //        int xVal = Tmp.firstLine[0];
    //        int yVal = Tmp.firstLine[1];
    //        int yValEnd = Tmp.firstLine[1] < Tmp.firstLine[3] ? Tmp.firstLine[1] : Tmp.firstLine[3];
    //        float angle = computeAngleFromVec4i(Tmp.firstLine);

    //        if(yVal > m_filterProperties.MidUpperRightY + 50){

    //            //TransmitTicksToLine(m_filterProperties.verticalTicks  + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yVal));

    //            if(xVal > m_filterProperties.MidUpperRightX){//stoplinie rechts
    //                if(nextManeuverRight){
    //                    vector<maneuver> gutesManeuver;
    //                    maneuver a;
    //                    a.ticks = m_filterProperties.verticalTicks + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yVal);
    //                    a.steeringAngle = angle*60/100;
    //                    a.speed = NO_LD_SPEED;
    //                    a.turnLeft = false;
    //                    a.turnRight = true;
    //                    maneuver b;
    //                    b.ticks = 160;
    //                    b.steeringAngle = 100;
    //                    b.speed = NO_LD_SPEED;
    //                    b.turnLeft = false;
    //                    b.turnRight = true;
    //                    LOG_INFO(cString::Format("angle %f", angle));
    //                    LOG_INFO(cString::Format("%f", a.steeringAngle));
    //                    gutesManeuver.push_back(b);
    //                    gutesManeuver.push_back(a);
    //                    setManeuvers(gutesManeuver);

    //                    LOG_INFO("Vertikal Rechts Stopline Rechts");
    //                    RETURN_NOERROR;
    //                }//rechts
    //            }else{
    //                if(nextManeuverLeft){

    //                    vector<maneuver> gutesManeuver;
    //                    maneuver a;
    //                    a.ticks = m_filterProperties.verticalTicks;
    //                    a.steeringAngle = (angle*60/100);
    //                    a.speed = NO_LD_SPEED;
    //                    a.turnLeft = true;
    //                    a.turnRight = false;
    //                    maneuver b;
    //                    b.ticks = 130 + m_filterProperties.pixelToTicks*(m_filterProperties.RightUpperRightY - yVal);
    //                    b.steeringAngle = 0;
    //                    b.speed = NO_LD_SPEED;
    //                    b.turnLeft = true;
    //                    b.turnRight = false;
    //                    maneuver c;
    //                    c.ticks = 235;
    //                    c.steeringAngle = -100;
    //                    c.speed = NO_LD_SPEED;
    //                    c.turnLeft = true;
    //                    c.turnRight = false;
    //                    LOG_INFO(cString::Format("angle %f", angle));
    //                    LOG_INFO(cString::Format("%f", a.steeringAngle));
    //                    gutesManeuver.push_back(c);
    //                    gutesManeuver.push_back(b);
    //                    gutesManeuver.push_back(a);
    //                    setManeuvers(gutesManeuver);

    //                    LOG_INFO("Vertikal Links Stopline Links");
    //                    RETURN_NOERROR;
    //                }//links
    //            }
    //        }
    //    }
    RETURN_NOERROR;
}

tResult cLaneDetection::setStartEndPoint(vector<Vec4i> &lines){
    for(unsigned int i=0; i<lines.size(); i++){
        //wenn yStart kleiner als yEnd dann tausche start und endpunkt
        if(lines[i][1] < lines[i][3]){
            swap(lines[i][0],lines[i][2]);
            swap(lines[i][1],lines[i][3]);
        }
    }
    RETURN_NOERROR;
}

tFloat32 cLaneDetection::computeAngleFromVec4i(const Vec4i vector){
    //endpointX - startpoinX
    int x = vector[2] - vector[0];

    //endpointY - startpoinY
    int y = vector[3] - vector[1];

    if(x == 0 && y == 0){
        return 0;
    }
    return 180*asin(x/(sqrt(x*x + y*y)))/3.14159;
}

tFloat32 cLaneDetection::computeRadianFromVec4i(const Vec4i vector){
    //endpointX - startpoinX
    int x = vector[2] - vector[0];
    //endpointY - startpoinY
    int y = vector[3] - vector[1];

    return asin(x/(sqrt(x*x + y*y)));
}

Vec4i cLaneDetection::computeNearestLineToCar(vector<Vec4i> &lineVector, string lanePosition){
    Vec4i nearestLineToCarLower = Vec4i (0,0,0,0);
    Vec4i nearestLineToCarUpper = Vec4i (0,0,0,0);

    int yDiffRight = m_filterProperties.RightLowerLeftY - m_filterProperties.RightUpperLeftY;
    int yDiffMiddle = m_filterProperties.MidLowerLeftY - m_filterProperties.MidUpperLeftY;
    int yDiffLeft = m_filterProperties.LeftLowerLeftY - m_filterProperties.LeftUpperLeftY;

    if(cString::IsEqual(lanePosition, "right")){
        for(unsigned int i = 0; i < lineVector.size(); i++){
            if(lineVector[i][1] > m_filterProperties.RightUpperLeftY+yDiffRight/2){
                //wenn es noch keine linie gibt setzte aktuelle linie
                if(nearestLineToCarLower[0] == 0){
                    nearestLineToCarLower = lineVector[i];
                }
                //Wenn der x Wert des startpunktes kleiner ist, ist die linie näher am auto
                else if(nearestLineToCarLower[0] > lineVector[i][0]){
                    nearestLineToCarLower = lineVector[i];
                }
            }else{
                //wenn es noch keine linie gibt setzte aktuelle linie
                if(nearestLineToCarUpper[0] == 0){
                    nearestLineToCarUpper = lineVector[i];
                }
                //Wenn der x Wert des startpunktes kleiner ist, ist die linie näher am auto
                else if(nearestLineToCarUpper[0] > lineVector[i][0]){
                    nearestLineToCarUpper = lineVector[i];
                }
            }
        }
    }
    else if(cString::IsEqual(lanePosition, "middle")){
        for( unsigned int i = 0; i < lineVector.size(); i++){
            if(lineVector[i][1] > m_filterProperties.MidUpperLeftY+yDiffMiddle/2){
                //wenn es noch keine linie gibt setzte aktuelle linie
                if(nearestLineToCarLower[0] == 0){
                    nearestLineToCarLower = lineVector[i];
                }
                //Wenn der x Wert des startpunktes größer ist, ist die linie näher am auto
                else if(nearestLineToCarLower[0] < lineVector[i][0]){
                    nearestLineToCarLower = lineVector[i];
                }
            }else{
                //wenn es noch keine linie gibt setzte aktuelle linie
                if(nearestLineToCarUpper[0] == 0){
                    nearestLineToCarUpper = lineVector[i];
                }
                //Wenn der x Wert des startpunktes größer ist, ist die linie näher am auto
                else if(nearestLineToCarUpper[0] < lineVector[i][0]){
                    nearestLineToCarUpper = lineVector[i];
                }
            }
        }
    }
    else if(cString::IsEqual(lanePosition, "left")){
        for( unsigned int i = 0; i < lineVector.size(); i++){
            if(lineVector[i][1] > m_filterProperties.LeftUpperLeftY+yDiffLeft/2){
                //wenn es noch keine linie gibt setzte aktuelle linie
                if(nearestLineToCarLower[0] == 0){
                    nearestLineToCarLower = lineVector[i];
                }
                //Wenn der x Wert des startpunktes größer ist, ist die linie näher am auto
                else if(nearestLineToCarLower[0] < lineVector[i][0]){
                    nearestLineToCarLower = lineVector[i];
                }
            }else{
                //wenn es noch keine linie gibt setzte aktuelle linie
                if(nearestLineToCarUpper[0] == 0){
                    nearestLineToCarUpper = lineVector[i];
                }
                //Wenn der x Wert des startpunktes größer ist, ist die linie näher am auto
                else if(nearestLineToCarUpper[0] < lineVector[i][0]){
                    nearestLineToCarUpper = lineVector[i];
                }
            }
        }

    }
    if(nearestLineToCarLower[0] == 0){
        return nearestLineToCarUpper;
    }
    else{
        return nearestLineToCarLower;
    }
}


tResult cLaneDetection::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormat = (*pFormat);
        //LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
    }
    RETURN_NOERROR;
}

tFloat32 cLaneDetection::computeSteeringAngle(Vec4i leftLane, Vec4i middleLane, Vec4i rightLane, Mat &outputImage){
    int bufferSize = 3;

    int rightX = 0;
    int middleX = 0;
    int leftX = 0;
    int laneXPosSumRight = 0;
    int laneXPosSumMiddle = 0;
    int laneXPosSumLeft = 0;
    tFloat32 laneAngleSumRight = 0;
    tFloat32 laneAngleSumMiddle = 0;
    tFloat32 laneAngleSumLeft = 0;

    tFloat32 laneAngleMiddle = computeRadianFromVec4i(middleLane);
    tFloat32 laneAngleRight = computeRadianFromVec4i(rightLane);
    tFloat32 laneAngleLeft = computeRadianFromVec4i(leftLane);

    tFloat32 laneAngleDif = 0;

    //neu
    float xPosDeviation = 0;
    float angleDeviation = 0;

    //Durchschnitt der letzten *bufferSize* X Koordinaten und Winkel der Fahrbahnlinien bilden um Schwankungen zu vermindern
    if(walkThroughCounter_steeringAngle <= bufferSize){
        laneBufferRight.insert(laneBufferRight.begin(), rightLane[0]);
        laneBufferMiddle.insert(laneBufferMiddle.begin(), middleLane[0]);
        laneBufferLeft.insert(laneBufferLeft.begin(), leftLane[0]);

        angleBufferRight.insert(angleBufferRight.begin(), laneAngleRight);
        angleBufferMiddle.insert(angleBufferMiddle.begin(), laneAngleMiddle);
        angleBufferLeft.insert(angleBufferLeft.begin(), laneAngleLeft);

        walkThroughCounter_steeringAngle++;
    }
    else{
        laneBufferRight.insert(laneBufferRight.begin(), rightLane[0]);
        laneBufferMiddle.insert(laneBufferMiddle.begin(), middleLane[0]);
        laneBufferLeft.insert(laneBufferLeft.begin(), leftLane[0]);
        laneBufferRight.pop_back();
        laneBufferMiddle.pop_back();
        laneBufferLeft.pop_back();

        angleBufferRight.insert(angleBufferRight.begin(), laneAngleRight);
        angleBufferMiddle.insert(angleBufferMiddle.begin(), laneAngleMiddle);
        angleBufferLeft.insert(angleBufferLeft.begin(), laneAngleLeft);
        angleBufferRight.pop_back();
        angleBufferMiddle.pop_back();
        angleBufferLeft.pop_back();

        for(int i=0; i< bufferSize; i++){
            laneXPosSumRight += laneBufferRight[i];
            laneXPosSumMiddle += laneBufferMiddle[i];
            laneXPosSumLeft += laneBufferLeft[i];

            laneAngleSumRight += angleBufferRight[i];
            laneAngleSumMiddle += angleBufferMiddle[i];
            laneAngleSumLeft += angleBufferLeft[i];
        }
        rightLane[0] = laneXPosSumRight/bufferSize;
        middleLane[0] = laneXPosSumMiddle/bufferSize;
        leftLane[0] = laneXPosSumLeft/bufferSize;

        laneAngleRight = laneAngleSumRight/bufferSize;
        laneAngleMiddle = laneAngleSumMiddle/bufferSize;
        laneAngleLeft = laneAngleSumLeft/bufferSize;
    }

    if(((computeAngleFromVec4i(rightLane)+computeAngleFromVec4i(middleLane)+computeAngleFromVec4i(leftLane))/3 > -20.0)
            && ((computeAngleFromVec4i(rightLane)+computeAngleFromVec4i(middleLane)+computeAngleFromVec4i(leftLane))/3 < 20.0)){
        //LOG_INFO("GERADE");
        if(faultyLineCounterMiddle == -1){
            angleDeviation = computeAngleFromVec4i(middleLane);
            laneAngleDif = -laneAngleMiddle;
            middleX = middleXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
            xPosDeviation = middleLane[0] - middleX;
        }
        else if(faultyLineCounterRight == -1){
            angleDeviation = computeAngleFromVec4i(rightLane);

            laneAngleDif = -laneAngleRight;
            rightX = rightXPos - sin(laneAngleDif) * m_filterProperties.FovCurveLeft;
            xPosDeviation = rightLane[0] - rightX;
        }
        else{
            angleDeviation = computeAngleFromVec4i(leftLane);

            laneAngleDif = -laneAngleLeft;
            leftX = leftXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
            xPosDeviation = leftLane[0] - leftX;
        }

    }
    else if((computeAngleFromVec4i(rightLane)+computeAngleFromVec4i(middleLane)+computeAngleFromVec4i(leftLane))/3 <= -20.0){
        //LOG_INFO("LINKS");

        if(faultyLineCounterRight == -1){
            angleDeviation = computeAngleFromVec4i(rightLane);

            laneAngleDif = -laneAngleRight;
            rightX = rightXPos - sin(laneAngleDif) * m_filterProperties.FovCurveLeft;
            xPosDeviation = rightLane[0] - rightX;
        }
        else if(faultyLineCounterMiddle == -1){
            angleDeviation = computeAngleFromVec4i(middleLane);

            laneAngleDif = -laneAngleMiddle;
            middleX = middleXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
            xPosDeviation = middleLane[0] - middleX;
        }
        else{
            angleDeviation = computeAngleFromVec4i(leftLane);

            laneAngleDif = -laneAngleLeft;
            leftX = leftXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
            xPosDeviation = leftLane[0] - leftX;
        }
    }
    else{
        //LOG_INFO("RECHTS");
        if(faultyLineCounterLeft == -1){
            angleDeviation = computeAngleFromVec4i(leftLane);

            laneAngleDif = -laneAngleLeft;
            leftX = leftXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
            xPosDeviation = leftLane[0] - leftX;
        }
        else if(faultyLineCounterMiddle == -1){
            angleDeviation = computeAngleFromVec4i(middleLane);

            laneAngleDif = -laneAngleMiddle;
            middleX = middleXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
            xPosDeviation = middleLane[0] - middleX;
        }
        else{
            angleDeviation = computeAngleFromVec4i(rightLane);

            laneAngleDif = -laneAngleRight;
            rightX = rightXPos - sin(laneAngleDif) * m_filterProperties.FovCurveLeft;
            xPosDeviation = rightLane[0] - rightX;
        }
    }

    //    tUInt64 sampleTimeInt = cSystem::GetTime() - oldTime;
    //    float samleTimeSec = sampleTimeInt/1000000.0;
    //    if(samleTimeSec == 0)
    //        samleTimeSec = 1;
    int ticksSinceLastFrame = (wheelCountLeft+wheelCountRight)/2 - oldTicks;

    if(carSpeed == 0){
        xPosDeviationSum = 0;
    }

    xPosDeviationSum += xPosDeviation*ticksSinceLastFrame;
    float xPosDerivation = (xPosDeviation-oldxPosDeviation)/ticksSinceLastFrame;

    float steeringAngle = m_filterProperties.K1*angleDeviation
            +m_filterProperties.K2*xPosDeviation
            +m_filterProperties.K3*xPosDerivation
            +m_filterProperties.K4*xPosDeviationSum;

    oldxPosDeviation = xPosDeviation;
    oldTicks = ticksSinceLastFrame;
    //    oldTime = cSystem::GetTime();

    if(steeringAngle != steeringAngle || steeringAngle > 10000 || steeringAngle < -10000){
        steeringAngle = 0;
        //LOG_INFO("Steeringangle out of range!!!");
    }
    lastSteeringangle = steeringAngle;

    return steeringAngle;
}

tResult cLaneDetection::GetSpeed(IMediaSample* pMediaSample)
{
    tFloat32 speed;
    //sobald der block verlassen wird, wird das lock aufgehoben
    {
        //read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdInputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdInputspeedControllerValue);
            //pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputspeedControllerTimeStamp);
            m_szIdInputSpeedSet = tTrue;
        }

        pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speed);
        //pCoderInput->Get(m_szIdInputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    carSpeed = speed;
    RETURN_NOERROR;
}

tResult cLaneDetection::adaptROI(Vec4i &lineRight, Vec4i &lineMiddle, Vec4i &lineLeft){

    tFloat32 laneAngleMiddle = computeRadianFromVec4i(lineMiddle);
    tFloat32 laneAngleRight = computeRadianFromVec4i(lineRight);
    tFloat32 laneAngleLeft = computeRadianFromVec4i(lineLeft);

    //TODO Default Werte aus File holen

    if((wheelCountLeft+wheelCountRight)/2 - faultyLineCounterRight > m_filterProperties.faultyCounterThreshold && faultyLineCounterRight != -1 || lineRight == Vec4i(0,0,0,0)){ // TODO Offsets ändern
        if(faultyLineCounterMiddle == -1){
            m_filterProperties.RightUpperLeftX = 630 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
            m_filterProperties.RightUpperRightX = 710 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
            m_filterProperties.RightLowerLeftX = 630 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
            m_filterProperties.RightLowerRightX = 710 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
        }else if(faultyLineCounterLeft == -1){
            m_filterProperties.RightUpperLeftX = 630 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
            m_filterProperties.RightUpperRightX = 710 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
            m_filterProperties.RightLowerLeftX = 630 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
            m_filterProperties.RightLowerRightX = 710 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
        }
        else{
            m_filterProperties.RightUpperLeftX = 630;
            m_filterProperties.RightUpperRightX = 710;
            m_filterProperties.RightLowerLeftX = 630;
            m_filterProperties.RightLowerRightX = 710;
        }
    }
    else{
        m_filterProperties.RightUpperLeftX = 630 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
        m_filterProperties.RightUpperRightX = 710 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
        m_filterProperties.RightLowerLeftX = 630 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
        m_filterProperties.RightLowerRightX = 710 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
    }

    if((wheelCountLeft+wheelCountRight)/2 - faultyLineCounterLeft > m_filterProperties.faultyCounterThreshold && faultyLineCounterLeft != -1 || lineLeft == Vec4i(0,0,0,0)){
        if(faultyLineCounterRight == -1){
            m_filterProperties.LeftUpperLeftX = 420 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
            m_filterProperties.LeftUpperRightX = 500 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
            m_filterProperties.LeftLowerLeftX = 420 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
            m_filterProperties.LeftLowerRightX = 500 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
        }else if(faultyLineCounterMiddle == -1){
            m_filterProperties.LeftUpperLeftX = 420 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
            m_filterProperties.LeftUpperRightX = 500 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
            m_filterProperties.LeftLowerLeftX = 420 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
            m_filterProperties.LeftLowerRightX = 500 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
        }else
        {
            m_filterProperties.LeftUpperLeftX = 420;
            m_filterProperties.LeftUpperRightX = 500;
            m_filterProperties.LeftLowerLeftX = 420;
            m_filterProperties.LeftLowerRightX = 500;
        }
    }
    else{
        m_filterProperties.LeftUpperLeftX = 420 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;;
        m_filterProperties.LeftUpperRightX = 500 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;;
        m_filterProperties.LeftLowerLeftX = 420 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
        m_filterProperties.LeftLowerRightX = 500 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
    }
    if((wheelCountLeft+wheelCountRight)/2 - faultyLineCounterMiddle > m_filterProperties.faultyCounterThreshold && faultyLineCounterMiddle != -1 || lineMiddle == Vec4i(0,0,0,0)){
        if(faultyLineCounterRight == -1){
            m_filterProperties.MidUpperLeftX = 550 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
            m_filterProperties.MidUpperRightX = 610 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
            m_filterProperties.MidLowerLeftX = 550 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
            m_filterProperties.MidLowerRightX = 610 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
        }else if(faultyLineCounterLeft == -1){
            m_filterProperties.MidUpperLeftX = 550 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
            m_filterProperties.MidUpperRightX = 610 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
            m_filterProperties.MidLowerLeftX = 550 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
            m_filterProperties.MidLowerRightX = 610 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
        }else
        {
            m_filterProperties.MidUpperLeftX = 550;
            m_filterProperties.MidUpperRightX = 610;
            m_filterProperties.MidLowerLeftX = 550;
            m_filterProperties.MidLowerRightX = 610;
        }
    }
    else{
        m_filterProperties.MidUpperLeftX = 550 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;;
        m_filterProperties.MidUpperRightX = 610 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;;
        m_filterProperties.MidLowerLeftX = 550 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
        m_filterProperties.MidLowerRightX = 610 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
    }

    RETURN_NOERROR;
}


tResult cLaneDetection::transmitSteeringAngle(tFloat32 SteeringAngle){
    //init mediasample
    cObjectPtr<IMediaSample> pMediaSample;
    //allocate memory to mediasample
    AllocMediaSample((tVoid**)&pMediaSample);

    //create interaction with ddl
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionSteeringAngle->GetMediaSampleSerializer(&pSerializer);

    //allocate buffer to write in mediasample
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    {
        //write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionSteeringAngle, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdOutputSteeringAngleSet)
        {
            pCoderInput->GetID("f32Value", m_szIdSteeringAngleValue);
            //pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdSteeringAngleTimestamp);
            m_szIdOutputSteeringAngleSet = tTrue;
        }
        pCoderInput->Set(m_szIdSteeringAngleValue, (tVoid*)&SteeringAngle);
    }

    //pMediaSample->SetTime(_clock->GetStreamTime());
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oSteeringAngleOutputPin.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cLaneDetection::transmitManeuverFinished(tInt16 maneuverID){
    //init mediasample
    cObjectPtr<IMediaSample> pMediaSample;
    //allocate memory to mediasample
    AllocMediaSample((tVoid**)&pMediaSample);

    //create interaction with ddl
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionManeuverFinished->GetMediaSampleSerializer(&pSerializer);

    //allocate buffer to write in mediasample
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static tBufferID szIDmaneuverID;
    static tBufferID szIDFinishedFlag;
    bool finishedFlag = tTrue;

    {
        //write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionManeuverFinished, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdOutputManeuverFinishedSet)
        {
            pCoderInput->GetID("i16ManeuverValue", szIDmaneuverID);
            pCoderInput->GetID("bFinishedFlag", szIDFinishedFlag);
            m_szIdOutputManeuverFinishedSet = tTrue;
        }
        pCoderInput->Set(szIDmaneuverID, (tVoid*)&maneuverID);
        pCoderInput->Set(szIDFinishedFlag, (tVoid*)&finishedFlag);
    }

    //pMediaSample->SetTime(_clock->GetStreamTime());
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oManeuverFinishedOutputPin.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cLaneDetection::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
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

tResult cLaneDetection::TransmitParkingspaces(cOutputPin* oPin, vector<bool> parkingSpaces)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitBool);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionParkingspaces->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szParkingSpace1;
    static tBufferID szParkingSpace2;
    static tBufferID szParkingSpace3;
    static tBufferID szParkingSpace4;

    bool ps[4];
    ps[0] = parkingSpaces[0];
    ps[1] = parkingSpaces[1];
    ps[2] = parkingSpaces[2];
    ps[3] = parkingSpaces[3];

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionParkingspaces, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("parkingSpace1", szParkingSpace1);
            pCoderOutput->GetID("parkingSpace2", szParkingSpace2);
            pCoderOutput->GetID("parkingSpace3", szParkingSpace3);
            pCoderOutput->GetID("parkingSpace4", szParkingSpace4);
            hasID = tTrue;
        }

        pCoderOutput->Set(szParkingSpace1, (tVoid*)&ps[0]);
        pCoderOutput->Set(szParkingSpace2, (tVoid*)&ps[1]);
        pCoderOutput->Set(szParkingSpace3, (tVoid*)&ps[2]);
        pCoderOutput->Set(szParkingSpace4, (tVoid*)&ps[3]);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cLaneDetection::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormat);

        //LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
    }
    RETURN_NOERROR;
}

tResult cLaneDetection::TransmitTicksToCrosspoint(int ticksToCrosspoint)
{
    //LOG_INFO(cString::Format("%d ticks to Crosspoint transmitted! ", ticksToCrosspoint));
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionTicksToLine->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDTicksToLine;
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionTicksToLine, pMediaSample, pCoderOutput);

        if (!hasID)
        {
            pCoderOutput->GetID("i32ticksToLine", szIDTicksToLine);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDTicksToLine, (tVoid*)&ticksToCrosspoint);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    m_oOutputTicksToLine.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cLaneDetection::SwitchToLaneLeft()
{

    //LOG_INFO("SWITCH TO LANE LEFT!");
    vector<maneuver> execs;

    maneuver a;
    a.ticks = 5;
    a.steeringAngle = 0;
    a.speed = -8;
    a.turnLeft = false;
    a.turnRight = false;
    execs.push_back(a);

    maneuver c;
    c.ticks = 80;
    c.steeringAngle = 100.0f;
    c.speed = -8;
    c.turnLeft = false;
    c.turnRight = false;
    execs.push_back(c);

//    maneuver i;
//    i.ticks = 40;
//    i.steeringAngle = 0.0f;
//    i.speed = -8;
//    i.turnLeft = true;
//    i.turnRight = false;
//    execs.push_back(i);

    maneuver h;
    h.ticks = 100;
    h.steeringAngle = -100.0f;
    h.speed = -8;
    h.turnLeft = true;
    h.turnRight = false;
    execs.push_back(h);

    setManeuvers(execs);
    isOnLeftLane = true;
    overtakingTicks = (wheelCountLeft + wheelCountRight)/2 + 400;

    RETURN_NOERROR;
}

//tBool cLaneDetection::Intersection(Vec4i Vec1, Vec4i Vec2){
//    //Point x = vec2Start - vec1Start;
//    Point d1 = Point(Vec1[2], Vec1[3]) - Point(Vec1[0], Vec1[1]);
//    Point d2 = Point(Vec2[2], Vec2[3]) - Point(Vec2[0], Vec2[1]);

//    int cross = d1.x*d2.y - d1.y*d2.x;
//    float s,t;
//    s = (-d1.y*())

//    if(abs(cross) < 0)
//        return false;
//    else
//        return true;
//}

tBool cLaneDetection::Intersection(Vec4i Vec1, Vec4i Vec2){
    float p0_x = (float)Vec1[0];
    float p0_y = (float)Vec1[1];
    float p1_x = (float)Vec1[2];
    float p1_y = (float)Vec1[3];
    float p2_x = (float)Vec2[0];
    float p2_y = (float)Vec2[1];
    float p3_x = (float)Vec2[2];
    float p3_y = (float)Vec2[3];

    float s1_x,s1_y,s2_x,s2_y;
    s1_x = p1_x - p0_x;
    s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;
    s2_y = p3_y - p2_y;

    float s,t;
    s = (-s1_y*(p0_x-p2_x)+s1_x*(p0_y-p2_y))/(-s2_x*s1_y+s1_x*s2_y);
    t = (s2_x*(p0_y-p2_y)-s2_y*(p0_x-p2_x))/(-s2_x*s1_y+s1_x*s2_y);

    if((s >= 0.0f) && (s <= 1.0f) && (t >= 0.0f) && (t <= 1.0f))
        return true;
    else
        return false;
}

tBool cLaneDetection::IntersectionUL(Vec4i line, int upperX,int lowerX,int upperY,int lowerY){
       if(line[1] >= upperY && line[3] <= lowerY){
           if(line[0] <= upperX && line[2] >= lowerX ||
              line[2] <= upperX && line[0] >= lowerX){
               return true;
           }
       }
       return false;
}


tResult cLaneDetection::SwitchToLaneRight()
{
    //LOG_INFO("SWITCH TO LANE RIGHT!");
    vector<maneuver> execs;

    maneuver a;
    a.ticks = 5;
    a.steeringAngle = 0;
    a.speed = -8;
    a.turnLeft = false;
    a.turnRight = false;
    execs.push_back(a);

    maneuver c;
    c.ticks = 90;
    c.steeringAngle = -100.0f;
    c.speed = -10;
    c.turnLeft = false;
    c.turnRight = false;
    execs.push_back(c);

    maneuver h;
    h.ticks = 110;
    h.steeringAngle = 70.0f;
    h.speed = -8;
    h.turnLeft = false;
    h.turnRight = true;
    execs.push_back(h);

    setManeuvers(execs);
    isOnLeftLane = false;
    RETURN_NOERROR;
}
