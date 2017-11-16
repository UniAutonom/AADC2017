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
* $Author:: spiesra $  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/
#include "stdafx.h"
#include "cImageAdapter.h"

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   cImageAdapter)


cImageAdapter::cImageAdapter(const tChar* __info) : cFilter(__info)
{
    SetPropertyInt("ImageAdaptionLaneDetection::lowerLeftX_laneD", 0);
    SetPropertyStr("ImageAdaptionLaneDetection::lowerLeftX_laneD" NSSUBPROP_DESCRIPTION, "X Coordinate of the lower Left point of the lane detection ROI");
    SetPropertyBool("ImageAdaptionLaneDetection::lowerLeftX_laneD" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionLaneDetection::lowerLeftY_laneD", 200);
    SetPropertyStr("ImageAdaptionLaneDetection::lowerLeftY_laneD" NSSUBPROP_DESCRIPTION, "Y Coordinate of the lower Left point of the lane detection ROI");
    SetPropertyBool("ImageAdaptionLaneDetection::lowerLeftY_laneD" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionLaneDetection::upperRightX_laneD", 200);
    SetPropertyStr("ImageAdaptionLaneDetection::upperRightX_laneD" NSSUBPROP_DESCRIPTION, "X Coordinate of the upper right point of the lane detection ROI");
    SetPropertyBool("ImageAdaptionLaneDetection::upperRightX_laneD" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionLaneDetection::upperRightY_laneD", 200);
    SetPropertyStr("ImageAdaptionLaneDetection::upperRightY_laneD" NSSUBPROP_DESCRIPTION, "Y Coordinate of the upper right point of the lane detection ROI");
    SetPropertyBool("ImageAdaptionLaneDetection::upperRightY_laneD" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("ImageAdaptionMarkerDetection::lowerLeftX_markerD", 0);
    SetPropertyStr("ImageAdaptionMarkerDetection::lowerLeftX_markerD" NSSUBPROP_DESCRIPTION, "X Coordinate of the lower left point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionMarkerDetection::lowerLeftX_markerD" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionMarkerDetection::lowerLeftY_markerD", 200);
    SetPropertyStr("ImageAdaptionMarkerDetection::lowerLeftY_markerD" NSSUBPROP_DESCRIPTION, "Y Coordinate of the lower left point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionMarkerDetection::lowerLeftY_markerD" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionMarkerDetection::upperRightX_markerD", 200);
    SetPropertyStr("ImageAdaptionMarkerDetection::upperRightX_markerD" NSSUBPROP_DESCRIPTION, "X Coordinate of the upper right point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionMarkerDetection::upperRightX_markerD" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionMarkerDetection::upperRightY_markerD", 200);
    SetPropertyStr("ImageAdaptionMarkerDetection::upperRightY_markerD" NSSUBPROP_DESCRIPTION, "Y Coordinate of the upper right point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionMarkerDetection::upperRightY_markerD" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt("ImageAdaptionNeuralNet::lowerLeftX_neuralN", 0);
    SetPropertyStr("ImageAdaptionNeuralNet::lowerLeftX_neuralN" NSSUBPROP_DESCRIPTION, "X Coordinate of the lower left point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionNeuralNet::lowerLeftX_neuralN" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionNeuralNet::lowerLeftY_neuralN", 200);
    SetPropertyStr("ImageAdaptionNeuralNet::lowerLeftY_neuralN" NSSUBPROP_DESCRIPTION, "Y Coordinate of the lower left point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionNeuralNet::lowerLeftY_neuralN" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionNeuralNet::upperRightX_neuralN", 200);
    SetPropertyStr("ImageAdaptionNeuralNet::upperRightX_neuralN" NSSUBPROP_DESCRIPTION, "X Coordinate of the upper right point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionNeuralNet::upperRightX_neuralN" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyInt("ImageAdaptionNeuralNet::upperRightY_neuralN", 200);
    SetPropertyStr("ImageAdaptionNeuralNet::upperRightY_neuralN" NSSUBPROP_DESCRIPTION, "Y Coordinate of the upper right point of the marker detection ROI");
    SetPropertyBool("ImageAdaptionNeuralNet::upperRightY_neuralN" NSSUBPROP_ISCHANGEABLE, tTrue);
}

cImageAdapter::~cImageAdapter()
{
}

tResult cImageAdapter::PropertyChanged(const tChar* strName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    //associate the properties to the member
    if (cString::IsEqual(strName, "ImageAdaptionLaneDetection::lowerLeftX_laneD"))
        m_filterProperties.lowerLeftX_laneD = GetPropertyInt("ImageAdaptionLaneDetection::lowerLeftX_laneD");
    else if (cString::IsEqual(strName, "ImageAdaptionLaneDetection::lowerLeftY_laneD"))
        m_filterProperties.lowerLeftY_laneD = GetPropertyInt("ImageAdaptionLaneDetection::lowerLeftY_laneD");
    else if (cString::IsEqual(strName, "ImageAdaptionLaneDetection::upperRightX_laneD"))
        m_filterProperties.upperRightX_laneD = GetPropertyInt("ImageAdaptionLaneDetection::upperRightX_laneD");
    else if (cString::IsEqual(strName, "ImageAdaptionLaneDetection::upperRightY_laneD"))
        m_filterProperties.upperRightY_laneD = GetPropertyInt("ImageAdaptionLaneDetection::upperRightY_laneD");
    else if (cString::IsEqual(strName, "ImageAdaptionMarkerDetection::lowerLeftX_markerD"))
        m_filterProperties.lowerLeftX_markerD = GetPropertyInt("ImageAdaptionMarkerDetection::lowerLeftX_markerD");
    else if (cString::IsEqual(strName, "ImageAdaptionMarkerDetection::lowerLeftY_markerD"))
        m_filterProperties.lowerLeftY_markerD = GetPropertyInt("ImageAdaptionMarkerDetection::lowerLeftY_markerD");
    else if (cString::IsEqual(strName, "ImageAdaptionMarkerDetection::upperRightX_markerD"))
        m_filterProperties.upperRightX_markerD = GetPropertyInt("ImageAdaptionMarkerDetection::upperRightX_markerD");
    else if (cString::IsEqual(strName, "ImageAdaptionMarkerDetection::upperRightY_markerD"))
        m_filterProperties.upperRightY_markerD = GetPropertyInt("ImageAdaptionMarkerDetection::upperRightY_markerD");
    else if (cString::IsEqual(strName, "ImageAdaptionNeuralNet::lowerLeftX_neuralN"))
        m_filterProperties.lowerLeftX_neuralN = GetPropertyInt("ImageAdaptionNeuralNet::lowerLeftX_neuralN");
    else if (cString::IsEqual(strName, "ImageAdaptionNeuralNet::lowerLeftY_neuralN"))
        m_filterProperties.lowerLeftY_neuralN = GetPropertyInt("ImageAdaptionNeuralNet::lowerLeftY_neuralN");
    else if (cString::IsEqual(strName, "ImageAdaptionNeuralNet::upperRightX_neuralN"))
        m_filterProperties.upperRightX_neuralN = GetPropertyInt("ImageAdaptionNeuralNet::upperRightX_neuralN");
    else if (cString::IsEqual(strName, "ImageAdaptionNeuralNet::upperRightY_neuralN"))
        m_filterProperties.upperRightY_neuralN = GetPropertyInt("ImageAdaptionNeuralNet::upperRightY_neuralN");

    RETURN_NOERROR;
}


tResult cImageAdapter::Start(__exception)
{

    return cFilter::Start(__exception_ptr);
}

tResult cImageAdapter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}
tResult cImageAdapter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPinBasler.Create("Video_Input_Basler", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPinBasler));
//        RETURN_IF_FAILED(m_oVideoInputPinRealsense.Create("Video_Input_Realsense", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
//        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPinRealsense));


        // Video Input
//        RETURN_IF_FAILED(m_oVideoOutputPinLaneDetection.Create("Video_Output_LaneDetection", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
//        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPinLaneDetection));
        RETURN_IF_FAILED(m_oVideoOutputPinMarkerDetection.Create("Video_Output_MarkerDetection", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPinMarkerDetection));
//        RETURN_IF_FAILED(m_oVideoOutputPinNeuralNet.Create("Video_Output_NeuralNet", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
//        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPinNeuralNet));

    }
    else if (eStage == StageNormal)
    {
    }

    else if (eStage == StageGraphReady)
    {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pTypeBasler;
        RETURN_IF_FAILED(m_oVideoInputPinBasler.GetMediaType(&pTypeBasler));
//        cObjectPtr<IMediaType> pTypeRealsense;
//        RETURN_IF_FAILED(m_oVideoInputPinRealsense.GetMediaType(&pTypeRealsense));

        cObjectPtr<IMediaTypeVideo> pTypeVideoBasler;
        RETURN_IF_FAILED(pTypeBasler->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideoBasler));
//        cObjectPtr<IMediaTypeVideo> pTypeVideoRealsense;
//        RETURN_IF_FAILED(pTypeRealsense->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideoRealsense));

        // set the image format of the input video pin
        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormatBasler(pTypeVideoBasler->GetFormat())))
        {
            LOG_ERROR("Invalid Basler Input Format for this filter");
        }
//        if (IS_FAILED(UpdateInputImageFormatRealsense(pTypeVideoRealsense->GetFormat())))
//        {
//            LOG_ERROR("Invalid Realsense Input Format for this filter");
//        }
    }

    RETURN_NOERROR;
}



tResult cImageAdapter::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageGraphReady)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cImageAdapter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if (pSource == &m_oVideoInputPinBasler)
        {
            //check if video format is still unkown
            if (m_sInputFormatBasler.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormatBasler(m_oVideoInputPinBasler.GetFormat()));
            }

            ProcessVideoBasler(pMediaSample);
        }
        else if (pSource == &m_oVideoInputPinRealsense)
        {
            //check if video format is still unkown
            if (m_sInputFormatRealsense.nPixelFormat == IImage::PF_UNKNOWN)
            {
                RETURN_IF_FAILED(UpdateInputImageFormatRealsense(m_oVideoInputPinRealsense.GetFormat()));
            }

            ProcessVideoRealsense(pMediaSample);
        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_oVideoInputPinBasler)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormatBasler(m_oVideoInputPinBasler.GetFormat()));
        }
        else if (pSource == &m_oVideoInputPinRealsense)
        {
            //the input format was changed, so the imageformat has to changed in this filter also
            RETURN_IF_FAILED(UpdateInputImageFormatRealsense(m_oVideoInputPinRealsense.GetFormat()));
        }
    }
    RETURN_NOERROR;
}

tResult cImageAdapter::ProcessVideoBasler(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    cv::Mat outputImageMarkerDetection;
    const tVoid* l_pSrcBuffer;

    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImageBasler.total() * m_inputImageBasler.elemSize()) == m_sInputFormatBasler.nSize)
        {
            m_inputImageBasler.data = (uchar*)(l_pSrcBuffer);
            outputImageMarkerDetection = m_inputImageBasler.clone();
        }
        pSample->Unlock(l_pSrcBuffer);
    }


    if (!outputImageMarkerDetection.empty())
    {
        Mat outputImageMarkerDetectionOut;
        rectangle(outputImageMarkerDetection, Point(m_filterProperties.lowerLeftX_markerD, m_filterProperties.lowerLeftY_markerD), Point(m_filterProperties.upperRightX_markerD, m_filterProperties.upperRightY_markerD), 255, 3);
        //Bild Ausschneiden
        Rect ROI_MD(Point(m_filterProperties.lowerLeftX_markerD, m_filterProperties.lowerLeftY_markerD), Point(m_filterProperties.upperRightX_markerD, m_filterProperties.upperRightY_markerD));

        outputImageMarkerDetectionOut = outputImageMarkerDetection(ROI_MD);



        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImageMarkerDetection;
        newImageMarkerDetection.Create(m_sOutputFormatMarkerDetection.nWidth, m_sOutputFormatMarkerDetection.nHeight, m_sOutputFormatMarkerDetection.nBitsPerPixel, m_sOutputFormatMarkerDetection.nBytesPerLine, outputImageMarkerDetection.data);

        UpdateOutputImageFormatMarkerDetection(outputImageMarkerDetectionOut);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImageMarkerDetection.GetBitmap(), newImageMarkerDetection.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPinMarkerDetection.Transmit(pMediaSample));

        outputImageMarkerDetection.release();
    }

    RETURN_NOERROR;
}

tResult cImageAdapter::ProcessVideoRealsense(IMediaSample* pSample)
{

    RETURN_IF_POINTER_NULL(pSample);
    // new image for result
    cv::Mat outputImageMarkerDetectionTmp;
    cv::Mat outputImageMarkerDetection;
    cv::Mat outputImageNeuralNetTmp;
    cv::Mat outputImageNeuralNet;
    const tVoid* l_pSrcBuffer;

    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat, be sure to select the right pixelformat
        if (tInt32(m_inputImageRealsense.total() * m_inputImageRealsense.elemSize()) == m_sInputFormatRealsense.nSize)
        {
            m_inputImageRealsense.data = (uchar*)(l_pSrcBuffer);
            m_inputImageRealsense.copyTo(outputImageMarkerDetectionTmp);
        }
        pSample->Unlock(l_pSrcBuffer);
    }

    outputImageMarkerDetectionTmp.copyTo(outputImageNeuralNetTmp);



    if (!outputImageNeuralNetTmp.empty())
    {
        rectangle(outputImageNeuralNetTmp, Point(m_filterProperties.lowerLeftX_neuralN, m_filterProperties.lowerLeftY_neuralN), Point(m_filterProperties.upperRightX_neuralN, m_filterProperties.upperRightY_neuralN), 255, 3);
        //Bild Ausschneiden
        Rect ROI_NN(Point(m_filterProperties.lowerLeftX_neuralN, m_filterProperties.lowerLeftY_neuralN), Point(m_filterProperties.upperRightX_neuralN, m_filterProperties.upperRightY_neuralN));
        outputImageNeuralNet = outputImageNeuralNetTmp(ROI_NN);
        UpdateOutputImageFormatNeuralNet(outputImageNeuralNet);

        //create a cImage from CV Matrix (not necessary, just for demonstration9
        cImage newImageNeuralNetwork;
        newImageNeuralNetwork.Create(m_sOutputFormatNeuralNet.nWidth, m_sOutputFormatNeuralNet.nHeight, m_sOutputFormatNeuralNet.nBitsPerPixel, m_sOutputFormatNeuralNet.nBytesPerLine, outputImageNeuralNet.data);

        //create the new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
        //updating media sample
        RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImageNeuralNetwork.GetBitmap(), newImageNeuralNetwork.GetSize(), IMediaSample::MSF_None));
        //transmitting
        RETURN_IF_FAILED(m_oVideoOutputPinNeuralNet.Transmit(pMediaSample));

        outputImageNeuralNetTmp.release();
    }

    RETURN_NOERROR;
}

tResult cImageAdapter::UpdateInputImageFormatBasler(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormatBasler = (*pFormat);
        LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormatBasler.nWidth, m_sInputFormatBasler.nHeight, m_sInputFormatBasler.nBytesPerLine, m_sInputFormatBasler.nSize, m_sInputFormatBasler.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormatBasler, m_inputImageBasler));
    }
    RETURN_NOERROR;
}

tResult cImageAdapter::UpdateInputImageFormatRealsense(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        //update member variable
        m_sInputFormatRealsense = (*pFormat);
        LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormatRealsense.nWidth, m_sInputFormatRealsense.nHeight, m_sInputFormatRealsense.nBytesPerLine, m_sInputFormatRealsense.nSize, m_sInputFormatRealsense.nPixelFormat));
        //create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormatRealsense, m_inputImageRealsense));
    }
    RETURN_NOERROR;
}

tResult cImageAdapter::UpdateOutputImageFormatLaneDetection(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormatLaneDetection.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormatLaneDetection);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatLaneDetection.nWidth, m_sOutputFormatLaneDetection.nHeight, m_sOutputFormatLaneDetection.nBytesPerLine, m_sOutputFormatLaneDetection.nSize, m_sOutputFormatLaneDetection.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPinLaneDetection.SetFormat(&m_sOutputFormatLaneDetection, NULL);
    }
    RETURN_NOERROR;
}

tResult cImageAdapter::UpdateOutputImageFormatMarkerDetection(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormatMarkerDetection.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormatMarkerDetection);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatMarkerDetection.nWidth, m_sOutputFormatMarkerDetection.nHeight, m_sOutputFormatMarkerDetection.nBytesPerLine, m_sOutputFormatMarkerDetection.nSize, m_sOutputFormatMarkerDetection.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPinMarkerDetection.SetFormat(&m_sOutputFormatMarkerDetection, NULL);
    }
    RETURN_NOERROR;
}

tResult cImageAdapter::UpdateOutputImageFormatNeuralNet(const cv::Mat& outputImage)
{
    //check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormatNeuralNet.nSize)
    {
        Mat2BmpFormat(outputImage, m_sOutputFormatNeuralNet);

        LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormatMarkerDetection.nWidth, m_sOutputFormatMarkerDetection.nHeight, m_sOutputFormatMarkerDetection.nBytesPerLine, m_sOutputFormatMarkerDetection.nSize, m_sOutputFormatMarkerDetection.nPixelFormat));
        //set output format for output pin
        m_oVideoOutputPinNeuralNet.SetFormat(&m_sOutputFormatNeuralNet, NULL);
    }
    RETURN_NOERROR;
}
