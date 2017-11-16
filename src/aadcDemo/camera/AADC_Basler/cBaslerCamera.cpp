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
* $Author:: spie#$  $Date:: 2017-05-22 15:15:31#$ $Rev:: 63721   $
**********************************************************************/

#include "cBaslerCamera.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cBaslerCamera)

cBaslerCamera::cBaslerCamera(const tChar* __info) : cFilter(__info)
{
    SetPropertyInt("Stream Width", 1280);
    SetPropertyStr("Stream Width" NSSUBPROP_DESCRIPTION, "Width of the Stream");

    SetPropertyInt("Stream Height", 960);
    SetPropertyStr("Stream Height" NSSUBPROP_DESCRIPTION, "Height of the Stream");

    SetPropertyFloat("Brightness", 0.3);
    SetPropertyStr("Brightness" NSSUBPROP_DESCRIPTION, "Target Brightness for Auto Brightness of Camera");

    SetPropertyInt("ROI::XOffset", 440);
    SetPropertyStr("ROI::XOffset" NSSUBPROP_DESCRIPTION, "X Offset for Region of Interest Rectangular");

    SetPropertyInt("ROI::YOffset", 330);
    SetPropertyStr("ROI::YOffset" NSSUBPROP_DESCRIPTION, "Y Offset for Region of Interest Rectangular");

    SetPropertyInt("ROI::Width", 400);
    SetPropertyStr("ROI::Width" NSSUBPROP_DESCRIPTION, "Width of the Region of Interest Rectangular");

    SetPropertyInt("ROI::Height", 300);
    SetPropertyStr("ROI::Height" NSSUBPROP_DESCRIPTION, "Height of the Region of Interest Rectangular");

    SetPropertyBool("ROI::Show ROI", false);
    SetPropertyStr("ROI::Show ROI" NSSUBPROP_DESCRIPTION, "Shows ROI as a Rectangle in Video");
    SetPropertyBool("ROI::Show ROI" NSSUBPROP_ISCHANGEABLE, tTrue);
}

cBaslerCamera::~cBaslerCamera()
{
    //destroy device

}

tResult cBaslerCamera::Start(__exception)
{
    try
    {
        m_camera.StartGrabbing(GrabStrategy_LatestImageOnly);
    }
    catch (GenICam::GenericException &e)
    {
        THROW_ERROR_DESC(ERR_FAILED, cString::Format("Camera cannot start grabbing: %s", e.GetDescription()));
    }
    //starting the Thread
    if (m_Thread.GetState() != cKernelThread::TS_Running)
    {
        m_Thread.Run();
    }
    return cFilter::Start(__exception_ptr);
}

tResult cBaslerCamera::Stop(__exception)
{
    //suspend the thread
    if (m_Thread.GetState() == cKernelThread::TS_Running)
    {
        m_Thread.Suspend(tTrue);
    }
    // stops grabbing
    m_camera.StopGrabbing();

    return cFilter::Stop(__exception_ptr);
}

tResult cBaslerCamera::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if (eStage == StageNormal)
    {

        m_Thread.Terminate(tTrue);
        m_Thread.Release();

        if (m_camera.IsOpen())
        {
            //close the camera
            m_camera.Close();

            //detach device
            m_camera.DetachDevice();

            //destroy device
            m_camera.DestroyDevice();
        }

        try
        {
            //terminates the pylon sdk
            PylonTerminate();
        }
        catch (GenICam::GenericException &e)
        {
            THROW_ERROR_DESC(ERR_FAILED, cString::Format("Pylon not deinitialized: %s", e.GetDescription()));
        }
    }
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cBaslerCamera::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_NOERROR;
}


tResult cBaslerCamera::Init(tInitStage eStage, __exception)
{
    //never miss calling the parent implementation first!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {

        //Creating Outputpin for Video
        m_videooutputRGB.Create("outputRGB", IPin::PD_Output, static_cast<IPinEventSink*>(this));
        RegisterPin(&m_videooutputRGB);

        m_outputGCLCommands.Create("ROI_GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this));
        RegisterPin(&m_outputGCLCommands);

    }
    if (eStage == StageNormal)
    {
        m_filterProperties.Width = GetPropertyInt("Stream Width");
        m_filterProperties.Height = GetPropertyInt("Stream Height");
        m_filterProperties.ROIWidth = GetPropertyInt("ROI::Width");
        m_filterProperties.ROIHeight = GetPropertyInt("ROI::Height");
        m_filterProperties.ROIOffsetX = GetPropertyInt("ROI::XOffset");
        m_filterProperties.ROIOffsetY = GetPropertyInt("ROI::YOffset");
        m_filterProperties.Brightness = GetPropertyFloat("Brightness");
        m_filterProperties.ROIShow = GetPropertyBool("ROI::Show ROI");
        
        //Setting Bitmapformat for Output
        m_BitmapFormatRGBOut.nPixelFormat = adtf_util::IImage::PF_BGR_888;
        m_BitmapFormatRGBOut.nBitsPerPixel = 24;
        m_BitmapFormatRGBOut.nWidth = m_filterProperties.Width;
        m_BitmapFormatRGBOut.nHeight = m_filterProperties.Height;
        m_BitmapFormatRGBOut.nBytesPerLine = m_BitmapFormatRGBOut.nWidth * m_BitmapFormatRGBOut.nBitsPerPixel / 8;
        m_BitmapFormatRGBOut.nSize = m_BitmapFormatRGBOut.nBytesPerLine * m_BitmapFormatRGBOut.nHeight;
        m_BitmapFormatRGBOut.nPaletteSize = 0;

        m_videooutputRGB.SetFormat(&m_BitmapFormatRGBOut, NULL);
    }
    else if (eStage == StageGraphReady)
    {
        //Initializing Pylon5
        //Needs to be calle before any Pylon5 functions can be used
        try
        {
            PylonInitialize();
        }
        catch (GenICam::GenericException &e)
        {
            THROW_ERROR_DESC(ERR_NOT_CONNECTED, cString::Format("Pylon not Initialized: %s", e.GetDescription()));
        }
        
        //Attaching camera to camera instance member
        try
        {
            // Only look for cameras supported by Camera_t. 
            CDeviceInfo info;
            info.SetDeviceClass(CBaslerUsbInstantCamera::DeviceClass());

            // Create an instant camera object with the first found camera device that matches the specified device class.
            m_camera.Attach(CTlFactory::GetInstance().CreateFirstDevice(info));

            m_camera.Open();
            //Pixel Format for camera Output
            m_camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
            //Setting Camera options from Properties
            m_camera.Width.SetValue(m_filterProperties.Width);
            m_camera.Height.SetValue(m_filterProperties.Height);
            m_camera.AutoFunctionROIWidth.SetValue(m_filterProperties.ROIWidth);
            m_camera.AutoFunctionROIHeight.SetValue(m_filterProperties.ROIHeight);
            m_camera.AutoFunctionROIOffsetX.SetValue(m_filterProperties.ROIOffsetX);
            m_camera.AutoFunctionROIOffsetY.SetValue(m_filterProperties.ROIOffsetY);
            m_camera.AutoTargetBrightness.SetValue(m_filterProperties.Brightness);
        }
        catch (GenICam::GenericException &e)
        {
            THROW_ERROR_DESC(ERR_NOT_CONNECTED, cString::Format("Camera could not be initialized: %s", e.GetDescription()));
        }

        //Creating Thread to grab Camera Results in
        m_Thread.Create(cKernelThread::TF_Suspended, static_cast<IKernelThreadFunc*>(this), NULL, 0);
    }

    RETURN_NOERROR;
}

tResult cBaslerCamera::PropertyChanged(const tChar* strName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    //associate the properties to the member
    if (cString::IsEqual(strName,"ROI::Show ROI"))
        m_filterProperties.ROIShow = GetPropertyBool("ROI::Show ROI");
    RETURN_NOERROR;
}

tResult cBaslerCamera::ThreadFunc(adtf::cKernelThread* Thread, tVoid* data, tSize size)
{
    //PTR for the Result
    CGrabResultPtr ptrGrabResult;
   
    // check for device
    if (m_camera.IsOpen() && m_camera.IsGrabbing())
    {
        //Waiting up to 5000ms to get Result
        try
        {
            m_camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
        }
        catch (GenICam::GenericException &e)
        {
            // Error handling.
            LOG_ERROR(cString::Format("An exception occurred.  %s", e.GetDescription()));
            RETURN_NOERROR;
        }

        if (ptrGrabResult->GrabSucceeded() && m_BitmapFormatRGBOut.nSize == tInt32(ptrGrabResult->GetImageSize()))
        {
            //Transmitting grabbed result
            transmit(ptrGrabResult->GetBuffer(), ptrGrabResult->GetImageSize());

            if (m_filterProperties.ROIShow)
            {
                transmitGCLROI();
            }
        }
    }

    ptrGrabResult.Release();
    RETURN_NOERROR;
}

tResult cBaslerCamera::transmit(const void *pData, const size_t size)
{
    //Creating new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    //updating media sample
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), pData, tInt(size), IMediaSample::MSF_None));
    //transmitting
    RETURN_IF_FAILED(m_videooutputRGB.Transmit(pMediaSample));

    RETURN_NOERROR;
}

tResult cBaslerCamera::transmitGCLROI()
{
    tUInt32* aGCLProc;
    tUInt32* GCLCommands;
    //Creating new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    //allocating buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(4096));
    //updating media sample
    pMediaSample->SetTime(_clock->GetStreamTime());
    //locking GCL memory
    RETURN_IF_FAILED(pMediaSample->WriteLock((tVoid**)&aGCLProc));
    GCLCommands = aGCLProc;
    //Setting GCL commands to show ROI in video
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_CLEAR);
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_DRAWRECT, m_filterProperties.ROIOffsetX, m_filterProperties.ROIOffsetY, m_filterProperties.ROIOffsetX + m_filterProperties.ROIWidth, m_filterProperties.ROIOffsetY + m_filterProperties.ROIHeight);
    cGCLWriter::StoreCommand(GCLCommands, GCL_CMD_END);
    //unlocking GCL memory
    pMediaSample->Unlock(aGCLProc);
    //transmitting
    RETURN_IF_FAILED(m_outputGCLCommands.Transmit(pMediaSample));

    RETURN_NOERROR;
}
