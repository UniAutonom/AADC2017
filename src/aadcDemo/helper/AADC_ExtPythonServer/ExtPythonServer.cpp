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
* $Author:: spie#$  $Date:: 2017-05-12 10:01:55#$ $Rev:: 63111   $
**********************************************************************/

#include "ExtPythonServer.h"
#include <fstream>

#define CONSOLE_LOG(_text, _log_level) if (m_filterProperties.enableDebugOutput) { LOG_FN_OUTPUT((_text), _log_level); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, adtf_util::LOG_LVL_INFO)                        //!< log info messages
#define CONSOLE_LOG_WARNING(_text)   CONSOLE_LOG(_text, adtf_util::LOG_LVL_WARNING)                     //!< log warning messages
#define CONSOLE_LOG_DUMP(_text)      CONSOLE_LOG(_text, adtf_util::LOG_LVL_DUMP)                        //!< log dump messages
#define CONSOLE_LOG_ERROR(_text)     CONSOLE_LOG(_text, adtf_util::LOG_LVL_ERROR)                       //!< log error messages


const cString ExtPythonServer::PropThriftPortName = "Thrift Port";
const cString ExtPythonServer::PropThriftPortDesc = "Port number for Thrift Server";
const tInt ExtPythonServer::PropThriftPortDefault = 1833;

const cString ExtPythonServer::PropThriftIPV4AddressName = "Thrift IP Address";
const cString ExtPythonServer::PropThriftIPV4AddressDesc = "the ip v4 adress of the thrift RPC server";
const cString ExtPythonServer::PropThriftIPV4AddressDefault = "localhost";

const cString ExtPythonServer::PropEnableConsoleLogName = "Enable debug outputs to console ";
const cString ExtPythonServer::PropEnableConsoleLogDesc = "If enabled additional debug information is printed to the console (Warning: decreases performance).";
const tBool  ExtPythonServer::PropEnableConsoleLogDefault = tFalse;

const cString ExtPythonServer::PropSendBitmapFormatName = "Send Bitmap Format";
const cString ExtPythonServer::PropSendBitmapFormatDesc = "If a complete bitmap including the bitmap header should be send to thrift server";
const tInt  ExtPythonServer::PropSendBitmapFormatDefault = 1;

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

using namespace  ::ext_iface;

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
                   OID_ADTF_FILTER_DEF,
                   ExtPythonServer)

ExtPythonServer::ExtPythonServer(const tChar* __info) : cFilter(__info)
{
    SetPropertyInt(PropThriftPortName, PropThriftPortDefault);
    SetPropertyStr(PropThriftPortName + NSSUBPROP_DESCRIPTION, PropThriftPortDesc);

    SetPropertyStr(PropThriftIPV4AddressName, PropThriftIPV4AddressDefault);
    SetPropertyStr(PropThriftIPV4AddressName + NSSUBPROP_DESCRIPTION, PropThriftIPV4AddressDesc);

    SetPropertyBool(PropEnableConsoleLogName, PropEnableConsoleLogDefault);
    SetPropertyStr(PropEnableConsoleLogName + NSSUBPROP_DESCRIPTION, PropEnableConsoleLogDesc);
    SetPropertyBool(PropEnableConsoleLogName + NSSUBPROP_ISCHANGEABLE, tFalse);

    SetPropertyInt(PropSendBitmapFormatName, PropSendBitmapFormatDefault);
    SetPropertyStr(PropSendBitmapFormatName + NSSUBPROP_DESCRIPTION, PropSendBitmapFormatDesc);
    SetPropertyStr(PropSendBitmapFormatName + NSSUBPROP_VALUELIST, "1@RAWIMAGE|2@BMP|3@JPG");

    SetPropertyBool("Debug:Show Outputimage", true);
    SetPropertyStr("Debug:Show Outputimage" NSSUBPROP_DESCRIPTION, "Show Outputimage");
    SetPropertyBool("Debug:Show Outputimage" NSSUBPROP_ISCHANGEABLE, tTrue);

}



ExtPythonServer::~ExtPythonServer()
{
}

tResult ExtPythonServer::Start(__exception)
{
    // start the thread if not running by now
    if (m_oThriftClientThread.GetState() != cKernelThread::TS_Running)
    {
        m_oThriftClientThread.Run();
    }
    return cFilter::Start(__exception_ptr);
}

tResult ExtPythonServer::Stop(__exception)
{
    //suspend the thread
    if (m_oThriftClientThread.GetState() == cKernelThread::TS_Running)
    {
        m_oThriftClientThread.Suspend();
    }

    return cFilter::Stop(__exception_ptr);
}

tResult ExtPythonServer::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    // release the thread
    m_oThriftClientThread.Terminate(tTrue);
    m_oThriftClientThread.Release();

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult ExtPythonServer::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if (pSource == &m_inputPinImageData)
        {
            // check if we have a valid image format by now
            if (m_sInputFormat.nWidth == 0 || m_sInputFormat.nHeight == 0)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_inputPinImageData.GetMediaType(&pType));

                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

                // set the image format of the input video pin
                UpdateInputImageFormat(pTypeVideo->GetFormat());
                RETURN_NOERROR;
            }

            if (m_sInputFormat.nSize != 0)
            {
                Process(pMediaSample);
            }
        }
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_inputPinImageData.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());
        RETURN_NOERROR;
    }
    RETURN_NOERROR;
}

tResult ExtPythonServer::ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData)
{
    if (pThread == &m_oThriftClientThread)
    {
        tBool clientIsOpen = tFalse;
        try
        {
            //try to open if not opened
            if (!m_thriftClient->getOutputProtocol()->getOutputTransport()->isOpen())
            {
                //open the port for the client
                m_thriftClient->getOutputProtocol()->getOutputTransport()->open();
            }
            //verify
            if (m_thriftClient->getOutputProtocol()->getOutputTransport()->isOpen())
            {
                clientIsOpen = tTrue;
            }

        }
        catch (apache::thrift::TException except)
        {
            LOG_ERROR("Thrift client could not connect to server");
        }

        if (clientIsOpen)
        {
            //enter mutex
            m_critSectionSendToPython.Enter();
            if (m_thriftRawMessageBuffer.raw_data.size() != 0)
            {
                try
                {
                    //                    int i = 0;
                    //                    std::ostringstream oss;
                    //                    oss << "/home/aadc/ADTF/Images/" << i << ".jpeg";

                    //                    while(std::ifstream(oss.str().c_str()))
                    //                    //while(access( oss.str().c_str(), F_OK ) != -1)
                    //                    //while(pfile != NULL)
                    //                    {
                    //                        oss.str("");
                    //                        oss << "/home/aadc/ADTF/Images/" << ++i << ".jpeg";
                    //                        LOG_INFO(cString::Format(oss.str().c_str()));
                    //                    }
                    //                    // this code can be used to save the image here to a file and to verify the data/

                    //                    cFilename filenameOutput(oss.str().c_str());
                    //                    ADTF_GET_CONFIG_FILENAME(filenameOutput);


                    //                    cFile newFile;
                    //                    if (newFile.Open(filenameOutput, cFile::OM_Write))
                    //                    {
                    //                        newFile.Write(m_thriftRawMessageBuffer.raw_data.data(), m_thriftRawMessageBuffer.raw_data.size());
                    //                        newFile.Close();
                    //                        LOG_INFO(cString::Format("Created file with %i", m_thriftRawMessageBuffer.raw_data.size()));
                    //                    }
                    //                    sleep((10));


                    /* this code can be used to save the image here to a file and to verify the data
                    cFilename filenameOutput("output_ADTF.bmp");
                    ADTF_GET_CONFIG_FILENAME(filenameOutput);
                    cFile newFile;
                    if (newFile.Open(filenameOutput, cFile::OM_Write))
                    {
                        newFile.Write(m_thriftRawMessageBuffer.raw_data.data(), m_thriftRawMessageBuffer.raw_data.size());
                        newFile.Close();
                        LOG_INFO(cString::Format("Created file with %i", m_thriftRawMessageBuffer.raw_data.size()));
                    }
*/
                    //send raw data to python server and receive response
                    std::vector<myClassificationResult> classificationResults;
                    TDataResultList response;
                    m_thriftClient->rawData(response, TransportDef::IMAGEDATA, m_thriftRawMessageBuffer, m_thriftImageParamsBuffer);
                    // transmit response to output pin
                    if (response.size() != 0)
                    {

                        if(m_filterProperties.m_outputImageDebug){
                            //LOG_INFO(cString::Format("%i", response.size()));

                            for (std::vector<class TDataResult>::iterator it = response.begin(); it != response.end(); ++it)
                            {
                                // Durch austesten herausgefunden dass es irgendwie behindert ist
                                float x1, x2, y1, y2;
                                x1 = it->x_2 ;
                                x2 = it->y_2 ;
                                y1 = it->x_1 ;
                                y2 = it->y_1 ;

                                cv::Rect box;
                                cv::Mat outputImage;
                                outputImage = m_inputImage;
                                //cv::rectangle(outputImage,ROI, 255,3);

                                if(it->probability > 0.5f){

                                    box = cv::Rect(cv::Point((int)(x1*outputImage.cols), (int)(y1*outputImage.rows)), cv::Point((int)(x2*outputImage.cols), (int)(y2*outputImage.rows)));
                                    cv::rectangle(outputImage,box,255,3);

                                    cv::putText(outputImage, it->classification, cv::Point(x1*outputImage.cols, y1*outputImage.rows), 1, 3, cv::Scalar(0, 0, 255), 3);

                                    //                                LOG_INFO(cString::Format("x1: %i", (int)(x1*outputImage.cols)));
                                    //                                LOG_INFO(cString::Format("y1: %i", (int)(y1*outputImage.rows)));
                                    //                                LOG_INFO(cString::Format("x2: %i", (int)(x2*outputImage.cols)));
                                    //                                LOG_INFO(cString::Format("y2: %i", (int)(y2*outputImage.rows)));

                                    //cv::line(outputImage,cv::Point(400,650),cv::Point(400,1000),255,3);
                                    //cv::line(outputImage,cv::Point(1500,650),cv::Point(1500,1000),255,3);
                                    classificationResults.push_back(myClassificationResult(it->classification, it->probability, box));
                                }
                                cv::line(outputImage, cv::Point(0,750), cv::Point(1920, 750),255,3);
                                if (!outputImage.empty() && m_oVideoOutputPin.IsConnected())
                                {
                                    UpdateOutputImageFormat(outputImage);
                                    //create a cImage from CV Matrix (not necessary, just for demonstration9
                                    cImage newImage;
                                    newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

                                    //create the new media sample
                                    cObjectPtr<IMediaSample> pMediaSample;
                                    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
                                    //updating media sample
                                    //RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
                                    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), outputImage.data, m_sOutputFormat.nSize, 0));
                                    //transmitting
                                    RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

                                    outputImage.release();
                                }
                            }
                        }


                        //LOG_INFO(cString::Format("Result: %s, %f", it->classification.c_str(), it->probability));
                        Transmit(classificationResults);
                    }
                    // clear the buffer
                    m_thriftRawMessageBuffer.raw_data.clear();
                    m_critSectionSendToPython.Leave();


                }
                catch (apache::thrift::TException except)
                {
                    //send method quit with exception so write it here to console
                    LOG_ERROR("Thrift client could not send to server");
                    LOG_ERROR(cString::Format("EXCEPTION: %s", except.what()));
                    m_critSectionSendToPython.Leave();
                    cSystem::Sleep(200000);
                }
            }
            else
            {
                //nothing to transmit
                m_critSectionSendToPython.Leave();
                cSystem::Sleep(50000);
            }
        }
        else
        {
            //no client open
            LOG_ERROR("Thrift client could not connect to server");
            cSystem::Sleep(200000);
        }
    }

    RETURN_NOERROR;
}


tResult ExtPythonServer::Init(tInitStage eStage, __exception)
{
    //never miss calling the parent implementation first!!s
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        //create pins
        RETURN_IF_FAILED(m_inputPinImageData.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPinImageData));

        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

        // response output pin
        RETURN_IF_FAILED(m_outputResponseData.Create("Response",
                                                     new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED),this));
        RETURN_IF_FAILED(RegisterPin(&m_outputResponseData));
    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {

        //init the socket, transport and the protocol from thrift
        boost::shared_ptr<TTransport> socket(new TSocket(m_filterProperties.server_addressIPV4.GetPtr(), m_filterProperties.server_port));
        boost::shared_ptr<TTransport> transport(new TBufferedTransport(socket));
        boost::shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));
        //make the client
        m_thriftClient = boost::make_shared<ext_iface::ExtServiceClient>(protocol);

        //create the client thread
        tResult nResult = m_oThriftClientThread.Create(cKernelThread::TF_Suspended, static_cast<adtf::IKernelThreadFunc*>(this));

        if (IS_FAILED(nResult))
        {
            THROW_ERROR_DESC(nResult, "Failed to create threads");
        }

        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_inputPinImageData.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());

    }

    RETURN_NOERROR;
}

tResult ExtPythonServer::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);

        //Check input image format and convert to openCV format
        switch (m_sInputFormat.nPixelFormat)
        {
        case adtf_util::IImage::PF_GREYSCALE_8:
            m_openCVType = CV_8UC1;
            break;
        case adtf_util::IImage::PF_BGR_888:
        case adtf_util::IImage::PF_RGB_888:
            m_openCVType = CV_8UC3;
            break;
        case adtf_util::IImage::PF_BGRA_8888:
        case adtf_util::IImage::PF_RGBA_8888:
            m_openCVType = CV_8UC4;
            break;
        default:
            m_openCVType = CV_8UC3;
            break;
        }
        // write the struct to be send to thrift
        m_thriftImageParamsBuffer.__set_bytesPerPixel(m_sInputFormat.nBitsPerPixel);
        m_thriftImageParamsBuffer.__set_height(int16_t(m_sInputFormat.nHeight));
        m_thriftImageParamsBuffer.__set_width(int16_t(m_sInputFormat.nWidth));
    }

    RETURN_NOERROR;
}

tResult ExtPythonServer::PropertyChanged(const tChar* strName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
    //associate the properties to the member
    if (strName == PropThriftPortName) m_filterProperties.server_port = GetPropertyInt(PropThriftPortName);
    else if (strName == PropEnableConsoleLogName)   m_filterProperties.enableDebugOutput = GetPropertyBool(PropEnableConsoleLogName);
    else if (strName == PropSendBitmapFormatName)   m_filterProperties.sendFormat = GetPropertyInt(PropSendBitmapFormatName);
    else if (strName == PropThriftIPV4AddressName)   m_filterProperties.server_addressIPV4 = GetPropertyStr(PropThriftIPV4AddressName);
    else if (cString::IsEqual(strName, "Debug:Show Outputimage")) m_filterProperties.m_outputImageDebug = GetPropertyBool("Debug:Show Outputimage");

    RETURN_NOERROR;
}

tResult ExtPythonServer::Process(IMediaSample* pMediaSample)
{
    //we have stuffed bytes here so we cannot process it correctly
    if (m_sInputFormat.nWidth * m_sInputFormat.nBitsPerPixel / 8 != m_sInputFormat.nBytesPerLine)
    {
        RETURN_ERROR(ERR_NOT_SUPPORTED);
    }

    /////
    //    const tVoid* l_pSrcBuffer;
    //    if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer))){
    //        if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
    //        {
    //            m_inputImage.data = (uchar*)(l_pSrcBuffer);
    //            pMediaSample->Unlock(l_pSrcBuffer);
    //        }
    //    }
    /////

    const tVoid* pIncomingData;
    if (IS_OK(pMediaSample->Lock(&pIncomingData)))
    {
        cv::Mat inputImageTmp = cv::Mat(m_sInputFormat.nHeight, m_sInputFormat.nWidth, m_openCVType, (tVoid*)pIncomingData, m_sInputFormat.nBytesPerLine);
        //enter mutex and set data from media to buffer
        //if still blocked we return and drop the data
        if (m_critSectionSendToPython.TryEnter())
        {
            // convert the received image to a real bitmap include its bitmap format header
            if (m_filterProperties.sendFormat == 1)
            {
                //copy raw data
                m_thriftRawMessageBuffer.raw_data.clear();
                m_thriftRawMessageBuffer.raw_data.assign((const char*)(pIncomingData), pMediaSample->GetSize());
            }
            else if (m_filterProperties.sendFormat == 2)
            {
                if(m_filterProperties.m_outputImageDebug){
                    m_inputImage = inputImageTmp.clone();
                }

                std::vector<unsigned char> outFileData;
                //convert to bitmap
                if (cv::imencode("*.jpeg", inputImageTmp, outFileData))
                {
                    m_thriftRawMessageBuffer.raw_data.clear();
                    m_thriftRawMessageBuffer.raw_data.assign((const char*)(outFileData.data()), outFileData.size());
                }
            }
            else if (m_filterProperties.sendFormat == 3)
            {
                if(m_filterProperties.m_outputImageDebug){
                    m_inputImage = inputImageTmp.clone();
                }

                std::vector<unsigned char> outFileData;
                //convert to bitmap
                if (cv::imencode("*.jpg", inputImageTmp, outFileData))
                {
                    m_thriftRawMessageBuffer.raw_data.clear();
                    m_thriftRawMessageBuffer.raw_data.assign((const char*)(outFileData.data()), outFileData.size());
                }
            }
            m_critSectionSendToPython.Leave();
        }

        RETURN_IF_FAILED(pMediaSample->Unlock(pIncomingData));

    }

    RETURN_NOERROR;
}

tResult ExtPythonServer::Transmit(const std::vector<myClassificationResult>& response)
{
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), response.data(), tInt(response.size() * sizeof(myClassificationResult)), 0));
    // transmit the sample on output pin
    RETURN_IF_FAILED(m_outputResponseData.Transmit(pMediaSample));

    RETURN_NOERROR;
}
/////////////////////
tResult ExtPythonServer::UpdateOutputImageFormat(const cv::Mat& outputImage)
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

///////////////////////
