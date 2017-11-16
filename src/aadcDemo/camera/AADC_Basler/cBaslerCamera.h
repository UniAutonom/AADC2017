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
* $Author:: spie#$  $Date:: 2017-05-12 12:53:59#$ $Rev:: 63132   $
**********************************************************************/

#ifndef AADC_cBaslerCamera_CLASS_H
#define AADC_cBaslerCamera_CLASS_H



#include "stdafx.h"

#define OID_ADTF_FILTER_DEF "adtf.aadc.baslercamera" //unique for a filter
#define ADTF_FILTER_DESC "AADC Basler Camera"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "cBaslerCamera_filter"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A basler camera filter \n$Rev:: 62962"


//necessary because stupid WinGDI.h redefines it
#undef GetObject

/*!
* @defgroup BaslerCamera Basler Camera
* @{
*  Grabber Filter for Basler Camera
*
* This filter grabs images from the Basler Camera. The grabbing parameter for the camera can be set by the filterproperties. The grabbing procedure needs the pylon5 SDK which is provided by Basler.
* \image html BaslerCamera.PNG "Plugin cBaslerCamera"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li Pylon5  v.5.0.5
*
* <b> Filter Properties</b>
* The filter has the following properties:
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Stream Width<td>Width of the Stream<td>1280
* <tr><td>Stream Height<td>Height of the Stream<td>960
* <tr><td>Brightness<td>target Brightness for Auto Gain Function<td>0.3
* <tr><td>ROI::XOffset <td>x-Offset of the ROI<td>440
* <tr><td>ROI::YOffset <td>y-Offset of the ROI<td>330
* <tr><td>ROI::Width <td>Width of the ROI<td>400
* <tr><td>ROI::Height <td>Height of the ROI<td>300
* </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>m_videooutputRGB<td>Outputpin for Camera Stream Imagetype: PF_BGR_888<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>m_outputGCLCommands<td>Outputpin for Camera GCL Stream<td>MEDIA_TYPE_COMMAND<td>MEDIA_SUBTYPE_COMMAND_GCL
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/camera/AADC_Basler
* <tr><td>Filename<td>aadc_BaslerCamera.plb
* <tr><td>Version<td>1.0.0
* </table>
*/

//!  Class of Basler Camera Filter
/*!
* This class is the main class of the Basler Camera Filter
*/
class cBaslerCamera : public adtf::cFilter, adtf::IKernelThreadFunc
{
public:

    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_SensorDevice,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

public:
    /*! constructor for cBaslerCamera class
        \param __info   [in] This is the name of the filter instance.
        */
    cBaslerCamera(const tChar* __info);

    /*! the destructor for this class
    */
    ~cBaslerCamera();

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

    /*! This Function is always called when any property has changed. This should be the only place
    *    to read from the properties itself and store their values in a member.
    *    \param [in] strName the name of the property that has changed.
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* strName);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException** __exception_ptr = NULL);


    /*! This Function will be called by all pins the filter is registered to.
     *   \param [in] pSource Pointer to the sending pin's IPin interface.
     *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
     *   \param [in] nParam1 Optional integer parameter.
     *   \param [in] nParam2 Optional integer parameter.
     *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
     *   \return   Returns a standard result code.
     *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
     *   You need to synchronize this call        your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
     */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

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

private:
    /*! transmits a value on output pin
     *   \param pData pointer to data to be transmitted
     *   \param size size of data to be transmittet to be transmitted
     *   \return Returns a standard result code.
     */
    tResult transmit(const void *pData, const size_t size);

    /*! transmits GCL commands to Show ROI
     *    \return Returns a standard result code.
     */
    tResult transmitGCLROI();

    /*! output pin for RGB stream */
    cVideoPin          m_videooutputRGB;

    /*! output for GCL Commands*/
    cOutputPin         m_outputGCLCommands;

    /*! the struct with all the properties*/
    struct filterProperties
    {
        /*! Width of the Stream*/
        int Width;
        /*! Height of the Stream*/
        int Height;
        /*! Offset of the ROI in the Stream*/
        int ROIOffsetX;
        /*! Offset of the ROI in the Stream*/
        int ROIOffsetY;
        /*! Width of the ROI*/
        int ROIWidth;
        /*! Height of the ROI*/
        int ROIHeight;
        /*! Target Brightness for Auto Gain Function*/
        tFloat64 Brightness;
        /*! Shows ROI as a Rectangle in the Video*/
        tBool ROIShow;
    }
    /*! the filter properties of this class */
    m_filterProperties;

    /*! Bitmatformat for the Output*/
    tBitmapFormat m_BitmapFormatRGBOut;
    /*! Thread Function to grab the Stream of the Camera
     *   \param Thread Thread associated with the funtion
     *   \param data external data for the Thread
     *   \param size of the data
     *   \result Returns a standard result code.
     */
    tResult ThreadFunc(adtf::cKernelThread* Thread, tVoid* data, tSize size);

    /*! Camera Instance */
    CBaslerUsbInstantCamera m_camera;

    /*! Thread to grab the Stream in*/
    cKernelThread m_Thread;

};

/*!
*@}
*/

#endif
