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
* $Author:: hart#$  $Date:: 2017-05-12 11:30:06#$ $Rev:: 63121   $
**********************************************************************/

#ifndef _STEERINGCONTROLLER_H_
#define _STEERINGCONTROLLER_H_

#include "stdafx.h"

#define OID_ADTF_STEERINGCONTROLLER "adtf.aadc.steeringController"

/*! @defgroup SteeringController Steering Controller
*  @{
*
* This filter can be used to set the steering servo with a curvature in meter. It loads a XML-file given
* in the properties and does a mapping of the given curvature to a servo angle based on the XML-file.
* The entries in the XML has to be found out by experiment and can be supported by the Steering
* Calibration Filter.
* A sample file was generated before and can be used for beginning. The sample file
* SteeringController.xml is located in the folder configuration_files.
*
*
*  \image html SteeringController.PNG "Plugin Steering Controller"
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Configuration File For Mapping<td>The XML to be loaded has to be set here<td>
* <tr><td>Debug Mode<td>If true debug infos are plotted to console<td>False
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>servoAngle<td><td>tSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>curvature<td><td>tSignalValue
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/algorithms/AADC_SteeringController
* <tr><td>Filename<td>aadc_steeringController.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this is the main class for the steering controller filter */
class cSteeringController : public adtf::cFilter
{
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_STEERINGCONTROLLER, "AADC Steering Controller", OBJCAT_DataFilter, "Steering Controller", 1, 0, 0, "");

    /*! the input pin for the set point value. */
    cInputPin m_oInputCurvature;
    /*! the output pin for the manipulated value. */
    cOutputPin m_oOutputServoAngle;

public:

    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cSteeringController(const tChar* __info);

    /*! Destructor. */
    virtual ~cSteeringController();

protected: // overwrites cFilter
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

private:
    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*!
    * Gets the time.
    *
    * \return  The streamtime in milliseconds
    */
    tTimeStamp GetTime();

    /*! reads the xml file which is set in the filter properties
    * \param configFilename filename of file to load
    * \return   Returns a standard result code.
    */
    tResult LoadConfigurationData(const cFilename& configFilename);

    /*! prints the configuration data to console
    * \return   Returns a standard result code.
    */
    tResult PrintConfigurationData();

    /*! holds the values for the negative supporting points */
    vector<std::pair<tFloat32, tFloat32> > m_f32ValuesNeg;
    /*! holds the values for the positive supporting points */
    vector<std::pair<tFloat32, tFloat32> > m_f32ValuesPos;

    /*! media description for the input pin set speed */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionCurvature;
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDCurvatureF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDCurvatureArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bCurvatureSet;

    /*! media description for the input pin set speed */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionServoAngle;
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDServoAngleF32Value;
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDServoAngleArduinoTimestamp;
    /*! indicates of bufferIDs were set */
    tBool m_bServoAngleSet;

    /*! if the debug mode is enabled */
    tBool m_bDebugModeEnabled;

    /*! getting the cosine interpolation value
    * \param f32Value the value which should be interpolated
    * \return the interpolated value
    */
    tFloat32 getCosineInterpolation(tFloat32 f32Value);

    /*! doing the cosine interpolation
    * \param f32y1 the preceding value
    * \param f32y2 the next value
    * \param f32mu the percentage of distance between f32y1 and f32y2 for that the interpolation should be done
    * \return the interpolated value
    */
    tFloat32 doCosineInterpolation(tFloat32 f32y1, tFloat32 f32y2, tFloat32 f32mu);

};
/*! @} */ // end of group
#endif // _STEERINGCONTROLLER_H_

