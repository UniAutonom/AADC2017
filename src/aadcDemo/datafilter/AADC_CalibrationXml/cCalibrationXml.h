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
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef _CALIBRATION_XML_H_
#define _CALIBRATION_XML_H_

#include "stdafx.h"

#define OID_ADTF_SENSOR_CALIBRATION "adtf.aadc.calibrationXML"
/*! @defgroup CalibrationXml Calibration XML
*  @{
*
* This filter does in comparison to the Calibration Filter an extended calibration. The user has to generate a XML-File which has to be set in the property. This XML-File must include a calibration table with x- and y-values and a mode which has to be used for interpolation.
* \n When the filter receives an input value it looks on the x-Axis of the calibration table and gets the corresponding value on the y-Axis using the set interpolation. This result is transmitted on the pin output_value. If the value on the input pin is greater than the maximum value in the table the maximum value is used, if it is smaller than the minimum value the minimum value is used.
* \n The x-Values in the table must be in increasing order otherwise the calibration does not work and prints an error.
* \n The structure of the XML has to be like this:
* \n
*   \code
* \n <?xml version="1.0" encoding="iso-8859-1" standalone="yes"?>
* \n <calibration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
* \n <span style="padding-left: 2em"><settings></span>
* \n 	<span style="padding-left: 4em"><mode>linear</mode></span>
* \n <span style="padding-left: 2em"></settings></span>
* \n <span style="padding-left: 2em"><supportingPoints></span>
* \n 	<span style="padding-left: 4em"><point></span>
* \n 		<span style="padding-left: 6em"><xValue>307</xValue></span>
* \n 		<span style="padding-left: 6em"><yValue>-45</yValue></span>
* \n 	<span style="padding-left: 4em"></point></span>
* \n 	<span style="padding-left: 4em"><point></span>
* \n 		<span style="padding-left: 6em"><xValue>572</xValue></span>
* \n 		<span style="padding-left: 6em"><yValue>45</yValue></span>
* \n 	<span style="padding-left: 4em"></point></span>
* \n <span style="padding-left: 2em"></supportingPoints></span>
* \n </calibration>
* \n
*   \endcode
* \n If no interpolation mode is set in the XML – File the interpolation mode is chosen from the filter properties (i.e. the mode in the XML-File always overwrite the filter property).

*
*  \image html CalibrationXML.PNG "Plugin CalibrationXML"
*
* \b Dependencies \n
* This plugin needs the following libraries:
*
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Border Warnings to Console<td>If enabled a warning is printed to console each time the border points of the given xml are reached<td>False
* <tr><td>Configuration File for Interpolation<td>The XML to be loaded has to be set here<td>
* <tr><td>Interpolation<td>Sets the mode of interpolation between the given points in the XML<td>linear
* <tr><td>Print initial table to Console<td>If enabled the loaded points of the interpolation table of the XML are printed to console<td>False
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>output_value<td><td>tSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>input_value<td><td>tSignalValue
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/datafilter/AADC_CalibrationXml
* <tr><td>Filename<td>aadc_calibrationXml.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/
/*! the main class of the calibration XML filter */
class cCalibrationXml : public adtf::cFilter
{
    /*! set the filter version */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SENSOR_CALIBRATION, "AADC Calibration XML", OBJCAT_DataFilter, "Calibration XML", 1, 0, 0, "");

    /*! input pin for the raw value */
    cInputPin m_oInput;
    /*! output pin for the calibrated value */
    cOutputPin m_oOutput;

public:
    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cCalibrationXml(const tChar* __info);

    /*! default destructor
    */
    virtual ~cCalibrationXml();

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
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
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
    *     called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *     and can be overwritten by the special filter.
    *     \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *     If not using the cException smart pointer, the interface has to
    *     be released by calling Unref().
    *     \return Standard Result Code.
    *     \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *     (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Stop(ucom::IException** __exception_ptr = NULL);

    /*! Implements the default cFilter state machine call. It will be
    *     called automatically by changing the filters state and needs
    *     to be overwritten by the special filter.
    *     Please see  page_filter_life_cycle for further information on when the state of a filter changes.
    *     \param  eStage [in]    The Init function will be called when the filter state changes as follows:\n
    *     \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *                        If not using the cException smart pointer, the interface has to
    *                        be released by calling Unref().
    *
    *     \return Standard Result Code.
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *    \param [in] pSource Pointer to the sending pin's IPin interface.
    *    \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *    \param [in] nParam1 Optional integer parameter.
    *    \param [in] nParam2 Optional integer parameter.
    *    \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *    \return   Returns a standard result code.
    *    \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *    You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
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
     * reads the xml file which is set in the filter properties.
     *
     * \return  standard ADTF error code
     */
    tResult LoadConfigurationData();

    /*!
     * checks the loaded configuration data; checks if the xvalues are in increasing order.
     *
     * \return  standard ADTF error code.
     */
    tResult CheckConfigurationData();

    /*! doing the linear interpolation
    * \param fl32InputValue the value which should be interpolated
    * \return linearinterpolated value
    */
    tFloat32 getLinearInterpolatedValue(tFloat32 fl32InputValue);

    /*! doing the cubic spline interpolation
    * \param fl32InputValue the value which should be interpolated
    * \return cubiv interpolated value
    */
    tFloat32 getCubicSplineInterpolatedValue(tFloat32 fl32InputValue);

    /*! holds the xml file for the supporting points*/
    cFilename m_fileConfig;

    /*! holds the yValues for the supporting points*/
    vector<tFloat32> m_yValues;
    /*! holds the xValues for the supporting points*/
    vector<tFloat32> m_xValues;

    /*! the class for cubic interpolation*/
    Cubic *m_cubicInterpolation;

    /*! if debug console output is enabled */
    tBool m_bDebugModeEnabled;

    /*! indicates which calibration mode is used: 1: linear, 2: cubic, 3: none*/
    tInt m_iCalibrationMode;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDSignalF32Value;
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDSignalArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSignalSet;

};



//*************************************************************************************************
/*! @} */ // end of group
#endif // _CALIBRATION_XML_H_ 

