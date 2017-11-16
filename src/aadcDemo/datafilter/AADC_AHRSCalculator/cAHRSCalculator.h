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
#ifndef AHRSCALCULATOR_H
#define AHRSCALCULATOR_H

#define OID_ADTF_FILTER_DEF                "adtf.aadc.AHRSCalculator"
#define ADTF_FILTER_DESC                   "AADC AHRS Calculator"
#define ADTF_FILTER_VERSION_SUB_NAME       "AHRS Calculator"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "AADCAHRSCalculator"
#define ADTF_FILTER_VERSION_STRING         "1.0.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          0
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL "This filter generates the AHRS data in normalized quaternions for the car. It is also possible to use the euler angles"
#define ADTF_CATEGORY OBJCAT_DataFilter

#include "stdafx.h"
#include "quaternionFilters.h"

/*! @defgroup AHRSCalculator AHRS Calculator
*  @{
*
* This Filter calculates the AHRS Data with the free algorithm of mahony. The angles are given as euler angles.
*
*  \image html AHRSCalculator.PNG "Plugin AHRS Calculator"
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
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Unit
* <tr><td>Roll<td>tSignalValue<td>Degree
* <tr><td>Pitch<td>tSignalValue<td>Degree
* <tr><td>Yaw<td>tSignalValue<td>Degree
* <tr><td>Q0w<td>tSignalValue<td>Normalized
* <tr><td>Q1x<td>tSignalValue<td>Normalized
* <tr><td>Q2y<td>tSignalValue<td>Normalized
* <tr><td>Q3z<td>tSignalValue<td>Normalized
* <tr><td>InerMeasUnit_Struct_Updated<td>tInerMeasUnitData<td>
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Unit
* <tr><td>InerMeasUnit_Struct<td>tInerMeasUnitData<td>-
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/datafilter/AADC_AHRSCalculator
* <tr><td>Filename<td>aadc_AHRSCalculator.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this is the main class of the AHRS calculator plugin */
class cAHRSCalculator : public cFilter
{
    /*! set the filter id and version etc */
    ADTF_FILTER_VERSION(
        OID_ADTF_FILTER_DEF,
        ADTF_FILTER_DESC,
        ADTF_CATEGORY,
        ADTF_FILTER_VERSION_SUB_NAME,
        ADTF_FILTER_VERSION_Major,
        ADTF_FILTER_VERSION_Minor,
        ADTF_FILTER_VERSION_Build,
        ADTF_FILTER_VERSION_LABEL);
public:

    /*!
     * Constructor
     *
     * \param   __info  default info pointer
     */
    cAHRSCalculator(const tChar* __info);

    /*! Destructor. */
    virtual ~cAHRSCalculator();

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
    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException** __exception_ptr = NULL);

    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException** __exception_ptr = NULL);

    /*! The input pin for inertial measurement unit struct*/
    cInputPin m_oInputInerMeasUnit;

    /*! media description for IMU dat */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;

    /*! The media description for signal value */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;

    /*! The media description for output of imu data */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitDataOutput;

    /*!
     * Process the inertial measurement unit sample contained in pMediaSample.
     *
     * \param [in,out]  pMediaSample    pointer to mediaSample with data
     *
     * \return standard adtf error code
     */
    tResult ProcessInerMeasUnitSample(IMediaSample* pMediaSample);

    /*! The output pin for inertial measurment unit struct */
    cOutputPin m_oOutputInerMeasUnit;

    /*! The output pin for roll angle */
    cOutputPin m_oOutputRoll;

    /*! The output pin for pitch angle */
    cOutputPin m_oOutputPitch;

    /*! The output pin for yaw angle */
    cOutputPin m_oOutputYaw;

    /*! The output pin for quaternion 0 */
    cOutputPin m_oOutputQ0;

    /*! The output pin for quaternion 1 */
    cOutputPin m_oOutputQ1;

    /*! The output pin for quaternion 2 */
    cOutputPin m_oOutputQ2;

    /*! The output pin for quaternion 3 */
    cOutputPin m_oOutputQ3;

    /*!
     * Transmit float value on selected output pin as tSignalValue
     *
     * \param [in,out]  oPin        output pin used for transmit
     * \param           value       the float value to transmit
     * \param           timestamp   The timestamp to use
     *
     * \return  standard adtf error code
     */
    tResult TransmitFloatValue(cOutputPin* oPin, tFloat32 value, tUInt32 timestamp);

    /*!
     * Transmit iner meas unit data.
     *
     * \param   ax                  The acceleration in x-axis
     * \param   ay                  The acceleration in y-axis
     * \param   az                  The acceleration in z-axis
     * \param   gx                  The gravity in x-axis
     * \param   gy                  The gravity in y-axis
     * \param   gz                  The gravity in z-axis
     * \param   mx                  The rotation rate in x-axis
     * \param   my                  The rotation rate in y-axis
     * \param   mz                  The rotation rate in z-axis
     * \param   roll                The roll angle
     * \param   pitch               The pitch angle
     * \param   yaw                 The yaw angle
     * \param   arduinoTimestamp    The arduino timestamp.
     *
     * \return  standard adtf error code
     */
    tResult TransmitInerMeasUnitData(
        tFloat32 ax, tFloat32 ay, tFloat32 az,
        tFloat32 gx, tFloat32 gy, tFloat32 gz,
        tFloat32 mx, tFloat32 my, tFloat32 mz,
        tFloat32 roll, tFloat32 pitch, tFloat32 yaw,
        tUInt32 arduinoTimestamp);

    /*!
     * Calculates standard adtf error code.
     *
     * \param [in,out]  roll    The roll angle
     * \param [in,out]  pitch   The pitch angle
     * \param [in,out]  yaw     The yaw angle
     *
     * \return  standard adtf error code
     */
    tResult CalculateEulerAngles(tFloat32* roll, tFloat32* pitch, tFloat32* yaw);
};

#endif /** @} */ // end of group
