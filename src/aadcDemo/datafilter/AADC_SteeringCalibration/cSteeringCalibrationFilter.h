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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/


#ifndef _STEERING_CALIBRATION_FILTER_H_
#define _STEERING_CALIBRATION_FILTER_H_

#define __guid "adtf.aadc.steeringCalibration"
#include "cCalibrationView.h"
#include "cCalibrationModel.h"

/*! @defgroup SteeringCalibration Steering Calibration
*  @{
*
* This filter can be used to obtain a calibration table for the Steering Controller which can be used to steer the car with given curvature and not only by setting the servo angle of the steering controller. The servo angle is an abstract value and does not describe the car behavior explicitly. The curvature is the more useable to control the car on the street.

The calibration is done by series of driven curvature defined by different servo steering angles as the GUI shows. If the button Go is clicked the car starts to drive a curvature with the servo steering angle in the line of the table. If the maximum arc distance or the maximum angle from the properties is reached the filter calculates the radius of the driven curvature by using the driven distance and the yaw angle obtained by the IMU. The calculated radius is shown in the column Radius.
With the button Store Calibration in XML File  the results can be saved to an XML File.
The output pin SpeedController must not be connected to the Arduino Actuators only by using the Wheel Speed Controller.
*
*  \image html SteeringCalibration.PNG "Plugin Steering Calibration"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT  v.4.7.1
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>average speed [m/s]<td>The average speed during calibration procedure<td>1
* <tr><td>max angle [°]<td>The maximum angle driven in calibration<td>170
* <tr><td>max arc distance [m]<td>The maximum arc distance driven in calibration<td>5
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>Media Description
* <tr><td>SteeringController<td><td>tSignalValue
* <tr><td>SpeedController<td><td>tSignalValue
*</table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>distance_overall<td><td>tSignalValue
* <tr><td>yaw<td><td>0<td>0
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/datafilter/AADC_SteeringCalibration
* <tr><td>Filename<td>aadc_steeringCalibration.plb
* <tr><td>Version<td>1.0.0
* </table>
*
*/


/*! this is the main class of the steering calibration filter */
class cSteeringCalibrationFilter : public cBaseQtFilter
{

    /*! set the filter ID and the filter version */
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Steering Calibration", adtf::OBJCAT_DataFilter, "Steering Calibration Filter", 1, 0,1, "");
    // Q_OBJECT

protected:
    /*! input pin for yaw angle input */
    cInputPin    m_oYawInput;
    /*! input pin for distance input */
    cInputPin    m_oDistanceInput;
    /*! output pin for speed output */
    cOutputPin    m_oSpeedOutput;
    /*! output for steering angle output */
    cOutputPin    m_oSteeringOutput;

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

public:
    /*! constructor for template class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cSteeringCalibrationFilter(const tChar* __info);

    /*! the destructor for this class
    */
    virtual ~cSteeringCalibrationFilter();

protected:

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

    /*! the maximum arc distance */
    tFloat32 m_prop_maxArcDistance;
    /*! the maximum angle */
    tFloat32 m_prop_maxAngle;
    /*! the average speed */
    tFloat32 m_prop_averageSpeed;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignal;
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDSignalF32Value;
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDSignalArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSignalSet;

    /*! transmits the speed value
    * \param speed the speed  value to be transmitted
    * \return   Returns a standard result code.
    */
    tResult transmitSpeed(tFloat32 speed);

    /*! transmits the angle value
    * \param angle the angle value to be transmitted
    * \return   Returns a standard result code.
    */
    tResult transmitSteerAngle(tFloat32 angle);

    /*! checks the distance
    * \param distance value to be checked
    * \return   Returns a standard result code.
    */
    tResult processDistance(tFloat32 distance);

    /*! checks the yaw
    * \param Yaw value to be checked
    * \return   Returns a standard result code.
    */
    tResult processYaw(tFloat32 Yaw);

    /*! the calibration model of the filter*/
    cCalibrationModel m_model;

    /*!
    * does the driving of car
    * \return   Returns a standard result code.
    */
    tResult doDriving();

    /*!
    * does the driving of car
    */
    void DriveFinished();


protected:
    // Implement cBaseQtFilter

    /*! Creates the widget instance
    *   \return handle to view
    */
    virtual tHandle CreateView();

    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    virtual tResult ReleaseView();

    /*! the widget of the filter */
    cCalibrationView *m_calibrationWidget;

};



//*************************************************************************************************
/*! @} */ // end of group
#endif // _STEERING_CALIBRATION_FILTER_H_ 
