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
* $Author:: spiesra $  $Date:: 2017-04-26 13:55:11#$ $Rev:: 62570   $
**********************************************************************/


#ifndef _AADC_VISUALIZATION
#define _AADC_VISUALIZATION

#define OID_ADTF_VISUALIZATION "adtf.aadc.aadc_visualization"

#include "stdafx.h"
#include "displaywidget.h"


/*! \brief cVisualization
* This filter opens a QT windows which visualizes most of the sensor data. There are several tables visualizing the data from the ultrasonic sensors, from the wheel speed sensors, from the voltage sensors and from the intertial measurement sensor (IMU). On the left side of the windows is also a graphical scene illustrating the values from the ultrasonic sensors.
*/
class cVisualization : public QObject, public cBaseQtFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_VISUALIZATION, "AADC Visualization", OBJCAT_Application, "Visualization", 1, 0, 0, "");    
    Q_OBJECT

public: // construction
    cVisualization(const tChar *);
    virtual ~cVisualization();

    // overrides cFilter
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    virtual tResult Start(__exception = NULL);
    virtual tResult Stop(__exception = NULL);
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

    /*!Handles all the input from the arduino*/
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);


protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

private:
    /*! Input pin for the ultrasonic front left data */
    cInputPin       m_oInputUsFrontLeft;

    /*! Input pin for the ultrasonic front center left data */
    cInputPin       m_oInputUsFrontCenterLeft;

    /*! Input pin for the ultrasonic front center  data */
    cInputPin       m_oInputUsFrontCenter;

    /*! Input pin for the ultrasonic front center right data */
    cInputPin       m_oInputUsFrontCenterRight;

    /*! Input pin for the ultrasonic front right data */
    cInputPin       m_oInputUsFrontRight;

    /*! Input pin for the ultrasonic side left data */
    cInputPin       m_oInputUsSideLeft;

    /*! Input pin for the ultrasonic side right data */
    cInputPin       m_oInputUsSideRight;

    /*! Input pin for the ultrasonic rear left data */
    cInputPin       m_oInputUsRearLeft;

    /*! Input pin for the ultrasonic rear center data */
    cInputPin       m_oInputUsRearCenter;

    /*! Input pin for the ultrasonic rear right data */
    cInputPin       m_oInputUsRearRight;

    /*! Input pin for the voltage data of easurement circuit */
    cInputPin       m_oInputVoltMeas;

    /*! Input pin for the voltage data of speed controller circuit */
    cInputPin       m_oInputVoltSpeedCtr;

    /*! Input pin for the inertial measurement unit data */
    cInputPin      m_oInputInerMeasUnit;

    /*! Input Pin for wheel struct*/        
    cInputPin      m_oInputWheelLeft;            

    /*! Input Pin for wheel struct*/        
    cInputPin      m_oInputWheelRight;            

    /*! descriptor for ultrasonic sensor data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionUsData; 
    /*! the id for the f32value of the media description for input pin of the ultrasoncic data */
    tBufferID m_szIDUltrasonicF32Value; 
    /*! the id for the arduino time stamp of the media description for input pin of the ultrasoncic data */
    tBufferID m_szIDUltrasonicArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsUltrasonicSet;


    /*! descriptor for voltage sensor data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionVoltData; 
    /*! the id for the f32value of the media description for input pin of the voltage data */
    tBufferID m_szIDVoltageF32Value; 
    /*! the id for the arduino time stamp of the media description for input pin of the voltage data */
    tBufferID m_szIDVoltageArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsVoltageSet;

    /*! descriptor for intertial measurement unit sensor data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionInerMeasUnitData;   
    /*! the id for the f32Q_w of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32Q_w; 
    /*! the id for the f32Q_x of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32Q_x; 
    /*! the id for the f32Q_y of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32Q_y; 
    /*! the id for the f32Q_z of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32Q_z; 
    /*! the id for the f32A_x of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32A_x; 
    /*! the id for the f32A_y of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32A_y; 
    /*! the id for the f32A_z of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitF32A_z; 
    /*! the id for the arduino time stamp of the media description for input pin of the imu data */
    tBufferID m_szIDInerMeasUnitArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsInerMeasUnitSet;

    /*! descriptor for wheel sensor data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelData; 
    /*! the id for the ui32WheelTach of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataUi32WheelTach; 
    /*! the id for the i8WheelDir of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataI8WheelDir; 
    /*! the id for the arduino time stamp of the media description for input pin of the WheelData data */
    tBufferID m_szIDWheelDataArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsWheelDataSet;


    /*! critical section for the processing of the ultrasonic data samples because function can be called from different onPinEvents */
    cCriticalSection    m_oProcessUsDataCritSection;

    /*! critical section for the processing of the voltage data samples because function can be called from different onPinEvents */
    cCriticalSection    m_oProcessVoltageDataCritSection;

    /*! critical section for the processing of the voltage data samples because function can be called from different onPinEvents */
    cCriticalSection    m_oProcessWheelDataCritSection;


private:
    /*! processes the mediasample with ultrasonic data to the gui
    @param pMediaSample the incoming mediasample
    @param usSensor the enum for the data of the mediasample
    */
    tResult ProcessUltrasonicSample(IMediaSample* pMediaSample, SensorDefinition::ultrasonicSensor usSensor);       

    /*! processes the mediasample with ultrasonic data to the gui
    @param pMediaSample the incoming mediasample
    @param voltageSensor the enum for the data of the mediasample
    */
    tResult ProcessVoltageSample(IMediaSample* pMediaSample, SensorDefinition::voltageSensor voltageSensor);       

    /*! processes the mediasample with wheel data to the gui
    @param pMediaSample the incoming mediasample
    @param wheelSensorType the enum for the data of the mediasample
    */
    tResult ProcessWheelSample(IMediaSample* pMediaSample, SensorDefinition::wheelSensor wheelSensorType);       

    /*! processes the imu  data to the gui
    @param pMediaSample the incoming mediasample
    */
    tResult ProcessInerMeasUnitSample(IMediaSample* pMediaSample);       

    /*! processes the steering sensor data to the gui
    @param pMediaSample the incoming mediasample
    */
    tResult ProcessSteeringSensSample(IMediaSample* pMediaSample);       

    /*! calculates the euler angles from the quaternion
    @param qW the w part of the quatuernion
    @param f32Qw the x part of the quatuernion
    @param f32Qy the y part of the quatuernion
    @param f32Qz the z part of the quatuernion
    @param f32Yaw_out pointer to the yaw angle
    @param f32Pitch_out pointer to the pitch angle
    @param f32Roll_out pointer ot the roll angle
    */
    tResult calulateEulerAngles(tFloat32 f32Qw, tFloat32 f32Qx, tFloat32 f32Qy, tFloat32 f32Qz, tFloat32 &f32Yaw_out, tFloat32 &f32Pitch_out, tFloat32 &f32Roll_out); 

signals:
    /*! this signal sends the ultrasonic data to the gui widget
    @param value the value for the gui
    @param usSensor the enum which identifies the sensordata
    */
    void SendUltrasonicData(float value, int usSensor);

    /*! this signal sends the voltage data to the gui widget
    @param value the value for the gui
    @param voltageSensor the enum which identifies the sensordata
    */
    void SendVoltageData(float value,int voltageSensor);

    /*! this signal sends the steering data to the gui widget
    @param value the value for the gui
    */
    void SendSteeringSensData(float value);

    /*! this signal sends the wheel data to the gui widget
    @param tach the tach value for the gui
    @param direction the direction value for the gui
    @param wheelSensorType the enum which identifies the sensordata
    */
    void SendWheelData(ulong tach, char direction, int wheelSensorType);

    /*! this signal sends the intertial measurement unit data to the gui widget
    @param i16A_x the acceleration in x-axis
    @param i16A_y the acceleration in y-axis
    @param i16A_z the acceleration in z-axis
    @param f32Yaw the yaw angle
    @param f32Pitch the pitch angle
    @param f32Roll the roll angle
    */
    void SendInerMeasUnitData(float f32A_x, float f32A_y, float f32A_z, float f32Yaw, float f32Pitch, float f32Roll);



};

#endif
