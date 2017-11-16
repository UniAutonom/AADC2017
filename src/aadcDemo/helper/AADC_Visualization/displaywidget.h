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

#ifndef DISPWIDGET
#define DISPWIDGET

#define GRAPHICSSCENE_WIDTH 500
#define GRAPHICSSCENE_HEIGHT 500
#define US_BEAM_DIM 400
#define US_BEAM_SPAN_ANGLE 30

#include "stdafx.h"

struct usSensorGeometrics{
// coordinates from the car middle
    tInt iXPos;
    tInt iYPos;
    tInt iStartAngle;
    tFloat32 f32MaxValue;
    };

class cVisualization;

namespace Ui {
class DisplayGUI;
}
/*! 
 *     This is the display widget for the filter class of the visualizaion
 */
class DisplayWidget : public QWidget
{
    Q_OBJECT

    public:
        DisplayWidget(QWidget* parent);
        ~DisplayWidget();
    public slots:
        
        /*! slot for receiving data of ultrasonic sensors for gui 
        @param value value of ultrasonic data
        @param usSensor type of ultrasonic sensor
        */
        void OnUltrasonicData(float value, int usSensor);
        
        /*!slot for receiving the voltage data to the gui widget
        @param value the value for the gui
        @param voltageSensor the enum which identifies the sensordata
        */
        void OnVoltageData(float value, int voltageSensor);
    
        /*! slot for receiving the wheel data to the gui widget
        @param tach the tach value for the gui
        @param direction the direction value for the gui
        @param wheelSensorType the enum which identifies the sensordata
        */
        void OnWheelData(ulong tach, char direction, int wheelSensorType);
    
        /*! slot for receiving the intertial measurement unit data to the gui widget
        @param a_x the acceleration in x-axis
        @param a_y the acceleration in y-axis
        @param a_z the acceleration in z-axis
        @param yaw the yaw angle
        @param pitch the pitch angle
        @param roll the roll angle
        */
        void OnInerMeasUnitData(float a_x, float a_y, float a_z, float yaw, float pitch, float roll);

    private:

        /*! this functions inits all the geometrics of the ultrasonic sensors */ 
        void initUsGeometrics();

        /*! this functions inits all the ultrasonic beams to their maximum values and draws them in the gui */
        void initUsBeams();

        /*! this function deletes the old graphicsitem and draws a new one with the new values
        @param pUsBeamItem pointer to the item which has to be drawn again
        @param sensorGeometrics struct holding the geometrics of the sensor to be drawn
        @param relValue the relative value (relative to max value in sensorGeometrics)
        @param brush the brush of the beam to be drawn
        */
        void drawUsBeam(QGraphicsEllipseItem* &pUsBeamItem, usSensorGeometrics sensorGeometrics, tFloat32 f32RelValue, QBrush brush = Qt::darkGreen);
        
        /*! initialize the widgets for the wheel encoder data */
        void createWheelWidgets();

        /*! initialize the widgets for the ultrasonic data */
        void createUltrasonicWidgets();

        /*! initialize the widgets for the gyroscope data */
        void createGyroscopeWidgets();
        
        /*! initialize the widgets for the voltage data */
        void createVoltageWidgets();

        /*! the main widget */
        QWidget* m_pWidget;
        
        /*! the main font for the widget */
        QFont* m_mainFont;

        /*! the smaller main font for the tableviews etc */
        QFont* m_mainFontSmall;

        /*! graphicssene for visualization*/ 
        QGraphicsScene *m_scene;

        /*! the main layout of the widget */
        QHBoxLayout* m_mainLayout;

        /*! the beam to visualize the us beam front center */
        QGraphicsEllipseItem *m_usBeamFrontCenter;
        
        /*! the beam to visualize the us beam front center */
        QGraphicsEllipseItem *m_usBeamFrontCenterLeft;
        
        /*! the beam to visualize the us beam front center */
        QGraphicsEllipseItem *m_usBeamFrontCenterRight;
                
        /*! the beam to visualize the us beam front left */
        QGraphicsEllipseItem *m_usBeamFrontLeft;

        /*! the beam to visualize the us beam front right */
        QGraphicsEllipseItem *m_usBeamFrontRight;
        
        /*! the beam to visualize the us beam rear center */
        QGraphicsEllipseItem *m_usBeamRearCenter;
                
        /*! the beam to visualize the us beam rear left */
        QGraphicsEllipseItem *m_usBeamRearLeft;

        /*! the beam to visualize the us beam rear right */
        QGraphicsEllipseItem *m_usBeamRearRight;

        /*! the beam to visualize the us beam right */
        QGraphicsEllipseItem *m_usBeamSideRight;

        /*! the beam to visualize the us beam left */
        QGraphicsEllipseItem *m_usBeamSideLeft;
        
        /*! the x coordinate of the car in the graphicsscene */
        const qreal m_qPCarCenter_x;

        /*! the y coordinate of the car in the graphicsscene */
        const qreal m_qPCarCenter_y;

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsFrontCenter;        

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsFrontCenterLeft;

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsFrontCenterRight;

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsFrontRight;

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsFrontLeft;

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsRearCenter;

        /*! struct with the geometrics o-f the ultrasonic sensor rear right */
        usSensorGeometrics m_stUsRearRight;

        /*! struct with the geometrics of the ultrasonic sensor front center */
        usSensorGeometrics m_stUsRearLeft;
        
        /*! struct with the geometrics of the ultrasonic sensor right side */
        usSensorGeometrics m_stUsSideRight;
        
        /*! struct with the geometrics of the ultrasonic sensor left side */
        usSensorGeometrics m_stUsSideLeft;

        /*! the table view for the ultrasonic sensor data */
        QTableView* m_usTableView;

        /*! the item model for the ultrasonic sensor data */
        QStandardItemModel *m_usItemModel;

        /*! the table view for the wheel sensor data */
        QTableView* m_wheelTableView;

        /*! the item model for the wheel sensor data */
        QStandardItemModel *m_wheelItemModel;

        /*! the table view for the voltage data */
        QTableView* m_voltageTableView;

        /*! the item model for the voltage data */
        QStandardItemModel *m_voltageItemModel;

        /*! the table view for the gyro sensor data */
        QTableView* m_imuTableView;

        /*! the item model for the gyro sensor data */
        QStandardItemModel *m_imuItemModel;
        
};

#endif
