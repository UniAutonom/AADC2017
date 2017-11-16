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

#include "stdafx.h"
#include "displaywidget.h"
#include "cVisualization.h"

DisplayWidget::DisplayWidget(QWidget* parent) :
QWidget(parent), m_qPCarCenter_x(GRAPHICSSCENE_WIDTH/2), m_qPCarCenter_y(GRAPHICSSCENE_HEIGHT/2) 
{
// init the ultrasound geometrics
    initUsGeometrics();
    
// initialize the main widget
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);		
    m_pWidget->setFixedSize(500,200);

//initialize the fonts
    m_mainFont = new QFont("Arial",12);
    m_mainFontSmall = new QFont("Arial",10);
    setFont(*m_mainFont);
// create new QGraphicsScene
    m_scene = new QGraphicsScene(this);
    
//create car item (blue rect)
    m_scene->addRect(QRectF(GRAPHICSSCENE_WIDTH/2-CAR_WIDTH/2,GRAPHICSSCENE_HEIGHT/2-CAR_HEIGHT/2,CAR_WIDTH,CAR_HEIGHT),QPen(Qt::gray),QBrush(Qt::gray));	
        
// create new main graphicsView for graphicscene
    QGraphicsView *mainGraphicsView = new QGraphicsView(m_scene,this);
    mainGraphicsView->setMinimumSize(GRAPHICSSCENE_WIDTH*1.1,GRAPHICSSCENE_HEIGHT*1.1);
    mainGraphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mainGraphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
// init the ultrasound geometrics (sensor positions etc)
     initUsGeometrics();

//create the widgets for the ultrasonic sensors, wheel encoder, voltage sensor and the gyroscope
    createUltrasonicWidgets();

    createWheelWidgets();

    createVoltageWidgets();

    createGyroscopeWidgets();

// create title
    QLabel *titleLable = new QLabel("AADC Visualization",this);
    titleLable->setFont(QFont("Arial",25));
    titleLable->setWordWrap(true);
    titleLable->setAlignment(Qt::AlignCenter);

// create the vertical layout for the second column
    QVBoxLayout* layoutcolumn_second = new QVBoxLayout(); 
    layoutcolumn_second->setSizeConstraint(QLayout::SetMinimumSize);
    layoutcolumn_second->addWidget(m_usTableView,0,Qt::AlignCenter);    
    layoutcolumn_second->addWidget(m_wheelTableView,0,Qt::AlignCenter);    
    layoutcolumn_second->addWidget(m_voltageTableView,0,Qt::AlignCenter);  

// create the vertical layout for the third column
    QVBoxLayout* layoutcolumn_third = new QVBoxLayout(); 
    layoutcolumn_third->addWidget(titleLable,0,Qt::AlignCenter);
    layoutcolumn_third->addWidget(m_imuTableView,0,Qt::AlignCenter);


// create main layout of widget
    m_mainLayout = new QHBoxLayout();
    m_mainLayout->addWidget(mainGraphicsView,0,Qt::AlignCenter);
    m_mainLayout->addLayout(layoutcolumn_second);
    m_mainLayout->addLayout(layoutcolumn_third);
    setLayout(m_mainLayout);	


// draw init ultrasonic beams
    initUsBeams();

}

DisplayWidget::~DisplayWidget()
{
    m_scene->clear();
}


void DisplayWidget::drawUsBeam(QGraphicsEllipseItem* &pUsBeamItem, usSensorGeometrics sensorGeometrics, tFloat32 f32RelValue, QBrush brush)
{
    if (pUsBeamItem!=NULL) 
    {
        m_scene->removeItem(pUsBeamItem);
        delete pUsBeamItem;    
        pUsBeamItem = NULL;
    }

    if (f32RelValue>sensorGeometrics.f32MaxValue) f32RelValue = sensorGeometrics.f32MaxValue;
    int beamSize = int(US_BEAM_DIM * f32RelValue/sensorGeometrics.f32MaxValue);
    pUsBeamItem = new QGraphicsEllipseItem(m_qPCarCenter_y-beamSize/2-sensorGeometrics.iYPos,m_qPCarCenter_x-beamSize/2-sensorGeometrics.iXPos,beamSize,beamSize);
    pUsBeamItem->setBrush(brush);
    pUsBeamItem->setPen(QPen(Qt::transparent));
    pUsBeamItem->setStartAngle((sensorGeometrics.iStartAngle+75)*16);
    pUsBeamItem->setSpanAngle(US_BEAM_SPAN_ANGLE*16);
    m_scene->addItem(pUsBeamItem);
}

void DisplayWidget::initUsGeometrics()
{
    // initialization of the us front center geometrics
    m_stUsFrontCenter.f32MaxValue = 400;
    m_stUsFrontCenter.iStartAngle = US_FRONT_CENTER_ZROT;
    m_stUsFrontCenter.iXPos = US_FRONT_CENTER_XPOS;
    m_stUsFrontCenter.iYPos = US_FRONT_CENTER_YPOS;

    // initialization of the us front center left geometrics
    m_stUsFrontCenterLeft.f32MaxValue = 400;
    m_stUsFrontCenterLeft.iStartAngle =US_FRONT_CENTER_LEFT_ZROT;
    m_stUsFrontCenterLeft.iXPos = US_FRONT_CENTER_LEFT_XPOS;
    m_stUsFrontCenterLeft.iYPos = US_FRONT_CENTER_LEFT_YPOS;
    
    // initialization of the us front center right geometrics
    m_stUsFrontCenterRight.f32MaxValue = 400;
    m_stUsFrontCenterRight.iStartAngle =US_FRONT_CENTER_RIGHT_ZROT;
    m_stUsFrontCenterRight.iXPos = US_FRONT_CENTER_RIGHT_XPOS;
    m_stUsFrontCenterRight.iYPos = US_FRONT_CENTER_RIGHT_YPOS;
    
    // initialization of the us front left geometrics
    m_stUsFrontLeft.f32MaxValue = 400;
    m_stUsFrontLeft.iStartAngle =US_FRONT_LEFT_ZROT;
    m_stUsFrontLeft.iXPos = US_FRONT_LEFT_XPOS;
    m_stUsFrontLeft.iYPos = US_FRONT_LEFT_YPOS;
    
    // initialization of the us front right geometrics
    m_stUsFrontRight.f32MaxValue = 400;
    m_stUsFrontRight.iStartAngle = US_FRONT_RIGHT_ZROT;
    m_stUsFrontRight.iXPos = US_FRONT_RIGHT_XPOS;
    m_stUsFrontRight.iYPos = US_FRONT_RIGHT_YPOS;

    // initialization of the us rear center geometrics
    m_stUsRearCenter.f32MaxValue = 400;
    m_stUsRearCenter.iStartAngle = US_REAR_CENTER_ZROT ;
    m_stUsRearCenter.iXPos = US_REAR_CENTER_XPOS;
    m_stUsRearCenter.iYPos = US_REAR_CENTER_YPOS;
    
    // initialization of the us rear left geometrics
    m_stUsRearLeft.f32MaxValue = 400;
    m_stUsRearLeft.iStartAngle = US_REAR_LEFT_ZROT;
    m_stUsRearLeft.iXPos = US_REAR_LEFT_XPOS;
    m_stUsRearLeft.iYPos = US_REAR_LEFT_YPOS;
    
    // initialization of the us rear right geometrics
    m_stUsRearRight.f32MaxValue = 400;
    m_stUsRearRight.iStartAngle = US_REAR_RIGHT_ZROT;
    m_stUsRearRight.iXPos = US_REAR_RIGHT_XPOS;
    m_stUsRearRight.iYPos = US_REAR_RIGHT_YPOS;
    
    // initialization of the right side geometrics
    m_stUsSideRight.f32MaxValue = 400;
    m_stUsSideRight.iStartAngle = US_SIDE_RIGHT_ZROT;
    m_stUsSideRight.iXPos = US_SIDE_RIGHT_XPOS;
    m_stUsSideRight.iYPos = US_SIDE_RIGHT_YPOS;

    // initialization of the left side geometrics
    m_stUsSideLeft.f32MaxValue = 400;
    m_stUsSideLeft.iStartAngle = US_SIDE_LEFT_ZROT;
    m_stUsSideLeft.iXPos = US_SIDE_LEFT_XPOS;
    m_stUsSideLeft.iYPos = US_SIDE_LEFT_YPOS;
}

void DisplayWidget::initUsBeams()
{
    // init us_front_beam center
    m_usBeamFrontCenter = NULL;
    QGraphicsEllipseItem *backGroundEllipseItemNull;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsFrontCenter,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamFrontCenter,m_stUsFrontCenter,m_stUsFrontCenter.f32MaxValue);    

    // init us_front_beam center
    m_usBeamFrontCenterLeft = NULL;    
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsFrontCenterLeft,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamFrontCenterLeft,m_stUsFrontCenterLeft,m_stUsFrontCenterLeft.f32MaxValue);    
   
    // init us_front_beam center
    m_usBeamFrontCenterRight = NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsFrontCenterRight,m_stUsFrontCenterRight.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamFrontCenterRight,m_stUsFrontCenterRight,m_stUsFrontCenterRight.f32MaxValue);    
    
    // init us_front_beam left
    m_usBeamFrontLeft = NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsFrontLeft,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamFrontLeft,m_stUsFrontLeft,m_stUsFrontLeft.f32MaxValue);    
    
    // init us_front_beam right
    m_usBeamFrontRight = NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsFrontRight,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamFrontRight,m_stUsFrontRight,m_stUsFrontRight.f32MaxValue);    

    // init us_rear _beam center
    m_usBeamRearCenter = NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsRearCenter,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamRearCenter,m_stUsRearCenter,m_stUsRearCenter.f32MaxValue);    
   
    // init us_rear_beam left
    m_usBeamRearLeft = NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsRearLeft,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamRearLeft,m_stUsRearLeft,m_stUsRearLeft.f32MaxValue);    
    
    // init us_rear_beam right
    m_usBeamRearRight = NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsRearRight,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamRearRight,m_stUsRearRight,m_stUsRearRight.f32MaxValue);   

    // init us_right
    m_usBeamSideRight =NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsSideRight,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamSideRight,m_stUsSideRight,m_stUsSideRight.f32MaxValue);
    
    // init us_left
    m_usBeamSideLeft =NULL;
    backGroundEllipseItemNull = NULL;
    drawUsBeam(backGroundEllipseItemNull,m_stUsSideLeft,m_stUsFrontCenter.f32MaxValue, Qt::darkGray);   
    drawUsBeam(m_usBeamSideLeft,m_stUsSideLeft,m_stUsSideLeft.f32MaxValue);
}

void DisplayWidget::createUltrasonicWidgets()
{
    // create the table view and the item model for the ultrasonic sensors
    m_usItemModel = new QStandardItemModel(10,3,this);
    m_usTableView = new QTableView(this);
    m_usTableView->setModel(m_usItemModel);
    m_usTableView->resizeRowsToContents();
    m_usTableView->verticalHeader()->setVisible(false);
    m_usTableView->setColumnWidth(1,50);
    m_usTableView->setColumnWidth(2,40);
    m_usTableView->setFixedWidth(m_usTableView->columnWidth(0)+m_usTableView->columnWidth(1)+m_usTableView->columnWidth(2));
    m_usTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_usTableView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);    
    m_usTableView->setFixedHeight(m_usTableView->rowHeight(0)*10+m_usTableView->horizontalHeader()->height());
    
    QStringList header;
    header << "Ultrasonic" << "Value" << "Unit";
    m_usItemModel->setHorizontalHeaderLabels(header);
    m_usItemModel->setData(m_usItemModel->index(0,0),"Front Left");
    m_usItemModel->setData(m_usItemModel->index(1,0),"Front Mid Left");
    m_usItemModel->setData(m_usItemModel->index(2,0),"Front Mid");
    m_usItemModel->setData(m_usItemModel->index(3,0),"Front Mid Right");
    m_usItemModel->setData(m_usItemModel->index(4,0),"Front Right");
    m_usItemModel->setData(m_usItemModel->index(5,0),"Side Left");
    m_usItemModel->setData(m_usItemModel->index(6,0),"Side Right");
    m_usItemModel->setData(m_usItemModel->index(7,0),"Rear Left");
    m_usItemModel->setData(m_usItemModel->index(8,0),"Rear Mid");
    m_usItemModel->setData(m_usItemModel->index(9,0),"Rear Right");
    for (int i=0; i<=9;i++)
        m_usItemModel->setData(m_usItemModel->index(i,2),"cm");
};

void DisplayWidget::createWheelWidgets()
{
    // create the table view and the item model for the wheel sensors
    m_wheelItemModel = new QStandardItemModel(4,3,this);
    m_wheelTableView = new QTableView(this);
    m_wheelTableView->setModel(m_wheelItemModel);
    m_wheelTableView->resizeRowsToContents();
    m_wheelTableView->verticalHeader()->setVisible(false);
    m_wheelTableView->setColumnWidth(1,50);
    m_wheelTableView->setColumnWidth(2,40);
    m_wheelTableView->setFixedWidth(m_wheelTableView->columnWidth(0)+m_wheelTableView->columnWidth(1)+m_wheelTableView->columnWidth(2));
    m_wheelTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_wheelTableView->setFixedHeight(m_wheelTableView->rowHeight(1)*4+m_wheelTableView->horizontalHeader()->height());
    m_wheelTableView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    
    QStringList header;
    header << "Wheels" << "Value" << "Unit";
    m_wheelItemModel->setHorizontalHeaderLabels(header);
    m_wheelItemModel->setData(m_wheelItemModel->index(0,0), "dir left");
    m_wheelItemModel->setData(m_wheelItemModel->index(1,0), "dir right");   
    m_wheelItemModel->setData(m_wheelItemModel->index(2,0), "counter left");
    m_wheelItemModel->setData(m_wheelItemModel->index(3,0), "counter right");
}

void DisplayWidget::createGyroscopeWidgets()
{
    // create the table view and the item model for the gyrocscope
    m_imuItemModel = new QStandardItemModel(6,3,this);
    m_imuTableView = new QTableView(this);
    m_imuTableView->setModel(m_imuItemModel);
    m_imuTableView->resizeRowsToContents();
    m_imuTableView->verticalHeader()->setVisible(false);
    m_imuTableView->setColumnWidth(1,50);
    m_imuTableView->setColumnWidth(2,50);
    m_imuTableView->setFixedWidth(m_imuTableView->columnWidth(0)+m_imuTableView->columnWidth(1)+m_imuTableView->columnWidth(2));
    m_imuTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_imuTableView->setFixedHeight(m_imuTableView->rowHeight(1)*6+m_imuTableView->horizontalHeader()->height());
    m_imuTableView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    
    QStringList header;
    header << "IMU" << "Value"<< "Unit";;
    m_imuItemModel->setHorizontalHeaderLabels(header);
    m_imuItemModel->setData(m_imuItemModel->index(0,0), "acceleration x");
    m_imuItemModel->setData(m_imuItemModel->index(1,0), "acceleration y");
    m_imuItemModel->setData(m_imuItemModel->index(2,0), "acceleration z");
    m_imuItemModel->setData(m_imuItemModel->index(3,0), "yaw");
    m_imuItemModel->setData(m_imuItemModel->index(4,0), "pitch");
    m_imuItemModel->setData(m_imuItemModel->index(5,0), "roll");
    m_imuItemModel->setData(m_imuItemModel->index(3,2), QString::fromUtf8("°"));
    m_imuItemModel->setData(m_imuItemModel->index(4,2), QString::fromUtf8("°"));
    m_imuItemModel->setData(m_imuItemModel->index(5,2), QString::fromUtf8("°"));    
    m_imuItemModel->setData(m_imuItemModel->index(0,2), QString::fromUtf8("m/sec2"));
    m_imuItemModel->setData(m_imuItemModel->index(1,2), QString::fromUtf8("m/sec2"));
    m_imuItemModel->setData(m_imuItemModel->index(2,2), QString::fromUtf8("m/sec2"));
}

void DisplayWidget::createVoltageWidgets()
{
    
// create the table view and the item model for the voltage sensors
    m_voltageItemModel = new QStandardItemModel(2,3,this);
    m_voltageTableView = new QTableView(this);
    m_voltageTableView->setModel(m_voltageItemModel);
    m_voltageTableView->resizeRowsToContents();
    m_voltageTableView->verticalHeader()->setVisible(false);
    m_voltageTableView->setColumnWidth(1,50);
    m_voltageTableView->setColumnWidth(2,40);
    m_voltageTableView->setFixedWidth(m_voltageTableView->columnWidth(0)+m_voltageTableView->columnWidth(1)+m_voltageTableView->columnWidth(2));
    m_voltageTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_voltageTableView->setFixedHeight(m_voltageTableView->rowHeight(1)*2+m_voltageTableView->horizontalHeader()->height());
    m_voltageTableView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);    
    
    QStringList header;
    header << "Voltage" << "Value"<< "Unit";;
    m_voltageItemModel->setHorizontalHeaderLabels(header);
    m_voltageItemModel->setData(m_voltageItemModel->index(0,0), "measurement");
    m_voltageItemModel->setData(m_voltageItemModel->index(0,2), "V");
    m_voltageItemModel->setData(m_voltageItemModel->index(1,0), "speed contr");    
    m_voltageItemModel->setData(m_voltageItemModel->index(1,2), "V");
}


void DisplayWidget::OnUltrasonicData(float value, int usSensor)
{
    switch (usSensor)
    {
        case SensorDefinition::usFrontLeft:
            m_usItemModel->setData(m_usItemModel->index(0,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamFrontLeft,m_stUsFrontLeft,value);   
            break;
        case SensorDefinition::usFrontCenterLeft:
            m_usItemModel->setData(m_usItemModel->index(1,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamFrontCenterLeft,m_stUsFrontCenterLeft,value);   
            break;
        case SensorDefinition::usFrontCenter:
            m_usItemModel->setData(m_usItemModel->index(2,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamFrontCenter,m_stUsFrontCenter,value);   
            break;
        case SensorDefinition::usFrontCenterRight:
            m_usItemModel->setData(m_usItemModel->index(3,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamFrontCenterRight,m_stUsFrontCenterRight,value);   
            break;
        case SensorDefinition::usFrontRight:
            m_usItemModel->setData(m_usItemModel->index(4,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamFrontRight,m_stUsFrontRight,value);   
            break;
        case SensorDefinition::usSideLeft:
            m_usItemModel->setData(m_usItemModel->index(5,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamSideLeft,m_stUsSideLeft,value);   
            break;
        case SensorDefinition::usSideRight:
            m_usItemModel->setData(m_usItemModel->index(6,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamSideRight,m_stUsSideRight,value);   
            break;
        case SensorDefinition::usRearLeft:
            m_usItemModel->setData(m_usItemModel->index(7,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamRearLeft,m_stUsRearLeft,value);   
            break;
        case SensorDefinition::usRearCenter:
            m_usItemModel->setData(m_usItemModel->index(8,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamRearCenter,m_stUsRearCenter,value);   
            break;
        case SensorDefinition::usRearRight:
            m_usItemModel->setData(m_usItemModel->index(9,1),QString::number(value,'f',2));
            drawUsBeam(m_usBeamRearRight,m_stUsRearRight,value);   
            break;
    }
};

 void DisplayWidget::OnVoltageData(float value, int voltageSensor)
{
    switch (voltageSensor)
    {
        case SensorDefinition::measurementCircuit:
            m_voltageItemModel->setData(m_voltageItemModel->index(0,1),QString::number(value,'f',2));
            break;
        case SensorDefinition::speedCntrlCircuit:
            m_voltageItemModel->setData(m_voltageItemModel->index(1,1),QString::number(value,'f',2));
            break;
    }
 }

void DisplayWidget::OnWheelData(ulong tach, char direction, int wheelSensorType)
{
    switch (wheelSensorType)
    {
        case SensorDefinition::wheelLeft:
            m_wheelItemModel->setData(m_wheelItemModel->index(0,1),QString::number(direction,'f',2));
             m_wheelItemModel->setData(m_wheelItemModel->index(2,1),QString::number(tach,'f',2));
            break;
        case SensorDefinition::wheelRight:
            m_wheelItemModel->setData(m_wheelItemModel->index(1,1),QString::number(direction,'f',2));
             m_wheelItemModel->setData(m_wheelItemModel->index(3,1),QString::number(tach,'f',2));
            break;
    }
}

void DisplayWidget::OnInerMeasUnitData(float a_x, float a_y, float a_z, float yaw, float pitch, float roll)
{
    m_imuItemModel->setData(m_imuItemModel->index(0,1), QString::number(a_x,'f',2));
    m_imuItemModel->setData(m_imuItemModel->index(1,1), QString::number(a_y,'f',2));
    m_imuItemModel->setData(m_imuItemModel->index(2,1), QString::number(a_z,'f',2));
    m_imuItemModel->setData(m_imuItemModel->index(3,1), QString::number(yaw,'f',2));
    m_imuItemModel->setData(m_imuItemModel->index(4,1), QString::number(pitch,'f',2));
    m_imuItemModel->setData(m_imuItemModel->index(5,1), QString::number(roll,'f',2));
}




