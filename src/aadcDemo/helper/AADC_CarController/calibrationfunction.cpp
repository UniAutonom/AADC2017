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

#include "calibrationfunction.h"


CalibrationFunction::CalibrationFunction(QWidget *parent) : QWidget(parent), throttle(0), steering(0)
{
    this->hide();
    QLabel* label = new QLabel(this);
    label->setText("Now you can calibrate your potis.\
                    \nClick Ok to to quit poti calibration");
    QPushButton* button = new QPushButton(this);
    button->setText("Ok");
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(label);
    layout->addWidget(button);

    this->setLayout(layout);


    connect(button, SIGNAL(clicked()), this, SLOT(handleButton()));
    timer.setInterval(50);
    connect(&timer, SIGNAL(timeout()), this, SLOT(update()));
    state = CALIB_POTI;

}

CalibrationFunction::~CalibrationFunction()
{
    timer.stop();
    state = CALIB_POTI;
    steering = 0;
    throttle = 0;
}

void CalibrationFunction::start()
{
    timer.start();
    this->show();
    state = CALIB_POTI;
}

void CalibrationFunction::stop()
{
    timer.stop();
    state = CALIB_POTI;
    steering = 0;
    throttle = 0;

}

void CalibrationFunction::handleButton()
{
    this->hide();
    state = PAUSE;
}

void CalibrationFunction::update()
{
    switch(state)
    {
    case CALIB_POTI:
        emit onSpeed(throttle);
        emit onSteering(steering);
        break;
    case PAUSE:
        stop();
        break;
    default:
        break;
    }
}

