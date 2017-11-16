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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef DISPWIDGET_CAMERACALIBRATION_FILTER
#define DISPWIDGET_CAMERACALIBRATION_FILTER



#include "stdafx.h"

/*! display widget for camera calibration filter
*/
class DisplayWidget: public QWidget
{
    Q_OBJECT

public:
    /*! constructor for the widget
    * \param parent the parent widget
    */
    DisplayWidget(QWidget* parent);
    ~DisplayWidget() {};

    /*! button to start calibration */
    QPushButton *m_btStart;

    /*! button to start calibration for fisheye camera */
    QPushButton *m_btStartFisheye;

public slots:
    /*! receives the image of the current media sample from the filter
    * \param newImage the image
    */
    void OnNewImage(const QImage &newImage);
    /*! receives the state from the filter
    * \param state the state with the enum
    */
    void OnSetState(int state);

    /*! slot called by the button*/
    void OnSaveAs();

signals:
    /*! signal for saving the file
    * \param filename the filename including path where to save
    */
    void SendSaveAs(QString filename);

private:
    /*! the main widget */
    QWidget* m_pWidget;

    /*! the main layout for the widget*/
    QHBoxLayout *m_mainLayout;

    /*! button to save calibration values */
    QPushButton *m_btSaveAs;

    /*! label which shows the video */
    QLabel *m_lblVideoPixmap;

};

#endif
