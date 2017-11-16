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

#ifndef CALIWIDGET_FILTER
#define CALIWIDGET_FILTER

#include "stdafx.h"


/*! A the widget of steering calibration filter */
class cCalibrationView: public QWidget
{
    Q_OBJECT

public:
    /*! constructor for the widget
    * \param model the standarditem model to use
    * \param parent the parent widget
    */
    cCalibrationView(QStandardItemModel *model,QWidget* parent = 0);

    /*! default destructor */
    virtual ~cCalibrationView();
signals:
    /*! start a section of calibration
    * \param index the index of section to start
    */
    void startDrive(int index);

private:
    /*! the main layout for the widget */
    QVBoxLayout   *m_pMainLayout;
    /*! the main table view */
    QTableView *m_tableView;
    /*! the main item model */
    QStandardItemModel *m_model;
    /*! push button */
    QPushButton *m_storeCalibrationXMLButton;

private slots:

    /*!
     * slot for pushbutton was pressed.
     *
     * \param   index   Zero-based index of the.
     */
    void onPushButtonClicked(int index);


    /*!
    * slot for store xml was pressed
    */
    void onStoreXMLButtonClicked();
};

#endif //CALIWIDGET_FILTER
