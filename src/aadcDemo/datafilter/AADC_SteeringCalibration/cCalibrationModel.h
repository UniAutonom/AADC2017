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
* $Author:: spiesra $  $Date:: 2017-05-18 16:51:13#$ $Rev:: 63512   $
**********************************************************************/

#ifndef C_CALIBRATION_SM_H
#define C_CALIBRATION_SM_H
#include <qobject.h>

/*! A data Model for the calibration. */
class cCalibrationModel : public QObject
{
    Q_OBJECT

public:

    /*!
     * Constructor.
     *
     * \param [in,out]  parent  (Optional) If non-null, the parent.
     */
    cCalibrationModel(QObject *parent = NULL);

    /*! Destructor. */
    virtual ~cCalibrationModel();

    /*!
     * Updates the yaw described by yaw.
     *
     * \param   yaw The yaw.
     */
    void updateYaw(const float yaw);

    /*!
     * Updates the distance described by distance.
     *
     * \param   distance    The distance.
     */
    void updateDistance(const float distance);

    /*!
     * Gets the yaw.
     *
     * \return  The yaw.
     */
    float getYaw();

    /*!
     * Gets steer angle.
     *
     * \return  The steer angle.
     */
    float getSteerAngle();

    /*!
     * Gets the distance.
     *
     * \return  The distance.
     */
    float getDistance();

    /*!
     * Gets delta yaw angle
     *
     * \return  The delta yaw.
     */
    float getDeltaYaw();

    /*!
     * Gets delta distance.
     *
     * \return  The delta distance.
     */
    float getDeltaDistance();

    /*!
     * Gets the radius.
     *
     * \return  The radius.
     */
    float getRadius();

    /*!
     * Sets the limits.
     *
     * \param   max_distance    The maximum distance.
     * \param   max_yaw         The maximum yaw.
     */
    void setLimits(const float max_distance,const float max_yaw);

    /*!
     * Query if this object is driving.
     *
     * \return  True if driving, false if not.
     */
    bool isDriving();

    /*!
     * Sets driving index.
     *
     * \param   index   Zero-based index of the.
     */
    void setDrivingIndex(int index);

    /*!
     * Gets drive view index.
     *
     * \return  The drive view index.
     */
    int getDriveViewIndex();

    /*! The table view model */
    QStandardItemModel m_tableViewModel;
public slots:

    /*!
     * Executes the start drive action.
     *
     * \param   driveViewIndex  Zero-based index of the drive view.
     */
    void onStartDrive(int driveViewIndex);
signals:

    /*!
     * sets drive finished.
     *
     * \param   max_distance    The maximum distance.
     * \param   max_yaw         The maximum yaw.
     */
    void driveFinished(float max_distance, float max_yaw);

private:
    /*! Check limits. */
    void checkLimits();

    /*! Updates the radius. */
    void updateRadius();

    /*! The distance as member variable */
    float m_distance;

    /*! The distance mutex */
    cReadWriteMutex m_distanceMutex;

    /*! The current yaw angle  */
    float m_yaw;

    /*! The yaw mutex */
    cReadWriteMutex m_yawMutex;

    /*! The inital distance at startup */
    float m_distanceInital;
    /*! The inital yaw angle at startup */
    float m_yawInital;
    /*! The maximum distance limit */
    float m_distanceLimit;
    /*! The maximum yaw limit */
    float m_yawLimit;

    /*! index of the is driving */
    int m_isDrivingIndex;

    /*! The drive mutex */
    cReadWriteMutex m_driveMutex;

};



#endif