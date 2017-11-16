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
* $Author:: spie#$  $Date:: 2017-05-18 16:51:13#$ $Rev:: 63512   $
**********************************************************************/


#include "stdafx.h"

#include "cCalibrationModel.h"
#include "angleDiff.h"

cCalibrationModel::cCalibrationModel(QObject *parent): QObject(parent), m_tableViewModel(9,5), m_distance(0), m_yaw(0), m_distanceInital(0), m_yawInital(0), m_distanceLimit(0), m_yawLimit(0), m_isDrivingIndex(-1)
{
    QStringList header;
    header.append("SteerAngle [°]");
    header.append("Distance [m]");
    header.append("Yaw [°]");
    header.append("Radius [m]");
    header.append("");

    m_tableViewModel.setHorizontalHeaderLabels(header);

    int i = 0;
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(60));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(65));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(70));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(80));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(90));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(100));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(110));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(115));
    m_tableViewModel.setData(m_tableViewModel.index(i++,0),QVariant(120));
}

cCalibrationModel::~cCalibrationModel()
{


}

float cCalibrationModel::getSteerAngle()
{
    if(!isDriving()) return 90;

    float angle = m_tableViewModel.data(m_tableViewModel.index(getDriveViewIndex(),0)).toFloat();

    return angle;
}

bool cCalibrationModel::isDriving()
{
    int ret;
    m_driveMutex.LockRead();
    {
        ret = m_isDrivingIndex;
    }
    m_driveMutex.UnlockRead();
    if(ret > -1 )
        return true;
    else
        return false;
}

void cCalibrationModel::setDrivingIndex(int index)
{
    m_driveMutex.LockWrite();
    {
        m_isDrivingIndex = index;
    }
    m_driveMutex.UnlockRead();
}

void cCalibrationModel::onStartDrive(int driveViewIndex)
{

    if(isDriving())
    {
        LOG_INFO("I'm already driving");
        return;
    }
    else
    {
        LOG_INFO("start driving");
    }
    m_distanceInital = getDistance();
    m_yawInital = getYaw();
    setDrivingIndex(driveViewIndex);
}

int cCalibrationModel::getDriveViewIndex()
{
    int ret = -1;
    m_driveMutex.LockRead();
    {
        ret = m_isDrivingIndex;
    }
    m_driveMutex.UnlockRead();
    return ret;
}

float cCalibrationModel::getDeltaYaw()
{
    float delta = angleDiff(m_yawInital,getYaw());
    return delta;
}

float cCalibrationModel::getDeltaDistance()
{
    return getDistance() - m_distanceInital;
}


void cCalibrationModel::updateYaw(const float yaw)
{
    m_yawMutex.LockWrite();
    {
        m_yaw = yaw;
    }
    m_yawMutex.UnlockWrite();
    checkLimits();

    if(isDriving())
    {
        m_tableViewModel.setData(m_tableViewModel.index(getDriveViewIndex(),2),QVariant(getDeltaYaw() * cStdMath::MATH_RAD2DEG));
        updateRadius();
    }

}


float cCalibrationModel::getRadius()
{
    return arcRadius(getDeltaDistance(),getDeltaYaw());
}

void cCalibrationModel::updateRadius()
{
    double radius = arcRadius(getDeltaDistance(),getDeltaYaw());

    m_tableViewModel.setData(m_tableViewModel.index(getDriveViewIndex(),3),QVariant(radius));
}


void cCalibrationModel::updateDistance(const float distance)
{
    m_distanceMutex.LockWrite();
    {
        m_distance = distance;
    }
    m_distanceMutex.UnlockWrite();

    checkLimits();

    if(isDriving())
    {
        m_tableViewModel.setData(m_tableViewModel.index(getDriveViewIndex(),1),QVariant(getDeltaDistance()));
        updateRadius();
    }
}

void cCalibrationModel::checkLimits()
{

    if(isDriving() && m_distanceLimit > 0 && m_distanceLimit < getDeltaDistance())
    {
        //Finished
        setDrivingIndex(-1);
        emit driveFinished(getDeltaDistance(),getDeltaYaw());
        return;
    }
    else if(isDriving() && m_yawLimit > 0 && m_yawLimit < fabs(getDeltaYaw()) /** fabs() don't care for left or right turn**/)
    {
        //Finished
        setDrivingIndex(-1);
        emit driveFinished(getDeltaDistance(),getDeltaYaw());
        return;
    }

}

void cCalibrationModel::setLimits(const float max_distance,const float max_yaw)
{
    m_distanceLimit = max_distance;
    m_yawLimit = max_yaw;
}



float cCalibrationModel::getYaw()
{
    float ret = 0;
    m_yawMutex.LockRead();
    ret = m_yaw;
    m_yawMutex.UnlockRead();
    return ret;
}



float cCalibrationModel::getDistance()
{
    float ret = 0;
    m_distanceMutex.LockRead();
    ret = m_distance;
    m_distanceMutex.UnlockRead();
    return ret;

}


