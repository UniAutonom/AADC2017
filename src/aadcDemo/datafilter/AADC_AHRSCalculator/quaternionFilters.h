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
* $Author:: spie#$  $Date:: 2017-05-15 10:33:38#$ $Rev:: 63228   $
**********************************************************************/


//credits to https://github.com/kriswiner/MPU-9250/blob/master/quaternionFilters.ino
//
#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

using namespace std;

/*!
 * Madgwick quaternion update.

 * \param   ax                  The acceleration in x-axis
 * \param   ay                  The acceleration in y-axis
 * \param   az                  The acceleration in z-axis
 * \param   gx                  The gravity in x-axis
 * \param   gy                  The gravity in y-axis
 * \param   gz                  The gravity in z-axis
 * \param   mx                  The rotation rate in x-axis
 * \param   my                  The rotation rate in y-axis
 * \param   mz                  The rotation rate in z-axis
 * \param   deltat  The delta time
 */
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);

/*!
 * Mahony quaternion update.
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
 * \param   deltat  The delta time
 */
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);

/*!
 * Gets the quaternions
 *
 * \return  pointer to quaternion struct.
 */
const float * getQ();

#endif // _QUATERNIONFILTERS_H_
