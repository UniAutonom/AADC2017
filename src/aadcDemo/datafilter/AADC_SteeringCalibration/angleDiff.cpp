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

#include "stdafx.h"

#include "angleDiff.h"

double angleDiff(double ang1, double ang2)
{
    double d;
    ang1 = angleWrap(ang1);
    ang2 = angleWrap(ang2);
    d = ang1-ang2;
    if ( d > cStdMath::MATH_PI )
        d -= 2*(double)cStdMath::MATH_PI;
    else if ( d < -(double)cStdMath::MATH_PI )
        d += 2*(double)cStdMath::MATH_PI;
    return d;
}

double angleDiff2(double ang1, double ang2)
{
    double d;
    ang1 = angleWrap2(ang1);
    ang2 = angleWrap2(ang2);
    d = ang1-ang2;
    if ( d > cStdMath::MATH_PI / 2 )
        d -= (double)cStdMath::MATH_PI;
    else if ( d < -(double)cStdMath::MATH_PI/2 )
        d += (double)cStdMath::MATH_PI;
    return d;
}


double angleWrap(double ang)
{
    if (ang > cStdMath::MATH_PI)
        return ang-2*(double)cStdMath::MATH_PI;
    else if (ang <= -(double)cStdMath::MATH_PI)
        return ang+2*(double)cStdMath::MATH_PI;
    else
        return ang;
}

double angleWrap2(double ang)
{
    if (ang > cStdMath::MATH_PI/2)
        return ang-(double)cStdMath::MATH_PI;
    else if (ang <= -(double)cStdMath::MATH_PI/2)
        return ang+(double)cStdMath::MATH_PI;
    else
        return ang;
}

double arcRadius(double arcDistance, double alpha)
{
    //check for div by zero
    if(alpha == 0.0)
    {
        return 0.0;
    }
    alpha = alpha * cStdMath::MATH_RAD2DEG;
    return  (180 * arcDistance)/(cStdMath::MATH_PI * alpha);
}
