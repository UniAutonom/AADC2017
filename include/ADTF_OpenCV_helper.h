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
* $Author:: spiesra $  $Date:: 2017-05-10 12:58:59#$ $Rev:: 62997   $
**********************************************************************/


/*!
 * converts the openCV type of matrix to ADTF Pixelformat
 * \param cv_mat_type opencv mat type
 * \return the tPixelFormat
 */
static IImage::tPixelFormat CVType2PixelFormat(const int& cv_mat_type);

/*!
* converts the ADTF Pixelformat to openCV type 
*    \param bmp_format_pixelformat the pixelformat opencv mat type
*    \return the opencv type
*/
static int PixelFormat2CVType(const tInt16& bmp_format_pixelformat);

/*!
 * creates a tBitmapFormat from a given openCV Matrix
 * \param cv_mat input cv matrix
 * \param bmp_format output bitmap format
 * \param forcePixelFormat the bitmap format must be this pixelformat
 */
static tResult Mat2BmpFormat(const cv::Mat& cv_mat, tBitmapFormat& bmp_format, IImage::tPixelFormat forcePixelFormat = IImage::PF_UNKNOWN);

/*!
* creates a matrix from a tBitmapFormat
* \param cv_mat output bitmap format
* \param bmp_format input cv matrix
*/
static tResult BmpFormat2Mat(const tBitmapFormat& bmp_format, cv::Mat& cv_mat);

static IImage::tPixelFormat CVType2PixelFormat(const int& cv_mat_type)
{
    switch (cv_mat_type)
    {
    case CV_8UC1:
        return  IImage::PF_GREYSCALE_8;
        break;
    case CV_8UC3:
        return  IImage::PF_RGB_888;
        break;
    case CV_8UC4:
        return  IImage::PF_RGBA_8888;
        break;
    default:
        return  IImage::PF_UNKNOWN;
    };
    return  IImage::PF_UNKNOWN;

}

static int PixelFormat2CVType(const tInt16& bmp_format_pixelformat)
{
    switch (bmp_format_pixelformat)
    {
    case adtf_util::cImage::PF_GREYSCALE_8:
        return CV_8UC1;
        break;
    case adtf_util::cImage::PF_RGB_888:
    case adtf_util::cImage::PF_BGR_888:
        return CV_8UC3;
        break;
    case adtf_util::cImage::PF_RGBA_8888:
    case adtf_util::cImage::PF_BGRA_8888:
    case adtf_util::cImage::PF_ABGR_8888:
    case adtf_util::cImage::PF_ARGB_8888:
        return CV_8UC4;
        break;
    default:
        return -1;
    };
    return -1;
}

static tResult Mat2BmpFormat(const cv::Mat& cv_mat, tBitmapFormat& bmp_format, IImage::tPixelFormat forcePixelFormat)
{
    //generate the mat_format
    bmp_format.nPaletteSize = 0;
    bmp_format.nHeight = cv_mat.rows;
    bmp_format.nWidth = cv_mat.cols;
    bmp_format.nBitsPerPixel = tInt16(cv_mat.elemSize() * 8);//m_current_frame.depth() * m_current_frame.channels();
    bmp_format.nBytesPerLine = tInt32(cv_mat.elemSize() * bmp_format.nWidth);
    bmp_format.nSize = bmp_format.nBytesPerLine * bmp_format.nHeight;
    
    if (forcePixelFormat != IImage::PF_UNKNOWN)
    {
        bmp_format.nPixelFormat = forcePixelFormat;
    }
    else
    {
        bmp_format.nPixelFormat = CVType2PixelFormat(cv_mat.type()); 
        if (bmp_format.nPixelFormat == adtf_util::cImage::PF_UNKNOWN)
        {
            RETURN_ERROR(ERR_INVALID_ARG);
        }
    } 

    RETURN_NOERROR;

}

static tResult BmpFormat2Mat(const tBitmapFormat& bmp_format, cv::Mat& cv_mat)
{
    cv_mat.release();
    int CVType = PixelFormat2CVType(bmp_format.nPixelFormat);

    if (CVType != -1)
    {
        cv_mat.create(cv::Size(bmp_format.nWidth, bmp_format.nHeight), CVType);
        RETURN_NOERROR;
    }

    RETURN_ERROR(ERR_INVALID_ARG);
}