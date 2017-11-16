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


#ifndef _CAMERACALIBRATIONFILTER_HEADER
#define _CAMERACALIBRATIONFILTER_HEADER
#define OID_ADTF_CAMERACALIBRATION

#include "stdafx.h"
#include "calibrationSettings.h"
#include "displaywidget.h"

#define OID_ADTF_FILTER_DEF "adtf.aadc.cameraCalibration" //unique for a filter
#define ADTF_FILTER_DESC "AADC Camera Calibration"  //this appears in the Component Tree in ADTF
#define ADTF_FILTER_VERSION_SUB_NAME "cCameraCalibration"//must match with accepted_version_...
#define ADTF_FILTER_VERSION_ACCEPT_LABEL "accepted_version"//sets the version entry
#define ADTF_FILTER_VERSION_STRING "1.0.0"//version string
#define ADTF_FILTER_VERSION_Major 1//this values will be compared, major version change - will not work
#define ADTF_FILTER_VERSION_Minor 0//change will work but notice
#define ADTF_FILTER_VERSION_Build 0//change will work but notice
//the detailed description of the filter
#define ADTF_FILTER_VERSION_LABEL "A camera calibration filter\n$Rev:: 62948"


/*!
* @defgroup CameraCalibration Camera Calibration
* @{
* \image html CameraCalibration.PNG "Plugin CameraCalibration"
*
* This filter can be used for doing the intrinsic calibration for any used camera. After starting a GUI is shown which displays the video stream and has three buttons for starting the calibration or saving the file.
* The calibration is done with the standard opencv function for intrinsic calibration described in the opencv documentation. The main functions are used from opencv samples in (opencv/samples/cpp/calibration.cpp). For further information have look in this documentation.
* \li To perform the calibration with this ADTF Filter a calibration pattern has to be printed first. A standard chessboard pattern is located in src\aadcDemo\camera\AADC_CameraCalibration\pattern.png  and has to be printed on an A3 paper. It is better to use a thick paper or stick the paper to solid background.
* \li The side length of one square of the printed chessboard pattern has to be measured and the length be set in the property Square Size in the Filter (unit is meter).
* \li After that the RGB Output of any camera filter has to be connected to the input pin of this filter and the ADTF configuration has to be started.
* \li Depending on the camera connected to plugin select "Start Calibration Fisheye" or "Start Calibration"
* \li Now the filter searches for the selected pattern in the image. If it was found it is shown in the windows and the points are saved as valid dataset. For each calibration routine a certain number of valid data sets is necessary (can be set in the filter properties)
* \li If the ammount of valid datasets are captured the button "Save Calibration File" can be pressed. The calibration will be checked and if it has a valid result it will be saved to the selected destination.
* If the calibration result can not be used just have another try or modify the number of datasets or the delay between.
* \li The resulting calibration file can be used for the plugins which need an intrnisic calibration file
*
* \image html CameraCalibrationGUI.PNG "GUI of Plugin"

* \image html CameraCalibrationFoundChessboard.PNG "Detected Chessboard of Plugin"

* \b Dependencies \n
* This plugin needs the following libraries:
* \li QT   v.4.7.1
*
* <b> Plugin Properties</b> \n
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Width [number of squares]<td>The number of squares in horizontal axis (Range: 5 to 15)<td>8
* <tr><td>Height [number of squares]<td>The number of squares in vertical axis (Range: 5 to 15)<td>6
* <tr><td>Square Size<td>Square size (length of one side) in meters<td>0.025
* <tr><td>Aspect Ratio<td>Fix aspect ratio (fx/fy) (1 by default)<td>1
* <tr><td>Calibration Pattern<td>Defines the pattern which is used for calibration<td>Chessboard
* <tr><td>Number of Datasets to use<td>Set the number of datasets which are used for calibration<td>10
* <tr><td>Delay [s]<td>Delay between captured datasets in seconds<td>1
* </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_Input<td>video input from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/camera/AADC_CameraCalibration
* <tr><td>Filename<td>aadc_cameraCalibration.plb
* <tr><td>Version<td>1.0.0
* </table>
*/

/*!
 * This is the main class for the for camera calibration filter
 */
class cCameraCalibration : public QObject, public cBaseQtFilter
{
    /*! This macro does all the plugin setup stuff */
    ADTF_FILTER_VERSION(OID_ADTF_FILTER_DEF,
                        ADTF_FILTER_DESC,
                        adtf::OBJCAT_SensorDevice,
                        ADTF_FILTER_VERSION_SUB_NAME,
                        ADTF_FILTER_VERSION_Major,
                        ADTF_FILTER_VERSION_Minor,
                        ADTF_FILTER_VERSION_Build,
                        ADTF_FILTER_VERSION_LABEL
                       );

    Q_OBJECT

public:
    /*! constructor for cBaslerCamera class
    *    \param __info   [in] This is the name of the filter instance.
    */
    cCameraCalibration(const tChar* __info);

    /*! the destructor for this class
    */
    virtual ~cCameraCalibration();

    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(adtf::IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, adtf::IMediaSample* pMediaSample);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \result Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

protected:
    /*! input Pin for video */
    cVideoPin m_oPinInputVideo;

    /*! Creates the widget instance
    *   \return handle to view
    */
    tHandle CreateView();

    /*! Destroys the widget instance
    *   \result Returns a standard result code.
    */
    tResult ReleaseView();

signals:
    /*! sends the image of the current media sample to the gui
    * \param newImage the image
    */
    void newImage(const QImage& newImage);

    /*! send the state to the gui
    * \param state the state with the enum
    */
    void sendState(int state);

public slots:

    /*! slot for starting the calibration */
    void OnStartCalibration();

    /*! slot for starting the calibration */
    void OnStartCalibrationFisheye();

    /*! slot for saving the file
    * \param qFilename the filename including path where to save
    */
    void OnSaveAs(QString qFilename);
private:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

    /*! the pointer which holds the image data shown in the gui */
    Mat m_matInputImage;

    /*! the timestamp of the last sample to measure the delay*/
    clock_t m_prevTimestamp;

    /*! the image points (i.e. results) of the measurements */
    vector<vector<Point2f> > m_imagePoints;

    /*! the current state of filter */
    tInt8 m_calibrationState;

    /*! the size of the frames*/
    Size m_matrixSize;

    /*! the camera matrix of the calibration */
    Mat m_cameraMatrix;

    /*! the distortion coefficients of the camera*/
    Mat m_distCoeffs;

    /*! if we use fisheye model for calibration*/
    calibrationSettings m_calibrationSettings;

    /*! bitmapformat of input image */
    tBitmapFormat      m_sInputFormat;

    /*! indicates wheter information is printed to the console or not */
    tBool m_bDebugModeEnabled;

    /*! function process the video data
    * \param pSample the new media sample to be processed
    * \result Returns a standard result code.
    */
    tResult ProcessVideo(IMediaSample* pSample);

    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    * \param pFormat the new format for the input and input pin
    * \result Returns a standard result code.
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! returns the current streamtime
    * \return timestamp returns stream time in microseconds
    */
    tTimeStamp GetTime();

    /*! calls the calculation of the matrixes and saving them to the given file
        (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param imageSize refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \result Returns a standard result code.
    */
    tResult runCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                  vector<vector<Point2f> > imagePoints);

    /*! does the calculation
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param imageSize refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \param rvecs refer to opencv/samples/cpp/calibration.cpp
    * \param tvecs refer to opencv/samples/cpp/calibration.cpp
    * \param reprojErrs refer to opencv/samples/cpp/calibration.cpp
    * \param totalAvgErr refer to opencv/samples/cpp/calibration.cpp
    * \result Returns a standard result code.
    */
    tResult runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                           vector<float>& reprojErrs, double& totalAvgErr);

    /*! saves the camera params
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param imageSize refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param rvecs refer to opencv/samples/cpp/calibration.cpp
    * \param tvecs refer to opencv/samples/cpp/calibration.cpp
    * \param reprojErrs refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \param totalAvgErr refer to opencv/samples/cpp/calibration.cpp
    * \result Returns nothing
    */
    tVoid saveCameraParams(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                           const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                           double totalAvgErr);

    /*! computes the reprojection errors
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param objectPoints refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \param rvecs refer to opencv/samples/cpp/calibration.cpp
    * \param tvecs refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param perViewErrors refer to opencv/samples/cpp/calibration.cpp
    * \param fisheye refer to opencv/samples/cpp/calibration.cpp
    * \result reprojection error
    */
    static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors, bool fisheye);

    /*! calculates the chessboard corners
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param boardSize size of board
    * \param squareSize size of square
    * \param corners vector with corners
    * \param patternType type of used pattern
    */
    static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners,
                                      calibrationSettings::Pattern patternType = calibrationSettings::CHESSBOARD);

    /*! this function resets all the calibrations results to start new one
    * \result Returns a standard result code.
    */
    tResult resetCalibrationResults();

};


/*!
*@}
*/


#endif //_CAMERACALIBRATIONFILTER_HEADER
