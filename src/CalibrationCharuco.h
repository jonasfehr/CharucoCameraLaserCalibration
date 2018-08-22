//
//  CalibrationCharuco.h
//  CharucoCameraLaserCalibration
//
//  Created by WandernderPunkt on 22.08.18.
//

#ifndef CalibrationCharuco_h
#define CalibrationCharuco_h

#include "ofMain.h"
#include "ofxCv.h"
#include <opencv2/aruco/charuco.hpp>

class CalibrationCharuco : public ofxCv::Calibration {
public:
    cv::Mat markersDetectedMat;
    cv::Mat cameraCalibratonMat;
    
    void setupBoards();
    
//    bool findBoard(cv::Mat img, std::vector<cv::Point2f>& pointBuf, bool refine);
    
    bool arucoMarkersDetectCollect(cv::Mat img, cv::Mat & imageOut);
    
    bool arucoBoardDetect(cv::Mat image, cv::Mat & imageOut);
    
    bool calibrateCamera(cv::Mat img, cv::Mat & imageOut);
    
    void reset();

    void setMarkerSize(float markerSize);
    float getMarkerSize() const;
    
    bool add(cv::Mat img);
    void calibrate();
    int size();
    void clean();
    
private:
    float markerSize;
    
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
    cv::Ptr<cv::aruco::Board> board;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    vector< vector< vector< cv::Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< cv::Mat > allImgs;
    
    cv::Mat cameraMatrix;
//    cv::Mat distCoeffs;
    
    float axisLength;
    
    double totalTime;
    int totalIterations;
};

//    if(calibration.add(camMat)) {
//        cout << "re-calibrating" << endl;
//        calibration.calibrate();
//        if(calibration.size() > startCleaning) {
//            calibration.clean();
//        }
//        calibration.save("calibrationCam.yml");
//        lastTime = ofGetElapsedTimef();
//    }

#endif /* CalibrationCharuco_h */
