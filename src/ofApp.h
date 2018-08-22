#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxIpCamStreamer.h"
#include "ofxGui.h"
#include <opencv2/aruco/charuco.hpp>


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    ofxIpCamStreamer ipCam;
    
    // Cam and Mat for Calibration
    ofImage undistortedImg;
    ofImage camImg;
    Mat previous;
    Mat diff;
    Mat camMat;
    Mat markersDetectedMat;
    Mat cameraCalibratonMat;

    float diffMean;
    
    bool calibrateOldSchool();

    float lastTime;
    
//    ofxCv::Calibration calibration;
    
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    cv::Size imgSize;
    Ptr<aruco::CharucoBoard> charucoboard;
    Ptr<aruco::Board> board;
    Ptr<aruco::Dictionary> dictionary;
        
    bool ready;
    
    Mat cameraMatrixCamera;
    Mat distCoeffsCamera;
    Mat cameraMatrix;
    Mat distCoeffs;
    
    int xCount = 7;
    int yCount = 5;
    float squareSize = 0.039;
    float markerSize = 0.0175;

    bool arucoMarkersDetectCollect(Mat image, Mat & imageOut);
    bool calibrateCamera(Mat & imageOut);
    bool arucoBoardDetect(Mat image, Mat & imageOut);

    bool updateDiffMean();
    
    void saveCameraCalibration(const std::string& filename, bool absolute = false) const;
    void loadCameraCalibration(const std::string& filename, bool absolute = false);
    enum Status{
        CALIBRATE_CAMERA,
        CALIBRATE_PROJECTOR
    } states;
    
    // for board detection
    float axisLength;
    
    double totalTime;
    int totalIterations;
    
    ofxCv::Intrinsics distortedIntrinsics;
    ofxCv::Intrinsics undistortedIntrinsics;

    double repError;
    
    void updateUndistortion();
    
    std::vector<std::vector<cv::Point2f> > imagePoints;
    std::vector<std::vector<cv::Point3f> > objectPoints;
    cv::Mat undistortMapX, undistortMapY;

};
