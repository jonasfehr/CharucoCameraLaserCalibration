#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxIpCamStreamer.h"
#include "ofxGui.h"
#include <opencv2/aruco/charuco.hpp>
#include "CalibrationCharuco.h"


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
    
    CalibrationCharuco calibration;
    
//    ofxCv::Calibration calibration;
    

    bool updateDiffMean();
    
    enum Status{
        CALIBRATE_CAMERA,
        CALIBRATE_PROJECTOR
    } states;

};
