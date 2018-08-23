#include "ofApp.h"
using namespace ofxCv;
using namespace cv;

const float diffThreshold = 2.5; // maximum amount of movement
const float timeThreshold = 1; // minimum time between snapshots
const int startCleaning = 10; // start cleaning outliers after this many samples

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30);
    ofSetVerticalSync(true);
    ofSleepMillis(100);
    
    camMat = ipCam.get();
    camMat.copyTo(previous);
    camMat.copyTo(diff);
    
    camImg.allocate(camMat.cols, camMat.rows, OF_IMAGE_COLOR);
    undistortedImg.allocate(camMat.cols, camMat.rows, OF_IMAGE_COLOR);
    
    
    
    FileStorage settings(ofToDataPath("settings.yml"), FileStorage::READ);
    if(settings.isOpened()) {
        int xCount = settings["xCount"], yCount = settings["yCount"];
        calibration.setPatternSize(xCount, yCount);
        float squareSize = settings["squareSize"];
        calibration.setSquareSize(squareSize);
        float markerSize = settings["markerSize"];
        calibration.setMarkerSize(markerSize);
        CalibrationPattern patternType;
        switch(settings["patternType"]) {
            case 0: patternType = CHESSBOARD; break;
            case 1: patternType = CIRCLES_GRID; break;
            case 2: patternType = ASYMMETRIC_CIRCLES_GRID; break;
            case 3: patternType = CHARUCO; break;
        }
        calibration.setPatternType(patternType);
        
        
    }
    calibration.setupBoards();
    
    lastTime = 0;
    
    
}

//--------------------------------------------------------------
void ofApp::update(){
    camMat = ipCam.get();
    
    //    switch(states){
    //        case CALIBRATE_CAMERA:
    //        {
    //            if(updateDiffMean()) {
    //                if(calibration.arucoMarkersDetectCollect(camMat, markersDetectedMat)){
    //                    calibration.calibrateCamera(camMat, cameraCalibratonMat);
    //                    lastTime = ofGetElapsedTimef();
    //                }
    //            }
    //        } break;
    //        case CALIBRATE_PROJECTOR:
    //        {
    //
    //        } break;
    //        DEFUALT:
    //            break;
    //    }
    
    if(updateDiffMean()) {

        if(calibration.add(camMat)) {
            cout << "re-calibrating" << endl;
            calibration.calibrate();
//            if(calibration.size() > startCleaning) {
//                calibration.clean(0.3f);
//            }
            if(calibration.ready) calibration.save("calibrationCam.yml");
            lastTime = ofGetElapsedTimef();
        }
    }
    
    
    if(calibration.ready){
        calibration.undistort(camMat, toCv(undistortedImg));
        //        undistort(camMat, toCv(undistortedImg), calibration.cameraMatrix, calibration.distCoeffs);
        Mat imageOut = toCv(undistortedImg);
        calibration.arucoBoardDetect(toCv(undistortedImg), imageOut);
        undistortedImg.update();
        
    }
    
}
//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(20);
    ofSetColor(255);
    toOf(camMat, camImg);
    camImg.update();
    camImg.draw(0, 720,512,288);
    undistortedImg.draw(512,720,512,288);
    drawMat(markersDetectedMat, 0,0);
    drawMat(cameraCalibratonMat, 1024,720,512,288);
    stringstream intrinsics;
    intrinsics << "fov: " << toOf(calibration.getDistortedIntrinsics().getFov()) << " distCoeffs: " << calibration.getDistCoeffs();
    string oneLine = intrinsics.str();
    ofStringReplace(oneLine, "\n", "");
    ofDrawBitmapStringHighlight(oneLine, 10, 20, yellowPrint, ofColor(0));
    ofDrawBitmapStringHighlight("movement: " + ofToString(diffMean), 10, 40, cyanPrint);
        ofDrawBitmapStringHighlight("reproj error: " + ofToString(calibration.getReprojectionError()) + " from " + ofToString(calibration.size()), 10, 60, magentaPrint);
    if(calibration.ready){
        for(int i = 0; i < calibration.size(); i++) {
            ofDrawBitmapStringHighlight(ofToString(i) + ": " + ofToString(calibration.getReprojectionError(i)), 10, 80 + 16 * i, magentaPrint);
        }
    }
    
    ofSetWindowTitle("FPS: "+ofToString(ofGetFrameRate()));
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key == 'x') {
        calibration.clean(0.5);
    }
    if(key == 'r') {
        calibration.reset();
    }
}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

//--------------------------------------------------------------
bool ofApp::updateDiffMean(){
    bool goodToGo = false;
    
    absdiff(previous, camMat, diff);
    camMat.copyTo(previous);
    
    diffMean = mean(Mat(mean(diff)))[0];
    
    if(ofGetElapsedTimef() - lastTime > timeThreshold && diffMean < diffThreshold) goodToGo = true;
    return goodToGo;
}

