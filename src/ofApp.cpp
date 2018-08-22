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
        xCount = settings["xCount"], yCount = settings["yCount"];
//        calibration.setPatternSize(xCount, yCount);
        squareSize = settings["squareSize"];
//        calibration.setSquareSize(squareSize);
        markerSize = settings["markerSize"];
//        calibration.setMarkerSize(markerSize);
        CalibrationPattern patternType;
        switch(settings["patternType"]) {
            case 0: patternType = CHESSBOARD; break;
            case 1: patternType = CIRCLES_GRID; break;
            case 2: patternType = ASYMMETRIC_CIRCLES_GRID; break;
            case 3: patternType = CHARUCO; break;
        }
//        calibration.setPatternType(patternType);

        
    }
    
    lastTime = 0;
    ready = false;
    
    dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    charucoboard = cv::aruco::CharucoBoard::create(xCount, yCount, squareSize, markerSize, dictionary);
    //    charucoboard = cv::aruco::CharucoBoard::create(calibration.getPatternSize().height, calibration.getPatternSize().width, calibration.getSquareSize(), calibration.getMarkerSize(), dictionary);
    board = charucoboard.staticCast<aruco::Board>();
    

    // for board detection
    axisLength = 0.5f * ((float)min(xCount, yCount) * (squareSize));
    
    totalTime = 0;
    totalIterations = 0;

}

//--------------------------------------------------------------
void ofApp::update(){
    camMat = ipCam.get();
    
    switch(states){
        case CALIBRATE_CAMERA:
        {
            if(updateDiffMean()) {
                if(arucoMarkersDetectCollect(camMat, markersDetectedMat)){
                    calibrateCamera(cameraCalibratonMat);
                    lastTime = ofGetElapsedTimef();
                }
            }
        } break;
        case CALIBRATE_PROJECTOR:
        {
            
        } break;
        DEFUALT:
            break;
    }
    
    
    if(ready){
        undistort(camMat, toCv(undistortedImg), cameraMatrixCamera, distCoeffsCamera);
        Mat imageOut = toCv(undistortedImg);
        arucoBoardDetect(toCv(undistortedImg), imageOut);
        undistortedImg.update();

    }
    
}

bool ofApp::calibrateOldSchool(){
//    if(calibration.add(camMat)) {
//        cout << "re-calibrating" << endl;
//        calibration.calibrate();
//        if(calibration.size() > startCleaning) {
//            calibration.clean();
//        }
//        calibration.save("calibrationCam.yml");
//        lastTime = ofGetElapsedTimef();
//    }
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
    intrinsics << "fov: " << toOf(distortedIntrinsics.getFov()) << " distCoeffs: " << distCoeffs;
    string oneLine = intrinsics.str();
    ofStringReplace(oneLine, "\n", "");
    ofDrawBitmapStringHighlight(oneLine, 10, 20, yellowPrint, ofColor(0));
    ofDrawBitmapStringHighlight("movement: " + ofToString(diffMean), 10, 40, cyanPrint);
//    ofDrawBitmapStringHighlight("reproj error: " + ofToString(calibration.getReprojectionError()) + " from " + ofToString(calibration.size()), 10, 60, magentaPrint);
//    for(int i = 0; i < calibration.size(); i++) {
//        ofDrawBitmapStringHighlight(ofToString(i) + ": " + ofToString(calibration.getReprojectionError(i)), 10, 80 + 16 * i, magentaPrint);
//    }
    ofSetWindowTitle("FPS: "+ofToString(ofGetFrameRate()));

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    if(key == 'x') {
        calibrateCamera(cameraCalibratonMat);
    }
    if(key == 'r') {
    allCorners.clear();
    allIds.clear();
    allImgs.clear();
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
bool ofApp::arucoMarkersDetectCollect(Mat img, Mat & imageOut){
    Mat image = img;
    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    // detect markers
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    if(corners.size()==0) return false;
    
    // refind strategy to detect more markers
    if(true) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);
    
    // interpolate charuco corners
    Mat currentCharucoCorners, currentCharucoIds;
    if(ids.size() > 0)
        aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
                                         currentCharucoIds);
    
    // estimate charuco board pose
    if(cameraMatrixCamera.total() != 0){
        Vec3d rvec, tvec;
        if(aruco::estimatePoseCharucoBoard(currentCharucoCorners, currentCharucoIds, charucoboard, cameraMatrixCamera, distCoeffsCamera, rvec, tvec)){
            aruco::drawAxis(imageOut, cameraMatrixCamera, distCoeffsCamera, rvec, tvec, axisLength);
        }
    }
    
    // draw results
    image.copyTo(imageOut);
    if(ids.size() > 0) aruco::drawDetectedMarkers(imageOut, corners);
    
//    if(rejected.size() > 0) aruco::drawDetectedMarkers(imageOut, rejected, noArray(), Scalar(100, 0, 255));
    
    if(currentCharucoCorners.total() > 0)
        aruco::drawDetectedCornersCharuco(imageOut, currentCharucoCorners, currentCharucoIds);
    
    if(ids.size() > 0 && corners.size()>4 && currentCharucoCorners.rows==(xCount-1)*(yCount-1)) {
        cout << "Frame captured" << endl;
        allCorners.push_back(corners);
        allIds.push_back(ids);
        allImgs.push_back(image);
        imgSize = image.size();
        return true;
    } else return false;
}

bool ofApp::arucoBoardDetect(Mat image, Mat & imageOut){
    
        Mat cameraMatrixGlobal = Mat::eye(3, 3, CV_64F);
        Mat distCoeffsGlobal;
    
    double tick = (double)getTickCount();
    
    vector< int > ids, charucoIds;
    vector< vector< Point2f > > markerCorners, rejectedMarkers;
    vector< Point2f > charucoCorners;
    Vec3d rvec, tvec;
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    // detect markers
    aruco::detectMarkers(image, dictionary, markerCorners, ids, detectorParams,
                         rejectedMarkers);
    
    // refind strategy to detect more markers
    if(true)
        aruco::refineDetectedMarkers(image, board, markerCorners, ids, rejectedMarkers,
                                     cameraMatrixCamera, distCoeffsCamera);
    
    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(ids.size() > 0)
        interpolatedCorners =
        aruco::interpolateCornersCharuco(markerCorners, ids, image, charucoboard,
                                         charucoCorners, charucoIds, cameraMatrixCamera, distCoeffsCamera);
    
    // estimate charuco board pose
    bool validPose = false;
    if(cameraMatrixCamera.total() != 0)
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                    cameraMatrixCamera, distCoeffsCamera, rvec, tvec);
    
    
    
    double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if(totalIterations % 30 == 0) {
        cout << "Detection Time = " << currentTime * 1000 << " ms "
        << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
    }
    
    
    
    // draw results
    if(ids.size() > 0) aruco::drawDetectedMarkers(imageOut, markerCorners);

//    if(showRejected && rejectedMarkers.size() > 0)
//        aruco::drawDetectedMarkers(imageOut, rejectedMarkers, noArray(), Scalar(100, 0, 255));

    if(interpolatedCorners > 0) aruco::drawDetectedCornersCharuco(imageOut, charucoCorners, charucoIds, Scalar(255, 0, 0));
    
    if(validPose) aruco::drawAxis(imageOut, cameraMatrixCamera, distCoeffsCamera, rvec, tvec, axisLength);
}

//bool ofApp::BoardAndProjectionDetect(cv::Mat img, cv::Mat processedImg){
//    
//    vector<cv::Point2f> chessImgPts;
//    bool bPrintedPatternFound = arucoBoardDetect(Mat image, Mat & imageOut)
////    bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);
//    cout << "printedPatternFound " << bPrintedPatternFound << endl;
//    if(bPrintedPatternFound) {
//        
//        vector<cv::Point2f> circlesImgPts;
//        bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, calibrationProjector.getPatternSize(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);
//        
//        if(bProjectedPatternFound){
//            
//            vector<cv::Point3f> circlesObjectPts;
//            cv::Mat boardRot;
//            cv::Mat boardTrans;
//            calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
//            calibrationCamera.backProject(boardRot, boardTrans, circlesImgPts, circlesObjectPts);
//            
//            calibrationCamera.imagePoints.push_back(chessImgPts);
//            calibrationCamera.getObjectPoints().push_back(calibrationCamera.getCandidateObjectPoints());
//            calibrationCamera.getBoardRotations().push_back(boardRot);
//            calibrationCamera.getBoardTranslations().push_back(boardTrans);
//            
//            calibrationProjector.imagePoints.push_back(calibrationProjector.getCandidateImagePoints());
//            calibrationProjector.getObjectPoints().push_back(circlesObjectPts);
//            
//            return true;
//        }
//    }
//    return false;
//}

bool ofApp::calibrateCamera(Mat & imageOut) {
//        cameraMatrix = Mat::eye(3, 3, CV_64F);
    //    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
//    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    
    
    string outputFile = "test.xml";
    
    int calibrationFlags = 0;
    float aspectRatio = 1;
    if(false) {
        calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = 16/9;
    }
    if(false) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    if(false) calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;
    
    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }
    
    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }
    
    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
                                              distCoeffs, noArray(), noArray(), calibrationFlags);
    
    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    vector< Mat > allCharucoCorners;
    vector< Mat > allCharucoIds;
    vector< Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);
    
    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        Mat currentCharucoCorners, currentCharucoIds;
        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);
        if(currentCharucoCorners.total()<4) continue;
        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }
    
    if(allCharucoCorners.size() < 4) {
        cerr << "Not enough corners for calibration" << endl;
        return 0;
    }
    
    // calibrate camera using charuco
    repError =
    aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                  cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);
    
    saveCameraCalibration("calibrationCamera.yml");
    
    cout << "Rep Error: " << repError << endl;
    cout << "Rep Error Aruco: " << arucoRepErr << endl;
    cout << "Calibration saved to " << outputFile << endl;
    
    
    // show interpolated charuco corners for debugging
    if(true) {
        imageOut = camMat;

        for(unsigned int frame = 0; frame < filteredImages.size(); frame++) {
//            imageOut = filteredImages[frame].clone();
            if(allIds[frame].size() > 0) {
                
                if(allCharucoCorners[frame].total() > 0) {
                    aruco::drawDetectedCornersCharuco( imageOut, allCharucoCorners[frame],
                                                      allCharucoIds[frame]);
                }
            }
        }
    }
    ready = checkRange(cameraMatrix) && checkRange(distCoeffs);
    cameraMatrix.copyTo(cameraMatrixCamera);
    distCoeffs.copyTo(distCoeffsCamera);
    
    distortedIntrinsics.setup(cameraMatrix, imgSize);
    updateUndistortion();

}



bool ofApp::updateDiffMean(){
    bool goodToGo = false;
    
    absdiff(previous, camMat, diff);
    camMat.copyTo(previous);
    
    diffMean = mean(Mat(mean(diff)))[0];
    
    if(ofGetElapsedTimef() - lastTime > timeThreshold && diffMean < diffThreshold) goodToGo = true;
    return goodToGo;
}

void ofApp::saveCameraCalibration(const std::string& filename, bool absolute) const {
    if(!ready){
        ofLog(OF_LOG_ERROR, "Calibration::save() failed, because your calibration isn't ready yet!");
    }
    cv::FileStorage fs(ofToDataPath(filename, absolute), cv::FileStorage::WRITE);
    cv::Size imageSize = distortedIntrinsics.getImageSize();
    cv::Size sensorSize = distortedIntrinsics.getSensorSize();
    cv::Mat cameraMatrix = distortedIntrinsics.getCameraMatrix();
    fs << "cameraMatrix" << cameraMatrix;
    fs << "imageSize_width" << imageSize.width;
    fs << "imageSize_height" << imageSize.height;
    fs << "sensorSize_width" << sensorSize.width;
    fs << "sensorSize_height" << sensorSize.height;
    fs << "distCoeffs" << distCoeffs;
    fs << "reprojectionError" << repError;
    fs << "features" << "[";
    
    
//    for(int i = 0; i < (int)imagePoints.size(); i++) {
//        fs << imagePoints[i];
//    }
    fs << "]";
}

void ofApp::loadCameraCalibration(const std::string& filename, bool absolute) {
    imagePoints.clear();
    cv::FileStorage fs(ofToDataPath(filename, absolute), cv::FileStorage::READ);
    cv::Size imageSize;
    cv::Size2f sensorSize;
    cv::Mat cameraMatrix;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["imageSize_width"] >> imageSize.width;
    fs["imageSize_height"] >> imageSize.height;
    fs["sensorSize_width"] >> sensorSize.width;
    fs["sensorSize_height"] >> sensorSize.height;
    fs["distCoeffs"] >> distCoeffs;
    fs["reprojectionError"] >> repError;
    cv::FileNode features = fs["features"];
    for(cv::FileNodeIterator it = features.begin(); it != features.end(); it++) {
        std::vector<cv::Point2f> cur;
        (*it) >> cur;
        imagePoints.push_back(cur);
    }
//    addedImageSize = imageSize;
    distortedIntrinsics.setup(cameraMatrix, imageSize, sensorSize);
    updateUndistortion();
    ready = true;
    }

void ofApp::updateUndistortion() {
    bool fillFrame = true;
    cv::Mat undistortedCameraMatrix = getOptimalNewCameraMatrix(distortedIntrinsics.getCameraMatrix(), distCoeffs, distortedIntrinsics.getImageSize(), fillFrame ? 0 : 1);
    initUndistortRectifyMap(distortedIntrinsics.getCameraMatrix(), distCoeffs, cv::Mat(), undistortedCameraMatrix, distortedIntrinsics.getImageSize(), CV_16SC2, undistortMapX, undistortMapY);
    undistortedIntrinsics.setup(undistortedCameraMatrix, distortedIntrinsics.getImageSize());
}
