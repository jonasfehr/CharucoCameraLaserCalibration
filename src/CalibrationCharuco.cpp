//
//  CalibrationCharuco.h
//  CharucoCameraLaserCalibration
//
//  Created by WandernderPunkt on 22.08.18.
//
#include "CalibrationCharuco.h"

using namespace ofxCv;
using namespace cv;

bool CalibrationCharuco::add(cv::Mat img){
    addedImageSize = img.size();
    
    bool found = arucoMarkersDetectCollect(img, markersDetectedMat);
    
//    if (!found) ofLog(OF_LOG_ERROR, "Calibration::add() failed, maybe your patternSize is wrong or the image has poor lighting?");
    
    return found;
}

bool CalibrationCharuco::calibrate(){
    if(size() < 1) {
        ofLog(OF_LOG_ERROR, "Calibration::calibrate() doesn't have any image data to calibrate from.");
        if(ready) {
            ofLog(OF_LOG_ERROR, "Calibration::calibrate() doesn't need to be called after Calibration::load().");
        }
        return ready;
    }
    
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

//    vector<Mat> rvecs;
//    vector<Mat> tvecs;
    
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
    
    updateObjectPoints();

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
        return false;
    }
    
    Mat stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrorsMat;
    
    // calibrate camera using charuco
    float rms = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, addedImageSize, cameraMatrix, distCoeffs, boardRotations, boardTranslations, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrorsMat, calibrationFlags);
    ofLog(OF_LOG_VERBOSE, "calibrateCamera() reports RMS error of " + ofToString(rms));
    
    ready = checkRange(cameraMatrix) && checkRange(distCoeffs);
    
    if(!ready) {
        ofLog(OF_LOG_ERROR, "Calibration::calibrate() failed to calibrate the camera");
    }
    
    distortedIntrinsics.setup(cameraMatrix, addedImageSize);
    updateReprojectionError(perViewErrorsMat);
    updateUndistortion();
    
    return ready;
}

void CalibrationCharuco::updateReprojectionError(Mat perViewErrorsMat) {

    int totalPoints = 0;
    double totalErr = 0;
    
    perViewErrors.clear();
    for(int i = 0; i<perViewErrorsMat.rows; i++){
        perViewErrors.push_back((float)perViewErrorsMat.at<double>(i,0));
    }
//    perViewErrors.assign((float*)perViewErrorsMat.datastart, (float*)perViewErrorsMat.dataend);

    for(auto & e : perViewErrors) totalErr += e;
    reprojectionError = totalErr/perViewErrors.size();
    
//    for(std::size_t i = 0; i < objectPoints.size(); i++) {
//        projectPoints(cv::Mat(objectPoints[i]), boardRotations[i], boardTranslations[i], distortedIntrinsics.getCameraMatrix(), distCoeffs, imagePoints2);
//        double err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
//        int n = objectPoints[i].size();
//        perViewErrors[i] = sqrt(err * err / n);
//        totalErr += err * err;
//        totalPoints += n;
//        ofLog(OF_LOG_VERBOSE, "view " + ofToString(i) + " has error of " + ofToString(perViewErrors[i]));
//    }
//    
//    reprojectionError = sqrt(totalErr / totalPoints);
    
    ofLog(OF_LOG_VERBOSE, "all views have error of " + ofToString(reprojectionError));
}


//void Calibration::updateObjectPoints() {
//    std::vector<cv::Point3f> points = createObjectPoints(patternSize, squareSize, patternType);
//    objectPoints.resize(imagePoints.size(), points);
//}

std::vector<cv::Point3f> CalibrationCharuco::createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType) {
    std::vector<cv::Point3f> corners;
    for(int i = 0; i < patternSize.width-1; i++)
        for(int j = 0; j < patternSize.height-1; j++)
            corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
    return corners;
}

//int CalibrationCharuco::size(){
//    Calibration::size();
//}

bool CalibrationCharuco::clean(float minReprojectionError){
    int removed = 0;
    for(int i = size() - 1; i >= 0; i--) {
        if(getReprojectionError(i) > minReprojectionError) {
            objectPoints.erase(objectPoints.begin() + i);
            imagePoints.erase(imagePoints.begin() + i);
            allCorners.erase(allCorners.begin() + i);
            allIds.erase(allIds.begin() + i);
            allImgs.erase(allImgs.begin() + i);
            removed++;
        }
    }
    if(size() > 0) {
        if(removed > 0) {
            return calibrate();
        } else {
            return true;
        }
    } else {
        ofLog(OF_LOG_ERROR, "Calibration::clean() removed the last object/image point pair");
        return false;
    }
}


void CalibrationCharuco::setupBoards(){
    patternType = CalibrationPattern::CHARUCO;
    dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    charucoboard = cv::aruco::CharucoBoard::create(getPatternSize().width, getPatternSize().height, squareSize, markerSize, dictionary);
//    charucoboard = cv::aruco::CharucoBoard::create(getPatternSize().width, getPatternSize().height, getSquareSize(), getMarkerSize(), dictionary);
    board = charucoboard.staticCast<aruco::Board>();
    
    // for board detection
    axisLength = 0.5f * ((float)min(getPatternSize().height, getPatternSize().width) * (squareSize));
    totalTime = 0;
    totalIterations = 0;
}

//bool CalibrationCharuco::findBoard(cv::Mat img, std::vector<cv::Point2f>& pointBuf, bool refine) {
//    vector< int > ids;
//    vector< vector< Point2f > > corners, rejected;
//    // detect markers
//    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
//
//    aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
//
//    // refind strategy to detect more markers
//    if(true) aruco::refineDetectedMarkers(img, board, corners, ids, rejected);
//
//    // interpolate charuco corners
//    Mat currentCharucoCorners, currentCharucoIds;
//    if(ids.size() > 0)
//        aruco::interpolateCornersCharuco(corners, ids, img, charucoboard, currentCharucoCorners, currentCharucoIds);
//
//    // draw results
//    //            camMat.copyTo(markersDetectedMat);
//    //            if(ids.size() > 0) aruco::drawDetectedMarkers(markersDetectedMat, corners);
//    //            if(currentCharucoCorners.total() > 0) aruco::drawDetectedCornersCharuco(markersDetectedMat, currentCharucoCorners, currentCharucoIds);
//
//    if(currentCharucoCorners.rows==((patternSize.height-1)*(patternSize.width-1)) && ids.size() > 16) {
//        //                cout << "Frame captured" << endl;
//        //                allCorners.push_back(corners);
//        //                allIds.push_back(ids);
//        //                allImgs.push_back(img);
//        //                imgSize = img.size();
//
//        vector< cv::Point2f > Vf2;
//        //copy mat to vector
//        Vf2.assign((cv::Point2f*)currentCharucoCorners.datastart, (cv::Point2f*)currentCharucoCorners.dataend);
//
//        for(int i = 0; i<(patternSize.height-1); i++){
//            for(int j = 0; j<(patternSize.width-1); j++){
//                int index = i+j*(patternSize.height-1);
//                pointBuf.push_back(Vf2.at(index));
//            }
//        }
//
//        return true;
//    }else return false;
//}

bool CalibrationCharuco::arucoMarkersDetectCollect(Mat img, Mat & imageOut){
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
    if(cameraMatrix.total() != 0){
        Vec3d rvec, tvec;
        if(aruco::estimatePoseCharucoBoard(currentCharucoCorners, currentCharucoIds, charucoboard, cameraMatrix, distCoeffs, rvec, tvec)){
            aruco::drawAxis(imageOut, cameraMatrix, distCoeffs, rvec, tvec, axisLength);
        }
    }
    
    // draw results
    image.copyTo(imageOut);
    if(ids.size() > 0) aruco::drawDetectedMarkers(imageOut, corners);
    
    //    if(rejected.size() > 0) aruco::drawDetectedMarkers(imageOut, rejected, noArray(), Scalar(100, 0, 255));
    
    if(currentCharucoCorners.total() > 0)
        aruco::drawDetectedCornersCharuco(imageOut, currentCharucoCorners, currentCharucoIds);
    
    if(ids.size() > 0 && corners.size()>4 && currentCharucoCorners.rows==(patternSize.width-1)*(patternSize.height-1)) {
        cout << "Frame captured" << endl;
        allCorners.push_back(corners);
        allIds.push_back(ids);
        allImgs.push_back(image);
        
        //copy mat to vector
        vector< cv::Point2f > Vf2;
        Vf2.assign((cv::Point2f*)currentCharucoCorners.datastart, (cv::Point2f*)currentCharucoCorners.dataend);
        imagePoints.push_back(Vf2);
        
        addedImageSize = image.size();
        return true;
    } else return false;
}

bool CalibrationCharuco::arucoBoardDetect(Mat image, Mat & imageOut){
    
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
                                     cameraMatrix, distCoeffs);
    
    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(ids.size() > 0)
        interpolatedCorners =
        aruco::interpolateCornersCharuco(markerCorners, ids, image, charucoboard,
                                         charucoCorners, charucoIds, cameraMatrix, distCoeffs);
    
    // estimate charuco board pose
    bool validPose = false;
    if(cameraMatrix.total() != 0)
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                    cameraMatrix, distCoeffs, rvec, tvec);
    
    
    
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
    
    if(validPose) aruco::drawAxis(imageOut, cameraMatrix, distCoeffs, rvec, tvec, axisLength);
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

bool CalibrationCharuco::calibrateCamera(cv::Mat img, Mat & imageOut) {
    //        cameraMatrix = Mat::eye(3, 3, CV_64F);
    //    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    //    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    
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
    
    //    // prepare data for calibration
    //    vector< vector< Point2f > > allCornersConcatenated;
    //    vector< int > allIdsConcatenated;
    //    vector< int > markerCounterPerFrame;
    //    markerCounterPerFrame.reserve(allCorners.size());
    //    for(unsigned int i = 0; i < allCorners.size(); i++) {
    //        markerCounterPerFrame.push_back((int)allCorners[i].size());
    //        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
    //            allCornersConcatenated.push_back(allCorners[i][j]);
    //            allIdsConcatenated.push_back(allIds[i][j]);
    //        }
    //    }
    //
    //    // calibrate camera using aruco markers
    //    double arucoRepErr;
    //    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
    //                                              markerCounterPerFrame, board, addedImageSize, cameraMatrix,
    //                                              distCoeffs, noArray(), noArray(), calibrationFlags);
    
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
    reprojectionError =
    aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, addedImageSize,
                                  cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);
    
    save("calibrationCamera.yml");
    
    cout << "Rep Error: " << reprojectionError << endl;
    //    cout << "Rep Error Aruco: " << arucoRepErr << endl;
    
    
    // show interpolated charuco corners for debugging
    if(true) {
        imageOut = img;
        
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
    //    cameraMatrix.copyTo(cameraMatrix);
    //    distCoeffs.copyTo(distCoeffs);
    
    distortedIntrinsics.setup(cameraMatrix, addedImageSize);
    updateUndistortion();
    
}


void CalibrationCharuco::setMarkerSize(float markerSize) {
    this->markerSize = markerSize;
}
float CalibrationCharuco::getMarkerSize() const {
    return markerSize;
}

void CalibrationCharuco::reset(){
    Calibration::reset();
    allCorners.clear();
    allIds.clear();
    allImgs.clear();
    boardRotations.clear();
    boardTranslations.clear();
}




