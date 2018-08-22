//
//  CalibrationCharuco.h
//  CharucoCameraLaserCalibration
//
//  Created by WandernderPunkt on 22.08.18.
//

#ifndef CalibrationCharuco_h
#define CalibrationCharuco_h

#include "ofxCv.h"
#include <opencv2/aruco/charuco.hpp>

class CalibrationCharuco : public Calibration {
    Mat markersDetectedMat;
    Mat cameraCalibratonMat;
    
    void setupBoards(){
        
    }
    
    
    
}

#endif /* CalibrationCharuco_h */
