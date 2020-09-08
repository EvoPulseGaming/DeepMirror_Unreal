// (c) 2019 Technical University of Munich
// Jakob Weiss <jakob.weiss@tum.de>

#pragma once
#include "CoreMinimal.h"
#include "OpenCVHelper.h"


OPENCV_INCLUDES_START
#undef check

// now you can include opencv
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#undef UpdateResource
OPENCV_INCLUDES_END


#include "DlibHelper.h"
DLIB_INCLUDES_START
#undef check
#undef verify
#undef TEXT
#undef template
// now you can include dlib
#include "Dlib/svm.h"
#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing.h" 
#include "dlib/opencv/cv_image.h"
#undef UpdateResource
DLIB_INCLUDES_END



