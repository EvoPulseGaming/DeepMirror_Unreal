// (c) 2019 Technical University of Munich
// Jakob Weiss <jakob.weiss@tum.de>

#pragma once
#include "CoreMinimal.h"
#include "OpenCVHelper.h"


OPENCV_INCLUDES_START
// disable shitty unreal macros
//#pragma push_macro("check")//already done in macro above
#undef check
//#pragma push_macro("CONSTEXPR")
//#undef CONSTEXPR
//#pragma push_macro("dynamic_cast")
//#undef dynamic_cast
//#pragma push_macro("PI")
//#undef PI
#pragma push_macro("TEXT")//dlib
#undef TEXT
#pragma push_macro("verify")//dlib
#undef verify
//#pragma push_macro("INT")//dlib
//#undef INT
//#pragma push_macro("FLOAT")//dlib
//#undef FLOAT

// now you can include opencv
//#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
//#include "opencv2/video.hpp"

#include "Dlib/svm.h"
#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing.h" 
#include "dlib/opencv/cv_image.h"

#undef UpdateResource
// end restore shitty unreal macros
//#pragma pop_macro("FLOAT")
//#pragma pop_macro("INT")
#pragma pop_macro("verify")
#pragma pop_macro("TEXT")
//#pragma pop_macro("PI")
//#pragma pop_macro("dynamic_cast")
//#pragma pop_macro("CONSTEXPR")

//#pragma pop_macro("check") //Already done in macro below
OPENCV_INCLUDES_END



