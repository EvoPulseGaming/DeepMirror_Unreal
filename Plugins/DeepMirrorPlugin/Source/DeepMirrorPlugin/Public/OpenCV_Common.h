// (c) 2019 Technical University of Munich
// Jakob Weiss <jakob.weiss@tum.de>

#pragma once

#include "CoreMinimal.h"

//// disable shitty unreal macros
//#pragma push_macro("CONSTEXPR")
//#undef CONSTEXPR
//#pragma push_macro("dynamic_cast")
//#undef dynamic_cast
//#pragma push_macro("check")
//#undef check
//#pragma push_macro("PI")
//#undef PI
//// now you can include opencv
//#include <opencv2/core.hpp>
//// end restore shitty unreal macros
//#pragma pop_macro("PI")
//#pragma pop_macro("check")
//#pragma pop_macro("dynamic_cast")
//#pragma pop_macro("CONSTEXPR")
#include "OpenCVHelper.h"


OPENCV_INCLUDES_START
#undef check
// your opencv include directives go here...
#include "opencv2/opencv.hpp"
OPENCV_INCLUDES_END

