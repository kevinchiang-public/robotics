FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hoverboard/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/hoverboard/ServoRaw.h"
  "../msg_gen/cpp/include/hoverboard/GPIORaw.h"
  "../msg_gen/cpp/include/hoverboard/ADCRaw.h"
  "../msg_gen/cpp/include/hoverboard/PWMRaw.h"
  "../msg_gen/cpp/include/hoverboard/GyroRaw.h"
  "../msg_gen/cpp/include/hoverboard/ThrusterRaw.h"
  "../msg_gen/cpp/include/hoverboard/LEDRaw.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
