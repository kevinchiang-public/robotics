FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hovercraft/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/hovercraft/BatteryVoltage.h"
  "../msg_gen/cpp/include/hovercraft/Thruster.h"
  "../msg_gen/cpp/include/hovercraft/Current.h"
  "../msg_gen/cpp/include/hovercraft/Gyro.h"
  "../msg_gen/cpp/include/hovercraft/LED.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
