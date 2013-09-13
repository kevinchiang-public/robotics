FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hovercraft/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/BatteryVoltage.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_BatteryVoltage.lisp"
  "../msg_gen/lisp/Thruster.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Thruster.lisp"
  "../msg_gen/lisp/Current.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Current.lisp"
  "../msg_gen/lisp/Gyro.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Gyro.lisp"
  "../msg_gen/lisp/LED.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_LED.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
