FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hoverboard/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/hoverboard/msg/__init__.py"
  "../src/hoverboard/msg/_ServoRaw.py"
  "../src/hoverboard/msg/_GPIORaw.py"
  "../src/hoverboard/msg/_ADCRaw.py"
  "../src/hoverboard/msg/_PWMRaw.py"
  "../src/hoverboard/msg/_GyroRaw.py"
  "../src/hoverboard/msg/_ThrusterRaw.py"
  "../src/hoverboard/msg/_LEDRaw.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
