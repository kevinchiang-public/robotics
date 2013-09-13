FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hovercraft/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/hovercraft/msg/__init__.py"
  "../src/hovercraft/msg/_BatteryVoltage.py"
  "../src/hovercraft/msg/_Thruster.py"
  "../src/hovercraft/msg/_Current.py"
  "../src/hovercraft/msg/_Gyro.py"
  "../src/hovercraft/msg/_LED.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
