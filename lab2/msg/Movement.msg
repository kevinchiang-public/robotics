Header header
float32 theta   #Angular Position or Thrust
float32 x	#Translational x vector component
float32 y	#Translational y vector component
float32 mag     #The magnitude of the left joystick w.r.t. x and y component
bool    lift	#Whether the lift is on or off
string  modType	#One of 'Add', 'Set', or 'Bound'.
                     #'Add': Adds theta to the current angle to set the target angle
                     #'Set': Sets theta as the target angle
                     #'Bound' : Expects theta to be in range [0,360).  Sets target
                     #orients the hovercraft with respect to its initial position
                     #(for use with the joystick angle position)
