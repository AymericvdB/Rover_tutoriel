# teleop_twist_keyboard_vf
Generic Keyboard Teleop for ROS2
#Launch
To run: `teleop_twist_keyboard_vf`

#Usage
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   ,    ;    :

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   ?    .    /

t : up (+z)
b : down (-z)

anything else : stop

a/q : increase/decrease max speeds by 10%
z/s : increase/decrease only linear speed by 10%
e/d : increase/decrease only angular speed by 10%

CTRL-C to quit
```

