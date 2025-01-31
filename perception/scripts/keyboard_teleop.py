#!/usr/bin/env python3

import robot_api
import rospy


import sys, select, termios, tty

msg = """
Control Your Fetch!
---------------------------
Moving around:
   q    w    e
   a    s    d

Space: force stop
q/e: move head
r/f: move tilt
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}
headBindings = {'q':(1,0),'e':(-1,0),'r':(0,-1),'f':(0,1),'c':(0,0)}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('fetch_teleop_key')
    base = robot_api.Base()
    head = robot_api.Head()
    
    x = 0
    th = 0
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    pan,tilt,ifmovehead = 0,0,False # for moving head
    try:
        print(msg)
        print(vels(speed, turn))
        while (1):
            key = getKey()
            key = key.lower()
            if key == 'z':
                break
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in headBindings.keys():
                ifmovehead = True
                if key != 'c':
                    pan += headBindings[key][0]*0.2 # pan speed
                    pan = min(max(pan,head.MIN_PAN),head.MAX_PAN)
                    tilt += headBindings[key][1]*0.2 # tilt speed
                    tilt = min(max(tilt,head.MIN_TILT),head.MAX_TILT)
                else:
                    pan = 0
                    tilt = 0
                count = 0

            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]
                count = 0

                print(vels(speed, turn))
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            
            elif key == ' ':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn

            base.move(control_speed, control_turn)
            if(ifmovehead):
                # move head
                head.pan_tilt(pan=pan,tilt=tilt)
                ifmovehead = False

    except Exception as e:
        rospy.logerr('{}'.format(e))
    finally:
        base.stop()
        head.pan_tilt(0,0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
