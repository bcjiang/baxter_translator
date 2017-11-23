#!/usr/bin/env python

import rospy
import baxter_interface

def main():

    print "Translator started."
    rospy.init_node("start_translator")

    limb = baxter_interface.Limb('right')

    angles = limb.joint_angles()

    wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, \
              'right_e0': 1.807, 'right_e1': 1.714, \
              'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}

    wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, \
              'right_e0': 1.831, 'right_e1': 1.981, \
              'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}

    for _move in range(6):
        limb.move_to_joint_positions(wave_1)
        limb.move_to_joint_positions(wave_2)

    print "Translator stopped."
    quit()

if __name__ == '__main__':
    main()
