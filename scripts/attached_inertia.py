#!/usr/bin/env python

import sys
import argparse

import rospy
from geometry_msgs.msg import Vector3
from telemanip_msgs.msg import AttachedInertia, Inertia

def main():

    rospy.init_node('attached_inertia')
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('ns',type=str,help='Namespace of object')
    parser.add_argument('id',type=int,help='ID of object')
    parser.add_argument('-m','--mass',type=float,default=0,help='Mass in kilograms')
    parser.add_argument('-c','--com',metavar=('CX','CY','CZ'),type=float,nargs=3,default=[0,0,0],help='Center of mass (x,y,z)  position in meters')
    parser.add_argument('-i','--inertia',metavar=('IXX','IXY','IXZ','IYY','IYZ','IZZ'),type=float,nargs=6,default=[0,0,0,0,0,0],help='Inertia tensor in kg-m^2')
    parser.add_argument('-d','--delete',action='store_true',default=False,help='Delete an attached inertia.')
    parser.add_argument('-f','--frame-id',type=str,default='',help='The TF frame in which the inertia is defined.')

    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    msg = AttachedInertia()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = args.frame_id
    msg.ns = args.ns
    msg.id = args.id
    msg.action = AttachedInertia.DELETE if args.delete else AttachedInertia.UPDATE
    msg.inertia = Inertia(*([args.mass]+[Vector3(*args.com)]+args.inertia))

    ee_pub = rospy.Publisher('end_effector_inertias', AttachedInertia)
    rospy.sleep(3)
    ee_pub.publish(msg)


if __name__ == '__main__':
    main()
