#!/usr/bin/env python3
# coding=utf-8

import sys
import rospy

from rosplan_wrapper import ROSPlanWrapper
from rosprolog_wrapper_for_rosplan import ROSPrologWrapperForROSPlanCRA

class ROSPlanCRA:
    def __init__(self):
        # services
        

        # variables
        self.plan_is_generated_ = False
        self.rosplan_wrapper_ = ROSPlanWrapper()
        self.rosprolog_wrapper_for_rosplan_cra_ = ROSPrologWrapperForROSPlanCRA()

        if (rospy.has_param('~plan_adaptation_case')):
            self.plan_adaptation_case_ = rospy.get_param('~plan_adaptation_case')
        else:
            pass