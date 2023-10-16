#!/usr/bin/env python3
# coding=utf-8
# Author: 
import sys
import rospy

from rosplan_cra_module import ROSPlanCRA

if __name__ == "__main__":
    rospy.init_node("rosplan_for_cra_node", sys.argv)
    rospy.loginfo(rospy.get_name() + ": ROSPlan + rosprolog node has been initialized.")
    
    rpcra = ROSPlanCRA()

    rpcra.rosplan_wrapper_.planning_pipeline() # generate and parse planning

    rpcra.rosplan_wrapper_.construct_plan_dict()
    rpcra.rosplan_wrapper_.plan_dict_['task_grounded_parameters_dict'].clear()

    plan_triples_list = rpcra.rosprolog_wrapper_for_rosplan_cra_.plan_dict_to_triples_list(rpcra.rosplan_wrapper_.plan_dict_)

    plan_assertion_query_text = rpcra.rosprolog_wrapper_for_rosplan_cra_.construct_query_text_for_multiple_triples_assertion(plan_triples_list, True)
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(plan_assertion_query_text)

    # run some logic-based rules to assert new comparative relations between any pair of plans that exist in the current ontological knowledge
    query_string_foo_ = "compare_all_existing_plans_in_pairs()."
    rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(query_string_foo_)
