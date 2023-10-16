#!/usr/bin/env python3
# coding=utf-8

"""
What is this code?
  - acxon test for ontology-based narratives. 
  You can expect to find here an example of use of the algorithm for explanatory ontology-based narratives in the 
  collaborative robotics and adaptation domain. Make sure that you have properly selected the algorithm parameters
  (e.g., the temporal locality (t_locality), the specificity, etc.). You can modify them below.

"""

import re
import csv
import time
import rospy
import roslib
import rospkg
import textstat
from utils.test_module import *
from acxon.acxon_module import *
from prolog.prolog_module import *
from readability import Readability
from rosprolog_client import PrologException, Prolog
from rosplan_cra_module import ROSPlanCRA

if __name__ == '__main__': 
    rospy.init_node('acxon_test_plan_disambiguation_with_neem')
    roslib.load_manifest('rosprolog')

    # TEST variables  
    start = time.time()
    triples_count = 0
    words_count = 0
    test_object = TestClassForExplanatoryNarratives()

    if (rospy.has_param('~specificity_level')):
      specificity = rospy.get_param('~specificity_level')
    else:
      rospy.loginfo(rospy.get_name() + ": ROS parameter cannot be read.")
      specificity = 3
    
    if (rospy.has_param('~domain_name')):
      planning_domain = rospy.get_param('~domain_name')
    else:
      rospy.loginfo(rospy.get_name() + ": ROS parameter cannot be read.")
      planning_domain = "unknown"
    
    if (rospy.has_param('~problem_name')):
      planning_problem = rospy.get_param('~problem_name')
    else:
      rospy.loginfo(rospy.get_name() + ": ROS parameter cannot be read.")
      planning_problem = "unknown"
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    t_locality = [1.0, 5.0]

    # pairs of plans to c-narrate
    classes_to_compare = [["dul:'Plan'", "dul:'Plan'"]] 
    ## constrained_ontological_scope = ["dul:'Quality'", "dul:'Event'"] # classes to constrain the scope of the narrative
    constrained_ontological_scope = [] # no constrain at all, the narrative will use all the stored knowledge 

    tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, classes_to_compare, t_locality, constrained_ontological_scope, specificity)

    for pair_to_compare_id, tuples_of_the_pair in tuples_dict.items():
      pair_of_plans_to_c_narrate_name = list()
      
      pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]))
      pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]))

      # narrative construction
      introductory_text_plans = "\n\n·····There are two plans to disambiguate: " + \
        pair_of_plans_to_c_narrate_name[0] \
        + " and " + \
        pair_of_plans_to_c_narrate_name[1] + ". \n\n"
      introductory_text_plans = introductory_text_plans.replace(pair_of_plans_to_c_narrate_name[0], "Plan_A")
      introductory_text_plans = introductory_text_plans.replace(pair_of_plans_to_c_narrate_name[1], "Plan_B")

      plans_c_narrative = construct_narrative(client_rosprolog, pairs_id_to_pairs_to_compare_dict[pair_to_compare_id], \
                                      tuples_of_the_pair)
      

      # Modify the narrative to make it more appealing
      combined_c_narrative_mod = combined_c_narrative.replace(pair_of_plans_to_c_narrate_name[0], "Plan_A")
      combined_c_narrative_mod = combined_c_narrative_mod.replace(pair_of_plans_to_c_narrate_name[1], "Plan_B")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has worse quality value", "has a higher value")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has better quality value", "has a lower value")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has equivalent quality value than", "has the same value as")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has role", "is classified as")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("is role of", "classifies")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("defines task", "includes task")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("_", " ")

      # Evaluation metrics
      words_count = len(combined_c_narrative_mod.split())
      triples_count = triples_count + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]]) + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]])
      read_score = textstat.dale_chall_readability_score(combined_c_narrative_mod)

      # Add introductory text
      combined_c_narrative_mod = introductory_text_plans + combined_c_narrative_mod
      ## print("\nC-Narrative")
      ## print(combined_c_narrative)

    print("[acxon_test_plan_disambiguation_with_neem.py] Narratives have been properly generated, check the 'txt'.")
    
    end = time.time()
    
    # results dictionary
    results_dict = {"Domain": planning_domain, "Problem" : planning_problem, "Specificity" : specificity, \
                    "Time" : (end - start), "Memory" : test_object.get_memory_usage()['vmpeak'], "Triples" : triples_count, \
                      "Words": words_count, "Readability score": read_score}
    
    
    print("\n ·· TEST ·····\n Elapsed time: ", results_dict["Time"], " seconds")
    print(" Memory usage (peak): ", results_dict["Memory"], " MB") # 'vmpeak' 'vmsize' 'vmlck' 'vmpin' 'vmhwm' 'vmrss' 'vmdata' 'vmstk' 'vmexe' 'vmlib' 'vmpte' 'vmswap'
    print(" Number of triples: ", results_dict["Triples"]) 
    print(" Narrative number of words (aprox.): ", results_dict["Words"]) 
    print(" Narrative readability (score): ", results_dict["Readability score"]) 
    print("\n")