#!/usr/bin/env python3
# coding=utf-8
import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import CompletePlan
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService, GetDomainOperatorDetailsService, \
    GetDomainTypeService, GetInstanceService, KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
import roslib
import time
import re

class ROSPlanWrapper:
    def __init__(self):
        # Define service clients
        self._get_goals = rospy.ServiceProxy("/rosplan_knowledge_base/state/goals", GetAttributeService)
        self._get_operator_details = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operator_details", GetDomainOperatorDetailsService)
        self._get_operators = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operators", GetDomainOperatorService)
        self._get_types = rospy.ServiceProxy("/rosplan_knowledge_base/domain/types", GetDomainTypeService)
        self._get_instances = rospy.ServiceProxy("/rosplan_knowledge_base/state/instances", GetInstanceService)
        self._get_propositions = rospy.ServiceProxy("/rosplan_knowledge_base/state/propositions", GetAttributeService)
        self._update_kb_srv = rospy.ServiceProxy("/rosplan_knowledge_base/update", KnowledgeUpdateService)
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        
        # Define topic subcriptions
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._parsed_plan_subs = rospy.Subscriber("/rosplan_parsing_interface/complete_plan", CompletePlan, self.parsed_plan_cb)

        # Define varibles
        self.plan_is_received_ = False
        self.plan_is_parsed_ = False
        self.domain_types_with_instances_dict_ = dict()
        self.problem_subgoals_dict_ = dict()
        self.domain_operators_details_dict_ = dict()
        self.domain_operators_list_ = list()
        self.plan_dict_ = dict()

        # Getting domain operators
        self._get_operators.wait_for_service()
        self.operators_ = [x.name for x in self._get_operators().operators]

        # Getting instances of garment type
        self._get_instances.wait_for_service()
        self.garments_ = self._get_instances('garment', False, False).instances

        # Getting propositions of current state
        self._get_propositions.wait_for_service()
        self.propositions_attributes_ = self._get_propositions().attributes
        ## print(self.propositions_attributes_)

        """ TO PLAY WITH IT WITHOUT CALLING THE METHODS FROM OUTSIDE
        # Generating problem and planning
        self.planning_pipeline()

        #print(self.generated_plan_string_)
        """


    def raw_plan_cb(self, msg): # it is called when a new plan is published
        rospy.loginfo(rospy.get_name() + ": Received raw plan")
        self.plan_is_received_ = True
        ## print(msg.data)
        self.generated_plan_string_ = msg.data
    
    def parsed_plan_cb(self, msg): # it is called when a new parsed plan is published
        rospy.loginfo(rospy.get_name() + ": Received parsed plan")
        self.plan_is_parsed_ = True
        ## print(msg.plan)
        self.generated_plan_parsed_ = msg.plan

    def planning_pipeline(self): 
        # TODO UPDATE PROBLEM
        rospy.loginfo(rospy.get_name() + ": Generating problem and planning")
        self._problem_gen.wait_for_service()
        self._problem_gen()
        self._planner.wait_for_service()
        self._planner()
        self._parse_plan.wait_for_service()
        self._parse_plan()

        time.sleep(2) # in seconds (to ensure that the callbacks are called before moving on)

    def update_planning_kb(self, update_type, knowledge_to_update):
        self._update_kb_srv.wait_for_service()
        self._update_kb_srv(update_type, knowledge_to_update)

    def add_or_remove_single_fact_planning_kb(self, update_type, fact_operator, fact_parameters_dict):
        # update_type = 0 for adding knowledge , 2 for removing knowledge (from KnowledgeUpdateService.srv)
        # fact_operator is a string containing the name of the domain operator to use in the fact (e.g. folded)
        # fact_parameters_dict contains the grounded parameters of the target operator, the key is the 
        #                      parameter's name (e.g. in 'folded ?g', the name is 'g') and the value is the instance (e.g. 'towel-01')
        
        knowledge_type = 1 # FACT (from KnowledgeItem.msg)
        
        msg = KnowledgeItem()
        msg.knowledge_type = knowledge_type
        msg.attribute_name = fact_operator

        parameters_list = list()
        for k, v in fact_parameters_dict.items():
            msg_values = KeyValue()
            msg_values.key = k
            msg_values.value = v
            parameters_list.append(msg_values)
        
        msg.values = parameters_list

        self.update_planning_kb(update_type, msg)

    def get_plan_cost(self):
        plan_path = rospy.get_param("rosplan_planner_interface/data_path")

        with open('/tmp/plan.pddl', 'r') as f:
            p = f.read()
            cost = re.findall(r'; Plan found with metric (\d+(?:.\d*)?)', p)
            cost = float(cost[-1])

            #print("\n\n", cost)
            #time.sleep(2) # in seconds (to ensure that the callbacks are called before moving on)

        return cost
    
    def construct_types_and_instances_dict(self): 
        rospy.loginfo(rospy.get_name() + ": Getting the domain types and their instances to assert them to the ontology KB")
        self._get_types.wait_for_service()
        domain_types_ans = self._get_types()
        for t in domain_types_ans.types:
            self.domain_types_with_instances_dict_[t] = self._get_instances(t, False, False).instances # (booleans) include_constants: include_subtypes:

    def construct_subgoals_dict(self):
        rospy.loginfo(rospy.get_name() + ": Getting the problem goal to assert it to the ontology KB")
        self._get_goals.wait_for_service()
        problem_goal_ans = self._get_goals()
        cont = 0
        for a in problem_goal_ans.attributes:
            if len(a.values) == 1: # 'object quality' goal component
                goal_component_tuple = [a.attribute_name, a.values[0].value]
            elif len(a.values) == 2: # 'object relationship' goal component
                goal_component_tuple = [a.attribute_name, a.values[0].value, a.values[1].value]
            else:
                rospy.logerr(rospy.get_name() + ": Part of the goal has an unexpected format")
                goal_component_tuple = []
            
            self.problem_subgoals_dict_['goal_component_'+str(cont)] = goal_component_tuple
            cont += 1

    def construct_plan_dict(self):
        rospy.loginfo(rospy.get_name() + ": Getting the plan to assert it to the ontology KB")
        if (self.plan_is_received_ and self.plan_is_parsed_):
            if (self.generated_plan_parsed_):
                rospy.loginfo(rospy.get_name() + ": There is at least a possible plan")

                aux_dict = dict()
                aux_dict["plan_id"] = "plan_" + str(self.generated_plan_parsed_[0].plan_id)
                aux_dict["task_id"] = list()
                aux_dict["task_name"] = list()
                aux_dict["task_grounded_parameters_dict"] = list()
                aux_dict["task_makespan"] = list()
                aux_dict["task_dispatch_time"] = list()
                for t in self.generated_plan_parsed_:
                    aux_dict["task_id"].append("task_" + str(t.action_id))
                    aux_dict["task_name"].append(t.name)
                    single_operator_grounded_parameters_dict = \
                        self.construct_single_operator_grounded_parameters_dict(t.parameters)
                    aux_dict["task_grounded_parameters_dict"].append( \
                        self.construct_grounded_single_operator_details_dict(t.name, single_operator_grounded_parameters_dict) )
                    aux_dict["task_makespan"].append(t.duration)
                    aux_dict["task_dispatch_time"].append(t.dispatch_time)

                aux_dict["plan_makespan"] = round(aux_dict["task_dispatch_time"][-1] + aux_dict["task_makespan"][-1], 2)
                aux_dict["plan_number_of_tasks"] = len(aux_dict["task_id"])
                aux_dict["plan_cost"] = round(self.get_plan_cost(), 2)
                aux_dict["plan_validity"] = True

                self.plan_dict_ = aux_dict.copy() 
            else: 
                rospy.loginfo(rospy.get_name() + ": The planner has not found a possible plan")
        else: 
            rospy.loginfo(rospy.get_name() + ": The plan is not received or parsed")
            self.planning_pipeline()

            time.sleep(2) # in seconds (to ensure the plan is ready)

            self.construct_plan_dict()          
    
    def construct_single_operator_grounded_parameters_dict(self, operator_grounded_parameters_list):
        aux_dict = dict()
        for param in operator_grounded_parameters_list:
            aux_dict[param.key] = param.value

        return aux_dict

    def construct_grounded_single_operator_details_dict(self, operator_name, operator_grounded_parameters_dict):
        ## rospy.loginfo(rospy.get_name() + ": Getting an operator's details as a dictionary")
        ## print(operator_details_ans)
        operator_details_dict = dict()

        effects_to_assert_list = list()
        effects_to_delete_list = list()
        conditions_list = list()

        self._get_operator_details.wait_for_service()
        operator_details_ans = self._get_operator_details(operator_name).op
        ## print(type(operator_details_ans.at_start_add_effects)) # they are lists

        aux_list = operator_details_ans.at_start_add_effects + operator_details_ans.at_end_add_effects
        for a in aux_list:
            if len(a.typed_parameters) == 1: # 'object quality' plan component
                plan_component_tuple = [a.name, operator_grounded_parameters_dict[a.typed_parameters[0].key]]
            elif len(a.typed_parameters) == 2: # 'object relationship' plan component
                plan_component_tuple = [a.name, \
                                        operator_grounded_parameters_dict[a.typed_parameters[0].key], \
                                        operator_grounded_parameters_dict[a.typed_parameters[1].key]]
            else:
                rospy.logerr(rospy.get_name() + ": Part of a plan condition (start add) has an unexpected format")
                plan_component_tuple = []
            
            effects_to_assert_list.append(plan_component_tuple)
        
        aux_list = operator_details_ans.at_start_del_effects + operator_details_ans.at_end_del_effects
        for a in aux_list:
            if len(a.typed_parameters) == 1: # 'object quality' plan component
                plan_component_tuple = [a.name, operator_grounded_parameters_dict[a.typed_parameters[0].key]]
            elif len(a.typed_parameters) == 2: # 'object relationship' plan component
                plan_component_tuple = [a.name, \
                                        operator_grounded_parameters_dict[a.typed_parameters[0].key], \
                                        operator_grounded_parameters_dict[a.typed_parameters[1].key]]
            else:
                rospy.logerr(rospy.get_name() + ": Part of a plan condition has an unexpected format")
                plan_component_tuple = []
            
            effects_to_delete_list.append(plan_component_tuple)

        aux_list = operator_details_ans.at_start_simple_condition + \
            operator_details_ans.over_all_simple_condition + operator_details_ans.at_end_simple_condition
        for a in aux_list:
            if len(a.typed_parameters) == 1: # 'object quality' plan component
                plan_component_tuple = [a.name, operator_grounded_parameters_dict[a.typed_parameters[0].key]]
            elif len(a.typed_parameters) == 2: # 'object relationship' plan component
                plan_component_tuple = [a.name, \
                                        operator_grounded_parameters_dict[a.typed_parameters[0].key], \
                                        operator_grounded_parameters_dict[a.typed_parameters[1].key]]
            else:
                rospy.logerr(rospy.get_name() + ": Part of a plan condition has an unexpected format")
                plan_component_tuple = []
            
            conditions_list.append(plan_component_tuple)

        operator_details_dict["effects_to_assert"] = effects_to_assert_list
        operator_details_dict["effects_to_delete"] = effects_to_delete_list
        operator_details_dict["conditions"] = conditions_list
        
        return operator_details_dict