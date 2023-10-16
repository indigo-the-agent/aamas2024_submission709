#!/usr/bin/env python3
# coding=utf-8
import rospy
import sys
import roslib
from datetime import datetime
from rosprolog_client import PrologException, Prolog


class ROSPrologWrapperForROSPlanCRA:
    def __init__(self):
        # Define service clients

        # Define varibles
        self.client_rosprolog_ = Prolog()
        self.current_plan_kb_uri_ = ""

        if (rospy.has_param('~semantic_map_namespace')):
            self.semantic_map_namespace_ = rospy.get_param('~semantic_map_namespace')
        else:
            self.semantic_map_namespace_ = "ocra_common"

        self.plan_types_to_ontology_classes_dict_ = {
                "garment" : "ocra_cloth:'Garment'", 
                "garment-type" : "ocra_cloth:'GarmentType'",
                "pile" : "ocra_cloth:'GarmentPile'", 
                "robot" : "dul:'PhysicalAgent'",
                "human" : "dul:'PhysicalAgent'",
                "concept" : "dul:'Concept'",
                "entity" : "dul:'Entity'",
                "object" : "dul:'Object'",
                "social-object" : "dul:'SocialObject'",
                "physical-object" : "dul:'PhysicalObject'",
        }

        self.ontology_classes_to_plan_types_dict_ = {
                "ocra_cloth:'Garment'" : "garment", 
                "ocra_cloth:'GarmentType'" : "garment-type",
                "ocra_cloth:'GarmentPile'" : "pile", 
                "dul:'PhysicalAgent'" : "robot",
                "dul:'PhysicalAgent'" : "human",
                "dul:'Concept'" : "concept",
                "dul:'Entity'" : "entity",
                "dul:'Object'" : "object",
                "dul:'SocialObject'" : "social-object",
                "dul:'PhysicalObject'" : "physical-object",
        }

        self.inverse_ontology_relations_dict_ = self.get_ontology_property_and_inverse_dict()

        self.unitary_plan_predicates_to_ontology_classes_dict_ = {
            "graspable" : "dul:'Role'", 
            "free-to-manipulate" : "dul:'Role'",
            "piled" : "dul:'Role'",
            "supported" : "dul:'Role'",
            "lifted" : "dul:'Role'",
            "folded" : "dul:'Role'",
            "unfolded" : "dul:'Role'",
            #"current-number-of-garments-on-pile" : "dul:'Quality'",
        }

        self.unitary_plan_predicates_to_ontology_relations_dict_ = {
            "graspable" : "dul:'isRoleOf'", 
            "free-to-manipulate" : "dul:'isRoleOf'",
            "piled" : "dul:'isRoleOf'",
            "supported" : "dul:'isRoleOf'",
            "lifted" : "dul:'isRoleOf'",
            "folded" : "dul:'isRoleOf'",
            "unfolded" : "dul:'isRoleOf'",
            #"current-number-of-garments-on-pile" : "dul:'isQualityOf'",
        }

        self.binary_plan_predicates_to_ontology_relations_dict_ =  {
            "grasped-by" : "ocra_common:'isGraspedBy'",
            "on-pile" : "dul:'hasLocation'",
            'is-classified-by' : "dul:'isClassifiedBy'",
            'has-quality' : "dul:'hasQuality'",
        }

    
    def types_and_instances_dict_to_triples_list(self, types_with_instances_dict):
        rospy.loginfo(rospy.get_name() + ": Formatting domain plan types and their instances as triples to assert them to the ontology KB")

        triples_list = list()
        for k, v in types_with_instances_dict.items():
            for i in v:
                triple = list()
                triple.append(self.semantic_map_namespace_ + ":'" + i.replace('-','_') + "'") # ontology does not like '-'
                triple.append("rdf:'type'")
                triple.append(self.plan_types_to_ontology_classes_dict_[k])
                
                triples_list.append(triple)
        
        return triples_list

    def subgoals_dict_to_triples_list(self, subgoals_dict):
        rospy.loginfo(rospy.get_name() + ": Formatting the problem goal as a set of triples to assert them to the ontology KB")

        triples_list = list()
        goal_id = "problem_goal_" + str(datetime.utcnow()).replace(" ", "_") + "-UTC"
        triples_list.append([self.semantic_map_namespace_ + ":'" + goal_id + "'", "rdf:'type'", "dul:'Goal'"])

        for k, v in subgoals_dict.items():
            triple_st = list()
            statement_id = goal_id + "_" + k
            triple_st.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
            triple_st.append("rdf:'type'")
            triple_st.append("rdf:'Statement'")
            triples_list.append(triple_st)

            triple_st_subject = list()
            triple_st_predicate = list()
            triple_st_object = list()
            triple_concept_individual = list()
            if len(v) == 2: # unitary planning predicate
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_subject.append("rdf:'subject'")
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + v[0].replace('-','_') + "'") # (concept) instance
                triples_list.append(triple_st_subject)

                triple_st_predicate.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_predicate.append("rdf:'predicate'")
                triple_st_predicate.append(self.unitary_plan_predicates_to_ontology_relations_dict_[v[0]]) 
                triples_list.append(triple_st_predicate)

                triple_st_object.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_object.append("rdf:'object'")
                triple_st_object.append(self.semantic_map_namespace_ + ":'" + v[1].replace('-','_') + "'") # problem (object) instance
                triples_list.append(triple_st_object)

                triple_concept_individual.append(self.semantic_map_namespace_ + ":'" + v[0].replace('-','_') + "'")
                triple_concept_individual.append("rdf:'type'")
                triple_concept_individual.append(self.unitary_plan_predicates_to_ontology_classes_dict_[v[0]])
                triples_list.append(triple_concept_individual)
            elif len(v) == 3: # binary planning predicate
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_subject.append("rdf:'subject'")
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + v[1].replace('-','_') + "'") 
                triples_list.append(triple_st_subject)

                triple_st_predicate.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_predicate.append("rdf:'predicate'")
                triple_st_predicate.append(self.binary_plan_predicates_to_ontology_relations_dict_[v[0]]) 
                triples_list.append(triple_st_predicate)

                triple_st_object.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_object.append("rdf:'object'")
                triple_st_object.append(self.semantic_map_namespace_ + ":'" + v[2].replace('-','_') + "'") 
                triples_list.append(triple_st_object)
            else:
                rospy.logerr(rospy.get_name() + ": Unexpected planning predicate length")

            triple_st_description = list()
            triple_st_description.append(self.semantic_map_namespace_ + ":'" + goal_id + "'")
            triple_st_description.append("ocra_common:'describesReifiedStatement'")
            triple_st_description.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
            triples_list.append(triple_st_description)

        return triples_list

    def plan_dict_to_triples_list(self, plan_dict):
        rospy.loginfo(rospy.get_name() + ": Formatting the plan sequence details as a set of triples to assert them to the ontology KB")

        triples_list = list()
        plan_id = plan_dict["plan_id"] + "_" + str(datetime.utcnow()).replace(" ", "_") + "-UTC"
        plan_kb_uri = self.semantic_map_namespace_ + ":'" + plan_id + "'"
        triples_list.append([plan_kb_uri, "rdf:'type'", "dul:'Plan'"])

        self.current_plan_kb_uri_ = plan_kb_uri

        # knowledge about plan properties
        triples_list.append([plan_kb_uri, "ocra_common:'hasExpectedMakespan'", \
                            self.semantic_map_namespace_ + ":'" + plan_id + "_makespan'"])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_makespan'", \
                            "dul:'hasDataValue'", str(plan_dict["plan_makespan"])])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_makespan'", \
                            "rdf:'type'", "dul:'Quality'"])
        
        triples_list.append([plan_kb_uri, "ocra_common:'hasNumberOfTasks'", \
                            self.semantic_map_namespace_ + ":'" + plan_id + "_number_of_tasks'"])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_number_of_tasks'", \
                            "dul:'hasDataValue'", str(plan_dict["plan_number_of_tasks"])])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_number_of_tasks'", \
                            "rdf:'type'", "dul:'Quality'"])
        
        triples_list.append([plan_kb_uri, "ocra_common:'hasCost'", \
                            self.semantic_map_namespace_ + ":'" + plan_id + "_cost'"])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_cost'", \
                            "dul:'hasDataValue'", str(plan_dict["plan_cost"])])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_cost'", \
                            "rdf:'type'", "dul:'Quality'"])
        
        """ ## deprecated - maybe it should be a role more than a quality (e.g. 'valid_plan')
        triples_list.append([plan_kb_uri, "ocra_common:'hasValidity'", \
                            self.semantic_map_namespace_ + ":'" + plan_id + "_validity'"])
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_validity'", \
                            "dul:'hasDataValue'", str(plan_dict["plan_validity"]).lower()]) # note that in OWL, xsd:boolean goes in lowercase
        triples_list.append([self.semantic_map_namespace_ + ":'" + plan_id + "_validity'", \
                            "rdf:'type'", "dul:'Quality'"])
        """

        plan_component_count = 0 
        # knowedge about plan sequence (e.g. workflow, pre-conditions, effects, etc.)
        for i in range(0, len(plan_dict["task_id"])):
            task_kb_id = plan_dict["task_id"][i] + "_" + plan_dict["task_name"][i]
            task_kb_uri = self.semantic_map_namespace_ + ":'" + task_kb_id + "'"
            triples_list.append([task_kb_uri, "dul:'isTaskDefinedIn'", plan_kb_uri])
            triples_list.append([task_kb_uri, "rdf:'type'", "dul:'Task'"])

            for j in range(i+1, len(plan_dict["task_id"])):
                ## TODO : it might be useful to consider which agent will execute the task 

                finish_time_i = plan_dict["task_makespan"][i] + plan_dict["task_dispatch_time"][i]
                diff_finish_time_i_and_init_time_j = abs(finish_time_i - plan_dict["task_dispatch_time"][j])
                if diff_finish_time_i_and_init_time_j < 0.01:
                    next_task_kb_id = plan_dict["task_id"][j] + "_" + plan_dict["task_name"][j]
                    next_task_kb_uri = self.semantic_map_namespace_ + ":'" + next_task_kb_id + "'"
                    triples_list.append([task_kb_uri, "dul:'directlyPrecedes'", next_task_kb_uri])
                else:
                    pass

            
            ## TODO : it might be useful to include the task's makespan and dispatching time

            if plan_dict["task_grounded_parameters_dict"]:
                for k, v_list in plan_dict["task_grounded_parameters_dict"][i].items():
                    for v in v_list:
                        statement_id = plan_id + "_reified_component_" + str(plan_component_count)
                        statement_kb_uri = self.semantic_map_namespace_ + ":'" + statement_id + "'"

                        triple_st = list()
                        triple_st.append(statement_kb_uri)
                        triple_st.append("rdf:'type'")
                        triple_st.append("rdf:'Statement'")
                        triples_list.append(triple_st)

                        triple_st_description = list()
                        triple_st_description.append(plan_kb_uri)
                        triple_st_description.append("ocra_common:'describesReifiedStatement'")
                        triple_st_description.append(statement_kb_uri)
                        triples_list.append(triple_st_description)

                        triple_st_expectation = list()
                        triple_st_expectation.append(statement_kb_uri)
                        triple_st_sign = list()
                        triple_st_sign.append(statement_kb_uri)
                        triple_st_sign.append("ocra_common:'isReifiedStatementWithSign'")
                        if k == "effects_to_assert" or k == "effects_to_delete":
                            triple_st_expectation.append("ocra_common:'isExpectedEffectOf'")
                            if k == "effects_to_delete":
                                triple_st_sign.append("'False'")
                            else: 
                                triple_st_sign.append("'True'")
                        elif k == "conditions":
                            triple_st_expectation.append("ocra_common:'isExpectedConditionOf'")
                            triple_st_sign.append("'True'")
                        else:
                            rospy.logerr(rospy.get_name() + ": Wrong task expectation label")
                        triple_st_expectation.append(task_kb_uri)
                        triples_list.append(triple_st_expectation)
                        triples_list.append(triple_st_sign)
                        
                        plan_component_count += 1

                        triple_st_subject = list()
                        triple_st_predicate = list()
                        triple_st_object = list()
                        triple_concept_individual = list()
                        if len(v) == 2: # unitary planning predicate
                            triple_st_subject.append(statement_kb_uri)
                            triple_st_subject.append("rdf:'subject'")
                            triple_st_subject.append(self.semantic_map_namespace_ + ":'" + v[0].replace('-','_') + "'") # (concept) instance
                            triples_list.append(triple_st_subject)

                            triple_st_predicate.append(statement_kb_uri)
                            triple_st_predicate.append("rdf:'predicate'")
                            triple_st_predicate.append(self.unitary_plan_predicates_to_ontology_relations_dict_[v[0]]) 
                            triples_list.append(triple_st_predicate)

                            triple_st_object.append(statement_kb_uri)
                            triple_st_object.append("rdf:'object'")
                            triple_st_object.append(self.semantic_map_namespace_ + ":'" + v[1].replace('-','_') + "'") # problem (object) instance
                            triples_list.append(triple_st_object)

                            triple_concept_individual.append(self.semantic_map_namespace_ + ":'" + v[0].replace('-','_') + "'")
                            triple_concept_individual.append("rdf:'type'")
                            triple_concept_individual.append(self.unitary_plan_predicates_to_ontology_classes_dict_[v[0]])
                            triples_list.append(triple_concept_individual)
                        elif len(v) == 3: # binary planning predicate
                            triple_st_subject.append(statement_kb_uri)
                            triple_st_subject.append("rdf:'subject'")
                            triple_st_subject.append(self.semantic_map_namespace_ + ":'" + v[1].replace('-','_') + "'") 
                            triples_list.append(triple_st_subject)

                            triple_st_predicate.append(statement_kb_uri)
                            triple_st_predicate.append("rdf:'predicate'")
                            triple_st_predicate.append(self.binary_plan_predicates_to_ontology_relations_dict_[v[0]]) 
                            triples_list.append(triple_st_predicate)

                            triple_st_object.append(statement_kb_uri)
                            triple_st_object.append("rdf:'object'")
                            triple_st_object.append(self.semantic_map_namespace_ + ":'" + v[2].replace('-','_') + "'") 
                            triples_list.append(triple_st_object)
                        else:
                            rospy.logerr(rospy.get_name() + ": Unexpected planning predicate length")
            else:
                pass
            
        return triples_list

    def construct_query_text_for_single_triple_assertion(self, triple_subject, triple_relation, triple_object, add_final_dot, add_inverse_triple):
        ## rospy.loginfo(rospy.get_name() + ": Construct query text for single triple assertion")
        query_text = "kb_project(triple(" + triple_subject + ", " + triple_relation + ", " + triple_object +"))"
        if add_inverse_triple:
            tp_relation_name = triple_relation.split("'")[1] # it works if the relation complies with "rdf:'type'" format
            if (tp_relation_name in self.inverse_ontology_relations_dict_):
                tp_relation_uri = triple_relation.split("'")[0] # it works if the relation complies with "rdf:'type'" format
                inverted_triple_relation = tp_relation_uri + "'" + self.inverse_ontology_relations_dict_[tp_relation_name] + "'"
                query_text = query_text + ", " + "kb_project(triple(" + triple_object + ", " + \
                    inverted_triple_relation + ", " + triple_subject + "))"

                ## print(query_text)
            else: 
                rospy.logwarn(rospy.get_name() + ": The triple property %s does not have inverse.", triple_relation) # for data properties is expected
        else: 
            pass
        if add_final_dot:
            query_text = query_text + "."
        else:
            pass

        ## query_text = query_text.replace('-','_') # ontology does not like '-'

        return query_text

    def construct_query_text_for_multiple_triples_assertion(self, triples, add_inverse_triple):
        rospy.loginfo(rospy.get_name() + ": Construct query text for multiple triples assertion")

        query_text = self.construct_query_text_for_single_triple_assertion(triples[0][0], triples[0][1], triples[0][2], False, add_inverse_triple) 

        for i in range(1, len(triples)):
            query_text = query_text + ", " + \
                self.construct_query_text_for_single_triple_assertion(triples[i][0], triples[i][1], triples[i][2], False, add_inverse_triple) 
        
        query_text = query_text + "."

        return query_text

    def rosprolog_assertion_query(self, query_text):
        rospy.loginfo(rospy.get_name() + ": New query to assert ontological knowledge")

        # query the knowldge base
        query = self.client_rosprolog_.query(query_text)
        query.finish()

    def get_ontology_property_and_inverse_dict(self):
        ont_property_inverse_dict = dict()

        query = self.client_rosprolog_.query("kb_call(triple(S, owl:'inverseOf', O))")

        for solution in query.solutions():
            subj_ = solution['S'].split('#')[-1] # without the ontology URI
            obj_ = solution['O'].split('#')[-1] # without the ontology URI

            ont_property_inverse_dict[subj_] = obj_
            ont_property_inverse_dict[obj_] = subj_

        query.finish()

        return ont_property_inverse_dict

