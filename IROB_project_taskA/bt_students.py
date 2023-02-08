#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		# self.name = "bt_students.py C"

		rospy.loginfo("Initialising behaviour tree")

		# A Level
		# 
		# Robot has localized itself in the apartment
		# Navigation to picking pose
		# Cube detected
		# Complete picking task 
		# Navigation with cube to second table
		# Complete placing task
		# Cube placed on table?
		# 	Yes: end of mission
		# 	No: go back to state 2 and repeat until success. For this, you need to respawn the cube to its original pose in case it has fallen.

		b_head_up = pt.composites.Selector(
			name="Is head up?", 
			children=[
				is_head("up"),
				movehead("up")
			]
		)

		b_head_down = pt.composites.Selector(
			name="Is head down?", 
			children=[
				is_head("down"),
				movehead("down")
			]
		)

		b_tuck_arm = pt.composites.Selector(
			name="Is arm tuck?", 
			children=[
				is_arm_tuck(),
				tuckarm()
			]
		)

		Relocalize_seq = pt.composites.Sequence(
					name="Relocalize seq",
					children=[
						b_head_up,
						relocalize_robot()
					]
				)

		Is_robot_localized_branch = pt.composites.Selector(
			name="Is robot localized branch",
			children=[
				is_robot_localized(),
				Relocalize_seq
			]
		)

		Is_robot_at_table_2_branch = pt.composites.Selector(
			name="Is robot at table 2 branch",
			children=[
				is_robot_at_table("table_2"),
				pt.composites.Sequence(
					name="Move to table 2 branch",
					children=[
						b_head_up,
						move_base_to_table("table_2")
					]
				)
			]
		)

		Is_robot_at_table_1_branch = pt.composites.Selector(
			name="Is robot at table 1 branch",
			children=[
				is_robot_at_table("table_1"),
				pt.composites.Sequence(
					name="Move to table 1 branch",
					children=[
						b_head_up,
						b_tuck_arm,
						move_base_to_table("table_1")
					]
				)
			]
		)

		Is_object_on_target_seq = pt.composites.Sequence(
			name="Is object on target? seq",
			children=[
				is_cube_on_table_2_belief(),
				is_robot_at_table("table_2"),
				b_head_down,
				pt.composites.Selector(
					name="Is cube detected?",
					children=[
						cube_localized(),
						reset_cube_belief()
					]
				)
				
			]
		)

		Localize_respawn_branch = pt.composites.Selector(
			name="Localize/ Respawn cube",
			children=[
				cube_localized(),
				respawn_cube()
			]
		)

		Pickup_cube_seq = pt.composites.Sequence(
			name="Pickup cube seq",
			children=[
				Is_robot_at_table_1_branch,
				b_head_down,
				Localize_respawn_branch,
				pick_up_cube()
			]
		)

		Is_holding_cube_branch = pt.composites.Selector(
			name="Is holding cube? branch",
			children=[
				is_holding_cube(),
				Pickup_cube_seq
			]
		)


		Place_on_table_2_seq = pt.composites.Sequence(
			name="Place on table 2",
			children=[
				Is_holding_cube_branch,
				Is_robot_at_table_2_branch,
				place_cube()
			]
		)

		Is_cube_on_table_2_branch = pt.composites.Selector(
			name="Is cube on table 2?",
			children=[
				Is_object_on_target_seq,
				Place_on_table_2_seq
			]
		)
		
		# MOVE UP THE FOKEN HEAD!!! It sees the floor as a wall!
		tree = RSequence(
			name="Main sequence", 
			children=[
				Is_robot_localized_branch,
				Is_cube_on_table_2_branch
			]
		)

		super(BehaviourTree, self).__init__(tree)

		# Print BT
		rospy.loginfo(pt.display.ascii_tree(tree=tree))

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
