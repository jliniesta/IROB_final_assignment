#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):
		# self.name = "bt_students.py C"

		rospy.loginfo("Initialising behaviour tree")

		# C LEVEL:
		# 
		# Detect cube
		# Complete picking task 
		# Carry cube to second table
		# Complete placing task
		# Cube placed on table?
		#     Yes: end of task
		#     No: go back to initial state in front of table 1

		# tuck the arm
		b0 = tuckarm()
		
		
		# lower head
		b1 = movehead("down")
		
		
		# localize cube
		b2 = cube_localized()
		
		# pick up cube
		b3 = pick_up_cube()

		# turn to table2
		b4 = moveb(0.5, 0, 64, "Rotate to table 2")

		# go to table2
		b5 = moveb(0, 0.5, 21, "Move to table 1")

		# place cube
		b6 = place_cube()

		go_to_table1 = pt.composites.Sequence(name = "move to table 1", 
					children = [moveb(-0.5, 0, 64, "Rotate to table 1"), moveb(0, 0.5, 21, "Go to table 1")]
					)

		# Checking cube is on table 2, if not move to table 1
		b7 = pt.composites.Selector(
			name="Detect cube",
			children=[cube_localized(), go_to_table1]
		)

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6, b7])
		
		super(BehaviourTree, self).__init__(tree)

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
