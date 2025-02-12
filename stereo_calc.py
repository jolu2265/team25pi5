import time
import numpy as np
from multiprocessing import Manager

baseline = 0.2794 # gotta check if m or in
focal_length = 975 # was 964 should do: 2954.49 or 1230.32, 1080, 1040 was best, 1024, 870

def compute_stereo_and_pose(left_data, right_data, left_coords, right_coords):
	while True:
		
		if left_data and right_data:
			
			left_distance, left_pose = left_data[0]
			right_distance, right_pose = right_data[0]
			
			# print(left_coords[0][0])
			
			# disp = abs(left_pose[0] - right_pose[0])
			
			l_x = left_coords
			l_tl = l_x[0][0][0][0]
			l_tr = l_x[0][0][1][0]
			l_br = l_x[0][0][2][0]
			l_bl = l_x[0][0][3][0]
			l_c = (l_tl + l_tr + l_br + l_bl)/4
			
			r_x = right_coords
			r_tl = r_x[0][0][0][0]
			r_tr = r_x[0][0][1][0]
			r_br = r_x[0][0][2][0]
			r_bl = r_x[0][0][3][0]
			r_c = (r_tl + r_tr + r_br + r_bl)/4
			
			
			disp = abs(l_c - r_c)
			
			if disp > 0:
				depth = (baseline * focal_length) / disp
				# print(depth)
				# print(left_pose)
				# print((left_distance + right_distance) / 2)
			else:
				print("THIS IS THE AVERAGE")
				depth = (left_distance + right_distance) / 2
		time.sleep(0.1)

'''		
if __name__ == "__main__":
	manager = Manager()
	left_data = manager.list()
	right_data = manager.list()
	compute_stereo_and_pose(left_data, right_data) 
'''
