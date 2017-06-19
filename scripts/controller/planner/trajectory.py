# Harrit Diwan
# 4-18-2017
# ------- Contact Info -------
# Email: hdiwan@clemson.edu
# Phone: 864-553-3973
# Organization: Clemson Univeristy International Center for AUtomotive Research
# Feel free to contact me any time if you have questions.
# --------------------------------------------#


import numpy as np
import math
import time
from frame import Frame

import numpy as np
import math
import time
from frame import Frame


class Trajectory_Planner_Blend:
	def __init__(self):
		self.itr = 1
		self.X = []
		self.Y = []
		self.V_X = []
		self.V_Y = []
		self.A_X = []
		self.A_Y = []
		self.H = []
		self.X_old = []
		self.Y_old = []
		self.x_closest = 0
		self.y_closest = 0

	# --------tentacle (trajectory) generation--------#
	def trajectory_planner(self, current_wp_x, current_wp_y, next_wp_x, next_wp_y, current_wp_v, next_wp_v, current_x,
						   current_y, inst_v, heading, inst_heading, wp_traversed):
		C_alpha = 0.3
		localtime = time.time()
		n = 16  # Number of sets of speeds
		rho = 1.15
		v_s = 0  # lowest speed in the set (m/s)
		v_e = 10  # max speed in the set
		phi = 1.2 * (np.pi / 2)
		q = []
		l = []
		R_j = []  # Base Radius of curvature
		v_j = []  # velocity of each tentacle set
		r_k = [[]]  # radius of curvature of each tentacle
		l_k = [[]]  # length of each tentacle
		theta_k = [[]]

		for j in range(0, 15):
			q.append(j / (n - 1))
			l.append(8 + (33.5 * math.pow(q[j], 1.2)))
			R_j.append(l[j] / (phi * (1 - math.pow(q[j], 0.9))))
			v_j.append(v_s + (v_e - v_s) * (math.pow(q[j], 1.2)))
			r_k.append([])
			l_k.append([])
			theta_k.append([])

			for k in range(0, 80):
				if k < 40:
					r_k[j].append(math.pow(rho, k) * R_j[j])
					l_k[j].append(l[j] + 20 * math.sqrt(k / 40))
					theta_k[j].append(l_k[j][k] / r_k[j][k])

				elif k > 40:
					r_k[j].append((-math.pow(rho, k - 41)) * R_j[j])
					l_k[j].append(l[j] + 20 * math.sqrt((k - 40) / 40))
					theta_k[j].append(l_k[j][k] / r_k[j][k])

				else:
					r_k[j].append(float("inf"))
					l_k[j].append(l[j] + 20 * math.sqrt(k / 40))
					theta_k[j].append(l_k[j][k] / r_k[j][k])

		# itr = itr + 1
		m = 0
		ref = abs(inst_v - v_j[0])
		for j in range(0, 15):
			speed_diff = abs(inst_v - v_j[j])
			if speed_diff < ref:
				ref = speed_diff
				m = j

		current_dist = []
		radius = []
		pts_x = []
		pts_y = []
		dist_pts_frm_wpt = []
		traj_val = []
		th = []
		cur_idx = 0

		if wp_traversed:
			self.X = []
			self.Y = []
			self.V_X = []
			self.V_Y = []
			self.A_X = []
			self.A_Y = []
			self.H = []

			if current_wp_v == 0:
				l = 0.5 * 2.2
				#d = 0.01 * 2.2
			else:
				l = 0.5 * current_wp_v
				#d = 0.01 * current_wp_v

			# dynamic number of trajectory points (to improve)
			#num_pts = max(10, int(math.sqrt(next_wp_x ** 2 + next_wp_y ** 2) / 0.02))
			num_pts = 30
			next_wp_x = next_wp_x - self.x_closest
			next_wp_y = next_wp_y - self.y_closest

			if next_wp_x > 0 and next_wp_y >= 0:
				for k in range(0, 39):
					dist_wpts = math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2)
					radius.append(r_k[m][k])
					pt_x = (dist_wpts ** 2) / (2 * r_k[m][k])
					pts_x.append(pt_x)
					pt_y = math.sqrt(abs((dist_wpts / (2 * r_k[m][k])) * (2 * r_k[m][k] - dist_wpts / (2 * r_k[m][k]))))
					pts_y.append(pt_y)
					t1 = math.atan(pt_y / pt_x)
					t = np.pi - 2 * t1
					th.append(t)
					d = math.sqrt((next_wp_x - pt_x) ** 2 + (next_wp_y - pt_y) ** 2)
					dist_pts_frm_wpt.append(d)
					heading = np.pi/2 - inst_heading + heading
					alpha = abs(np.pi / 2 - t - heading)
					traj_val.append(C_alpha * alpha + d)

				des_index = traj_val.index(min(traj_val))

				#d_l = (math.sqrt(next_wp_x ** 2 + next_wp_y ** 2))
				d_l = (math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2))
				# theta = l / radius[des_index]
				theta = 2 * math.asin(d_l / (2 * radius[des_index]))

				for i in range(0, num_pts):
					#if i < len(X_old) - 1:
					#	self.X.append((1 - i / num_pts) * X_old[i] + (i / num_pts) * radius[des_index] * (
					#	1 - math.cos(i * theta / num_pts)))
					#	self.Y.append((1 - i / num_pts) * Y_old[i] + (i / num_pts) * radius[des_index] * math.sin(
					#		i * theta / num_pts))
					#else:
					#	self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.X.append(self.x_closest + radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					self.Y.append(self.y_closest + radius[des_index] * math.sin(i * theta / num_pts))
					self.A_X.append(
						next_wp_x * next_wp_x * math.cos(np.pi / 2 - th[des_index]) / (2 * pts_x[des_index]))
					self.A_Y.append(
						(next_wp_y * next_wp_y * math.sin(np.pi / 2 - th[des_index]) - current_wp_y ** 2) / (
							2 * pts_y[des_index]))
					if i == 0:
						self.V_X.append(0)
						self.V_Y.append(current_wp_y)
					else:
						self.V_X.append(-math.sqrt(abs(2 * self.A_X[i] * self.X[i])))
						self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					if wp_traversed:
						self.H.append(np.pi / 2 - (i * theta / num_pts))
					else:
						self.H.append(inst_heading - (i * theta / num_pts))

			if next_wp_x < 0 and next_wp_y >= 0:
				for k in range(41, 80):
					dist_wpts = math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2)
					radius.append(r_k[m][k])
					pt_x = (dist_wpts ** 2) / (2 * r_k[m][k])
					pts_x.append(pt_x)
					pt_y = math.sqrt(abs((dist_wpts / (2 * r_k[m][k])) * (2 * r_k[m][k] - dist_wpts / (2 * r_k[m][k]))))
					pts_y.append(pt_y)
					t1 = math.atan(abs(pt_y / pt_x))
					t = np.pi - 2 * t1
					th.append(t)
					d = math.sqrt((next_wp_x - pt_x) ** 2 + (next_wp_y - pt_y) ** 2)
					dist_pts_frm_wpt.append(d)
					alpha = abs(np.pi / 2 + t - heading)
					traj_val.append(C_alpha * alpha + d)

				des_index = traj_val.index(min(traj_val))

				# theta = l / radius[des_index]
				d_l = (math.sqrt(next_wp_x ** 2 + next_wp_y ** 2))
				theta = 2 * math.asin(d_l / (2 * radius[des_index]))

				for i in range(0, num_pts):
					#if i < len(X_old) - 1:
					#	self.X.append((i / num_pts) * X_old[i] + (1 - i / num_pts) * radius[des_index] * (
					#	1 - math.cos(i * theta / num_pts)))
					#	self.Y.append((1 - i / num_pts) * Y_old[i] + (1 - i / num_pts) * radius[des_index] * math.sin(
					#		i * theta / num_pts))
					#else:
					#	self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.X.append(self.x_closest + radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					self.Y.append(self.y_closest + radius[des_index] * math.sin(i * theta / num_pts))
					self.A_X.append(
						next_wp_x * next_wp_x * math.cos(np.pi / 2 - th[des_index]) / (2 * pts_x[des_index]))
					self.A_Y.append(
						(next_wp_y * next_wp_y * math.sin(np.pi / 2 - th[des_index]) - current_wp_y ** 2) / (
							2 * pts_y[des_index]))
					if i == 0:
						self.V_X.append(0)
						self.V_Y.append(current_wp_y)
					else:
						self.V_X.append(-math.sqrt(abs(2 * self.A_X[i] * self.X[i])))
						self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					if wp_traversed:
						self.H.append(np.pi / 2 + i * theta / num_pts)
					else:
						self.H.append(inst_heading - i * theta / num_pts)

			if next_wp_x == 0 and next_wp_y >= 0:
				k = 40
				for i in range(0, num_pts):
					self.X.append(self.x_closest)
					self.Y.append(self.y_closest + (i * d))
					self.A_X.append(0)
					self.A_Y.append((next_wp_v ** 2 - current_wp_v ** 2) / (2 * 30 * d))
					self.V_X.append(0)
					self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					if wp_traversed:
						self.H.append(np.pi/2)
					else:
						self.H.append(inst_heading)

			self.X_old = self.X
			self.Y_old = self.Y

			return [self.X, self.Y, self.V_X, self.V_Y, self.A_X, self.A_Y, self.H, localtime]


		else:
			print("x_old", self.X_old)
			for i in range(len(self.X_old)):
				current_dist.append(math.sqrt((self.X_old[i] - current_x) ** 2 + (self.Y_old[i] - current_y) ** 2))
			#print(current_dist)
			if self.X_old:
				if len(current_dist) > 0:
					cur_idx = current_dist.index(min(current_dist))

				self.x_closest = self.X_old[cur_idx]
				self.y_closest = self.Y_old[cur_idx]
			else:
				self.x_closest=0
				self.y_closest=0

			self.X = []
			self.Y = []
			self.V_X = []
			self.V_Y = []
			self.A_X = []
			self.A_Y = []
			self.H = []

			if current_wp_v == 0:
				l = 0.5 * 2.2
				#d = 0.01 * 2.2
			else:
				l = 0.5 * current_wp_v
				#d = 0.01 * current_wp_v

			# dynamic number of trajectory points (to improve)
			#num_pts = max(10, int(math.sqrt(next_wp_x ** 2 + next_wp_y ** 2) / 0.02))
			num_pts = 30
			next_wp_x = next_wp_x - self.x_closest
			next_wp_y = next_wp_y - self.y_closest

			if next_wp_x > 0 and next_wp_y >= 0:
				for k in range(0, 39):
					dist_wpts = math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2)
					radius.append(r_k[m][k])
					pt_x = (dist_wpts ** 2) / (2 * r_k[m][k])
					pts_x.append(pt_x)
					pt_y = math.sqrt(abs((dist_wpts / (2 * r_k[m][k])) * (2 * r_k[m][k] - dist_wpts / (2 * r_k[m][k]))))
					pts_y.append(pt_y)
					t1 = math.atan(pt_y / pt_x)
					t = np.pi - 2 * t1
					th.append(t)
					d = math.sqrt((next_wp_x - pt_x) ** 2 + (next_wp_y - pt_y) ** 2)
					dist_pts_frm_wpt.append(d)
					heading = np.pi/2 - inst_heading + heading
					alpha = abs(np.pi / 2 - t - heading)
					traj_val.append(C_alpha * alpha + d)

				des_index = traj_val.index(min(traj_val))

				#d_l = (math.sqrt(next_wp_x ** 2 + next_wp_y ** 2))
				d_l = (math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2))
				# theta = l / radius[des_index]
				theta = 2 * math.asin(d_l / (2 * radius[des_index]))

				for i in range(0, num_pts):
					#if i < len(X_old) - 1:
					#	self.X.append((1 - i / num_pts) * X_old[i] + (i / num_pts) * radius[des_index] * (
					#	1 - math.cos(i * theta / num_pts)))
					#	self.Y.append((1 - i / num_pts) * Y_old[i] + (i / num_pts) * radius[des_index] * math.sin(
					#		i * theta / num_pts))
					#else:
					#	self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.X.append(self.x_closest + radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					self.Y.append(self.y_closest + radius[des_index] * math.sin(i * theta / num_pts))
					self.A_X.append(
						next_wp_x * next_wp_x * math.cos(np.pi / 2 - th[des_index]) / (2 * pts_x[des_index]))
					self.A_Y.append(
						(next_wp_y * next_wp_y * math.sin(np.pi / 2 - th[des_index]) - current_wp_y ** 2) / (
							2 * pts_y[des_index]))
					if i == 0:
						self.V_X.append(0)
						self.V_Y.append(current_wp_y)
					else:
						self.V_X.append(-math.sqrt(abs(2 * self.A_X[i] * self.X[i])))
						self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					if wp_traversed:
						self.H.append(np.pi / 2 - (i * theta / num_pts))
					else:
						self.H.append(inst_heading - (i * theta / num_pts))

			if next_wp_x < 0 and next_wp_y >= 0:
				for k in range(41, 80):
					dist_wpts = math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2)
					radius.append(r_k[m][k])
					pt_x = (dist_wpts ** 2) / (2 * r_k[m][k])
					pts_x.append(pt_x)
					pt_y = math.sqrt(abs((dist_wpts / (2 * r_k[m][k])) * (2 * r_k[m][k] - dist_wpts / (2 * r_k[m][k]))))
					pts_y.append(pt_y)
					t1 = math.atan(abs(pt_y / pt_x))
					t = np.pi - 2 * t1
					th.append(t)
					d = math.sqrt((next_wp_x - pt_x) ** 2 + (next_wp_y - pt_y) ** 2)
					dist_pts_frm_wpt.append(d)
					alpha = abs(np.pi / 2 + t - heading)
					traj_val.append(C_alpha * alpha + d)

				des_index = traj_val.index(min(traj_val))

				# theta = l / radius[des_index]
				d_l = (math.sqrt(next_wp_x ** 2 + next_wp_y ** 2))
				theta = 2 * math.asin(d_l / (2 * radius[des_index]))

				for i in range(0, num_pts):
					#if i < len(X_old) - 1:
					#	self.X.append((i / num_pts) * X_old[i] + (1 - i / num_pts) * radius[des_index] * (
					#	1 - math.cos(i * theta / num_pts)))
					#	self.Y.append((1 - i / num_pts) * Y_old[i] + (1 - i / num_pts) * radius[des_index] * math.sin(
					#		i * theta / num_pts))
					#else:
					#	self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.X.append(self.x_closest + radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					self.Y.append(self.y_closest + radius[des_index] * math.sin(i * theta / num_pts))
					self.A_X.append(
						next_wp_x * next_wp_x * math.cos(np.pi / 2 - th[des_index]) / (2 * pts_x[des_index]))
					self.A_Y.append(
						(next_wp_y * next_wp_y * math.sin(np.pi / 2 - th[des_index]) - current_wp_y ** 2) / (
							2 * pts_y[des_index]))
					if i == 0:
						self.V_X.append(0)
						self.V_Y.append(current_wp_y)
					else:
						self.V_X.append(-math.sqrt(abs(2 * self.A_X[i] * self.X[i])))
						self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					if wp_traversed:
						self.H.append(np.pi / 2 + i * theta / num_pts)
					else:
						self.H.append(inst_heading - i * theta / num_pts)

			if next_wp_x == 0 and next_wp_y >= 0:
				k = 40
				for i in range(0, num_pts):
					self.X.append(self.x_closest)
					self.Y.append(self.y_closest + (i * d))
					self.A_X.append(0)
					self.A_Y.append((next_wp_v ** 2 - current_wp_v ** 2) / (2 * 30 * d))
					self.V_X.append(0)
					self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					if wp_traversed:
						self.H.append(np.pi/2)
					else:
						self.H.append(inst_heading)

			return [self.X, self.Y, self.V_X, self.V_Y, self.A_X, self.A_Y, self.H, localtime]



		# else:
		# 	for i in range(len(self.X)):
		# 		current_dist.append(math.sqrt((self.X[i] - current_x) ** 2 + (self.Y[i] - current_y) ** 2))
		# 	# print(current_dist)
		# 	if len(current_dist) > 0:
		# 		cur_idx = current_dist.index(min(current_dist))
		# 		for i in range(len(self.X) - cur_idx):
		# 			X_.append(self.X[cur_idx + i])
		# 			# print(X_)
		# 			Y_.append(self.Y[cur_idx + i])
		# 			V_X_.append(self.V_X[cur_idx + i])
		# 			V_Y_.append(self.V_Y[cur_idx + i])
		# 			A_X_.append(self.A_X[cur_idx + i])
		# 			A_Y_.append(self.A_Y[cur_idx + i])
		# 			H_.append(self.H[cur_idx + i])
		# 		return [X_, Y_, V_X_, V_Y_, A_X_, A_Y_, H_, localtime]


class Trajectory_Planner:
	def __init__(self):
		self.itr = 1
		self.X = []
		self.Y = []
		self.V_X = []
		self.V_Y = []
		self.A_X = []
		self.A_Y = []
		self.H = []

	# --------tentacle (trajectory) generation--------#
	def trajectory_planner(self, current_wp_x, current_wp_y, next_wp_x, next_wp_y, current_wp_v, next_wp_v, current_x,
						   current_y, inst_v, heading, inst_heading, wp_traversed):
		C_alpha = 0.3
		A1 = 1
		A2 = 0.1
		A3 = 0.1
		A4 = 0.1
		A5 = 0.1
		steering_kappa = 0.5
		localtime = time.time()
		n = 16  # Number of sets of speeds
		rho = 1.15
		v_s = 0  # lowest speed in the set (m/s)
		v_e = 10  # max speed in the set
		phi = 1.2 * (np.pi / 2)
		q = []
		l = []
		R_j = []  # Base Radius of curvature
		v_j = []  # velocity of each tentacle set
		r_k = [[]]  # radius of curvature of each tentacle
		l_k = [[]]  # length of each tentacle
		theta_k = [[]]

		for j in range(0, 15):
			q.append(j / (n - 1))
			l.append(8 + (33.5 * math.pow(q[j], 1.2)))
			R_j.append(l[j] / (phi * (1 - math.pow(q[j], 0.9))))
			v_j.append(v_s + (v_e - v_s) * (math.pow(q[j], 1.2)))
			r_k.append([])
			l_k.append([])
			theta_k.append([])

			for k in range(0, 80):
				if k < 40:
					r_k[j].append(math.pow(rho, k) * R_j[j])
					l_k[j].append(l[j] + 20 * math.sqrt(k / 40))
					theta_k[j].append(l_k[j][k] / r_k[j][k])

				elif k > 40:
					r_k[j].append((-math.pow(rho, k - 41)) * R_j[j])
					l_k[j].append(l[j] + 20 * math.sqrt((k - 40) / 40))
					theta_k[j].append(l_k[j][k] / r_k[j][k])

				else:
					r_k[j].append(float("inf"))
					l_k[j].append(l[j] + 20 * math.sqrt(k / 40))
					theta_k[j].append(l_k[j][k] / r_k[j][k])

		# itr = itr + 1
		m = 0
		ref = abs(inst_v - v_j[0])
		for j in range(0, 15):
			speed_diff = abs(inst_v - v_j[j])
			if speed_diff < ref:
				ref = speed_diff
				m = j

		current_dist = []
		radius = []
		pts_x = []
		pts_y = []
		dist_pts_frm_wpt = []
		traj_val = []
		th = []
		X_ = []
		Y_ = []
		V_X_ = []
		V_Y_ = []
		A_X_ = []
		A_Y_ = []
		H_ = []

		if wp_traversed:
			X_old = self.X
			Y_old = self.Y
			self.X = []
			self.Y = []
			self.V_X = []
			self.V_Y = []
			self.A_X = []
			self.A_Y = []
			self.H = []
			if current_wp_v == 0:
				l = 0.5 * 2.2
				d = 0.01 * 2.2
			else:
				l = 0.5 * current_wp_v
				d = 0.01 * current_wp_v

			# dynamic number of trajectory points (to improve)
			#num_pts = max(30, int(math.sqrt(next_wp_x ** 2 + next_wp_y ** 2) / 0.02))
			num_pts = 30
			if next_wp_x > 0 and next_wp_y >= 0:
				for k in range(0, 39):
					dist_wpts = math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2)
					radius.append(r_k[m][k])
					pt_x = (dist_wpts ** 2) / (2 * r_k[m][k])
					pts_x.append(pt_x)
					pt_y = math.sqrt(abs((dist_wpts / (2 * r_k[m][k])) * (2 * r_k[m][k] - dist_wpts / (2 * r_k[m][k]))))
					pts_y.append(pt_y)
					t1 = math.atan(pt_y / pt_x)
					t = np.pi - 2 * t1
					th.append(t)
					d = math.sqrt((next_wp_x - pt_x) ** 2 + (next_wp_y - pt_y) ** 2)
					dist_pts_frm_wpt.append(d)
					alpha = abs(np.pi / 2 - t - heading)
					traj_val.append(C_alpha * alpha + d)

				des_index = traj_val.index(min(traj_val))

				d_l = (math.sqrt(next_wp_x ** 2 + next_wp_y ** 2))
				# theta = l / radius[des_index]
				theta = 2 * math.asin(d_l / (2 * radius[des_index]))

				for i in range(0, num_pts):
					# if i < len(X_old)-1:
					#	self.X.append((1-i/num_pts)*X_old[i] + (i/num_pts)*radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append((1-i/num_pts)*Y_old[i] + (i/num_pts)*radius[des_index] * math.sin(i * theta / num_pts))
					# else:
					#	self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.A_X.append(
						next_wp_x * next_wp_x * math.cos(np.pi / 2 - th[des_index]) / (2 * pts_x[des_index]))
					self.A_Y.append(
						(next_wp_y * next_wp_y * math.sin(np.pi / 2 - th[des_index]) - current_wp_y ** 2) / (
							2 * pts_y[des_index]))
					if i == 0:
						self.V_X.append(0)
						self.V_Y.append(current_wp_y)
					else:
						self.V_X.append(-math.sqrt(abs(2 * self.A_X[i] * self.X[i])))
						self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					self.H.append(np.pi / 2 - (i * theta / num_pts))

			if next_wp_x < 0 and next_wp_y >= 0:
				for k in range(41, 80):
					dist_wpts = math.sqrt((next_wp_x) ** 2 + (next_wp_y) ** 2)
					radius.append(r_k[m][k])
					pt_x = (dist_wpts ** 2) / (2 * r_k[m][k])
					pts_x.append(pt_x)
					pt_y = math.sqrt(abs((dist_wpts / (2 * r_k[m][k])) * (2 * r_k[m][k] - dist_wpts / (2 * r_k[m][k]))))
					pts_y.append(pt_y)
					t1 = math.atan(abs(pt_y / pt_x))
					t = np.pi - 2 * t1
					th.append(t)
					d = math.sqrt((next_wp_x - pt_x) ** 2 + (next_wp_y - pt_y) ** 2)
					dist_pts_frm_wpt.append(d)
					alpha = abs(np.pi / 2 + t - heading)
					traj_val.append(C_alpha * alpha + d)

				des_index = traj_val.index(min(traj_val))

				# theta = l / radius[des_index]
				d_l = (math.sqrt(next_wp_x ** 2 + next_wp_y ** 2))
				theta = 2 * math.asin(d_l / (2 * radius[des_index]))

				for i in range(0, num_pts):
					# if i < len(X_old)-1:
					#	self.X.append((i/num_pts)*X_old[i] + (1-i/num_pts)*radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append((1-i/num_pts)*Y_old[i] + (1-i/num_pts)*radius[des_index] * math.sin(i * theta / num_pts))
					# else:
					#	self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					#	self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.X.append(radius[des_index] * (1 - math.cos(i * theta / num_pts)))
					self.Y.append(radius[des_index] * math.sin(i * theta / num_pts))
					self.A_X.append(
						next_wp_x * next_wp_x * math.cos(np.pi / 2 - th[des_index]) / (2 * pts_x[des_index]))
					self.A_Y.append(
						(next_wp_y * next_wp_y * math.sin(np.pi / 2 - th[des_index]) - current_wp_y ** 2) / (
							2 * pts_y[des_index]))
					if i == 0:
						self.V_X.append(0)
						self.V_Y.append(current_wp_y)
					else:
						self.V_X.append(-math.sqrt(abs(2 * self.A_X[i] * self.X[i])))
						self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					self.H.append(np.pi / 2 + i * theta / num_pts)

			if next_wp_x == 0 and next_wp_y >= 0:
				k = 40
				for i in range(0, num_pts):
					self.X.append(0)
					self.Y.append(i * d)
					self.A_X.append(0)
					self.A_Y.append((next_wp_v ** 2 - current_wp_v ** 2) / (2 * 50 * d))
					self.V_X.append(0)
					self.V_Y.append(math.sqrt(abs(current_wp_y + 2 * self.A_Y[i] * self.Y[i])))
					self.H.append(np.pi / 2)

			return [self.X, self.Y, self.V_X, self.V_Y, self.A_X, self.A_Y, self.H, localtime]

		else:
			return [self.X, self.Y, self.V_X, self.V_Y, self.A_X, self.A_Y, self.H, localtime]

			for i in range(len(self.X)):
				current_dist.append(math.sqrt((self.X[i] - current_x) ** 2 + (self.Y[i] - current_y) ** 2))
			# print(current_dist)
			if len(current_dist) > 0:
				cur_idx = current_dist.index(min(current_dist))
				for i in range(len(self.X) - cur_idx):
					X_.append(self.X[cur_idx + i])
					# print(X_)
					Y_.append(self.Y[cur_idx + i])
					V_X_.append(self.V_X[cur_idx + i])
					V_Y_.append(self.V_Y[cur_idx + i])
					A_X_.append(self.A_X[cur_idx + i])
					A_Y_.append(self.A_Y[cur_idx + i])
					H_.append(self.H[cur_idx + i])
				return [X_, Y_, V_X_, V_Y_, A_X_, A_Y_, H_, localtime]
	

if __name__ == "__main__":
	car = (10, 32, math.radians(45))
	fr = Frame(car[0], car[1], car[2])
	pt = (10, 34)
	lc_x, lc_y = fr.global2local(pt[0], pt[1])
	print lc_x, lc_y
	gc_x, gc_y = fr.local2global(lc_x, lc_y)
	print gc_x, gc_y
