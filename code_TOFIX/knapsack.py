import random
import bisect
import math
from sympy import *	
import numpy as np
from model import compute_prefixes
import time
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from shapely import geometry
from functools import reduce
import operator
import pandas as pd
import interval
import os
from opt_CPLEX import *
import scipy.stats as stats
from ast import literal_eval

############## FUNCTION FROM RETTA_FINAL ###################
def compute_wind_classes(n_class,median=False):
	## Function that construct the wind classes representation 
	WIND_CLASSES = {}
	WIND_SECTOR = {}
	RELATIVE_WIND_VALUES= []
	n_sector = n_class * 2
	sector =(int)(360 / n_sector)
	angle = 0

	lower_bound = 1
	upper_bound = sector
	while(upper_bound<= 360):
		if(not median):
			if(abs(math.cos(math.radians(lower_bound))) > abs(math.cos(math.radians(upper_bound)))):
				angle = lower_bound
			else:
				angle = upper_bound
		else:
			angle = math.floor(statistics.median(range(lower_bound,(upper_bound+1))))
		WIND_CLASSES[range(lower_bound,(upper_bound+1))] = angle % 360
		RELATIVE_WIND_VALUES.append(angle % 360)
		WIND_SECTOR[range(lower_bound,(upper_bound+1))] = math.floor(lower_bound/sector)
		lower_bound = lower_bound + sector
		upper_bound = upper_bound + sector
	return WIND_CLASSES, WIND_SECTOR, RELATIVE_WIND_VALUES

def wind_class(angle):
	if(angle == 0):
		angle = 360
	for key in WIND_SECTOR_REP:
		if angle in (key):
			return WIND_SECTOR_REP[key]

def wind_sector(angle):
	if(angle == 0):
		angle = 360
	for key in WIND_SECTOR:
		if angle in (key):
			return WIND_SECTOR[key]

def get_side(segment,DP):
	pos = ""
	if(np.cross(DP - segment.p1, segment.p2 - segment.p1) < 0):
		pos="SX"
	else:
		pos="DX"
	return pos

def list_delivery_slack(n_class, atan_MT,WIND_DIRECTION):
	## Function that return the list of angles to test in an adaptive way
	list_neg_alpha = []
	list_pos_alpha = []

	t = 0
	n_sector = n_class * 2
	sector = 360 / n_sector
	pos_slack = (WIND_DIRECTION - atan_MT) % sector 
	neg_slack = (-(WIND_DIRECTION - atan_MT)) % sector
	
	#if ((WIND_DIRECTION - atan_MT)%360) % sector == 0:
	if pos_slack == 0:
		list_neg_alpha.append(0)
		t = (int) (n_sector / n_class)
		neg_slack = sector
		pos_slack = sector
	else:
		t = (int)(n_sector / n_class) + 1
	#print("P ",pos_slack," N ",neg_slack)
	for i in range(t):
		neg = neg_slack + i*sector
		pos = pos_slack + i*sector - 1
		list_neg_alpha.append(neg)
		list_pos_alpha.append(pos)
	list_pos_alpha.append(90)
	list_neg_alpha.append(90)
	if(pos_slack - sector == 0):
		list_pos_alpha[t] = list_pos_alpha[t] - 1
		list_pos_alpha.append(90)

	#print("P ", list_pos_alpha," N ", list_neg_alpha)
	return list_pos_alpha, list_neg_alpha

def find_sectors(wind,arctan,pslack,nslack,n_class):
	res = ""
	start = (wind - arctan) % 360
	lower = start
	upper = 0
	sector_width = 360/(n_class*2)
	res = res + "POSITIVE ALPHA\n"
	print("POSITIVE ALPHA")
	for index in range(len(pslack)):
		upper = (start - pslack[index]) % 360
		repre = wind_class(upper)
		print("[ "+str(lower)+","+str(upper)+" ]-> SECTOR: "+str(wind_sector(repre))+" mu/sin(alpha)[payload] -> "+str(abs(UNIT_ENERGY[2,repre]/math.sin(math.radians(pslack[index]))))+" mu/sin(alpha)[empty] ->"+str(abs(UNIT_ENERGY[0,repre]/math.sin(math.radians(pslack[index]))))+"\n")
		res = res + "[ "+str(lower)+","+str(upper)+" ]-> SECTOR: "+str(wind_sector(repre))+" mu/sin(alpha)[payload] -> "+str(abs(UNIT_ENERGY[2,repre]/math.sin(math.radians(pslack[index]))))+" mu/sin(alpha)[empty] ->"+str(abs(UNIT_ENERGY[0,repre]/math.sin(math.radians(pslack[index]))))+"\n"
		lower = (start - pslack[index]-1) % 360
	

	print("NEGATIVE ALPHA")
	res = res + "NEGATIVE ALPHA\n"
	lower = start
	upper = 0
	for index in range(len(pslack)):
		upper = (start + nslack[index]) % 360
		repre = wind_class(lower)
		print("[ "+str(lower)+","+str(upper)+" ]-> SECTOR: "+str(wind_sector(repre))+" mu/sin(alpha)[payload] -> "+str(abs(UNIT_ENERGY[2,repre]/math.sin(math.radians(nslack[index]))))+" mu/sin(alpha)[empty] ->"+str(abs(UNIT_ENERGY[0,repre]/math.sin(math.radians(nslack[index]))))+"\n")
		res = res + "[ "+str(lower)+","+str(upper)+" ]-> SECTOR: "+str(wind_sector(repre))+" mu/sin(alpha)[payload] -> "+str(abs(UNIT_ENERGY[2,repre]/math.sin(math.radians(nslack[index]))))+" mu/sin(alpha)[empty] ->"+str(abs(UNIT_ENERGY[0,repre]/math.sin(math.radians(nslack[index]))))+"\n"
		lower = (start + nslack[index]+1) % 360
	return res

def compute_point_Takeoff(deviation,mt_angle,alpha_t,point,segment):
	atan = 0
	mt = Line(segment[0].p1,segment[0].p2)
	if(math.isfinite(deviation)):
		atan = (mt_angle + alpha_t) % 360
		TP_line = Line(point,slope = math.tan(math.radians(atan)))
		T = TP_line.intersect(mt)
		T = list(T)[0]
		x = T.x
		y = T.y
		x = round(x,4)
		y = round(y,4)
		T = Point(x,y)
		TP = Segment(T,point)
		#print("lunghezza: ",N(TP.length),"\n")
	else:
		if(segment[2]==0):
			x = -math.inf
			y = -math.inf
		else: 
			x = math.inf
			y = math.inf
		T = Point(x,y)
		TP = Segment(T,point)

	return T, TP

def compute_point_Landing(deviation,mt_angle,alpha_l,point,segment):
	atan = 0
	mt = Line(segment[0].p1,segment[0].p2)
	if(math.isfinite(deviation)):
		atan = (mt_angle + alpha_l) % 360
		PL_line = Line(point,slope = math.tan(math.radians(atan)))
		L = PL_line.intersect(mt)
		L = list(L)[0]
		x = L.x
		y = L.y
		x = round(x,4)
		y = round(y,4)
		L = Point(x,y)
		PL = Segment(point,L)
		#print(N(L),"\n")
	else:
		if(segment[2]==0):
			x = math.inf
			y = math.inf
		else: 
			x = -math.inf
			y = -math.inf
		L = Point(x,y)
		PL = Segment(point,L)

	return L, PL
#############################################################################################
################################## NEW FUNCTIONS ############################################
def random_convex_polygon(n,rayd):
	random.seed(123)
	circle_r = rayd
	circle_x = 0
	circle_y = 0
	points = []
	for i in range(n):
		alpha = 2 * math.pi * random.random()
		#alpha = 2 * math.pi * random.uniform(0, 359)
		x = circle_r * math.cos(alpha) + circle_x
		y = circle_r * math.sin(alpha) + circle_y
		points.append([x,y])
	points = np.array(points)
	return points

def generate_random_delivery(points,n,path):
	poly = geometry.Polygon(points)
	pts = random_points_within(poly,n,path)
	
	return pts

def plot_polygon(segments, points, dp):
	plt.plot(points[:,0], points[:,1], 'o')
	for index in range(len(segments)):
		HP = segments[index][0].perpendicular_segment(dp[0])
		H = HP.p2
		plt.plot(H.x, H.y, 'o')
		plt.annotate(str(index), H)
		x_values = [segments[index][0].p1.x, segments[index][0].p2.x]
		y_values = [segments[index][0].p1.y, segments[index][0].p2.y]
		plt.plot(x_values, y_values, 'k-')
		plt.annotate(str(index), segments[index][0].p1)		
	for p in dp:
		plt.plot(p.x, p.y, 'bo')
	plt.show()
	return

def check_perpendicular(point,path):
	for segment,angle,dire in path:
		HP = segment.perpendicular_segment(point)
		H = HP.p2
		if(isOnSegment(segment.p1,segment.p2,H)):
			return True
	return False

def random_points_within(poly, num_points,path):
	min_x, min_y, max_x, max_y = poly.bounds

	points = []

	while len(points) < num_points:
		random_point = geometry.Point([random.randrange(round(min_x), round(max_x)), random.randrange(round(min_y), round(max_y))])
		
		while (not(check_perpendicular(Point(random_point.x,random_point.y),path)) or  not(random_point.within(poly))):
			random_point = geometry.Point([random.randrange(round(min_x), round(max_x)), random.randrange(round(min_y), round(max_y))])
		points.append(Point(random_point.x,random_point.y))

	return points

def clockwiseangle_and_distance(point):
	origin = [0, 0]
	refvec = [0, 1]
	# Vector between point and the origin: v = p - o
	vector = [point[0]-origin[0], point[1]-origin[1]]
	# Length of vector: ||v||
	lenvector = math.hypot(vector[0], vector[1])
	# If length is zero there is no angle
	if lenvector == 0:
		return -math.pi, 0
	# Normalize vector: v/||v||
	normalized = [vector[0]/lenvector, vector[1]/lenvector]
	dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
	diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
	angle = math.atan2(diffprod, dotprod)
	# Negative angles represent counter-clockwise angles so we need to subtract them 
	# from 2*pi (360 degrees)
	if angle < 0:
		return 2*math.pi+angle, lenvector
	# I return first the angle because that's the primary sorting criterium
	# but if two vectors have the same angle then the shorter distance should come first.
	return angle, lenvector

def compute_MT_path(vertices,clockwise=True,startpoint=0):
	start = vertices[startpoint]
	check = set(start)
	segment_path_list = []
	points = sorted(vertices, key=clockwiseangle_and_distance)
	for e in range(len(points)):
		if(set(points[e]) == check):
			lower = e 
	if clockwise:
		upper = lower + 1
		if(upper > (len(points)-1)):
			upper = 0
		while(set(points[upper]) != check):
			p1 = Point(points[lower][0],points[lower][1])
			p2 = Point(points[upper][0],points[upper][1])
			print("lato(", lower," , ",upper,")\n")
			print("lato(", N(p1)," , ",N(p2),")\n")
			segment_path_list.append(Segment(p1,p2))
			lower = lower + 1
			if(lower > (len(points)-1)):
				lower = 0
			upper = upper + 1
			if(upper > (len(points)-1)):
				upper = 0
		p1 = Point(points[lower][0],points[lower][1])
		p2 = Point(points[upper][0],points[upper][1])
		print("lato(", lower," , ",upper,")\n")
		print("lato(", N(p1)," , ",N(p2),")\n")
		segment_path_list.append(Segment(p1,p2))
	else:
		upper = lower -1
		if(upper < 0):
			upper = len(points) - 1
		while(set(points[upper]) != check):
			p1 = Point(points[lower][0],points[lower][1])
			p2 = Point(points[upper][0],points[upper][1])
			print("lato(", lower," , ",upper,")\n")
			print("lato(", N(p1)," , ",N(p2),")\n")
			segment_path_list.append(Segment(p1,p2))
			lower = lower -1 
			if(lower < 0):
				lower = len(points) - 1
			upper = upper - 1
			if(upper < 0):
				upper = len(points) - 1
		p1 = Point(points[lower][0],points[lower][1])
		p2 = Point(points[upper][0],points[upper][1])
		print("lato(", lower," , ",upper,")\n")
		print("lato(", N(p1)," , ",N(p2),")\n")
		segment_path_list.append(Segment(p1,p2))
	return segment_path_list

def segment_direction(segment):
	angle = 0
	xv, yv = segment.direction
	m = yv/ xv
	angle = math.degrees(math.atan(m)) % 360
	direction = 1
	if(xv < 0):
		angle = (angle + 180) % 360
		direction = 0
	return angle, direction

def augmentation_segments_info(segments):
	### result format [segment,angle,direction]
	augmentated_list = []
	angle_of = 0
	direction = 0
	for s in segments:
		x1, y1 = N(s.p1)
		x2, y2 = N(s.p2)
		
		angle_of, direction = segment_direction(s)
		augmentated_list.append([s,round(angle_of),direction])
		print("(",x1,",",y1,"),(",x2,",",y2,")")
		print(angle_of,"° direction: ",direction)
	return augmentated_list

def isOnSegment(A,B,C):
	AB = Segment(A,B)
	AC = Segment(A,C)
	CB = Segment(C,B)
	return N(AC.length) + N(CB.length) == N(AB.length)

def compute_takeoff_landing(segment,point,n_wind_classes,H,HP,wind_direction,dict_unit,side,p,n):
	"""
		H: point P projection on line MT
		HP: segment perpendicular to MT
		w_direction: wind direction
		DP_side: point P line side
		p: alpha angles first quadrant
		n: alpha angles fourth quadrant
		mt_angle: line (vectorial) direction
	"""
	mt_angle = segment[1]
	takeoff_points = []
	landing_points = []
	which_alhaT = 0
	which_alhaL = 0
	sector_t = 0
	sector_l = 0

	for i in range(len(p)):
		if(mt_angle in range(0,90)):
			if(side == "DX"):
				#TAKEOFF -alpha, LANDING alpha
				atan_landing = (mt_angle + p[i]) % 360
				atan_takeoff = (mt_angle - n[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				which_alhaT = -n[i]
				which_alhaL = p[i]
				
			else:
				#TAKEOFF alpha, LANDING -alpha
				atan_landing = (mt_angle - n[i]) % 360
				atan_takeoff = (mt_angle + p[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				which_alhaT = p[i]
				which_alhaL = -n[i]
				
		elif(mt_angle in range(90,180)):
			if(side == "DX"):
				#TAKEOFF alpha, LANDING -alpha
				atan_landing = (mt_angle - n[i]) % 360
				atan_takeoff = (mt_angle + p[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				which_alhaT = p[i]
				which_alhaL = -n[i]
				
			else:
				#TAKEOFF -alpha, LANDING alpha
				atan_landing = (mt_angle + p[i]) % 360
				atan_takeoff = (mt_angle - n[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				which_alhaT = -n[i]
				which_alhaL = p[i]
				
		elif(mt_angle in range(180,270)):
			if(side == "DX"):
				#TAKEOFF -alpha, LANDING alpha
				atan_landing = (mt_angle + p[i]) % 360
				atan_takeoff = (mt_angle - n[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				which_alhaT = -n[i]
				which_alhaL = p[i]
				
			else:
				#TAKEOFF alpha, LANDING -alpha
				atan_landing = (mt_angle - n[i]) % 360
				atan_takeoff = (mt_angle + p[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				which_alhaT = p[i]
				which_alhaL = -n[i]
				
		elif(mt_angle in range(270,361)):
			if(side == "DX"):
				#TAKEOFF -alpha, LANDING alpha
				atan_landing = (mt_angle + p[i]) % 360
				atan_takeoff = (mt_angle - n[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				which_alhaT = -n[i]
				which_alhaL = p[i]
				
			else:
				#TAKEOFF alpha, LANDING -alpha
				atan_landing = (mt_angle - n[i]) % 360
				atan_takeoff = (mt_angle + p[i]) % 360
				HT_segment = (N(HP.length) / math.tan(math.radians(p[i])))
				HL_segment = (N(HP.length) / math.tan(math.radians(-n[i])))
				which_alhaT = p[i]
				which_alhaL = -n[i]
				
		if(isOnSegment(segment[0].p1,segment[0].p2,H)):
			L, PL = compute_point_Landing(HL_segment,mt_angle,which_alhaL,point,segment)
			if(isOnSegment(segment[0].p1,segment[0].p2,L)):
				relative_w_l = ((wind_direction - (atan_landing))) % 360
				wc_l = wind_class(relative_w_l)
				sector_l = wind_sector(relative_w_l)
				unit_l = dict_unit[0,wc_l]
				cost_l = unit_l * N(PL.length)
				landing_points.append((L,PL,unit_l,cost_l,atan_landing,N(PL.length),wc_l,which_alhaL,sector_l,segment))
				#print("A")
			else:
				if(abs(which_alhaL)==90):
					relative_w_l = ((wind_direction - (atan_landing))) % 360
					wc_l = wind_class(relative_w_l)
					sector_l = wind_sector(relative_w_l)
					unit_l = dict_unit[0,wc_l]
					cost_l = unit_l * N(HP.length)
					landing_points.append((H,HP,unit_l,cost_l,atan_landing,N(PL.length),wc_l,which_alhaL,sector_l,segment))
					#print("AC")
			T,TP = compute_point_Takeoff(HT_segment,mt_angle,which_alhaT,point,segment)
			if(isOnSegment(segment[0].p1,segment[0].p2,T)):
				relative_w_t = ((wind_direction - (atan_takeoff))) % 360
				wc_t = wind_class(relative_w_t)
				sector_t = wind_sector(relative_w_t)
				unit_t = dict_unit[PAYLOAD,wc_t]
				cost_t = unit_t * N(TP.length)
				takeoff_points.append((T,TP,unit_t,cost_t,atan_takeoff,N(TP.length),wc_t,which_alhaT,sector_t,segment))
				#print("B")
			else:
				if(abs(which_alhaT)==90):
					relative_w_t = ((wind_direction - (atan_takeoff))) % 360
					wc_t = wind_class(relative_w_t)
					sector_t = wind_sector(relative_w_t)
					unit_t = dict_unit[PAYLOAD,wc_t]
					cost_t = unit_t * N(HP.length)
					takeoff_points.append((H,HP,unit_t,cost_t,atan_takeoff,N(TP.length),wc_t,which_alhaT,sector_t,segment))
					#print("BC")
		#print(w_direction," - ",atan_takeoff," - ",relative_w_t," - ",which_alhaT)
		#print(HL_segment,H)
		#print(T,TP)
		
	return takeoff_points , landing_points

def exhaustive_test_same_segment(path,delivery_points,n_class,wind_direction,wind_speed,payload,dict_energy):
	list_mission = []
	list_mission_90 = []
	list_missions = []
	list_missions_90 = []
	for P in delivery_points:
		list_mission = []
		list_mission_90 = []
		for segment in path:
			pslack, nslack = list_delivery_slack(n_class,segment[1],wind_direction)
			side = get_side(segment[0], P)
			HP = segment[0].perpendicular_segment(P)
			H = HP.p2
			A = segment[0].p1
			B = segment[0].p2
			if(isOnSegment(A,B,H)):
				takeoffs , landings = compute_takeoff_landing(segment,P,n_class,H,HP,wind_direction,dict_energy,side,pslack,nslack)
				
				best_takeoff = min(takeoffs, key = lambda t: t[3])
				best_landing = min(landings, key = lambda t: t[3])
				best_cost = best_takeoff[3] + best_landing[3]
				list_mission.append([best_takeoff,best_landing,best_cost])

				index_t = (len(takeoffs)-1)
				index_l = (len(landings)-1)

				shortest_takeoff = takeoffs[index_t]
				shortest_landing = landings[index_l]
				shortest_cost = shortest_takeoff[3] + shortest_landing[3]
				list_mission_90.append([shortest_takeoff,shortest_landing,shortest_cost])

		optimal_mission = min(list_mission, key = lambda t: t[2])
		optimal_mission_90 = min(list_mission_90, key = lambda t: t[2])
		list_missions.append(optimal_mission)
		list_missions_90.append(optimal_mission_90)
		#print("OPTIMAL MISSION: "+str(optimal_mission)+"\n")
		#print("OPTIMAL SHORTEST MISSION: "+str(optimal_mission_90)+"\n")
	print("Wind: wd->",wind_direction," ws->",wind_speed," ss_test done.")
	return list_missions,list_missions_90
	
def exhaustive_test_best_segment(path,delivery_points,n_class,wind_direction,wind_speed,payload,dict_energy):
	
	list_best_takeoff = []
	list_best_landing = []
	list_best_takeoff_90 = []
	list_best_landing_90 = []
	list_missions = []
	list_missions_90 = []

	for P in delivery_points:
		#I forgot to initialiaze again these lists
		list_best_takeoff = []
		list_best_landing = []
		list_best_takeoff_90 = []
		list_best_landing_90 = []
		for segment in path:
			pslack, nslack = list_delivery_slack(n_class,segment[1],wind_direction)
			side = get_side(segment[0], P)
			HP = segment[0].perpendicular_segment(P)
			H = HP.p2
			A = segment[0].p1
			B = segment[0].p2
			if(isOnSegment(A,B,H)):
				takeoffs , landings = compute_takeoff_landing(segment,P,n_class,H,HP,wind_direction,dict_energy,side,pslack,nslack)
				best_takeoff = min(takeoffs, key = lambda t: t[3])				
				best_landing = min(landings, key = lambda t: t[3])

				list_best_landing.append(best_landing)
				list_best_takeoff.append(best_takeoff)

				index_t = (len(takeoffs)-1)
				index_l = (len(landings)-1)
				list_best_landing_90.append(landings[index_l])
				list_best_takeoff_90.append(takeoffs[index_t])

		min_element_t = list_best_takeoff[0]
		min_element_l = list_best_landing[0]
		for i in range(len(list_best_takeoff)):
			for j in range(i,len(list_best_landing)):
				if(list_best_takeoff[i][3]+list_best_landing[j][3] < min_element_t[3]+min_element_l[3]):
							min_element_t = list_best_takeoff[i]
							min_element_l = list_best_landing[j]
		optimal_mission = [min_element_t , min_element_l]

		min_element_t = list_best_takeoff_90[0]
		min_element_l = list_best_landing_90[0]
		for i in range(len(list_best_takeoff_90)):
			for j in range(i,len(list_best_landing_90)):
				if(list_best_takeoff_90[i][3]+list_best_landing_90[j][3] < min_element_t[3]+min_element_l[3]):
							min_element_t = list_best_takeoff_90[i]
							min_element_l = list_best_landing_90[j]
		optimal_mission_90 = [min_element_t,min_element_l]

		list_missions.append(optimal_mission)
		list_missions_90.append(optimal_mission_90)

		#print("OPTIMAL MISSION: "+str(optimal_mission)+"\n")
		#print("OPTIMAL SHORTEST MISSION: "+str(optimal_mission_90)+"\n")
	print("Wind: wd->",wind_direction," ws->",wind_speed," ms_test done.")
	return list_missions,list_missions_90

def read_24h_wind(start):
	import xlrd
	math_wind = []
	wb = xlrd.open_workbook("wind_dataset\CAP_CORSE.xls")
	sheet = wb.sheet_by_index(0)
	for index in range(start,start+240):
		m_wind = (-(sheet.cell_value(index, 2)) + 270) % 360
		speed = sheet.cell_value(index, 3)
		hour = xlrd.xldate.xldate_as_datetime(sheet.cell_value(index,1),wb.datemode)
		#print("h",hour," speed",speed," wind",m_wind)
		math_wind.append([m_wind,speed,str(hour.strftime("%H"))])
	return math_wind

def calculate_length_slack(path):
	list_slack = []
	slack = 0
	for side in path:
		list_slack.append(slack)
		slack = slack + N(side[0].length)	
	return list_slack

def trajectory_2d_to_1d_gain(path,best_trajectories,shortest_trajectories):
	list_slack = calculate_length_slack(path)
	points_1d = []
	points_1d.append([0,0,0,0])
	for index in range(len(best_trajectories)):
		t = N(best_trajectories[index][0][0])
		l = N(best_trajectories[index][1][0])

		length_t = N(Segment(best_trajectories[index][0][9][0].p1,t).length)
		length_l = N(Segment(best_trajectories[index][1][9][0].p1,l).length)

		t_i = path.index(best_trajectories[index][0][9])
		l_i = path.index(best_trajectories[index][1][9])

		x1d = length_t + list_slack[t_i]
		y1d = length_l + list_slack[l_i]

		w = round(best_trajectories[index][0][3] + best_trajectories[index][1][3])

		p = round((shortest_trajectories[index][0][3] + shortest_trajectories[index][1][3])- w)

		
		if(x1d > y1d):
			points_1d.append([round(N(y1d)),round(N(x1d)),w,p])
			print("- ",[round(N(y1d)),round(N(x1d)),w,p])
			#print("HIT")
		else:
			points_1d.append([round(N(x1d)),round(N(y1d)),w,p])
			print("- ",[round(N(x1d)),round(N(y1d)),w,p])

	return sorted(points_1d, key=lambda x: (x[1]))

def trajectory_2d_to_1d(path,best_trajectories,dictionary,payload):
	list_slack = calculate_length_slack(path)
	points_1d = []
	points_1d.append([0,0,0,0])
	max_0, max_p = max_reward(dictionary,payload)
	for trajectory in best_trajectories:
		t = N(trajectory[0][0])
		l = N(trajectory[1][0])
		
		length_t = N(Segment(trajectory[0][9][0].p1,t).length)
		length_l = N(Segment(trajectory[1][9][0].p1,l).length)
		

		t_i = path.index(trajectory[0][9])
		l_i = path.index(trajectory[1][9])

		x1d = length_t + list_slack[t_i]
		y1d = length_l + list_slack[l_i]
		#print("A: ",N(trajectory[0][9][0].p1)," B: ",N(trajectory[0][9][0].p2)," direction: ", N(trajectory[0][9][2])," A1: ",N(trajectory[1][9][0].p1)," B1: ",N(trajectory[1][9][0].p2)," direction: ",N(trajectory[1][9][2]))
		#print("takeoff: ",t," landing: ",l,"T: ",t_i," L: ",l_i,"Length t: ", length_t, " Length l: ",length_l," x1d: ",x1d," y1d: ",y1d,"\n")
		p = abs(max_p - (trajectory[0][2]/math.sin(math.radians(trajectory[0][7])))) + abs(max_0 - (trajectory[1][2]/math.sin(math.radians(trajectory[1][7]))))

		w = round(trajectory[0][3] + trajectory[1][3])
		
		if(x1d > y1d):
			points_1d.append([round(N(y1d)),round(N(x1d)),w,p])
			#print("HIT")
		else:
			points_1d.append([round(N(x1d)),round(N(y1d)),w,p])

	return sorted(points_1d, key=lambda x: (x[1]))

def PreProcessingDynamic(interval_1d):
    ### predecessor list, Pred_p[i] contain the last delivery compatible with delivery i 
	Pred_list = []
	for i in range(len(interval_1d)):
		Pred_list.append(findLastCompatibleDelivery(interval_1d,i))
	return Pred_list

#############################WEIGHTED INTERVAL SCHEDULING##############################
def previous_intervals(I):
	p = []
	start = [task[0] for task in I]
	finish = [task[1] for task in I]

	for i in range(len(I)):
		idx = bisect.bisect(finish,start[i]) - 1
		if(i == idx):
			idx = idx - 1
		p.append(idx)
	return p

def compute_opt(j,I,OPT,p,O):
	#use recursive formula max(wj + OPT(p(j)),OPT(j-1))
	if j == -1:
		return 0
	elif (0 <= j) and (j < len(OPT)):
		return OPT[j]
	else:
		return max(I[j][3] + compute_opt(p[j],I,OPT,p,O), compute_opt(j-1,I,OPT,p,O))

def weighted_interval(I,OPT,p,O):
	for j in range(len(I)):
		opt_j = compute_opt(j,I,OPT,p,O)
		OPT.append(opt_j)
	find_solution(len(I)-1,I,OPT,p,O)
	return OPT[-1]

def find_solution(j,I,OPT,p,O):
	if j == -1:
		return
	else:
		if (I[j][3] + OPT[p[j]] >= OPT[j-1]):
			O.append(I[j])
			find_solution(p[j],I,OPT,p,O)
		else:
			find_solution(j-1,I,OPT,p,O)

def weighted_interval_scheduling_dynamic_programming(I):
	OPT = []
	O=[]
	I.sort(key = lambda tup : tup[1])
	p = previous_intervals(I)
	opt_sol = weighted_interval(I,OPT,p,O)
	#print('Maximum weight: ' + str(opt_sol))
	#print('The best items to take are: ' + str(O[::-1]))
	return O[::-1]

def bin_packing_strategies(I,budget=5000,naive=False):
	#bin-packing first-fit
	I_copy = I.copy()
	for interval in I_copy[:]:
		if interval[2] > budget:
			I_copy.remove(interval)
	sol = weighted_interval_scheduling_dynamic_programming(I_copy)
	bin_sol = []
	reward = [0]*len(sol)
	cost = [0]*len(sol)
	for i in range(len(sol)):
		bin_sol.append([])
	bins = 0
	for k in range(len(sol)):
		assigned = False
		for j in range(bins):
			if (cost[j]+sol[k][2] <= budget and assigned == False):
				assigned = True
				bin_sol[j].append(sol[k])
				reward[j] = reward[j] + sol[k][3]
				cost[j] = cost[j] + sol[k][2]
		if (assigned == False):
			bins = bins + 1
			assigned = True
			bin_sol.append([])
			reward.append(0)
			cost.append(0)
			bin_sol[bins].append(sol[k])
			reward[bins] = reward[bins] + sol[k][3]
			cost[bins] = cost[bins] + sol[k][2]
	#print("reward: ", reward)
	#print("cost: ", cost)
	#print("BINS: ",bin_sol)
	index_sol = reward.index(max(reward))
	count = 0
	for pack in bin_sol:
		if(pack!=[]):
			count= count + 1 
	#reward,cost,sol,#bin
	if(naive == False):
		return [reward[index_sol],cost[index_sol],bin_sol[index_sol],count]
	else: 
		return bin_sol

def naive_bin_packing(I,drones, budget):
	elements = []
	sol = []
	r = 0
	c = 0

	I_copy = I.copy()
	bin_sol = bin_packing_strategies(I_copy, budget, True)
	for e in bin_sol:
		if e != []:
			elements.append(e[0])
	
	elements = sorted(elements, key=lambda x: (-x[3]))
	if (len(elements) >= drones):
		for i in range(drones):
			r = r + elements[i][3]
			c = c + elements[i][2]
			sol.append(elements[i])
	else:
		for e in elements:
			r = r + e[3]
			c = c + e[3]
			sol.append(e)
	return [r,c,sol]

def generalize_bin_packing(I,drones,budget):
	solution = []
	last_elments = []
	onedrone_solution = []
	intervals_copy = I.copy()
	reward = 0
	cost = 0
	bins = 0
	for d in range(drones):
		if (last_elments != []):
			for e in last_elments:
				intervals_copy.remove(e)
		onedrone_solution = bin_packing_strategies(intervals_copy,budget)
		last_elments = onedrone_solution[2]
		solution.append(last_elments)
		reward = reward + onedrone_solution[0]
		cost = cost + onedrone_solution[1]
		bins = bins + onedrone_solution[3]
	return [reward, cost, solution, bins]
######################################################################################

def normalize_pred(Pred_list):
	### adjust the Pred index adding an element on the top 
	Pred_list.insert(0, 0)
	N = len(Pred_list)
	for i in range(N):
		if (Pred_list[i] + 1) < i or Pred_list[i] < 0:
			Pred_list[i]= Pred_list[i]+1
	return Pred_list

def knapsack_variant_unitary_reward(W,intervals,pred, unitary=True):
	"""
		M[i][w][0]: reward
		M[i][w][1]: weight
		M[i][w][2]: selected indexes
	"""
	D = len(intervals)
	reward = []
	if(unitary):
		reward = [1] * (D-1)
		reward.insert(0,0)
	else:
		for i in intervals:
			reward.append(i[3])

	M = [[[0,0,set()] for _ in range(W+1)] for _ in range(D)]
	for i in range(D):
		for w in range(W+1):
			if i == 0 or w == 0:
				M[i][w][0] = 0
				M[i][w][1] = 0
				M[i][w][2] = set()
			elif intervals[i][2] <= w:
				i_reward = reward[i]
				i_cost = intervals[i][2]
				pred_reward = M[pred[i]][w - intervals[i][2]][0]
				row_reward = M[i-1][w][0]
				pred_cost = M[pred[i]][w - intervals[i][2]][1]
				row_cost = M[i-1][w][1]
				if ((i_reward + pred_reward) >= row_reward):
					M[i][w][0] = i_reward + pred_reward
					M[i][w][1] = pred_cost + i_cost
					M[i][w][2] = (M[pred[i]][w - intervals[i][2]][2]).copy()
					M[i][w][2].add(i)
					#M[i][w][0], M[i][w][1] = reward_knapsack_recompute(M[i][w],intervals)
				else:
					M[i][w][0] = row_reward
					M[i][w][1] = row_cost
					M[i][w][2] = M[i-1][w][2]
			else:
				M[i][w] = M[i-1][w]
			#print("M[ ",i,", ",w," ]->",M[i][w])	
	return M[D-1][W]
	
def knapsack_multidrone(intervals,drones,budget=5000):
	
	solution = []
	done_index = []
	onedrone_solution = []
	intervals_copy = intervals.copy()
	reward = 0
	cost = 0
	ilp = 0
	for i in range(drones):
		intervals_copy = delete_jobs_done(done_index,intervals_copy)
		intervals_copy = sorted(intervals_copy, key=lambda x: (x[1]))
		pred_interval_1d = previous_intervals(intervals_copy)
		onedrone_solution = knapsack_variant_unitary_reward(budget,intervals_copy,pred_interval_1d,False)
		#ilp = opt_ilp_cplex(intervals_copy, 5000, 1, False)
		#print("KNAPSACK SINGLE Solution ", onedrone_solution)
		#for e in onedrone_solution[2]:
		#	app = intervals.index(intervals_copy[e])
		#	print("-",app," w: ",intervals[app][2]," r: ",intervals[app][3])
		#print("CHECK KNAPSACK: ",ilp/onedrone_solution[0])
		done_index = list(onedrone_solution[2])
		reward = reward + onedrone_solution[0]
		cost = cost + onedrone_solution[1]
		solution.append(onedrone_solution)
	return [reward,cost,solution]

def reward_knapsack_recompute(solution,intervals):
	reward = 0
	weight = 0
	for i in solution[2]:
		reward = reward + intervals[i][3]
		weight = weight + intervals[i][2]
	return reward, weight

"""
def greedy_weight_selection(intervals,drone=1,budget=5000):
	solution = []
	total_reward = 0
	if(drone >= len(intervals)):
		print("TRIVIAL SOLUTION")
		return solution
	for i in range(drone):
		solution.append([])
	batteries = [budget] * drone
	intervals = sorted(intervals, key=lambda x: (x[2]))
	#print(intervals)
	while(i < len(intervals)):
		for b in range(len(batteries)):
			#print(solution[b]," index: ",b," len: ",len(solution[b]))
			if(len(solution[b]) == 0):
				if(batteries[b] >= intervals[i][2]):
					solution[b].append(intervals[i])
					total_reward = total_reward + intervals[i][3]
					batteries[b] = batteries[b] - intervals[i][2]
					i = i + 1
			else:
				last_i = solution[b][len(solution[b])-1]
				if(batteries[b] >= intervals[i][2] and last_i[1] <= intervals[i][0]):
					solution[b].append(intervals[i])
					total_reward = total_reward + intervals[i][3]
					batteries[b] = batteries[b] - intervals[i][2]
					i = i + 1
		i = i + 1
				
	return [solution, total_reward]

def greedy_reward_selection(intervals,drone=1,budget=5000):
	solution = []
	total_reward = 0
	if(drone >= len(intervals)):
		print("TRIVIAL SOLUTION")
		return solution
	for i in range(drone):
		solution.append([])
	batteries = [budget] * drone
	intervals = sorted(intervals, key=lambda x: (-x[3]))
	#print(intervals)
	while(i < len(intervals)):
		for b in range(len(batteries)):
			#print(solution[b]," index: ",b," len: ",len(solution[b]))
			if(len(solution[b]) == 0):
				if(batteries[b] >= intervals[i][2]):
					solution[b].append(intervals[i])
					total_reward = total_reward + intervals[i][3]
					batteries[b] = batteries[b] - intervals[i][2]
					i = i + 1
			else:
				last_i = solution[b][len(solution[b])-1]
				if(batteries[b] >= intervals[i][2] and last_i[1] <= intervals[i][0]):
					solution[b].append(intervals[i])
					total_reward = total_reward + intervals[i][3]
					batteries[b] = batteries[b] - intervals[i][2]
					i = i + 1
		i = i + 1
				
	return [solution, total_reward]

def greedy_ending_selection(intervals,drone=1,budget=5000):
	solution = []
	total_reward = 0
	if(drone >= len(intervals)):
		print("TRIVIAL SOLUTION")
		return solution
	for i in range(drone):
		solution.append([])
	batteries = [budget] * drone
	intervals = sorted(intervals, key=lambda x: (x[1]))
	#print(intervals)
	while(i < len(intervals)):
		for b in range(len(batteries)):
			#print(solution[b]," index: ",b," len: ",len(solution[b]))
			if(len(solution[b]) == 0):
				if(batteries[b] >= intervals[i][2]):
					solution[b].append(intervals[i])
					total_reward = total_reward + intervals[i][3]
					batteries[b] = batteries[b] - intervals[i][2]
					i = i + 1
			else:
				last_i = solution[b][len(solution[b])-1]
				if(batteries[b] >= intervals[i][2] and last_i[1] <= intervals[i][0]):
					solution[b].append(intervals[i])
					total_reward = total_reward + intervals[i][3]
					batteries[b] = batteries[b] - intervals[i][2]
					i = i + 1
		i = i + 1
	 			
	return [solution, total_reward]
"""
def greedy_ending_selection(intervals,drone=1,budget=5000):
	total_reward = 0
	cost = 0
	intervals_copy = intervals.copy()
	solution = []
	batteries = []
	for i in range(drone):
		batteries.append(budget)
		solution.append([])
	intervals_copy = sorted(intervals_copy, key=lambda x: (x[1]))
	while len(intervals_copy) != 0 and sum(batteries) != 0:
		b = batteries.index(max(batteries))
		task = intervals_copy.pop(0)
		if(len(solution[b]) == 0):
			if(task[2]<=batteries[b]):
				solution[b].append(task)
				total_reward = total_reward + task[3]
				batteries[b] = batteries[b] - task[2]
				cost = cost + task[2]
		else:
			if task[2]<=batteries[b] and check_correct_interval(solution[b], task):
				solution[b].append(task)
				total_reward = total_reward + task[3]
				batteries[b] = batteries[b] - task[2]
				cost = cost + task[2]
	return [solution,total_reward,cost]

def greedy_reward_selection(intervals,drone=1,budget=5000):
	total_reward = 0
	cost = 0
	intervals_copy = intervals.copy()
	solution = []
	batteries = []
	for i in range(drone):
		batteries.append(budget)
		solution.append([])
	intervals_copy = sorted(intervals_copy, key=lambda x: (-x[3]))
	while len(intervals_copy) != 0 and sum(batteries) != 0:
		b = batteries.index(max(batteries))
		task = intervals_copy.pop(0)
		if(len(solution[b]) == 0):
			if(task[2]<=batteries[b]):
				solution[b].append(task)
				total_reward = total_reward + task[3]
				batteries[b] = batteries[b] - task[2]
				cost = cost + task[2]
		else:
			if task[2]<=batteries[b] and check_correct_interval(solution[b], task):
				solution[b].append(task)
				total_reward = total_reward + task[3]
				batteries[b] = batteries[b] - task[2]
				cost = cost + task[2]
	return [solution,total_reward,cost]

def greedy_weight_selection(intervals,drone=1,budget=5000):
	total_reward = 0
	intervals_copy = intervals.copy()
	solution = []
	batteries = []
	cost = 0
	for i in range(drone):
		batteries.append(budget)
		solution.append([])
	intervals_copy = sorted(intervals_copy, key=lambda x: (x[2]))
	while len(intervals_copy) != 0 and sum(batteries) != 0:
		b = batteries.index(max(batteries))
		task = intervals_copy.pop(0)
		if(len(solution[b]) == 0):
			if(task[2]<=batteries[b]):
				solution[b].append(task)
				total_reward = total_reward + task[3]
				batteries[b] = batteries[b] - task[2]
				cost = cost + task[2]
		else:
			if task[2]<=batteries[b] and check_correct_interval(solution[b], task):
				solution[b].append(task)
				total_reward = total_reward + task[3]
				batteries[b] = batteries[b] - task[2]
				cost = cost + task[2]
	return [solution,total_reward,cost]

def getOverlap(a,b):
	i1 = interval.interval[a[0], a[1]]
	i2 = interval.interval[b[0], b[1]]
	return len(i1 & i2)

def check_correct_interval(list_of_intervals, interval):
	list_of_intervals = sorted(list_of_intervals, key=lambda x: (x[1]))
	for i in list_of_intervals:
		if(getOverlap(i,interval)!=0):
			return False
	return True

def max_reward(dictionary, payload):
	max_reward_0 = []
	max_reward_payload = []
	for p,angle in UNIT_ENERGY:
		max_reward_0.append(dictionary[0,angle]/math.sin(math.radians(1)))
		max_reward_payload.append(dictionary[payload,angle]/math.sin(math.radians(1)))
	return max(max_reward_0) , max(max_reward_payload)

def greedy_submodular(intervals,budget = 5000):
	ratio = []
	solution = []
	sol_int = []
	cost = 0
	reward = 0
	for i in range(1,len(intervals)):
		#print("cost: ",intervals[i][2]," reward: ", intervals[i][3] ," ratio: ",intervals[i][3]/intervals[i][2])
		ratio.append([intervals[i],intervals[i][3]/intervals[i][2]])
	
	best = 0
	index = 0
	while cost < budget and len(ratio) != 0:
		best = max(ratio, key = lambda t: t[1])
		if(len(solution)==0):
			if(best[0][2] <= budget):
				cost = cost + best[0][2]
				reward = reward + best[0][3]
				sol_int.append(intervals.index(best[0]))
				solution.append(best[0])
		else:
			if(check_correct_interval(solution,best[0]) and cost + best[0][2] <= budget):
				solution.append(best[0])
				sol_int.append(intervals.index(best[0]))
				cost = cost + best[0][2]
				reward = reward + best[0][3]
		ratio.remove(best)
	return [solution, cost, reward, sol_int]

def greedy_intersection_submodular(intervals,budget = 5000):
	ratio = []
	solution = []
	sol_int = []
	cost = 0
	reward = 0
	for i in range(1,len(intervals)):
		#print("cost: ",intervals[i][2]," reward: ", intervals[i][3] ," ratio: ",intervals[i][3]/intervals[i][2])
		ratio.append([intervals[i],(intervals[i][3]/intervals[i][2])/( how_many_intersection(intervals[i], intervals) + 1 )])
	
	best = 0
	index = 0
	while cost < budget and len(ratio) != 0:
		best = max(ratio, key = lambda t: t[1])
		if(len(solution)==0):
			if(best[0][2] <= budget):
				cost = cost + best[0][2]
				reward = reward + best[0][3]
				sol_int.append(intervals.index(best[0]))
				solution.append(best[0])
		else:
			if(check_correct_interval(solution,best[0]) and cost + best[0][2] <= budget):
				solution.append(best[0])
				sol_int.append(intervals.index(best[0]))
				cost = cost + best[0][2]
				reward = reward + best[0][3]
		ratio.remove(best)
	return [solution, cost, reward, sol_int]

def delete_jobs_done(index,intervals):
	if(index==[]):
		return intervals
	indexs = sorted(index, key=lambda x: (-x))
	print("INDEX: ",index," INDEXS: ",indexs)
	for i in indexs:
		intervals.pop(i)
	return intervals

"""
def APX_coloring(intervals,budget=5000):
	intervals_copy = intervals.copy()
	intervals_copy[0].append(0)
	interval_augmented = intervals.copy()
	for i in range(1,len(intervals_copy)):
		print("- ",intervals_copy[i])
		intervals_copy[i].append(intervals_copy[i][3]/intervals_copy[i][2])
	intervals_copy = sorted(intervals_copy, key=lambda x: (x[1]))
	color = []
	colors = []
	####COLORING
	while len(intervals_copy) != 0:
		color = []
		task = intervals_copy.pop(0)
		color.append(task)
		#print("PRE-COLOR:",color)
		for interval in intervals_copy:
			if(check_correct_interval(color, interval)):
				color.append(interval)
				#print("Selected: ",interval)
				intervals_copy.remove(interval)
		colors.append(color)
	for e in colors:
		print("-C ",e)
	#print("C ",colors)
	tot=0
	for i in colors:
		tot=tot+len(i)
	print("ELEM:",tot)
	####SET SELECTION
	set_S = []
	for color in colors:
		color = sorted(color, key=lambda x: (-x[4]))
		cost = 0
		reward = 0
		indexes = []
		selected_elements = []
		for task in color:
			if cost + task[2] <= budget:
				cost = cost + task[2]
				reward = reward + task[3]
				#indexes.append(color.index(task))
				selected_elements.append(task)
				color.remove(task)
		set_S.append([reward,cost,selected_elements])
	####AUGMENT SOLUTION
	#tot = 0
	for e in set_S:
		tot = tot + len(e[2])
		print("- ",e," LEN: ",len(e[2]))
	#print("ELEM1: ",tot)
	solution = max(set_S, key=lambda x: x[0])
	na_solution = solution.copy()
	print("MAX",solution)
	for e in solution[2]:
		interval_augmented.remove(e)
	print("SELECTABLE INTERVAL: ",interval_augmented)
	interval_augmented = sorted(interval_augmented, key=lambda x: (-x[4]))
	for e in interval_augmented:
		if solution[1] + e[2] <= budget:
			if check_correct_interval(solution[2],e):
				solution[0] = solution[0] + e[3]
				solution[1] = solution[1] + e[2]
				solution[2].append(e)
		
	print("SOL",solution)
	print("AUGMENTED: ",solution[0]," NON-AUGMENTED: ",na_solution[0])
	exit()
	return solution
"""
def APX_coloring_s(intervals, budget=5000):
	
	intervals_copy = intervals.copy()
	interval_augmented = intervals.copy()

	intervals_copy.pop(0)
	interval_augmented.pop(0)

	intervals_copy = sorted(intervals_copy, key=lambda x: (x[1]))
	## [inizio,fine,costo,reward,ratio] ##
	for i in range(len(intervals_copy)):
		#print("- ",intervals_copy[i])
		intervals_copy[i].append(intervals_copy[i][3]/intervals_copy[i][2])
	
	color = []
	colors = []
	### COLORING
	while len(intervals_copy) != 0:
		color = []
		task = intervals_copy.pop(0)
		color.append(task)
		for element in intervals_copy[:]:
			#print("CONFRONTO: ",task," - ",element)
			if getOverlap(task,element)==0:
				color.append(element)
				intervals_copy.remove(element)
				task = element
		colors.append(color)
	#for c in range(len(colors)):
		#print("\nCOLORE ",c,"\n")
		#for e in colors[c]:
			#print(e)
	###S_i CALCULATION		
	set_S = []
	for c in colors:
		c = sorted(c, key=lambda x: (-x[4]))
		cost = 0
		reward = 0
		selected_elements = []
		for task in c[:]:
			if cost + task[2] <= budget:
				cost = cost + task[2]
				reward = reward + task[3]
				selected_elements.append(task)
				#print("RIMOSSO ",task)
				c.remove(task)
		set_S.append([reward,cost,selected_elements])
	#for c in range(len(set_S)):
		#print("\nSET S_",c,"\n")
		#for e in set_S[c]:
			#print(e)
	###AUGMENTATION
	solution = max(set_S, key=lambda x: x[0])
	na_solution = solution.copy()
	for e in solution[2]:
		#print("ALREADY SELECTED: ",e)
		interval_augmented.remove(e)
	#for i in interval_augmented:
		#print("SELECTABLE INTERVAL: ",i)
	interval_augmented = sorted(interval_augmented, key=lambda x: (-x[4]))
	for e in interval_augmented:
		if solution[1] + e[2] <= budget:
			if check_correct_interval(solution[2],e):
				solution[0] = solution[0] + e[3]
				solution[1] = solution[1] + e[2]
				solution[2].append(e)
				#print("Augmented: ",e)
		
	#print("SOL",solution)
	print("AUGMENTED: ",solution[0]," NON-AUGMENTED: ",na_solution[0])
	
	return solution, na_solution

def coloring_interval(intervals):
	intervals_local = intervals.copy()
	intervals_local = sorted(intervals_local, key=lambda x: (x[1]))
	## [inizio,fine,costo,reward,ratio] ##
	for i in range(len(intervals_local)):
		#print("- ",intervals[i])
		intervals_local[i].append(intervals_local[i][3]/intervals_local[i][2])
	
	color = []
	colors = []
	### COLORING
	while len(intervals_local) != 0:
		color = []
		task = intervals_local.pop(0)
		color.append(task)
		for element in intervals_local[:]:
			#print("CONFRONTO: ",task," - ",element)
			if getOverlap(task,element)==0:
				color.append(element)
				intervals_local.remove(element)
				task = element
		colors.append(color)

	return colors

def compute_s_i(colors,budget):
	set_S = []
	for c in colors:
		c = sorted(c, key=lambda x: (-x[4]))
		cost = 0
		reward = 0
		selected_elements = []
		for task in c[:]:
			if cost + task[2] <= budget:
				cost = cost + task[2]
				reward = reward + task[3]
				selected_elements.append(task)
				#print("FA PARTE DI S_I ",task)
				c.remove(task)
		set_S.append([reward,cost,selected_elements])

	return set_S, colors

def augmentation_multiple_drone(solution,intervals,budget):
	list_interval = intervals.copy()
	non_augmented = solution.copy()

	for es in solution[2]:
		for e in es[2]: 
			list_interval.remove(e)
	list_interval = sorted(list_interval, key=lambda x: (-x[4]))
	drone = len(solution[2])
	index = 0
	if len(list_interval) != 0:
		while len(list_interval) != 0 and index != drone:
			for e in list_interval[:]:
				if solution[2][index][1] + e[2] <= budget:
					if check_correct_interval(solution[2][index][2],e):
						#print("AGGIUNGO A ",index," COSTA: ",solution[2][index][1], " E ", e)
						solution[2][index][0] = solution[2][index][0] + e[3]
						solution[2][index][1] = solution[2][index][1] + e[2]
						solution[0] = solution[0] + e[3]
						solution[1] = solution[1] + e[2]
						solution[2][index][2].append(e)
						list_interval.remove(e)
			index = index + 1

	return solution, non_augmented

def APX_coloring_M(intervals, drone, budget=50000):

	intervals_copy = intervals.copy()
	interval_augmented = intervals.copy()
	if drone == 1:
		###SINGLE DRONE
		return APX_coloring_s(intervals_copy,budget)
	else:
		###MULTI DRONE
		interval_augmented.pop(0)
		intervals_copy.pop(0)
		colors = coloring_interval(intervals_copy)
		#for c in range(len(colors)):
			#print("\nCOLORE ",c,"\n")
			#for e in colors[c]:
				#print(e)
		set_S, colors = compute_s_i(colors,budget)
		#for c in range(len(set_S)):
			#print("\nSET S_",c,"\n")
			#for e in set_S[c]:
				#print(e)
		singles = []
		if len(colors) >= drone:
			### DRONE <= CHI
			print("DRONE <= CHI")
			reward = 0
			weight = 0
			for i in range(drone):
				single_solution = max(set_S, key=lambda x: x[0])
				set_S.remove(single_solution)
				singles.append(single_solution)
				reward = reward + single_solution[0]
				weight = weight + single_solution[1]
			solution = [reward,weight,singles]
			solution, non_augmented = augmentation_multiple_drone(solution,interval_augmented,budget)
			return solution, non_augmented
		else:
			### DRONE > CHI
			singles = []
			reward = 0
			weight = 0
			print("DRONE > CHI")
			for e in set_S:
				reward = reward + e[0]
				weight = weight + e[1]
				singles.append(e)
				drone = drone - 1
				for interval in e[2]:
					interval_augmented.remove(interval)
			solution = [reward,weight,singles]
			#print("DRONI RESIDUI ",drone)
			while len(interval_augmented) != 0 and drone != 0:
				colors = coloring_interval(interval_augmented)
				for c in range(len(colors)):
					#print("\nCOLORE ",c,"\n")
					for e in colors[c]:
						print(e)
				set_S, colors = compute_s_i(colors,budget)
				for c in range(len(set_S)):
					#print("\nSET S_",c,"\n")
					for e in set_S[c]:
						print(e)
				#print(len(set_S))
				for i in range(len(set_S)):
					if drone > 0:
						single_solution = max(set_S, key=lambda x: x[0])
						set_S.remove(single_solution)
						for e in single_solution[2]:
							interval_augmented.remove(e)
						solution[2].append(single_solution)
						solution[0] = solution[0] + single_solution[0]
						solution[1] = solution[1] + single_solution[1]
						drone = drone - 1
			intervals_copy = intervals.copy()
			intervals_copy.pop(0)
			solution, non_augmented = augmentation_multiple_drone(solution,intervals_copy,budget)
			return solution, non_augmented

def greedy_submodular_multidrone(intervals, drones, budget=5000):
	solution = []
	onedrone_solution = []
	done_index = []
	intervals_copy = intervals.copy()
	reward = 0
	cost = 0
	for i in range(drones):
		intervals_copy = delete_jobs_done(done_index,intervals_copy)
		print("LEN intervals: ",len(intervals_copy)," LEN Delete: ",len(done_index))
		onedrone_solution = greedy_submodular(intervals_copy,budget)
		done_index = onedrone_solution[3]
		reward = reward + onedrone_solution[2]
		cost = cost + onedrone_solution[1]
		solution.append(onedrone_solution)

	return [reward,cost,solution]

def how_many_intersection(interval, intervals):
	count = 0
	for i in intervals:
		if(getOverlap(interval,i) != 0):
			count = count + 1
	return count

def generate_random_intervals():
	
	np.random.seed(123)
	intervals = []
	single_intervals = []

	#thetas = [0,0.8,1.8,2.7]
	n_points = [25,50,75,100]
	thetas = [0, 0.4, 0.8, 1.0]
	spans = [1500,10000,20000,30000]
	weights = [2500,5000,7500,30000]
	rewards = 100
	test_per_n_of_point = 30
	max_departure = 30000

	departure = 0
	arrival = 0
	span = 0
	weight = 0

	x = np.arange(1, rewards + 1)

	for k in range(test_per_n_of_point):
		for n in n_points:
			for theta in thetas:
				prob = x ** (-theta)
				prob = prob / prob.sum()
				bounded_zipf = stats.rv_discrete(name='bounded_zipf', values=(x, prob))
				s = bounded_zipf.rvs(size=n)
				for i in range(len(spans)):
					single_intervals = []
					for j in range(n):
						span = np.random.randint(1,spans[i])
						departure = np.random.randint(0,max_departure-span)
						arrival = departure + span
						weight = np.random.randint(1,weights[i])
						single_intervals.append([departure,arrival,weight,s[j]])
					single_intervals.insert(0,[0,0,0,0])
					intervals.append(single_intervals)

	return intervals

def test_avg_ratio_fixed_poly_fixed_points(path,dps,wind_step):
	
	direction_wind = 0
	knapsack_sol = 0
	knapsack_sol_r = 0
	submodular_sol = 0
	ILP_sol = 0
	weight_sol = 0
	reward_sol = 0
	end_sol = 0
	intersection = 0
	avg_inter = 0
	avg_ddsp = 0
	avg_geft = 0
	avg_gw = 0
	avg_gp = 0
	iteratore = 0

	ratio_wind_knapsack = []
	ratio_wind_APX = []
	ratio_wind_GEFT = []
	ratio_wind_GW = []
	ratio_wind_GP = []

	reward_knapsack = []
	reward_APX = []
	reward_GEFT = []
	reward_GW = []
	reward_GP = []

	avg_ratio_wind_knapsack = []
	avg_ratio_wind_APX_intersect = []
	avg_ratio_wind_APX = []
	avg_ratio_wind_GEFT = []
	avg_ratio_wind_GW = []
	avg_ratio_wind_GP = []
	avg_intersection = []

	wind_directions = []
	interaval_intersection = []
	for dp in dps:
		ratio_wind_knapsack = []
		ratio_wind_APX = []
		ratio_wind_APX_intersect = []
		ratio_wind_GEFT = []
		ratio_wind_GW = []
		ratio_wind_GP = []
		wind_directions = []
		reward_knapsack = []
		reward_APX = []
		reward_GEFT = []
		reward_GW = []
		reward_GP = []
		interaval_intersection = []
		direction_wind = 0
		print("Test one drone n: ",len(dp))
		f = open("onedrone_verbose_result_n"+str(len(dp))+".txt","w")
		while(direction_wind <= 359):
			print("computing for wind direction ",direction_wind,"...")
			f.write("\nWIND DIRECTION: "+str(direction_wind)+"\n")
			WIND_SECTOR_REP, WIND_SECTOR, RELATIVE_WIND_VALUES = compute_wind_classes(n_wind_classes,False)
			UNIT_ENERGY = compute_prefixes(PAYLOAD,DRONE_SPEED,WIND_SPEED,RELATIVE_WIND_VALUES)
			optimal = []
			optimal_90 = [] 
			optimal, optimal_90 = exhaustive_test_best_segment(path,dp,n_wind_classes,direction_wind,WIND_SPEED,PAYLOAD,UNIT_ENERGY)
			interval_1d = trajectory_2d_to_1d(MT_PATH,optimal,UNIT_ENERGY,PAYLOAD)
			intersection = 0
			for inter in interval_1d:
				intersection = intersection + how_many_intersection(inter, interval_1d)
			avg_inter = avg_inter + intersection
			f.write("AVG INTERSECTION: "+str(intersection/len(interval_1d))+"\n")
			print("INTERSECTION AVG: ",intersection/len(interval_1d),"\n")
			pred_interval_1d = previous_intervals(interval_1d)
			f.write("INTERVAL:\n")
			for i in range(len(interval_1d)):
				print(i,": [",interval_1d[i][0],",",interval_1d[i][1],"] -> ",pred_interval_1d[i]," w: ",interval_1d[i][2]," r: ", interval_1d[i][3])
				f.write(str(i)+":["+str(interval_1d[i][0])+","+str(interval_1d[i][1])+"] -> delta(i): "+str(pred_interval_1d[i])+" w: "+str(interval_1d[i][2])+" r: "+str(interval_1d[i][3]))
			ILP_sol = opt_ilp(interval_1d, 5000, 1, False)
			knapsack_sol = knapsack_variant_unitary_reward(5000,interval_1d,pred_interval_1d,False)
			print("CHECK: ",ILP_sol/knapsack_sol[0])
			weight_sol = greedy_weight_selection(interval_1d,1)
			reward_sol = greedy_reward_selection(interval_1d,1)
			end_sol = greedy_ending_selection(interval_1d,1)
			submodular_sol = greedy_submodular(interval_1d)
			submodular_intersect = greedy_intersection_submodular(interval_1d)
			print("APX-S solution: ",submodular_sol[3]," reward: ",submodular_sol[2])
			print("KNAPSCAK solution: ",knapsack_sol)
			print("APX-intersection solution: ",submodular_intersect[3]," reward: ",submodular_intersect[2])
			print("RATIO: APX-S-> ",submodular_sol[2]/knapsack_sol[0], " GEFT-S-> ",end_sol[1]/knapsack_sol[0]," GW-S-> ",weight_sol[1]/knapsack_sol[0]," GP-S-> ",reward_sol[1]/knapsack_sol[0]," APX-i-> ",submodular_intersect[2]/knapsack_sol[0])
			######CSV LIST FORMATING
			ratio_wind_APX.append(submodular_sol[2]/knapsack_sol[0])
			ratio_wind_APX_intersect.append(submodular_intersect[2]/knapsack_sol[0])
			ratio_wind_GEFT.append(end_sol[1]/knapsack_sol[0])
			ratio_wind_GW.append(weight_sol[1]/knapsack_sol[0])
			ratio_wind_GP.append(reward_sol[1]/knapsack_sol[0])
			wind_directions.append(direction_wind)
			interaval_intersection.append(intersection/len(interval_1d))
			reward_knapsack.append(knapsack_sol[0])
			reward_APX.append(submodular_sol[2])
			reward_GEFT.append(end_sol[1])
			reward_GW.append(weight_sol[1])
			reward_GP.append(reward_sol[1])
			######
			direction_wind = direction_wind + wind_step

		rs = pd.DataFrame()
		#######CSV SAVING
		rs["APX_ratio"] = ratio_wind_APX
		rs["APX_inter_ratio"] = ratio_wind_APX_intersect
		rs["GEFT_ratio"] = ratio_wind_GEFT
		rs["GW_ratio"] = ratio_wind_GW
		rs["GP_ratio"] = ratio_wind_GP
		rs["KNAPSACK_reward"] = reward_knapsack
		rs["APX_reward"] = reward_APX
		rs["GEFT_reward"] = reward_GEFT
		rs["GW_reward"] = reward_GW
		rs["GP_reward"] = reward_GP
		rs["wind_direction"] = wind_directions
		rs["intersection"] = interaval_intersection
		###########
		rs.to_csv("onedrone_wind"+str(direction_wind)+"_result_n"+str(len(dp))+".csv")
		f.write("AVG INTERSECTION: "+str(sum(interaval_intersection) / len(interaval_intersection))+" AVG APX: "+str(sum(ratio_wind_APX) / len(ratio_wind_APX))+" AVG GEFT: "+str(sum(ratio_wind_GEFT) / len(ratio_wind_GEFT))+" AVG GP: "+str(sum(ratio_wind_GP) / len(ratio_wind_GP))+" AVG GW: "+str(sum(ratio_wind_GW) / len(ratio_wind_GW))+" AVG APX-i: "+str(sum(ratio_wind_APX_intersect) / len(ratio_wind_APX_intersect)))
		print("AVG INTERSECTION: ",sum(interaval_intersection) / len(interaval_intersection)," AVG APX: ",sum(ratio_wind_APX) / len(ratio_wind_APX)," AVG GEFT: ",sum(ratio_wind_GEFT) / len(ratio_wind_GEFT)," avg_gp: ",sum(ratio_wind_GP) / len(ratio_wind_GP)," avg_gw: ",sum(ratio_wind_GW) / len(ratio_wind_GW)," AVG APX-i: ",sum(ratio_wind_APX_intersect) / len(ratio_wind_APX_intersect))
		
		avg_intersection.append(sum(interaval_intersection) / len(interaval_intersection))
		avg_ratio_wind_APX.append(sum(ratio_wind_APX) / len(ratio_wind_APX))
		avg_ratio_wind_GEFT.append(sum(ratio_wind_GEFT) / len(ratio_wind_GEFT))
		avg_ratio_wind_GW.append(sum(ratio_wind_GW) / len(ratio_wind_GW))
		avg_ratio_wind_GP.append(sum(ratio_wind_GP) / len(ratio_wind_GP))
		avg_ratio_wind_APX_intersect.append(sum(ratio_wind_APX_intersect) / len(ratio_wind_APX_intersect))
	rs1 = pd.DataFrame()
	rs1["APX"] = avg_ratio_wind_APX
	rs1["GEFT"] = avg_ratio_wind_GEFT
	rs1["GW"] = avg_ratio_wind_GW
	rs1["GP"] = avg_ratio_wind_GP
	rs1["APX-inter"] = avg_ratio_wind_APX_intersect
	rs1["INTERSECTION"] = avg_intersection
	rs1.to_csv("onedrone_solution_result_n"+str(len(dp))+".csv")
	return

def print_test_results_avg_corse_wind(path,delivery_points,n_class,payload,drone_speed, start_i):

	shortest_mission = []
	optimal_mission = []
	paths = []

	drones = [5]

	start = start_i
	real_wind = read_24h_wind(start)

	df = pd.DataFrame()
	"""
	f = open("result/corsepath_points"+str(len(delivery_points))+".txt","w")
	contatore = 0
	print("Trajectories computing...")
	for wind_direction, wind_speed, hour in real_wind:
		wind_to_rep, wind_to_sect, relative_values = compute_wind_classes(n_class,False)
		dict_energy = compute_prefixes(payload,drone_speed,wind_speed,RELATIVE_WIND_VALUES)
		f.write("INTERVALS:\n")
		optimal_mission, shortest_mission = exhaustive_test_best_segment(path,delivery_points,n_class,wind_direction,wind_speed,payload,dict_energy)
		optimal = trajectory_2d_to_1d(path,optimal_mission,dict_energy,payload)
		contatore = contatore + 1
		print("ESECUZIONE ",contatore)
		paths.append(optimal)
	f.write(str(paths))
	f.close()

	"""
	print("Trajectories computed.")
	
	print("Start test on real paths...")
	for d in drones:
		opt = 0
		algo_reward = 0
		ratio_knapsack = 0
		ratio_submodular = 0
		ratio_gp = 0
		ratio_gw = 0
		ratio_geft = 0
		knapsack = []
		submodular = []
		gp = []
		gw = []
		geft = []
		apxS = []
		apxS_na = []
		thetas = []
		r_spans = []
		avg_intersections = []
		n_points = []
		intersection = 0
		avg_intersection = 0
		print("Test with drones: ",d)

		for path in paths:
			n_points.append(len(path))
			intersection = 0
			avg_intersection = 0

			for interval in path:
				intersection = intersection + how_many_intersection(interval, path)

			avg_intersection = intersection / len(path)

			print("Intersection: ",avg_intersection)
			opt = opt_ilp_cplex(path, 5000, d, False)
			
			algo_reward = knapsack_multidrone(path,d,5000)
			ratio_knapsack = algo_reward[0]/opt
			print("knapsack: ",ratio_knapsack)

			algo_reward = greedy_submodular_multidrone(path,d,5000)
			ratio_submodular = algo_reward[0]/opt
			print("MR: ",ratio_submodular)

			algo_reward = greedy_weight_selection(path,d,5000)
			ratio_gw = algo_reward[1]/opt
			print("GSw: ",ratio_gw)

			algo_reward = greedy_ending_selection(path,d,5000)
			ratio_geft = algo_reward[1]/opt
			print("GeRt: ",ratio_geft)

			algo_reward = greedy_reward_selection(path,d,5000)
			ratio_gp = algo_reward[1]/opt
			print("GLp: ",ratio_gp)
			
			apx_s, apx_s_na = APX_coloring_M(path,d,5000)
			ratio_apx_s = apx_s[0]/opt
			ratio_apx_s_na = apx_s_na[0]/opt
			apxS.append(round(ratio_apx_s,4))
			apxS_na.append(round(ratio_apx_s_na,4))
			print("APX: ",ratio_apx_s," COST: ",apx_s[1]," REWARD: ",apx_s[0])
			print("APX NON-AUGMENTED: ",ratio_apx_s_na," COST: ",apx_s_na[1]," REWARD: ",apx_s_na[0])

			knapsack.append(round(ratio_knapsack,4))
			submodular.append(round(ratio_submodular,4))
			gw.append(round(ratio_gw,4))
			geft.append(round(ratio_geft,4))
			gp.append(round(ratio_gp,4))
			avg_intersections.append(round(avg_intersection,4))
		print("Csv creation...")
		results = pd.DataFrame()
		
		results["apx"] = apxS
		results["apx-na"] = apxS_na
		print("AVG RATIO apx: ",sum(apxS) / len(apxS))
		print("AVG RATIO apx NON-AUGMENTED: ",sum(apxS_na) / len(apxS_na))
		results["knapsack"] = knapsack
		print("AVG RATIO knapsack: ",sum(knapsack) / len(knapsack))
		results["MR"] = submodular
		print("AVG RATIO MR: ",sum(submodular) / len(submodular))
		results["GeRt"] = geft
		print("AVG RATIO geft: ",sum(geft) / len(geft))
		results["GLp"] = gp
		print("AVG RATIO gp: ",sum(gp) / len(gp))
		results["GSw"] = gw
		print("AVG RATIO gw: ",sum(gw) / len(gw))
		results["intersection"] = avg_intersections
		results["n_points"] = [elem - 1 for elem in n_points]
		results.to_csv("corse_d"+str(d)+"_p_"+str(len(delivery_points))+".csv")
		print("Csv created.")
		
	
	return

def print_test_results_avg_corse_wind_formatted_intervals(istanze, path,delivery_points,n_class,payload,drone_speed, start_i):
	energy_budget = 5000
	shortest_mission = []
	optimal_mission = []
	paths = []

	drones = [3]

	start = start_i
	real_wind = read_24h_wind(start)

	df = pd.DataFrame()

	print("Start test on real paths...")
	for d in drones:
		opt = 0
		algo_reward = 0
		ratio_knapsack = 0
		ratio_submodular = 0
		ratio_gp = 0
		ratio_gw = 0
		ratio_geft = 0
		ratio_bin = 0
		ratio_bin_naive = 0

		reward_knapsack = []
		reward_submodular = []
		reward_gp = []
		reward_gw = []
		reward_GEFT = []
		reward_apx = []
		reward_apx_na = []
		reward_bin = []
		reward_bin_naive = []

		n_bins=[]

		cost_knapsack = []
		cost_submodular = []
		cost_gp = []
		cost_gw = []
		cost_GEFT = []
		cost_apx = []
		cost_apx_na = []
		cost_bin = []
		cost_bin_naive = []

		knapsack = []
		submodular = []
		gp = []
		gw = []
		geft = []
		apxS = []
		apxS_na = []
		bin_pack = []
		bin_pack_naive = []
		thetas = []
		r_spans = []
		avg_intersections = []
		n_points = []
		intersection = 0
		avg_intersection = 0
		print("Test with drones: ",d)

		for path in istanze:
			n_points.append(len(path))
			intersection = 0
			avg_intersection = 0

			for interval in path:
				intersection = intersection + how_many_intersection(interval, path)

			avg_intersection = intersection / len(path)

			print("Intersection: ",avg_intersection)
			opt = opt_ilp_cplex(path, energy_budget, d, False)
			
			algo_reward = knapsack_multidrone(path,d,energy_budget)
			reward_knapsack.append(algo_reward[0])
			cost_knapsack.append(algo_reward[1])
			if(algo_reward[0] != 0):
				ratio_knapsack = algo_reward[0]/opt
			else:
				ratio_knapsack = 0
			print("knapsack: ",ratio_knapsack)

			algo_reward = greedy_submodular_multidrone(path,d,energy_budget)
			reward_submodular.append(algo_reward[0])
			cost_submodular.append(algo_reward[1])
			if(algo_reward[0] != 0):
				ratio_submodular = algo_reward[0]/opt
			else:
				ratio_submodular = 0
			
			print("MR: ",ratio_submodular)

			algo_reward = greedy_weight_selection(path,d,energy_budget)
			reward_gw.append(algo_reward[1])
			cost_gw.append(algo_reward[2])
			if(algo_reward[1] != 0):
				ratio_gw = algo_reward[1]/opt
			else:
				ratio_gw = 0
			print("GSw: ",ratio_gw)

			algo_reward = greedy_ending_selection(path,d,energy_budget)
			reward_GEFT.append(algo_reward[1])
			cost_GEFT.append(algo_reward[2])
			if(algo_reward[1] != 0):
				ratio_geft = algo_reward[1]/opt
			else:
				ratio_geft = 0
			print("GeRt: ",ratio_geft)

			algo_reward = greedy_reward_selection(path,d,energy_budget)
			reward_gp.append(algo_reward[1])
			cost_gp.append(algo_reward[2])
			if(algo_reward[1] != 0):
				ratio_gp = algo_reward[1]/opt
			else:
				ratio_gp = 0
			print("GLp: ",ratio_gp)
			
			apx_s, apx_s_na = APX_coloring_M(path,d,energy_budget)
			if(apx_s[0]!=0):
				ratio_apx_s = apx_s[0]/opt
				ratio_apx_s_na = apx_s_na[0]/opt
			else:
				ratio_apx_s = 0
				ratio_apx_s_na = 0
			apxS.append(round(ratio_apx_s,4))
			apxS_na.append(round(ratio_apx_s_na,4))
			reward_apx.append(apx_s[0])
			reward_apx_na.append(apx_s_na[0])
			cost_apx.append(apx_s[1])
			cost_apx_na.append(apx_s_na[1])
			print("APX: ",ratio_apx_s," COST: ",apx_s[1]," REWARD: ",apx_s[0])
			print("APX NON-AUGMENTED: ",ratio_apx_s_na," COST: ",apx_s_na[1]," REWARD: ",apx_s_na[0])

			algo_reward = generalize_bin_packing(path,d,energy_budget)
			if (algo_reward[0]!=0):
				ratio_bin = algo_reward[0]/opt
			else:
				ratio_bin = 0
			print("BIN PACKING RATIO: ",ratio_bin, " COST: ",algo_reward[1], " REWARD: ", algo_reward[0], "#bin: ", algo_reward[3])
			bin_pack.append(round(ratio_bin,4))
			reward_bin.append(algo_reward[0])
			cost_bin.append(algo_reward[1])
			n_bins.append(algo_reward[3])

			algo_reward = naive_bin_packing(path,d,energy_budget)
			if (algo_reward[0] != 0):
				ratio_bin_naive = algo_reward[0]/opt
			else:
				ratio_bin_naive = 0
			print("NAIVE BIN PACKING RATIO: ",ratio_bin_naive, " COST: ",algo_reward[1], " REWARD: ", algo_reward[0])
			bin_pack_naive.append(round(ratio_bin_naive,4))
			reward_bin_naive.append(algo_reward[0])
			cost_bin_naive.append(algo_reward[1])

			knapsack.append(round(ratio_knapsack,4))
			submodular.append(round(ratio_submodular,4))
			gw.append(round(ratio_gw,4))
			geft.append(round(ratio_geft,4))
			gp.append(round(ratio_gp,4))
			avg_intersections.append(round(avg_intersection,4))
		print("Csv creation...")
		results = pd.DataFrame()
		
		results["apx"] = apxS
		results["apx-na"] = apxS_na
		print("AVG RATIO apx: ",sum(apxS) / len(apxS))
		print("AVG RATIO apx NON-AUGMENTED: ",sum(apxS_na) / len(apxS_na))
		results["knapsack"] = knapsack
		print("AVG RATIO knapsack: ",sum(knapsack) / len(knapsack))
		results["MR"] = submodular
		print("AVG RATIO MR: ",sum(submodular) / len(submodular))
		results["GeRt"] = geft
		print("AVG RATIO geft: ",sum(geft) / len(geft))
		results["GLp"] = gp
		print("AVG RATIO gp: ",sum(gp) / len(gp))
		results["GSw"] = gw
		print("AVG RATIO gw: ",sum(gw) / len(gw))

		results["BIN"] = bin_pack
		print("AVG RATIO BIN: ",sum(bin_pack) / len(bin_pack))
		results["BIN_naive"] = bin_pack_naive
		print("AVG RATIO NAIVE BIN: ",sum(bin_pack_naive) / len(bin_pack_naive))
		results["intersection"] = avg_intersections
		results["n_points"] = [elem - 1 for elem in n_points]

		results["apx_reward"] = reward_apx
		results["apx_cost"] = cost_apx

		results["apx-na_reward"] = reward_apx_na
		results["apx-na_cost"] = cost_apx_na

		results["knapsack_reward"] = reward_knapsack
		results["knapsack_cost"] = cost_knapsack

		results["MR_reward"] = reward_submodular
		results["MR_cost"] = cost_submodular

		results["GLp_reward"] = reward_gp
		results["GLp_cost"] = cost_gp

		results["GSw_reward"] = reward_gw
		results["GSw_cost"] = cost_gw

		results["GeRt_reward"] = reward_GEFT
		results["GeRt_cost"] = cost_GEFT

		
		results["BIN_reward"] = reward_bin
		results["BIN_cost"] = cost_bin
		results["BIN_naive_reward"] = reward_bin_naive
		results["BIN_naive_cost"] = cost_bin_naive
		results["#BIN"] = n_bins
		results.to_csv("corse_d"+str(d)+"_p"+str(len(delivery_points))+"_b"+str(energy_budget)+"_s"+str(drone_speed)+".csv")
		print("Csv created.")
		
	
	return

def format_takeoff_landing(path,delivery_points,n_class,payload,drone_speed, start_i):
	import timeit
	shortest_mission = []
	optimal_mission = []
	paths = []

	start = start_i
	real_wind = read_24h_wind(start)

	df = pd.DataFrame()
	

	f = open("result/intervals_with_gain"+str(len(delivery_points))+"_s"+str(drone_speed)+".txt","w")
	contatore = 0
	print("Trajectories computing...")
	for wind_direction, wind_speed, hour in real_wind:

		start = timeit.default_timer()
		wind_to_rep, wind_to_sect, relative_values = compute_wind_classes(n_class,False)
		dict_energy = compute_prefixes(payload,drone_speed,wind_speed,RELATIVE_WIND_VALUES)
		f.write("INTERVALS:\n")
		optimal_mission, shortest_mission = exhaustive_test_best_segment(path,delivery_points,n_class,wind_direction,wind_speed,payload,dict_energy)
		ss_optimal_mission, shortest_path_mission = exhaustive_test_same_segment(path,delivery_points,n_class,wind_direction,wind_speed,payload,dict_energy)
		optimal = trajectory_2d_to_1d_gain(path,optimal_mission,shortest_path_mission)
		contatore = contatore + 1
		print("ESECUZIONE ",contatore)
		paths.append(optimal)
		stop = timeit.default_timer()
		print('Time: ', stop - start) 
	f.write(str(paths))
	f.close()

	print("Trajectories computed.")
	
	return

def random_paths_tests():
	
	theta = [0,0.4,0.8,1]
	spans = [1500,10000,20000,30000]

	cont_theta = 0
	theta_index = 0
	budget = 5000
	drones = [5] #manca 1
	paths = generate_random_intervals()
	#THETAx3
	print("Test starting...")
	for d in drones:
		#f = open("random_paths_tests_verbose"+str(d)+".txt","w")
		opt = 0
		algo_reward = 0
		ratio_knapsack = 0
		ratio_bin = 0
		ratio_submodular = 0
		ratio_gp = 0
		ratio_gw = 0
		ratio_geft = 0
		knapsack = []
		submodular = []
		gp = []
		gw = []
		bin = []
		geft = []
		apxS = []
		apxS_na = []
		thetas = []
		r_spans = []
		avg_intersections = []
		n_points = []
		intersection = 0
		avg_intersection = 0
		#f.write("DRONES: "+str(d)+"\n")
		print("Test with drones: ",d)

		for path in paths:
			print("\nPath: \n")

			#for i in path:
			#	print(i)
			if cont_theta - len(spans) == 0:
				theta_index = (theta_index + 1) % len(theta)
				cont_theta = 0
			r_spans.append(spans[cont_theta])
			thetas.append(theta[theta_index])
			n_points.append(len(path))
			cont_theta = cont_theta + 1
			intersection = 0
			avg_intersection = 0
			#f.write("PATH: "+str(path)+"\n")

			for interval in path:
				intersection = intersection + how_many_intersection(interval, path)

			avg_intersection = intersection / len(path)
			#f.write("AVG INTERSECTION: "+str(avg_intersection)+"\n")

			print("Intersection: ",avg_intersection)
			opt = opt_ilp_cplex(path, 5000, d, False)
			
			algo_reward = knapsack_multidrone(path,d,5000)
			ratio_knapsack = algo_reward[0]/opt
			print("knapsack: ",ratio_knapsack)

			algo_reward = greedy_submodular_multidrone(path,d,5000)
			ratio_submodular = algo_reward[0]/opt
			print("MR: ",ratio_submodular)

			algo_reward = greedy_weight_selection(path,d,5000)
			ratio_gw = algo_reward[1]/opt
			print("GSw: ",ratio_gw)

			algo_reward = greedy_ending_selection(path,d,5000)
			ratio_geft = algo_reward[1]/opt
			print("GeRt: ",ratio_geft)

			algo_reward = greedy_reward_selection(path,d,5000)
			ratio_gp = algo_reward[1]/opt
			print("GLp: ",ratio_gp)

			algo_reward = generalize_bin_packing(path,d,5000)
			if (algo_reward[0]!=0):
				ratio_bin = algo_reward[0]/opt
			else:
				ratio_bin = 0

			apx_s, apx_s_na = APX_coloring_M(path,d,5000)
			ratio_apx_s = apx_s[0]/opt
			ratio_apx_s_na = apx_s_na[0]/opt
			apxS.append(round(ratio_apx_s,4))
			apxS_na.append(round(ratio_apx_s_na,4))
			print("APX: ",ratio_apx_s," COST: ",apx_s[1]," REWARD: ",apx_s[0]," INDEX: ",apx_s[2])
			print("APX NON-AUGMENTED: ",ratio_apx_s_na," COST: ",apx_s_na[1]," REWARD: ",apx_s_na[0]," INDEX: ",apx_s_na[2])

			#f.write("RATIO: knapsack-> "+str(ratio_knapsack)+" MR-> "+str(ratio_submodular)+" GSw-> "+str(ratio_gw)+" GeRt-> "+str(ratio_geft)+" GLp-> "+str(ratio_gp)+" APX-> "+str(ratio_apx_s)+"\n")
			
			
			knapsack.append(round(ratio_knapsack,4))
			submodular.append(round(ratio_submodular,4))
			gw.append(round(ratio_gw,4))
			geft.append(round(ratio_geft,4))
			gp.append(round(ratio_gp,4))
			bin.append(round(ratio_bin,4))
			avg_intersections.append(round(avg_intersection,4))
		print("Csv creation...")
		results = pd.DataFrame()

		#f.write("AVG RATIO:\n")
		
		results["apx"] = apxS
		results["apx-na"] = apxS_na
		print("AVG RATIO apx: ",sum(apxS) / len(apxS))
		print("AVG RATIO apx NON-AUGMENTED: ",sum(apxS_na) / len(apxS_na))
		results["knapsack"] = knapsack
		print("AVG RATIO knapsack: ",sum(knapsack) / len(knapsack))
		results["MR"] = submodular
		print("AVG RATIO MR: ",sum(submodular) / len(submodular))
		results["GeRt"] = geft
		print("AVG RATIO geft: ",sum(geft) / len(geft))
		results["GLp"] = gp
		print("AVG RATIO gp: ",sum(gp) / len(gp))
		results["GSw"] = gw
		print("AVG RATIO gw: ",sum(gw) / len(gw))
		results["BIN"] = bin
		results["intersection"] = avg_intersections
		results["theta"] = thetas
		results["n_points"] = [elem - 1 for elem in n_points]
		results["spans"] = r_spans
		#f.write("knapsack: "+str(sum(knapsack) / len(knapsack))+"\n")
		#f.write("MR: "+str(sum(submodular) / len(submodular))+"\n")
		#f.write("GeRt: "+str(sum(geft) / len(geft))+"\n")
		#f.write("GLp: "+str(sum(gp) / len(gp))+"\n")
		#f.write("GSw: "+str(sum(gw) / len(gw))+"\n")
		#f.write("APX: "+str(sum(apxS) / len(apxS))+"\n")
		#f.write("APX-NON-AUGMENTED: "+str(sum(apxS_na) / len(apxS_na))+"\n")
		#f.close()
		results.to_csv("random_paths_tests_drone"+str(d)+".csv")
		print("Csv created.")
		
	return

if __name__ == '__main__':

	n_wind_classes = 10
	DRONE_SPEED = 20 #m/s
	WIND_DIRECTION = 110 #degrees
	WIND_SPEED = 20 #m/s
	PAYLOAD = 6 #kg
	AREA_RAY = 10000 #meters
	NUMBER_OF_SIDES = 8
	UNIT_ENERGY = {}#dictionary of unitary cost the key: [payload,relative_wind]
	WIND_SECTOR_REP = {}#dictionary of wind classes range the key: [range(lower,upper)]
	WIND_SECTOR = {}#dictionary of wind sector range the key: [range(lower,upper)]

	WIND_SECTOR_REP, WIND_SECTOR, RELATIVE_WIND_VALUES = compute_wind_classes(n_wind_classes,False)
	UNIT_ENERGY = compute_prefixes(PAYLOAD,DRONE_SPEED,WIND_SPEED,RELATIVE_WIND_VALUES)
	#test = generate_random_intervals()
	#test = [[[0,0,0,0],[6,7,2000,30],[8,9,3000,30],[10,11,2100,30],[14,15,1400,30],[18,19,2000,30],[20,21,2000,30],[33,37,2000,30],[38,40,2000,30],[56,57,2000,30]]]
	"""for path in test:
		s,s_na = APX_coloring_M(path, 3, 5000)
		#print("MIAO1")
		opt = opt_ilp_cplex(path, 5000, 3, False)
		#print("MIAO")
		print("\n\n RATIO: ",s[0]/opt," RATIO-na: ",s_na[0]/opt)
	"""
	random_paths_tests()
	print("FINE")
	exit()
	vertices = random_convex_polygon(NUMBER_OF_SIDES,AREA_RAY)
	MT_PATH = compute_MT_path(vertices,False)
	MT_PATH = augmentation_segments_info(MT_PATH)

	print("FINE")

	#print_test_results_avg_corse_wind(MT_PATH,rnd_delivery_points,n_wind_classes,PAYLOAD,DRONE_SPEED, 2)
	#rnd_delivery_points = generate_random_delivery(vertices,100,MT_PATH)

	#print_test_results_avg_corse_wind(MT_PATH,rnd_delivery_points,n_wind_classes,PAYLOAD,DRONE_SPEED, 2)
	exit()
	optimal = []
	optimal_90 = [] 
	tic = time.perf_counter()
	vertices = random_convex_polygon(NUMBER_OF_SIDES,AREA_RAY)
	print(vertices)
	MT_PATH = compute_MT_path(vertices,False)
	MT_PATH = augmentation_segments_info(MT_PATH)
	print("CONVEX POLYGON GENERATION CORRECTNESS: ",len(MT_PATH) == NUMBER_OF_SIDES)
	point_step=[50,100,150,200]
	rnd_points = []
	for n in point_step:
		rnd_delivery_points = generate_random_delivery(vertices,n,MT_PATH)
		rnd_points.append(rnd_delivery_points)
	#P = rnd_delivery_points.pop(0)
	
	test_avg_ratio_fixed_poly_fixed_points(MT_PATH,rnd_points,30)
	"""
	optimal, optimal_90 = exhaustive_test_best_segment(MT_PATH,rnd_delivery_points,n_wind_classes,WIND_DIRECTION,WIND_SPEED,PAYLOAD,UNIT_ENERGY)
	interval_1d = trajectory_2d_to_1d(MT_PATH,optimal,UNIT_ENERGY,PAYLOAD)
	print(interval_1d[1])

	pred_interval_1d = previous_intervals(interval_1d)
	print("INPUT: ",interval_1d," PREDECESSOR: ",pred_interval_1d)
	#for i in range(len(interval_1d)):
	#	print(i,": [",interval_1d[i][0],",",interval_1d[i][1],"] -> ",pred_interval_1d[i]," w: ",interval_1d[i][2]," r: ", interval_1d[i][3])
	
	sol = knapsack_variant_unitary_reward(5000,interval_1d,pred_interval_1d,False)
	print("KNAPSACK: cost-> ",sol[1]," reward-> ",sol[0]," interval-> ",sol[2])
	#print("SUBMODULAR MULTI: ",greedy_submodular_multidrone(interval_1d,3,5000))
	#sol = greedy_weight_selection(interval_1d,1)
	#re = 0
	
	cost = 0
	for i in sol[0]:
		re = re + i[3]
		cost = cost + i[2]
	print("GREEDY_Weight: cost-> ",cost," reward-> ",re)
	sol = greedy_reward_selection(interval_1d,1)
	re = 0
	cost = 0
	for i in sol[0]:
		re = re + i[3]
		cost = cost + i[2]
	print("GREEDY_Reward: cost-> ",cost," reward-> ",re)
	sol = greedy_ending_selection(interval_1d,1)
	re = 0
	cost = 0
	for i in sol[0]:
		re = re + i[3]
		cost = cost + i[2]
	print("GREEDY_Ending: cost-> ",cost," reward-> ",re)

	sol,cost,re = greedy_submodular(interval_1d)
	print("GREEDY sub: cost-> ",cost," reward-> ",re)

	toc = time.perf_counter()
	print(f"Test in {toc - tic:0.4f} seconds")
	plot_polygon(MT_PATH, vertices, rnd_delivery_points)
	"""