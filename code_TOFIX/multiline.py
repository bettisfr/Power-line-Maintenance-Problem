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


