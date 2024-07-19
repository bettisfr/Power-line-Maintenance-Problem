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
		atan = mt_angle + alpha_t
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
		atan = mt_angle + alpha_l
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

def generate_random_delivery(points,n):
	poly = geometry.Polygon(points)
	pts = random_points_within(poly,n)
	
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

def random_points_within(poly, num_points):
	min_x, min_y, max_x, max_y = poly.bounds

	points = []

	while len(points) < num_points:
		random_point = geometry.Point([random.randrange(round(min_x), round(max_x)), random.randrange(round(min_y), round(max_y))])
		if (random_point.within(poly)):
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
	p1H = Segment(segment[0].p1,H)
	Hp2 = Segment(H,segment[0].p2)
	#print("AH: ",N(p1H.p1),",",N(p1H.p2),"\n")
	#print("HB: ",N(Hp2.p1),",",N(Hp2.p2),"\n")
	takeoff_threshold = N(p1H.length)
	landing_threshold = N(Hp2.length)
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
				
		if(HL_segment <= landing_threshold):
		
			L, PL = compute_point_Landing(HL_segment,mt_angle,which_alhaL,point,segment)
			if(isOnSegment(segment[0].p1,segment[0].p2,L)):
				relative_w_l = ((wind_direction - (atan_landing))) % 360
				wc_l = wind_class(relative_w_l)
				sector_l = wind_sector(relative_w_l)
				unit_l = dict_unit[0,wc_l]
				cost_l = unit_l * N(PL.length)
				landing_points.append((L,PL,unit_l,cost_l,atan_landing,N(PL.length),wc_l,which_alhaL,sector_l,segment))
		if(HT_segment <= takeoff_threshold):
		
			T,TP = compute_point_Takeoff(HT_segment,mt_angle,which_alhaT,point,segment)
			if(isOnSegment(segment[0].p1,segment[0].p2,T)):
				relative_w_t = ((wind_direction - (atan_takeoff))) % 360
				wc_t = wind_class(relative_w_t)
				sector_t = wind_sector(relative_w_t)
				unit_t = dict_unit[PAYLOAD,wc_t]
				cost_t = unit_t * N(TP.length)
				takeoff_points.append((T,TP,unit_t,cost_t,atan_takeoff,N(TP.length),wc_t,which_alhaT,sector_t,segment))
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
	for index in range(start,start+24):
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
		print(trajectory[0])

		t_i = path.index(trajectory[0][9])
		l_i = path.index(trajectory[1][9])

		x1d = length_t + list_slack[t_i]
		y1d = length_l + list_slack[l_i]
		print("A: ",N(trajectory[0][9][0].p1)," B: ",N(trajectory[0][9][0].p2)," direction: ", N(trajectory[0][9][2])," A1: ",N(trajectory[1][9][0].p1)," B1: ",N(trajectory[1][9][0].p2)," direction: ",N(trajectory[1][9][2]))
		print("takeoff: ",t," landing: ",l,"T: ",t_i," L: ",l_i,"Length t: ", length_t, " Length l: ",length_l," x1d: ",x1d," y1d: ",y1d,"\n")
		p = abs(max_p - (trajectory[0][2]/math.sin(math.radians(trajectory[0][7])))) + abs(max_0 - (trajectory[1][2]/math.sin(math.radians(trajectory[1][7]))))

		w = round(trajectory[0][3] + trajectory[1][3])
		points_1d.append([round(N(x1d)),round(N(y1d)),w,p])

	return sorted(points_1d, key=lambda x: (x[1]))

def PreProcessingDynamic(interval_1d):
    ### predecessor list, Pred_p[i] contain the last delivery compatible with delivery i 
	Pred_list = []
	for i in range(len(interval_1d)):
		Pred_list.append(findLastCompatibleDelivery(interval_1d,i))
	return Pred_list

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
		reward.insert(0,0)

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
				else:
					M[i][w][0] = row_reward
					M[i][w][1] = row_cost
					M[i][w][2] = M[i-1][w][2]
			else:
				M[i][w] = M[i-1][w]
			#print("M[ ",i,", ",w," ]->",M[i][w])	
	return M[D-1][W]

def reward_knapsack_recompute(solution,intervals):
	reward = 0
	for i in solution[2]:
		reward = reward + intervals[i][3]
	return reward

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

def brute_force_multiknapsack(intervals,n_drones,budget=5000):
	#############TODO
	return

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

def how_many_intersection(interval, intervals):
	count = 0
	for i in intervals:
		if(getOverlap(interval,i) != 0):
			count = count + 1
	return count

def test_avg_ratio_fixed_poly_fixed_points(path,dps,wind_step):
	direction_wind = 0
	knapsack_sol = 0
	knapsack_sol_r = 0
	submodular_sol = 0
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
	while(direction_wind <= 359):
		print("computing for wind direction ",direction_wind,"...")
		WIND_SECTOR_REP, WIND_SECTOR, RELATIVE_WIND_VALUES = compute_wind_classes(n_wind_classes,False)
		UNIT_ENERGY = compute_prefixes(PAYLOAD,DRONE_SPEED,WIND_SPEED,RELATIVE_WIND_VALUES)
		optimal = []
		optimal_90 = [] 
		optimal, optimal_90 = exhaustive_test_best_segment(path,dps,n_wind_classes,direction_wind,WIND_SPEED,PAYLOAD,UNIT_ENERGY)
		interval_1d = trajectory_2d_to_1d(MT_PATH,optimal,UNIT_ENERGY,PAYLOAD)
		intersection = 0
		for inter in interval_1d:
			intersection = intersection + how_many_intersection(inter, interval_1d)
		avg_inter = avg_inter + intersection
		print("INTERSECTION AVG: ",intersection/len(interval_1d),"\n")
		pred_interval_1d = previous_intervals(interval_1d)
		for i in range(len(interval_1d)):
			print(i,": [",interval_1d[i][0],",",interval_1d[i][1],"] -> ",pred_interval_1d[i]," w: ",interval_1d[i][2]," r: ", interval_1d[i][3])
		knapsack_sol = knapsack_variant_unitary_reward(5000,interval_1d,pred_interval_1d,False)
		weight_sol = greedy_weight_selection(interval_1d,1)
		reward_sol = greedy_reward_selection(interval_1d,1)
		end_sol = greedy_ending_selection(interval_1d,1)
		submodular_sol = greedy_submodular(interval_1d)
		knapsack_sol_r = reward_knapsack_recompute(knapsack_sol,interval_1d)
		print("SOLUTION: KNAPSACK-> ",knapsack_sol[2], " DDSP-> ",submodular_sol[3],"\n")
		print("RATIO: DDSP-> ",submodular_sol[2]/knapsack_sol_r, " GEFT-> ",end_sol[1]/knapsack_sol_r," GW-> ",weight_sol[1]/knapsack_sol_r," GP-> ",reward_sol[1]/knapsack_sol_r)
		avg_ddsp = avg_ddsp + submodular_sol[2]/knapsack_sol_r
		avg_geft = avg_geft + end_sol[1]/knapsack_sol_r
		avg_gw = avg_gw + weight_sol[1]/knapsack_sol_r
		avg_gp = avg_gp + reward_sol[1]/knapsack_sol_r
		direction_wind = direction_wind + wind_step
		iteratore = iteratore + 1
	print("AVG INTERSECTION: ",avg_inter/iteratore," AVG DDSP: ",avg_ddsp/iteratore," AVG GEFT: ",avg_geft/iteratore," avg_gp: ",avg_gp/iteratore," avg_gw: ",avg_gw/iteratore)
	return

if __name__ == '__main__':

	n_wind_classes = 6
	DRONE_SPEED = 20 #m/s
	WIND_DIRECTION = 0 #degrees
	WIND_SPEED = 20 #m/s
	PAYLOAD = 6 #kg
	AREA_RAY = 5000 #meters
	NUMBER_OF_SIDES = 8
	UNIT_ENERGY = {}#dictionary of unitary cost the key: [payload,relative_wind]
	WIND_SECTOR_REP = {}#dictionary of wind classes range the key: [range(lower,upper)]
	WIND_SECTOR = {}#dictionary of wind sector range the key: [range(lower,upper)]

	WIND_SECTOR_REP, WIND_SECTOR, RELATIVE_WIND_VALUES = compute_wind_classes(n_wind_classes,False)
	UNIT_ENERGY = compute_prefixes(PAYLOAD,DRONE_SPEED,WIND_SPEED,RELATIVE_WIND_VALUES)

	optimal = []
	optimal_90 = [] 

	vertices = random_convex_polygon(NUMBER_OF_SIDES,AREA_RAY)
	print(vertices)
	MT_PATH = compute_MT_path(vertices,False)
	MT_PATH = augmentation_segments_info(MT_PATH)
	print("CONVEX POLYGON GENERATION CORRECTNESS: ",len(MT_PATH) == NUMBER_OF_SIDES)
	rnd_delivery_points = generate_random_delivery(vertices,50)
	#P = rnd_delivery_points.pop(0)
	tic = time.perf_counter()
	test_avg_ratio_fixed_poly_fixed_points(MT_PATH,rnd_delivery_points,30)
	#optimal, optimal_90 = exhaustive_test_best_segment(MT_PATH,rnd_delivery_points,n_wind_classes,WIND_DIRECTION,WIND_SPEED,PAYLOAD,UNIT_ENERGY)
	#interval_1d = trajectory_2d_to_1d(MT_PATH,optimal,UNIT_ENERGY,PAYLOAD)
	#pred_interval_1d = previous_intervals(interval_1d)
	#for i in range(len(interval_1d)):
	#	print(i,": [",interval_1d[i][0],",",interval_1d[i][1],"] -> ",pred_interval_1d[i]," w: ",interval_1d[i][2]," r: ", interval_1d[i][3])
	#sol = knapsack_variant_unitary_reward(5000,interval_1d,pred_interval_1d,False)
	#print("KNAPSACK: cost-> ",sol[1]," reward-> ",sol[0]," interval-> ",sol[2])
	#sol = greedy_weight_selection(interval_1d,1)
	#re = 0
	"""
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
	"""
	toc = time.perf_counter()
	print(f"Test in {toc - tic:0.4f} seconds")
	plot_polygon(MT_PATH, vertices, rnd_delivery_points)