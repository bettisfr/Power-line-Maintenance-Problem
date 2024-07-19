import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
import math
from scipy import stats

def reward_func(row):
	return row['knapsack_reward'] / row['knapsack']

def cost_func(row):
	return row['knapsack_cost'] / row['knapsack']

drones = [1,3,5]
points = [25,50,75]
budget = [5000,10000]
speeds = [10,20]

opt_mean = []
pos = [0,1,2,3,4,5,6,7,8,9]

for s in speeds:
	for p in points:
		for b in budget:
			for d in drones:
				opt_mean = []
				stringa = "c"+str(s)+"ms/corse_d"+str(d)+"_p"+str(p)+"_b"+str(b)+"_s"+str(s)
				if d == 1:
					stringa = "c"+str(s)+"ms/corse_d"+str(d)+"_p"+str(p)+"_b"+str(b)
				print(stringa)
				df = pd.read_csv(stringa+".csv")
				#if(d == 1):
				#	df['knapsack'] = df['knapsack'].replace(0, 1)
				df['opt_reward'] = df.apply (lambda row: reward_func(row), axis=1)
				media = 0
				for index, row in df.iterrows():
					if (index+1) % 24 == 0:
						opt_mean.append(round(media/24,4))
						if(not math.isnan(row['opt_reward'])):
							media = row['opt_reward']
						print('mean')
					else:
						if(not math.isnan(row['opt_reward'])):
							media = media + row['opt_reward']
						
				print(len(opt_mean))
				print(len(pos))
				rs = pd.DataFrame()
				rs['index'] = pos
				rs['opt_reward'] = opt_mean

				rs.to_csv("raw_s"+str(s)+"_p"+str(p)+"_b"+str(b)+"_d"+str(d)+".csv")
				