import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
from scipy import stats

def reward_func(row):
	return row['knapsack_reward'] / row['knapsack']

def cost_func(row):
	return row['knapsack_cost'] / row['knapsack']

drones = [1,3,5]
points = [25,50,75]
budget = [5000,10000]
speeds = [10,20]

knapsack = []
r_knapsack = []
c_knapsack = []

col = []
r_col = []
c_col = []

col_na = []
r_col_na = []
c_col_na = []

bins = []
r_bins = []
c_bins = []

CRp = []
r_CRp = []
c_CRp = []

Sw = []
r_Sw = []
c_Sw = []

Lp = []
r_Lp = []
c_Lp = []

MR = []
r_MR = []
c_MR = []

r_opt = []
c_opt = []

for s in speeds:
	for p in points:
		for b in budget:
			knapsack = []
			r_knapsack = []
			c_knapsack = []
			std_knapsack = []
			std_reward_knap =[]

			col = []
			r_col = []
			c_col = []
			std_col = []
			std_reward_col = []

			col_na = []
			r_col_na = []
			c_col_na = []
			std_col_na = []
			std_reward_col_na = []

			bins = []
			r_bins = []
			c_bins = []
			std_bins = []
			std_reward_bins = []

			CRp = []
			r_CRp = []
			c_CRp = []
			std_CRp = []
			std_reward_CRp = []

			Sw = []
			r_Sw = []
			c_Sw = []
			std_Sw = []
			std_reward_Sw = []

			Lp = []
			r_Lp = []
			c_Lp = []
			std_Lp = []
			std_reward_Lp = []

			MR = []
			r_MR = []
			c_MR = []
			std_MR = []
			std_reward_MR = []

			r_opt = []
			c_opt = []
			std_opt = []
			std_reward_opt = []

			for d in drones:
				stringa = "c"+str(s)+"ms/corse_d"+str(d)+"_p"+str(p)+"_b"+str(b)+"_s"+str(s)
				if d == 1:
					stringa = "c"+str(s)+"ms/corse_d"+str(d)+"_p"+str(p)+"_b"+str(b)
				print(stringa)
				df = pd.read_csv(stringa+".csv")
				#if(d == 1):
				#	df['knapsack'] = df['knapsack'].replace(0, 1)
				
				df['opt_reward'] = df.apply (lambda row: reward_func(row), axis=1)
				df.rename( columns={'Unnamed: 0':'nn'}, inplace=True )
				df = df.groupby(df.index // 24).mean()
				df.to_csv("raw_s"+str(s)+"_p"+str(p)+"_b"+str(b)+".csv")
				#opt
				r_opt.append(round(df['opt_reward'].mean(),4))
				#knapsack
				
				
				knapsack.append(round(df['knapsack'].mean(),4))
				r_knapsack.append(round(df['knapsack_reward'].mean(),4))
				c_knapsack.append(round(df['knapsack_cost'].mean(),4))
				std_knapsack.append(round(df['knapsack'].std(),4))
				std_reward_knap.append(round(df['knapsack_reward'].std(),4))
				#COL-na
				col_na.append(round(df['apx-na'].mean(),4))
				r_col_na.append(round(df['apx-na_reward'].mean(),4))
				c_col_na.append(round(df['apx-na_cost'].mean(),4))
				std_col_na.append(round(df['apx-na'].std(),4))
				std_reward_col_na.append(round(df['apx-na_reward'].std(),4))
				#COL
				col.append(round(df['apx'].mean(),4))
				r_col.append(round(df['apx_reward'].mean(),4))
				c_col.append(round(df['apx_cost'].mean(),4))
				std_col.append(round(df['apx'].std(),4))
				std_reward_col.append(round(df['apx_reward'].std(),4))
				#bin
				bins.append(round(df['BIN'].mean(),4))
				r_bins.append(round(df['BIN_reward'].mean(),4))
				c_bins.append(round(df['BIN_cost'].mean(),4))
				std_bins.append(round(df['BIN'].std(),4))
				std_reward_bins.append(round(df['BIN_reward'].std(),4))
				#CRp
				CRp.append(round(df['GeRt'].mean(),4))
				r_CRp.append(round(df['GeRt_reward'].mean(),4))
				c_CRp.append(round(df['GeRt_cost'].mean(),4))
				std_CRp.append(round(df['GeRt'].std(),4))
				std_reward_CRp.append(round(df['GeRt_reward'].std(),4))
				#Sw
				Sw.append(round(df['GSw'].mean(),4))
				r_Sw.append(round(df['GSw_reward'].mean(),4))
				c_Sw.append(round(df['GSw_cost'].mean(),4))
				std_Sw.append(round(df['GSw'].std(),4))
				std_reward_Sw.append(round(df['GSw_reward'].std(),4))
				#Lp
				Lp.append(round(df['GLp'].mean(),4))
				r_Lp.append(round(df['GLp_reward'].mean(),4))
				c_Lp.append(round(df['GLp_cost'].mean(),4))
				std_Lp.append(round(df['GLp'].std(),4))
				std_reward_Lp.append(round(df['GLp_reward'].std(),4))
				#Mr
				MR.append(round(df['MR'].mean(),4))
				r_MR.append(round(df['MR_reward'].mean(),4))
				c_MR.append(round(df['MR_cost'].mean(),4))
				std_MR.append(round(df['MR'].std(),4))
				std_reward_MR.append(round(df['MR_reward'].std(),4))

			print('Saving procedure starts...')
			rs = pd.DataFrame()
			rs['opt_reward'] = r_opt
			rs['opt'] = [1,1,1]
			rs['opt_std'] = [0,0,0]
			
			rs['knapsack'] = knapsack
			rs['knapsack_reward'] = r_knapsack
			rs['knapsack_cost'] = c_knapsack
			rs['knapsack_std'] = std_knapsack
			rs['knapsack_std_reward'] = std_reward_knap

			rs['col'] = col
			rs['col_reward'] = r_col
			rs['col_cost'] = c_col
			rs['col_std'] = std_col
			rs['col_std_reward'] = std_reward_col

			rs['col_na'] = col_na
			rs['col_reward_na'] = r_col_na
			rs['col_cost_na'] = c_col_na
			rs['col_na_std'] = std_col_na
			rs['col_std_reward_na'] = std_reward_col_na

			rs['BIN'] = bins
			rs['BIN_reward'] = r_bins
			rs['BIN_cost'] = c_bins
			rs['BIN_std'] = std_bins
			rs['BIN_std_reward'] = std_reward_bins

			rs['CRp'] = CRp
			rs['CRp_reward'] = r_CRp
			rs['CRp_cost'] = c_CRp
			rs['CRp_std'] = std_CRp
			rs['CRp_std_reward'] = std_reward_CRp


			rs['Sw'] = Sw
			rs['Sw_reward'] = r_Sw
			rs['Sw_cost'] = c_Sw
			rs['Sw_std'] = std_Sw
			rs['Sw_std_reward'] = std_reward_Sw

			rs['Lp'] = Lp
			rs['Lp_reward'] = r_Lp
			rs['Lp_cost'] = c_Lp
			rs['Lp_std'] = std_Lp
			rs['Lp_std_reward'] = std_reward_Lp

			rs['MR'] = MR
			rs['MR_reward'] = r_MR
			rs['MR_cost'] = c_MR
			rs['MR_std'] = std_MR
			rs['MR_std_reward'] = std_reward_MR
			
			rs.to_csv("agg_s"+str(s)+"_p"+str(p)+"_b"+str(b)+".csv")
			print('Saving procedure done.')