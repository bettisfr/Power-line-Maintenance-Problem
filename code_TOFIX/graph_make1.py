import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
from scipy import stats



#cost_ss
#cost_90
#cost_90_ss
#cost_o


#################################################################################################################################
drones = [3,5]
points = [25,50,75]
budget = [5000,10000]
speeds = [10,20] 

#apx_reward,apx_cost,apx-na_reward,apx-na_cost,knapsack_reward,knapsack_cost,MR_reward,MR_cost,GLp_reward,GLp_cost,GSw_reward,GSw_cost,GeRt_reward,GeRt_cost
for d in drones:
	for p in points:
		for b in budget:
			for s in speeds:
				stringa = "corse_d"+str(d)+"_p"+str(p)+"_b"+str(b)+"_s"+str(s)
				df = pd.read_csv(stringa+".csv")
				
				df.rename( columns={'Unnamed: 0' :'index'}, inplace=True)
				# gca stands for 'get current axis'
				ax = plt.gca()
				df.plot(kind='line',x='index',y='knapsack_reward',color='#2A9D8F',ax=ax)
				df.plot(kind='line',x='index',y='MR_reward', color='#190E4F', ax=ax)
				df.plot(kind='line',x='index',y='apx_reward', color='#B33C86', ax=ax)
				df.plot(kind='line',x='index',y='apx-na_reward', color='#d00000', ax=ax)
				df.plot(kind='line',x='index',y='GLp_reward', color='#606c38', ax=ax)
				df.plot(kind='line',x='index',y='GSw_reward', color='#88A09E', ax=ax)
				df.plot(kind='line',x='index',y='GeRt_reward', color='#fca311', ax=ax)
				df.plot(kind='line',x='index',y='BIN_reward', color='#588157', ax=ax)
				df.plot(kind='line',x='index',y='BIN_naive_reward', color='#F1B5CB', ax=ax)
				plt.xticks(np.arange(0, 240, step=24))
				plt.ylabel('Energy Consumption Gain (kJ)')
				plt.xlabel('Hours')
				plt.savefig("graphs/"+stringa+'deep.pdf')
				print("Plot ",stringa," done.")
				plt.clf()
