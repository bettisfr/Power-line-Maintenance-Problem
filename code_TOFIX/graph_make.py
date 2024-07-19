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
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica"]})

mpl.rcParams['text.usetex']=True
mpl.rcParams['text.latex.unicode']=True
#################################################################################################################################
drones = [3,5]
points = [25,50,75]
budget = [5000,10000]
speeds = [10,20]
for d in drones:
	for p in points:
		for b in budget:
			for s in speeds:
				stringa = "corse_d"+str(d)+"_p"+str(p)+"_b"+str(b)+"_s"+str(s)
				df = pd.read_csv(stringa+".csv")
				df.rename( columns={'Unnamed: 0' :'index'}, inplace=True)
				#apx,apx-na,knapsack,MR,GeRt,GLp,GSw,
				#apx_reward,apx_cost,apx-na_reward,apx-na_cost,knapsack_reward,knapsack_cost,MR_reward,MR_cost,GLp_reward,GLp_cost,GSw_reward,GSw_cost,GeRt_reward,GeRt_cost

				#x_axes = df[['knapsack gain',' MR gain','apx gain','apx-na gain','GLp gain','GSw gain','GeRt gain']]
				#print(df['BIN_naive_reward'].mean())
				x = ['\sc{knapsack}',' \sc{MR}','\sc{APX}','\sc{APX-na}','\sc{GLp}','\sc{GSw}','\sc{GeRt}','\sc{BIN}','\sc{BIN-naive}']
				#x = ['\sc{knapsack}',' \sc{MR}','\sc{APX}','\sc{APX-na}','\sc{GLp}','\sc{GSw}','\sc{GeRt}','\sc{BIN}']
				y = [df['knapsack_reward'].mean(),df['MR_reward'].mean(),df['apx_reward'].mean(),df['apx-na_reward'].mean(),df['GLp_reward'].mean(),df['GSw_reward'].mean(),df['GeRt_reward'].mean(),df['BIN_reward'].mean(),df['BIN_naive_reward'].mean()]
				#y = [df['knapsack_reward'].mean(),df['MR_reward'].mean(),df['apx_reward'].mean(),df['apx-na_reward'].mean(),df['GLp_reward'].mean(),df['GSw_reward'].mean(),df['GeRt_reward'].mean(),df['BIN_reward'].mean()]

				y_pos = np.arange(len(x))
				plt.bar(y_pos, y, align='center',color=['#2A9D8F','#F4A261','#d00000','#606c38','#88A09E','#ca6702','#fca311','#588157','#F1B5CB'], zorder=3)
				plt.grid(zorder=0)
				plt.xticks(y_pos, x)
				plt.ylabel('\sc{Average Energy Consumption Gain} (kJ)')
				plt.xlabel('\sc{Algorithms}')
				plt.savefig("graphs/"+stringa+'.pdf')
				print("Plot ",stringa," done.")
			

