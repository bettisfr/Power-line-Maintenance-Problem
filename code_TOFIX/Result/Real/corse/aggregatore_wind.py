import pandas as pd

d = [1,3,5]
points = [25,50,75,100]
for i in d:
	for j in points:
		df = pd.read_csv("corse_d"+str(i)+"_p_"+str(j)+".csv",sep=",")

		grouped_multiple = df.groupby(df.index//24).agg({
			'apx': ['mean', 'min', 'max','std','var'],
			'apx-na': ['mean', 'min', 'max','std','var'],
			'MR': ['mean', 'min', 'max','std','var'],
			'GeRt': ['mean', 'min', 'max','std','var'],
			'GLp': ['mean', 'min', 'max','std','var'],
			'GSw': ['mean', 'min', 'max','std','var'],
			'knapsack': ['mean', 'min', 'max','std','var']
			})
		grouped_multiple.columns=[
		'Col-M_mean', 'Col-M_min', 'Col-M_max','Col-M_std','Col-M_var',
		'Col-M-na_mean', 'Col-M-na_min', 'Col-M-na_max','Col-M-na_std','Col-M-na_var',
		'MR-M_mean', 'MR-M_min', 'MR-M_max','MR-M_std','MR-M_var',
		'GeRt-M_mean', 'GeRt-M_min', 'GeRt-M_max','GeRt-M_std','GeRt-M_var',
		'GLp-M_mean', 'GLp-M_min', 'GLp-M_max','GLp-M_std','GLp-M_var',
		'GSw-M_mean', 'GSw-M_min', 'GSw-M_max','GSw-M_std','GSw-M_var',
		'knapsack_mean', 'knapsacks_min', 'knapsack_max','knapsack_std','knapsack_var'
		]
		grouped_multiple = grouped_multiple.reset_index()
		print(grouped_multiple)

		grouped_multiple.to_csv("aggreagati"+str(i)+"_"+str(j)+".csv")