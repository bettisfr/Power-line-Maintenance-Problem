import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Directory where the CSV files are located
output_dir = 'output'  # Change this to the correct path if needed

# List to store dataframes
df_list = []

# Loop through all files in the directory
for filename in os.listdir(output_dir):
    if filename.endswith('.csv'):
        # Construct the file path
        file_path = os.path.join(output_dir, filename)

        # Read the CSV file into a dataframe
        df = pd.read_csv(file_path)

        # Append the dataframe to the list
        df_list.append(df)

# Combine all dataframes into a single dataframe
final_df = pd.concat(df_list, ignore_index=True)

# Display the resulting dataframe (optional)
# print(final_df.head())  # Print first few rows for inspection

# Parameter vectors
NUM_DELIVERIES_VEC = [5, 10, 15, 20]
MAX_WEIGHT_VEC = [1, 5]
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
ALGORITHMS = [0, 1, 2, 4, 5, 6, 7, 8] # remember to re-add 3

# Make sure 'plots' directory exists
os.makedirs('plots', exist_ok=True)

# Set the style for the plot
sns.set_theme(style="whitegrid")

# Iterate through combinations of fixed parameters
for max_weight in MAX_WEIGHT_VEC:
    for drone_load in DRONE_LOAD_VEC:
        for drone_battery in DRONE_BATTERY_VEC:
            # Filter the dataframe based on the fixed parameters
            filtered_df = final_df[
                (final_df['max_weight'] == max_weight) &
                (final_df['drone_load'] == drone_load) &
                (final_df['drone_battery'] == drone_battery)
                ]

            # Debugging: Check the filtered data
            # print(f"Filtered Data for max_weight={max_weight}, drone_load={drone_load}, drone_battery={drone_battery}:")
            # print(filtered_df[['num_deliveries', 'algorithm', 'total_profit_avg', 'total_profit_std']])

            if filtered_df.empty:
                print(f"No data found for max_weight={max_weight}, drone_load={drone_load}, drone_battery={drone_battery}")
                continue

            # Plot setup
            plt.figure(figsize=(10, 6))

            # Plot total_profit_avg and total_profit_std for each algorithm
            for algorithm in ALGORITHMS:
                algo_data = filtered_df[filtered_df['algorithm'] == algorithm]
                # Sort the data by num_deliveries to ensure the x-axis is in order
                algo_data = algo_data.sort_values(by='num_deliveries')

                if algo_data.empty:
                    print(f"No data for algorithm {algorithm} with the given filter")
                    continue

                # print(f"Algo={algorithm}")
                # print(algo_data[['num_deliveries', 'algorithm', 'total_profit_avg', 'total_profit_std']])

                plt.errorbar(
                    algo_data['num_deliveries'],
                    algo_data['total_profit_avg'],
                    yerr=algo_data['total_profit_std'],
                    label=f'Algorithm {algorithm}',
                    fmt='-o',
                    capsize=5
                )

            # Labels and title
            plt.title(f'Total Profit vs. Number of Deliveries\n(max_weight={max_weight}, drone_load={drone_load}, drone_battery={drone_battery})', fontsize=14)
            plt.xlabel('Number of Deliveries', fontsize=12)
            plt.ylabel('Total Profit', fontsize=12)
            plt.xticks(range(min(final_df['num_deliveries']), max(final_df['num_deliveries']) + 1, 5))
            plt.legend(title="Algorithms", bbox_to_anchor=(1.05, 1), loc='upper left')

            # Save the plot to the "plots" folder
            plot_filename = f"plots/profit_vs_deliveries_maxw{max_weight}_load{drone_load}_batt{drone_battery}.pdf"
            plt.tight_layout()

            # Check if plot has data before saving
            if not plt.gca().has_data():
                print(f"No data to plot for max_weight={max_weight}, drone_load={drone_load}, drone_battery={drone_battery}")
                plt.close()
                continue

            plt.savefig(plot_filename)
            plt.close()

            print(f"Plot saved to {plot_filename}")
