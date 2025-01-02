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

# Parameter vectors
NUM_DELIVERIES_VEC = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
MAX_WEIGHT_VEC = [1, 5]
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
ALGORITHMS = [0, 1, 2, 3, 4, 5, 6]

algorithm_str = {
    0: "OPT",
    1: "BIN",
    2: "KNA",
    3: "COL",
    4: "GMP",
    5: "GmE",
    6: "GMP-E",
    # 7: "GM profit/load",
    # 8: "GM profit+neigh",
}

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

            if filtered_df.empty:
                print(f"No data found for max_weight={max_weight}, drone_load={drone_load}, drone_battery={drone_battery}")
                continue

            # Define the metrics to plot
            metrics = [
                ('total_profit_avg', 'total_profit_std', 'Total Profit'),
                ('total_energy_avg', 'total_energy_std', 'Total Energy'),
                ('total_flights_avg', 'total_flights_std', 'Total Flights')
            ]

            for metric_avg, metric_std, metric_label in metrics:
                # Plot setup
                plt.figure(figsize=(4.5, 3.5))

                # Plot the metric for each algorithm
                for algorithm in ALGORITHMS:
                    algo_data = filtered_df[filtered_df['algorithm'] == algorithm]
                    # Sort the data by num_deliveries to ensure the x-axis is in order
                    algo_data = algo_data.sort_values(by='num_deliveries')

                    if algo_data.empty:
                        print(f"No data for algorithm {algorithm} with the given filter")
                        continue

                    plt.errorbar(
                        algo_data['num_deliveries'],
                        algo_data[metric_avg],
                        yerr=algo_data[metric_std],
                        label=f'{algorithm_str[algorithm]}',
                        fmt='-o',
                        capsize=5
                    )

                # Labels and title
                plt.xlabel('n', fontsize=9)
                plt.ylabel(metric_label, fontsize=9)
                plt.xticks(NUM_DELIVERIES_VEC, fontsize=9)
                plt.yticks(fontsize=9)
                plt.legend(
                    title="",
                    bbox_to_anchor=(0.5, 1.025),
                    loc='lower center',
                    ncol=4,
                    fontsize=9
                )

                # Save the plot to the "plots" folder
                plot_filename = f"plots/{metric_label.lower().replace(' ', '_')}_vs_deliveries_maxw{max_weight}_load{drone_load}_batt{drone_battery}.pdf"
                plt.tight_layout(rect=[0, 0, 1, 1.025])

                # Check if plot has data before saving
                if not plt.gca().has_data():
                    print(f"No data to plot for {metric_label} (max_weight={max_weight}, drone_load={drone_load}, drone_battery={drone_battery})")
                    plt.close()
                    continue

                plt.savefig(plot_filename)
                plt.close()

                print(f"Plot saved to {plot_filename}")
