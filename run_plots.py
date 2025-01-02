import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": "Helvetica",
})

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
ALGORITHMS = [1, 2, 3, 4, 5, 6, 0]

algorithm_str = {
    0: "OPT",
    1: "BIN",
    2: "KNA",
    3: "COL",
    4: "GMP",
    5: "GmE",
    6: "GMP-E",
}

# Make sure 'plots' directory exists
os.makedirs('plots', exist_ok=True)

# Set the style for the plot
sns.set_theme(style="whitegrid")

# Define the metrics to plot
metrics = [
    ('total_profit_avg', 'total_profit_std', 'Total Profit'),
    ('total_energy_avg', 'total_energy_std', 'Total Energy'),
    ('total_flights_avg', 'total_flights_std', 'Total Flights'),
]

# Iterate through each metric for combined plots
for metric_avg, metric_std, metric_label in metrics:
    fig, axes = plt.subplots(2, 4, figsize=(10, 5), sharex=True, sharey=True)

    # Flatten the axes array for easier indexing
    axes = axes.flatten()

    # Plot for each combination of parameters
    subplot_idx = 0
    for max_weight in MAX_WEIGHT_VEC:
        for drone_load in DRONE_LOAD_VEC:
            for drone_battery in DRONE_BATTERY_VEC:
                if subplot_idx >= len(axes):
                    break

                ax = axes[subplot_idx]
                subplot_idx += 1

                # Filter the dataframe for the current parameters
                filtered_df = final_df[
                    (final_df['max_weight'] == max_weight) &
                    (final_df['drone_load'] == drone_load) &
                    (final_df['drone_battery'] == drone_battery)
                    ]

                if filtered_df.empty:
                    ax.set_title(f"Empty: w={max_weight}, l={drone_load}, b={drone_battery}", fontsize=10)
                    continue

                # Plot the metric for each algorithm
                for algorithm in ALGORITHMS:
                    algo_data = filtered_df[filtered_df['algorithm'] == algorithm]
                    algo_data = algo_data.sort_values(by='num_deliveries')

                    if algo_data.empty:
                        continue

                    # Define line styles and colors based on the algorithm
                    if algorithm in [4, 5, 6]:  # Dashed for GMP, GmE, GMP-E
                        linestyle = '--'
                    else:
                        linestyle = '-'  # Solid for others

                    if algorithm == 0:  # Black solid for OPT
                        color = 'black'
                    else:
                        color = None  # Default color cycle

                    # Plot with specified styles
                    ax.errorbar(
                        algo_data['num_deliveries'],
                        algo_data[metric_avg],
                        yerr=algo_data[metric_std],
                        label=f'{algorithm_str[algorithm]}',
                        fmt='o',  # Marker
                        capsize=3,
                        markersize=4,
                        linewidth=0.8,
                        linestyle=linestyle,
                        color=color
                    )

                ax.set_title(f"W={drone_load}, max w={max_weight}, B={drone_battery}", fontsize=10)

                # Add x-label only for the second row
                if (subplot_idx-1) // 4 == 1:
                    ax.set_xlabel('n', fontsize=9)

                # Add y-label only for the first column
                if subplot_idx % 4 == 1:
                    ax.set_ylabel(metric_label, fontsize=9)

                ax.tick_params(axis='both', which='major', labelsize=8)

    # Add a single legend to the figure
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=8, fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    # Save the plot
    plot_filename = f"plots/{metric_label.lower().replace(' ', '_')}_combined.pdf"
    plt.savefig(plot_filename)
    plt.close()

    print(f"Combined plot saved to {plot_filename}")
