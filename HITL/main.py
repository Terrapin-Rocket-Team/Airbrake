import os
import pandas as pd
import matplotlib.pyplot as plt

# Define directories
data_folder = 'data'
graphs_folder = 'graphs'
os.makedirs(graphs_folder, exist_ok=True)

# Columns to plot
time_col = 'State - Time (s)'

left_axis_cols = ['State - PZ (m)', 'State - Est Apo (m)', 'State - Target Apogee (m)']
right_axis_cols = ['State - VZ (m/s)', 'State - AZ (m/s/s)']
third_axis_cols = ['State - Actuation Angle (deg)', 'State - Acutal Angle (deg)']

for filename in os.listdir(data_folder):
    if filename.endswith('.csv'):
        csv_path = os.path.join(data_folder, filename)
        try:
            df = pd.read_csv(csv_path)
            print(f"Loaded {filename}")

            # Basic check for required columns
            required_cols = [time_col] + left_axis_cols + right_axis_cols + third_axis_cols
            if not all(col in df.columns for col in required_cols):
                print(f"Skipping {filename}: missing required columns.")
                continue

            time = df[time_col]

            # Compute apogee error
            max_pz = df['State - PZ (m)'].max()
            target_apogee = df['State - Target Apogee (m)'].iloc[0]
            apogee_error = max_pz - target_apogee

            # Compute amount shaved off
            max_est_apo = df['State - Est Apo (m)'].max()
            shaved_off = max_est_apo - max_pz

            # Start plotting
            fig, ax1 = plt.subplots(figsize=(12, 6))

            # Left y-axis
            for col in left_axis_cols:
                ax1.plot(time, df[col], label=col)
            ax1.set_ylabel("Altitude (m)")
            ax1.set_xlabel("Time (s)")
            ax1.grid(True)
            ax1.tick_params(axis='y')
            y_max = min(12000, df[left_axis_cols].max().max())
            ax1.set_ylim(top=y_max)
            lines1, labels1 = ax1.get_legend_handles_labels()

            # Right y-axis
            # ax2 = ax1.twinx()
            # for col in right_axis_cols:
            #     ax2.plot(time, df[col], linestyle='--', label=col)
            # ax2.set_ylabel("Velocity / Acceleration (m/s or m/s/s)")
            # ax2.tick_params(axis='y')
            # lines2, labels2 = ax2.get_legend_handles_labels()

            # Third y-axis
            ax3 = ax1.twinx()
            # ax3.spines["right"].set_position(("axes", 1.12))  # offset third axis
            for col in third_axis_cols:
                ax3.plot(time, df[col], linestyle=':', label=col)
            ax3.set_ylabel("Angle (deg)")
            ax3.tick_params(axis='y')
            lines3, labels3 = ax3.get_legend_handles_labels()

            # Combine legends
            lines = lines1 + lines3
            labels = labels1 + labels3
            # lines = lines1 + lines2 + lines3
            # labels = labels1 + labels2 + labels3
            ax1.legend(lines, labels, loc='center right', fontsize='small')

            # Apogee error text
            plt.title(f"Flight Data - {os.path.splitext(filename)[0]}")
            # Place annotation inside plot, near the top center
            y_max = df['State - PZ (m)'].max()
            y_min = df['State - PZ (m)'].min()
            y_range = y_max - y_min
            y_pos = y_max - 0.1 * y_range  # 10% below max

            x_pos = df[time_col].iloc[df['State - PZ (m)'].idxmax()]
            
            line1 = f"Apogee Error: {apogee_error:.2f} m"
            line2 = f"Shaved off: {shaved_off:.2f} m"
            ax1.text(x_pos + 0.15, y_pos, line1, fontsize=10, ha='center', va='top',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray'))
            ax1.text(x_pos + 0.15, y_pos - 0.06 * y_range, line2, fontsize=10, ha='center', va='top',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray'))

            # Save plot
            base_name = os.path.splitext(filename)[0]
            output_path = os.path.join(graphs_folder, f'{base_name}_plot.png')
            plt.title(f"Flight Data - {base_name}")
            plt.tight_layout()
            plt.savefig(output_path)
            plt.close()
            print(f"Saved graph to {output_path}")

        except Exception as e:
            print(f"Error processing {filename}: {e}")