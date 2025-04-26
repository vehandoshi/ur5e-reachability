# Data Visualisation
import pandas as pd
import matplotlib.pyplot as plt

def visualize_plane(csv_file, plane_x):
    # Load the CSV file
    df = pd.read_csv(csv_file)

    # Setup plot
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title(f"Reachability on x = {plane_x:.2f} plane")
    ax.set_xlabel("Y-axis (m)")
    ax.set_ylabel("Z-axis (m)")
    ax.set_aspect('equal')

    # Plot points
    for _, row in df.iterrows():
        color = 'green' if row['reachable'] == 1 else 'red'
        ax.plot(row['y'], row['z'], 's', color=color, markersize=4)

    plt.grid(True)
    plt.tight_layout()
    plt.show()

visualize_plane("reachability_xn0.95.csv", plane_x=-0.95)