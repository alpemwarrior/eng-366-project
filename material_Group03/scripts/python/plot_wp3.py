#!/usr/bin/env python3

from matplotlib import pyplot as plt
import pandas as pd

def load_file(filename: str):
    '''Load a csv file into a pandas dataframe and strip spaces from column names'''
    df = pd.read_csv(filename)
    df.rename(columns=lambda x: x.strip(), inplace=True)
    return df


def plot_wp3_lights(lights_df, traj_df=None):
    '''
    Plot WP3 results: robot trajectory + detected lights with classification
    '''
    fig = plt.figure(figsize=(9, 8))
    ax = plt.subplot(1, 1, 1)

    # Plot robot trajectory if available
    if traj_df is not None and not traj_df.empty:
        ax.plot(traj_df['x'], traj_df['y'], 'b-', linewidth=2.0, label='Robot Trajectory')

    # Plot detected lights
    if not lights_df.empty:
        colors = {'good': 'green', 'flicker': 'orange', 'defect': 'red', 'unknown': 'gray'}
        
        for status in lights_df['status'].unique():
            subset = lights_df[lights_df['status'] == status]
            color = colors.get(status, 'black')
            ax.scatter(subset['x'], subset['y'], 
                      c=color, s=150, marker='*', edgecolors='black', linewidth=1.2,
                      label=f'Light - {status} ({len(subset)})')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('WP3 - Light Detection and Classification')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='upper right')

    plt.tight_layout()
    return fig


if __name__ == '__main__':
    # Load data
    lights_path = '../../controllers/controller/data/lights_data.csv'
    traj_path = '../../controllers/controller/data/trajectory.csv'

    lights = load_file(lights_path)
    print(f"Loaded {len(lights)} detected lights")
    try:
        traj = load_file(traj_path)
        print(f"Loaded trajectory with {len(traj)} points")
    except:
        traj = None
        print("Warning: trajectory.csv not found")

    # Create the plot
    fig = plot_wp3_lights(lights, traj)
    
    # Save and show
    plt.savefig('wp3_lights_plot.png', dpi=300, bbox_inches='tight')
    plt.show()

    print("Plot saved as 'wp3_lights_plot.png'")