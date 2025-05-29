# results_plot.py
import csv
import matplotlib.pyplot as plt

def read_csv_xy(filename):

    xs, ys = [], []
    with open(filename, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return xs, ys

def main():
    # Filenames exported by your C program, now in data/ subdirectory
    scan_file    = 'data/lidarDataCartesian.csv'
    corners_file = 'data/outputCorners.csv'

    # Load data
    x_all, y_all       = read_csv_xy(scan_file)
    x_c,   y_c         = read_csv_xy(corners_file)

    # Plot
    plt.figure(figsize=(8,8))
    # All scan points in blue
    plt.scatter(x_all, y_all, s=1, color='blue', label='Lidar scan')
    # Detected corners in red
    plt.scatter(x_c, y_c,   s=50, color='red', marker='o', label='Detected corners')

    # Labels & legend
    plt.xlabel('X [mm]')
    plt.ylabel('Y [mm]')
    plt.title('LiDAR Scan with Detected Corners')
    plt.axis('equal')
    plt.legend(loc='upper right')
    plt.grid(True)

    # Show or save
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
