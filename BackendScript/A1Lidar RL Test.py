import numpy as np
from rplidar import RPLidar
import matplotlib.pyplot as plt

# Connect to the lidar
lidar = RPLidar(port='/dev/ttyUSB0', baudrate=115200)

# Create empty obstacle map
map_size = 200
map_data = np.ones((map_size, map_size))

# Open a figure for the map plot
fig = plt.figure()
ax = fig.add_subplot(111)

# Scan loop
while True:

    # Get the scan data
    scan_data = lidar.iter_scans(max_buf_meas=1024)

    # Iterate through each scan measurement
    for meas in scan_data:

        # Convert angles and distances to x,y pixel coordinates
        angle = meas[2] / 180.0 * np.pi
        dist = meas[3]
        x = int(dist * np.cos(angle) * 100) + map_size // 2
        y = int(dist * np.sin(angle) * 100) + map_size // 2

        # Mark obstacles on the map
        if x >= 0 and x < map_size and y >= 0 and y < map_size:
            map_data[y, x] = 0

    # Clear the plot
    ax.clear()

    # Plot the obstacle map
    ax.imshow(map_data, extent=[0, map_size, 0, map_size], cmap='gray_r')

    # Update the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
