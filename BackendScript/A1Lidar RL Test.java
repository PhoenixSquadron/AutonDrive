import java.io.IOException;
import java.util.Scanner;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public class RPLIDARMap {

    public static void main(String[] args) throws IOException {
        // Create a new RPLIDAR object
        RPLidar lidar = new RPLidar("/dev/ttyUSB0");

        // Start the lidar scan
        lidar.startScan();

        // Initialize the map
        Map<Integer, Integer> map = Maps.newHashMap();

        // Main loop
        while (true) {
            // Get the latest scan data
            List<ScanData> scanData = lidar.getScanData();

            // Convert the scan data to polar coordinates
            List<Double> angles = Lists.newArrayList();
            List<Double> distances = Lists.newArrayList();

            for (ScanData data : scanData) {
                angles.add(data.getAngle());
                distances.add(data.getDistance());
            }

            // Update the map
            for (int i = 0; i < angles.size(); i++) {
                double angle = angles.get(i);
                double distance = distances.get(i);

                // Convert the angle to radians
                double radians = angle * Math.PI / 180;

                // Calculate the x and y coordinates of the point
                double x = distance * Math.cos(radians);
                double y = distance * Math.sin(radians);

                // Add the point to the map
                map.put((int) x, (int) y);
            }

            // Display the map
            System.out.println(map);
        }

        // Stop the lidar scan
        lidar.stopScan();
    }
}
