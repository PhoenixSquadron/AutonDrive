# PFL AutonDrive: UGV [![CI](https://github.com/bitcraze/bitcraze-website/workflows/CI/badge.svg)](https://github.com/PhoenixSquadron/AutonDrive/actions)

## Overview
The proposed vehicle is an unmanned ground vehicle (UGV) designed for autonomous operation in indoor and outdoor logistics applications. Powered by an electric drivetrain and advanced sensing and computing capabilities, the UGV aims to automate material handling and fulfillment tasks currently performed manually throughout industrial warehouses.

## Technical Details: 
The UGV features an advanced sensing and computing suite to enable autonomous navigation. At its core is a 4-wheel electric drivetrain powered by standard lithium-ion batteries, providing an expected operational range of up to 8 hours on a single charge. An Intel RealSense D455 depth camera is mounted on the front to provide RGB and depth images for obstacle detection. Additionally, a 360-degree LiDAR sensor with a 12-meter detection range spins continuously to generate a precise 3D point cloud map of the vehicle's surroundings. Short-range ultrasonic sensors are placed around the perimeter to detect close-range objects. A high-definition camera mounted on top offers a bird's eye view for the operator. An onboard Nvidia Jetson Nano computer powers the autonomous stack, processing data from the diverse sensors in real time. It runs algorithms for simultaneous localization and mapping (SLAM), path planning, and obstacle avoidance. Wireless connectivity is enabled via WiFi to allow the UGV to communicate with a central cloud-based fleet management system. This sends commands to the vehicle and receives telemetry data like location updates, diagnostic reports, and video footage from its operations.

## Application Scenario: 
Potential applications include material transport within warehouses and between facilities, item retrieval, and delivery, and autonomous valet deliveries to pick-up/drop-off zones. The UGV aims to make logistics operations more efficient, flexible, and scalable through automation.

## Functionalities

-	Remote operation mode: Enables teleoperation via a console for testing, demonstrations, and special maneuvers.
-	Autonomous navigation: Uses sensor fusion and SLAM to autonomously follow pre-programmed paths between waypoints indoors and outdoors.
-	Obstacle detection & avoidance: Stops and navigates safely around static and dynamic obstacles detected using the 3D sensors.
-	Status sharing: Shares real-time location, diagnostics, and video footage with fleet operators via the cloud platform.

## Project Flow Chart ( UpToDateVersion)
<img width="468" alt="image" src="https://github.com/PhoenixSquadron/AutonDrive/assets/82762631/fa33bef9-929d-4422-a99c-44f69b2ebd0c">
<img width="1050" alt="Screenshot 2024-01-20 at 12 36 09 AM" src="https://github.com/PhoenixSquadron/AutonDrive/assets/82762631/1a87c9a4-648b-480b-a5dd-dfc570fde640">
<img width="1049" alt="Screenshot 2024-01-20 at 12 39 04 AM" src="https://github.com/PhoenixSquadron/AutonDrive/assets/82762631/021697a8-3540-4c79-b7d6-481662372b84">

***

## Meet the Team: 

Lead Development Team: 
- Steve Yin
- Marcus Chu
- Shreyas Sharma

Student Researchers: 
- Ellen Zhao
- Simon Jang
- Warren He

Special Thanks to: 
- Caleb Hsu: Fabrication & Materials
- Lucas Jeong: Creative Visualization Lead

Mentors: 
- Dr. Peter Tong
- Mr. Jake Stephens

