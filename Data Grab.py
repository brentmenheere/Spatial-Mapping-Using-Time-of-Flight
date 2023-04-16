
#Written by: Brent Menheere, menheerb, 400362843
#Python Code for collecting and plotting data transmitted by ToF sensor through microcontroller
#Visualization is done on Open3D

import serial
import math
import open3d as o3d

def wait_for_scan(): #function for detecting when user wants to stop scanning
    while True:
        raw = s.readline()
        message = raw.decode().strip()
        if message == "start scanning": #flag outputs true and tells us to collect data
            return True
        elif message == "stop": #flag outputs false tells us to stop collecting data
            return False
        else:
            try:
                distance = float(message)
                return distance
            except ValueError:
                continue

def create_line_set(points): #function for connecting points in desired shape
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    lines = []

    num_planes = len(points) // 32 #32 points per plane
    for p in range(num_planes):
        for i in range(32):
            if i < points_per_plane - 1:
                lines.append([p * 32 + i, p * 32 + i + 1])
            else:
                lines.append([p * 32 + i, p * 32]) #connect the last point in the plane to the first point in the plane
            if p < num_planes - 1:  #connecting all points of a plane to all points in the next plane
                lines.append([p * 32 + i, (p + 1) * 32 + i])

        lines.append([p * 32, p * 32 + 32 - 1]) #connect the first and last points in the plane

    line_set.lines = o3d.utility.Vector2iVector(lines)
    
    return line_set

def create_point_cloud(points): #function for creating point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def visualize_point_cloud(point_cloud, line_set): #function for plotting point cloude
    o3d.visualization.draw_geometries([point_cloud, line_set])

def polar_to_cartesian(distance, angle):
    x = depth                                    #depth changes based on 'walking' incremented evert 360 frame scans
    y = distance * math.cos(math.radians(angle)) #for y coord
    z = distance * math.sin(math.radians(angle)) #for z coord
    return x, y, z


s = serial.Serial('COM4', 115200, timeout = 10)
print("Opening: " + s.name)

s.reset_output_buffer()
s.reset_input_buffer()

f = open("2dx3points.xyz", "w") #file for Open3D to use

angle_increment = 360 / 32

input("Press Enter to start communication...")
s.write('s'.encode())
points = []

i = 0
depth = 0
points_per_plane = 32 #32 points per plane

while True:

    result = wait_for_scan()
    if result == True: #if scan is still on continue
        continue
    elif result == False: #if scan is off break loop
        break
    else:
        distance = result
        angle = i * angle_increment

        x, y, z = polar_to_cartesian(distance, angle) #get xyz coords
        print(f"x: {x}, y: {y}, z: {z}")

        points.append((x, y, z))
        f.write(f"{x} {y} {z}\n")

        i += 1

        if i % 32 == 0 and i != 0: #for slices, every 360 degrees we take a new slize 2 m from the one before
            depth += 700
            print("Take a step forward")

print("Closing: " + s.name) #close ports
s.close()
f.close()

point_cloud = create_point_cloud(points) #plot the scan
line_set = create_line_set(points)
lines = line_set.lines
visualize_point_cloud(point_cloud, line_set)
