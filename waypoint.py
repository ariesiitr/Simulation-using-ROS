# imports
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

# some intial varaibles for the functions
a = 0
connection_string = '127.0.0.1:14550'
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def get_distance_metres(x, x1, y, y1, z, z1):

    dlat = x-x1
    dlong = y-y1
    dalt = z-z1
    return math.sqrt((dlat**2)+(dlong**2)+(dalt**2))


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint-1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    v_p = vehicle.location.global_relative_frame
    return get_distance_metres(v_p.lat, lat, v_p.lon, lon, v_p.alt, alt)


def add_mission():
    """
    takes way points from the user and then uploads them to the vehicle commands

    """
    global a
    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear()
    a = 'd'
    print(" Define/add new commands.")
    # Add new commands.
    l = True
    while(l):
        try:
            a = int(input('no of way points :- '))
            l = False
        except:
            continue
    d = []
    for i in range(a):
        p = []
        # using errror handling to prevent crash of script
        x = 0
        y = 0
        z = 0
        l = True
        while(l):
            try:
                x = float(input('x cordinates :- '))
                y = float(input('y cordinates :- '))
                z = float(input('z cordinates :- '))
                l = False
            except:
                print('retry')

        p += [x]
        p += [y]
        p += [z]
        d += [p]

    for i in d:
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, i[0], i[1], i[2]))

    print("Upload new commands to the vehicle")
    cmds.upload()


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # to stop user from trying to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Trigger just below target alt.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# starting the helicopter to fly.
arm_and_takeoff(10)
# adding missions
add_mission()

# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
print("Starting mission")
print()
print()
print("current postion")
print("x cordinate "+str(vehicle.location.global_relative_frame.lat))
print("y cordinate "+str(vehicle.location.global_relative_frame.lon))
print("z cordinate "+str(vehicle.location.global_relative_frame.alt))

# to monitor drones movemnt get its distce from the way point
while True:
    nextwaypoint = vehicle.commands.next
    print("going towards waypoint no "+str(nextwaypoint))
    print("Distance = "+str(distance_to_current_waypoint()))
    if nextwaypoint == a:
        print("Going towards final way point ")
        break
    time.sleep(2)  # to avoid too many prints on the screen


print('Return to launch')
vehicle.mode = VehicleMode("RTL")
time.sleep(15)  # to see changes on  Gazebo

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
