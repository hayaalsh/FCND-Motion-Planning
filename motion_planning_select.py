import argparse
import time
import msgpack
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from enum import Enum, auto

import numpy as np
import re

from planning_utils import a_star, heuristic, create_grid, prune_path, onclick
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, global_goal_position=None):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.global_goal_position = global_goal_position
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        header = open('colliders.csv').readline()
        s = re.findall(r"[-+]?\d*\.\d+|\d+", header)
        lat0 = float(s[1])
        lon0 = float(s[3])
        print(f'Home lat : {lat0}, lon : {lon0}')

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        print('current global home: north = %.2f, east = %0.2f, down = %0.2f' % (self.global_position[0],self.global_position[1],self.global_position[2]))

        # TODO: convert to current local position using global_to_local()
        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # plot grid with start and goal point
        fig = plt.figure(figsize=(5,6)) 
        plt.imshow(grid, origin='lower') 

        global coords
        coords = []

        def onclick(event):
            x = int(np.round(event.xdata))
            y = int(np.round(event.ydata))

            global grid_start
            global grid_goal
            coords.append((y, x))

            if len(coords) == 1:
                x, y = coords[0]
                plt.plot(y, x, 'bo')
                fig.canvas.draw()
                grid_start = (x, y)
                print('Select from the graph a destination point ...')

            if len(coords) == 2:
                fig.canvas.mpl_disconnect(cid)
                x, y = coords[1]
                plt.plot(y, x, 'r+')
                fig.canvas.draw()
                grid_goal = (x, y)
                plt.pause(0.1)
                # Run A* to find a path from start to goal
                # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
                print('Local Start and Goal: ', grid_start, grid_goal)
                print('A* and path prunning ...')
                path, _ = a_star(grid, heuristic, grid_start, grid_goal)
                # TODO: prune path to minimize number of waypoints
                path = prune_path(path)
                print('Path Count: {len(path):3.0f}')

                # plot path
                print('Plotting the path ...')
                waypoints = np.array([[p[0], p[1] , TARGET_ALTITUDE, 0] for p in path])
                plt.plot( waypoints[:,1], waypoints[:, 0], 'b')
                fig.canvas.draw()
                
                print('Check the expected fly path. The simulator will run the path shortly.')
                plt.pause(0.1)
                plt.close(1)
                # Convert path to waypoints
                waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
                # Set self.waypoints
                self.waypoints = waypoints
                # TODO: send waypoints to sim (this is just for visualization of waypoints)
                self.send_waypoints()
                
            return 
        # select start and destination
        print('Select from the graph a starting point ...')
        cid = plt.gcf().canvas.mpl_connect('button_press_event', onclick)
        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()
        

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--lon_goal', type=float, default=-150, help="goal longitude")
    parser.add_argument('--lat_goal', type=float, default=50,   help="goal latitude")
    parser.add_argument('--alt_goal', type=float, default=50,   help="goal altitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    global_goal_position = np.array([args.lon_goal, args.lat_goal, args.alt_goal], dtype='Float64')
    drone = MotionPlanning(conn, global_goal_position=global_goal_position)
    time.sleep(1)

    drone.start()
