from objects.waypoint import Waypoint
from .RNDF import s, graph  # Import Map Information (Segment, Velocity, Graph dependencies)


class Mission:
    def __init__(self):
        self.waypoints = []
        self.current = 0
        self.min_distance_to_target = float('Inf')

    # Find the shortest path according to the minimum number of waypoints
    def find_shortest_path(self, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if start not in graph:
            return None
        shortest = None
        for node in graph[start]:
            if node not in path:
                new_path = self.find_shortest_path(node, end, path)
                if new_path:
                    if not shortest or len(new_path) < len(shortest):
                        shortest = new_path
        return shortest

    # Find the closest waypoint to any starting point
    def find_closest_waypoint(self, x, y):
        short_dist = 5000

        for i in s:
            for j in i:
                xi = j[0]
                yi = j[1]
                distx = abs(x - xi)
                disty = abs(y - yi)
                dist = distx + disty

                if dist < short_dist:
                    short_dist = dist
                    selected_x = xi
                    selected_y = yi

        return (selected_x, selected_y)

    def update(self, vehicle):

        if not self.exists():
            return []

        # Safe guard in case there is a call to update when the mission is over
        if self.target_waypoint().is_checkpoint():
            return self.target_waypoint()

        # Focus must remain on STOP until the vehicle is still
        if self.target_waypoint().is_stop() and not vehicle.is_still():
            return self.target_waypoint()

        tolerance = 1  # meter

        distance_to_waypoint = self.target_waypoint().distance(vehicle.pose)

        # if the distance_to_waypoint is not monotonically decreasing
        # then I probably missed a waypoint
        if distance_to_waypoint > self.min_distance_to_target+tolerance:
            self.miss_counter += self.miss_counter

            if self.miss_counter > 2:
                pass  # notice the Mission Planner

            return self.next_waypoint()  # or query mission planner for a new path

        self.miss_counter = 0

        if distance_to_waypoint < self.min_distance_to_target:
            self.min_distance_to_target = distance_to_waypoint

        max_radius = 3  # meters
        min_radius = 2  # meters

        # the radius by which I determine whether or not the objects has passed
        # through a waypoint depends on how fit the waypoints are given
        # by the Mission Planner.

        fitting = self.get_fitting()

        radius = min(fitting/3, max_radius)
        radius = max(radius, min_radius)

        # if I'm close enough to the next waypoint
        # I consider I passed through it
        if distance_to_waypoint < radius:
            return self.next_waypoint()
        else:
            return self.target_waypoint()

    def get_fitting(self):

        if not self.waypoints:
            return 0

        fitting = float('Inf')

        if self.current < len(self.waypoints)-2:
            fitting = self.waypoints[self.current+1].distance(self.waypoints[self.current+2])

        return fitting

    def get_heading(self):
        target = self.target_waypoint()
        next_point = self.peak_next_waypoint()

        if next_point is None:
            next_point = target

        heading = target.direction(next_point)

        return heading

    def next_waypoint(self):
        if not self.waypoints:
            return

        self.current += 1
        self.min_distance_to_target = float('Inf')
        return self.waypoints[self.current]

    def peak_next_waypoint(self):
        if self.waypoints and len(self.waypoints) > self.current+1:
            return self.waypoints[self.current+1]
        else:
            return None

    def target_waypoint(self):
        if self.waypoints:
            return self.waypoints[self.current]

    def current_waypoint(self):
        if self.waypoints and self.current:
            return self.waypoints[self.current-1]
        elif self.waypoints:
            return Waypoint(0,0)

    def exists(self):
        return True if self.waypoints else False

    # Construct Mission
    def set_mission(self, mission_msg):
        self.waypoints = []

        for new_waypoint in mission_msg.waypoints:
            self.add_waypoint(new_waypoint.x, new_waypoint.y, new_waypoint.type, new_waypoint.speed_limit)

        self.current = mission_msg.current

    def add_waypoint(self, x, y, point_type='road', speed_limit=10):
        self.waypoints.append(Waypoint(x, y, point_type=point_type, speed_limit=speed_limit))

    def add_entry(self, x, y, speed_limit=10):
        self.waypoints.append(Waypoint(x, y, "entry", speed_limit=speed_limit))

    def add_exit(self, x, y, speed_limit=10):
        self.waypoints.append(Waypoint(x, y, "exit", speed_limit=speed_limit))

    def add_stop(self, x, y, speed_limit=10):
        self.waypoints.append(Waypoint(x, y, "stop", speed_limit=speed_limit))

    def add_checkpoint(self, x, y, speed_limit=10):
        self.waypoints.append(Waypoint(x, y, "checkpoint", speed_limit=speed_limit))
