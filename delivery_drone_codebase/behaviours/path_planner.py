#!/usr/bin/env python3
import math
from itertools import permutations

class PathPlanner:
    """
    Traveling Salesman Problem solver for drone waypoint optimization.
    Finds the shortest path visiting all waypoints starting and ending at home.
    """
    
    def haversine_distance(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates in meters."""
        R = 6371000  # Earth radius in meters
        
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return R * c
    
    
    def calculate_path_distance(waypoints, home_lat, home_lon):
        """
        Calculate total distance of a path starting from home.
        waypoints: list of [id, lat, lon]
        Returns: total distance in meters
        """
        total_distance = 0
        current_lat, current_lon = home_lat, home_lon
        
        for waypoint in waypoints:
            _, lat, lon = waypoint
            total_distance += PathPlanner.haversine_distance(
                current_lat, current_lon, lat, lon
            )
            current_lat, current_lon = lat, lon
        
        # Return to home
        total_distance += PathPlanner.haversine_distance(
            current_lat, current_lon, home_lat, home_lon
        )
        
        return total_distance
    
    
    def find_shortest_path(waypoints, home_lat, home_lon):
        """
        Find the shortest path through all waypoints using brute force TSP.
        
        Args:
            waypoints: list of [id, lat, lon] - the geotags to visit
            home_lat: home latitude
            home_lon: home longitude
            
        Returns:
            tuple: (optimal_path, total_distance)
                optimal_path: reordered list of waypoints
                total_distance: total distance in meters
        """
        if len(waypoints) == 0:
            return [], 0
        
        if len(waypoints) == 1:
            distance = PathPlanner.calculate_path_distance(waypoints, home_lat, home_lon)
            return waypoints
        
        # For small number of waypoints (≤10), brute force is acceptable
        best_path = None
        best_distance = float('inf')
        
        # Try all permutations
        for perm in permutations(waypoints):
            distance = PathPlanner.calculate_path_distance(perm, home_lat, home_lon)
            if distance < best_distance:
                best_distance = distance
                best_path = list(perm)
        print(best_path)
        
        return best_path 
    
    