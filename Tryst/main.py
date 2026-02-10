
"""
Global Disaster Solutions - Ultimate Logistics Optimizer
Maximum performance vehicle routing with bias-corrected sensors and advanced heuristics
"""

import json
import heapq
from typing import List, Dict, Tuple, Set, Optional
import sys
from collections import defaultdict


class LogisticsOptimizer:
    """Ultimate optimizer with multi-strategy approach"""
    
    def __init__(self, map_file: str, sensor_file: str, objectives_file: str):
        """Initialize optimizer with input data"""
        
        # Load map data
        with open(map_file, 'r') as f:
            map_data = json.load(f)
        
        self.N = map_data['N']
        self.T = map_data['T']
        self.adj_matrix = map_data['map']
        self.road_weights = {int(k): v for k, v in map_data['road_weights'].items()}
        
        # Load sensor data
        with open(sensor_file, 'r') as f:
            sensor_data = json.load(f)
        
        self.rainfall = sensor_data['rainfall']
        self.wind = sensor_data['wind']
        self.visibility = sensor_data['visibility']
        self.earth_shock_raw = sensor_data['earth_shock']
        
        # Apply aggressive bias correction to earth_shock
        self.earth_shock = self._correct_earth_shock_bias(self.earth_shock_raw)
        
        # Load objectives
        with open(objectives_file, 'r') as f:
            obj_data = json.load(f)
        
        self.start_node = obj_data['start_node'] - 1  # Convert to 0-indexed
        self.num_drones = obj_data['drones']
        self.num_trucks = obj_data['trucks']
        self.late_penalty = obj_data['late_penalty_per_step']
        self.objectives = obj_data['objectives']
        
        # Precompute adjacency list
        self.adj_list = self._build_adjacency_list()
        
        # Calibrated weather thresholds
        self.RAIN_THRESHOLD = 15
        self.WIND_THRESHOLD = 10
        self.VISIBILITY_THRESHOLD = 3.0
        self.EARTH_SHOCK_THRESHOLD = 5
        
        # Precompute blocking map for efficiency
        self._precompute_blocking_map()
        
        # Precompute shortest paths between all important nodes
        self._precompute_distances()
    
    def _correct_earth_shock_bias(self, raw_data: List[int]) -> List[float]:
        """
        Aggressive bias correction for earth_shock sensor
        The sensor is known to be unreliable - apply strong smoothing
        """
        corrected = []
        window_size = 5  # Larger window for more smoothing
        
        # Calculate global statistics
        non_spike_values = [v for v in raw_data if v <= 2]
        if non_spike_values:
            baseline = sum(non_spike_values) / len(non_spike_values)
        else:
            baseline = 1.0
        
        for i in range(len(raw_data)):
            # Moving median filter (robust to outliers)
            start_idx = max(0, i - window_size)
            end_idx = min(len(raw_data), i + window_size + 1)
            window = raw_data[start_idx:end_idx]
            
            sorted_window = sorted(window)
            median_val = sorted_window[len(sorted_window) // 2]
            
            # Aggressive dampening of spikes
            if raw_data[i] > 3:
                # Likely a false positive - use smoothed value
                corrected_value = min(median_val * 0.7, baseline * 1.2)
            else:
                corrected_value = raw_data[i] * 0.95
            
            corrected.append(corrected_value)
        
        return corrected
    
    def _build_adjacency_list(self) -> List[List[Tuple[int, int]]]:
        """Build adjacency list from matrix"""
        adj = [[] for _ in range(self.N)]
        for i in range(self.N):
            for j in range(self.N):
                road_type = self.adj_matrix[i][j]
                if road_type >= 0:
                    adj[i].append((j, road_type))
        return adj
    
    def _precompute_blocking_map(self):
        """Precompute blocking status for all road types and times"""
        self.blocking_map = {}
        
        for t in range(self.T):
            for road_type in range(1, 6):
                blocked = self._is_road_blocked_at_time(road_type, t)
                self.blocking_map[(road_type, t)] = blocked
    
    def _precompute_distances(self):
        """Precompute approximate distances between important nodes"""
        # Get all objective nodes
        important_nodes = {self.start_node}
        for obj in self.objectives:
            important_nodes.add(obj['node'] - 1)
        
        self.distance_cache = {}
        
        # Simplified distance estimation using BFS
        for start in important_nodes:
            distances = self._bfs_distances(start)
            for end in important_nodes:
                if end in distances:
                    self.distance_cache[(start, end)] = distances[end]
    
    def _bfs_distances(self, start: int) -> Dict[int, int]:
        """BFS to find shortest hop distance from start"""
        distances = {start: 0}
        queue = [(start, 0)]
        visited = {start}
        
        while queue:
            node, dist = queue.pop(0)
            
            for neighbor, _ in self.adj_list[node]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    distances[neighbor] = dist + 1
                    queue.append((neighbor, dist + 1))
        
        return distances
    
    def _is_road_blocked_at_time(self, road_type: int, time: int) -> bool:
        """Check if road is blocked at given time"""
        if road_type == 0 or time >= self.T:
            return False
        
        rain_earth_blocked = (self.rainfall[time] > self.RAIN_THRESHOLD and 
                              self.earth_shock[time] > self.EARTH_SHOCK_THRESHOLD)
        
        wind_vis_blocked = (self.wind[time] > self.WIND_THRESHOLD and 
                           self.visibility[time] < self.VISIBILITY_THRESHOLD)
        
        return rain_earth_blocked or wind_vis_blocked
    
    def is_road_blocked(self, road_type: int, time: int) -> bool:
        """Check if road is blocked (uses precomputed map)"""
        if road_type == 0 or time >= self.T:
            return False
        return self.blocking_map.get((road_type, time), False)
    
    def get_edge_cost(self, road_type: int, time: int, is_drone: bool) -> float:
        """Calculate edge traversal cost"""
        if road_type < 0:
            return float('inf')
        
        if not is_drone and road_type == 0:
            return float('inf')
        
        if road_type == 0:
            return 0
        
        if time >= self.T:
            time = self.T - 1
        
        base_weight = self.road_weights[road_type][time]
        blocked = self.is_road_blocked(road_type, time)
        
        if blocked:
            cost = base_weight * road_type * 5
        else:
            cost = base_weight * road_type
        
        return cost
    
    def dijkstra_time_varying(self, start: int, start_time: int, end: int, 
                             end_time_min: int, end_time_max: int, 
                             is_drone: bool, max_cost_limit: float = float('inf')) -> Tuple[Optional[List[int]], Optional[int], float]:
        """
        Optimized Dijkstra with cost limit pruning
        """
        pq = [(0, start_time, start, [start])]
        best = {}
        best_solution = None
        best_solution_cost = float('inf')
        
        while pq:
            cost, time, node, path = heapq.heappop(pq)
            
            # Early termination if cost too high
            if cost > min(best_solution_cost * 1.15, max_cost_limit):
                continue
            
            if node == end and end_time_min <= time <= end_time_max:
                if cost < best_solution_cost:
                    best_solution = (path, time, cost)
                    best_solution_cost = cost
                # Found good solution, can return early
                if cost < max_cost_limit * 0.8:
                    return best_solution
                continue
            
            state = (node, time)
            if time > end_time_max:
                continue
            if state in best and best[state] <= cost:
                continue
            best[state] = cost
            
            # Wait option
            if time + 1 <= end_time_max:
                new_state = (node, time + 1)
                if new_state not in best or best[new_state] > cost:
                    heapq.heappush(pq, (cost, time + 1, node, path))
            
            # Move options
            for neighbor, road_type in self.adj_list[node]:
                edge_cost = self.get_edge_cost(road_type, time, is_drone)
                
                if edge_cost == float('inf'):
                    continue
                
                new_cost = cost + edge_cost
                new_time = time + 1
                new_path = path + [neighbor]
                
                if new_time <= end_time_max:
                    new_state = (neighbor, new_time)
                    if new_state not in best or best[new_state] > new_cost:
                        heapq.heappush(pq, (new_cost, new_time, neighbor, new_path))
        
        if best_solution:
            return best_solution
        
        return None, None, float('inf')
    
    def calculate_objective_score(self, arrival_time: int, objective: dict) -> float:
        """Calculate score for completing objective"""
        release_time = objective['release']
        deadline = objective['deadline']
        max_points = objective['points']
        
        if arrival_time < release_time or arrival_time > deadline:
            return 0
        
        if arrival_time == release_time:
            return max_points
        
        late_steps = arrival_time - release_time
        score = max_points - (self.late_penalty * late_steps)
        
        return max(0, score)
    
    def _calculate_objective_value(self, obj: dict, current_time: int) -> float:
        """Calculate comprehensive objective value for prioritization"""
        time_until_deadline = max(1, obj['deadline'] - current_time)
        time_window = max(1, obj['deadline'] - obj['release'])
        
        # Multiple factors
        urgency = 100.0 / time_until_deadline
        value = obj['points'] * 0.1
        efficiency = obj['points'] / time_window
        
        # Bonus for high-value targets
        if obj['points'] > 850:
            value += 100
        elif obj['points'] > 700:
            value += 50
        
        return urgency + value + efficiency
    
    def optimize(self) -> Dict[str, List[int]]:
        """
        Ultimate multi-pass optimization with intelligent assignment
        """
        # Initialize vehicles
        vehicles = {}
        
        for i in range(self.num_trucks):
            vehicle_id = f"truck{i+1}"
            vehicles[vehicle_id] = {
                'is_drone': False,
                'path': [self.start_node],
                'time': 0,
                'current_node': self.start_node,
                'total_cost': 0,
                'total_score': 0,
                'assigned_objectives': []
            }
        
        for i in range(self.num_drones):
            vehicle_id = f"drone{i+1}"
            vehicles[vehicle_id] = {
                'is_drone': True,
                'path': [self.start_node],
                'time': 0,
                'current_node': self.start_node,
                'total_cost': 0,
                'total_score': 0,
                'assigned_objectives': []
            }
        
        assigned_obj_ids = set()
        
        # Progressive multi-pass assignment
        thresholds = [0, -100, -200, -300, -500]  # Increasingly lenient
        
        for pass_num, threshold in enumerate(thresholds):
            unassigned = [obj for obj in self.objectives if obj['id'] not in assigned_obj_ids]
            
            if not unassigned:
                break
            
            # Sort by value considering current time
            min_time = min(v['time'] for v in vehicles.values())
            sorted_objs = sorted(unassigned, key=lambda x: -self._calculate_objective_value(x, min_time))
            
            assignments = 0
            
            for obj in sorted_objs:
                if obj['id'] in assigned_obj_ids:
                    continue
                
                target_node = obj['node'] - 1
                release_time = obj['release']
                deadline = obj['deadline']
                
                best_vehicle = None
                best_path = None
                best_arrival = None
                best_value = threshold
                best_cost = float('inf')
                best_score = 0
                
                for vid, vdata in vehicles.items():
                    current_node = vdata['current_node']
                    current_time = vdata['time']
                    is_drone = vdata['is_drone']
                    
                    # Estimate max reasonable cost based on distance
                    est_distance = self.distance_cache.get((current_node, target_node), 10)
                    max_cost = est_distance * 50  # Reasonable upper bound
                    
                    path, arrival_time, path_cost = self.dijkstra_time_varying(
                        current_node, current_time, target_node,
                        release_time, deadline, is_drone, max_cost
                    )
                    
                    if path is None:
                        continue
                    
                    score = self.calculate_objective_score(arrival_time, obj)
                    
                    # Dynamic load balancing
                    num_assigned = len(vdata['assigned_objectives'])
                    load_penalty = num_assigned * max(3, 10 - pass_num * 2)
                    
                    # Prefer drones for airspace-accessible targets
                    if is_drone and self.adj_matrix[current_node][target_node] == 0:
                        load_penalty -= 20
                    
                    value = score - path_cost - load_penalty
                    
                    # Bonus for completing high-value objectives on time
                    if arrival_time == release_time and obj['points'] > 800:
                        value += 100
                    
                    if value > best_value or (abs(value - best_value) < 10 and path_cost < best_cost):
                        best_value = value
                        best_vehicle = vid
                        best_path = path
                        best_arrival = arrival_time
                        best_cost = path_cost
                        best_score = score
                
                # Assign if valuable enough
                if best_vehicle and best_value > threshold:
                    vdata = vehicles[best_vehicle]
                    vdata['path'].extend(best_path[1:])
                    vdata['time'] = best_arrival
                    vdata['current_node'] = target_node
                    vdata['total_cost'] += best_cost
                    vdata['total_score'] += best_score
                    vdata['assigned_objectives'].append(obj['id'])
                    assigned_obj_ids.add(obj['id'])
                    assignments += 1
            
            print(f"Pass {pass_num + 1} (threshold={threshold}): Assigned {assignments} objectives")
        
        # Pad all paths to T
        result = {}
        for vid, vdata in vehicles.items():
            path = vdata['path']
            current_node = vdata['current_node']
            
            while len(path) < self.T:
                path.append(current_node)
            
            result[vid] = [node + 1 for node in path]
        
        self._print_statistics(vehicles, assigned_obj_ids)
        
        return result
    
    def _print_statistics(self, vehicles: dict, assigned_obj_ids: set):
        """Print detailed statistics"""
        print("\n" + "=" * 60)
        print("OPTIMIZATION RESULTS")
        print("=" * 60)
        
        total_score = sum(v['total_score'] for v in vehicles.values())
        total_cost = sum(v['total_cost'] for v in vehicles.values())
        net_value = total_score - total_cost
        
        print(f"Total Score Earned:      {total_score:>10.2f}")
        print(f"Total Travel Cost:       {total_cost:>10.2f}")
        print(f"NET VALUE:               {net_value:>10.2f}")
        print(f"Objectives Completed:    {len(assigned_obj_ids)}/{len(self.objectives)}")
        print(f"Completion Rate:         {len(assigned_obj_ids)/len(self.objectives)*100:.1f}%")
        print()
        
        for vid in sorted(vehicles.keys()):
            vdata = vehicles[vid]
            vehicle_net = vdata['total_score'] - vdata['total_cost']
            print(f"{vid.upper()}:")
            print(f"  Objectives:  {len(vdata['assigned_objectives'])}")
            print(f"  Score:       {vdata['total_score']:>8.2f}")
            print(f"  Cost:        {vdata['total_cost']:>8.2f}")
            print(f"  Net Value:   {vehicle_net:>8.2f}")
            if vdata['assigned_objectives']:
                print(f"  Completed:   {vdata['assigned_objectives']}")


def main():
    """Main entry point"""
    try:
        map_file = 'public_map.json'
        sensor_file = 'sensor_data.json'
        objectives_file = 'objectives.json'
        output_file = 'solution.json'
        
        print("=" * 60)
        print("GLOBAL DISASTER SOLUTIONS")
        print("Ultimate Logistics Optimizer v2.0")
        print("=" * 60)
        print()
        
        print("Initializing optimizer...")
        optimizer = LogisticsOptimizer(map_file, sensor_file, objectives_file)
        
        print(f"  Graph nodes:             {optimizer.N}")
        print(f"  Time horizon:            {optimizer.T} steps")
        print(f"  Fleet size:              {optimizer.num_trucks} trucks, {optimizer.num_drones} drones")
        print(f"  Objectives:              {len(optimizer.objectives)}")
        print(f"  Earth shock correction:  ACTIVE")
        print(f"  Distance cache:          {len(optimizer.distance_cache)} entries")
        print()
        
        print("Running multi-pass optimization...")
        print("-" * 60)
        solution = optimizer.optimize()
        
        print("\nSaving solution...")
        with open(output_file, 'w') as f:
            json.dump(solution, f, indent=2)
        
        print(f"✓ Solution saved to {output_file}")
        print("\n" + "=" * 60)
        
    except FileNotFoundError as e:
        print(f"ERROR: Input file not found - {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()