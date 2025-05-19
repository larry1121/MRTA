#include "schedular.h"
#include <cstdlib> // For rand() in the original idle_action if needed
#include <set>     // For std::set in priority queue for Dijkstra (custom comparator)
#include <iostream> // Required for std::cout in print_task_total_costs_table
#include <iomanip>  // Required for std::setw in print_task_total_costs_table

// Helper for Dijkstra:
// Using std::map for dist and parent for simplicity with Coord keys.
// std::priority_queue typically needs a custom comparator for structs or pairs.
// We'll store pairs of <cost, Coord> and use std::greater to make it a min-priority queue.

// Comparison for Coord to be used in std::map or std::set if needed.
// Coord already has operator< defined in simulator.h

int Scheduler::dijkstra(const Coord& start,
                        const Coord& goal,
                        const ROBOT& robot,
                        const int task_cost_for_robot,
                        const vector<vector<vector<int>>>& known_cost_map,
                        const vector<vector<OBJECT>>& known_object_map,
                        int map_size, // Assuming map_size = known_cost_map.size()
                        std::vector<ROBOT::ACTION>& out_path_actions,
                        std::vector<Coord>& out_path_coords)
{
    out_path_actions.clear();
    out_path_coords.clear();

    if (start == goal) {
        out_path_coords.push_back(start);
        return 0; // No cost if already at the goal
    }
    
    if (map_size <= 0 || known_cost_map.empty() || (known_cost_map[0].empty() && map_size >0) ) {
        return std::numeric_limits<int>::max(); 
    }

    std::map<Coord, int> dist;
    std::map<Coord, Coord> parent_coord;
    std::map<Coord, ROBOT::ACTION> parent_action;

    std::priority_queue<std::pair<int, Coord>,
                        std::vector<std::pair<int, Coord>>,
                        std::greater<std::pair<int, Coord>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    const ROBOT::TYPE robot_type = robot.type;
    const int initial_robot_energy = robot.get_energy();

    Coord current_coord_pq; // Renamed to avoid conflict with other current_coord variables if any
    int current_cost_pq;

    while (!pq.empty()) {
        current_cost_pq = pq.top().first;
        current_coord_pq = pq.top().second;
        pq.pop();

        if (dist.count(current_coord_pq) && current_cost_pq > dist.at(current_coord_pq)) { // Use .at() for safety after check
            continue;
        }

        if (current_coord_pq == goal) { 
            Coord backtrack_coord = goal;
            while (backtrack_coord != start) {
                if (!parent_coord.count(backtrack_coord)) { 
                    return std::numeric_limits<int>::max(); 
                }
                out_path_actions.push_back(parent_action.at(backtrack_coord));
                out_path_coords.push_back(backtrack_coord);
                backtrack_coord = parent_coord.at(backtrack_coord);
            }
            out_path_coords.push_back(start); 
            std::reverse(out_path_actions.begin(), out_path_actions.end());
            std::reverse(out_path_coords.begin(), out_path_coords.end());
            return current_cost_pq; 
        }

        ROBOT::ACTION all_actions[] = {ROBOT::ACTION::UP, ROBOT::ACTION::DOWN, ROBOT::ACTION::LEFT, ROBOT::ACTION::RIGHT};
        for (ROBOT::ACTION action : all_actions) {
            Coord delta = action_to_delta(action);
            Coord next_coord = current_coord_pq + delta;

            if (next_coord.x < 0 || next_coord.x >= map_size || next_coord.y < 0 || next_coord.y >= map_size) {
                continue; 
            }
            if (known_object_map.at(next_coord.x).at(next_coord.y) == OBJECT::WALL) {
                continue; 
            }
            size_t robot_type_idx = static_cast<size_t>(robot_type);
            if (robot_type_idx >= known_cost_map.at(next_coord.x).at(next_coord.y).size() ||
                known_cost_map.at(next_coord.x).at(next_coord.y).at(robot_type_idx) == -1 ) {
                continue;
            }

            int edge_cost = Scheduler::calculate_move_cost(current_coord_pq, next_coord, robot_type, known_cost_map);

            if (edge_cost == INFINITE) {
                continue;
            }
            
            int new_cost = current_cost_pq + edge_cost;
            
            // Energy pruning conditions from problem statement
            if (task_cost_for_robot != INFINITE) { // Only prune if task_cost is known (not for pure pathfinding)
                 if (new_cost >= initial_robot_energy - task_cost_for_robot) { // Original condition: dist >= robot.energy - taskCost
                    if (next_coord == goal) { // If it's the goal, this cost is fine if it's exactly energy - task_cost
                        if (new_cost > initial_robot_energy - task_cost_for_robot) continue; // Strictly more, then prune
                    } else {
                        continue; // Not the goal, and already at the energy limit for path part
                    }
                }
            } else { // Pure pathfinding, no task cost to consider for this specific pruning rule
                 if (new_cost >= initial_robot_energy && next_coord != goal) { // Path cost alone exceeds total energy
                    continue;
                }
            }
            // General check: total energy for path + task must not exceed initial energy IF it's the goal
            if (next_coord == goal && task_cost_for_robot != INFINITE && (new_cost + task_cost_for_robot > initial_robot_energy) ){
                continue;
            }
             // General check: path cost must not exceed initial energy
            if (new_cost > initial_robot_energy) {
                 continue;
            }

            if (!dist.count(next_coord) || new_cost < dist.at(next_coord)) {
                dist[next_coord] = new_cost;
                parent_coord[next_coord] = current_coord_pq;
                parent_action[next_coord] = action;
                pq.push({new_cost, next_coord});
            }
        }
    }
    return std::numeric_limits<int>::max(); 
}

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    task_total_costs.clear();
    int map_size = known_cost_map.size(); 

    for (auto it = path_cache.begin(); it != path_cache.end(); /* no increment */) {
        int robot_id = it->first;
        bool robot_exists = false;
        for(const auto& r_ptr : robots) {
            if (r_ptr->id == robot_id) {
                robot_exists = true;
                break;
            }
        }
        if (!robot_exists) { // Robot might have been destroyed or is no longer relevant
            it = path_cache.erase(it);
            continue;
        }

        for (auto inner_it = it->second.begin(); inner_it != it->second.end(); /* no increment */) {
            bool invalidate_this_cached_path = false;
            const PathInfo& cached_p_info = inner_it->second;
            for (const auto& coord_in_path : cached_p_info.coordinates) {
                if (updated_coords.count(coord_in_path)) {
                    // A more sophisticated check: has the object at coord_in_path or its cost changed significantly?
                    // For now, any update on the path invalidates it.
                    invalidate_this_cached_path = true;
                    break;
                }
            }
            if (invalidate_this_cached_path) {
                inner_it = it->second.erase(inner_it);
            } else {
                ++inner_it;
            }
        }
        if (it->second.empty()) {
            it = path_cache.erase(it);
        } else {
            ++it;
        }
    }

    for (const auto& robot_ptr : robots) {
        const ROBOT& robot = *robot_ptr;
        if (robot.get_status() == ROBOT::STATUS::EXHAUSTED) continue;

        // Invalidate current path if robot deviated
        if (robot_current_paths.count(robot.id)) {
            const auto& current_path = robot_current_paths[robot.id];
            if (!current_path.coordinates.empty() && robot.get_coord() != current_path.coordinates.front()) {
                 // If the robot is not at the expected start of the remaining path segment
                 robot_current_paths.erase(robot.id);
                 robot_target_task_id.erase(robot.id);
                 // Also clear its general cache to this specific target as it might be based on old start pos
                 if(!current_path.coordinates.empty()){ // If path was not empty
                    Coord old_target = current_path.coordinates.back();
                    if(path_cache.count(robot.id)){
                        path_cache[robot.id].erase(old_target);
                    }
                 }
            }
        }

        for (const auto& task_ptr : active_tasks) {
            const TASK& task = *task_ptr;
            if (task.is_done()) continue;
            // Allow recalculation even if assigned, in case current robot is better or assignment changes
            // if (task.get_assigned_robot_id() != -1 && task.get_assigned_robot_id() != robot.id) continue;

            std::vector<ROBOT::ACTION> path_actions;
            std::vector<Coord> path_coords;
            int task_execution_cost = task.get_cost(robot.type);

            if (task_execution_cost == INFINITE) {
                 task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                 continue;
            }

            int path_c = std::numeric_limits<int>::max();
            bool found_in_cache = false;

            if (path_cache.count(robot.id) && path_cache[robot.id].count(task.coord)) {
                const PathInfo& cached_path = path_cache[robot.id][task.coord];
                 // Check if robot has enough energy for this cached path + task
                if (robot.get_energy() >= cached_path.cost + task_execution_cost) {
                    path_c = cached_path.cost;
                    path_actions = cached_path.actions; // Store for potential use later
                    path_coords = cached_path.coordinates;
                    found_in_cache = true;
                } else {
                    // Not enough energy for cached path, remove it or mark as unusable for current energy
                    path_cache[robot.id].erase(task.coord);
                }
            }

            if (!found_in_cache) {
                 path_c = dijkstra(robot.get_coord(), task.coord, robot, task_execution_cost,
                                    known_cost_map, known_object_map, map_size,
                                    path_actions, path_coords);
            }

            if (path_c != std::numeric_limits<int>::max()) {
                if (robot.get_energy() >= path_c + task_execution_cost) {
                    task_total_costs[robot.id][task.id] = path_c + task_execution_cost;
                    if (!found_in_cache) { // Only update cache if Dijkstra was run and successful
                         path_cache[robot.id][task.coord] = PathInfo(path_actions, path_c, path_coords);
                    }
                } else {
                    task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max(); 
                }
            } else {
                task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max(); 
            }
        }
    }
}

bool Scheduler::on_task_reached(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots,
                                const ROBOT &robot,
                                const TASK &task)
{
    // Robot is at task.coord, clear its current path tracking for this task.
    if(robot_target_task_id.count(robot.id) && robot_target_task_id[robot.id] == task.id) {
        robot_current_paths.erase(robot.id);
        robot_target_task_id.erase(robot.id);
    }

    if (robot.type == ROBOT::TYPE::DRONE) { 
        return false;
    }
    int task_cost = task.get_cost(robot.type);
    if (task_cost == INFINITE) return false; 

    if (robot.get_energy() >= task_cost) {
        return true;
    }
    return false;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    if (robot.get_status() != ROBOT::STATUS::IDLE) { 
        return ROBOT::ACTION::HOLD;
    }

    // Continue current path if one exists and is valid
    if (robot_current_paths.count(robot.id)) {
        PathInfo& current_path = robot_current_paths.at(robot.id);
        // Ensure the path is not empty and the robot is at the expected location for the next step
        if (!current_path.actions.empty() && !current_path.coordinates.empty() && robot.get_coord() == current_path.coordinates.front()) {
            ROBOT::ACTION next_action = current_path.actions.front();
            current_path.actions.erase(current_path.actions.begin());
            current_path.coordinates.erase(current_path.coordinates.begin()); 
            
            if (current_path.actions.empty()) { // Path segment consumed
                // If this was the last action, actual arrival is handled by simulator calling on_task_reached.
                // If actions are empty, the robot_current_paths entry for this robot might be cleared then, or when a new task is chosen.
            }
            return next_action;
        } else {
            // Path is invalid (empty, or robot deviated). Clear it.
            robot_current_paths.erase(robot.id);
            robot_target_task_id.erase(robot.id);
        }
    }

    int best_task_id = -1;
    int min_total_cost = std::numeric_limits<int>::max();
    Coord best_task_coord;

    if (task_total_costs.count(robot.id)) {
        for (const auto& entry : task_total_costs.at(robot.id)) { // C++14 compatible
            int task_id = entry.first;
            int total_cost = entry.second;
            if (total_cost == std::numeric_limits<int>::max()) continue;

            bool task_is_available_and_valid = false;
            Coord current_task_coord;
            for(const auto& t_ptr : active_tasks) {
                if (t_ptr->id == task_id) {
                    if (!t_ptr->is_done() && (t_ptr->get_assigned_robot_id() == -1 || t_ptr->get_assigned_robot_id() == robot.id)) {
                        task_is_available_and_valid = true;
                        current_task_coord = t_ptr->coord;
                    }
                    break;
                }
            }
            if (!task_is_available_and_valid) continue;
            
            // Path should exist in general cache if total_cost is valid.
            // And energy check was done during on_info_updated for this total_cost.
            if (path_cache.count(robot.id) && path_cache.at(robot.id).count(current_task_coord)){
                 const PathInfo& p_info = path_cache.at(robot.id).at(current_task_coord);
                 int task_exec_cost = 0;
                 for(const auto& t_ptr : active_tasks){ if(t_ptr->id == task_id) {task_exec_cost = t_ptr->get_cost(robot.type); break;}}

                 if (robot.get_energy() >= p_info.cost + task_exec_cost) { // Double check energy with current
                    if (total_cost < min_total_cost) {
                        min_total_cost = total_cost;
                        best_task_id = task_id;
                        best_task_coord = current_task_coord;
                    }
                 }
            }
        }
    }

    if (best_task_id != -1) {
        if (path_cache.count(robot.id) && path_cache.at(robot.id).count(best_task_coord)) {
            const PathInfo& chosen_path_info = path_cache.at(robot.id).at(best_task_coord);
            if (!chosen_path_info.actions.empty()) {
                robot_current_paths[robot.id] = chosen_path_info; 
                robot_target_task_id[robot.id] = best_task_id;

                PathInfo& path_to_follow = robot_current_paths.at(robot.id);
                ROBOT::ACTION next_action = path_to_follow.actions.front();
                path_to_follow.actions.erase(path_to_follow.actions.begin());
                path_to_follow.coordinates.erase(path_to_follow.coordinates.begin());
                return next_action;
            }
        }
    }

    return ROBOT::ACTION::HOLD;
}

void Scheduler::print_task_total_costs_table(const vector<shared_ptr<ROBOT>>& all_robots, const vector<shared_ptr<TASK>>& all_tasks) const
{
    std::cout << "\n--- Task Total Costs Table (Robot_ID -> Task_ID -> Total Cost) ---" << std::endl;

    if (task_total_costs.empty()) {
        std::cout << "Table is currently empty." << std::endl;
        return;
    }

    for (const auto& robot_entry : task_total_costs) {
        int robot_id = robot_entry.first;
        const auto& task_map_for_robot = robot_entry.second;

        if (task_map_for_robot.empty()) {
            // std::cout << "Robot ID: " << robot_id << " has no task cost entries." << std::endl;
            continue; // Skip robots with no task entries for cleaner output
        }

        std::cout << "Robot ID: " << robot_id << std::endl;
        std::cout << "  Task ID | Total Cost" << std::endl;
        std::cout << "  --------------------" << std::endl;

        for (const auto& task_entry : task_map_for_robot) {
            int task_id = task_entry.first;
            int total_cost = task_entry.second;

            std::cout << "  " << std::setw(7) << task_id << " | ";
            if (total_cost == std::numeric_limits<int>::max()) {
                std::cout << "INF (or N/A)" << std::endl;
            } else {
                std::cout << std::setw(10) << total_cost << std::endl;
            }
        }
        std::cout << std::endl; // Add a blank line between robots for readability
    }
    std::cout << "--------------------------------------------------------------------" << std::endl;
}

// Helper to check if map is fully revealed
bool Scheduler::is_map_fully_revealed(const vector<vector<OBJECT>>& known_object_map) const {
    if (known_object_map.empty()) return false;
    for (const auto& row : known_object_map) {
        for (const auto& cell : row) {
            if (cell == OBJECT::UNKNOWN) {
                return false;
            }
        }
    }
    return true;
}

void Scheduler::perform_task_assignment(const vector<shared_ptr<ROBOT>>& robots,
                                        const vector<shared_ptr<TASK>>& active_tasks,
                                        const vector<vector<OBJECT>>& known_object_map) // known_cost_map and map_size are implicitly available via this->task_total_costs or members
{
    if (active_tasks.empty() || robots.empty() || task_total_costs.empty()) {
        return;
    }

    std::vector<bool> robot_assigned_in_this_round(robots.size() + 10, false); // Max robot ID + buffer, or use a map
    std::vector<bool> task_assigned_in_this_round(active_tasks.size() + 10, false); // Max task ID + buffer, or use a map
    // A safer way for direct indexing if IDs are not 0-N: map ID to index or use std::map<int, bool>
    std::map<int, bool> robot_newly_assigned_map;
    std::map<int, bool> task_newly_assigned_map;

    int assignments_made_this_iteration;
    do {
        assignments_made_this_iteration = 0;
        int best_overall_robot_id = -1;
        int best_overall_task_id = -1;
        Coord best_overall_task_coord;
        int min_overall_cost = std::numeric_limits<int>::max();

        for (const auto& robot_ptr : robots) {
            const ROBOT& robot = *robot_ptr;
            if (robot.type == ROBOT::TYPE::DRONE || robot.get_status() == ROBOT::STATUS::EXHAUSTED) continue;
            if (robotToTask.count(robot.id) || robot_newly_assigned_map.count(robot.id)) continue; // Already has a task or assigned this round

            if (!task_total_costs.count(robot.id)) continue; // No cost entries for this robot

            const auto& costs_for_this_robot = task_total_costs.at(robot.id);
            
            for (const auto& task_ptr_inner : active_tasks) {
                const TASK& task = *task_ptr_inner;
                if (task.is_done() || task_newly_assigned_map.count(task.id)) continue; // Task done or assigned this round
                
                // Check if task is already assigned to *another* robot from a previous persistent assignment
                bool already_globally_assigned_to_another = false;
                for(const auto& entry : robotToTask){
                    if(entry.second == task.id && entry.first != robot.id){
                        already_globally_assigned_to_another = true;
                        break;
                    }
                }
                if(already_globally_assigned_to_another) continue;


                if (!costs_for_this_robot.count(task.id)) continue; // No cost entry for this robot-task pair

                int current_total_cost = costs_for_this_robot.at(task.id);

                if (current_total_cost < min_overall_cost) {
                    // Check if robot has energy for path + task (already embedded in task_total_costs if calculated correctly)
                    // task_total_costs[robot.id][task.id] should be max_int if not enough energy
                    if (current_total_cost != std::numeric_limits<int>::max()) {
                        min_overall_cost = current_total_cost;
                        best_overall_robot_id = robot.id;
                        best_overall_task_id = task.id;
                        best_overall_task_coord = task.coord;
                    }
                }
            }
        }

        if (best_overall_robot_id != -1 && best_overall_task_id != -1) {
            robotToTask[best_overall_robot_id] = best_overall_task_id;
            robot_newly_assigned_map[best_overall_robot_id] = true;
            task_newly_assigned_map[best_overall_task_id] = true;
            assignments_made_this_iteration++;

            // Set current path for the assigned robot
            if (path_cache.count(best_overall_robot_id) && path_cache.at(best_overall_robot_id).count(best_overall_task_coord)) {
                const PathInfo& chosen_path = path_cache.at(best_overall_robot_id).at(best_overall_task_coord);
                if (!chosen_path.actions.empty()) {
                    robot_current_paths[best_overall_robot_id] = chosen_path;
                    // The first action will be popped by idle_action
                } else {
                     robotToTask.erase(best_overall_robot_id); // Path is empty, cannot assign
                     robot_newly_assigned_map.erase(best_overall_robot_id);
                     task_newly_assigned_map.erase(best_overall_task_id);
                     assignments_made_this_iteration--;
                }
            } else {
                // No path found in cache for this assignment, assignment is not possible
                robotToTask.erase(best_overall_robot_id);
                robot_newly_assigned_map.erase(best_overall_robot_id);
                task_newly_assigned_map.erase(best_overall_task_id);
                assignments_made_this_iteration--;
            }
        }
    } while (assignments_made_this_iteration > 0);
}