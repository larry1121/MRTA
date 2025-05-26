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
    // Clear path cache for robots that no longer exist
    for (auto it = path_cache.begin(); it != path_cache.end(); /* no increment */) {
        int robot_id = it->first;
        bool robot_exists = false;
        for(const auto& r_ptr : robots) {
            if (r_ptr->id == robot_id) {
                robot_exists = true;
                break;
            }
        }
        if (!robot_exists) {
            it = path_cache.erase(it);
            continue;
        }

        for (auto inner_it = it->second.begin(); inner_it != it->second.end(); /* no increment */) {
            bool invalidate_this_cached_path = false;
            const PathInfo& cached_p_info = inner_it->second;
            for (const auto& coord_in_path : cached_p_info.coordinates) {
                if (updated_coords.count(coord_in_path)) {
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

    // Check if there are any newly discovered tasks
    bool has_new_tasks = false;
    for (const auto& coord : updated_coords) {
        if (static_cast<int>(known_object_map[coord.x][coord.y]) & static_cast<int>(OBJECT::TASK)) {
            has_new_tasks = true;
            break;
        }
    }

    // Only calculate costs and perform task assignment if there are new tasks
    if (has_new_tasks) {
        task_total_costs.clear();
        int map_size = known_cost_map.size();

        // Create a set of already assigned task IDs
        std::set<int> assigned_task_ids;
        for (const auto& task_ptr : active_tasks) {
            if (task_ptr->get_assigned_robot_id() != -1) {
                assigned_task_ids.insert(task_ptr->id);
            }
        }

        for (const auto& robot_ptr : robots) {
            const ROBOT& robot = *robot_ptr;
            if (robot.get_status() == ROBOT::STATUS::EXHAUSTED) continue;

            // Invalidate current path if robot deviated
            if (robot_current_paths.count(robot.id)) {
                const auto& current_path = robot_current_paths[robot.id];
                if (!current_path.coordinates.empty() && robot.get_coord() != current_path.coordinates.front()) {
                    robot_current_paths.erase(robot.id);
                    robot_target_task_id.erase(robot.id);
                    if(!current_path.coordinates.empty()) {
                        Coord old_target = current_path.coordinates.back();
                        if(path_cache.count(robot.id)) {
                            path_cache[robot.id].erase(old_target);
                        }
                    }
                }
            }

            for (const auto& task_ptr : active_tasks) {
                const TASK& task = *task_ptr;
                if (task.is_done()) continue;

                // Skip if task is already assigned to another robot
                if (assigned_task_ids.count(task.id) && task.get_assigned_robot_id() != robot.id) {
                    continue;
                }

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
                    if (robot.get_energy() >= cached_path.cost + task_execution_cost) {
                        path_c = cached_path.cost;
                        path_actions = cached_path.actions;
                        path_coords = cached_path.coordinates;
                        found_in_cache = true;
                    } else {
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
                        if (!found_in_cache) {
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

        // Perform task assignment only when new tasks are discovered
        perform_task_assignment(robots, active_tasks, known_object_map);
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

    if (robot.get_energy() >= task_cost && task.get_assigned_robot_id() == -1) {
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
                robot_current_paths.erase(robot.id);
                robot_target_task_id.erase(robot.id);
            }
            return next_action;
        } else {
            // Path is invalid (empty, or robot deviated). Clear it.
            robot_current_paths.erase(robot.id);
            robot_target_task_id.erase(robot.id);
        }
    }

    // If robot already has an assigned task, continue with that task
    if (robotToTask.count(robot.id)) {
        int assigned_task_id = robotToTask[robot.id];
        // Find the assigned task
        for (const auto& task_ptr : active_tasks) {
            if (task_ptr->id == assigned_task_id) {
                // If task is still valid and not done
                if (!task_ptr->is_done() && task_ptr->get_assigned_robot_id() == robot.id) {
                    // Check if we have a valid path to the task
                    if (path_cache.count(robot.id) && path_cache.at(robot.id).count(task_ptr->coord)) {
                        const PathInfo& chosen_path = path_cache.at(robot.id).at(task_ptr->coord);
                        if (!chosen_path.actions.empty()) {
                            robot_current_paths[robot.id] = chosen_path;
                            robot_target_task_id[robot.id] = assigned_task_id;
                            return chosen_path.actions.front();
                        }
                    }
                }
                // If task is done or invalid, clear the assignment
                robotToTask.erase(robot.id);
                break;
            }
        }
    }

    // Create a set of already assigned task IDs
    std::set<int> assigned_task_ids;
    for (const auto& task_ptr : active_tasks) {
        if (task_ptr->get_assigned_robot_id() != -1) {
            assigned_task_ids.insert(task_ptr->id);
        }
    }

    int best_task_id = -1;
    int min_total_cost = std::numeric_limits<int>::max();
    Coord best_task_coord;

    if (task_total_costs.count(robot.id)) {
        for (const auto& entry : task_total_costs.at(robot.id)) {
            int task_id = entry.first;
            int total_cost = entry.second;
            
            // Skip if task is already assigned to another robot
            if (assigned_task_ids.count(task_id)) {
                continue;
            }
            
            if (total_cost == std::numeric_limits<int>::max()) continue;

            bool task_is_available_and_valid = false;
            Coord current_task_coord;
            for(const auto& t_ptr : active_tasks) {
                if (t_ptr->id == task_id) {
                    if (!t_ptr->is_done() && t_ptr->get_assigned_robot_id() == -1) {
                        task_is_available_and_valid = true;
                        current_task_coord = t_ptr->coord;
                    }
                    break;
                }
            }
            if (!task_is_available_and_valid) continue;
            
            if (path_cache.count(robot.id) && path_cache.at(robot.id).count(current_task_coord)) {
                const PathInfo& p_info = path_cache.at(robot.id).at(current_task_coord);
                int task_exec_cost = 0;
                for(const auto& t_ptr : active_tasks) {
                    if(t_ptr->id == task_id) {
                        task_exec_cost = t_ptr->get_cost(robot.type);
                        break;
                    }
                }

                if (robot.get_energy() >= p_info.cost + task_exec_cost) {
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
            const PathInfo& chosen_path = path_cache.at(robot.id).at(best_task_coord);
            if (!chosen_path.actions.empty()) {
                robot_current_paths[robot.id] = chosen_path;
                robot_target_task_id[robot.id] = best_task_id;
                robotToTask[robot.id] = best_task_id;
                return chosen_path.actions.front();
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
                                        const vector<vector<OBJECT>>& known_object_map)
{
    if (active_tasks.empty() || robots.empty() || task_total_costs.empty()) {
        return;
    }

    std::map<int, bool> robot_newly_assigned_map;
    std::map<int, bool> task_newly_assigned_map;

    if (current_algorithm == Algorithm::MIN_MIN) {
        bool assignment_made;
        do {
            assignment_made = false;
            
            // Step 1: For each task, find the robot with minimum completion time
            std::map<int, std::pair<int, int>> task_min_robot; // task_id -> {robot_id, completion_time}
            
            for (const auto& task_ptr : active_tasks) {
                const TASK& task = *task_ptr;
                if (!is_task_available(task, task_newly_assigned_map)) continue;
                
                int min_completion_time = std::numeric_limits<int>::max();
                int best_robot_id = -1;
                
                for (const auto& robot_ptr : robots) {
                    const ROBOT& robot = *robot_ptr;
                    if (!is_robot_available(robot, robot_newly_assigned_map)) continue;
                    
                    if (!can_assign_task_to_robot(robot.id, task.id, robot_newly_assigned_map, task_newly_assigned_map)) {
                        continue;
                    }
                    
                    int completion_time = task_total_costs.at(robot.id).at(task.id);
                    if (completion_time < min_completion_time) {
                        min_completion_time = completion_time;
                        best_robot_id = robot.id;
                    }
                }
                
                if (best_robot_id != -1) {
                    task_min_robot[task.id] = {best_robot_id, min_completion_time};
                }
            }
            
            // Step 2: Among all tasks, find the one with minimum completion time
            int min_task_id = -1;
            int min_task_completion_time = std::numeric_limits<int>::max();
            
            for (const auto& [task_id, robot_info] : task_min_robot) {
                if (robot_info.second < min_task_completion_time) {
                    min_task_completion_time = robot_info.second;
                    min_task_id = task_id;
                }
            }
            
            // Step 3: Assign the task with minimum completion time
            if (min_task_id != -1) {
                int assigned_robot_id = task_min_robot[min_task_id].first;
                
                // Double check if assignment is still valid
                if (!can_assign_task_to_robot(assigned_robot_id, min_task_id,
                                            robot_newly_assigned_map, task_newly_assigned_map)) {
                    continue;
                }
                
                // Find the task coordinates
                Coord task_coord;
                for (const auto& task_ptr : active_tasks) {
                    if (task_ptr->id == min_task_id) {
                        task_coord = task_ptr->coord;
                        break;
                    }
                }
                
                // Make the assignment
                robotToTask[assigned_robot_id] = min_task_id;
                robot_newly_assigned_map[assigned_robot_id] = true;
                
                // Mark task as done immediately
                mark_task_as_done(min_task_id, task_newly_assigned_map, active_tasks);
                
                // Update path for the assigned robot
                if (path_cache.count(assigned_robot_id) && path_cache.at(assigned_robot_id).count(task_coord)) {
                    const PathInfo& chosen_path = path_cache.at(assigned_robot_id).at(task_coord);
                    if (!chosen_path.actions.empty()) {
                        robot_current_paths[assigned_robot_id] = chosen_path;
                        assignment_made = true;
                    } else {
                        // If path is invalid, undo the assignment
                        robotToTask.erase(assigned_robot_id);
                        robot_newly_assigned_map.erase(assigned_robot_id);
                        task_newly_assigned_map.erase(min_task_id);
                    }
                } else {
                    // If no path exists, undo the assignment
                    robotToTask.erase(assigned_robot_id);
                    robot_newly_assigned_map.erase(assigned_robot_id);
                    task_newly_assigned_map.erase(min_task_id);
                }
            }
        } while (assignment_made);
        
    } else if (current_algorithm == Algorithm::SUFFERAGE) {
        std::map<int, std::map<int, int>> sufferage_values; // task_id -> robot_id -> sufferage value
        
        // Calculate sufferage values for each task
        for (const auto& task_ptr : active_tasks) {
            const TASK& task = *task_ptr;
            if (!is_task_available(task, task_newly_assigned_map)) continue;
            
            int min_cost = std::numeric_limits<int>::max();
            int second_min_cost = std::numeric_limits<int>::max();
            int best_robot_id = -1;
            
            for (const auto& robot_ptr : robots) {
                const ROBOT& robot = *robot_ptr;
                if (!is_robot_available(robot, robot_newly_assigned_map)) continue;
                
                if (!can_assign_task_to_robot(robot.id, task.id, robot_newly_assigned_map, task_newly_assigned_map)) {
                    continue;
                }
                
                int cost = task_total_costs.at(robot.id).at(task.id);
                if (cost < min_cost) {
                    second_min_cost = min_cost;
                    min_cost = cost;
                    best_robot_id = robot.id;
                } else if (cost < second_min_cost) {
                    second_min_cost = cost;
                }
            }
            
            if (best_robot_id != -1 && second_min_cost != std::numeric_limits<int>::max()) {
                sufferage_values[task.id][best_robot_id] = second_min_cost - min_cost;
            }
        }
        
        // Assign tasks based on highest sufferage values
        while (!sufferage_values.empty()) {
            int max_sufferage = -1;
            int best_task_id = -1;
            int best_robot_id = -1;
            
            // Find task with highest sufferage value
            for (const auto& task_entry : sufferage_values) {
                for (const auto& robot_entry : task_entry.second) {
                    if (robot_entry.second > max_sufferage) {
                        max_sufferage = robot_entry.second;
                        best_task_id = task_entry.first;
                        best_robot_id = robot_entry.first;
                    }
                }
            }
            
            if (best_task_id == -1 || best_robot_id == -1) break;
            
            // Double check if assignment is still valid
            if (!can_assign_task_to_robot(best_robot_id, best_task_id,
                                        robot_newly_assigned_map, task_newly_assigned_map)) {
                sufferage_values.erase(best_task_id);
                continue;
            }
            
            // Make the assignment
            robotToTask[best_robot_id] = best_task_id;
            robot_newly_assigned_map[best_robot_id] = true;
            
            // Mark task as done immediately
            mark_task_as_done(best_task_id, task_newly_assigned_map, active_tasks);
            
            // Update paths
            for (const auto& task_ptr : active_tasks) {
                if (task_ptr->id == best_task_id) {
                    if (path_cache.count(best_robot_id) && path_cache.at(best_robot_id).count(task_ptr->coord)) {
                        const PathInfo& chosen_path = path_cache.at(best_robot_id).at(task_ptr->coord);
                        if (!chosen_path.actions.empty()) {
                            robot_current_paths[best_robot_id] = chosen_path;
                        } else {
                            robotToTask.erase(best_robot_id);
                            robot_newly_assigned_map.erase(best_robot_id);
                            task_newly_assigned_map.erase(best_task_id);
                        }
                    } else {
                        robotToTask.erase(best_robot_id);
                        robot_newly_assigned_map.erase(best_robot_id);
                        task_newly_assigned_map.erase(best_task_id);
                    }
                    break;
                }
            }
            
            // Remove assigned task from sufferage values
            sufferage_values.erase(best_task_id);
            
            // Remove assigned robot from all other tasks' sufferage values
            for (auto& task_entry : sufferage_values) {
                task_entry.second.erase(best_robot_id);
            }
        }
    } else {
        // OLB algorithm implementation
        std::vector<std::pair<int, int>> available_robots; // {robot_id, current_energy}
        
        // Collect available robots and their current energy
        for (const auto& robot_ptr : robots) {
            const ROBOT& robot = *robot_ptr;
            if (!is_robot_available(robot, robot_newly_assigned_map)) continue;
            
            available_robots.push_back({robot.id, robot.get_energy()});
        }
        
        // Sort robots by their current energy (descending)
        std::sort(available_robots.begin(), available_robots.end(),
                 [](const auto& a, const auto& b) { return a.second > b.second; });
        
        // For each available robot, find the closest task
        for (const auto& [robot_id, energy] : available_robots) {
            if (!task_total_costs.count(robot_id)) continue;
            
            const auto& costs_for_this_robot = task_total_costs.at(robot_id);
            int best_task_id = -1;
            int min_cost = std::numeric_limits<int>::max();
            Coord best_task_coord;
            
            // Find the closest task for this robot
            for (const auto& task_ptr : active_tasks) {
                const TASK& task = *task_ptr;
                if (!is_task_available(task, task_newly_assigned_map)) continue;
                
                if (!can_assign_task_to_robot(robot_id, task.id, robot_newly_assigned_map, task_newly_assigned_map)) {
                    continue;
                }
                
                int current_cost = costs_for_this_robot.at(task.id);
                if (current_cost < min_cost) {
                    min_cost = current_cost;
                    best_task_id = task.id;
                    best_task_coord = task.coord;
                }
            }
            
            // Assign the task if found
            if (best_task_id != -1) {
                // Double check if assignment is still valid
                if (!can_assign_task_to_robot(robot_id, best_task_id,
                                            robot_newly_assigned_map, task_newly_assigned_map)) {
                    continue;
                }
                
                robotToTask[robot_id] = best_task_id;
                robot_newly_assigned_map[robot_id] = true;
                
                // Mark task as done immediately
                mark_task_as_done(best_task_id, task_newly_assigned_map, active_tasks);
                
                // Update path for the assigned robot
                if (path_cache.count(robot_id) && path_cache.at(robot_id).count(best_task_coord)) {
                    const PathInfo& chosen_path = path_cache.at(robot_id).at(best_task_coord);
                    if (!chosen_path.actions.empty()) {
                        robot_current_paths[robot_id] = chosen_path;
                    } else {
                        robotToTask.erase(robot_id);
                        robot_newly_assigned_map.erase(robot_id);
                        task_newly_assigned_map.erase(best_task_id);
                    }
                } else {
                    robotToTask.erase(robot_id);
                    robot_newly_assigned_map.erase(robot_id);
                    task_newly_assigned_map.erase(best_task_id);
                }
            }
        }
    }
}
