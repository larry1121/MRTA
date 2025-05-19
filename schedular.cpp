
#include "schedular.h"
#include <cstdlib> // For rand() in the original idle_action if needed
#include <set>     // For std::set in priority queue for Dijkstra (custom comparator)

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
    
    // Check if map_size is valid
    if (map_size <= 0 || known_cost_map.empty() || known_cost_map[0].empty()) {
        return std::numeric_limits<int>::max(); // Invalid map data
    }


    std::map<Coord, int> dist;
    std::map<Coord, Coord> parent_coord;
    std::map<Coord, ROBOT::ACTION> parent_action;

    // Min-priority queue: stores {cost, coordinate}
    std::priority_queue<std::pair<int, Coord>,
                        std::vector<std::pair<int, Coord>>,
                        std::greater<std::pair<int, Coord>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    const ROBOT::TYPE robot_type = robot.type;
    const int initial_robot_energy = robot.get_energy();

    Coord current_coord;
    int current_cost;

    while (!pq.empty()) {
        current_cost = pq.top().first;
        current_coord = pq.top().second;
        pq.pop();

        if (dist.count(current_coord) && current_cost > dist[current_coord]) {
            continue;
        }

        if (current_coord == goal) { // Goal reached
            Coord backtrack_coord = goal;
            while (backtrack_coord != start) {
                if (!parent_coord.count(backtrack_coord)) { 
                    return std::numeric_limits<int>::max(); 
                }
                out_path_actions.push_back(parent_action[backtrack_coord]);
                out_path_coords.push_back(backtrack_coord);
                backtrack_coord = parent_coord[backtrack_coord];
            }
            out_path_coords.push_back(start); 
            std::reverse(out_path_actions.begin(), out_path_actions.end());
            std::reverse(out_path_coords.begin(), out_path_coords.end());
            return current_cost; 
        }

        ROBOT::ACTION all_actions[] = {ROBOT::ACTION::UP, ROBOT::ACTION::DOWN, ROBOT::ACTION::LEFT, ROBOT::ACTION::RIGHT};
        for (ROBOT::ACTION action : all_actions) {
            Coord delta = action_to_delta(action);
            Coord next_coord = current_coord + delta;

            if (next_coord.x < 0 || next_coord.x >= map_size || next_coord.y < 0 || next_coord.y >= map_size) {
                continue; 
            }
            if (known_object_map[next_coord.x][next_coord.y] == OBJECT::WALL) {
                continue; 
            }
            if (static_cast<size_t>(static_cast<int>(robot_type)) >= known_cost_map[next_coord.x][next_coord.y].size() ||
                known_cost_map[next_coord.x][next_coord.y][static_cast<int>(robot_type)] == -1 ) {
                continue;
            }

            int edge_cost = Scheduler::calculate_move_cost(current_coord, next_coord, robot_type, known_cost_map);

            if (edge_cost == INFINITE) {
                continue;
            }
            
            int new_cost = current_cost + edge_cost;

            if (new_cost > initial_robot_energy - task_cost_for_robot && next_coord != goal) {
                 continue;
            }
             if (new_cost >= initial_robot_energy && next_coord != goal) { // Path cost alone exceeds total energy
                continue;
            }
            if (new_cost + task_cost_for_robot > initial_robot_energy && next_coord == goal) { // Cannot complete task even if goal is reached
                continue;
            }


            if (!dist.count(next_coord) || new_cost < dist[next_coord]) {
                dist[next_coord] = new_cost;
                parent_coord[next_coord] = current_coord;
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