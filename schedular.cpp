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

        // Early termination if cost exceeds 8000
        if (current_cost_pq >= 8000) {
            return std::numeric_limits<int>::max();
        }

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
            
            // Ensure the path includes the final move to the goal
            if (!out_path_coords.empty() && out_path_coords.back() != goal) {
                out_path_coords.push_back(goal);
            }
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
                // Check if adding task cost would cause overflow
                if (new_cost > std::numeric_limits<int>::max() - task_cost_for_robot) {
                    continue;  // Skip if overflow would occur
                }
                
                int total_cost = new_cost + task_cost_for_robot;
                if (total_cost >= initial_robot_energy) { // Original condition: dist >= robot.energy - taskCost
                    if (next_coord == goal) { // If it's the goal, this cost is fine if it's exactly energy - task_cost
                        if (total_cost > initial_robot_energy) continue; // Strictly more, then prune
                    } else {
                        continue; // Not the goal, and already at the energy limit for path part
                    }
                }
            } else { // Pure pathfinding, no task cost to consider for this specific pruning rule
                if (new_cost >= initial_robot_energy && next_coord != goal) { // Path cost alone exceeds total energy
                    continue;
                }
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

void Scheduler::checkForNewTasks(const vector<shared_ptr<TASK>>& active_tasks) {
    for (const auto& task : active_tasks) {
        if (!task->is_done() && !newly_discovered_tasks.count(task->id)) {
            newly_discovered_tasks.insert(task->id);
            needs_reassignment = true;
        }
    }
}

void Scheduler::checkForCompletedTasks(const vector<shared_ptr<TASK>>& active_tasks) {
    for (const auto& task : active_tasks) {
        if (task->is_done() && !newly_completed_tasks.count(task->id)) {
            newly_completed_tasks.insert(task->id);
            needs_reassignment = true;
        }
    }
}

void Scheduler::checkForExhaustedRobots(const vector<shared_ptr<ROBOT>>& robots) {
    for (const auto& robot : robots) {
        if (robot->get_status() == ROBOT::STATUS::EXHAUSTED &&
            !newly_exhausted_robots.count(robot->id)) {
            newly_exhausted_robots.insert(robot->id);
            needs_reassignment = true;
        }
    }
}

bool Scheduler::shouldTriggerReassignment(const set<Coord>& updated_coords,
                                 const vector<shared_ptr<TASK>>& active_tasks,
                                 const vector<shared_ptr<ROBOT>>& robots) const {
    // Always trigger reassignment
    return true;
}

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    int map_size = known_cost_map.size();
    
    // Check if we should start task assignment (9 active tasks or already started)
    static bool has_started_assignments = false;
    if (!has_started_assignments && active_tasks.size() >= 10) {
        has_started_assignments = true;
    }

    // Initialize robot positions if not set
    for (const auto& robot : robots) {
        if (!robotExpectedPosition.count(robot->id)) {
            robotExpectedPosition[robot->id] = robot->get_coord();
        }
    }

    // Check for conditions that should trigger task reassignment
    checkForNewTasks(active_tasks);
    checkForCompletedTasks(active_tasks);
    checkForExhaustedRobots(robots);

    // Check for idle robots with available tasks
    bool has_idle_robots = false;
    bool has_available_tasks = false;
    bool has_new_tasks = false;
    
    for (const auto& robot : robots) {
        if (robot->get_status() == ROBOT::STATUS::IDLE &&
            robotTaskQueue[robot->id].empty()) {
            has_idle_robots = true;
            break;
        }
    }
    
    for (const auto& task : active_tasks) {
        if (!task->is_done()) {
            has_available_tasks = true;
            if (!newly_discovered_tasks.empty() || !updated_coords.empty()) {
                has_new_tasks = true;
            }
            break;
        }
    }
    
    // Only trigger reassignment if there are new tasks or map changes
    if (has_started_assignments && has_idle_robots && has_available_tasks &&
        (has_new_tasks || !updated_coords.empty())) {
        needs_reassignment = true;
    }

    // Remove completed tasks from robot queues
    for (const auto& task : active_tasks) {
        if (task->is_done()) {
            for (auto& robot_queue : robotTaskQueue) {
                std::queue<int> temp_queue;
                while (!robot_queue.second.empty()) {
                    if (robot_queue.second.front() != task->id) {
                        temp_queue.push(robot_queue.second.front());
                    }
                    robot_queue.second.pop();
                }
                robot_queue.second = temp_queue;
            }
        }
    }

    // Only calculate costs if we've started assignments and there are changes
    if (has_started_assignments && (needs_reassignment || !newly_discovered_tasks.empty() || !updated_coords.empty())) {
        // Perform task clustering only when new tasks are discovered or map is updated
        if (!newly_discovered_tasks.empty() || !updated_coords.empty()) {
            std::cout << "\n=== Task Clustering ===" << std::endl;
            // Perform task clustering
            clusterTasks(active_tasks, robots, known_cost_map, known_object_map);
            
            // Print cluster information
            for (size_t i = 0; i < task_clusters.size(); i++) {
                const auto& cluster = task_clusters[i];
                std::cout << "\nCluster " << i << ":" << std::endl;
                std::cout << "  Tasks: ";
                for (int task_id : cluster.task_ids) {
                    std::cout << task_id << " ";
                }
                std::cout << "\n  Start Task: " << cluster.start_task_id
                         << " at " << cluster.start_pos << std::endl;
                std::cout << "  End Task: " << cluster.end_task_id
                         << " at " << cluster.end_pos << std::endl;
                std::cout << "  Total Cost: " << cluster.total_cost << std::endl;
            }
        }

        std::cout << "\n=== Cost Calculation ===" << std::endl;
        // Calculate costs only for robots that need reassignment
        for (const auto& robot_ptr : robots) {
            const ROBOT& robot = *robot_ptr;
            if (robot.get_status() == ROBOT::STATUS::EXHAUSTED) continue;

            // Skip cost calculation if robot has a valid path and no map changes
            if (!updated_coords.empty() || !newly_discovered_tasks.empty() || needs_reassignment) {
                
                std::cout << "\nRobot " << robot.id << " (" << robot.type << ") at " << robotExpectedPosition[robot.id] << std::endl;
                for (const auto& task_ptr : active_tasks) {
                    const TASK& task = *task_ptr;
                    if (task.is_done()) continue;

                    std::vector<ROBOT::ACTION> path_actions;
                    std::vector<Coord> path_coords;
                    int task_execution_cost = task.get_cost(robot.type);

                    if (task_execution_cost == INFINITE) {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        std::cout << "  Task " << task.id << " at " << task.coord << ": INFINITE (task cost)" << std::endl;
                        continue;
                    }

                    int path_c = std::numeric_limits<int>::max();
                    bool found_in_cache = false;

                    // Check if we have a valid cached path
                    if (path_cache.count(robot.id) && path_cache[robot.id].count(task.coord)) {
                        const PathInfo& cached_path = path_cache[robot.id][task.coord];
                        if (robot.get_energy() >= cached_path.cost + task_execution_cost) {
                            path_c = cached_path.cost;
                            path_actions = cached_path.actions;
                            path_coords = cached_path.coordinates;
                            found_in_cache = true;
                            std::cout << "  Task " << task.id << " at " << task.coord << ": "
                                     << path_c << " (cached) + " << task_execution_cost
                                     << " = " << (path_c + task_execution_cost) << std::endl;
                        } else {
                            path_cache[robot.id].erase(task.coord);
                        }
                    }

                    // Only recalculate if necessary
                    if (!found_in_cache && (updated_coords.empty() ||
                        std::any_of(updated_coords.begin(), updated_coords.end(),
                            [&](const Coord& c) { return c == task.coord; }))) {
                        path_c = dijkstra(robotExpectedPosition[robot.id], task.coord, robot, task_execution_cost,
                                        known_cost_map, known_object_map, map_size,
                                        path_actions, path_coords);
                        
                        // Skip if path cost is too high (e.g., more than 99% of robot's energy)
                        if (path_c > robot.get_energy() * 0.99) {
                            path_c = std::numeric_limits<int>::max();
                        }
                        
                        std::cout << "  Task " << task.id << " at " << task.coord << ": "
                                 << path_c << " (new) + " << task_execution_cost
                                 << " = " << (path_c == std::numeric_limits<int>::max() ?
                                            std::numeric_limits<int>::max() :
                                            path_c + task_execution_cost) << std::endl;
                    }

                    if (path_c == std::numeric_limits<int>::max()) {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        std::cout << "  Task " << task.id << " at " << task.coord << ": INFINITE (no path or too costly)" << std::endl;
                    } else if (robot.get_energy() >= path_c + task_execution_cost) {
                        task_total_costs[robot.id][task.id] = path_c + task_execution_cost;
                        if (!found_in_cache) {
                            path_cache[robot.id][task.coord] = PathInfo(path_actions, path_c, path_coords);
                        }
                    } else {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        std::cout << "  Task " << task.id << " at " << task.coord << ": INFINITE (insufficient energy)" << std::endl;
                    }
                }
            }
        }

        // Perform task assignment if needed
        if (shouldTriggerReassignment(updated_coords, active_tasks, robots)) {
            std::cout << "\n=== Task Assignment ===" << std::endl;
#ifdef USE_MINMIN
            performMinMinAssignment(active_tasks, robots, known_cost_map, known_object_map);
#elif defined(USE_SUFFERAGE)
            performSufferageAssignment(robots, active_tasks, known_cost_map, known_object_map);
#elif defined(USE_OLB)
            performOLBAssignment(robots, active_tasks, known_cost_map, known_object_map);
#endif
            
            // Reset reassignment flags
            needs_reassignment = false;
            newly_discovered_tasks.clear();
            newly_completed_tasks.clear();
            newly_exhausted_robots.clear();
        }

        // Update robot paths based on their task queues
        for (const auto& robot : robots) {
            if (robot->get_status() == ROBOT::STATUS::EXHAUSTED) continue;

            if (!robotTaskQueue[robot->id].empty()) {
                int next_task_id = robotTaskQueue[robot->id].front();
                for (const auto& task : active_tasks) {
                    if (task->id == next_task_id) {
                        if (path_cache.count(robot->id) && path_cache[robot->id].count(task->coord)) {
                            robot_current_paths[robot->id] = path_cache[robot->id][task->coord];
                            robot_target_task_id[robot->id] = next_task_id;
                        }
                        break;
                    }
                }
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
    // Robot is at task.coord, clear its current path tracking for this task
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
        // Remove task from queue when starting work
        if (!robotTaskQueue[robot.id].empty() && robotTaskQueue[robot.id].front() == task.id) {
            robotTaskQueue[robot.id].pop();  // Remove task from queue when starting work
            updateRobotPosition(robot.id, task.coord);
            
            // If there are more tasks in the queue, prepare for the next one
            if (!robotTaskQueue[robot.id].empty()) {
                int next_task_id = robotTaskQueue[robot.id].front();
                for (const auto& next_task : active_tasks) {
                    if (next_task->id == next_task_id) {
                        // Recalculate path to next task
                        std::vector<ROBOT::ACTION> path_actions;
                        std::vector<Coord> path_coords;
                        int next_task_cost = next_task->get_cost(robot.type);
                        
                        int path_cost = dijkstra(task.coord, next_task->coord, robot, next_task_cost,
                                               known_cost_map, known_object_map,
                                               known_cost_map.size(),
                                               path_actions, path_coords);
                        
                        if (path_cost != std::numeric_limits<int>::max()) {
                            path_cache[robot.id][next_task->coord] = PathInfo(path_actions, path_cost, path_coords);
                        }
                        break;
                    }
                }
            }
        }
        if(task.get_assigned_robot_id() == -1){
            return true;
        }
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

    // Update robot's expected position to match actual position
    robotExpectedPosition[robot.id] = robot.get_coord();

    // Continue current path if one exists and is valid
    if (robot_current_paths.count(robot.id)) {
        PathInfo& current_path = robot_current_paths.at(robot.id);
        if (!current_path.actions.empty() && !current_path.coordinates.empty()) {
            // Check if robot is at the expected position for the next move
            if (robot.get_coord() == current_path.coordinates.front()) {
                ROBOT::ACTION next_action = current_path.actions.front();
                current_path.actions.erase(current_path.actions.begin());
                current_path.coordinates.erase(current_path.coordinates.begin());
                return next_action;
            } else {
                // Robot deviated from path, recalculate path to current target
                if (robot_target_task_id.count(robot.id)) {
                    int target_task_id = robot_target_task_id[robot.id];
                    for (const auto& task : active_tasks) {
                        if (task->id == target_task_id) {
                            std::vector<ROBOT::ACTION> path_actions;
                            std::vector<Coord> path_coords;
                            int task_cost = task->get_cost(robot.type);
                            int path_cost = dijkstra(robot.get_coord(), task->coord, robot, task_cost,
                                                   known_cost_map, known_object_map,
                                                   known_cost_map.size(),
                                                   path_actions, path_coords);
                            
                            if (path_cost != std::numeric_limits<int>::max()) {
                                robot_current_paths[robot.id] = PathInfo(path_actions, path_cost, path_coords);
                                ROBOT::ACTION next_action = path_actions.front();
                                robot_current_paths[robot.id].actions.erase(robot_current_paths[robot.id].actions.begin());
                                robot_current_paths[robot.id].coordinates.erase(robot_current_paths[robot.id].coordinates.begin());
                                return next_action;
                            }
                            break;
                        }
                    }
                }
                // Clear invalid path
                robot_current_paths.erase(robot.id);
                robot_target_task_id.erase(robot.id);
            }
        } else {
            // Path is empty, clear it
            robot_current_paths.erase(robot.id);
            robot_target_task_id.erase(robot.id);
        }
    }

    // If robot has tasks in queue, get the next task
    if (!robotTaskQueue[robot.id].empty()) {
        int next_task_id = robotTaskQueue[robot.id].front();
        for (const auto& task : active_tasks) {
            if (task->id == next_task_id) {
                // Always calculate new path to ensure it's valid
                std::vector<ROBOT::ACTION> path_actions;
                std::vector<Coord> path_coords;
                int task_cost = task->get_cost(robot.type);
                int path_cost = dijkstra(robot.get_coord(), task->coord, robot, task_cost,
                                       known_cost_map, known_object_map,
                                       known_cost_map.size(),
                                       path_actions, path_coords);
                
                if (path_cost != std::numeric_limits<int>::max()) {
                    // Update path cache and current path
                    path_cache[robot.id][task->coord] = PathInfo(path_actions, path_cost, path_coords);
                    robot_current_paths[robot.id] = PathInfo(path_actions, path_cost, path_coords);
                    robot_target_task_id[robot.id] = next_task_id;
                    ROBOT::ACTION next_action = path_actions.front();
                    robot_current_paths[robot.id].actions.erase(robot_current_paths[robot.id].actions.begin());
                    robot_current_paths[robot.id].coordinates.erase(robot_current_paths[robot.id].coordinates.begin());
                    return next_action;
                }
                break;
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

void Scheduler::updateRobotPosition(int robotId, const Coord& newPosition) {
    robotExpectedPosition[robotId] = newPosition;
}

void Scheduler::recalculateCostsForRobot(int robotId,
                                       const vector<shared_ptr<ROBOT>>& robots,
                                       const vector<shared_ptr<TASK>>& active_tasks,
                                       const vector<vector<vector<int>>>& known_cost_map,
                                       const vector<vector<OBJECT>>& known_object_map) {
    // Clear existing costs for this robot
    if (task_total_costs.count(robotId)) {
        task_total_costs[robotId].clear();
    }

    // Get robot's expected position
    Coord robotPos = robotExpectedPosition.count(robotId) ?
                    robotExpectedPosition[robotId] :
                    Coord(-1, -1);

    if (robotPos.x == -1) return; // Robot position not set

    // Find the robot
    auto robot_it = std::find_if(robots.begin(), robots.end(),
                                [robotId](const auto& r) { return r->id == robotId; });
    if (robot_it == robots.end()) return;

    // Recalculate costs for all tasks
    for (const auto& task_ptr : active_tasks) {
        const TASK& task = *task_ptr;
        if (task.is_done()) continue;

        std::vector<ROBOT::ACTION> path_actions;
        std::vector<Coord> path_coords;
        int task_execution_cost = task.get_cost((*robot_it)->type);

        int path_cost = dijkstra(robotPos, task.coord,
                                **robot_it,
                                task_execution_cost,
                                known_cost_map, known_object_map,
                                known_cost_map.size(),
                                path_actions, path_coords);

        if (path_cost != std::numeric_limits<int>::max()) {
            task_total_costs[robotId][task.id] = path_cost + task_execution_cost;
            path_cache[robotId][task.coord] = PathInfo(path_actions, path_cost, path_coords);
        } else {
            task_total_costs[robotId][task.id] = std::numeric_limits<int>::max();
        }
    }
}

int Scheduler::calculateTaskCompletionTime(int robotId, int taskId,
                                         const vector<shared_ptr<TASK>>& active_tasks,
                                         const vector<vector<vector<int>>>& known_cost_map,
                                         const vector<vector<OBJECT>>& known_object_map,
                                         const vector<shared_ptr<ROBOT>>& robots) {
    int currentTime = robotCurrentTaskEndTime.count(robotId) ?
                     robotCurrentTaskEndTime[robotId] : 0;

    // Find the cluster containing this task
    for (const auto& cluster : task_clusters) {
        if (std::find(cluster.task_ids.begin(), cluster.task_ids.end(), taskId) != cluster.task_ids.end()) {
            // This is the start task of the cluster
            if (taskId == cluster.start_task_id) {
                // Get robot's current position
                Coord robot_pos = robotExpectedPosition.count(robotId) ?
                                robotExpectedPosition[robotId] :
                                Coord(-1, -1);
                
                if (robot_pos.x == -1) return std::numeric_limits<int>::max();

                // Find the robot
                auto robot_it = std::find_if(robots.begin(), robots.end(),
                                           [robotId](const auto& r) { return r->id == robotId; });
                if (robot_it == robots.end()) return std::numeric_limits<int>::max();

                // Calculate path cost to cluster start
                std::vector<ROBOT::ACTION> path_actions;
                std::vector<Coord> path_coords;
                int path_cost = dijkstra(robot_pos, cluster.start_pos,
                                       **robot_it, 0,
                                       known_cost_map, known_object_map,
                                       known_cost_map.size(),
                                       path_actions, path_coords);

                if (path_cost == std::numeric_limits<int>::max()) {
                    return std::numeric_limits<int>::max();
                }

                // Add cluster's total cost (includes all task execution times and movement costs)
                return currentTime + path_cost + cluster.total_cost;
            }
        }
    }

    return std::numeric_limits<int>::max();
}

bool Scheduler::isTaskAlreadyAssigned(int taskId) const {
    // Check task queues
    for (const auto& robot_queue : robotTaskQueue) {
        std::queue<int> temp_queue = robot_queue.second;
        while (!temp_queue.empty()) {
            if (temp_queue.front() == taskId) {
                return true;
            }
            temp_queue.pop();
        }
    }
    
    // Check currently assigned tasks
    for (const auto& assignment : robotToTask) {
        if (assignment.second == taskId) {
            return true;
        }
    }
    
    return false;
}

void Scheduler::performMinMinAssignment(const vector<shared_ptr<TASK>>& active_tasks,
                                      const vector<shared_ptr<ROBOT>>& robots,
                                      const vector<vector<vector<int>>>& known_cost_map,
                                      const vector<vector<OBJECT>>& known_object_map) {
    std::cout << "\nPerforming Cluster-based Min-Min Assignment" << std::endl;
    
    // Clear existing assignments for reassignment
    for (auto& robot_queue : robotTaskQueue) {
        while (!robot_queue.second.empty()) {
            robot_queue.second.pop();
        }
    }
    robotToTask.clear();
    robotCurrentTaskEndTime.clear();

    // Create set of unassigned clusters
    std::set<int> unassigned_clusters;
    for (size_t i = 0; i < task_clusters.size(); i++) {
        unassigned_clusters.insert(i);
    }

    while (!unassigned_clusters.empty()) {
        int best_cluster_idx = -1;
        int best_robot_id = -1;
        int min_completion_time = std::numeric_limits<int>::max();

        std::cout << "\nFinding best cluster-robot pair from " << unassigned_clusters.size() << " unassigned clusters" << std::endl;

        // Find cluster-robot pair with minimum completion time
        for (int cluster_idx : unassigned_clusters) {
            const TaskCluster& cluster = task_clusters[cluster_idx];
            std::cout << "\nEvaluating Cluster " << cluster_idx << ":" << std::endl;
            std::cout << "  Tasks: ";
            for (int task_id : cluster.task_ids) {
                std::cout << task_id << " ";
            }
            std::cout << "\n  Start Task: " << cluster.start_task_id
                     << " at " << cluster.start_pos << std::endl;
            std::cout << "  End Task: " << cluster.end_task_id
                     << " at " << cluster.end_pos << std::endl;

            for (const auto& robot : robots) {
                if (robot->type == ROBOT::TYPE::DRONE ||
                    robot->get_status() == ROBOT::STATUS::EXHAUSTED) continue;

                // Get robot's current position
                Coord robot_pos = robotExpectedPosition.count(robot->id) ?
                                robotExpectedPosition[robot->id] :
                                robot->get_coord();

                // Check if robot's expected position is valid
                if (known_object_map[robot_pos.x][robot_pos.y] == OBJECT::WALL) {
                    // If expected position is invalid, use actual position
                    robot_pos = robot->get_coord();
                    robotExpectedPosition[robot->id] = robot_pos;
                }

                // Calculate path cost to both endpoints
                std::vector<ROBOT::ACTION> start_path_actions, end_path_actions;
                std::vector<Coord> start_path_coords, end_path_coords;
                int start_path_cost = dijkstra(robot_pos,
                                             cluster.start_pos,
                                             *robot,
                                             0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                             known_cost_map,
                                             known_object_map,
                                             known_cost_map.size(),
                                             start_path_actions,
                                             start_path_coords);

                int end_path_cost = dijkstra(robot_pos,
                                           cluster.end_pos,
                                           *robot,
                                           0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                           known_cost_map,
                                           known_object_map,
                                           known_cost_map.size(),
                                           end_path_actions,
                                           end_path_coords);

                // Skip if no valid path to either point
                if (start_path_cost == std::numeric_limits<int>::max() &&
                    end_path_cost == std::numeric_limits<int>::max()) {
                    continue;
                }

                // Create a modified cluster with the correct task order
                TaskCluster modified_cluster = cluster;
                int path_cost;
                
                if (start_path_cost <= end_path_cost) {
                    // Use original order (start -> end)
                    path_cost = start_path_cost;
                } else {
                    // Reverse the task order (end -> start)
                    path_cost = end_path_cost;
                    std::reverse(modified_cluster.task_ids.begin(), modified_cluster.task_ids.end());
                    modified_cluster.start_pos = cluster.end_pos;
                    modified_cluster.end_pos = cluster.start_pos;
                    modified_cluster.start_task_id = cluster.end_task_id;
                    modified_cluster.end_task_id = cluster.start_task_id;
                }
                
                // Calculate total cost including cluster's internal cost
                int total_cost;
                if (path_cost > std::numeric_limits<int>::max() - modified_cluster.total_cost) {
                    // Overflow would occur, skip this assignment
                    continue;
                }
                total_cost = path_cost + modified_cluster.total_cost;

                // Skip if insufficient energy
                int required_energy = total_cost;
                if (robot->get_energy() < required_energy) {
                    continue;
                }

                // Calculate completion time
                int completion_time = calculateTaskCompletionTime(robot->id, modified_cluster.start_task_id,
                                                               active_tasks,
                                                               known_cost_map,
                                                               known_object_map,
                                                               robots);

                // Skip if completion time is infinite
                if (completion_time == std::numeric_limits<int>::max()) continue;

                std::cout << "  Robot " << robot->id << " (" << robot->type << ") at " << robot_pos
                         << ": path cost = " << path_cost
                         << ", cluster cost = " << modified_cluster.total_cost
                         << ", total = " << total_cost
                         << ", completion time = " << completion_time << std::endl;

                // If this is a better completion time
                if (completion_time < min_completion_time) {
                    min_completion_time = completion_time;
                    best_cluster_idx = cluster_idx;
                    best_robot_id = robot->id;
                    // Store the modified cluster for later use
                    task_clusters[cluster_idx] = modified_cluster;
                    std::cout << "  -> New best assignment found!" << std::endl;
                }
            }
        }

        if (best_cluster_idx == -1) {
            std::cout << "No valid assignments possible" << std::endl;
            break;
        }

        const TaskCluster& best_cluster = task_clusters[best_cluster_idx];
        std::cout << "\nAssigning Cluster " << best_cluster_idx << " to Robot " << best_robot_id
                 << " (completion time: " << min_completion_time << ")" << std::endl;

        // Assign all tasks in the cluster to the robot
        for (int task_id : best_cluster.task_ids) {
            robotTaskQueue[best_robot_id].push(task_id);
        }
        robotToTask[best_robot_id] = best_cluster.start_task_id;
        robotCurrentTaskEndTime[best_robot_id] = min_completion_time;

        // Update robot's expected position to the end position of the cluster
        updateRobotPosition(best_robot_id, best_cluster.end_pos);
        std::cout << "  Updated Robot " << best_robot_id << "'s expected position to " << best_cluster.end_pos << std::endl;

        unassigned_clusters.erase(best_cluster_idx);
    }
}

void Scheduler::performSufferageAssignment(const vector<shared_ptr<ROBOT>>& robots,
                                         const vector<shared_ptr<TASK>>& active_tasks,
                                         const vector<vector<vector<int>>>& known_cost_map,
                                         const vector<vector<OBJECT>>& known_object_map) {
    std::cout << "\nPerforming Cluster-based Sufferage Assignment" << std::endl;
    
    // Clear existing assignments for reassignment
    for (auto& robot_queue : robotTaskQueue) {
        while (!robot_queue.second.empty()) {
            robot_queue.second.pop();
        }
    }
    robotToTask.clear();
    robotCurrentTaskEndTime.clear();

    // Create set of unassigned clusters
    std::set<int> unassigned_clusters;
    for (size_t i = 0; i < task_clusters.size(); i++) {
        unassigned_clusters.insert(i);
    }

    while (!unassigned_clusters.empty()) {
        int best_cluster_idx = -1;
        int best_robot_id = -1;
        int max_sufferage = -1;
        TaskCluster best_modified_cluster;  // Store the best modified cluster

        std::cout << "\nFinding best cluster-robot pair from " << unassigned_clusters.size() << " unassigned clusters" << std::endl;

        // Calculate sufferage for each cluster
        for (int cluster_idx : unassigned_clusters) {
            const TaskCluster& cluster = task_clusters[cluster_idx];
            std::cout << "\nEvaluating Cluster " << cluster_idx << ":" << std::endl;
            std::cout << "  Tasks: ";
            for (int task_id : cluster.task_ids) {
                std::cout << task_id << " ";
            }
            std::cout << "\n  Start Task: " << cluster.start_task_id
                     << " at " << cluster.start_pos << std::endl;
            std::cout << "  End Task: " << cluster.end_task_id
                     << " at " << cluster.end_pos << std::endl;

            int min_time = std::numeric_limits<int>::max();
            int second_min_time = std::numeric_limits<int>::max();
            int min_robot_id = -1;
            TaskCluster current_modified_cluster;  // Store the current modified cluster

            for (const auto& robot : robots) {
                if (robot->type == ROBOT::TYPE::DRONE ||
                    robot->get_status() == ROBOT::STATUS::EXHAUSTED) continue;

                // Get robot's current position
                Coord robot_pos = robotExpectedPosition.count(robot->id) ?
                                robotExpectedPosition[robot->id] :
                                robot->get_coord();

                // Check if robot's expected position is valid
                if (known_object_map[robot_pos.x][robot_pos.y] == OBJECT::WALL) {
                    robot_pos = robot->get_coord();
                    robotExpectedPosition[robot->id] = robot_pos;
                }

                // Calculate path cost to both endpoints
                std::vector<ROBOT::ACTION> start_path_actions, end_path_actions;
                std::vector<Coord> start_path_coords, end_path_coords;
                int start_path_cost = dijkstra(robot_pos,
                                             cluster.start_pos,
                                             *robot,
                                             0,
                                             known_cost_map,
                                             known_object_map,
                                             known_cost_map.size(),
                                             start_path_actions,
                                             start_path_coords);

                int end_path_cost = dijkstra(robot_pos,
                                           cluster.end_pos,
                                           *robot,
                                           0,
                                           known_cost_map,
                                           known_object_map,
                                           known_cost_map.size(),
                                           end_path_actions,
                                           end_path_coords);

                // Skip if no valid path to either point
                if (start_path_cost == std::numeric_limits<int>::max() &&
                    end_path_cost == std::numeric_limits<int>::max()) {
                    continue;
                }

                // Create a modified cluster with the correct task order
                current_modified_cluster = cluster;
                int path_cost;
                
                if (start_path_cost <= end_path_cost) {
                    path_cost = start_path_cost;
                } else {
                    path_cost = end_path_cost;
                    std::reverse(current_modified_cluster.task_ids.begin(), current_modified_cluster.task_ids.end());
                    current_modified_cluster.start_pos = cluster.end_pos;
                    current_modified_cluster.end_pos = cluster.start_pos;
                    current_modified_cluster.start_task_id = cluster.end_task_id;
                    current_modified_cluster.end_task_id = cluster.start_task_id;
                }

                // Calculate total cost for the cluster including all tasks
                int total_cluster_cost = path_cost;
                for (size_t i = 0; i < current_modified_cluster.task_ids.size(); i++) {
                    // Add task execution cost
                    for (const auto& task : active_tasks) {
                        if (task->id == current_modified_cluster.task_ids[i]) {
                            total_cluster_cost += task->get_cost(robot->type);
                            break;
                        }
                    }
                    
                    // Add path cost to next task (if not the last task)
                    if (i < current_modified_cluster.task_ids.size() - 1) {
                        const TASK* current_task = nullptr;
                        const TASK* next_task = nullptr;
                        
                        for (const auto& task : active_tasks) {
                            if (task->id == current_modified_cluster.task_ids[i]) current_task = task.get();
                            if (task->id == current_modified_cluster.task_ids[i + 1]) next_task = task.get();
                        }
                        
                        if (current_task && next_task) {
                            std::vector<ROBOT::ACTION> inter_path_actions;
                            std::vector<Coord> inter_path_coords;
                            int inter_path_cost = dijkstra(current_task->coord,
                                                         next_task->coord,
                                                         *robot,
                                                         0,
                                                         known_cost_map,
                                                         known_object_map,
                                                         known_cost_map.size(),
                                                         inter_path_actions,
                                                         inter_path_coords);
                            
                            if (inter_path_cost == std::numeric_limits<int>::max()) {
                                total_cluster_cost = std::numeric_limits<int>::max();
                                break;
                            }
                            total_cluster_cost += inter_path_cost;
                        }
                    }
                }

                // Skip if total cost is too high
                if (total_cluster_cost > robot->get_energy() * 0.8) {
                    total_cluster_cost = std::numeric_limits<int>::max();
                }

                // Calculate completion time based on total cluster cost
                int completion_time = robotCurrentTaskEndTime.count(robot->id) ?
                                    robotCurrentTaskEndTime[robot->id] + total_cluster_cost :
                                    total_cluster_cost;

                if (completion_time == std::numeric_limits<int>::max()) continue;

                std::cout << "  Robot " << robot->id << " (" << robot->type << ") at " << robot_pos
                         << ": path cost = " << path_cost
                         << ", cluster cost = " << total_cluster_cost
                         << ", completion time = " << completion_time << std::endl;

                if (completion_time < min_time) {
                    second_min_time = min_time;
                    min_time = completion_time;
                    min_robot_id = robot->id;
                    current_modified_cluster = current_modified_cluster;  // Store the best modified cluster for this robot
                } else if (completion_time < second_min_time) {
                    second_min_time = completion_time;
                }
            }

            // Calculate sufferage for this cluster
            if (min_time != std::numeric_limits<int>::max() &&
                second_min_time != std::numeric_limits<int>::max()) {
                int sufferage = second_min_time - min_time;
                if (sufferage > max_sufferage) {
                    max_sufferage = sufferage;
                    best_cluster_idx = cluster_idx;
                    best_robot_id = min_robot_id;
                    best_modified_cluster = current_modified_cluster;  // Store the best modified cluster
                    std::cout << "  -> New best assignment found! (sufferage: " << sufferage << ")" << std::endl;
                }
            }
        }

        if (best_cluster_idx == -1) {
            std::cout << "No valid assignments possible" << std::endl;
            break;
        }

        std::cout << "\nAssigning Cluster " << best_cluster_idx << " to Robot " << best_robot_id
                 << " (sufferage: " << max_sufferage << ")" << std::endl;

        // Assign all tasks in the cluster to the robot
        for (int task_id : best_modified_cluster.task_ids) {
            robotTaskQueue[best_robot_id].push(task_id);
        }
        robotToTask[best_robot_id] = best_modified_cluster.start_task_id;
        robotCurrentTaskEndTime[best_robot_id] = calculateTaskCompletionTime(best_robot_id, best_modified_cluster.start_task_id,
                                                                           active_tasks,
                                                                           known_cost_map,
                                                                           known_object_map,
                                                                           robots);

        // Update robot's expected position to the end position of the cluster
        updateRobotPosition(best_robot_id, best_modified_cluster.end_pos);
        std::cout << "  Updated Robot " << best_robot_id << "'s expected position to " << best_modified_cluster.end_pos << std::endl;

        unassigned_clusters.erase(best_cluster_idx);
    }
}

void Scheduler::performOLBAssignment(const vector<shared_ptr<ROBOT>>& robots,
                                   const vector<shared_ptr<TASK>>& active_tasks,
                                   const vector<vector<vector<int>>>& known_cost_map,
                                   const vector<vector<OBJECT>>& known_object_map) {
    std::set<int> unassigned_tasks;
    for (const auto& task : active_tasks) {
        if (!task->is_done()) {
            unassigned_tasks.insert(task->id);
        }
    }

    while (!unassigned_tasks.empty()) {
        // Find robot with minimum workload
        int min_workload_robot = -1;
        int min_workload = std::numeric_limits<int>::max();

        for (const auto& robot : robots) {
            if (robot->type == ROBOT::TYPE::DRONE ||
                robot->get_status() == ROBOT::STATUS::EXHAUSTED) continue;

            int workload = robotCurrentTaskEndTime.count(robot->id) ?
                          robotCurrentTaskEndTime[robot->id] : 0;

            if (workload < min_workload) {
                min_workload = workload;
                min_workload_robot = robot->id;
            }
        }

        if (min_workload_robot == -1) break;

        // Find closest task to the robot's current position
        int best_task_id = -1;
        int min_cost = std::numeric_limits<int>::max();

        for (int task_id : unassigned_tasks) {
            if (task_total_costs.count(min_workload_robot) &&
                task_total_costs[min_workload_robot].count(task_id)) {
                int cost = task_total_costs[min_workload_robot][task_id];
                if (cost < min_cost) {
                    min_cost = cost;
                    best_task_id = task_id;
                }
            }
        }

        if (best_task_id == -1) break;

        // Assign task to robot
        robotTaskQueue[min_workload_robot].push(best_task_id);
        robotCurrentTaskEndTime[min_workload_robot] = calculateTaskCompletionTime(min_workload_robot, best_task_id,
                                                                               active_tasks,
                                                                               known_cost_map,
                                                                               known_object_map,
                                                                               robots);

        // Update robot's expected position
        for (const auto& task : active_tasks) {
            if (task->id == best_task_id) {
                updateRobotPosition(min_workload_robot, task->coord);
                break;
            }
        }

        // Recalculate costs for the assigned robot
        recalculateCostsForRobot(min_workload_robot, robots, active_tasks,
                               known_cost_map, known_object_map);

        unassigned_tasks.erase(best_task_id);
    }
}

void Scheduler::clusterTasks(const vector<shared_ptr<TASK>>& active_tasks,
                           const vector<shared_ptr<ROBOT>>& robots,
                           const vector<vector<vector<int>>>& known_cost_map,
                           const vector<vector<OBJECT>>& known_object_map) {
    task_clusters.clear();
    
    // 아직 클러스터링되지 않은 태스크들을 추적
    set<int> unclustered_tasks;
    for (const auto& task : active_tasks) {
        if (!task->is_done()) {
            unclustered_tasks.insert(task->id);
        }
    }

    // WHEEL과 CATERPILLAR 로봇 찾기
    shared_ptr<ROBOT> wheel_robot = nullptr;
    shared_ptr<ROBOT> caterpillar_robot = nullptr;
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::WHEEL) {
            wheel_robot = robot;
        } else if (robot->type == ROBOT::TYPE::CATERPILLAR) {
            caterpillar_robot = robot;
        }
    }
    if (!wheel_robot || !caterpillar_robot) {
        std::cout << "Both WHEEL and CATERPILLAR robots are required for clustering" << std::endl;
        return;
    }

    std::cout << "Using Robot " << wheel_robot->id << " (WHEEL) and Robot "
              << caterpillar_robot->id << " (CATERPILLAR) for clustering" << std::endl;

    while (!unclustered_tasks.empty()) {
        TaskCluster new_cluster;
        int seed_task_id = *unclustered_tasks.begin();
        new_cluster.task_ids.push_back(seed_task_id);
        unclustered_tasks.erase(seed_task_id);

        // 현재 클러스터에 추가할 수 있는 태스크 찾기
        bool cluster_growing = true;
        while (cluster_growing) {
            cluster_growing = false;
            int best_task_id = -1;
            int min_cost = -1;
            if(active_tasks.size()>4){
                min_cost = CLUSTER_DISTANCE_THRESHOLD;
            }

            // 현재 클러스터의 모든 태스크에 대해
            for (int current_task_id : new_cluster.task_ids) {
                // 아직 클러스터링되지 않은 모든 태스크에 대해
                for (int candidate_id : unclustered_tasks) {
                    // 현재 태스크와 후보 태스크 사이의 비용 계산
                    const TASK* current_task = nullptr;
                    const TASK* candidate_task = nullptr;
                    
                    for (const auto& task : active_tasks) {
                        if (task->id == current_task_id) current_task = task.get();
                        if (task->id == candidate_id) candidate_task = task.get();
                    }
                    
                    if (!current_task || !candidate_task) continue;

                    std::vector<ROBOT::ACTION> wheel_path_actions, caterpillar_path_actions;
                    std::vector<Coord> wheel_path_coords, caterpillar_path_coords;
                    
                    int wheel_path_cost = dijkstra(current_task->coord,
                                                 candidate_task->coord,
                                                 *wheel_robot,
                                                 0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                                 known_cost_map,
                                                 known_object_map,
                                                 known_cost_map.size(),
                                                 wheel_path_actions,
                                                 wheel_path_coords);

                    int caterpillar_path_cost = dijkstra(current_task->coord,
                                                      candidate_task->coord,
                                                      *caterpillar_robot,
                                                      0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                                      known_cost_map,
                                                      known_object_map,
                                                      known_cost_map.size(),
                                                      caterpillar_path_actions,
                                                      caterpillar_path_coords);

                    // 두 로봇 타입 중 더 큰 비용을 사용
                    int path_cost = std::max(wheel_path_cost, caterpillar_path_cost);

                    if (path_cost < min_cost) {
                        min_cost = path_cost;
                        best_task_id = candidate_id;
                    }
                }
            }

            // 가장 가까운 태스크를 클러스터에 추가
            if (best_task_id != -1) {
                // Check size limit before adding the task
                if (new_cluster.task_ids.size() >= MAX_CLUSTER_SIZE) {
                    break;  // Stop if we've reached the maximum cluster size
                }

                new_cluster.task_ids.push_back(best_task_id);
                unclustered_tasks.erase(best_task_id);
                cluster_growing = true;
            }
        }

        // 클러스터의 시작점과 끝점 찾기
        findClusterEndpoints(new_cluster, active_tasks, robots, known_cost_map, known_object_map);
        
        // 클러스터의 총 비용 계산
        new_cluster.total_cost = 0;
        if (new_cluster.task_ids.size() > 1) {
            for (size_t i = 0; i < new_cluster.task_ids.size() - 1; i++) {
                const TASK* current_task = nullptr;
                const TASK* next_task = nullptr;
                
                for (const auto& task : active_tasks) {
                    if (task->id == new_cluster.task_ids[i]) current_task = task.get();
                    if (task->id == new_cluster.task_ids[i + 1]) next_task = task.get();
                }
                
                if (!current_task || !next_task) continue;

                std::vector<ROBOT::ACTION> wheel_path_actions, caterpillar_path_actions;
                std::vector<Coord> wheel_path_coords, caterpillar_path_coords;
                
                int wheel_path_cost = dijkstra(current_task->coord,
                                             next_task->coord,
                                             *wheel_robot,
                                             0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                             known_cost_map,
                                             known_object_map,
                                             known_cost_map.size(),
                                             wheel_path_actions,
                                             wheel_path_coords);

                int caterpillar_path_cost = dijkstra(current_task->coord,
                                                  next_task->coord,
                                                  *caterpillar_robot,
                                                  0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                                  known_cost_map,
                                                  known_object_map,
                                                  known_cost_map.size(),
                                                  caterpillar_path_actions,
                                                  caterpillar_path_coords);
                
                // 두 로봇 타입 중 더 큰 비용을 사용
                int path_cost = std::max(wheel_path_cost, caterpillar_path_cost);
                
                new_cluster.total_cost += path_cost;
            }
        } else {
            // 태스크가 하나일 경우, 두 로봇 타입 중 더 큰 실행 비용을 클러스터 비용으로 설정
            for (const auto& task : active_tasks) {
                if (task->id == new_cluster.task_ids[0]) {
                    int wheel_cost = task->get_cost(ROBOT::TYPE::WHEEL);
                    int caterpillar_cost = task->get_cost(ROBOT::TYPE::CATERPILLAR);
                    new_cluster.total_cost = std::max(wheel_cost, caterpillar_cost);
                    break;
                }
            }
        }

        task_clusters.push_back(new_cluster);
    }
}

void Scheduler::findClusterEndpoints(TaskCluster& cluster,
                                   const vector<shared_ptr<TASK>>& active_tasks,
                                   const vector<shared_ptr<ROBOT>>& robots,
                                   const vector<vector<vector<int>>>& known_cost_map,
                                   const vector<vector<OBJECT>>& known_object_map) {
    if (cluster.task_ids.empty()) return;

    // 태스크가 하나일 경우, 해당 태스크의 위치를 시작점과 끝점으로 설정
    if (cluster.task_ids.size() == 1) {
        for (const auto& task : active_tasks) {
            if (task->id == cluster.task_ids[0]) {
                cluster.start_task_id = task->id;
                cluster.end_task_id = task->id;
                cluster.start_pos = task->coord;
                cluster.end_pos = task->coord;
                return;
            }
        }
        return;
    }

    // WHEEL과 CATERPILLAR 로봇 찾기
    shared_ptr<ROBOT> wheel_robot = nullptr;
    shared_ptr<ROBOT> caterpillar_robot = nullptr;
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::WHEEL) {
            wheel_robot = robot;
        } else if (robot->type == ROBOT::TYPE::CATERPILLAR) {
            caterpillar_robot = robot;
        }
    }
    if (!wheel_robot || !caterpillar_robot) {
        std::cout << "Both WHEEL and CATERPILLAR robots are required for finding cluster endpoints" << std::endl;
        return;
    }
    
    // 모든 태스크 쌍에 대한 거리 계산
    map<pair<int, int>, int> distances;
    for (int i = 0; i < cluster.task_ids.size(); i++) {
        for (int j = i + 1; j < cluster.task_ids.size(); j++) {
            const TASK* task1 = nullptr;
            const TASK* task2 = nullptr;
            
            for (const auto& task : active_tasks) {
                if (task->id == cluster.task_ids[i]) task1 = task.get();
                if (task->id == cluster.task_ids[j]) task2 = task.get();
            }
            
            if (!task1 || !task2) continue;

            std::vector<ROBOT::ACTION> wheel_path_actions, caterpillar_path_actions;
            std::vector<Coord> wheel_path_coords, caterpillar_path_coords;
            
            int wheel_path_cost = dijkstra(task1->coord,
                                         task2->coord,
                                         *wheel_robot,
                                         0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                         known_cost_map,
                                         known_object_map,
                                         known_cost_map.size(),
                                         wheel_path_actions,
                                         wheel_path_coords);

            int caterpillar_path_cost = dijkstra(task1->coord,
                                              task2->coord,
                                              *caterpillar_robot,
                                              0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                              known_cost_map,
                                              known_object_map,
                                              known_cost_map.size(),
                                              caterpillar_path_actions,
                                              caterpillar_path_coords);
            
            // 두 로봇 타입 중 더 큰 비용을 사용
            int path_cost = std::max(wheel_path_cost, caterpillar_path_cost);
            
            distances[{cluster.task_ids[i], cluster.task_ids[j]}] = path_cost;
            distances[{cluster.task_ids[j], cluster.task_ids[i]}] = path_cost;
        }
    }

    // 가장 먼 두 태스크를 찾기
    int max_distance = -1;
    for (const auto& dist : distances) {
        if (dist.second > max_distance) {
            max_distance = dist.second;
            cluster.start_task_id = dist.first.first;
            cluster.end_task_id = dist.first.second;
        }
    }

    // 시작점과 끝점의 좌표 설정
    for (const auto& task : active_tasks) {
        if (task->id == cluster.start_task_id) {
            cluster.start_pos = task->coord;
        }
        if (task->id == cluster.end_task_id) {
            cluster.end_pos = task->coord;
        }
    }
}

int Scheduler::calculatePathCost(const Coord& start, const Coord& end,
                               const ROBOT& robot,
                               const vector<vector<vector<int>>>& known_cost_map,
                               const vector<vector<OBJECT>>& known_object_map) {
    std::vector<ROBOT::ACTION> path_actions;
    std::vector<Coord> path_coords;
    
    return dijkstra(start, end, robot, 0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                   known_cost_map, known_object_map,
                   known_cost_map.size(),
                   path_actions, path_coords);
}

