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

// Dijkstra: 밝혀지지 않은 영역(OBJECT::UNKNOWN)을 caterpillar는 398, wheel은 846으로 처리
int Scheduler::dijkstra(const Coord& start,
    const Coord& goal,
    const ROBOT& robot,
    const int task_cost_for_robot,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    int map_size,
    std::vector<ROBOT::ACTION>& out_path_actions,
    std::vector<Coord>& out_path_coords)
{
    out_path_actions.clear();
    out_path_coords.clear();

    if (start == goal) {
        out_path_coords.push_back(start);
        return 0;
    }

    if (map_size <= 0 || known_cost_map.empty() || (known_cost_map[0].empty() && map_size > 0)) {
        return std::numeric_limits<int>::max();
    }

    std::map<Coord, int> dist;
    std::map<Coord, Coord> parent_coord;
    std::map<Coord, ROBOT::ACTION> parent_action;

    std::priority_queue<std::pair<int, Coord>,
        std::vector<std::pair<int, Coord>>,
        std::greater<std::pair<int, Coord>>> pq;

    dist[start] = 0;
    pq.push({ 0, start });

    const ROBOT::TYPE robot_type = robot.type;
    const int robot_type_idx = static_cast<int>(robot_type);
    const int initial_robot_energy = robot.get_energy();

    Coord current_coord_pq;
    int current_cost_pq;

    while (!pq.empty()) {
        current_cost_pq = pq.top().first;
        current_coord_pq = pq.top().second;
        pq.pop();

        if (dist.count(current_coord_pq) && current_cost_pq > dist.at(current_coord_pq)) {
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

        ROBOT::ACTION all_actions[] = { ROBOT::ACTION::UP, ROBOT::ACTION::DOWN, ROBOT::ACTION::LEFT, ROBOT::ACTION::RIGHT };
        for (ROBOT::ACTION action : all_actions) {
            Coord delta = action_to_delta(action);
            Coord next_coord = current_coord_pq + delta;

            if (next_coord.x < 0 || next_coord.x >= map_size || next_coord.y < 0 || next_coord.y >= map_size) {
                continue;
            }

            OBJECT cell_obj = known_object_map.at(next_coord.x).at(next_coord.y);

            if (cell_obj == OBJECT::WALL) {
                continue;
            }

            int cost1, cost2;

            // 출발지 cost 계산
            if (known_object_map.at(current_coord_pq.x).at(current_coord_pq.y) == OBJECT::UNKNOWN) {
                if (robot_type == ROBOT::TYPE::CATERPILLAR)
                    cost1 = 398;
                else if (robot_type == ROBOT::TYPE::WHEEL)
                    cost1 = 846;
                else
                    cost1 = 1000000; // 기타 로봇타입일 때 적당히 큰 값(예외처리)
            }
            else {
                cost1 = known_cost_map.at(current_coord_pq.x).at(current_coord_pq.y).at(robot_type_idx);
            }

            // 도착지 cost 계산
            if (cell_obj == OBJECT::UNKNOWN) {
                if (robot_type == ROBOT::TYPE::CATERPILLAR)
                    cost2 = 398;
                else if (robot_type == ROBOT::TYPE::WHEEL)
                    cost2 = 846;
                else
                    cost2 = 1000000; // 기타 로봇타입일 때 적당히 큰 값(예외처리)
            }
            else {
                cost2 = known_cost_map.at(next_coord.x).at(next_coord.y).at(robot_type_idx);
            }

            // 만약 cost1 또는 cost2가 -1(아예 미정)이면 위와 같이 처리
            if (cost1 == -1) {
                if (robot_type == ROBOT::TYPE::CATERPILLAR)
                    cost1 = 398;
                else if (robot_type == ROBOT::TYPE::WHEEL)
                    cost1 = 846;
                else
                    cost1 = 1000000;
            }
            if (cost2 == -1) {
                if (robot_type == ROBOT::TYPE::CATERPILLAR)
                    cost2 = 398;
                else if (robot_type == ROBOT::TYPE::WHEEL)
                    cost2 = 846;
                else
                    cost2 = 1000000;
            }

            // 둘 중 하나라도 int 너무 크면 overflow 방지용 skip (필요시)
            if (cost1 >= 0x7fffffff / 2 || cost2 >= 0x7fffffff / 2) {
                continue;
            }

            int edge_cost = (cost1 + cost2) / 2;
            int new_cost = current_cost_pq + edge_cost;

            // 이하 기존 에너지, 목표 등 처리 부분
            if (task_cost_for_robot != INFINITE) {
                if (new_cost >= initial_robot_energy - task_cost_for_robot) {
                    if (next_coord == goal) {
                        if (new_cost > initial_robot_energy - task_cost_for_robot) continue;
                    }
                    else {
                        continue;
                    }
                }
            }
            else {
                if (new_cost >= initial_robot_energy && next_coord != goal) {
                    continue;
                }
            }
            if (next_coord == goal && task_cost_for_robot != INFINITE && (new_cost + task_cost_for_robot > initial_robot_energy)) {
                continue;
            }
            if (new_cost > initial_robot_energy) {
                continue;
            }
            if (!dist.count(next_coord) || new_cost < dist.at(next_coord)) {
                dist[next_coord] = new_cost;
                parent_coord[next_coord] = current_coord_pq;
                parent_action[next_coord] = action;
                pq.push({ new_cost, next_coord });
            }
        }
    }
    return std::numeric_limits<int>::max();
}
// ===== [여기까지 dijkstra 주요변경] =====

// 이하 나머지 함수 on_info_updated, on_task_reached, idle_action 등은 기존과 동일하게 둡니다.

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

void Scheduler::checkForMapChanges(const set<Coord>& updated_coords) {
    // If there are significant map changes (e.g., new walls discovered),
    // we should trigger reassignment
    if (!updated_coords.empty()) {
        needs_reassignment = true;
    }
}

bool Scheduler::shouldTriggerReassignment(const set<Coord>& updated_coords,
                                        const vector<shared_ptr<TASK>>& active_tasks,
                                        const vector<shared_ptr<ROBOT>>& robots) const {
    // Check if any of the trigger conditions are met
    return needs_reassignment;
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
    if (!has_started_assignments && active_tasks.size() >= 9) {
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
    checkForMapChanges(updated_coords);

    // Check for idle robots with available tasks
    bool has_idle_robots = false;
    bool has_available_tasks = false;
    
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
            break;
        }
    }
    
    // If we've started assignments and there are idle robots with available tasks,
    // trigger reassignment
    if (has_started_assignments && has_idle_robots && has_available_tasks) {
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

    // If new tasks are discovered, clear all task queues and costs
    if (!newly_discovered_tasks.empty()) {
        // Clear all task queues
        for (auto& robot_queue : robotTaskQueue) {
            while (!robot_queue.second.empty()) {
                robot_queue.second.pop();
            }
        }
        // Clear all costs and paths
        task_total_costs.clear();
        path_cache.clear();
        robot_current_paths.clear();
        robot_target_task_id.clear();
        // Reset robot expected positions to their current positions
        for (const auto& robot : robots) {
            robotExpectedPosition[robot->id] = robot->get_coord();
        }
    }

    // Only calculate costs if we've started assignments
    if (has_started_assignments) {
        // Calculate initial costs for all robot-task pairs
        for (const auto& robot_ptr : robots) {
            const ROBOT& robot = *robot_ptr;
            if (robot.get_status() == ROBOT::STATUS::EXHAUSTED) continue;

            for (const auto& task_ptr : active_tasks) {
                const TASK& task = *task_ptr;
                if (task.is_done()) continue;

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
                    path_c = dijkstra(robotExpectedPosition[robot.id], task.coord, robot, task_execution_cost,
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

        // Perform task assignment if needed
        if (shouldTriggerReassignment(updated_coords, active_tasks, robots)) {
#ifdef USE_MINMIN
            performMinMinAssignment(robots, active_tasks, known_cost_map, known_object_map);
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

    // Update robot's expected position to match actual position
    robotExpectedPosition[robot.id] = robot.get_coord();

    // Check if we have valid path cache
    if (isPathCacheValid(robot.id, robot.get_coord())) {
        TaskPathInfo& current_path = robot_path_cache[robot.id].front();
        if (!current_path.actions.empty() && !current_path.coordinates.empty()) {
            ROBOT::ACTION next_action = current_path.actions.front();
            current_path.actions.erase(current_path.actions.begin());
            current_path.coordinates.erase(current_path.coordinates.begin());
            
            // If this path is complete, move to next task's path
            if (current_path.actions.empty()) {
                robot_path_cache[robot.id].pop();
            }
            
            return next_action;
        }
    }

    // If no valid path cache, recalculate paths for all tasks in queue
    if (!robotTaskQueue[robot.id].empty()) {
        updatePathCache(robot.id, active_tasks, known_cost_map, known_object_map, robots);
        if (isPathCacheValid(robot.id, robot.get_coord())) {
            TaskPathInfo& current_path = robot_path_cache[robot.id].front();
            if (!current_path.actions.empty()) {
                ROBOT::ACTION next_action = current_path.actions.front();
                current_path.actions.erase(current_path.actions.begin());
                current_path.coordinates.erase(current_path.coordinates.begin());
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
                                         const vector<vector<OBJECT>>& known_object_map) {
    int currentTime = robotCurrentTaskEndTime.count(robotId) ?
                     robotCurrentTaskEndTime[robotId] : 0;

    if (!task_total_costs.count(robotId) ||
        !task_total_costs[robotId].count(taskId)) {
        return std::numeric_limits<int>::max();
    }

    return currentTime + task_total_costs[robotId][taskId];
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

void Scheduler::performMinMinAssignment(const vector<shared_ptr<ROBOT>>& robots,
                                      const vector<shared_ptr<TASK>>& active_tasks,
                                      const vector<vector<vector<int>>>& known_cost_map,
                                      const vector<vector<OBJECT>>& known_object_map) {
    // Clear existing assignments for reassignment
    for (auto& robot_queue : robotTaskQueue) {
        while (!robot_queue.second.empty()) {
            robot_queue.second.pop();
        }
    }
    robotToTask.clear();
    robotCurrentTaskEndTime.clear();

    // Create set of unassigned tasks
    std::set<int> unassigned_tasks;
    for (const auto& task : active_tasks) {
        if (!task->is_done()) {
            unassigned_tasks.insert(task->id);
        }
    }

    while (!unassigned_tasks.empty()) {
        int best_task_id = -1;
        int best_robot_id = -1;
        int min_completion_time = std::numeric_limits<int>::max();
        int max_remaining_energy = -1;

        // Find task-robot pair with minimum completion time
        for (int task_id : unassigned_tasks) {
            for (const auto& robot : robots) {
                // Skip if robot is drone, exhausted, or already has a task
                if (robot->type == ROBOT::TYPE::DRONE ||
                    robot->get_status() == ROBOT::STATUS::EXHAUSTED ||
                    !robotTaskQueue[robot->id].empty()) continue;

                // Calculate completion time (includes energy check from task_total_costs)
                int completion_time = calculateTaskCompletionTime(robot->id, task_id,
                                                               active_tasks,
                                                               known_cost_map,
                                                               known_object_map);

                // Skip if completion time is infinite (insufficient energy or no valid path)
                if (completion_time == std::numeric_limits<int>::max()) continue;

                // Also explicitly check if the robot has enough energy for the entire task including travel
                int task_execution_cost = -1;
                for(const auto& task_ptr : active_tasks) {
                    if(task_ptr->id == task_id) {
                        task_execution_cost = task_ptr->get_cost(robot->type);
                        break;
                    }
                }
                if (task_execution_cost == INFINITE) continue;

                int path_cost = task_total_costs.count(robot->id) && task_total_costs[robot->id].count(task_id)
                                ? task_total_costs[robot->id][task_id] - task_execution_cost
                                : std::numeric_limits<int>::max();

                if (path_cost == std::numeric_limits<int>::max() ||
                    robot->get_energy() < path_cost + task_execution_cost) {
                    continue;
                }

                // If this is a better completion time, or equal completion time but more remaining energy,
                // or equal completion time and energy but smaller robot ID
                if (completion_time < min_completion_time ||
                    (completion_time == min_completion_time &&
                     (robot->get_energy() > max_remaining_energy ||
                      (robot->get_energy() == max_remaining_energy &&
                       robot->id < best_robot_id)))) {
                    min_completion_time = completion_time;
                    max_remaining_energy = robot->get_energy();
                    best_task_id = task_id;
                    best_robot_id = robot->id;
                }
            }
        }

        if (best_task_id == -1) break; // No valid assignments possible

        // Assign task to robot
        robotTaskQueue[best_robot_id].push(best_task_id);
        robotToTask[best_robot_id] = best_task_id;
        robotCurrentTaskEndTime[best_robot_id] = min_completion_time;

        // Update robot's expected position
        for (const auto& task : active_tasks) {
            if (task->id == best_task_id) {
                updateRobotPosition(best_robot_id, task->coord);
                break;
            }
        }

        unassigned_tasks.erase(best_task_id);
    }
}

void Scheduler::performSufferageAssignment(const vector<shared_ptr<ROBOT>>& robots,
                                         const vector<shared_ptr<TASK>>& active_tasks,
                                         const vector<vector<vector<int>>>& known_cost_map,
                                         const vector<vector<OBJECT>>& known_object_map) {
    // Create set of unassigned tasks
    std::set<int> unassigned_tasks;
    for (const auto& task : active_tasks) {
        if (!task->is_done()) {
            unassigned_tasks.insert(task->id);
        }
    }

    while (!unassigned_tasks.empty()) {
        int best_task_id = -1;
        int best_robot_id = -1;
        int max_sufferage = -1;

        // Calculate sufferage for each task
        for (int task_id : unassigned_tasks) {
            int min_time = std::numeric_limits<int>::max();
            int second_min_time = std::numeric_limits<int>::max();
            int min_robot_id = -1;

            for (const auto& robot : robots) {
                if (robot->type == ROBOT::TYPE::DRONE ||
                    robot->get_status() == ROBOT::STATUS::EXHAUSTED) continue;

                int completion_time = calculateTaskCompletionTime(robot->id, task_id,
                                                               active_tasks,
                                                               known_cost_map,
                                                               known_object_map);

                // Skip if completion time is infinite (insufficient energy or no valid path)
                if (completion_time == std::numeric_limits<int>::max()) continue;

                // Also explicitly check if the robot has enough energy for the entire task including travel
                int task_execution_cost = -1;
                for(const auto& task_ptr : active_tasks) {
                    if(task_ptr->id == task_id) {
                        task_execution_cost = task_ptr->get_cost(robot->type);
                        break;
                    }
                }
                if (task_execution_cost == INFINITE) continue;

                int path_cost = task_total_costs.count(robot->id) && task_total_costs[robot->id].count(task_id)
                                ? task_total_costs[robot->id][task_id] - task_execution_cost
                                : std::numeric_limits<int>::max();

                if (path_cost == std::numeric_limits<int>::max() ||
                    robot->get_energy() < path_cost + task_execution_cost) {
                    continue;
                }

                if (completion_time < min_time) {
                    second_min_time = min_time;
                    min_time = completion_time;
                    min_robot_id = robot->id;
                } else if (completion_time < second_min_time) {
                    second_min_time = completion_time;
                }
            }

            // Only calculate sufferage if we found both min and second min times
            if (min_time != std::numeric_limits<int>::max() &&
                second_min_time != std::numeric_limits<int>::max()) {
                int sufferage = second_min_time - min_time;
                if (sufferage > max_sufferage) {
                    max_sufferage = sufferage;
                    best_task_id = task_id;
                    best_robot_id = min_robot_id;
                }
            }
        }

        if (best_task_id == -1) break; // No valid assignments possible

        // Assign task to robot
        robotTaskQueue[best_robot_id].push(best_task_id);
        robotToTask[best_robot_id] = best_task_id;
        robotCurrentTaskEndTime[best_robot_id] = calculateTaskCompletionTime(best_robot_id, best_task_id,
                                                                           active_tasks,
                                                                           known_cost_map,
                                                                           known_object_map);

        // Update robot's expected position
        for (const auto& task : active_tasks) {
            if (task->id == best_task_id) {
                updateRobotPosition(best_robot_id, task->coord);
                break;
            }
        }

        // Update path cache for the assigned robot
        updatePathCache(best_robot_id, active_tasks, known_cost_map, known_object_map, robots);

        // Recalculate costs for all robots after assignment
        for (const auto& robot : robots) {
            if (robot->type != ROBOT::TYPE::DRONE &&
                robot->get_status() != ROBOT::STATUS::EXHAUSTED) {
                recalculateCostsForRobot(robot->id, robots, active_tasks,
                                       known_cost_map, known_object_map);
            }
        }

        unassigned_tasks.erase(best_task_id);
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
                                                                               known_object_map);

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

void Scheduler::updatePathCache(int robot_id,
                              const std::vector<shared_ptr<TASK>>& active_tasks,
                              const vector<vector<vector<int>>>& known_cost_map,
                              const vector<vector<OBJECT>>& known_object_map,
                              const vector<shared_ptr<ROBOT>>& robots) {
    // Clear existing path cache for this robot
    clearPathCache(robot_id);

    // Get robot's current position
    Coord current_pos = robotExpectedPosition[robot_id];
    
    // For each task in the robot's queue, calculate and cache the path
    std::queue<int> temp_queue = robotTaskQueue[robot_id];
    while (!temp_queue.empty()) {
        int task_id = temp_queue.front();
        temp_queue.pop();

        // Find the task
        const TASK* task = nullptr;
        for (const auto& t : active_tasks) {
            if (t->id == task_id) {
                task = t.get();
                break;
            }
        }
        if (!task) continue;

        // Calculate path to this task
        std::vector<ROBOT::ACTION> path_actions;
        std::vector<Coord> path_coords;
        int task_cost = task->get_cost(robots[robot_id]->type);
        
        int path_cost = dijkstra(current_pos, task->coord, *robots[robot_id], task_cost,
                               known_cost_map, known_object_map,
                               known_cost_map.size(),
                               path_actions, path_coords);

        if (path_cost != std::numeric_limits<int>::max()) {
            // Cache the path
            robot_path_cache[robot_id].push(TaskPathInfo(path_actions, path_coords,
                                                       path_cost, task_cost, task->coord));
            // Update current position for next path calculation
            current_pos = task->coord;
        }
    }
}

void Scheduler::clearPathCache(int robot_id) {
    while (!robot_path_cache[robot_id].empty()) {
        robot_path_cache[robot_id].pop();
    }
}

bool Scheduler::isPathCacheValid(int robot_id, const Coord& current_pos) const {
    if (!robot_path_cache.count(robot_id) || robot_path_cache.at(robot_id).empty()) {
        return false;
    }
    
    // Check if the first path in cache starts from current position
    const TaskPathInfo& first_path = robot_path_cache.at(robot_id).front();
    if (first_path.coordinates.empty()) {
        return false;
    }
    
    return first_path.coordinates.front() == current_pos;
}
