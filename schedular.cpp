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

set<Coord> now_coords;

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
        if (current_cost_pq >= 6000) {
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


bool Scheduler::shouldTriggerReassignment(const set<Coord>& updated_coords,
                                 const vector<shared_ptr<TASK>>& active_tasks,
                                 const vector<shared_ptr<ROBOT>>& robots) const {
    // Always trigger reassignment
    if(needs_reassignment){
        return true;
    }
    return false;
}

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    ++tick_counter;                                     // 1) tick 증가
    unsigned long long now_tick = tick_counter;         // 2) 현재 tick 값을 지역변수에 저장

    /* 3) --- isDormant 람다를 먼저 선언 --- */
    auto isDormant = [&](int tid) -> bool {
        auto it = task_dormant_until.find(tid);
        return it != task_dormant_until.end() && it->second > now_tick;
        };

    int map_size = int(known_cost_map.size());
    
    bool NO_IDLE = true;
    for(const auto& robot : robots){
        if(robot->type != ROBOT::TYPE::DRONE){
            if(robot->get_status() == ROBOT::STATUS::IDLE){
                NO_IDLE = false;
            }
        }
    }
    if(NO_IDLE == true) {return;}
    needs_reassignment=false;
    // Check if we should start task assignment (9 active tasks or already started) 수정필
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

    // Check for idle robots with available tasks
    bool has_idle_robots = false;
    bool has_available_tasks = false;
    bool has_new_tasks = false;
    
    for (const auto& r : robots) {
        if (r->type == ROBOT::TYPE::DRONE) continue;              // 정찰용 제외
        if (r->get_status() != ROBOT::STATUS::IDLE) continue;     // IDLE 아니면 건너뜀

        /* 이 로봇이 수행할 수 있는 태스크가 최소 1개라도 있는가? */
        bool feasible = false;
        for (const auto& t : active_tasks) {
            if (isDormant(t->id) || t->is_done()) continue;       // 휴면·완료 태스크 skip
            int cost = t->get_cost(r->type);
            if (cost == INFINITE || cost > r->get_energy()) continue; // 불가능
            feasible = true; break;
        }
        if (feasible) { has_idle_robots = true; break; }
    }
    
    for (const auto& task : active_tasks) {
        if (!task->is_done()) {
            has_available_tasks = true;
            if (!newly_discovered_tasks.empty() || now_coords == updated_coords) {
                has_new_tasks = true;
            }
                break;
            }
        }
    now_coords = updated_coords;
    // Only trigger reassignment if there are new tasks or map changes
    if (has_started_assignments && has_idle_robots && has_available_tasks &&
        (has_new_tasks || !updated_coords.empty())) {
        needs_reassignment = true;
    }
    if(needs_reassignment == false) return;
    
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
    if (has_started_assignments && (needs_reassignment || has_new_tasks)) {
        // Perform task clustering only when new tasks are discovered or map is updated
        if (has_new_tasks) {
            std::cout << "\n=== Task Clustering ===" << std::endl;
            // Perform task clustering
            clusterTasks(active_tasks, robots, known_cost_map, known_object_map);
            
            // Print cluster information
            for (size_t i = 0; i < task_clusters.size(); i++) {
                const auto& cluster = task_clusters[i];
                //std::cout << "\nCluster " << i << ":" << std::endl;
                //std::cout << "  Tasks: ";
                for (int task_id : cluster.task_ids) {
                //    std::cout << task_id << " ";
                }
                //std::cout << "\n  Start Task: " << cluster.start_task_id
                //         << " at " << cluster.start_pos << std::endl;
                //std::cout << "  End Task: " << cluster.end_task_id
                //         << " at " << cluster.end_pos << std::endl;
                //std::cout << "  Total Cost: " << cluster.total_cost << std::endl;
            }
        }

        std::cout << "\n=== Cost Calculation ===" << std::endl;
        // Calculate costs only for robots that need reassignment
    for (const auto& robot_ptr : robots) {
            const ROBOT& robot = *robot_ptr;
            if (robot.get_status() == ROBOT::STATUS::EXHAUSTED || robot.type == ROBOT::TYPE::DRONE) continue;

            // Skip cost calculation if robot has a valid path and no map changes
            if (has_new_tasks || needs_reassignment) {
                
                //std::cout << "\nRobot " << robot.id << " (" << robot.type << ") at " << robotExpectedPosition[robot.id] << std::endl;
                for (const auto& task_ptr : active_tasks) {
                    const TASK& task = *task_ptr;
                    if (task.is_done()) continue;

                    int task_execution_cost = task.get_cost(robot.type);

                    // 태스크 자체의 실행 비용이 INF인 경우, 이 로봇에게 할당 불가능
                    if (task_execution_cost == INFINITE) {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        // 이 경우 로그 출력 건너뛰기
                        //std::cout << "  Task " << task.id << " at " << task.coord << ": UNREACHABLE (task cost)" << std::endl;
                        continue;
                    }

                    std::vector<ROBOT::ACTION> path_actions;
                    std::vector<Coord> path_coords;
                    int path_c = std::numeric_limits<int>::max();
                    bool found_in_cache = false;
                    
                    // Check if we have a valid cached path
                    if (path_cache.count(robot.id) && path_cache[robot.id].count(task.coord)) {
                        const PathInfo& cached_path = path_cache[robot.id][task.coord];
                        // 캐시된 경로 + 태스크 비용이 로봇 에너지보다 많으면 캐시 사용 안함
                         if (robot.get_energy() >= cached_path.cost + task_execution_cost) {
                            path_c = cached_path.cost;
                            path_actions = cached_path.actions; // Keep actions and coords for potential use
                            path_coords = cached_path.coordinates;
                            found_in_cache = true;
                            // 유효한 캐시 사용 로그 출력
                            //std::cout << "  Task " << task.id << " at " << task.coord << ": "
                                     //<< path_c << " (cached) + " << task_execution_cost
                                     //<< " = " << (path_c + task_execution_cost) << std::endl;
                        } else {
                            // 에너지가 부족하여 캐시 사용 불가
                             path_cache[robot.id].erase(task.coord); // Invalidate cache
                        }
                    }

                    // Only recalculate if necessary (no valid cache or map/task changes)
                    if (!found_in_cache && (updated_coords.empty() ||
                        std::any_of(updated_coords.begin(), updated_coords.end(),
                            [&](const Coord& c) { return c == task.coord; }))) {

                        // 다익스트라로 경로 비용 계산
                        path_c = dijkstra(robotExpectedPosition[robot.id], task.coord, robot, task_execution_cost,
                                        known_cost_map, known_object_map, map_size,
                                        path_actions, path_coords);

                        // 경로 비용이 너무 높으면 (에너지의 99% 초과) 할당 불가능으로 처리
                        if (path_c != std::numeric_limits<int>::max() && path_c > robot.get_energy() * 0.99) {
                            path_c = std::numeric_limits<int>::max();
                        }

                        // 새로운 경로 계산 로그 (INF가 아닌 경우만 출력)
                        if (path_c <= 100000000) {
                             //std::cout << "  Task " << task.id << " at " << task.coord << ": "
                                       //<< path_c << " (new) + " << task_execution_cost
                                       //<< " = " << (path_c + task_execution_cost) << std::endl;
                        }
                    }

                    // 최종 총 비용 계산 및 저장
                    if (path_c == std::numeric_limits<int>::max()) {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        //std::cout << "  Task " << task.id << " at " << task.coord << ": UNREACHABLE (no path or too costly)" << std::endl;
                    } else if (robot.get_energy() >= path_c + task_execution_cost) {
                         // 로봇 에너지가 충분한 경우
                        task_total_costs[robot.id][task.id] = path_c + task_execution_cost;
                        // 유효한 새 경로를 찾았다면 캐시에 저장
                        if (!found_in_cache && path_c != std::numeric_limits<int>::max()) {
                            path_cache[robot.id][task.coord] = PathInfo(path_actions, path_c, path_coords);
                        }
                    } else {
                         // 로봇 에너지가 부족한 경우
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                         // 에너지 부족으로 인한 할당 불가 로그 출력 건너뛰기
                        //std::cout << "  Task " << task.id << " at " << task.coord << ": UNREACHABLE (insufficient energy)" << std::endl;
                    }
                }
            }
        }

        // Perform task assignment if needed
        if (shouldTriggerReassignment(updated_coords, active_tasks, robots)) {
            //std::cout << "\n=== Task Assignment ===" << std::endl;
#ifdef USE_MINMIN
            performMinMinAssignment(active_tasks, robots, known_cost_map, known_object_map);
#elif defined(USE_SUFFERAGE)
            performSufferageAssignment(robots, active_tasks, known_cost_map, known_object_map);
#endif
            
            // Reset reassignment flags
            needs_reassignment = false;
            newly_discovered_tasks.clear();
            newly_completed_tasks.clear();
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
                                               int(known_cost_map.size()),
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
                                                   int(known_cost_map.size()),
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

// ────────────────────────────────────────────────────────────────
// ⬇️ NEW: 태스크 쌍 거리 캐시 Helper
// ----------------------------------------------------------------
int Scheduler::getPairDistance(int tid1, int tid2,
    const vector<shared_ptr<TASK>>& active_tasks,
    const shared_ptr<ROBOT>& wheel_robot,
    const shared_ptr<ROBOT>& cater_robot,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map)
{
    if (tid1 == tid2) return 0;
    long long key = (static_cast<long long>(min(tid1, tid2)) << 32) | max(tid1, tid2);
    auto it = pair_dist_cache.find(key);
    if (it != pair_dist_cache.end()) return it->second;

    const TASK* t1 = nullptr; const TASK* t2 = nullptr;
    for (const auto& t : active_tasks) {
        if (t->id == tid1) t1 = t.get();
        if (t->id == tid2) t2 = t.get();
    }
    if (!t1 || !t2) return INFINITE;

    std::vector<ROBOT::ACTION> a1, a2; std::vector<Coord> c1, c2;
    int d1 = dijkstra(t1->coord, t2->coord, *wheel_robot, 0,
        known_cost_map, known_object_map,
        known_cost_map.size(), a1, c1);
    int d2 = dijkstra(t1->coord, t2->coord, *cater_robot, 0,
        known_cost_map, known_object_map,
        known_cost_map.size(), a2, c2);
    int dist = std::max(d1, d2);
    pair_dist_cache[key] = dist;
    return dist;
}

// ────────────────────────────────────────────────────────────────
// ⬇️ NEW: Adaptive Split (재귀)
// ----------------------------------------------------------------
void Scheduler::splitClusterAdaptive(int cluster_idx,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map)
{
    unsigned long long now_tick = tick_counter;

    if (cluster_idx < 0 || cluster_idx >= static_cast<int>(task_clusters.size())) return;
    TaskCluster cluster = task_clusters[cluster_idx];
    if (cluster.task_ids.size() <= 1) return;   // 더 못 나눔

    // 작업 로봇 두 종류 찾기
    shared_ptr<ROBOT> wheel_robot = nullptr, cater_robot = nullptr;
    for (const auto& r : robots) {
        if (r->type == ROBOT::TYPE::WHEEL)       wheel_robot = r;
        else if (r->type == ROBOT::TYPE::CATERPILLAR) cater_robot = r;
    }
    if (!wheel_robot || !cater_robot) return;

    // 1) 가장 긴 내부 edge 탐색
    int max_dist = -1, cut_idx = -1;
    for (size_t i = 0;i < cluster.task_ids.size() - 1;++i) {
        int d = getPairDistance(cluster.task_ids[i], cluster.task_ids[i + 1],
            active_tasks, wheel_robot, cater_robot,
            known_cost_map, known_object_map);
        if (d > max_dist) {
            max_dist = d; cut_idx = static_cast<int>(i);
        }
    }
    if (cut_idx == -1) return;

    // 2) 두 개의 task id 벡터로 분할
    vector<int> left(cluster.task_ids.begin(), cluster.task_ids.begin() + cut_idx + 1);
    vector<int> right(cluster.task_ids.begin() + cut_idx + 1, cluster.task_ids.end());

    auto buildCluster = [&](const vector<int>& ids)->TaskCluster {
        TaskCluster nc; nc.task_ids = ids;
        // 시작/끝/비용 간단히 채워두고 후속 findClusterEndpoints 호출에 맡김
        nc.start_task_id = ids.front(); nc.end_task_id = ids.back();
        return nc;
        };

    TaskCluster leftC = buildCluster(left);
    TaskCluster rightC = buildCluster(right);

    // 3) 기존 위치에 left 덮어쓰기, right 는 push_back
    task_clusters[cluster_idx] = leftC;
    task_clusters.push_back(rightC);

    if (left.size() == cluster.task_ids.size() || right.empty()) {
        // 실제로 쪼개지지 않았음 → 더 이상 split 불가
        unassignable_clusters.insert(cluster_idx);
        return;
    }
}

// ────────────────────────────────────────────────────────────────
// ⬇️ NEW: Greedy Merge (여유 에너지 삽입)
// ----------------------------------------------------------------
void Scheduler::optimizeRobotSlack(const vector<shared_ptr<ROBOT>>& robots,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map)
{
    unsigned long long now_tick = tick_counter;

    for (const auto& r : robots) {
        if (r->type == ROBOT::TYPE::DRONE || r->get_status() == ROBOT::STATUS::EXHAUSTED) continue;
        if (robotTaskQueue[r->id].empty()) continue;

        // slack 계산
        int used = 0;
        std::queue<int> qcpy = robotTaskQueue[r->id];
        int prev_task_id = -1;
        while (!qcpy.empty()) {
            int tid = qcpy.front(); qcpy.pop();
            for (const auto& t : active_tasks) {
                if (t->id == tid) {
                    used += t->get_cost(r->type);
                    if (prev_task_id != -1) {
                        used += getPairDistance(prev_task_id, tid,
                            active_tasks,
                            (r->type == ROBOT::TYPE::WHEEL ? r : robots[0]), // dummy
                            (r->type == ROBOT::TYPE::CATERPILLAR ? r : robots[0]),
                            known_cost_map, known_object_map);
                    }
                    prev_task_id = tid;
                    break;
                }
            }
        }
        int slack = r->get_energy() - used;
        if (slack < 50) continue; // 여유 거의 없음

        // 아직 queue 에 없고, 미할당이며 가까운 태스크 하나 삽입 시도
        int best_tid = -1, best_extra = INT_MAX;
        Coord refCoord;
        // find coord of last task
        int last_tid = robotTaskQueue[r->id].back();
        for (const auto& t : active_tasks) {
            if (t->id == last_tid) { refCoord = t->coord; break; }
        }

        for (const auto& t : active_tasks) {
            if (isTaskAlreadyAssigned(t->id)) continue;
            int extra = dijkstra(refCoord, t->coord, *r, t->get_cost(r->type),
                known_cost_map, known_object_map,
                known_cost_map.size(),
                *(new vector<ROBOT::ACTION>),
                *(new vector<Coord>));
            if (extra == INFINITE) continue;
            extra += t->get_cost(r->type);
            if (extra < best_extra && extra < slack * 0.9) {
                best_extra = extra; best_tid = t->id;
            }
        }
        if (best_tid != -1) {
            robotTaskQueue[r->id].push(best_tid);
            // 할당 마킹
            robotToTask[r->id] = robotTaskQueue[r->id].front();
        }
    }
}

/*───────────────────────────────────────────────────────────*/
/*  Adaptive Min-Min + Dormant & Unassignable 처리 버전     */
/*───────────────────────────────────────────────────────────*/
void Scheduler::performMinMinAssignment(
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map)
{
    std::cout << "\nPerforming *Adaptive* Min-Min Assignment\n";

    /* 0) 이전 결과 초기화 */
    for (auto& kv : robotTaskQueue) while (!kv.second.empty()) kv.second.pop();
    robotToTask.clear();
    robotCurrentTaskEndTime.clear();

    /* 1) 시간값 & 헬퍼 준비 */
    unsigned long long now_tick = tick_counter;

    auto isDormant = [&](int tid) -> bool {
        auto it = task_dormant_until.find(tid);
        return it != task_dormant_until.end() && it->second > now_tick;
        };

    /* 2) unassigned 클러스터 set 준비  */
    unassignable_clusters.clear();                 // 이번 라운드에 새로 판단
    std::set<int> unassigned;
    for (size_t i = 0; i < task_clusters.size(); ++i)
        unassigned.insert(static_cast<int>(i));

    /* 3) 메인 루프 : 할당 가능한 (로봇, 클러스터) 쌍을 반복적으로 확정 */
    while (!unassigned.empty())
    {
        int best_cluster = -1;
        int best_robot = -1;
        int best_cost = std::numeric_limits<int>::max();

        /* 3-1) 모든 조합 탐색 */
        for (int cidx : unassigned)
        {
            if (unassignable_clusters.count(cidx)) continue;  // 이미 불가 판정
            TaskCluster& C = task_clusters[cidx];

            /* (클러스터 안에 dormant 태스크만 있다면 skip) */
            bool allDormant = true;
            for (int tid : C.task_ids)
                if (!isDormant(tid)) { allDormant = false; break; }
            if (allDormant) continue;

            for (const auto& rp : robots)
            {
                if (rp->type == ROBOT::TYPE::DRONE ||
                    rp->get_status() == ROBOT::STATUS::EXHAUSTED)
                    continue;

                /* ① 로봇 → 클러스터 첫 태스크까지 경로 비용 */
                const TASK* firstTask = nullptr;
                for (auto& t : active_tasks)
                    if (t->id == C.task_ids.front()) { firstTask = t.get(); break; }
                if (!firstTask) continue;

                std::vector<ROBOT::ACTION> tmpA; std::vector<Coord> tmpC;
                int pathCost = dijkstra(rp->get_coord(), firstTask->coord,
                    *rp, 0, known_cost_map, known_object_map,
                    known_cost_map.size(), tmpA, tmpC);
                if (pathCost == INFINITE) continue;

                /* ② 클러스터 내부 작업/이동 비용 단순합 */
                int execCost = 0;
                for (size_t k = 0; k < C.task_ids.size(); ++k)
                {
                    /* dormant task 는 건너뛰면 클러스터 의미가 깨지므로
                       여기서는 포함하되 에너지 초과면 나중에 Split 하도록 함   */
                    const TASK* taskPtr = nullptr;
                    for (auto& t : active_tasks)
                        if (t->id == C.task_ids[k]) { taskPtr = t.get(); break; }

                    if (!taskPtr) { execCost = INFINITE; break; }
                    execCost += taskPtr->get_cost(rp->type);

                    /* 태스크 간 이동 비용 */
                    if (k + 1 < C.task_ids.size())
                    {
                        int nextTid = C.task_ids[k + 1];
                        execCost += getPairDistance(taskPtr->id, nextTid,
                            active_tasks,
                            rp, rp,               // 아무 로봇이나 전달, 내부에서 타입무시
                            known_cost_map,
                            known_object_map);
                    }

                    if (execCost >= INFINITE / 2) break;
                }

                if (execCost >= INFINITE / 2) continue;

                int totalCost = pathCost + execCost;
                if (totalCost > rp->get_energy() * 0.95) continue;  // 에너지 초과

                if (totalCost < best_cost)
                {
                    best_cost = totalCost;
                    best_cluster = cidx;
                    best_robot = rp->id;
                }
            }
        }

        /* 3-2) 이번 라운드에서 할당 가능한 쌍을 찾지 못함 → Split or Dormant */
        if (best_cluster == -1)
        {
            /* 후보 중 첫 클러스터를 Split 시도 */
            int victim = *unassigned.begin();
            TaskCluster& V = task_clusters[victim];

            /* 더 이상 쪼갤 수 없는 경우( size==1 ) ⇒ dormant 처리 */
            if (V.task_ids.size() == 1)
            {
                int tid = V.task_ids.front();
                task_dormant_until[tid] = now_tick + DORMANT_TTL;
                unassigned.erase(victim);
                continue;
            }

            /* 분할 */
            splitClusterAdaptive(victim, active_tasks, robots,
                known_cost_map, known_object_map);

            /* split 후, original index 위치엔 left 클러스터,
               right 클러스터가 push_back 되었으므로 unassigned 재구성 */
            unassigned.clear();
            for (size_t i = 0; i < task_clusters.size(); ++i)
                if (!unassignable_clusters.count(static_cast<int>(i)))
                    unassigned.insert(static_cast<int>(i));
            continue;   // 다시 루프
        }

        /* 3-3) best (로봇, 클러스터) 확정 */
        TaskCluster& chosen = task_clusters[best_cluster];
        std::cout << "  ▶ Robot " << best_robot << " ⇦ Cluster "
            << best_cluster << "  (cost " << best_cost << ")\n";

        for (int tid : chosen.task_ids)
            robotTaskQueue[best_robot].push(tid);

        robotToTask[best_robot] = robotTaskQueue[best_robot].front();
        unassigned.erase(best_cluster);
    }

    /* 4) 여유 에너지로 태스크 끼워넣기(slack merge) */
    optimizeRobotSlack(robots, active_tasks, known_cost_map, known_object_map);

    std::cout << "Min-Min Assignment finished.\n";
}

// ────────────────────────────────────────────────────────────────



//void Scheduler::performMinMinAssignment(const vector<shared_ptr<TASK>>& active_tasks,
//                                      const vector<shared_ptr<ROBOT>>& robots,
//                                      const vector<vector<vector<int>>>& known_cost_map,
//                                      const vector<vector<OBJECT>>& known_object_map) {
//    std::cout << "\nPerforming Cluster-based Min-Min Assignment" << std::endl;
//    
//    // Clear existing assignments for reassignment
//    for (auto& robot_queue : robotTaskQueue) {
//        while (!robot_queue.second.empty()) {
//            robot_queue.second.pop();
//        }
//    }
//    robotToTask.clear();
//    robotCurrentTaskEndTime.clear();
//    
//    for (auto& robot : robots){
//        robotExpectedPosition[robot->id] = robot->get_coord();
//    }
//    
//    // Create set of unassigned clusters
//    std::set<int> unassigned_clusters;
//    for (size_t i = 0; i < task_clusters.size(); i++) {
//        unassigned_clusters.insert(i);
//    }
//
//    while (!unassigned_clusters.empty()) {
//        int best_cluster_idx = -1;
//        int best_robot_id = -1;
//        int min_completion_time = std::numeric_limits<int>::max();
//
//        std::cout << "\nFinding best cluster-robot pair from " << unassigned_clusters.size() << " unassigned clusters" << std::endl;
//
//        // Find cluster-robot pair with minimum completion time
//        for (int cluster_idx : unassigned_clusters) {
//            const TaskCluster& cluster = task_clusters[cluster_idx];
//            std::cout << "\nEvaluating Cluster " << cluster_idx << ":" << std::endl;
//            std::cout << "  Tasks: ";
//            for (int task_id : cluster.task_ids) {
//                std::cout << task_id << " ";
//            }
//            std::cout << "\n  Start Task: " << cluster.start_task_id
//                     << " at " << cluster.start_pos << std::endl;
//            std::cout << "  End Task: " << cluster.end_task_id
//                     << " at " << cluster.end_pos << std::endl;
//
//            for (const auto& robot : robots) {
//                if (robot->type == ROBOT::TYPE::DRONE ||
//                    robot->get_status() == ROBOT::STATUS::EXHAUSTED) continue;
//
//                // Get robot's current position
//                Coord robot_pos = robotExpectedPosition.count(robot->id) ?
//                                robotExpectedPosition[robot->id] :
//                                robot->get_coord();
//
//                // Check if robot's expected position is valid
//                if (known_object_map[robot_pos.x][robot_pos.y] == OBJECT::WALL) {
//                    // If expected position is invalid, use actual position
//                    robot_pos = robot->get_coord();
//                    robotExpectedPosition[robot->id] = robot_pos;
//                }
//
//                // Calculate path cost to both endpoints
//                std::vector<ROBOT::ACTION> start_path_actions, end_path_actions;
//                std::vector<Coord> start_path_coords, end_path_coords;
//                int start_path_cost = dijkstra(robot_pos,
//                                             cluster.start_pos,
//                                             *robot,
//                                             0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
//                                             known_cost_map,
//                                             known_object_map,
//                                             known_cost_map.size(),
//                                             start_path_actions,
//                                             start_path_coords);
//
//                int end_path_cost = dijkstra(robot_pos,
//                                           cluster.end_pos,
//                                           *robot,
//                                           0,  // task_cost는 0으로 설정 (순수 이동 비용만 계산)
//                                           known_cost_map,
//                                           known_object_map,
//                                           known_cost_map.size(),
//                                           end_path_actions,
//                                           end_path_coords);
//
//                // Skip if no valid path to either point
//                if (start_path_cost == std::numeric_limits<int>::max() &&
//                    end_path_cost == std::numeric_limits<int>::max()) {
//                    continue;
//                }
//
//                // Create a modified cluster with the correct task order
//                TaskCluster modified_cluster = cluster;
//                int path_cost;
//                
//                if (start_path_cost <= end_path_cost) {
//                    // Use original order (start -> end)
//                    path_cost = start_path_cost;
//                } else {
//                    // Reverse the task order (end -> start)
//                    path_cost = end_path_cost;
//                    std::reverse(modified_cluster.task_ids.begin(), modified_cluster.task_ids.end());
//                    modified_cluster.start_pos = cluster.end_pos;
//                    modified_cluster.end_pos = cluster.start_pos;
//                    modified_cluster.start_task_id = cluster.end_task_id;
//                    modified_cluster.end_task_id = cluster.start_task_id;
//                }
//                
//                // Calculate total cost including cluster's internal cost
//                int total_cost;
//                if (path_cost > std::numeric_limits<int>::max() - modified_cluster.total_cost) {
//                    // Overflow would occur, skip this assignment
//                    continue;
//                }
//                total_cost = path_cost + modified_cluster.total_cost;
//
//                // Skip if insufficient energy
//                int required_energy = total_cost*1.05;
//                if(cluster.task_ids.size()>=3) required_energy = total_cost*1.1;
//                if (robot->get_energy() < required_energy) {
//                    continue;
//                }
//
//                // Calculate completion time
//                int completion_time = calculateTaskCompletionTime(robot->id, 
//                                                                  modified_cluster.start_task_id,
//                                                               active_tasks,
//                                                               known_cost_map,
//                                                               known_object_map,
//                                                               robots);
//
//                // Skip if completion time is infinite
//                if (completion_time == std::numeric_limits<int>::max()) continue;
//
//                std::cout << "  Robot " << robot->id << " (" << robot->type << ") at " << robot_pos
//                         << ": path cost = " << path_cost
//                         << ", cluster cost = " << modified_cluster.total_cost
//                         << ", total = " << total_cost
//                         << ", completion time = " << completion_time << std::endl;
//
//                
//                // If this is a better completion time
//                if (completion_time < min_completion_time) {
//                    min_completion_time = completion_time;
//                    best_cluster_idx = cluster_idx;
//                    best_robot_id = robot->id;
//                    // Store the modified cluster for later use
//                    task_clusters[cluster_idx] = modified_cluster;
//                    std::cout << "  -> New best assignment found!" << std::endl;
//                }
//            }
//        }
//
//        if (best_cluster_idx == -1) {
//            std::cout << "\nNo assignable cluster found in this iteration. Splitting remaining unassigned clusters." << std::endl;
//
//            // 현재 unassigned_clusters에 있는 인덱스들을 임시 저장
//            std::vector<int> clusters_to_split_indices;
//            for(int cluster_idx : unassigned_clusters) {
//                clusters_to_split_indices.push_back(cluster_idx);
//            }
//
//            unassigned_clusters.clear(); // 현재 unassigned 목록을 비우고 새로 추가될 단일 태스크 클러스터로 채울 준비
//
//            for(int original_cluster_idx : clusters_to_split_indices) {
//                const TaskCluster& cluster_to_split = task_clusters[original_cluster_idx];
//                 std::cout << "  Splitting Cluster " << cluster_to_split.task_ids[0] << "... (size " << cluster_to_split.task_ids.size() << ")" << std::endl;
//                
//                for(int task_id : cluster_to_split.task_ids) {
//                    TaskCluster single_task_cluster;
//                    single_task_cluster.task_ids.push_back(task_id);
//                    single_task_cluster.start_task_id = task_id;
//                    single_task_cluster.end_task_id = task_id;
//                    
//                    // 태스크 좌표 찾기
//                    Coord task_coord;
//                    for (const auto& task : active_tasks) {
//                        if (task->id == task_id) {
//                            task_coord = task->coord;
//                            break;
//                        }
//                    }
//                    single_task_cluster.start_pos = task_coord;
//                    single_task_cluster.end_pos = task_coord;
//                    
//                    // 단일 태스크 클러스터의 총 비용은 해당 태스크의 최대 실행 비용
//                    int wheel_task_cost = std::numeric_limits<int>::max();
//                    int caterpillar_task_cost = std::numeric_limits<int>::max();
//                    for (const auto& task : active_tasks) {
//                        if (task->id == task_id) {
//                             wheel_task_cost = task->get_cost(ROBOT::TYPE::WHEEL);
//                             caterpillar_task_cost = task->get_cost(ROBOT::TYPE::CATERPILLAR);
//                            break;
//                        }
//                    }
//                    single_task_cluster.total_cost = std::max(wheel_task_cost, caterpillar_task_cost);
//
//                    // 분리된 단일 태스크를 새로운 클러스터 목록에 추가
//                    task_clusters.push_back(single_task_cluster);
//                    // 새로 추가된 클러스터의 인덱스를 unassigned 목록에 넣음
//                    unassigned_clusters.insert(task_clusters.size() - 1);
//                     std::cout << "    -> Created single task cluster for Task " << task_id << " (new index: " << task_clusters.size() - 1 << ")" << std::endl;
//                }
//                 // 분리된 원본 클러스터는 더 이상 고려하지 않음 (unassigned_cluster_indices에서 이미 비웠으므로)
//            }
//            break;
//            // 모든 불가능 클러스터를 분리했으므로, 다음 반복에서 분리된 단일 태스크 클러스터들을 대상으로 할당 재시도
//        }
//
//        const TaskCluster& best_cluster = task_clusters[best_cluster_idx];
//        std::cout << "\nAssigning Cluster " << best_cluster_idx << " to Robot " << best_robot_id
//                 << " (completion time: " << min_completion_time << ")" << std::endl;
//
//        // Assign all tasks in the cluster to the robot
//        for (int task_id : best_cluster.task_ids) {
//            robotTaskQueue[best_robot_id].push(task_id);
//        }
//        robotToTask[best_robot_id] = best_cluster.start_task_id;
//        robotCurrentTaskEndTime[best_robot_id] = min_completion_time;
//
//        // Update robot's expected position to the end position of the cluster
//        updateRobotPosition(best_robot_id, best_cluster.end_pos);
//        std::cout << "  Updated Robot " << best_robot_id << "'s expected position to " << best_cluster.end_pos << std::endl;
//
//        unassigned_clusters.erase(best_cluster_idx);
//    }
//     std::cout << "\nMin-Min Assignment finished." << std::endl;
//}

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
            int min_cost = CLUSTER_DISTANCE_THRESHOLD;

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

