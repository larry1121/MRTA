#include "schedular.h"
#include <cstdlib>  // For rand() in the original idle_action if needed
#include <set>      // For std::set in priority queue for Dijkstra (custom comparator)
#include <iostream> // Required for std::cout in print_task_total_costs_table
#include <iomanip>  // Required for std::setw in print_task_total_costs_table
#include <limits>
#include <queue>
#include <algorithm>
#include <tuple>
#include <cmath>

// Helper for Dijkstra:
// Using std::map for dist and parent for simplicity with Coord keys.
// std::priority_queue typically needs a custom comparator for structs or pairs.
// We'll store pairs of <cost, Coord> and use std::greater to make it a min-priority queue.

// Comparison for Coord to be used in std::map or std::set if needed.
// Coord already has operator< defined in simulator.h

set<Coord> now_coords;

int Scheduler::dijkstra(const Coord &start,
                        const Coord &goal,
                        const ROBOT &robot,
                        const int task_cost_for_robot,
                        const vector<vector<vector<int>>> &known_cost_map,
                        const vector<vector<OBJECT>> &known_object_map,
                        int map_size, // Assuming map_size = known_cost_map.size()
                        std::vector<ROBOT::ACTION> &out_path_actions,
                        std::vector<Coord> &out_path_coords)
{
    out_path_actions.clear();
    out_path_coords.clear();

    if (start == goal)
    {
        out_path_coords.push_back(start);
        return 0; // No cost if already at the goal
    }

    if (map_size <= 0 || known_cost_map.empty() || (known_cost_map[0].empty() && map_size > 0))
    {
        return std::numeric_limits<int>::max();
    }

    std::map<Coord, int> dist;
    std::map<Coord, Coord> parent_coord;
    std::map<Coord, ROBOT::ACTION> parent_action;

    std::priority_queue<std::pair<int, Coord>,
                        std::vector<std::pair<int, Coord>>,
                        std::greater<std::pair<int, Coord>>>
        pq;

    dist[start] = 0;
    pq.push({0, start});

    const ROBOT::TYPE robot_type = robot.type;
    const int initial_robot_energy = robot.get_energy();

    Coord current_coord_pq; // Renamed to avoid conflict with other current_coord variables if any
    int current_cost_pq;

    while (!pq.empty())
    {
        current_cost_pq = pq.top().first;
        current_coord_pq = pq.top().second;
        pq.pop();

        // Early termination if cost exceeds 8000
        if (current_cost_pq >= 8000)
        {
            return std::numeric_limits<int>::max();
        }

        if (dist.count(current_coord_pq) && current_cost_pq > dist.at(current_coord_pq))
        { // Use .at() for safety after check
            continue;
        }

        if (current_coord_pq == goal)
        {
            Coord backtrack_coord = goal;
            while (backtrack_coord != start)
            {
                if (!parent_coord.count(backtrack_coord))
                {
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
            if (!out_path_coords.empty() && out_path_coords.back() != goal)
            {
                out_path_coords.push_back(goal);
            }
            return current_cost_pq;
        }

        ROBOT::ACTION all_actions[] = {ROBOT::ACTION::UP, ROBOT::ACTION::DOWN, ROBOT::ACTION::LEFT, ROBOT::ACTION::RIGHT};
        for (ROBOT::ACTION action : all_actions)
        {
            Coord delta = action_to_delta(action);
            Coord next_coord = current_coord_pq + delta;

            if (next_coord.x < 0 || next_coord.x >= map_size || next_coord.y < 0 || next_coord.y >= map_size)
            {
                continue;
            }
            if (known_object_map.at(next_coord.x).at(next_coord.y) == OBJECT::WALL)
            {
                continue;
            }

            int edge_cost = Scheduler::calculate_move_cost(current_coord_pq, next_coord, robot_type, known_cost_map);

            if (edge_cost == INFINITE)
            {
                continue;
            }

            int new_cost = current_cost_pq + edge_cost;

            // Energy pruning conditions from problem statement
            if (task_cost_for_robot != INFINITE)
            { // Only prune if task_cost is known (not for pure pathfinding)
                // Check if adding task cost would cause overflow
                if (new_cost > std::numeric_limits<int>::max() - task_cost_for_robot)
                {
                    continue; // Skip if overflow would occur
                }

                int total_cost = new_cost + task_cost_for_robot;
                if (total_cost >= initial_robot_energy)
                { // Original condition: dist >= robot.energy - taskCost
                    if (next_coord == goal)
                    { // If it's the goal, this cost is fine if it's exactly energy - task_cost
                        if (total_cost > initial_robot_energy)
                            continue; // Strictly more, then prune
                    }
                    else
                    {
                        continue; // Not the goal, and already at the energy limit for path part
                    }
                }
            }
            else
            { // Pure pathfinding, no task cost to consider for this specific pruning rule
                if (new_cost >= initial_robot_energy && next_coord != goal)
                { // Path cost alone exceeds total energy
                    continue;
                }
            }
            // General check: path cost must not exceed initial energy
            if (new_cost > initial_robot_energy)
            {
                continue;
            }

            if (!dist.count(next_coord) || new_cost < dist.at(next_coord))
            {
                dist[next_coord] = new_cost;
                parent_coord[next_coord] = current_coord_pq;
                parent_action[next_coord] = action;
                pq.push({new_cost, next_coord});
            }
        }
    }
    return std::numeric_limits<int>::max();
}

void Scheduler::checkForNewTasks(const vector<shared_ptr<TASK>> &active_tasks)
{
    for (const auto &task : active_tasks)
    {
        if (!task->is_done() && !newly_discovered_tasks.count(task->id))
        {
            newly_discovered_tasks.insert(task->id);
            needs_reassignment = true;
        }
    }
}

void Scheduler::checkForCompletedTasks(const vector<shared_ptr<TASK>> &active_tasks)
{
    for (const auto &task : active_tasks)
    {
        if (task->is_done() && !newly_completed_tasks.count(task->id))
        {
            newly_completed_tasks.insert(task->id);
            needs_reassignment = true;
        }
    }
}

bool Scheduler::shouldTriggerReassignment(const set<Coord> &updated_coords,
                                          const vector<shared_ptr<TASK>> &active_tasks,
                                          const vector<shared_ptr<ROBOT>> &robots) const
{
    // Always trigger reassignment
    if (needs_reassignment)
    {
        return true;
    }
    ///////////////////
    bool idle_exists = std::any_of(robots.begin(), robots.end(),
                                   [](const auto &r)
                                   {
                                       return r->type != ROBOT::TYPE::DRONE &&
                                              r->get_status() == ROBOT::STATUS::IDLE;
                                   });

    bool unfinished_task_exists = std::any_of(active_tasks.begin(), active_tasks.end(),
                                              [](const auto &t)
                                              { return !t->is_done(); });

    if (idle_exists && unfinished_task_exists)
        return true;
    ///////////////////
    return false;
}
Scheduler::Scheduler()
    : tick_counter_(0), // 초기화 리스트
      has_started_assignments(false)
{
    // 생성자 본문이 비어 있어도 됨
}
void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    /*----- 틱 카운트 -----*/
    ++tick_counter_;

    {
        DRONE_init_tiles(known_object_map);
        DRONE_update_tile_info(known_object_map);
        DRONE_assigned_targets.clear();

        /* 드론만 추려 ID 순 정렬 */
        std::vector<std::shared_ptr<ROBOT>> drones;
        for (const auto &r : robots)
            if (r->type == ROBOT::TYPE::DRONE)
                drones.push_back(r);
        std::sort(drones.begin(), drones.end(),
                  [](const auto &a, const auto &b)
                  { return a->id < b->id; });

        for (const auto &dr_ptr : drones)
        {
            int rid = dr_ptr->id;
            Coord pos = dr_ptr->get_coord();

            /* 목표지 근방이면 유지 */
            if (dr_ptr->get_status() == ROBOT::STATUS::MOVING &&
                DRONE_drone_targets.count(rid) && !DRONE_drone_paths[rid].empty())
            {
                Coord tgt = DRONE_drone_targets[rid];
                int manhattan = abs(pos.x - tgt.x) + abs(pos.y - tgt.y);
                if (manhattan > DRONE_distance_threshold)
                {
                    DRONE_assigned_targets.insert({tgt.x, tgt.y});
                    continue;
                }
            }

            /* 가장 높은 스코어 타일 탐색 */
            double best_score = -1e9;
            Coord best_center = pos;
            std::vector<Coord> best_path;

            std::vector<std::tuple<double, int, int>> cand;
            for (int i = 0; i < DRONE_tile_rows; ++i)
                for (int j = 0; j < DRONE_tile_cols; ++j)
                {
                    if (DRONE_tiles[i][j].unseen_cells == 0)
                        continue;
                    Coord center = DRONE_tiles[i][j].center;
                    if (DRONE_assigned_targets.count({center.x, center.y}))
                        continue;
                    if (known_object_map[center.x][center.y] == OBJECT::WALL)
                        continue;

                    double dist = abs(pos.x - center.x) + abs(pos.y - center.y);
                    double score = double(DRONE_tiles[i][j].unseen_cells) / (dist + 1);
                    if (DRONE_tiles[i][j].unseen_cells > DRONE_tile_size * DRONE_tile_size * 0.7)
                        score *= DRONE_high_priority_weight;
                    else if (DRONE_tiles[i][j].unseen_cells > DRONE_tile_size * DRONE_tile_size * 0.4)
                        score *= DRONE_mid_priority_weight;

                    cand.emplace_back(score, i, j);
                }

            /* 재탐색 후보가 없고 pause 종료전이라면 그대로 HOLD */
            if (cand.empty() && DRONE_is_exploration_time(known_cost_map) == false)
            {
                DRONE_drone_paths[rid].clear();
                DRONE_drone_targets[rid] = pos;
                continue;
            }

            std::sort(cand.begin(), cand.end(),
                      [](auto &a, auto &b)
                      { return std::get<0>(a) > std::get<0>(b); });

            for (const auto &t : cand)
            {
                if (std::get<0>(t) <= best_score * DRONE_candidate_threshold)
                    break;
                int i = std::get<1>(t), j = std::get<2>(t);
                Coord center = DRONE_tiles[i][j].center;
                auto path = DRONE_plan_path(pos, center, known_cost_map, *dr_ptr, known_object_map);
                if (path.empty())
                    continue;
                double final_score = double(DRONE_tiles[i][j].unseen_cells + 1) / (path.size() + 1);
                if (final_score > best_score)
                {
                    best_score = final_score;
                    best_center = center;
                    best_path = path;
                }
            }
            if (best_path.empty() && known_object_map[pos.x][pos.y] == OBJECT::UNKNOWN)
            {
                best_path.push_back(pos);
                best_center = pos;
            }

            if (!best_path.empty())
            {
                DRONE_drone_paths[rid] = std::deque<Coord>(best_path.begin(), best_path.end());
                DRONE_drone_targets[rid] = best_center;
                DRONE_assigned_targets.insert({best_center.x, best_center.y});
            }
            else
            {
                DRONE_drone_paths[rid].clear();
                DRONE_drone_targets[rid] = pos;
            }
        }
    } /* ──── 드론 로직 끝 ──── */

    /*----- 100틱 체크 -----*/
    if (!has_started_assignments && tick_counter_ >= 100)
        has_started_assignments = true;

    if (!has_started_assignments)
        return; // 아직 100틱 미만이면 아무것도 하지 않음

    int map_size = int(known_cost_map.size());

    bool NO_IDLE = true;
    for (const auto &robot : robots)
    {
        if (robot->type != ROBOT::TYPE::DRONE)
        {
            if (robot->get_status() == ROBOT::STATUS::IDLE)
            {
                NO_IDLE = false;
            }
        }
    }
    if (NO_IDLE == true)
    {
        return;
    }
    needs_reassignment = false;

    // Initialize robot positions if not set
    for (const auto &robot : robots)
    {
        if (!robotExpectedPosition.count(robot->id))
        {
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

    for (const auto &robot : robots)
    {
        if (robot->get_status() == ROBOT::STATUS::IDLE &&
            robotTaskQueue[robot->id].empty() && robot->type != ROBOT::TYPE::DRONE)
        {
            has_idle_robots = true;
            break;
        }
    }

    for (const auto &task : active_tasks)
    {
        if (!task->is_done())
        {
            has_available_tasks = true;
            if (!newly_discovered_tasks.empty() || now_coords == updated_coords)
            {
                has_new_tasks = true;
            }
            break;
        }
    }
    now_coords = updated_coords;
    // Only trigger reassignment if there are new tasks or map changes
    if (has_started_assignments && has_idle_robots && has_available_tasks &&
        (has_new_tasks || !updated_coords.empty()))
    {
        needs_reassignment = true;
    }
    if (needs_reassignment == false)
        return;

    // Remove completed tasks from robot queues
    for (const auto &task : active_tasks)
    {
        if (task->is_done())
        {
            for (auto &robot_queue : robotTaskQueue)
            {
                std::queue<int> temp_queue;
                while (!robot_queue.second.empty())
                {
                    if (robot_queue.second.front() != task->id)
                    {
                        temp_queue.push(robot_queue.second.front());
                    }
                    robot_queue.second.pop();
                }
                robot_queue.second = temp_queue;
            }
        }
    }

    // Only calculate costs if we've started assignments and there are changes
    if (has_started_assignments && (needs_reassignment || has_new_tasks))
    {
        // Perform task clustering only when new tasks are discovered or map is updated
        if (has_new_tasks)
        {
            std::cout << "\n=== Task Clustering ===" << std::endl;
            // Perform task clustering
            clusterTasks(active_tasks, robots, known_cost_map, known_object_map);

            // Print cluster information
            for (size_t i = 0; i < task_clusters.size(); i++)
            {
                const auto &cluster = task_clusters[i];
                // std::cout << "\nCluster " << i << ":" << std::endl;
                // std::cout << "  Tasks: ";
                for (int task_id : cluster.task_ids)
                {
                    //    std::cout << task_id << " ";
                }
            }
        }

        std::cout << "\n=== Cost Calculation ===" << std::endl;
        // Calculate costs only for robots that need reassignment
        for (const auto &robot_ptr : robots)
        {
            const ROBOT &robot = *robot_ptr;
            if (robot.get_status() == ROBOT::STATUS::EXHAUSTED || robot.type == ROBOT::TYPE::DRONE)
                continue;

            // Skip cost calculation if robot has a valid path and no map changes
            if (has_new_tasks || needs_reassignment)
            {

                // std::cout << "\nRobot " << robot.id << " (" << robot.type << ") at " << robotExpectedPosition[robot.id] << std::endl;
                for (const auto &task_ptr : active_tasks)
                {
                    const TASK &task = *task_ptr;
                    if (task.is_done())
                        continue;

                    int task_execution_cost = task.get_cost(robot.type);

                    // 태스크 자체의 실행 비용이 INF인 경우, 이 로봇에게 할당 불가능
                    if (task_execution_cost == INFINITE)
                    {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        // 이 경우 로그 출력 건너뛰기
                        // std::cout << "  Task " << task.id << " at " << task.coord << ": UNREACHABLE (task cost)" << std::endl;
                        continue;
                    }

                    std::vector<ROBOT::ACTION> path_actions;
                    std::vector<Coord> path_coords;
                    int path_c = std::numeric_limits<int>::max();
                    bool found_in_cache = false;

                    // Check if we have a valid cached path
                    if (path_cache.count(robot.id) && path_cache[robot.id].count(task.coord))
                    {
                        const PathInfo &cached_path = path_cache[robot.id][task.coord];
                        // 캐시된 경로 + 태스크 비용이 로봇 에너지보다 많으면 캐시 사용 안함
                        if (robot.get_energy() >= cached_path.cost + task_execution_cost)
                        {
                            path_c = cached_path.cost;
                            path_actions = cached_path.actions; // Keep actions and coords for potential use
                            path_coords = cached_path.coordinates;
                            found_in_cache = true;
                            // 유효한 캐시 사용 로그 출력
                            // std::cout << "  Task " << task.id << " at " << task.coord << ": "
                            //<< path_c << " (cached) + " << task_execution_cost
                            //<< " = " << (path_c + task_execution_cost) << std::endl;
                        }
                        else
                        {
                            // 에너지가 부족하여 캐시 사용 불가
                            path_cache[robot.id].erase(task.coord); // Invalidate cache
                        }
                    }

                    // Only recalculate if necessary (no valid cache or map/task changes)
                    if (!found_in_cache && (updated_coords.empty() ||
                                            std::any_of(updated_coords.begin(), updated_coords.end(),
                                                        [&](const Coord &c)
                                                        { return c == task.coord; })))
                    {

                        // 다익스트라로 경로 비용 계산
                        path_c = dijkstra(robotExpectedPosition[robot.id], task.coord, robot, task_execution_cost,
                                          known_cost_map, known_object_map, map_size,
                                          path_actions, path_coords);

                        // 경로 비용이 너무 높으면 (에너지의 99% 초과) 할당 불가능으로 처리
                        if (path_c != std::numeric_limits<int>::max() && path_c > robot.get_energy() * 0.99)
                        {
                            path_c = std::numeric_limits<int>::max();
                        }

                        // 새로운 경로 계산 로그 (INF가 아닌 경우만 출력)
                        if (path_c <= 100000000)
                        {
                            // std::cout << "  Task " << task.id << " at " << task.coord << ": "
                            //<< path_c << " (new) + " << task_execution_cost
                            //<< " = " << (path_c + task_execution_cost) << std::endl;
                        }
                    }

                    // 최종 총 비용 계산 및 저장
                    if (path_c == std::numeric_limits<int>::max())
                    {
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        // std::cout << "  Task " << task.id << " at " << task.coord << ": UNREACHABLE (no path or too costly)" << std::endl;
                    }
                    else if (robot.get_energy() >= path_c + task_execution_cost)
                    {
                        // 로봇 에너지가 충분한 경우
                        task_total_costs[robot.id][task.id] = path_c + task_execution_cost;
                        // 유효한 새 경로를 찾았다면 캐시에 저장
                        if (!found_in_cache && path_c != std::numeric_limits<int>::max())
                        {
                            path_cache[robot.id][task.coord] = PathInfo(path_actions, path_c, path_coords);
                        }
                    }
                    else
                    {
                        // 로봇 에너지가 부족한 경우
                        task_total_costs[robot.id][task.id] = std::numeric_limits<int>::max();
                        // 에너지 부족으로 인한 할당 불가 로그 출력 건너뛰기
                        // std::cout << "  Task " << task.id << " at " << task.coord << ": UNREACHABLE (insufficient energy)" << std::endl;
                    }
                }
            }
        }

        // Perform task assignment if needed
        if (shouldTriggerReassignment(updated_coords, active_tasks, robots))
        {
            // std::cout << "\n=== Task Assignment ===" << std::endl;
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
        for (const auto &robot : robots)
        {
            if (robot->get_status() == ROBOT::STATUS::EXHAUSTED)
                continue;

            if (!robotTaskQueue[robot->id].empty())
            {
                int next_task_id = robotTaskQueue[robot->id].front();
                for (const auto &task : active_tasks)
                {
                    if (task->id == next_task_id)
                    {
                        if (path_cache.count(robot->id) && path_cache[robot->id].count(task->coord))
                        {
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
    if (robot_target_task_id.count(robot.id) && robot_target_task_id[robot.id] == task.id)
    {
        robot_current_paths.erase(robot.id);
        robot_target_task_id.erase(robot.id);
    }

    if (robot.type == ROBOT::TYPE::DRONE)
    {
        return false;
    }

    int task_cost = task.get_cost(robot.type);
    if (task_cost == INFINITE)
        return false;

    if (robot.get_energy() >= task_cost)
    {
        // Remove task from queue when starting work
        if (!robotTaskQueue[robot.id].empty() && robotTaskQueue[robot.id].front() == task.id)
        {
            robotTaskQueue[robot.id].pop(); // Remove task from queue when starting work
            updateRobotPosition(robot.id, task.coord);

            // If there are more tasks in the queue, prepare for the next one
            if (!robotTaskQueue[robot.id].empty())
            {
                int next_task_id = robotTaskQueue[robot.id].front();
                for (const auto &next_task : active_tasks)
                {
                    if (next_task->id == next_task_id)
                    {
                        // Recalculate path to next task
                        std::vector<ROBOT::ACTION> path_actions;
                        std::vector<Coord> path_coords;
                        int next_task_cost = next_task->get_cost(robot.type);

                        int path_cost = dijkstra(task.coord, next_task->coord, robot, next_task_cost,
                                                 known_cost_map, known_object_map,
                                                 int(known_cost_map.size()),
                                                 path_actions, path_coords);

                        if (path_cost != std::numeric_limits<int>::max())
                        {
                            path_cache[robot.id][next_task->coord] = PathInfo(path_actions, path_cost, path_coords);
                        }
                        break;
                    }
                }
            }
        }
        if (task.get_assigned_robot_id() == -1)
        {
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
    if (robot.type == ROBOT::TYPE::DRONE)
    {
        if (!DRONE_is_exploration_time(known_cost_map))
            return ROBOT::ACTION::HOLD;
        int rid = robot.id;
        Coord pos = robot.get_coord();
        if (DRONE_drone_paths.count(rid) && !DRONE_drone_paths[rid].empty())
        {
            if (DRONE_coord_equal(pos, DRONE_drone_paths[rid].front()))
                DRONE_drone_paths[rid].pop_front();
            if (!DRONE_drone_paths[rid].empty())
            {
                Coord nxt = DRONE_drone_paths[rid].front();
                if (known_object_map[nxt.x][nxt.y] == OBJECT::WALL)
                    return ROBOT::ACTION::HOLD;
                return DRONE_get_direction(pos, nxt);
            }
        }
        return ROBOT::ACTION::HOLD;
    }

    if (robot.get_status() != ROBOT::STATUS::IDLE)
    {
        return ROBOT::ACTION::HOLD;
    }

    // Update robot's expected position to match actual position
    robotExpectedPosition[robot.id] = robot.get_coord();

    // Continue current path if one exists and is valid
    if (robot_current_paths.count(robot.id))
    {
        PathInfo &current_path = robot_current_paths.at(robot.id);
        if (!current_path.actions.empty() && !current_path.coordinates.empty())
        {
            // Check if robot is at the expected position for the next move
            if (robot.get_coord() == current_path.coordinates.front())
            {
                ROBOT::ACTION next_action = current_path.actions.front();
                current_path.actions.erase(current_path.actions.begin());
                current_path.coordinates.erase(current_path.coordinates.begin());
                return next_action;
            }
            else
            {
                // Robot deviated from path, recalculate path to current target
                if (robot_target_task_id.count(robot.id))
                {
                    int target_task_id = robot_target_task_id[robot.id];
                    for (const auto &task : active_tasks)
                    {
                        if (task->id == target_task_id)
                        {
                            std::vector<ROBOT::ACTION> path_actions;
                            std::vector<Coord> path_coords;
                            int task_cost = task->get_cost(robot.type);
                            int path_cost = dijkstra(robot.get_coord(), task->coord, robot, task_cost,
                                                     known_cost_map, known_object_map,
                                                     int(known_cost_map.size()),
                                                     path_actions, path_coords);

                            if (path_cost != std::numeric_limits<int>::max())
                            {
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
        }
        else
        {
            // Path is empty, clear it
            robot_current_paths.erase(robot.id);
            robot_target_task_id.erase(robot.id);
        }
    }

    // If robot has tasks in queue, get the next task
    if (!robotTaskQueue[robot.id].empty())
    {
        int next_task_id = robotTaskQueue[robot.id].front();
        for (const auto &task : active_tasks)
        {
            if (task->id == next_task_id)
            {
                // Always calculate new path to ensure it's valid
                std::vector<ROBOT::ACTION> path_actions;
                std::vector<Coord> path_coords;
                int task_cost = task->get_cost(robot.type);
                int path_cost = dijkstra(robot.get_coord(), task->coord, robot, task_cost,
                                         known_cost_map, known_object_map,
                                         known_cost_map.size(),
                                         path_actions, path_coords);

                if (path_cost != std::numeric_limits<int>::max())
                {
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
bool Scheduler::is_map_fully_revealed(const vector<vector<OBJECT>> &known_object_map) const
{
    if (known_object_map.empty())
        return false;
    for (const auto &row : known_object_map)
    {
        for (const auto &cell : row)
        {
            if (cell == OBJECT::UNKNOWN)
            {
                return false;
            }
        }
    }
    return true;
}

void Scheduler::perform_task_assignment(const vector<shared_ptr<ROBOT>> &robots,
                                        const vector<shared_ptr<TASK>> &active_tasks,
                                        const vector<vector<OBJECT>> &known_object_map) // known_cost_map and map_size are implicitly available via this->task_total_costs or members
{
    if (active_tasks.empty() || robots.empty() || task_total_costs.empty())
    {
        return;
    }

    std::vector<bool> robot_assigned_in_this_round(robots.size() + 10, false);      // Max robot ID + buffer, or use a map
    std::vector<bool> task_assigned_in_this_round(active_tasks.size() + 10, false); // Max task ID + buffer, or use a map
    // A safer way for direct indexing if IDs are not 0-N: map ID to index or use std::map<int, bool>
    std::map<int, bool> robot_newly_assigned_map;
    std::map<int, bool> task_newly_assigned_map;

    int assignments_made_this_iteration;
    do
    {
        assignments_made_this_iteration = 0;
        int best_overall_robot_id = -1;
        int best_overall_task_id = -1;
        Coord best_overall_task_coord;
        int min_overall_cost = std::numeric_limits<int>::max();

        for (const auto &robot_ptr : robots)
        {
            const ROBOT &robot = *robot_ptr;
            if (robot.type == ROBOT::TYPE::DRONE || robot.get_status() == ROBOT::STATUS::EXHAUSTED)
                continue;
            if (robotToTask.count(robot.id) || robot_newly_assigned_map.count(robot.id))
                continue; // Already has a task or assigned this round

            if (!task_total_costs.count(robot.id))
                continue; // No cost entries for this robot

            const auto &costs_for_this_robot = task_total_costs.at(robot.id);

            for (const auto &task_ptr_inner : active_tasks)
            {
                const TASK &task = *task_ptr_inner;
                if (task.is_done() || task_newly_assigned_map.count(task.id))
                    continue; // Task done or assigned this round

                // Check if task is already assigned to *another* robot from a previous persistent assignment
                bool already_globally_assigned_to_another = false;
                for (const auto &entry : robotToTask)
                {
                    if (entry.second == task.id && entry.first != robot.id)
                    {
                        already_globally_assigned_to_another = true;
                        break;
                    }
                }
                if (already_globally_assigned_to_another)
                    continue;

                if (!costs_for_this_robot.count(task.id))
                    continue; // No cost entry for this robot-task pair

                int current_total_cost = costs_for_this_robot.at(task.id);

                if (current_total_cost < min_overall_cost)
                {
                    // Check if robot has energy for path + task (already embedded in task_total_costs if calculated correctly)
                    // task_total_costs[robot.id][task.id] should be max_int if not enough energy
                    if (current_total_cost != std::numeric_limits<int>::max())
                    {
                        min_overall_cost = current_total_cost;
                        best_overall_robot_id = robot.id;
                        best_overall_task_id = task.id;
                        best_overall_task_coord = task.coord;
                    }
                }
            }
        }

        if (best_overall_robot_id != -1 && best_overall_task_id != -1)
        {
            robotToTask[best_overall_robot_id] = best_overall_task_id;
            robot_newly_assigned_map[best_overall_robot_id] = true;
            task_newly_assigned_map[best_overall_task_id] = true;
            assignments_made_this_iteration++;

            // Set current path for the assigned robot
            if (path_cache.count(best_overall_robot_id) && path_cache.at(best_overall_robot_id).count(best_overall_task_coord))
            {
                const PathInfo &chosen_path = path_cache.at(best_overall_robot_id).at(best_overall_task_coord);
                if (!chosen_path.actions.empty())
                {
                    robot_current_paths[best_overall_robot_id] = chosen_path;
                    // The first action will be popped by idle_action
                }
                else
                {
                    robotToTask.erase(best_overall_robot_id); // Path is empty, cannot assign
                    robot_newly_assigned_map.erase(best_overall_robot_id);
                    task_newly_assigned_map.erase(best_overall_task_id);
                    assignments_made_this_iteration--;
                }
            }
            else
            {
                // No path found in cache for this assignment, assignment is not possible
                robotToTask.erase(best_overall_robot_id);
                robot_newly_assigned_map.erase(best_overall_robot_id);
                task_newly_assigned_map.erase(best_overall_task_id);
                assignments_made_this_iteration--;
            }
        }
    } while (assignments_made_this_iteration > 0);
}

void Scheduler::updateRobotPosition(int robotId, const Coord &newPosition)
{
    robotExpectedPosition[robotId] = newPosition;
}

void Scheduler::recalculateCostsForRobot(int robotId,
                                         const vector<shared_ptr<ROBOT>> &robots,
                                         const vector<shared_ptr<TASK>> &active_tasks,
                                         const vector<vector<vector<int>>> &known_cost_map,
                                         const vector<vector<OBJECT>> &known_object_map)
{
    // Clear existing costs for this robot
    if (task_total_costs.count(robotId))
    {
        task_total_costs[robotId].clear();
    }

    // Get robot's expected position
    Coord robotPos = robotExpectedPosition.count(robotId) ? robotExpectedPosition[robotId] : Coord(-1, -1);

    if (robotPos.x == -1)
        return; // Robot position not set

    // Find the robot
    auto robot_it = std::find_if(robots.begin(), robots.end(),
                                 [robotId](const auto &r)
                                 { return r->id == robotId; });
    if (robot_it == robots.end())
        return;

    // Recalculate costs for all tasks
    for (const auto &task_ptr : active_tasks)
    {
        const TASK &task = *task_ptr;
        if (task.is_done())
            continue;

        std::vector<ROBOT::ACTION> path_actions;
        std::vector<Coord> path_coords;
        int task_execution_cost = task.get_cost((*robot_it)->type);

        int path_cost = dijkstra(robotPos, task.coord,
                                 **robot_it,
                                 task_execution_cost,
                                 known_cost_map, known_object_map,
                                 known_cost_map.size(),
                                 path_actions, path_coords);

        if (path_cost != std::numeric_limits<int>::max())
        {
            task_total_costs[robotId][task.id] = path_cost + task_execution_cost;
            path_cache[robotId][task.coord] = PathInfo(path_actions, path_cost, path_coords);
        }
        else
        {
            task_total_costs[robotId][task.id] = std::numeric_limits<int>::max();
        }
    }
}

int Scheduler::calculateTaskCompletionTime(int robotId, int taskId,
                                           const vector<shared_ptr<TASK>> &active_tasks,
                                           const vector<vector<vector<int>>> &known_cost_map,
                                           const vector<vector<OBJECT>> &known_object_map,
                                           const vector<shared_ptr<ROBOT>> &robots)
{
    int currentTime = robotCurrentTaskEndTime.count(robotId) ? robotCurrentTaskEndTime[robotId] : 0;

    // Find the cluster containing this task
    for (const auto &cluster : task_clusters)
    {
        if (std::find(cluster.task_ids.begin(), cluster.task_ids.end(), taskId) != cluster.task_ids.end())
        {
            // This is the start task of the cluster
            if (taskId == cluster.start_task_id)
            {
                // Get robot's current position
                Coord robot_pos = robotExpectedPosition.count(robotId) ? robotExpectedPosition[robotId] : Coord(-1, -1);

                if (robot_pos.x == -1)
                    return std::numeric_limits<int>::max();

                // Find the robot
                auto robot_it = std::find_if(robots.begin(), robots.end(),
                                             [robotId](const auto &r)
                                             { return r->id == robotId; });
                if (robot_it == robots.end())
                    return std::numeric_limits<int>::max();

                // Calculate path cost to cluster start
                std::vector<ROBOT::ACTION> path_actions;
                std::vector<Coord> path_coords;
                int path_cost = dijkstra(robot_pos, cluster.start_pos,
                                         **robot_it, 0,
                                         known_cost_map, known_object_map,
                                         known_cost_map.size(),
                                         path_actions, path_coords);

                if (path_cost == std::numeric_limits<int>::max())
                {
                    return std::numeric_limits<int>::max();
                }

                // Add cluster's total cost (includes all task execution times and movement costs)
                return currentTime + path_cost + cluster.total_cost;
            }
        }
    }

    return std::numeric_limits<int>::max();
}

bool Scheduler::isTaskAlreadyAssigned(int taskId) const
{
    // Check task queues
    for (const auto &robot_queue : robotTaskQueue)
    {
        std::queue<int> temp_queue = robot_queue.second;
        while (!temp_queue.empty())
        {
            if (temp_queue.front() == taskId)
            {
                return true;
            }
            temp_queue.pop();
        }
    }

    // Check currently assigned tasks
    for (const auto &assignment : robotToTask)
    {
        if (assignment.second == taskId)
        {
            return true;
        }
    }

    return false;
}

void Scheduler::performMinMinAssignment(const vector<shared_ptr<TASK>> &active_tasks,
                                        const vector<shared_ptr<ROBOT>> &robots,
                                        const vector<vector<vector<int>>> &known_cost_map,
                                        const vector<vector<OBJECT>> &known_object_map)
{
    std::cout << "\n=== Cluster-based Min-Min Assignment ===\n";

    /* 0) 초기화 ---------------------------------------------------------------- */
    for (auto &q : robotTaskQueue)
        while (!q.second.empty())
            q.second.pop();
    robotToTask.clear();
    robotCurrentTaskEndTime.clear();

    for (auto &rb : robots) // 현재 위치를 기대 위치로
        robotExpectedPosition[rb->id] = rb->get_coord();

    std::set<int> unassigned_clusters;
    for (size_t i = 0; i < task_clusters.size(); ++i)
        unassigned_clusters.insert(static_cast<int>(i));

    /* 1) 반복적으로 (로봇,클러스터) 최적 짝 선정 -------------------------------- */
    while (!unassigned_clusters.empty())
    {
        int best_cluster_idx = -1;
        int best_robot_id = -1;
        int best_finish_time = std::numeric_limits<int>::max();

        std::cout << "\n▶ searching (" << unassigned_clusters.size()
                  << " clusters remain)\n";

        /* ── 모든 후보 평가 ─────────────────────────────────── */
        for (int cl_idx : unassigned_clusters)
        {
            const TaskCluster &orig = task_clusters[cl_idx];

            std::cout << "\n  ■ Cluster " << cl_idx << " : ";
            for (int t : orig.task_ids)
                std::cout << t << ' ';
            std::cout << "\n     start " << orig.start_pos
                      << " , end " << orig.end_pos << '\n';

            for (const auto &rb_ptr : robots)
            {
                const ROBOT &rb = *rb_ptr;
                if (rb.type == ROBOT::TYPE::DRONE ||
                    rb.get_status() == ROBOT::STATUS::EXHAUSTED)
                    continue;

                /* a) 로봇 현위치 -------------------------------------------------- */
                Coord rb_pos = robotExpectedPosition.count(rb.id) ? robotExpectedPosition.at(rb.id) : rb.get_coord();
                if (known_object_map[rb_pos.x][rb_pos.y] == OBJECT::WALL)
                { // 기대 위치가 잘못됐으면 교정
                    rb_pos = rb.get_coord();
                    robotExpectedPosition[rb.id] = rb_pos;
                }

                /* b) 두 끝점까지 경로(정방향/역방향) ----------------------------- */
                std::vector<ROBOT::ACTION> pAct1, pAct2;
                std::vector<Coord> pCrd1, pCrd2;

                int cost_to_start = dijkstra(rb_pos, orig.start_pos, rb,
                                             0, known_cost_map, known_object_map,
                                             known_cost_map.size(), pAct1, pCrd1);

                int cost_to_end = dijkstra(rb_pos, orig.end_pos, rb,
                                           0, known_cost_map, known_object_map,
                                           known_cost_map.size(), pAct2, pCrd2);

                if (cost_to_start == INFINITE && cost_to_end == INFINITE)
                    continue;

                /* c) 작업 순서 결정 (가까운 쪽을 시작점으로) -------------------- */
                TaskCluster cluster = orig; // 복사본
                int path_cost_to_cluster = cost_to_start;

                if (cost_to_end < cost_to_start)
                {
                    path_cost_to_cluster = cost_to_end;
                    std::reverse(cluster.task_ids.begin(), cluster.task_ids.end());
                    cluster.start_pos = orig.end_pos;
                    cluster.end_pos = orig.start_pos;
                    cluster.start_task_id = orig.end_task_id;
                    cluster.end_task_id = orig.start_task_id;
                }

                /* d) 실행 + 내부 이동비용 동적 합산 ----------------------------- */
                long long exec_and_inner = 0;
                for (size_t i = 0; i < cluster.task_ids.size(); ++i)
                {
                    /*   i-1 번째 → i 번째 이동비용(첫 태스크는 이동 없음)   */
                    const TASK *cur = nullptr;
                    const TASK *nxt = nullptr;

                    for (const auto &tp : active_tasks)
                    {
                        if (tp->id == cluster.task_ids[i])
                            cur = tp.get();
                        if (i + 1 < cluster.task_ids.size() &&
                            tp->id == cluster.task_ids[i + 1])
                            nxt = tp.get();
                    }
                    if (!cur)
                    {
                        exec_and_inner = INFINITE;
                        break;
                    }

                    exec_and_inner += cur->get_cost(rb.type); // 실행비
                    if (exec_and_inner >= INFINITE / 2)
                    {
                        exec_and_inner = INFINITE;
                        break;
                    }

                    if (nxt) // 내부 이동
                    {
                        std::vector<ROBOT::ACTION> tmpA;
                        std::vector<Coord> tmpC;
                        int inner = dijkstra(cur->coord, nxt->coord, rb,
                                             0, known_cost_map, known_object_map,
                                             known_cost_map.size(), tmpA, tmpC);

                        if (inner == INFINITE)
                        {
                            exec_and_inner = INFINITE;
                            break;
                        }

                        exec_and_inner += inner;
                        if (exec_and_inner >= INFINITE / 2)
                        {
                            exec_and_inner = INFINITE;
                            break;
                        }
                    }
                }
                if (exec_and_inner == INFINITE)
                    continue;

                /* e) 총 비용(경로+실행+내부) & 예상완료시각 ----------------------- */
                long long total_ll = static_cast<long long>(path_cost_to_cluster) + exec_and_inner; // cluster.total_cost 는 제외!

                if (total_ll > std::numeric_limits<int>::max())
                    continue;
                int total_cost = static_cast<int>(total_ll);

                double margin = (cluster.task_ids.size() >= 3) ? 1.10 : 1.05;
                if (rb.get_energy() < static_cast<int>(total_cost * margin))
                    continue;

                int finish_time = robotCurrentTaskEndTime.count(rb.id) ? robotCurrentTaskEndTime[rb.id] + total_cost : total_cost;

                std::cout << "     Robot " << rb.id << " (" << rb.type << ")  "
                          << "path " << path_cost_to_cluster
                          << " , exec+inner " << exec_and_inner
                          << " , total " << total_cost
                          << " , finish @ " << finish_time << '\n';

                if (finish_time < best_finish_time)
                {
                    best_finish_time = finish_time;
                    best_cluster_idx = cl_idx;
                    best_robot_id = rb.id;
                    task_clusters[cl_idx] = cluster; // ← 역순 결정 결과 반영
                    std::cout << "       ↳ new best so far\n";
                }
            } // robots
        } // clusters

        if (best_cluster_idx == -1)
        {
            /* 아직 split 허용 시점이 아니라면 → 일단 루틴 종료 */
            if (tick_counter_ < CLUSTER_SPLIT_ENABLE_TICK)
            {
                std::cout << "  ↪ splitting is deferred until "
                          << CLUSTER_SPLIT_ENABLE_TICK << " tick\n";
                break; // ← while 탈출·함수 종료. 다음 reassignment 때 재시도
            }

            std::cout << "\n  ‼ no assignable cluster ― splitting ...\n";

            std::vector<int> to_split(unassigned_clusters.begin(),
                                      unassigned_clusters.end());
            unassigned_clusters.clear();

            bool did_split = false;
            for (int idx : to_split)
            {
                const TaskCluster &big = task_clusters[idx];

                if (big.task_ids.size() == 1)
                {             // 이미 단일 → 다른 로봇으로는 진짜 불가
                    continue; // (unassigned 에 다시 넣지 X)
                }

                /* 단일 태스크로 분해 */
                for (int tid : big.task_ids)
                {
                    TaskCluster single;
                    single.task_ids = {tid};
                    single.start_task_id = single.end_task_id = tid;

                    /* 태스크 좌표 찾아 복사 */
                    Coord t_coord;
                    for (const auto &tp : active_tasks)
                        if (tp->id == tid)
                        {
                            t_coord = tp->coord;
                            break;
                        }

                    single.start_pos = single.end_pos = t_coord;
                    single.total_cost = 0; // 내부이동 0

                    task_clusters.push_back(single);
                    unassigned_clusters.insert((int)task_clusters.size() - 1);
                    did_split = true;
                    std::cout << "     ▸ created single-task cluster (T"
                              << tid << ", idx " << task_clusters.size() - 1 << ")\n";
                }
            }

            if (!did_split)
            {
                std::cout << "  ↪ every cluster truly unassignable → giving up on them\n";
                break; // while 탈출 — 더 돌려도 소득이 없으니 종료
            }

            continue; // split 성공 → 새 클러스터 대상으로 while 재진입
        }

        /* 2-b) 최적 (로봇,클러스터) 확정 → 큐/상태 갱신 ------------------------ */
        const TaskCluster &chosen = task_clusters[best_cluster_idx];

        std::cout << "\n  ◎ assign Cluster " << best_cluster_idx
                  << " to Robot " << best_robot_id
                  << "  (finish @" << best_finish_time << ")\n";

        for (int tid : chosen.task_ids)
            robotTaskQueue[best_robot_id].push(tid);

        robotToTask[best_robot_id] = chosen.start_task_id;
        robotCurrentTaskEndTime[best_robot_id] = best_finish_time;
        updateRobotPosition(best_robot_id, chosen.end_pos);

        unassigned_clusters.erase(best_cluster_idx);
    } // while

    std::cout << "\n=== Min-Min Assignment finished ===\n";
}

void Scheduler::performSufferageAssignment(const vector<shared_ptr<ROBOT>> &robots,
                                           const vector<shared_ptr<TASK>> &active_tasks,
                                           const vector<vector<vector<int>>> &known_cost_map,
                                           const vector<vector<OBJECT>> &known_object_map)
{
    std::cout << "\nPerforming Cluster-based Sufferage Assignment" << std::endl;

    // Clear existing assignments for reassignment
    for (auto &robot_queue : robotTaskQueue)
    {
        while (!robot_queue.second.empty())
        {
            robot_queue.second.pop();
        }
    }
    robotToTask.clear();
    robotCurrentTaskEndTime.clear();

    // Create set of unassigned clusters
    std::set<int> unassigned_clusters;
    for (size_t i = 0; i < task_clusters.size(); i++)
    {
        unassigned_clusters.insert(i);
    }

    while (!unassigned_clusters.empty())
    {
        int best_cluster_idx = -1;
        int best_robot_id = -1;
        int max_sufferage = -1;
        TaskCluster best_modified_cluster; // Store the best modified cluster

        std::cout << "\nFinding best cluster-robot pair from " << unassigned_clusters.size() << " unassigned clusters" << std::endl;

        // Calculate sufferage for each cluster
        for (int cluster_idx : unassigned_clusters)
        {
            const TaskCluster &cluster = task_clusters[cluster_idx];
            std::cout << "\nEvaluating Cluster " << cluster_idx << ":" << std::endl;
            std::cout << "  Tasks: ";
            for (int task_id : cluster.task_ids)
            {
                std::cout << task_id << " ";
            }
            std::cout << "\n  Start Task: " << cluster.start_task_id
                      << " at " << cluster.start_pos << std::endl;
            std::cout << "  End Task: " << cluster.end_task_id
                      << " at " << cluster.end_pos << std::endl;

            int min_time = std::numeric_limits<int>::max();
            int second_min_time = std::numeric_limits<int>::max();
            int min_robot_id = -1;
            TaskCluster current_modified_cluster; // Store the current modified cluster

            for (const auto &robot : robots)
            {
                if (robot->type == ROBOT::TYPE::DRONE ||
                    robot->get_status() == ROBOT::STATUS::EXHAUSTED)
                    continue;

                // Get robot's current position
                Coord robot_pos = robotExpectedPosition.count(robot->id) ? robotExpectedPosition[robot->id] : robot->get_coord();

                // Check if robot's expected position is valid
                if (known_object_map[robot_pos.x][robot_pos.y] == OBJECT::WALL)
                {
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
                    end_path_cost == std::numeric_limits<int>::max())
                {
                    continue;
                }

                // Create a modified cluster with the correct task order
                current_modified_cluster = cluster;
                int path_cost;

                if (start_path_cost <= end_path_cost)
                {
                    path_cost = start_path_cost;
                }
                else
                {
                    path_cost = end_path_cost;
                    std::reverse(current_modified_cluster.task_ids.begin(), current_modified_cluster.task_ids.end());
                    current_modified_cluster.start_pos = cluster.end_pos;
                    current_modified_cluster.end_pos = cluster.start_pos;
                    current_modified_cluster.start_task_id = cluster.end_task_id;
                    current_modified_cluster.end_task_id = cluster.start_task_id;
                }

                // Calculate total cost for the cluster including all tasks
                int total_cluster_cost = path_cost;
                for (size_t i = 0; i < current_modified_cluster.task_ids.size(); i++)
                {
                    // Add task execution cost
                    for (const auto &task : active_tasks)
                    {
                        if (task->id == current_modified_cluster.task_ids[i])
                        {
                            total_cluster_cost += task->get_cost(robot->type);
                            break;
                        }
                    }

                    // Add path cost to next task (if not the last task)
                    if (i < current_modified_cluster.task_ids.size() - 1)
                    {
                        const TASK *current_task = nullptr;
                        const TASK *next_task = nullptr;

                        for (const auto &task : active_tasks)
                        {
                            if (task->id == current_modified_cluster.task_ids[i])
                                current_task = task.get();
                            if (task->id == current_modified_cluster.task_ids[i + 1])
                                next_task = task.get();
                        }

                        if (current_task && next_task)
                        {
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

                            if (inter_path_cost == std::numeric_limits<int>::max())
                            {
                                total_cluster_cost = std::numeric_limits<int>::max();
                                break;
                            }
                            total_cluster_cost += inter_path_cost;
                        }
                    }
                }

                // Skip if total cost is too high
                if (total_cluster_cost > robot->get_energy() * 0.8)
                {
                    total_cluster_cost = std::numeric_limits<int>::max();
                }

                // Calculate completion time based on total cluster cost
                int completion_time = robotCurrentTaskEndTime.count(robot->id) ? robotCurrentTaskEndTime[robot->id] + total_cluster_cost : total_cluster_cost;

                if (completion_time == std::numeric_limits<int>::max())
                    continue;

                std::cout << "  Robot " << robot->id << " (" << robot->type << ") at " << robot_pos
                          << ": path cost = " << path_cost
                          << ", cluster cost = " << total_cluster_cost
                          << ", completion time = " << completion_time << std::endl;

                if (completion_time < min_time)
                {
                    second_min_time = min_time;
                    min_time = completion_time;
                    min_robot_id = robot->id;
                    current_modified_cluster = current_modified_cluster; // Store the best modified cluster for this robot
                }
                else if (completion_time < second_min_time)
                {
                    second_min_time = completion_time;
                }
            }

            // Calculate sufferage for this cluster
            if (min_time != std::numeric_limits<int>::max() &&
                second_min_time != std::numeric_limits<int>::max())
            {
                int sufferage = second_min_time - min_time;
                if (sufferage > max_sufferage)
                {
                    max_sufferage = sufferage;
                    best_cluster_idx = cluster_idx;
                    best_robot_id = min_robot_id;
                    best_modified_cluster = current_modified_cluster; // Store the best modified cluster
                    std::cout << "  -> New best assignment found! (sufferage: " << sufferage << ")" << std::endl;
                }
            }
        }

        if (best_cluster_idx == -1)
        {
            std::cout << "No valid assignments possible" << std::endl;
            break;
        }

        std::cout << "\nAssigning Cluster " << best_cluster_idx << " to Robot " << best_robot_id
                  << " (sufferage: " << max_sufferage << ")" << std::endl;

        // Assign all tasks in the cluster to the robot
        for (int task_id : best_modified_cluster.task_ids)
        {
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

void Scheduler::clusterTasks(const vector<shared_ptr<TASK>> &active_tasks,
                             const vector<shared_ptr<ROBOT>> &robots,
                             const vector<vector<vector<int>>> &known_cost_map,
                             const vector<vector<OBJECT>> &known_object_map)
{
    task_clusters.clear();

    // 아직 클러스터링되지 않은 태스크들을 추적
    set<int> unclustered_tasks;
    for (const auto &task : active_tasks)
    {
        if (!task->is_done())
        {
            unclustered_tasks.insert(task->id);
        }
    }

    // WHEEL과 CATERPILLAR 로봇 찾기
    shared_ptr<ROBOT> wheel_robot = nullptr;
    shared_ptr<ROBOT> caterpillar_robot = nullptr;
    for (const auto &robot : robots)
    {
        if (robot->type == ROBOT::TYPE::WHEEL)
        {
            wheel_robot = robot;
        }
        else if (robot->type == ROBOT::TYPE::CATERPILLAR)
        {
            caterpillar_robot = robot;
        }
    }
    if (!wheel_robot || !caterpillar_robot)
    {
        std::cout << "Both WHEEL and CATERPILLAR robots are required for clustering" << std::endl;
        return;
    }

    while (!unclustered_tasks.empty())
    {
        TaskCluster new_cluster;
        int seed_task_id = *unclustered_tasks.begin();
        new_cluster.task_ids.push_back(seed_task_id);
        unclustered_tasks.erase(seed_task_id);

        // 현재 클러스터에 추가할 수 있는 태스크 찾기
        bool cluster_growing = true;
        while (cluster_growing)
        {
            cluster_growing = false;
            int best_task_id = -1;
            int min_cost = CLUSTER_DISTANCE_THRESHOLD;

            // 현재 클러스터의 모든 태스크에 대해
            for (int current_task_id : new_cluster.task_ids)
            {
                // 아직 클러스터링되지 않은 모든 태스크에 대해
                for (int candidate_id : unclustered_tasks)
                {
                    // 현재 태스크와 후보 태스크 사이의 비용 계산
                    const TASK *current_task = nullptr;
                    const TASK *candidate_task = nullptr;

                    for (const auto &task : active_tasks)
                    {
                        if (task->id == current_task_id)
                            current_task = task.get();
                        if (task->id == candidate_id)
                            candidate_task = task.get();
                    }

                    if (!current_task || !candidate_task)
                        continue;

                    std::vector<ROBOT::ACTION> wheel_path_actions, caterpillar_path_actions;
                    std::vector<Coord> wheel_path_coords, caterpillar_path_coords;

                    int wheel_path_cost = dijkstra(current_task->coord,
                                                   candidate_task->coord,
                                                   *wheel_robot,
                                                   0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                                   known_cost_map,
                                                   known_object_map,
                                                   known_cost_map.size(),
                                                   wheel_path_actions,
                                                   wheel_path_coords);

                    int caterpillar_path_cost = dijkstra(current_task->coord,
                                                         candidate_task->coord,
                                                         *caterpillar_robot,
                                                         0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                                         known_cost_map,
                                                         known_object_map,
                                                         known_cost_map.size(),
                                                         caterpillar_path_actions,
                                                         caterpillar_path_coords);

                    // 두 로봇 타입 중 더 큰 비용을 사용
                    int path_cost = std::max(wheel_path_cost, caterpillar_path_cost);

                    if (path_cost < min_cost)
                    {
                        min_cost = path_cost;
                        best_task_id = candidate_id;
                    }
                }
            }

            // 가장 가까운 태스크를 클러스터에 추가
            if (best_task_id != -1)
            {
                // Check size limit before adding the task
                if (new_cluster.task_ids.size() >= MAX_CLUSTER_SIZE)
                {
                    break; // Stop if we've reached the maximum cluster size
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
        if (new_cluster.task_ids.size() > 1)
        {
            for (size_t i = 0; i < new_cluster.task_ids.size() - 1; i++)
            {
                const TASK *current_task = nullptr;
                const TASK *next_task = nullptr;

                for (const auto &task : active_tasks)
                {
                    if (task->id == new_cluster.task_ids[i])
                        current_task = task.get();
                    if (task->id == new_cluster.task_ids[i + 1])
                        next_task = task.get();
                }

                if (!current_task || !next_task)
                    continue;

                std::vector<ROBOT::ACTION> wheel_path_actions, caterpillar_path_actions;
                std::vector<Coord> wheel_path_coords, caterpillar_path_coords;

                int wheel_path_cost = dijkstra(current_task->coord,
                                               next_task->coord,
                                               *wheel_robot,
                                               0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                               known_cost_map,
                                               known_object_map,
                                               known_cost_map.size(),
                                               wheel_path_actions,
                                               wheel_path_coords);

                int caterpillar_path_cost = dijkstra(current_task->coord,
                                                     next_task->coord,
                                                     *caterpillar_robot,
                                                     0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                                     known_cost_map,
                                                     known_object_map,
                                                     known_cost_map.size(),
                                                     caterpillar_path_actions,
                                                     caterpillar_path_coords);

                // 두 로봇 타입 중 더 큰 비용을 사용
                int path_cost = std::max(wheel_path_cost, caterpillar_path_cost);

                new_cluster.total_cost += path_cost;
            }
        }
        else
        {
            // 태스크가 하나일 경우, 두 로봇 타입 중 더 큰 실행 비용을 클러스터 비용으로 설정
            for (const auto &task : active_tasks)
            {
                if (task->id == new_cluster.task_ids[0])
                {
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

void Scheduler::findClusterEndpoints(TaskCluster &cluster,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map)
{
    if (cluster.task_ids.empty())
        return;

    // 태스크가 하나일 경우, 해당 태스크의 위치를 시작점과 끝점으로 설정
    if (cluster.task_ids.size() == 1)
    {
        for (const auto &task : active_tasks)
        {
            if (task->id == cluster.task_ids[0])
            {
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
    for (const auto &robot : robots)
    {
        if (robot->type == ROBOT::TYPE::WHEEL)
        {
            wheel_robot = robot;
        }
        else if (robot->type == ROBOT::TYPE::CATERPILLAR)
        {
            caterpillar_robot = robot;
        }
    }
    if (!wheel_robot || !caterpillar_robot)
    {
        std::cout << "Both WHEEL and CATERPILLAR robots are required for finding cluster endpoints" << std::endl;
        return;
    }

    // 모든 태스크 쌍에 대한 거리 계산
    map<pair<int, int>, int> distances;
    for (int i = 0; i < cluster.task_ids.size(); i++)
    {
        for (int j = i + 1; j < cluster.task_ids.size(); j++)
        {
            const TASK *task1 = nullptr;
            const TASK *task2 = nullptr;

            for (const auto &task : active_tasks)
            {
                if (task->id == cluster.task_ids[i])
                    task1 = task.get();
                if (task->id == cluster.task_ids[j])
                    task2 = task.get();
            }

            if (!task1 || !task2)
                continue;

            std::vector<ROBOT::ACTION> wheel_path_actions, caterpillar_path_actions;
            std::vector<Coord> wheel_path_coords, caterpillar_path_coords;

            int wheel_path_cost = dijkstra(task1->coord,
                                           task2->coord,
                                           *wheel_robot,
                                           0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                                           known_cost_map,
                                           known_object_map,
                                           known_cost_map.size(),
                                           wheel_path_actions,
                                           wheel_path_coords);

            int caterpillar_path_cost = dijkstra(task1->coord,
                                                 task2->coord,
                                                 *caterpillar_robot,
                                                 0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
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
    for (const auto &dist : distances)
    {
        if (dist.second > max_distance)
        {
            max_distance = dist.second;
            cluster.start_task_id = dist.first.first;
            cluster.end_task_id = dist.first.second;
        }
    }

    // 시작점과 끝점의 좌표 설정
    for (const auto &task : active_tasks)
    {
        if (task->id == cluster.start_task_id)
        {
            cluster.start_pos = task->coord;
        }
        if (task->id == cluster.end_task_id)
        {
            cluster.end_pos = task->coord;
        }
    }
}

int Scheduler::calculatePathCost(const Coord &start, const Coord &end,
                                 const ROBOT &robot,
                                 const vector<vector<vector<int>>> &known_cost_map,
                                 const vector<vector<OBJECT>> &known_object_map)
{
    std::vector<ROBOT::ACTION> path_actions;
    std::vector<Coord> path_coords;

    return dijkstra(start, end, robot, 0, // task_cost는 0으로 설정 (순수 이동 비용만 계산)
                    known_cost_map, known_object_map,
                    known_cost_map.size(),
                    path_actions, path_coords);
}

bool Scheduler::DRONE_coord_equal(const Coord &a, const Coord &b) { return a.x == b.x && a.y == b.y; }

void Scheduler::DRONE_init_tiles(const std::vector<std::vector<OBJECT>> &known_object_map)
{
    if (DRONE_map_size != -1)
        return;
    DRONE_map_size = static_cast<int>(known_object_map.size());
    DRONE_tile_rows = (DRONE_map_size + DRONE_tile_size - 1) / DRONE_tile_size;
    DRONE_tile_cols = DRONE_tile_rows;
    DRONE_tiles.assign(DRONE_tile_rows, std::vector<DRONE_TileInfo>(DRONE_tile_cols));

    for (int i = 0; i < DRONE_tile_rows; ++i)
    {
        for (int j = 0; j < DRONE_tile_cols; ++j)
        {
            int cx = j * DRONE_tile_size + DRONE_tile_range;
            int cy = i * DRONE_tile_size + DRONE_tile_range;
            cx = std::min(cx, DRONE_map_size - 1);
            cy = std::min(cy, DRONE_map_size - 1);
            if (known_object_map[cx][cy] == OBJECT::WALL)
            {
                bool found = false;
                for (int d = 1; d <= DRONE_tile_range + 1 && !found; ++d)
                {
                    for (int dx = -d; dx <= d && !found; ++dx)
                        for (int dy = -d; dy <= d && !found; ++dy)
                        {
                            int nx = cx + dx, ny = cy + dy;
                            if (nx >= 0 && nx < DRONE_map_size && ny >= 0 && ny < DRONE_map_size &&
                                known_object_map[nx][ny] != OBJECT::WALL)
                            {
                                cx = nx;
                                cy = ny;
                                found = true;
                            }
                        }
                }
            }
            DRONE_tiles[i][j].center = Coord(cx, cy);
        }
    }
}
void Scheduler::DRONE_update_tile_info(const std::vector<std::vector<OBJECT>> &known_object_map)
{
    for (int i = 0; i < DRONE_tile_rows; ++i)
    {
        for (int j = 0; j < DRONE_tile_cols; ++j)
        {
            int ux = DRONE_tiles[i][j].center.x - DRONE_tile_range;
            int uy = DRONE_tiles[i][j].center.y - DRONE_tile_range;
            int unseen = 0;
            for (int dx = 0; dx < DRONE_tile_size; ++dx)
                for (int dy = 0; dy < DRONE_tile_size; ++dy)
                {
                    int x = ux + dx, y = uy + dy;
                    if (x >= 0 && x < DRONE_map_size && y >= 0 && y < DRONE_map_size &&
                        known_object_map[x][y] == OBJECT::UNKNOWN)
                        unseen++;
                }
            DRONE_tiles[i][j].unseen_cells = unseen;
        }
    }
}

std::vector<Coord> Scheduler::DRONE_plan_path(const Coord &start, const Coord &goal,
                                              const std::vector<std::vector<std::vector<int>>> &known_cost_map,
                                              const ROBOT &robot,
                                              const std::vector<std::vector<OBJECT>> &known_object_map)
{
    if (robot.type == ROBOT::TYPE::DRONE)
    {
        std::vector<std::vector<bool>> vis(DRONE_map_size, std::vector<bool>(DRONE_map_size, false));
        std::vector<std::vector<Coord>> prev(DRONE_map_size, std::vector<Coord>(DRONE_map_size, Coord(-1, -1)));
        std::queue<Coord> q;
        vis[start.x][start.y] = true;
        q.push(start);
        static const int dx[4] = {0, 0, -1, 1}, dy[4] = {1, -1, 0, 0};
        while (!q.empty())
        {
            Coord cur = q.front();
            q.pop();
            if (DRONE_coord_equal(cur, goal))
                break;
            for (int d = 0; d < 4; ++d)
            {
                int nx = cur.x + dx[d], ny = cur.y + dy[d];
                if (nx < 0 || ny < 0 || nx >= DRONE_map_size || ny >= DRONE_map_size)
                    continue;
                if (known_object_map[nx][ny] == OBJECT::WALL)
                    continue;
                if (vis[nx][ny])
                    continue;
                vis[nx][ny] = true;
                prev[nx][ny] = cur;
                q.push(Coord(nx, ny));
            }
        }
        std::vector<Coord> path;
        Coord p = goal;
        if (prev[p.x][p.y].x == -1 && !DRONE_coord_equal(start, goal))
            return path;
        while (!DRONE_coord_equal(p, start))
        {
            path.push_back(p);
            p = prev[p.x][p.y];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    /* ground 로봇일 경우 기존 dijkstra 호출 */
    std::vector<ROBOT::ACTION> tmpA;
    std::vector<Coord> tmpC;

    int cost = dijkstra(start, goal, robot, 0, known_cost_map, known_object_map,
                        static_cast<int>(known_cost_map.size()), tmpA, tmpC);
    if (cost == std::numeric_limits<int>::max())
        return {};
    return tmpC; // coordinates 포함 - 첫 항이 start 이므로 pop_front 필요
}

ROBOT::ACTION Scheduler::DRONE_get_direction(const Coord &from, const Coord &to)
{
    int dx = to.x - from.x, dy = to.y - from.y;
    if (dx == 0 && dy == 1)
        return ROBOT::ACTION::UP;
    if (dx == 0 && dy == -1)
        return ROBOT::ACTION::DOWN;
    if (dx == -1 && dy == 0)
        return ROBOT::ACTION::LEFT;
    if (dx == 1 && dy == 0)
        return ROBOT::ACTION::RIGHT;
    return ROBOT::ACTION::HOLD;
}
bool Scheduler::DRONE_is_exploration_time(const vector<vector<vector<int>>> &known_cost_map) const
{
    int map_size_d = int(known_cost_map.size());
    return (tick_counter_ < DRONE_exploration_pause_start) ||
           (tick_counter_ >= DRONE_exploration_resume && tick_counter_ <= map_size_d * 57.5 - DRONE_exploration_pause_start + DRONE_exploration_resume);
}