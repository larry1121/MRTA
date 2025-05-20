#include "schedular.h"
#include <limits>
#include <queue>
#include <algorithm>
#include <cstdio> 
#include <cmath>  
#include <vector> 

bool Scheduler::coord_equal(const Coord& a, const Coord& b) {
    return a.x == b.x && a.y == b.y;
}

void Scheduler::init_scheduler_state(const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map) {
    if (map_size == -1 && !known_object_map.empty()) {
        map_size = static_cast<int>(known_object_map.size());
    }

    if (learned_uniform_drone_cost_ == -1 && map_size > 0) {
        for (int r = 0; r < map_size; ++r) {
            for (int c = 0; c < map_size; ++c) {
                if (known_object_map[r][c] != OBJECT::WALL && known_object_map[r][c] != OBJECT::UNKNOWN) {
                    int cost = known_cost_map[r][c][static_cast<int>(ROBOT::TYPE::DRONE)];
                    if (cost > 0 && cost < 5000) { // Valid cost found
                        learned_uniform_drone_cost_ = cost;
                        // printf("[DEBUG] Learned uniform drone cost: %d\n", learned_uniform_drone_cost_);
                        return; // Found, no need to search further
                    }
                }
            }
        }
    }
}

std::vector<Coord> Scheduler::plan_path(const Coord& start, const Coord& goal,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map, ROBOT::TYPE type,
    const std::vector<std::vector<OBJECT>>& known_object_map) {
    if (map_size == -1) return {};
    if (start.x < 0 || start.x >= map_size || start.y < 0 || start.y >= map_size ||
        goal.x < 0 || goal.x >= map_size || goal.y < 0 || goal.y >= map_size) {
        return {};
    }

    typedef std::pair<int, Coord> PQItem;
    std::vector<std::vector<int>> dist(map_size, std::vector<int>(map_size, std::numeric_limits<int>::max() / 4));
    std::vector<std::vector<Coord>> prev(map_size, std::vector<Coord>(map_size, Coord(-1, -1)));
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

    dist[start.x][start.y] = 0;
    pq.push({ 0, start });

    static const int dx[] = { 0, 0, -1, 1 };
    static const int dy[] = { 1, -1, 0, 0 };

    while (!pq.empty()) {
        int cost = pq.top().first;
        Coord cur = pq.top().second;
        pq.pop();

        if (cost > dist[cur.x][cur.y]) continue;
        if (coord_equal(cur, goal)) break;

        for (int i = 0; i < 4; ++i) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= map_size || ny >= map_size) continue;
            if (known_object_map[nx][ny] == OBJECT::WALL) continue;

            int ncost_val = known_cost_map[nx][ny][static_cast<int>(type)];
            if (type == ROBOT::TYPE::DRONE && learned_uniform_drone_cost_ > 0) {
                if (known_object_map[nx][ny] != OBJECT::UNKNOWN) { // If cell type is known and not wall, use uniform cost
                    ncost_val = learned_uniform_drone_cost_;
                }
                else { // For unknown cells, planner still uses high cost
                    ncost_val = 10000;
                }
            }
            else if (ncost_val == -1) {
                ncost_val = 10000; // Path planner's cost for unknown if not drone or uniform cost not learned
            }

            if (ncost_val >= std::numeric_limits<int>::max() / 2) continue;

            int alt = cost + ncost_val;
            if (alt < dist[nx][ny]) {
                dist[nx][ny] = alt;
                prev[nx][ny] = cur;
                pq.push({ alt, Coord(nx, ny) });
            }
        }
    }

    std::vector<Coord> path_result;
    Coord p = goal;
    if (coord_equal(start, goal)) return path_result;
    if (prev[p.x][p.y].x == -1 && prev[p.x][p.y].y == -1 && !coord_equal(start, p)) return path_result;

    while (!coord_equal(p, start)) {
        if (p.x == -1 || p.y == -1) {
            path_result.clear();
            break;
        }
        path_result.push_back(p);
        p = prev[p.x][p.y];
        if (path_result.size() > static_cast<size_t>(map_size * map_size)) {
            path_result.clear();
            break;
        }
    }
    std::reverse(path_result.begin(), path_result.end());
    return path_result;
}

ROBOT::ACTION Scheduler::get_direction(const Coord& from, const Coord& to) {
    int dx_val = to.x - from.x;
    int dy_val = to.y - from.y;
    if (dx_val == 0 && dy_val == 1) return ROBOT::ACTION::UP;
    if (dx_val == 0 && dy_val == -1) return ROBOT::ACTION::DOWN;
    if (dx_val == -1 && dy_val == 0) return ROBOT::ACTION::LEFT;
    if (dx_val == 1 && dy_val == 0) return ROBOT::ACTION::RIGHT;
    return ROBOT::ACTION::HOLD;
}

int Scheduler::count_observable_unknowns(const Coord& pos,
    const std::vector<std::vector<OBJECT>>& known_map) const {
    if (map_size == -1) return 0;
    int unknowns = 0;
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            int nx = pos.x + dx;
            int ny = pos.y + dy;
            if (nx < 0 || nx >= map_size || ny < 0 || ny >= map_size) continue;
            if (known_map[nx][ny] == OBJECT::UNKNOWN) {
                unknowns++;
            }
        }
    }
    return unknowns;
}

long long Scheduler::calculate_path_energy(const std::vector<Coord>& path,
    const Coord& start_pos,
    ROBOT::TYPE robot_type,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map) const {
    if (map_size == -1) return 0;
    if (path.empty()) return 0; // No movement, no energy cost for path itself

    long long total_energy = 0;

    if (robot_type == ROBOT::TYPE::DRONE && learned_uniform_drone_cost_ > 0) {
        // If uniform drone cost is learned, each step costs this uniform value.
        // Energy for one step A to B = (cost_A/2 + cost_B/2). If cost_A = cost_B = learned_uniform_drone_cost_, then energy = learned_uniform_drone_cost_
        total_energy = static_cast<long long>(path.size()) * learned_uniform_drone_cost_;
    }
    else {
        // Fallback to detailed calculation if not drone or uniform cost not learned
        Coord current_cell = start_pos;
        int robot_type_idx = static_cast<int>(robot_type);
        const int default_cost_for_energy_calc = (robot_type == ROBOT::TYPE::DRONE) ? 150 : 5000; // Higher default for non-drones if cost unknown

        for (const auto& next_cell : path) {
            if (current_cell.x < 0 || current_cell.x >= map_size || current_cell.y < 0 || current_cell.y >= map_size ||
                next_cell.x < 0 || next_cell.x >= map_size || next_cell.y < 0 || next_cell.y >= map_size) {
                return std::numeric_limits<long long>::max();
            }

            int cost_current_cell = known_cost_map[current_cell.x][current_cell.y][robot_type_idx];
            int cost_next_cell = known_cost_map[next_cell.x][next_cell.y][robot_type_idx];

            if (cost_current_cell == -1 || cost_current_cell > 5000) cost_current_cell = default_cost_for_energy_calc;
            if (cost_next_cell == -1 || cost_next_cell > 5000) cost_next_cell = default_cost_for_energy_calc;

            total_energy += (static_cast<long long>(cost_current_cell) / 2) + (static_cast<long long>(cost_next_cell) / 2);
            current_cell = next_cell;
        }
    }
    return total_energy;
}

void Scheduler::on_info_updated(const std::set<Coord>&,
    const std::set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map,
    const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>& robots)
{
    init_scheduler_state(known_object_map, known_cost_map);
    if (map_size == -1) return;

    cell_reserved.clear();
    for (const auto& robot_ptr_for_res : robots) {
        if (robot_ptr_for_res->type == ROBOT::TYPE::DRONE) {
            int temp_rid = robot_ptr_for_res->id;
            if (drone_targets.count(temp_rid) && robot_ptr_for_res->get_energy() > 0 &&
                drone_paths.count(temp_rid) && !drone_paths[temp_rid].empty() &&
                !coord_equal(drone_targets[temp_rid], robot_ptr_for_res->get_coord())) {
                cell_reserved.insert(drone_targets[temp_rid]);
            }
        }
    }


    for (const auto& robot_ptr : robots) {
        if (robot_ptr->type != ROBOT::TYPE::DRONE) continue;

        int rid = robot_ptr->id;
        Coord drone_pos = robot_ptr->get_coord();
        int current_energy = robot_ptr->get_energy();

        if (current_energy <= 0) {
            if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid])) {
                cell_reserved.erase(drone_targets[rid]);
            }
            drone_paths[rid].clear();
            drone_targets[rid] = drone_pos;
            continue;
        }

        // Always re-evaluate for drones to find the best current option
        bool old_target_was_reserved = false;
        Coord old_target_coord;
        if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid])) {
            old_target_was_reserved = true;
            old_target_coord = drone_targets[rid];
            cell_reserved.erase(drone_targets[rid]); // Temporarily un-reserve for self re-evaluation
        }
        drone_paths[rid].clear(); // Force re-planning


        std::vector<Coord> candidate_target_coords;
        for (int x = 0; x < map_size; ++x) {
            for (int y = 0; y < map_size; ++y) {
                Coord current_candidate(x, y);
                if (known_object_map[x][y] == OBJECT::WALL) continue;
                if (count_observable_unknowns(current_candidate, known_object_map) > 0) {
                    candidate_target_coords.push_back(current_candidate);
                }
            }
        }

        std::vector<Coord> best_path_for_drone;
        Coord best_target_for_drone = drone_pos;
        double max_score = -1.0;

        for (const auto& candidate_coord : candidate_target_coords) {
            if (cell_reserved.count(candidate_coord)) { // Check if reserved by ANOTHER drone
                continue;
            }

            std::vector<Coord> current_path = plan_path(drone_pos, candidate_coord, known_cost_map, ROBOT::TYPE::DRONE, known_object_map);

            if (current_path.empty() && !coord_equal(drone_pos, candidate_coord)) continue;

            long long path_total_energy = calculate_path_energy(current_path, drone_pos, ROBOT::TYPE::DRONE, known_cost_map);

            if (path_total_energy > current_energy) {
                continue;
            }

            int num_unknowns = count_observable_unknowns(candidate_coord, known_object_map);
            double current_score = 0.0;

            if (num_unknowns > 0) {
                if (current_path.empty()) { // Already at a target that reveals unknowns
                    current_score = static_cast<double>(num_unknowns) * 1000.0; // High score for 0-cost exploration
                }
                else {
                    current_score = static_cast<double>(num_unknowns) / (path_total_energy + 1.0);
                }
            }

            if (current_score > max_score) {
                max_score = current_score;
                best_path_for_drone = current_path;
                best_target_for_drone = candidate_coord;
            }
        }

        // If the old target was better and this drone re-selected it, re-reserve it.
        // Otherwise, if a new target is chosen, it will be reserved.
        // If no target chosen, old reservation (if any) remains un-reserved.
        // This is slightly complex; the global rebuild of cell_reserved at start is simpler.

        if (max_score > 0) {
            drone_paths[rid].assign(best_path_for_drone.begin(), best_path_for_drone.end());
            drone_targets[rid] = best_target_for_drone;
            // The global cell_reserved rebuild at the start of on_info_updated handles reservations for chosen targets.
            // No need to individually add here if we rely on that global rebuild.
            // For immediate effect within this loop for subsequent drones, we could add it:
            if (!coord_equal(drone_pos, best_target_for_drone) || best_path_for_drone.empty()) {
                cell_reserved.insert(best_target_for_drone); // Reserve the new target
            }
            // printf("[DEBUG] Drone %d: Path set. Target=(%d,%d), Score=%.2f, Energy=%lld, Len=%zu\n",
            //        rid, best_target_for_drone.x, best_target_for_drone.y, max_score, 
            //        calculate_path_energy(best_path_for_drone, drone_pos, ROBOT::TYPE::DRONE, known_cost_map), 
            //        best_path_for_drone.size());

        }
        else {
            // No suitable path found. If old target was unreserved, it stays unreserved.
            drone_paths[rid].clear();
            drone_targets[rid] = drone_pos;
            // printf("[DEBUG] Drone %d: NO suitable path found.\n", rid);
            if (old_target_was_reserved && !cell_reserved.count(old_target_coord)) {
                // If it was this drone's, and it's not picked again, it should not be globally reserved by this drone
                // This case is complex due to order. Safer to rely on global rebuild or specific logic.
                // For now, if no new path, the drone_targets[rid] = drone_pos, which won't be added to cell_reserved
                // if check `!coord_equal(drone_targets[temp_rid], robot_ptr_for_res->get_coord())` is used in rebuild.
            }
        }
    }
}

bool Scheduler::on_task_reached(const std::set<Coord>&,
    const std::set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>&,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&,
    const ROBOT& robot,
    const TASK&)
{
    if (robot.type == ROBOT::TYPE::DRONE) {
        return false;
    }
    return true;
}

ROBOT::ACTION Scheduler::idle_action(const std::set<Coord>&,
    const std::set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&,
    const ROBOT& robot)
{
    if (map_size == -1 && !known_object_map.empty()) {
        init_scheduler_state(known_object_map, {}); // Pass empty cost map if only for map_size
    }
    if (map_size == -1) return ROBOT::ACTION::HOLD;


    if (robot.type != ROBOT::TYPE::DRONE) return ROBOT::ACTION::HOLD;

    int rid = robot.id;
    Coord cur = robot.get_coord();

    if (drone_paths.count(rid) && !drone_paths[rid].empty()) {
        Coord next_target_in_path = drone_paths[rid].front();

        if (coord_equal(cur, next_target_in_path)) {
            drone_paths[rid].pop_front();
            if (drone_paths[rid].empty()) {
                // Target reached, unreserve if it was this drone's target
                // This unreserving is tricky because cell_reserved is rebuilt globally.
                // It's better that on_info_updated handles reservation state accurately.
                return ROBOT::ACTION::HOLD;
            }
            next_target_in_path = drone_paths[rid].front();
        }

        if (next_target_in_path.x < 0 || next_target_in_path.x >= map_size || next_target_in_path.y < 0 || next_target_in_path.y >= map_size ||
            known_object_map[next_target_in_path.x][next_target_in_path.y] == OBJECT::WALL) {

            // Path blocked, clear path and target reservation. Target will be re-evaluated.
            if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid])) {
                //This specific erase might be problematic if cell_reserved is purely managed at start of on_info_updated
                //cell_reserved.erase(drone_targets[rid]); 
            }
            drone_paths[rid].clear();
            drone_targets[rid] = cur;
            // printf("[DEBUG] Drone %d: Path to %d,%d blocked in idle_action. Holding.\n", rid, next_target_in_path.x, next_target_in_path.y);
            return ROBOT::ACTION::HOLD;
        }
        return get_direction(cur, next_target_in_path);
    }
    return ROBOT::ACTION::HOLD;
}