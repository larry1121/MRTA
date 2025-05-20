#include "schedular.h"
#include <limits>
#include <queue>
#include <algorithm>
#include <cstdio> // For printf
#include <cmath>  // For ceil
#include <vector> // Ensure vector is included for vector operations

// Helper to check coordinate equality
bool Scheduler::coord_equal(const Coord& a, const Coord& b) {
    return a.x == b.x && a.y == b.y;
}

// Initialize map_size if not already done
void Scheduler::init_map_size(const std::vector<std::vector<OBJECT>>& known_object_map) {
    if (map_size != -1) return;
    if (!known_object_map.empty()) {
        map_size = static_cast<int>(known_object_map.size());
    }
}

// Path planning using A*
std::vector<Coord> Scheduler::plan_path(const Coord& start, const Coord& goal,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map, ROBOT::TYPE type, // Corrected: ROBOT::TYPE
    const std::vector<std::vector<OBJECT>>& known_object_map) {
    if (map_size == -1) {
        // Attempt to initialize map_size if it wasn't, though ideally it should be.
        if (!known_object_map.empty()) init_map_size(known_object_map);
        if (map_size == -1) return {}; // Still not initialized, cannot proceed
    }
    if (start.x < 0 || start.x >= map_size || start.y < 0 || start.y >= map_size ||
        goal.x < 0 || goal.x >= map_size || goal.y < 0 || goal.y >= map_size) {
        // Start or goal is out of bounds
        return {};
    }


    typedef std::pair<int, Coord> PQItem; // {cost, coordinate}
    std::vector<std::vector<int>> dist(map_size, std::vector<int>(map_size, std::numeric_limits<int>::max() / 4));
    std::vector<std::vector<Coord>> prev(map_size, std::vector<Coord>(map_size, Coord(-1, -1)));
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

    dist[start.x][start.y] = 0;
    pq.push({ 0, start });

    // Corrected dx/dy for actions: UP, DOWN, LEFT, RIGHT (map to simulator.h action definitions)
    // simulator.h: ACTION::UP (0,1), DOWN (0,-1), LEFT (-1,0), RIGHT (1,0)
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
            if (ncost_val == -1) ncost_val = 10000;
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
    if (prev[p.x][p.y].x == -1 && prev[p.x][p.y].y == -1 && !coord_equal(start, p)) return path_result; // No path

    while (!coord_equal(p, start)) {
        if (p.x == -1 || p.y == -1) { // Path reconstruction failed
            path_result.clear();
            break;
        }
        path_result.push_back(p);
        p = prev[p.x][p.y];
        if (path_result.size() > static_cast<size_t>(map_size * map_size)) { // Safety break for too long path
            path_result.clear();
            break;
        }
    }
    std::reverse(path_result.begin(), path_result.end());
    return path_result;
}

// Get action type from two coordinates
ROBOT::ACTION Scheduler::get_direction(const Coord& from, const Coord& to) { // Corrected: ROBOT::ACTION
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
    ROBOT::TYPE robot_type, // Corrected: ROBOT::TYPE
    const std::vector<std::vector<std::vector<int>>>& known_cost_map) const {
    if (map_size == -1 || path.empty()) return 0;

    long long total_energy = 0;
    Coord current_cell = start_pos;
    int robot_type_idx = static_cast<int>(robot_type);

    for (const auto& next_cell : path) {
        if (current_cell.x < 0 || current_cell.x >= map_size || current_cell.y < 0 || current_cell.y >= map_size ||
            next_cell.x < 0 || next_cell.x >= map_size || next_cell.y < 0 || next_cell.y >= map_size) {
            return std::numeric_limits<long long>::max(); // Invalid path component
        }

        int cost_current_cell = known_cost_map[current_cell.x][current_cell.y][robot_type_idx];
        int cost_next_cell = known_cost_map[next_cell.x][next_cell.y][robot_type_idx];

        // Default cost for drone if unknown or planner's high cost
        const int default_drone_cost_for_energy_calc = 150;
        if (robot_type == ROBOT::TYPE::DRONE) {
            if (cost_current_cell == -1 || cost_current_cell > 5000) cost_current_cell = default_drone_cost_for_energy_calc;
            if (cost_next_cell == -1 || cost_next_cell > 5000) cost_next_cell = default_drone_cost_for_energy_calc;
        }
        else { // For other types, if -1, it's an issue, but we must provide a value.
            if (cost_current_cell == -1) cost_current_cell = 10000; // Fallback, should be known
            if (cost_next_cell == -1) cost_next_cell = 10000;
        }


        total_energy += (static_cast<long long>(cost_current_cell) / 2) + (static_cast<long long>(cost_next_cell) / 2);
        current_cell = next_cell;
    }
    return total_energy;
}

void Scheduler::on_info_updated(const std::set<Coord>& /*observed_coords*/,
    const std::set<Coord>& /*updated_coords*/,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map,
    const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>& /*active_tasks*/,
    const std::vector<std::shared_ptr<ROBOT>>& robots) // Corrected signature
{
    init_map_size(known_object_map);
    if (map_size == -1) return;

    // Rebuild cell_reserved based on current valid drone_targets
    // This ensures that if a drone target becomes invalid (e.g. drone exhausted), it's not blocking others.
    cell_reserved.clear();
    for (const auto& robot_ptr_for_res : robots) {
        if (robot_ptr_for_res->type == ROBOT::TYPE::DRONE) {
            int temp_rid = robot_ptr_for_res->id;
            if (drone_targets.count(temp_rid) && !coord_equal(drone_targets[temp_rid], robot_ptr_for_res->get_coord())) {
                // Only reserve if it's a distinct target, not just current pos of an idle/exhausted drone
                if (robot_ptr_for_res->get_energy() > 0 && drone_paths.count(temp_rid) && !drone_paths[temp_rid].empty()) {
                    cell_reserved.insert(drone_targets[temp_rid]);
                }
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

        // If drone has a path, and its target is still good (e.g. has unknowns), let it continue
        // Otherwise, force re-planning by clearing path.
        if (drone_paths.count(rid) && !drone_paths[rid].empty()) {
            bool replan = false;
            if (drone_targets.count(rid)) {
                Coord current_planned_target = drone_targets[rid];
                if (count_observable_unknowns(current_planned_target, known_object_map) == 0) {
                    replan = true; // Target no longer provides new info
                }
            }
            else { // No target associated with path - should not happen
                replan = true;
            }
            if (replan) {
                if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid])) {
                    cell_reserved.erase(drone_targets[rid]);
                }
                drone_paths[rid].clear();
                // drone_targets will be set below if new path found
            }
            else {
                continue; // Stick to current path
            }
        }


        std::vector<Coord> candidate_target_coords;
        for (int x = 0; x < map_size; ++x) {
            for (int y = 0; y < map_size; ++y) {
                Coord current_candidate(x, y);
                if (known_object_map[x][y] == OBJECT::WALL) continue;
                // Check reservation later, only for the chosen best_target.
                // Allow considering reserved cells for scoring, but final choice cannot be a cell reserved by another active drone.

                if (count_observable_unknowns(current_candidate, known_object_map) > 0) {
                    candidate_target_coords.push_back(current_candidate);
                }
            }
        }

        std::vector<Coord> best_path_for_drone;
        Coord best_target_for_drone = drone_pos; // Default to current position
        double max_score = -1.0;

        // Temporarily un-reserve this drone's current target if it had one, to allow re-evaluation
        // This drone's old target is now fair game for itself or others if it was reserved.
        // The cell_reserved set is rebuilt at the start anyway reflecting current active targets.

        for (const auto& candidate_coord : candidate_target_coords) {
            // A drone can re-target its own previously reserved cell.
            // But it cannot target a cell actively reserved by *another* drone.
            bool is_reserved_by_other = false;
            if (cell_reserved.count(candidate_coord)) {
                if (!drone_targets.count(rid) || !coord_equal(drone_targets[rid], candidate_coord)) {
                    // If it's in cell_reserved AND it's not this drone's current target, then it's reserved by another.
                    is_reserved_by_other = true;
                }
            }
            if (is_reserved_by_other) continue;


            std::vector<Coord> current_path = plan_path(drone_pos, candidate_coord, known_cost_map, ROBOT::TYPE::DRONE, known_object_map);

            // Path to self is empty, but can be a valid target if it reveals unknowns and drone is already there.
            if (current_path.empty() && !coord_equal(drone_pos, candidate_coord)) continue;

            long long path_energy = calculate_path_energy(current_path, drone_pos, ROBOT::TYPE::DRONE, known_cost_map);

            if (path_energy > current_energy) {
                continue;
            }

            int num_unknowns = count_observable_unknowns(candidate_coord, known_object_map);
            // num_unknowns should be > 0 due to candidate_target_coords selection, but check score logic

            double current_score = 0.0;
            if (num_unknowns > 0) {
                current_score = static_cast<double>(num_unknowns) / (current_path.size() + 1.0);
            }


            if (current_score > max_score) {
                max_score = current_score;
                best_path_for_drone = current_path;
                best_target_for_drone = candidate_coord;
            }
        }

        // Clear old reservation if target changes
        if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid])) {
            if (!coord_equal(drone_targets[rid], best_target_for_drone)) { // if target changed or no target found
                cell_reserved.erase(drone_targets[rid]);
            }
        }


        if (max_score > 0) { // max_score will be > 0 if num_unknowns > 0 for the best candidate
            drone_paths[rid].assign(best_path_for_drone.begin(), best_path_for_drone.end());
            drone_targets[rid] = best_target_for_drone;
            // Only reserve if it's not the drone's current position (unless it's a 0-length path to current pos)
            if (!coord_equal(drone_pos, best_target_for_drone) || best_path_for_drone.empty()) {
                cell_reserved.insert(best_target_for_drone);
            }
            // printf("[DEBUG] Drone %d: New path, target=(%d,%d), score=%.2f, energy_cost=%lld, path_len=%zu\n",
            //       rid, best_target_for_drone.x, best_target_for_drone.y, max_score,
            //       calculate_path_energy(best_path_for_drone, drone_pos, ROBOT::TYPE::DRONE, known_cost_map), best_path_for_drone.size());
        }
        else {
            drone_paths[rid].clear();
            drone_targets[rid] = drone_pos;
            // printf("[DEBUG] Drone %d: NO suitable path found.\n", rid);
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
    if (robot.type == ROBOT::TYPE::DRONE) { // Corrected: ROBOT::TYPE::DRONE
        return false; // Drones do not perform tasks
    }
    return true; // Other robots perform tasks by default
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
        init_map_size(known_object_map);
    }
    if (map_size == -1) return ROBOT::ACTION::HOLD; // Corrected: ROBOT::ACTION::HOLD


    if (robot.type != ROBOT::TYPE::DRONE) return ROBOT::ACTION::HOLD; // Corrected

    int rid = robot.id;
    Coord cur = robot.get_coord();

    if (drone_paths.count(rid) && !drone_paths[rid].empty()) {
        Coord next_target_in_path = drone_paths[rid].front();

        if (coord_equal(cur, next_target_in_path)) {
            drone_paths[rid].pop_front();
            if (drone_paths[rid].empty()) {
                if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid]) && coord_equal(drone_targets[rid], cur)) {
                    cell_reserved.erase(drone_targets[rid]);
                }
                return ROBOT::ACTION::HOLD; // Corrected
            }
            next_target_in_path = drone_paths[rid].front();
        }

        if (next_target_in_path.x < 0 || next_target_in_path.x >= map_size || next_target_in_path.y < 0 || next_target_in_path.y >= map_size ||
            known_object_map[next_target_in_path.x][next_target_in_path.y] == OBJECT::WALL) {

            if (drone_targets.count(rid) && cell_reserved.count(drone_targets[rid])) {
                cell_reserved.erase(drone_targets[rid]);
            }
            drone_paths[rid].clear();
            drone_targets[rid] = cur; // Reset target
            // printf("[DEBUG] Drone %d: Path to %d,%d blocked. Holding.\n", rid, next_target_in_path.x, next_target_in_path.y);
            return ROBOT::ACTION::HOLD; // Corrected
        }
        return get_direction(cur, next_target_in_path);
    }
    return ROBOT::ACTION::HOLD; // Corrected
}