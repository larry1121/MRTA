#include "schedular.h"
#include <cstdlib>   // rand()
#include <iostream>  // For debugging, remove in final version if not needed for VERBOSE
#include <cmath>     // For std::abs, std::sqrt, std::atan2
#include <algorithm> // For std::sort, std::min_element, std::max
#include <vector>
#include <queue>
#include <map>
#include <set>

// Anonymous namespace for helper functions not part of the class
namespace
{
    // Helper for distance calculation (Manhattan distance as per simplified pathfinding)
    int manhattan_distance(const Coord &a, const Coord &b)
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    // Check if a coordinate is valid and not a wall -- THIS WILL BE MOVED AND MODIFIED
    /* bool is_valid_and_not_wall(int r, int c, int height, int width, const std::vector<std::vector<OBJECT>> &map)
    {
        return r >= 0 && r < height && c >= 0 && c < width && map[r][c] != OBJECT::WALL;
    } */
} // anonymous namespace

Scheduler::Scheduler() : map_width_(0),
                         map_height_(0),
                         current_tick_(0),
                         known_ratio_(0.0),
                         initial_map_setup_done_(false),
                         explore_complete_logged_(false),
                         re_explore_triggered_(false)
{
    // Initialize any other members if necessary
}

void Scheduler::initialize_scheduler_state(const vector<vector<OBJECT>> &known_object_map, const vector<shared_ptr<ROBOT>> &robots)
{
    if (known_object_map.empty() || known_object_map[0].empty())
    {
        return; // Not enough info to determine map size
    }
    map_height_ = known_object_map.size();
    map_width_ = known_object_map[0].size();

    std::cout << "Map Initialized: Width=" << map_width_ << ", Height=" << map_height_ << std::endl;

    generate_virtual_centers_grid();
    // sort_centers_for_spiral_path(virtual_centers_); // Sorting will be more complex

    for (const auto &robot_ptr : robots)
    {
        if (robot_ptr->type == ROBOT::TYPE::DRONE)
        {
            drone_states_[robot_ptr->id] = DroneState::EXPLORE;
            current_waypoint_idx_[robot_ptr->id] = 0;
            // Initial target assignment will be handled by FSM
        }
    }
    initial_map_setup_done_ = true;
}

void Scheduler::update_map_knowledge(const vector<vector<OBJECT>> &known_object_map, const vector<shared_ptr<ROBOT>> &robots)
{
    if (map_width_ == 0 || map_height_ == 0)
        return; // Map not initialized

    // Debug: Print specific cell values at the beginning of update_map_knowledge
    if (!known_object_map.empty() && !known_object_map[0].empty())
    {
        int h = known_object_map.size();
        int w = known_object_map[0].size();
        // Check if map dimensions match internal state, though map_width_ and map_height_ should be set by now.
        if (h != map_height_ || w != map_width_)
        {
            // std::cout << "[UpdateMapWarn Tick: " << current_tick_ << "] Mismatch! Internal H:" << map_height_ << " W:" << map_width_
            //           << ", Received H:" << h << " W:" << w << std::endl;
        }
        if (map_height_ > 10 && map_width_ > 10)
        { // Use internal map_height_ and map_width_ for safety
          // std::cout << "[UpdateMap_Debug Tick: " << current_tick_ << "] Using map for known_ratio. Cell (6,5) obj: " << static_cast<int>(known_object_map[5][6])
          //           << ", Cell (6,10) obj: " << static_cast<int>(known_object_map[10][6]) << std::endl;
        }
    }
    else
    {
        // std::cout << "[UpdateMap_Debug Tick: " << current_tick_ << "] empty known_object_map in update_map_knowledge." << std::endl;
        return; // Cant process if map is empty
    }

    int known_cells = 0;
    for (int i = 0; i < map_height_; ++i)
    {
        for (int j = 0; j < map_width_; ++j)
        {
            if (known_object_map[i][j] != OBJECT::UNKNOWN)
            {
                known_cells++;
            }
        }
    }
    known_ratio_ = static_cast<double>(known_cells) / (map_width_ * map_height_);
    // std::cout << "Tick: " << current_tick_ << " Known Ratio: " << known_ratio_ * 100 << "%" << std::endl;
}

void Scheduler::generate_virtual_centers_grid()
{
    virtual_centers_.clear();
    if (map_width_ == 0)
        return;

    int gap = std::max(4, map_width_ / 10);
    if (gap == 0)
        gap = 1; // Prevent gap from being zero with very small maps

    for (int r = gap / 2; r < map_height_; r += gap)
    {
        for (int c = gap / 2; c < map_width_; c += gap)
        {
            virtual_centers_.push_back({c, r}); // Coord is {x, y}
        }
    }
    // std::cout << "Generated " << virtual_centers_.size() << " virtual centers with gap " << gap << std::endl;

    // Simple sort: top-to-bottom, left-to-right. Spiral sort is more complex.
    // For now, we'll let drones pick the closest unassigned.
    // A proper spiral sort would be needed for strict outer-ring -> inner-ring.
    // The "pick nearest unvisited centre" from requirement 3.b simplifies this.
}

// A proper spiral sort is complex. This is a placeholder or needs a different approach.
// The requirement "Each drone picks the nearest unvisited centre as its starting anchor"
// and "Visit centres outer-ring → inner-ring in clockwise spiral order"
// might imply that drones dynamically pick from the available set based on proximity and a global ordering.
// For now, we'll use a simpler assignment logic: closest unassigned.
void Scheduler::sort_centers_for_spiral_path(std::vector<Coord> &centers)
{
    if (centers.empty() || map_width_ == 0 || map_height_ == 0)
        return;

    Coord map_center_approx = {map_width_ / 2, map_height_ / 2};

    std::sort(centers.begin(), centers.end(), [&](const Coord &a, const Coord &b)
              {
                  int dist_a = manhattan_distance(a, map_center_approx);
                  int dist_b = manhattan_distance(b, map_center_approx);

                  if (dist_a != dist_b)
                  {
                      return dist_a > dist_b; // Outer rings first (greater distance from center)
                  }

                  // For same distance, sort by angle (crude clockwise for outer rings)
                  // This is a simplified approach for spiral.
                  // True spiral requires layer-by-layer processing.
                  double angle_a = std::atan2(a.y - map_center_approx.y, a.x - map_center_approx.x);
                  double angle_b = std::atan2(b.y - map_center_approx.y, b.x - map_center_approx.x);

                  // Adjust angles to be 0 to 2PI starting from "up" or "right"
                  // For simplicity, direct atan2 comparison might work for basic ordering.
                  // A common starting point for clockwise is -Y axis (North), then +X (East), then +Y (South), then -X (West)
                  // atan2 results are in (-PI, PI]. We might need to adjust.

                  // Simplified: If on an outer ring, prefer top-right-ish start and go clockwise.
                  // This sort is non-trivial for a perfect spiral.
                  // The "pick nearest unvisited" might be the dominant strategy.
                  return angle_a < angle_b; // Needs refinement for true clockwise.
              });
    // std::cout << "Virtual centers sorted (partially for spiral)." << std::endl;
}

// Pathfinding Member function implementation
bool Scheduler::is_valid_and_not_wall(int r, int c, const std::vector<std::vector<OBJECT>> &known_map)
{
    if (locally_discovered_walls_.count({c, r}))
    { // Coord is {x,y} so {c,r} for set lookup
        // std::cout << "[SchedulerDebug] Cell (" << c << "," << r << ") is in locally_discovered_walls_." << std::endl;
        return false;
    }
    return r >= 0 && r < map_height_ && c >= 0 && c < map_width_ && known_map[r][c] != OBJECT::WALL;
}

std::vector<Coord> Scheduler::find_path_bfs(const Coord &start, const Coord &goal,
                                            const vector<vector<OBJECT>> &known_object_map,
                                            const Coord *avoid_cell)
{
    std::vector<Coord> path;
    // Check if start or goal is invalid or a wall initially
    if (!is_valid_and_not_wall(start.y, start.x, known_object_map))
    {
        // std::cout << "[BFS_Debug] Start position (" << start.x << "," << start.y << ") is invalid or a wall. No path." << std::endl;
        return {};
    }
    if (!is_valid_and_not_wall(goal.y, goal.x, known_object_map))
    {
        // std::cout << "[BFS_Debug] Goal position (" << goal.x << "," << goal.y << ") is invalid or a wall. No path." << std::endl;
        return {};
    }

    if (map_height_ == 0 || map_width_ == 0 || start == goal)
    {
        if (start == goal)
            path.push_back(start);
        return path;
    }

    std::queue<std::vector<Coord>> q;
    std::set<Coord> visited_bfs;

    q.push({start});
    visited_bfs.insert(start);

    int dr[] = {-1, 1, 0, 0}; // UP, DOWN
    int dc[] = {0, 0, -1, 1}; // LEFT, RIGHT

    while (!q.empty())
    {
        std::vector<Coord> current_path = q.front();
        q.pop();
        Coord current_coord = current_path.back();

        if (current_coord == goal)
        {
            return current_path;
        }

        for (int i = 0; i < 4; ++i)
        {
            int next_r = current_coord.y + dr[i]; // Assuming Coord {x,y} -> map[y][x]
            int next_c = current_coord.x + dc[i];
            Coord next_coord = {next_c, next_r};

            if (is_valid_and_not_wall(next_r, next_c, known_object_map) &&
                visited_bfs.find(next_coord) == visited_bfs.end())
            {
                if (avoid_cell && next_coord == *avoid_cell && next_coord != goal) // Modified condition
                {                                                                  // Check if next_coord is the cell to avoid, but not if it's the goal itself
                    // std::cout << "[BFS_Debug] Robot Unknown: Avoiding recently stuck cell: (" << next_coord.x << "," << next_coord.y << ") for goal (" << goal.x << "," << goal.y << ")" << std::endl;
                    continue; // Skip this cell, don't add to path or queue
                }

                visited_bfs.insert(next_coord);
                std::vector<Coord> new_path = current_path;
                new_path.push_back(next_coord);
                q.push(new_path);
            }
        }
    }
    return {}; // No path found
}

ROBOT::ACTION Scheduler::get_move_action_to_target(const Coord &current_pos, const Coord &target_pos)
{
    if (current_pos.y > target_pos.y)
        return ROBOT::ACTION::UP;
    if (current_pos.y < target_pos.y)
        return ROBOT::ACTION::DOWN;
    if (current_pos.x > target_pos.x)
        return ROBOT::ACTION::LEFT;
    if (current_pos.x < target_pos.x)
        return ROBOT::ACTION::RIGHT;
    return ROBOT::ACTION::HOLD;
}

Coord Scheduler::get_closest_unassigned_center(const Coord &drone_pos, const std::shared_ptr<ROBOT> &robot, const vector<vector<OBJECT>> &known_object_map)
{
    Coord best_target = {-1, -1};
    int min_dist = std::numeric_limits<int>::max();
    std::vector<Coord> centers_to_evaluate = virtual_centers_; // Create a copy to iterate

    // Optional: Sort centers_to_evaluate by distance to drone_pos first for potentially faster finding
    std::sort(centers_to_evaluate.begin(), centers_to_evaluate.end(),
              [&](const Coord &a, const Coord &b)
              {
                  return manhattan_distance(drone_pos, a) < manhattan_distance(drone_pos, b);
              });

    for (const auto &center : centers_to_evaluate)
    {
        if (visited_centers_.count(center) || assigned_centers_.count(center) || unreachable_centers_.count(center))
        {
            continue;
        }

        if (!is_valid_and_not_wall(center.y, center.x, known_object_map))
        {
            // std::cout << "[CenterCheck] Center (" << center.x << "," << center.y << ") is a wall or locally discovered wall. Marking unreachable." << std::endl;
            unreachable_centers_.insert(center); // Mark as unreachable if the center itself is invalid
            continue;
        }

        // Perform BFS check here
        std::vector<Coord> path_to_center = find_path_bfs(drone_pos, center, known_object_map, nullptr);

        if (path_to_center.empty())
        {
            // std::cout << "[CenterCheck] Robot " << robot->id << " could NOT find path to potential center (" << center.x << "," << center.y << ") in get_closest. Marking unreachable." << std::endl;
            unreachable_centers_.insert(center);
            continue; // Try next center
        }

        // If path exists, this center is a candidate.
        // We are looking for the closest one based on initial Manhattan distance.
        // The sort above helps find a valid one faster, but we still need to ensure it's the *overall* closest among valid ones.
        // For simplicity with the current loop structure (already sorted by distance),
        // the first one we find a path to will be the best_target.
        // To be absolutely sure it's the closest *overall* even if an earlier (closer by manhattan) center had its path check fail,
        // we would need to iterate all, check paths for all non-unreachable, then pick.
        // However, the current requirement is "nearest unvisited", sorting helps.

        int dist = manhattan_distance(drone_pos, center); // Re-calculate or use sorted order
        if (dist < min_dist)                              // This logic is somewhat redundant if we take the first valid from sorted list
        {
            // This check is to ensure we are picking the closest if the list wasn't pre-sorted by distance for the BFS check.
            // Since we did sort, the first `center` that passes all checks (visited, assigned, unreachable, wall, path_exists)
            // should be our best candidate.
            min_dist = dist; // Not strictly needed if taking the first valid from sorted list
            best_target = center;
            // std::cout << "[CenterCheck] Robot " << robot->id << " found potential valid target (" << center.x << "," << center.y << ") with dist " << dist << std::endl;
            return best_target; // Found the closest (due to pre-sort) valid center, return it.
        }
    }
    // If loop finishes, best_target might still be {-1,-1} or the last one that met criteria if not returning early.
    // If we exit the loop because all centers were checked and no suitable one was found after pre-sorting and BFS checks
    return best_target; // Could be {-1,-1} if no center is reachable or suitable
}

void Scheduler::assign_new_exploration_target(const shared_ptr<ROBOT> &robot,
                                              const vector<vector<OBJECT>> &known_object_map)
{
    if (robot->type != ROBOT::TYPE::DRONE)
        return;

    Coord current_pos = robot->get_coord();
    Coord target_center = get_closest_unassigned_center(current_pos, robot, known_object_map);

    if (target_center.x != -1 && target_center.y != -1)
    {
        // Path was already checked in get_closest_unassigned_center,
        // but map state might have changed or that BFS was just for reachability.
        // Re-finding path here is safer and ensures current robot's specific last_stuck_inducing_target_cell_ is considered if any.
        // Coord const *cell_to_avoid_ptr = nullptr; // MODIFIED: Always nullptr for new target assignment
        // if (last_stuck_inducing_target_cell_.count(robot->id))
        // {
        //     cell_to_avoid_ptr = &last_stuck_inducing_target_cell_.at(robot->id);
        //     std::cout << "Robot " << robot->id << " [assign_new_exploration_target] Attempting BFS to new target (" << target_center.x << "," << target_center.y
        //               << ") avoiding last stuck cell (" << cell_to_avoid_ptr->x << "," << cell_to_avoid_ptr->y << ")" << std::endl;
        // }
        // else
        // {
        std::cout << "Robot " << robot->id << " [assign_new_exploration_target] Attempting BFS to new target (" << target_center.x << "," << target_center.y << ") (no specific cell to avoid for new target)." << std::endl;
        // }

        std::vector<Coord> path = find_path_bfs(current_pos, target_center, known_object_map, nullptr); // MODIFIED: avoid_cell is nullptr

        if (!path.empty())
        {
            std::cout << "Robot " << robot->id << " new exploration target: (" << target_center.x << "," << target_center.y << ") - Path FOUND." << std::endl;
            drone_target_centers_[robot->id] = target_center;
            assigned_centers_.insert(target_center);
            target_fail_counts_[robot->id][target_center] = 0;
            if (last_stuck_inducing_target_cell_.count(robot->id))
            {
                last_stuck_inducing_target_cell_.erase(robot->id); // Clear last stuck cell on new target
                std::cout << "Robot " << robot->id << " [assign_new_exploration_target] Cleared last_stuck_inducing_target_cell_ for new target (" << target_center.x << "," << target_center.y << ")." << std::endl;
            }

            waypoint_cache_[robot->id] = path;
            if (path.size() > 1)
                current_waypoint_idx_[robot->id] = 1;
            else
                current_waypoint_idx_[robot->id] = 0;
        }
        else
        {
            std::cout << "Robot " << robot->id << " could NOT find initial BFS path to potential target (" << target_center.x << "," << target_center.y << "). Marking unreachable." << std::endl;
            unreachable_centers_.insert(target_center);
        }
    }
    else
    {
        // std::cout << "Robot " << robot->id << " found no unassigned centers during target assignment attempt." << std::endl;
    }
}

void Scheduler::handle_drone_fsm(const shared_ptr<ROBOT> &robot,
                                 const vector<vector<vector<int>>> &known_cost_map,
                                 const vector<vector<OBJECT>> &known_object_map)
{
    if (robot->type != ROBOT::TYPE::DRONE)
        return;

    DroneState &current_state = drone_states_[robot->id];
    Coord robot_pos = robot->get_coord();
    int robot_id = robot->id;

    // State Transitions
    if (current_state == DroneState::EXPLORE)
    {
        if (initial_explore_phase_complete_by_centers_)
        {
            std::cout << "Robot " << robot_id << " (EXPLORE): Initial explore by centers complete." << std::endl;
            if (current_tick_ < REEXPLORE_TICK_THRESHOLD)
            {
                current_state = DroneState::WAIT_TASKS;
                std::cout << "Robot " << robot_id << " transitioning from EXPLORE to WAIT_TASKS (centers met)" << std::endl;
            }
            else
            {
                // Will be caught by global re-explore trigger if time is up
                std::cout << "Robot " << robot_id << " (EXPLORE): Centers met, but REEXPLORE_TICK_THRESHOLD reached/passed. Awaiting global REEXPLORE." << std::endl;
            }
            waypoint_cache_.erase(robot_id);
            drone_target_centers_.erase(robot_id); // Clear target as phase is done
            // target_fail_counts_ for this robot might need clearing or handling if it re-enters EXPLORE
            last_stuck_inducing_target_cell_.erase(robot_id); // Clear on phase change
        }
    }
    else if (current_state == DroneState::REEXPLORE)
    {
        if (known_ratio_ >= EXPLORATION_GOAL_RATIO && re_explore_triggered_)
        { // Check re_explore_triggered_ to ensure it's this phase
            if (!explore_complete_logged_)
            { // Log only once per re-explore phase
                std::cout << "RE-EXPLORE_COMPLETE for Robot " << robot_id << " at tick " << current_tick_ << " with known ratio " << known_ratio_ * 100 << "%" << std::endl;
                explore_complete_logged_ = true; // Mark as logged for this robot/phase
            }
            current_state = DroneState::HOLD;
            std::cout << "Robot " << robot_id << " transitioning from REEXPLORE to HOLD" << std::endl;
            waypoint_cache_.erase(robot_id);
            drone_target_centers_.erase(robot_id);
            if (last_stuck_inducing_target_cell_.count(robot_id))
            {
                last_stuck_inducing_target_cell_.erase(robot_id); // Clear on phase change
                std::cout << "Robot " << robot_id << " [FSM] REEXPLORE->HOLD: Cleared last_stuck_inducing_target_cell_." << std::endl;
            }
        }
    }
    // Global re-exploration trigger in on_info_updated handles setting all drones to REEXPLORE.
    // Here, individual drones act based on that state.

    // Actions within States
    switch (current_state)
    {
    case DroneState::EXPLORE:
    case DroneState::REEXPLORE:
        // If initial explore by centers is complete for EXPLORE state, FSM transition above handles it.
        // So, if we are here in EXPLORE state, it means initial_explore_phase_complete_by_centers_ is false.

        if (robot->get_status() == ROBOT::STATUS::IDLE)
        {
            bool has_target_center = drone_target_centers_.count(robot_id);
            bool path_is_valid = waypoint_cache_.count(robot_id) &&
                                 current_waypoint_idx_[robot_id] < waypoint_cache_[robot_id].size();

            if (has_target_center && robot_pos == drone_target_centers_.at(robot_id))
            { // Reached the high-level center
                std::cout << "Robot " << robot_id << " reached center: (" << robot_pos.x << "," << robot_pos.y << ")" << std::endl;
                visited_centers_.insert(robot_pos);
                assigned_centers_.erase(robot_pos);
                drone_target_centers_.erase(robot_id);
                waypoint_cache_.erase(robot_id);
                current_waypoint_idx_.erase(robot_id);
                target_fail_counts_[robot_id].erase(robot_pos); // Clear fail count for this now-visited center
                if (last_stuck_inducing_target_cell_.count(robot_id))
                {
                    last_stuck_inducing_target_cell_.erase(robot_id); // Clear on reaching target
                    std::cout << "Robot " << robot_id << " [FSM] Reached center (" << robot_pos.x << "," << robot_pos.y << "): Cleared last_stuck_inducing_target_cell_." << std::endl;
                }

                // Check if EXPLORE phase should end before assigning new target
                if (current_state == DroneState::EXPLORE && initial_explore_phase_complete_by_centers_)
                {
                    // This check is a bit redundant if FSM transition logic above is robust, but good for safety
                    // FSM transition should already have moved to WAIT_TASKS or HOLD
                    std::cout << "Robot " << robot_id << " (FSM): Reached center, initial explore by centers already marked complete. Not assigning new target." << std::endl;
                }
                else
                {
                    assign_new_exploration_target(robot, known_object_map);
                }
            }
            else if (!has_target_center && !(current_state == DroneState::EXPLORE && initial_explore_phase_complete_by_centers_))
            {
                // No target center, and (if in EXPLORE state) the center-based exploration is not yet complete.
                assign_new_exploration_target(robot, known_object_map);
            }
            else if (has_target_center && !path_is_valid)
            {
                // Has a target, but no valid path (e.g., path cleared by idle_action due to wall, or BFS failed previously)
                Coord current_target = drone_target_centers_.at(robot_id);

                // ADDED CHECK: If the target itself is invalid, mark unreachable immediately.
                if (!is_valid_and_not_wall(current_target.y, current_target.x, known_object_map))
                {
                    std::cout << "Robot " << robot_id << " [FSM]: Current target (" << current_target.x << "," << current_target.y
                              << ") is a wall or locally discovered obstacle. Marking unreachable directly." << std::endl;
                    if (assigned_centers_.count(current_target))
                    { // Check if it exists before erasing
                        assigned_centers_.erase(current_target);
                    }
                    unreachable_centers_.insert(current_target);
                    drone_target_centers_.erase(robot_id); // Clear this robot's target

                    if (target_fail_counts_.count(robot_id) && target_fail_counts_.at(robot_id).count(current_target))
                    {
                        target_fail_counts_.at(robot_id).erase(current_target); // Clear fail counts for this target
                    }

                    if (last_stuck_inducing_target_cell_.count(robot_id))
                    {
                        // If the robot was stuck trying to reach this now-invalid target, clear the stuck cell info.
                        // This is a general clear, as the target itself is the problem.
                        if (last_stuck_inducing_target_cell_.at(robot_id) == current_target)
                        {
                            std::cout << "Robot " << robot_id << " [FSM] Target (" << current_target.x << "," << current_target.y
                                      << ") is wall and was the last_stuck_inducing_target_cell_. Clearing." << std::endl;
                        }
                        else
                        {
                            std::cout << "Robot " << robot_id << " [FSM] Target (" << current_target.x << "," << current_target.y
                                      << ") is wall. Clearing potentially related last_stuck_inducing_target_cell_." << std::endl;
                        }
                        last_stuck_inducing_target_cell_.erase(robot_id);
                    }
                    // The FSM will attempt to assign a new target in the current or next cycle if state is EXPLORE/REEXPLORE
                }
                else // Target cell itself IS valid, proceed with trying to find a path
                {
                    std::cout << "Robot " << robot_id << " [FSM]: Has target (" << current_target.x << "," << current_target.y << "), but no valid path. Attempting BFS." << std::endl;

                    Coord const *cell_to_avoid_ptr_fsm = nullptr;
                    if (last_stuck_inducing_target_cell_.count(robot_id))
                    {
                        cell_to_avoid_ptr_fsm = &last_stuck_inducing_target_cell_.at(robot_id);
                        std::cout << "Robot " << robot_id << " [FSM] Attempting BFS to (" << current_target.x << "," << current_target.y
                                  << ") avoiding last stuck cell (" << cell_to_avoid_ptr_fsm->x << "," << cell_to_avoid_ptr_fsm->y << ")" << std::endl;
                    }
                    // else
                    // {
                    //     // std::cout << "Robot " << robot_id << " [FSM] Attempting BFS to (" << current_target.x << "," << current_target.y << ") (no specific cell to avoid)." << std::endl;
                    // }
                    std::vector<Coord> path = find_path_bfs(robot_pos, current_target, known_object_map, cell_to_avoid_ptr_fsm);

                    if (!path.empty()) // Path found and it inherently avoided the stuck cell if one was provided to BFS
                    {
                        waypoint_cache_[robot_id] = path;
                        if (path.size() > 1)
                            current_waypoint_idx_[robot_id] = 1;
                        else
                            current_waypoint_idx_[robot_id] = 0;
                        std::cout << "Robot " << robot_id << " [FSM]: Path to (" << current_target.x << "," << current_target.y << ") FOUND (avoided stuck if applicable)." << std::endl;
                        if (last_stuck_inducing_target_cell_.count(robot_id))
                        {
                            last_stuck_inducing_target_cell_.erase(robot_id);
                            std::cout << "Robot " << robot_id << " [FSM] Path found, BFS already avoided stuck: Cleared last_stuck_inducing_target_cell_ for target (" << current_target.x << "," << current_target.y << ")." << std::endl;
                        }
                    }
                    else // Path not found by BFS (either truly no path, or stuck cell blocked only path)
                    {
                        // if (path_leads_to_recent_stuck_cell) // This part of the log is no longer reachable if BFS handles avoid_cell
                        // {
                        //     std::cout << "Robot " << robot_id << " [FSM]: Path to (" << current_target.x << "," << current_target.y
                        //               << ") found, BUT it leads directly into recently stuck cell ("
                        //               << last_stuck_inducing_target_cell_.at(robot_id).x << "," << last_stuck_inducing_target_cell_.at(robot_id).y
                        //               << "). Invalidating path and consuming last_stuck_inducing_target_cell_." << std::endl;
                        //     last_stuck_inducing_target_cell_.erase(robot_id); // Consume this info
                        // }
                        // else
                        // {
                        std::cout << "Robot " << robot_id << " [FSM]: Path to (" << current_target.x << "," << current_target.y << ") NOT FOUND (tried avoiding stuck if applicable)." << std::endl;
                        // }
                        target_fail_counts_[robot_id][current_target]++;
                        std::cout << "Robot " << robot_id << " incremented fail count for target (" << current_target.x << "," << current_target.y
                                  << ") to " << target_fail_counts_[robot_id][current_target] << std::endl;

                        if (target_fail_counts_[robot_id][current_target] >= MAX_TARGET_FAIL_COUNT)
                        {
                            std::cout << "Robot " << robot_id << " target (" << current_target.x << "," << current_target.y
                                      << ") reached MAX_TARGET_FAIL_COUNT. Marking as unreachable." << std::endl;
                            assigned_centers_.erase(current_target);
                            unreachable_centers_.insert(current_target);
                            drone_target_centers_.erase(robot_id);
                            target_fail_counts_[robot_id].erase(current_target);
                            if (last_stuck_inducing_target_cell_.count(robot_id))
                            {
                                last_stuck_inducing_target_cell_.erase(robot_id); // Clear on giving up target
                                std::cout << "Robot " << robot_id << " [FSM] MAX_FAIL: Cleared last_stuck_inducing_target_cell_ for target (" << current_target.x << "," << current_target.y << ")." << std::endl;
                            }
                        }
                    }
                }
            }
            // If it has a target and a valid path, idle_action will handle movement.
        }
        break;
    case DroneState::WAIT_TASKS:
        // Drone just holds, task handling is not part of this spec for drones
        // No specific action here, idle_action will return HOLD
        if (last_stuck_inducing_target_cell_.count(robot_id))
        {
            last_stuck_inducing_target_cell_.erase(robot_id); // Clear when waiting
            std::cout << "Robot " << robot_id << " [FSM] State WAIT_TASKS: Cleared last_stuck_inducing_target_cell_." << std::endl;
        }
        break;
    case DroneState::HOLD:
        // Exploration and (re-exploration if triggered) are complete.
        // No specific action here, idle_action will return HOLD
        if (last_stuck_inducing_target_cell_.count(robot_id))
        {
            last_stuck_inducing_target_cell_.erase(robot_id); // Clear when holding
            std::cout << "Robot " << robot_id << " [FSM] State HOLD: Cleared last_stuck_inducing_target_cell_." << std::endl;
        }
        break;
    }
}

// MAIN SCHEDULER METHODS

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    current_tick_++;

    // Debug: Print received map dimensions and specific cell values at the beginning of on_info_updated
    if (!known_object_map.empty() && !known_object_map[0].empty())
    {
        int h = known_object_map.size();
        int w = known_object_map[0].size();
        // std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Received map H:" << h << ", W:" << w << std::endl;
        // Print some critical/problematic cells previously identified
        if (h > 10 && w > 10)
        { // Basic boundary check for test coordinates
          // std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Cell (6,5) obj: " << static_cast<int>(known_object_map[5][6])
          //           << ", Cell (6,10) obj: " << static_cast<int>(known_object_map[10][6])
          //           << ", Cell (2,10) obj: " << static_cast<int>(known_object_map[10][2]) << std::endl;
        }
        if (h > 18 && w > 18)
        {
            // std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Cell (18,17) obj: " << static_cast<int>(known_object_map[17][18])
            //           << ", Cell (14,14) obj: " << static_cast<int>(known_object_map[14][14]) << std::endl;
        }
    }
    else
    {
        // std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Received empty known_object_map." << std::endl;
    }

    if (!initial_map_setup_done_)
    {
        initialize_scheduler_state(known_object_map, robots);
    }

    if (!initial_map_setup_done_ && (known_object_map.empty() || known_object_map[0].empty()))
    {
        std::cout << "Tick " << current_tick_ << ": Waiting for map data to initialize." << std::endl;
        return; // Not enough info yet
    }
    // Ensure map dimensions are set if not done during initial_map_setup_done_ check
    // This can happen if known_object_map was empty on the very first tick(s)
    if ((map_width_ == 0 || map_height_ == 0) && !known_object_map.empty() && !known_object_map[0].empty())
    {
        map_height_ = known_object_map.size();
        map_width_ = known_object_map[0].size();
        if (!initial_map_setup_done_)
        {                                                         // If somehow setup was skipped but map dimensions are now available
            initialize_scheduler_state(known_object_map, robots); // Try re-init
        }
        else
        { // if setup was done but map dimensions were 0, just update them and regenerate centers
            generate_virtual_centers_grid();
            // sort_centers_for_spiral_path(virtual_centers_);
        }
        std::cout << "Tick " << current_tick_ << ": Map dimensions confirmed/updated. W: " << map_width_ << " H: " << map_height_ << std::endl;
    }

    update_map_knowledge(known_object_map, robots);

    // Check if all non-unreachable virtual centers have been visited or are assigned
    // This check should be done for initial_explore_phase_complete_by_centers_
    if (!initial_explore_phase_complete_by_centers_ && initial_map_setup_done_ && !virtual_centers_.empty())
    {
        bool all_centers_accounted_for = true;
        for (const auto &center : virtual_centers_)
        {
            // A center is accounted for if it's visited OR marked as unreachable.
            if (!visited_centers_.count(center) && !unreachable_centers_.count(center))
            {
                all_centers_accounted_for = false;
                break;
            }
        }

        if (all_centers_accounted_for)
        {
            std::cout << "Tick " << current_tick_ << ": All virtual centers are now either visited or marked unreachable. Initial exploration by centers complete." << std::endl;
            initial_explore_phase_complete_by_centers_ = true;
            // Drones in EXPLORE state will transition to WAIT_TASKS in their next FSM cycle.
        }
    }

    // The re-exploration trigger needs to iterate all drones. This should be done once.
    if (current_tick_ >= REEXPLORE_TICK_THRESHOLD && !re_explore_triggered_)
    {
        std::cout << "Tick " << current_tick_ << ": Global REEXPLORE trigger initiated." << std::endl;
        visited_centers_.clear();
        assigned_centers_.clear();
        unreachable_centers_.clear();
        target_fail_counts_.clear();
        locally_discovered_walls_.clear();
        if (!last_stuck_inducing_target_cell_.empty())
        {
            last_stuck_inducing_target_cell_.clear(); // Clear for all robots on global re-explore
            std::cout << "[on_info_updated] Global REEXPLORE: Cleared all last_stuck_inducing_target_cell_ entries." << std::endl;
        }

        for (const auto &robot_ptr : robots)
        {
            if (robot_ptr->type == ROBOT::TYPE::DRONE)
            {
                drone_states_[robot_ptr->id] = DroneState::REEXPLORE;
                waypoint_cache_.erase(robot_ptr->id);
                drone_target_centers_.erase(robot_ptr->id);
                current_waypoint_idx_[robot_ptr->id] = 0;
            }
        }
        re_explore_triggered_ = true;
        explore_complete_logged_ = false;
        std::cout << "All drones set to REEXPLORE state." << std::endl;
    }

    for (const auto &robot_ptr : robots)
    {
        if (robot_ptr->type == ROBOT::TYPE::DRONE)
        {
            // Ensure state exists
            if (drone_states_.find(robot_ptr->id) == drone_states_.end())
            {
                drone_states_[robot_ptr->id] = DroneState::EXPLORE; // Default if new drone
                current_waypoint_idx_[robot_ptr->id] = 0;
            }
            handle_drone_fsm(robot_ptr, known_cost_map, known_object_map);
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
    // Drones do not perform tasks as per current spec focus on exploration
    if (robot.type == ROBOT::TYPE::DRONE)
    {
        return false;
    }
    // Default behavior for other robots (from original file)
    return true;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    if (robot.type != ROBOT::TYPE::DRONE)
    {
        return ROBOT::ACTION::HOLD;
    }
    int robot_id = robot.id; // For convenience

    if (drone_states_.find(robot_id) == drone_states_.end())
    {
        std::cerr << "Warning: Drone " << robot_id << " state not found in idle_action. Holding." << std::endl;
        // Update maps before returning
        previous_pos_map_[robot_id] = robot.get_coord();
        last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }

    Coord current_pos_this_tick = robot.get_coord();

    // Stuck detection logic
    if (last_action_commanded_map_.count(robot_id) &&
        last_action_commanded_map_.at(robot_id) != ROBOT::ACTION::HOLD &&
        previous_pos_map_.count(robot_id) &&
        previous_pos_map_.at(robot_id) == current_pos_this_tick)
    {
        ROBOT::ACTION last_action = last_action_commanded_map_.at(robot_id);
        Coord stuck_pos = previous_pos_map_.at(robot_id);
        Coord failed_target_coord = stuck_pos;

        if (last_action == ROBOT::ACTION::UP)
            failed_target_coord.y++;
        else if (last_action == ROBOT::ACTION::DOWN)
            failed_target_coord.y--;
        else if (last_action == ROBOT::ACTION::LEFT)
            failed_target_coord.x--;
        else if (last_action == ROBOT::ACTION::RIGHT)
            failed_target_coord.x++;

        if (failed_target_coord != stuck_pos)
        {
            // Only add to locally_discovered_walls if it's NOT the current high-level target center
            bool is_dtarget = drone_target_centers_.count(robot_id) && drone_target_centers_.at(robot_id) == failed_target_coord;
            if (!is_dtarget)
            {
                locally_discovered_walls_.insert(failed_target_coord);
                std::cout << "Robot " << robot_id << " [idle_action] STUCK: Added (" << failed_target_coord.x << "," << failed_target_coord.y
                          << ") to locally_discovered_walls_." << std::endl;
            }
            else
            {
                // If it IS the DTarget, it might still be a wall. Let FSM handle it, but log here.
                // The FSM's is_valid_and_not_wall check should catch this if it's truly a wall.
                // However, to ensure it's considered by FSM if map updates are slow, we can also add it here.
                locally_discovered_walls_.insert(failed_target_coord); // Consider adding even if DTarget
                std::cout << "Robot " << robot_id << " [idle_action] STUCK: Target (" << failed_target_coord.x << "," << failed_target_coord.y
                          << ") is current DTarget. Added to locally_discovered_walls_ anyway for FSM check." << std::endl;
            }

            // Record as stuck-inducing, even if it IS the current high-level target center.
            last_stuck_inducing_target_cell_[robot_id] = failed_target_coord;
            std::cout << "Robot " << robot_id << " [idle_action] STUCK: Recorded (" << failed_target_coord.x << "," << failed_target_coord.y
                      << ") as last_stuck_inducing_target_cell_ (even if it's the DTarget)." << std::endl;
        }
        else
        {
            // if failed_target_coord is same as stuck_pos, it implies no move was attempted or HOLD was commanded.
            // This case might not be a typical "stuck against wall" scenario for recording stuck cell.
            if (last_stuck_inducing_target_cell_.count(robot_id))
            { // Clear if it existed from previous unrelated event
                last_stuck_inducing_target_cell_.erase(robot_id);
                std::cout << "Robot " << robot_id << " [idle_action] STUCK: (No specific failed_target_coord or it was self) Cleared last_stuck_inducing_target_cell_." << std::endl;
            }
        }

        std::cout << "Robot " << robot_id << " [idle_action] STUCK DETECTED at ("
                  << current_pos_this_tick.x << "," << current_pos_this_tick.y
                  << "). Last action: " << static_cast<int>(last_action) << ". Clearing path." << std::endl;

        waypoint_cache_.erase(robot_id);
        current_waypoint_idx_.erase(robot_id);

        if (drone_target_centers_.count(robot_id))
        {
            Coord current_target = drone_target_centers_.at(robot_id);
            target_fail_counts_[robot_id][current_target]++;
            std::cout << "Robot " << robot_id << " [idle_action] STUCK: Incremented fail count for target ("
                      << current_target.x << "," << current_target.y << ") to "
                      << target_fail_counts_[robot_id][current_target] << std::endl;

            if (target_fail_counts_[robot_id][current_target] >= MAX_TARGET_FAIL_COUNT)
            {
                std::cout << "Robot " << robot_id << " [idle_action] STUCK: Target (" << current_target.x << "," << current_target.y
                          << ") reached MAX_TARGET_FAIL_COUNT. Clearing target and marking unreachable." << std::endl;
                assigned_centers_.erase(current_target);
                unreachable_centers_.insert(current_target);
                drone_target_centers_.erase(robot_id);
                target_fail_counts_[robot_id].erase(current_target);
                if (last_stuck_inducing_target_cell_.count(robot_id))
                {
                    last_stuck_inducing_target_cell_.erase(robot_id); // Clear on giving up target
                    std::cout << "Robot " << robot_id << " [idle_action] STUCK_MAX_FAIL: Cleared last_stuck_inducing_target_cell_ for target (" << current_target.x << "," << current_target.y << ")." << std::endl;
                }
            }
        }

        previous_pos_map_[robot_id] = current_pos_this_tick;
        last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }
    else if (last_action_commanded_map_.count(robot_id) && previous_pos_map_.count(robot_id) && previous_pos_map_.at(robot_id) != current_pos_this_tick)
    {
        // Moved successfully, clear the last stuck cell if any
        if (last_stuck_inducing_target_cell_.count(robot_id))
        {
            last_stuck_inducing_target_cell_.erase(robot_id);
            std::cout << "Robot " << robot_id << " [idle_action] Moved successfully: Cleared last_stuck_inducing_target_cell_." << std::endl;
        }
    }

    if (robot.get_status() != ROBOT::STATUS::IDLE)
    {
        previous_pos_map_[robot_id] = current_pos_this_tick;
        last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD; // Implicitly holding if busy
        return ROBOT::ACTION::HOLD;
    }

    DroneState current_state = drone_states_[robot_id];

    if (current_state == DroneState::WAIT_TASKS || current_state == DroneState::HOLD ||
        (current_state == DroneState::EXPLORE && initial_explore_phase_complete_by_centers_)) // Also hold if initial explore by centers is done
    {
        previous_pos_map_[robot_id] = current_pos_this_tick;
        last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }

    // EXPLORE (before centers complete) or REEXPLORE state
    if (waypoint_cache_.count(robot_id) && !waypoint_cache_[robot_id].empty())
    {
        size_t &waypoint_idx = current_waypoint_idx_[robot_id];
        const std::vector<Coord> &path = waypoint_cache_[robot_id];
        // Coord current_pos = robot.get_coord(); // current_pos_this_tick already available

        if (waypoint_idx >= path.size())
        {
            // This case should ideally be handled by FSM re-pathing or target completion.
            // If it happens, path is invalid.
            // std::cout << "Robot " << robot_id << " [idle_action] has out-of-bounds waypoint_idx. Clearing path." << std::endl;
            waypoint_cache_.erase(robot_id);
            current_waypoint_idx_.erase(robot_id);
            // FSM will re-evaluate in next cycle.
            previous_pos_map_[robot_id] = current_pos_this_tick;
            last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
            return ROBOT::ACTION::HOLD;
        }

        Coord target_waypoint_for_this_step = path[waypoint_idx];

        if (current_pos_this_tick == target_waypoint_for_this_step)
        {
            waypoint_idx++;
            if (waypoint_idx >= path.size())
            { // Reached end of current path (which should be the drone_target_center)
                // std::cout << "Robot " << robot_id << " [idle_action] completed path, current_pos: "
                //           << current_pos_this_tick.x << "," << current_pos_this_tick.y << std::endl;
                // FSM's job to mark center as visited and assign new one. Path is now empty.
                waypoint_cache_.erase(robot_id);
                current_waypoint_idx_.erase(robot_id);
                // If current_pos_this_tick is the target center, FSM will handle it.
                // If not (e.g. path was short/cleared), FSM will re-evaluate.
                previous_pos_map_[robot_id] = current_pos_this_tick;
                last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
                return ROBOT::ACTION::HOLD;
            }
            target_waypoint_for_this_step = path[waypoint_idx];
        }

        ROBOT::ACTION planned_action = get_move_action_to_target(current_pos_this_tick, target_waypoint_for_this_step);

        if (planned_action == ROBOT::ACTION::HOLD) // Should not happen if current_pos != target_waypoint
        {
            previous_pos_map_[robot_id] = current_pos_this_tick;
            last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
            return ROBOT::ACTION::HOLD;
        }

        Coord immediate_target_cell = current_pos_this_tick;
        // Determine immediate_target_cell based on 스케줄러's planned_action (before Y-swap)
        if (planned_action == ROBOT::ACTION::UP)
            immediate_target_cell.y--; // UP for scheduler means y decreases
        else if (planned_action == ROBOT::ACTION::DOWN)
            immediate_target_cell.y++; // DOWN for scheduler means y increases
        else if (planned_action == ROBOT::ACTION::LEFT)
            immediate_target_cell.x--;
        else if (planned_action == ROBOT::ACTION::RIGHT)
            immediate_target_cell.x++;

        if (map_height_ == 0 || map_width_ == 0)
        { /* error handling */
        }

        // is_valid_and_not_wall uses scheduler's coordinate system (y decreases upwards)
        bool is_valid_move = is_valid_and_not_wall(immediate_target_cell.y, immediate_target_cell.x, known_object_map);

        if (!is_valid_move)
        {
            std::cout << "Robot " << robot_id << " [idle_action] detected wall/invalid for next step "
                      << "(" << immediate_target_cell.x << "," << immediate_target_cell.y << ") from ("
                      << current_pos_this_tick.x << "," << current_pos_this_tick.y << ") towards path waypoint ("
                      << target_waypoint_for_this_step.x << "," << target_waypoint_for_this_step.y
                      << "). Clearing path." << std::endl;

            // Only add to locally_discovered_walls if it's NOT the current high-level target center
            bool is_dtarget_wall_detect = drone_target_centers_.count(robot_id) && drone_target_centers_.at(robot_id) == immediate_target_cell;
            if (!is_dtarget_wall_detect)
            {
                locally_discovered_walls_.insert(immediate_target_cell);
                std::cout << "Robot " << robot_id << " [idle_action] WallDetect: Added (" << immediate_target_cell.x << "," << immediate_target_cell.y
                          << ") to locally_discovered_walls_." << std::endl;
            }
            else
            {
                locally_discovered_walls_.insert(immediate_target_cell); // Consider adding even if DTarget
                std::cout << "Robot " << robot_id << " [idle_action] WallDetect: Target (" << immediate_target_cell.x << "," << immediate_target_cell.y
                          << ") is current DTarget. Added to locally_discovered_walls_ anyway for FSM check." << std::endl;
            }

            // Record as stuck-inducing, even if it IS the current high-level target center.
            last_stuck_inducing_target_cell_[robot_id] = immediate_target_cell;
            std::cout << "Robot " << robot_id << " [idle_action] WallDetect: Recorded (" << immediate_target_cell.x << "," << immediate_target_cell.y
                      << ") as last_stuck_inducing_target_cell_ (even if it's the DTarget)." << std::endl;

            waypoint_cache_.erase(robot_id); // Clear path only
            current_waypoint_idx_.erase(robot_id);

            if (drone_target_centers_.count(robot_id))
            {
                Coord current_target = drone_target_centers_.at(robot_id);
                target_fail_counts_[robot_id][current_target]++;
                std::cout << "Robot " << robot_id << " [idle_action] WallDetect: Incremented fail count for target ("
                          << current_target.x << "," << current_target.y << ") to "
                          << target_fail_counts_[robot_id][current_target] << std::endl;

                if (target_fail_counts_[robot_id][current_target] >= MAX_TARGET_FAIL_COUNT)
                {
                    std::cout << "Robot " << robot_id << " [idle_action] WallDetect: Target (" << current_target.x << "," << current_target.y
                              << ") reached MAX_TARGET_FAIL_COUNT. Clearing target and marking unreachable." << std::endl;
                    assigned_centers_.erase(current_target);
                    unreachable_centers_.insert(current_target);
                    drone_target_centers_.erase(robot_id);
                    target_fail_counts_[robot_id].erase(current_target);
                    if (last_stuck_inducing_target_cell_.count(robot_id))
                    {
                        last_stuck_inducing_target_cell_.erase(robot_id); // Clear on giving up target
                        std::cout << "Robot " << robot_id << " [idle_action] WallDetect_MAX_FAIL: Cleared last_stuck_inducing_target_cell_ for target (" << current_target.x << "," << current_target.y << ")." << std::endl;
                    }
                }
                // FSM will handle finding new path or new target

                previous_pos_map_[robot_id] = current_pos_this_tick;
                last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
                return ROBOT::ACTION::HOLD;
            }
        }

        // Y-axis swap for simulator
        ROBOT::ACTION action_to_return_to_simulator = planned_action;
        if (planned_action == ROBOT::ACTION::UP)
            action_to_return_to_simulator = ROBOT::ACTION::DOWN;
        else if (planned_action == ROBOT::ACTION::DOWN)
            action_to_return_to_simulator = ROBOT::ACTION::UP;

        previous_pos_map_[robot_id] = current_pos_this_tick;
        last_action_commanded_map_[robot_id] = action_to_return_to_simulator;
        return action_to_return_to_simulator;
    }
    else // No path in cache
    {
        // FSM should assign a target and/or path if appropriate for the current state.
        // If drone is IDLE and in EXPLORE/REEXPLORE, FSM will attempt to assign/re-path.
        // std::cout << "Robot " << robot_id << " [idle_action] No path in cache. Holding for FSM." << std::endl;
        previous_pos_map_[robot_id] = current_pos_this_tick;
        last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }

    // Fallback HOLD
    previous_pos_map_[robot_id] = current_pos_this_tick;
    last_action_commanded_map_[robot_id] = ROBOT::ACTION::HOLD;
    return ROBOT::ACTION::HOLD;
}
