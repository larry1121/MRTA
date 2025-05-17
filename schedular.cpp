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
            std::cout << "[UpdateMapWarn Tick: " << current_tick_ << "] Mismatch! Internal H:" << map_height_ << " W:" << map_width_
                      << ", Received H:" << h << " W:" << w << std::endl;
        }
        if (map_height_ > 10 && map_width_ > 10)
        { // Use internal map_height_ and map_width_ for safety
            std::cout << "[UpdateMap_Debug Tick: " << current_tick_ << "] Using map for known_ratio. Cell (6,5) obj: " << static_cast<int>(known_object_map[5][6])
                      << ", Cell (6,10) obj: " << static_cast<int>(known_object_map[10][6]) << std::endl;
        }
    }
    else
    {
        std::cout << "[UpdateMap_Debug Tick: " << current_tick_ << "] empty known_object_map in update_map_knowledge." << std::endl;
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
    std::cout << "Generated " << virtual_centers_.size() << " virtual centers with gap " << gap << std::endl;

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
    std::cout << "Virtual centers sorted (partially for spiral)." << std::endl;
}

// Pathfinding Member function implementation
bool Scheduler::is_valid_and_not_wall(int r, int c, const std::vector<std::vector<OBJECT>> &known_map)
{
    if (locally_discovered_walls_.count({c, r}))
    { // Coord is {x,y} so {c,r} for set lookup
        std::cout << "[SchedulerDebug] Cell (" << c << "," << r << ") is in locally_discovered_walls_." << std::endl;
        return false;
    }
    return r >= 0 && r < map_height_ && c >= 0 && c < map_width_ && known_map[r][c] != OBJECT::WALL;
}

std::vector<Coord> Scheduler::find_path_bfs(const Coord &start, const Coord &goal,
                                            const vector<vector<OBJECT>> &known_object_map)
{
    std::vector<Coord> path;
    // Check if start or goal is invalid or a wall initially
    if (!is_valid_and_not_wall(start.y, start.x, known_object_map))
    {
        std::cout << "[BFS_Debug] Start position (" << start.x << "," << start.y << ") is invalid or a wall. No path." << std::endl;
        return {};
    }
    if (!is_valid_and_not_wall(goal.y, goal.x, known_object_map))
    {
        std::cout << "[BFS_Debug] Goal position (" << goal.x << "," << goal.y << ") is invalid or a wall. No path." << std::endl;
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

    for (const auto &center : virtual_centers_)
    {
        if (visited_centers_.count(center) || assigned_centers_.count(center) || unreachable_centers_.count(center))
        {
            continue;
        }

        // Check if the center itself is a wall or locally discovered wall
        if (!is_valid_and_not_wall(center.y, center.x, known_object_map))
        {
            // std::cout << "[CenterCheck] Center (" << center.x << "," << center.y << ") is a wall or locally discovered wall. Skipping." << std::endl;
            continue;
        }

        int dist = manhattan_distance(drone_pos, center);
        if (dist < min_dist)
        {
            min_dist = dist;
            best_target = center;
        }
    }

    if (best_target.x != -1)
    { // Found a target
      // assigned_centers_.insert(best_target); // Assign it when path is confirmed or drone starts moving
    }
    return best_target;
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
        std::cout << "Robot " << robot->id << " new exploration target: (" << target_center.x << "," << target_center.y << ")" << std::endl;
        drone_target_centers_[robot->id] = target_center;
        assigned_centers_.insert(target_center); // Mark as assigned

        std::vector<Coord> path = find_path_bfs(current_pos, target_center, known_object_map);
        if (!path.empty())
        {
            waypoint_cache_[robot->id] = path;
            if (path.size() > 1)
                current_waypoint_idx_[robot->id] = 1; // BFS path usually includes start, so skip current pos
            else
                current_waypoint_idx_[robot->id] = 0; // Path is just the target itself or empty after this check
        }
        else
        {
            std::cout << "Robot " << robot->id << " could not find path to (" << target_center.x << "," << target_center.y << ")" << std::endl;
            assigned_centers_.erase(target_center);
            unreachable_centers_.insert(target_center); // Add to unreachable set
            std::cout << "Robot " << robot->id << " added (" << target_center.x << "," << target_center.y << ") to unreachable_centers_." << std::endl;
            drone_target_centers_.erase(robot->id);
        }
    }
    else
    {
        std::cout << "Robot " << robot->id << " found no unassigned centers." << std::endl;
        // No more centers to explore for this drone, based on current logic
        // FSM should handle transition to WAIT_TASKS or HOLD
    }
}

void Scheduler::handle_drone_fsm(const shared_ptr<ROBOT> &robot,
                                 const vector<vector<vector<int>>> &known_cost_map, // Not used due to uniform cost
                                 const vector<vector<OBJECT>> &known_object_map)
{
    if (robot->type != ROBOT::TYPE::DRONE)
        return;

    DroneState &current_state = drone_states_[robot->id];
    Coord robot_pos = robot->get_coord();

    // State Transitions
    if (current_state == DroneState::EXPLORE || current_state == DroneState::REEXPLORE)
    {
        if (known_ratio_ >= EXPLORATION_GOAL_RATIO)
        {
            if (!explore_complete_logged_ && current_state == DroneState::EXPLORE) // Log for initial EXPLORE
            {
                std::cout << "EXPLORE_COMPLETE at tick " << current_tick_ << " with known ratio " << known_ratio_ * 100 << "%" << std::endl;
                explore_complete_logged_ = true;
            }
            // explore_complete_logged_ is reset to false by on_info_updated when re_explore_triggered_ becomes true
            if (!explore_complete_logged_ && current_state == DroneState::REEXPLORE && re_explore_triggered_)
            {
                std::cout << "RE-EXPLORE_COMPLETE at tick " << current_tick_ << " with known ratio " << known_ratio_ * 100 << "%" << std::endl;
                explore_complete_logged_ = true; // Mark as logged for this re-explore phase
            }

            // Transition logic based on achieving exploration goal
            if (current_state == DroneState::EXPLORE)
            {
                if (current_tick_ < REEXPLORE_TICK_THRESHOLD)
                {
                    current_state = DroneState::WAIT_TASKS;
                    std::cout << "Robot " << robot->id << " transitioning from EXPLORE to WAIT_TASKS" << std::endl;
                    waypoint_cache_.erase(robot->id);
                    drone_target_centers_.erase(robot->id);
                }
                else
                {
                    // If EXPLORE goal is met and current_tick_ >= REEXPLORE_TICK_THRESHOLD,
                    // on_info_updated handles the global transition to REEXPLORE state for all drones.
                    // This drone will then adopt the REEXPLORE state in the next FSM cycle via on_info_updated.
                    // No direct state change to REEXPLORE here; rely on the global mechanism.
                    // It effectively means EXPLORE phase is done and it will switch to REEXPLORE.
                }
            }
            else if (current_state == DroneState::REEXPLORE)
            { // REEXPLORE goal met
                current_state = DroneState::HOLD;
                std::cout << "Robot " << robot->id << " transitioning from REEXPLORE to HOLD" << std::endl;
                waypoint_cache_.erase(robot->id);
                drone_target_centers_.erase(robot->id);
            }
        }
    }

    // The global re-exploration trigger (setting states to REEXPLORE for all drones
    // and setting re_explore_triggered_ = true) is handled in on_info_updated.
    // handle_drone_fsm acts based on the state set by on_info_updated.

    // Actions within States
    switch (current_state)
    {
    case DroneState::EXPLORE:
    case DroneState::REEXPLORE:
        // If robot is idle (not moving towards a waypoint_cache target) or has no target center
        if (robot->get_status() == ROBOT::STATUS::IDLE)
        {
            bool has_target_center = drone_target_centers_.count(robot->id);
            bool path_finished = !waypoint_cache_.count(robot->id) ||
                                 current_waypoint_idx_[robot->id] >= waypoint_cache_[robot->id].size();

            if (has_target_center && robot_pos == drone_target_centers_[robot->id])
            { // Reached the high-level center
                std::cout << "Robot " << robot->id << " reached center: (" << robot_pos.x << "," << robot_pos.y << ")" << std::endl;
                visited_centers_.insert(robot_pos);
                assigned_centers_.erase(robot_pos); // Unassign as it's now visited
                drone_target_centers_.erase(robot->id);
                waypoint_cache_.erase(robot->id);
                current_waypoint_idx_[robot->id] = 0;
                assign_new_exploration_target(robot, known_object_map); // Get next one
            }
            else if (path_finished || !has_target_center)
            { // Path finished or no center assigned
                // If path finished but not at target center (e.g. path was empty), or no target at all
                if (has_target_center && drone_target_centers_[robot->id] != robot_pos && path_finished)
                {
                    // This case implies it had a target, path is done, but not at target.
                    // This might happen if path was short or target was current pos.
                    // Or if path calculation failed earlier and it's trying to re-evaluate.
                    std::cout << "Robot " << robot->id << " path finished, but not at center. Re-evaluating." << std::endl;
                    // It should have been marked visited if at target. If not, re-assign.
                    visited_centers_.insert(drone_target_centers_[robot->id]); // Assume it was reached if path is done.
                    assigned_centers_.erase(drone_target_centers_[robot->id]);
                    drone_target_centers_.erase(robot->id);
                }
                assign_new_exploration_target(robot, known_object_map);
            }
            // If it has a target and a path, idle_action will handle movement.
        }
        break;
    case DroneState::WAIT_TASKS:
        // Drone just holds, task handling is not part of this spec for drones
        // No specific action here, idle_action will return HOLD
        break;
    case DroneState::HOLD:
        // Exploration and (re-exploration if triggered) are complete.
        // No specific action here, idle_action will return HOLD
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
        std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Received map H:" << h << ", W:" << w << std::endl;
        // Print some critical/problematic cells previously identified
        if (h > 10 && w > 10)
        { // Basic boundary check for test coordinates
            std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Cell (6,5) obj: " << static_cast<int>(known_object_map[5][6])
                      << ", Cell (6,10) obj: " << static_cast<int>(known_object_map[10][6])
                      << ", Cell (2,10) obj: " << static_cast<int>(known_object_map[10][2]) << std::endl;
        }
        if (h > 18 && w > 18)
        {
            std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Cell (18,17) obj: " << static_cast<int>(known_object_map[17][18])
                      << ", Cell (14,14) obj: " << static_cast<int>(known_object_map[14][14]) << std::endl;
        }
    }
    else
    {
        std::cout << "[OnInfoUpdated_Debug Tick: " << current_tick_ << "] Received empty known_object_map." << std::endl;
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

    // Placeholder for accessing robots in handle_drone_fsm's re-explore logic
    // This is a bit of a hack. Ideally, `robots` would be a member or passed differently.
    static const vector<shared_ptr<ROBOT>> *robots_ptr_for_fsm = &robots;
    // This static variable trick is problematic if on_info_updated instances are different
    // For now, we'll try to pass robots directly or make it a member if issues arise.
    // For the re-explore loop:
    // if (current_tick_ >= REEXPLORE_TICK_THRESHOLD && !re_explore_triggered_) {
    //    // ... logic from handle_drone_fsm for triggering re-explore ...
    //    // This needs access to the `robots` list to reset all drone states.
    // }
    // Let's refine this: the FSM transition for REEXPLORE should occur within handle_drone_fsm,
    // and it can iterate over the `robots` list passed to on_info_updated for resetting states.
    // The placeholder `robots_for_fsm_access_placeholder` in `handle_drone_fsm` needs to be replaced.
    // It should be:
    // if (current_tick_ >= REEXPLORE_TICK_THRESHOLD && !re_explore_triggered_) {
    //     // ...
    //     for (const auto& r_ptr : *robots_ptr_for_fsm) { // or just `robots` if passed in
    //         if (r_ptr->type == ROBOT::TYPE::DRONE) { /* reset logic */ }
    //     }
    // }
    // The `handle_drone_fsm` needs the full robots list for the re-exploration trigger.
    // Let's modify `handle_drone_fsm` signature or pass `robots` to it.
    // For now, the FSM logic for re-exploration in handle_drone_fsm will be adapted
    // to use the 'robots' list available in on_info_updated's scope if needed,
    // or handle individual robot state changes without needing the full list in *that exact moment*.

    // The re-exploration trigger needs to iterate all drones. This should be done once.
    if (current_tick_ >= REEXPLORE_TICK_THRESHOLD && !re_explore_triggered_)
    {
        std::cout << "Tick " << current_tick_ << ": Global REEXPLORE trigger initiated." << std::endl;
        visited_centers_.clear();
        assigned_centers_.clear();
        unreachable_centers_.clear(); // Clear unreachable centers on re-explore

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
        explore_complete_logged_ = false; // Allow logging for re-exploration phase
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

    // drone_states_에 robot.id가 없을 경우를 대비 (on_info_updated에서 처리되지만 방어적으로)
    if (drone_states_.find(robot.id) == drone_states_.end())
    {
        std::cerr << "Warning: Drone " << robot.id << " state not found in idle_action. Holding." << std::endl;
        return ROBOT::ACTION::HOLD;
    }

    Coord current_pos_this_tick = robot.get_coord();

    // Stuck detection logic
    if (last_action_commanded_map_.count(robot.id) &&
        last_action_commanded_map_.at(robot.id) != ROBOT::ACTION::HOLD && // Was commanded to move
        previous_pos_map_.count(robot.id) &&
        previous_pos_map_.at(robot.id) == current_pos_this_tick) // And position hasn't changed
    {
        ROBOT::ACTION last_action = last_action_commanded_map_.at(robot.id);
        Coord stuck_pos = previous_pos_map_.at(robot.id);
        Coord failed_target_coord = stuck_pos; // Initialize with stuck_pos

        // Determine the cell the robot tried to move into based on simulator's action interpretation
        // Action 0 (Sim UP, y+), 1 (Sim DOWN, y-), 2 (Sim LEFT, x-), 3 (Sim RIGHT, x+)
        if (last_action == ROBOT::ACTION::UP)
        { // Simulator's UP (0), our previously returned ROBOT::ACTION::UP
            failed_target_coord.y++;
        }
        else if (last_action == ROBOT::ACTION::DOWN)
        { // Simulator's DOWN (1)
            failed_target_coord.y--;
        }
        else if (last_action == ROBOT::ACTION::LEFT)
        { // Simulator's LEFT (2)
            failed_target_coord.x--;
        }
        else if (last_action == ROBOT::ACTION::RIGHT)
        { // Simulator's RIGHT (3)
            failed_target_coord.x++;
        }

        if (failed_target_coord != stuck_pos)
        { // Ensure it was a directional move
            locally_discovered_walls_.insert(failed_target_coord);
            std::cout << "Robot " << robot.id << " [idle_action] Added (" << failed_target_coord.x << "," << failed_target_coord.y
                      << ") to locally_discovered_walls_." << std::endl;
        }

        std::cout << "Robot " << robot.id << " [idle_action] STUCK DETECTED at ("
                  << current_pos_this_tick.x << "," << current_pos_this_tick.y
                  << "). Last action: " << static_cast<int>(last_action) << ". Clearing path and target." << std::endl;

        waypoint_cache_.erase(robot.id);
        current_waypoint_idx_.erase(robot.id);
        if (drone_target_centers_.count(robot.id))
        {
            Coord high_level_target = drone_target_centers_.at(robot.id);
            assigned_centers_.erase(high_level_target);
            drone_target_centers_.erase(robot.id);
            std::cout << "Robot " << robot.id << " [idle_action] Cleared high-level target due to being stuck." << std::endl;
        }

        previous_pos_map_[robot.id] = current_pos_this_tick;
        last_action_commanded_map_[robot.id] = ROBOT::ACTION::HOLD; // Command HOLD now
        return ROBOT::ACTION::HOLD;
    }

    if (robot.get_status() != ROBOT::STATUS::IDLE)
    {
        // Update maps before returning, so next tick knows current pos and that we commanded HOLD implicitly
        previous_pos_map_[robot.id] = current_pos_this_tick;
        last_action_commanded_map_[robot.id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }

    DroneState current_state = drone_states_[robot.id];

    if (current_state == DroneState::WAIT_TASKS || current_state == DroneState::HOLD)
    {
        previous_pos_map_[robot.id] = current_pos_this_tick;
        last_action_commanded_map_[robot.id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }

    // EXPLORE or REEXPLORE state
    if (waypoint_cache_.count(robot.id) && !waypoint_cache_[robot.id].empty())
    {
        size_t &waypoint_idx = current_waypoint_idx_[robot.id];
        const std::vector<Coord> &path = waypoint_cache_[robot.id];
        Coord current_pos = robot.get_coord();

        if (waypoint_idx >= path.size())
        {
            std::cout << "Robot " << robot.id << " [idle_action] has out-of-bounds waypoint_idx. Clearing path." << std::endl;
            waypoint_cache_.erase(robot.id);
            if (drone_target_centers_.count(robot.id))
            {
                assigned_centers_.erase(drone_target_centers_[robot.id]);
                drone_target_centers_.erase(robot.id);
            }
            return ROBOT::ACTION::HOLD;
        }

        Coord target_waypoint_for_this_step = path[waypoint_idx];

        if (current_pos == target_waypoint_for_this_step)
        {
            waypoint_idx++;
            if (waypoint_idx >= path.size())
            {
                std::cout << "Robot " << robot.id << " [idle_action] completed path to target center " << current_pos.x << "," << current_pos.y << std::endl;
                visited_centers_.insert(current_pos);
                if (drone_target_centers_.count(robot.id) && drone_target_centers_[robot.id] == current_pos)
                {
                    assigned_centers_.erase(drone_target_centers_[robot.id]);
                    drone_target_centers_.erase(robot.id);
                }
                else
                {
                    assigned_centers_.erase(current_pos);
                }
                waypoint_cache_.erase(robot.id);
                current_waypoint_idx_.erase(robot.id);
                return ROBOT::ACTION::HOLD;
            }
            target_waypoint_for_this_step = path[waypoint_idx];
        }

        ROBOT::ACTION planned_action = get_move_action_to_target(current_pos, target_waypoint_for_this_step);

        if (planned_action == ROBOT::ACTION::HOLD)
        {
            return ROBOT::ACTION::HOLD;
        }

        Coord immediate_target_cell = current_pos;
        if (planned_action == ROBOT::ACTION::UP)
            immediate_target_cell.y--;
        else if (planned_action == ROBOT::ACTION::DOWN)
            immediate_target_cell.y++;
        else if (planned_action == ROBOT::ACTION::LEFT)
            immediate_target_cell.x--;
        else if (planned_action == ROBOT::ACTION::RIGHT)
            immediate_target_cell.x++;

        if (map_height_ == 0 || map_width_ == 0)
        {
            std::cerr << "Error: map dimensions (H:" << map_height_ << ", W:" << map_width_ << ") not set in idle_action for robot " << robot.id << std::endl;
            return ROBOT::ACTION::HOLD;
        }

        // Debugging output before is_valid_and_not_wall
        if (immediate_target_cell.y >= 0 && immediate_target_cell.y < map_height_ &&
            immediate_target_cell.x >= 0 && immediate_target_cell.x < map_width_)
        {
            std::cout << "Robot " << robot.id << " [idle_action_debug]: Checking cell (" << immediate_target_cell.x << "," << immediate_target_cell.y
                      << "). Scheduler sees it as: " << static_cast<int>(known_object_map[immediate_target_cell.y][immediate_target_cell.x])
                      << " (OBJECT enum val)" << std::endl;
        }
        else
        {
            std::cout << "Robot " << robot.id << " [idle_action_debug]: Immediate target cell (" << immediate_target_cell.x << "," << immediate_target_cell.y
                      << ") is OUT OF BOUNDS for known_object_map (H:" << map_height_ << ", W:" << map_width_ << ")" << std::endl;
        }

        bool is_valid_move = is_valid_and_not_wall(immediate_target_cell.y, immediate_target_cell.x, known_object_map);
        std::cout << "Robot " << robot.id << " [idle_action_debug]: is_valid_move result: " << (is_valid_move ? "true" : "false") << std::endl;

        if (!is_valid_move)
        {
            std::cout << "Robot " << robot.id << " [idle_action] detected wall/invalid for next step " // 메시지 수정
                      << "(" << immediate_target_cell.x << "," << immediate_target_cell.y << ") from ("
                      << current_pos.x << "," << current_pos.y << ") towards path waypoint ("
                      << target_waypoint_for_this_step.x << "," << target_waypoint_for_this_step.y
                      << "). Invalidating path and current high-level target." << std::endl;

            waypoint_cache_.erase(robot.id);
            current_waypoint_idx_.erase(robot.id);

            if (drone_target_centers_.count(robot.id))
            {
                Coord high_level_target = drone_target_centers_[robot.id];
                assigned_centers_.erase(high_level_target);
                drone_target_centers_.erase(robot.id);
                std::cout << "Robot " << robot.id << " [idle_action] Cleared high-level target (" << high_level_target.x << "," << high_level_target.y << ")" << std::endl;
            }
            // Update maps before returning HOLD
            previous_pos_map_[robot.id] = current_pos_this_tick; // Assuming current_pos_this_tick is available
            last_action_commanded_map_[robot.id] = ROBOT::ACTION::HOLD;
            return ROBOT::ACTION::HOLD;
        }

        ROBOT::ACTION action_to_return_to_simulator;
        if (planned_action == ROBOT::ACTION::UP)
        {
            action_to_return_to_simulator = ROBOT::ACTION::DOWN;
        }
        else if (planned_action == ROBOT::ACTION::DOWN)
        {
            action_to_return_to_simulator = ROBOT::ACTION::UP;
        }
        else
        {
            action_to_return_to_simulator = planned_action; // LEFT, RIGHT, HOLD
        }

        previous_pos_map_[robot.id] = current_pos_this_tick; // Assuming current_pos_this_tick is available
        last_action_commanded_map_[robot.id] = action_to_return_to_simulator;
        return action_to_return_to_simulator;
    }
    else // 경로 캐시가 비어있는 경우
    {
        previous_pos_map_[robot.id] = current_pos_this_tick; // Assuming current_pos_this_tick is available
        last_action_commanded_map_[robot.id] = ROBOT::ACTION::HOLD;
        return ROBOT::ACTION::HOLD;
    }

    // Defensive HOLD at the very end if somehow logic falls through (should not happen)
    previous_pos_map_[robot.id] = current_pos_this_tick; // Assuming current_pos_this_tick is available
    last_action_commanded_map_[robot.id] = ROBOT::ACTION::HOLD;
    return ROBOT::ACTION::HOLD;
}
