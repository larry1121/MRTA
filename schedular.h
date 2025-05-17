#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits> // Required for std::numeric_limits

// Forward declaration if Coord is defined elsewhere and not included by simulator.h
// struct Coord;

class Scheduler
{
public:
    // Enum for FSM states, nested as requested
    enum class DroneState
    {
        EXPLORE,
        WAIT_TASKS,
        REEXPLORE,
        HOLD
    };

    // Constructor (if needed for initialization)
    Scheduler();

    void on_info_updated(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots);

    bool on_task_reached(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots,
                         const ROBOT &robot,
                         const TASK &task);

    ROBOT::ACTION idle_action(const set<Coord> &observed_coords,
                              const set<Coord> &updated_coords,
                              const vector<vector<vector<int>>> &known_cost_map,
                              const vector<vector<OBJECT>> &known_object_map,
                              const vector<shared_ptr<TASK>> &active_tasks,
                              const vector<shared_ptr<ROBOT>> &robots,
                              const ROBOT &robot);

private:
    // Member Variables
    std::map<int, DroneState> drone_states_;
    std::map<int, std::vector<Coord>> waypoint_cache_; // Stores path for each drone
    std::map<int, size_t> current_waypoint_idx_;       // Current target waypoint index in the cache
    std::map<int, Coord> drone_target_centers_;        // Current high-level spiral center target for each drone

    int map_width_ = 0;
    int map_height_ = 0;
    int current_tick_ = 0;
    double known_ratio_ = 0.0;

    std::vector<Coord> virtual_centers_; // List of all potential centers to visit
    std::set<Coord> assigned_centers_;   // Centers currently assigned or path planned for a drone
    std::set<Coord> visited_centers_;    // Centers that have been reached by a drone

    bool initial_map_setup_done_ = false;
    bool explore_complete_logged_ = false;
    bool re_explore_triggered_ = false;

    // Compile-time constant as per requirements
    static constexpr int TOTAL_TICKS = 2000;
    static constexpr double EXPLORATION_GOAL_RATIO = 0.95;
    static constexpr int REEXPLORE_TICK_THRESHOLD = 1500;

    // Members for stuck detection
    std::map<int, Coord> previous_pos_map_;
    std::map<int, ROBOT::ACTION> last_action_commanded_map_;
    std::set<Coord> locally_discovered_walls_; // For walls found by failed moves
    std::set<Coord> unreachable_centers_;      // Centers found to be unreachable by BFS

    // Private Helper Functions
    void initialize_scheduler_state(const vector<vector<OBJECT>> &known_object_map, const vector<shared_ptr<ROBOT>> &robots);
    void update_map_knowledge(const vector<vector<OBJECT>> &known_object_map, const vector<shared_ptr<ROBOT>> &robots);

    void generate_virtual_centers_grid();
    void sort_centers_for_spiral_path(std::vector<Coord> &centers); // To order virtual_centers_

    // Pathfinding
    std::vector<Coord> find_path_bfs(const Coord &start, const Coord &goal,
                                     const vector<vector<OBJECT>> &known_object_map);
    ROBOT::ACTION get_move_action_to_target(const Coord &current_pos, const Coord &target_pos);
    bool is_valid_and_not_wall(int r, int c, const std::vector<std::vector<OBJECT>> &known_map);

    // Drone FSM and Action Logic
    void handle_drone_fsm(const shared_ptr<ROBOT> &robot,
                          const vector<vector<vector<int>>> &known_cost_map,
                          const vector<vector<OBJECT>> &known_object_map);

    Coord get_closest_unassigned_center(const Coord &drone_pos, const std::shared_ptr<ROBOT> &robot, const vector<vector<OBJECT>> &known_object_map);
    void assign_new_exploration_target(const shared_ptr<ROBOT> &robot,
                                       const vector<vector<OBJECT>> &known_object_map);
};

#endif /* SCHEDULER_H_ */
