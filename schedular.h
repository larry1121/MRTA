#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <queue>
#include <map>
#include <limits> // For std::numeric_limits
#include <algorithm> // For std::reverse, std::min

// Forward declaration
class ROBOT;
class TASK;
struct Coord; // Already in simulator.h, but good for clarity if used standalone

// Helper struct to store path information
struct PathInfo {
    std::vector<ROBOT::ACTION> actions;
    int cost;
    std::vector<Coord> coordinates; // To check for cache invalidation

    PathInfo() : cost(std::numeric_limits<int>::max()) {}
    PathInfo(std::vector<ROBOT::ACTION> acts, int c, std::vector<Coord> coords) : actions(std::move(acts)), cost(c), coordinates(std::move(coords)) {}
};

class Scheduler
{
public:
    // Algorithm selection enum
    enum class Algorithm {
        MIN_MIN,    // Original algorithm
        SUFFERAGE,  // Sufferage algorithm
        OLB         // Opportunistic Load Balancing algorithm
    };

    // Constructor to set initial algorithm
    Scheduler(Algorithm initial_algorithm = Algorithm::MIN_MIN) : current_algorithm(initial_algorithm) {}

    // Method to toggle algorithm
    void toggle_algorithm() {
        switch (current_algorithm) {
            case Algorithm::MIN_MIN:
                current_algorithm = Algorithm::SUFFERAGE;
                break;
            case Algorithm::SUFFERAGE:
                current_algorithm = Algorithm::OLB;
                break;
            case Algorithm::OLB:
                current_algorithm = Algorithm::MIN_MIN;
                break;
        }
    }

    // Method to get current algorithm
    Algorithm get_current_algorithm() const { return current_algorithm; }

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
    // Current algorithm selection
    Algorithm current_algorithm;

    // Robot_id -> goal_coord -> PathInfo
    std::map<int, std::map<Coord, PathInfo>> path_cache;

    // Robot_id -> task_id -> total_cost (path_cost + task_execution_cost)
    std::map<int, std::map<int, int>> task_total_costs;
    
    // Robot_id -> current target task_id (if any) - This might be redundant if robotToTask is primary
    std::map<int, int> robot_target_task_id; // Kept for now, role might merge with robotToTask

    // Robot_id -> current path being followed by the robot
    std::map<int, PathInfo> robot_current_paths;

    // Teammate's structure for assignment: Robot_id -> Task_id
    std::map<int, int> robotToTask; // Using std::map for consistency, teammate used unordered_map

    bool initial_map_revealed_and_assigned = false;

    // Helper to convert action to coordinate change
    Coord action_to_delta(ROBOT::ACTION action) {
        switch (action) {
            case ROBOT::ACTION::UP:    return {0, 1};
            case ROBOT::ACTION::DOWN:  return {0, -1};
            case ROBOT::ACTION::LEFT:  return {-1, 0};
            case ROBOT::ACTION::RIGHT: return {1, 0};
            case ROBOT::ACTION::HOLD:  return {0, 0};
            default:                   return {0, 0}; // Should not happen
        }
    }

    // Dijkstra pathfinding algorithm
    // Returns path cost, fills out_path_actions and out_path_coords.
    // Returns std::numeric_limits<int>::max() if no path or not enough energy.
    int dijkstra(const Coord& start,
                 const Coord& goal,
                 const ROBOT& robot,
                 const int task_cost_for_robot, // task.get_cost(robot.type)
                 const vector<vector<vector<int>>>& known_cost_map,
                 const vector<vector<OBJECT>>& known_object_map,
                 int map_size,
                 std::vector<ROBOT::ACTION>& out_path_actions,
                 std::vector<Coord>& out_path_coords);

    // Task assignment logic (adapted from teammate)
    void perform_task_assignment(const vector<shared_ptr<ROBOT>>& robots,
                                 const vector<shared_ptr<TASK>>& active_tasks,
                                 const vector<vector<OBJECT>>& known_object_map);
    
    // Helper to check if map is fully revealed
    bool is_map_fully_revealed(const vector<vector<OBJECT>>& known_object_map) const;

    // Helper functions for task assignment
    bool is_task_available(const TASK& task,
                          const std::map<int, bool>& task_newly_assigned_map) const {
        return !task.is_done() && !task_newly_assigned_map.count(task.id);
    }

    bool is_robot_available(const ROBOT& robot,
                           const std::map<int, bool>& robot_newly_assigned_map) const {
        return robot.type != ROBOT::TYPE::DRONE &&
               robot.get_status() != ROBOT::STATUS::EXHAUSTED &&
               !robotToTask.count(robot.id) &&
               !robot_newly_assigned_map.count(robot.id);
    }

    bool is_task_already_assigned(int task_id) const {
        for (const auto& [robot_id, assigned_task_id] : robotToTask) {
            if (assigned_task_id == task_id) return true;
        }
        return false;
    }

    // New method to mark task as done and update related maps
    void mark_task_as_done(int task_id,
                          std::map<int, bool>& task_newly_assigned_map,
                          const vector<shared_ptr<TASK>>& active_tasks) {
        // Add to newly assigned map to prevent further assignments
        task_newly_assigned_map[task_id] = true;
    }

    bool can_assign_task_to_robot(int robot_id, int task_id,
                                 const std::map<int, bool>& robot_newly_assigned_map,
                                 const std::map<int, bool>& task_newly_assigned_map) const {
        // Check if robot is already assigned
        if (robotToTask.count(robot_id) || robot_newly_assigned_map.count(robot_id)) {
            return false;
        }
        
        // Check if task is already assigned
        if (is_task_already_assigned(task_id) || task_newly_assigned_map.count(task_id)) {
            return false;
        }
        
        // Check if cost information exists and is valid
        if (!task_total_costs.count(robot_id) ||
            !task_total_costs.at(robot_id).count(task_id) ||
            task_total_costs.at(robot_id).at(task_id) == std::numeric_limits<int>::max()) {
            return false;
        }
        
        return true;
    }

public:
    // Static helper function for move cost
    static inline int calculate_move_cost(const Coord& c1, const Coord& c2, ROBOT::TYPE r_type,
                                 const vector<vector<vector<int>>>& cost_map)
    {
        if (c1 == c2) return 0; // No cost if not moving

        int type_idx = static_cast<int>(r_type);
        // Cost to leave c1 and enter c2. Per problem: (cost[c1] + cost[c2]) / 2
        // From simulator.h, cost_map[x][y] is a vector of costs for different robot types.
        // cost_at(coord, type) returns cost_map[coord.x][coord.y][static_cast<size_t>(type)]
        
        // Ensure coordinates are within map bounds before accessing cost_map
        // This check should ideally be part of a map utility or done by the caller (Dijkstra)
        // For now, assume valid coords are passed or Dijkstra handles it.
        int cost1 = cost_map[c1.x][c1.y][type_idx];
        int cost2 = cost_map[c2.x][c2.y][type_idx];

        if (cost1 == INFINITE || cost2 == INFINITE) {
            return INFINITE;
        }
        return (cost1 + cost2) / 2;
    }
    
    // Method to print the task_total_costs table
    void print_task_total_costs_table(const vector<shared_ptr<ROBOT>>& all_robots, const vector<shared_ptr<TASK>>& all_tasks) const;
};

#endif SCHEDULER_H_
