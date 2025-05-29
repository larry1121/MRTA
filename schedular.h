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
    // Robot_id -> goal_coord -> PathInfo
    std::map<int, std::map<Coord, PathInfo>> path_cache;

    // Robot_id -> task_id -> total_cost (path_cost + task_execution_cost)
    std::map<int, std::map<int, int>> task_total_costs;
    
    // Robot_id -> current target task_id (if any)
    std::map<int, int> robot_target_task_id;

    // Robot_id -> current path being followed by the robot
    std::map<int, PathInfo> robot_current_paths;

    // Teammate's structure for assignment: Robot_id -> Task_id
    std::map<int, int> robotToTask;

    bool initial_map_revealed_and_assigned = false;

    // New variables to track when reassignment is needed
    bool needs_reassignment = false;
    int last_assignment_time = 0;
    std::set<int> newly_discovered_tasks;
    std::set<int> newly_completed_tasks;
    std::set<int> newly_exhausted_robots;

    // Helper methods for task assignment triggers
    bool shouldTriggerReassignment(const set<Coord>& updated_coords,
                                 const vector<shared_ptr<TASK>>& active_tasks,
                                 const vector<shared_ptr<ROBOT>>& robots) const;
    void checkForNewTasks(const vector<shared_ptr<TASK>>& active_tasks);
    void checkForCompletedTasks(const vector<shared_ptr<TASK>>& active_tasks);
    void checkForExhaustedRobots(const vector<shared_ptr<ROBOT>>& robots);
    void checkForMapChanges(const set<Coord>& updated_coords);

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

    // New structures for task assignment
    struct TaskAssignmentInfo {
        int taskId;
        int robotId;
        int completionTime;
        int pathCost;
        int taskCost;
        
        TaskAssignmentInfo(int tid, int rid, int ct, int pc, int tc)
            : taskId(tid), robotId(rid), completionTime(ct), pathCost(pc), taskCost(tc) {}
    };

    // Robot task queue management
    std::map<int, std::queue<int>> robotTaskQueue;  // robotId -> queue of taskIds
    std::map<int, int> robotCurrentTaskEndTime;     // robotId -> current task end time
    std::map<int, Coord> robotExpectedPosition;     // robotId -> expected position after current task

    // Helper methods
    void updateRobotPosition(int robotId, const Coord& newPosition);
    void recalculateCostsForRobot(int robotId,
                                 const vector<shared_ptr<ROBOT>>& robots,
                                 const vector<shared_ptr<TASK>>& active_tasks,
                                 const vector<vector<vector<int>>>& known_cost_map,
                                 const vector<vector<OBJECT>>& known_object_map);
    
    int calculateTaskCompletionTime(int robotId, int taskId,
                                  const vector<shared_ptr<TASK>>& active_tasks,
                                  const vector<vector<vector<int>>>& known_cost_map,
                                  const vector<vector<OBJECT>>& known_object_map);

    // New helper method to check if a task is already assigned
    bool isTaskAlreadyAssigned(int taskId) const;

    // Task assignment algorithms
    void performMinMinAssignment(const vector<shared_ptr<ROBOT>>& robots,
                                const vector<shared_ptr<TASK>>& active_tasks,
                                const vector<vector<vector<int>>>& known_cost_map,
                                const vector<vector<OBJECT>>& known_object_map);
    
    void performSufferageAssignment(const vector<shared_ptr<ROBOT>>& robots,
                                   const vector<shared_ptr<TASK>>& active_tasks,
                                   const vector<vector<vector<int>>>& known_cost_map,
                                   const vector<vector<OBJECT>>& known_object_map);
    
    void performOLBAssignment(const vector<shared_ptr<ROBOT>>& robots,
                             const vector<shared_ptr<TASK>>& active_tasks,
                             const vector<vector<vector<int>>>& known_cost_map,
                             const vector<vector<OBJECT>>& known_object_map);

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
