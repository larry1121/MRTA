#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <deque>
#include <unordered_map>
#include <tuple>
#include <limits>
#include <algorithm>
#include <functional>
#include <iomanip>

// Algorithm selection macros
#define USE_MINMIN
// #define USE_SUFFERAGE
//#define USE_OLB

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

// Add after the PathInfo struct
struct TaskCluster {
    vector<int> task_ids;  // 클러스터에 포함된 태스크 ID들
    int total_cost;        // 클러스터의 총 비용
    int start_task_id;     // 시작 태스크 ID
    int end_task_id;       // 끝 태스크 ID
    Coord start_pos;       // 시작 위치
    Coord end_pos;         // 끝 위치

    TaskCluster() : total_cost(0), start_task_id(-1), end_task_id(-1) {}
};

class Scheduler
{
public:
    void on_info_updated(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots);

    bool on_task_reached(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const ROBOT& robot,
        const TASK& task);

    ROBOT::ACTION idle_action(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const ROBOT& robot);
    Scheduler();

private:
    // Constants for clustering and energy management
    const int CLUSTER_DISTANCE_THRESHOLD = 1400;  // 클러스터링 거리 임계값
    const int ENERGY_MARGIN_PERCENT = 10;         // 에너지 여유 비율 (%)
    const int MAX_CLUSTER_SIZE = 3;              // 최대 클러스터 크기 (태스크 개수)

    int  tick_counter_ = 0;   // 틱 카운트
    bool has_started_assignments = false; // 100틱 이후 true

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


    /* ---------- DRONE(맵 탐사) ---------- */
    struct DRONE_TileInfo { Coord center; int unseen_cells = 0; };

    int    DRONE_tile_size = 5;
    int    DRONE_tile_range = 2;

    int    DRONE_map_size = -1;
    int    DRONE_tile_rows = 0;
    int    DRONE_tile_cols = 0;

    std::vector<std::vector<DRONE_TileInfo>> DRONE_tiles;
    std::unordered_map<int, std::deque<Coord>> DRONE_drone_paths;
    std::unordered_map<int, Coord>             DRONE_drone_targets;
    std::set<std::pair<int, int>>               DRONE_assigned_targets;

    /*   드론-최적화 파라미터   */
    int    DRONE_distance_threshold = 0;
    double DRONE_high_priority_weight = 80.0;
    double DRONE_mid_priority_weight = 60.0;
    double DRONE_candidate_threshold = 0.005;
    int    DRONE_exploration_pause_start = 100;
    int    DRONE_exploration_resume = 950;

    // Helper methods for task assignment triggers
    bool shouldTriggerReassignment(const set<Coord>& updated_coords,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots) const;
    void checkForNewTasks(const vector<shared_ptr<TASK>>& active_tasks);
    void checkForCompletedTasks(const vector<shared_ptr<TASK>>& active_tasks);
    void checkForExhaustedRobots(const vector<shared_ptr<ROBOT>>& robots);
    void checkForMapChanges(const set<Coord>& updated_coords);

    /* ─────── DRONE helper ─────── */
    /* 이름 충돌 방지를 위해 모두 DRONE_ 접두사 */
    static bool DRONE_coord_equal(const Coord&, const Coord&);
    void  DRONE_init_tiles(const std::vector<std::vector<OBJECT>>&);
    void  DRONE_update_tile_info(const std::vector<std::vector<OBJECT>>&);
    std::vector<Coord> DRONE_plan_path(const Coord&, const Coord&,
        const std::vector<std::vector<std::vector<int>>>&,
        const ROBOT&,
        const std::vector<std::vector<OBJECT>>&);

    ROBOT::ACTION DRONE_get_direction(const Coord&, const Coord&);
    bool  DRONE_is_exploration_time() const;

    // Helper to convert action to coordinate change
    Coord action_to_delta(ROBOT::ACTION action) {
        switch (action) {
        case ROBOT::ACTION::UP:    return { 0, 1 };
        case ROBOT::ACTION::DOWN:  return { 0, -1 };
        case ROBOT::ACTION::LEFT:  return { -1, 0 };
        case ROBOT::ACTION::RIGHT: return { 1, 0 };
        case ROBOT::ACTION::HOLD:  return { 0, 0 };
        default:                   return { 0, 0 }; // Should not happen
        }
    }

    // Dijkstra pathfinding algorithm
    int dijkstra(const Coord& start,
        const Coord& goal,
        const ROBOT& robot,
        const int task_cost_for_robot,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        int map_size,
        std::vector<ROBOT::ACTION>& out_path_actions,
        std::vector<Coord>& out_path_coords);

    // Task assignment logic
    void performMinMinAssignment(const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const vector<vector<vector<int>>>& known_cost_map,
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
            : taskId(tid), robotId(rid), completionTime(ct), pathCost(pc), taskCost(tc) {
        }
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
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<ROBOT>>& robots);

    bool isTaskAlreadyAssigned(int taskId) const;

    void perform_task_assignment(const vector<shared_ptr<ROBOT>>& robots,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<vector<OBJECT>>& known_object_map);

    void performSufferageAssignment(const vector<shared_ptr<ROBOT>>& robots,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    void performOLBAssignment(const vector<shared_ptr<ROBOT>>& robots,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    // Path cache structures
    struct TaskPathInfo {
        std::vector<ROBOT::ACTION> actions;
        std::vector<Coord> coordinates;
        int path_cost;
        int task_cost;
        Coord task_coord;

        TaskPathInfo() : path_cost(std::numeric_limits<int>::max()), task_cost(std::numeric_limits<int>::max()) {}
        TaskPathInfo(const std::vector<ROBOT::ACTION>& acts,
            const std::vector<Coord>& coords,
            int pc, int tc, const Coord& tcrd)
            : actions(acts), coordinates(coords), path_cost(pc), task_cost(tc), task_coord(tcrd) {
        }
    };

    // Robot_id -> queue of TaskPathInfo for each task in the robot's queue
    std::map<int, std::queue<TaskPathInfo>> robot_path_cache;

    // Helper methods for path cache
    void updatePathCache(int robot_id,
        const std::vector<shared_ptr<TASK>>& active_tasks,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<ROBOT>>& robots);

    void clearPathCache(int robot_id);
    bool isPathCacheValid(int robot_id, const Coord& current_pos) const;

    // Add new private members
    vector<TaskCluster> task_clusters;

    // Add new private methods
    void clusterTasks(const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    void findClusterEndpoints(TaskCluster& cluster,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    int calculatePathCost(const Coord& start, const Coord& end,
        const ROBOT& robot,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    static constexpr int UNKNOWN_COST_CATERPILLAR = 398;
    static constexpr int UNKNOWN_COST_WHEEL = 846;

    // 변하지 않는 helper
    static inline int default_unknown_cost(ROBOT::TYPE t)
    {
        switch (t)
        {
        case ROBOT::TYPE::CATERPILLAR: return UNKNOWN_COST_CATERPILLAR;
        case ROBOT::TYPE::WHEEL:       return UNKNOWN_COST_WHEEL;
        default:                       return INFINITE / 4; // 드론은 안 씀
        }
    }

    // ★ 새 calculate_move_cost ★
    static inline int calculate_move_cost(const Coord& c1, const Coord& c2,
        ROBOT::TYPE r_type,
        const vector<vector<vector<int>>>& cost_map)
    {
        if (c1 == c2) return 0;

        const int idx = static_cast<int>(r_type);
        int cost1 = cost_map[c1.x][c1.y][idx];
        int cost2 = cost_map[c2.x][c2.y][idx];

        //  -1 (=미탐색) 이면 각 타입의 ‘최대값’으로 대체
        if (cost1 == -1) cost1 = default_unknown_cost(r_type);
        if (cost2 == -1) cost2 = default_unknown_cost(r_type);

        if (cost1 == INFINITE || cost2 == INFINITE) return INFINITE;
        return (cost1 + cost2) / 2;
    }

public:

    //void set_tile_parameters(int size, int range) { DRONE_tile_size = size; DRONE_tile_range = range; }
    //void set_optimization_parameters(int dist, bool high, double mid, double cand, int pause, int resume)
    //{
    //    DRONE_distance_threshold = dist; DRONE_high_priority_weight = high; DRONE_mid_priority_weight = mid;
    //    DRONE_candidate_threshold = cand; DRONE_exploration_pause_start = pause; DRONE_exploration_resume = resume;
    //}

    ///* ------------- static util ------------- */
    //static inline int calculate_move_cost(const Coord&, const Coord&, ROBOT::TYPE,
    //    const std::vector<std::vector<std::vector<int>>>&);

    ///* 디버깅용 출력 */
    //void print_robot_task_queues(const std::vector<std::shared_ptr<ROBOT>>&,
    //    const std::vector<std::shared_ptr<TASK>>&) const;

    // Static helper function for move cost
    //static inline int calculate_move_cost(const Coord& c1, const Coord& c2, ROBOT::TYPE r_type,
    //    const vector<vector<vector<int>>>& cost_map)
    //{
    //    if (c1 == c2) return 0; // No cost if not moving

    //    int type_idx = static_cast<int>(r_type);
    //    int cost1 = cost_map[c1.x][c1.y][type_idx];
    //    int cost2 = cost_map[c2.x][c2.y][type_idx];

    //    if (cost1 == INFINITE || cost2 == INFINITE) {
    //        return INFINITE;
    //    }
    //    return (cost1 + cost2) / 2;
    //}

    // Method to print the task_total_costs table
    void print_task_total_costs_table(const vector<shared_ptr<ROBOT>>& all_robots, const vector<shared_ptr<TASK>>& all_tasks) const;

    // Debug method to print robot task queues
    void print_robot_task_queues(const vector<shared_ptr<ROBOT>>& robots, const vector<shared_ptr<TASK>>& active_tasks) const {
        std::cout << "\n=== Robot Task Queues Status ===" << std::endl;
        for (const auto& robot : robots) {
            std::cout << "Robot " << robot->id << " (" << robot->type << "): ";
            if (robotTaskQueue.count(robot->id) && !robotTaskQueue.at(robot->id).empty()) {
                std::cout << "Queue: [";
                std::queue<int> temp_queue = robotTaskQueue.at(robot->id);
                bool first = true;
                while (!temp_queue.empty()) {
                    if (!first) std::cout << " -> ";
                    int task_id = temp_queue.front();
                    temp_queue.pop();

                    // Find task coordinates
                    Coord task_coord;
                    for (const auto& task : active_tasks) {
                        if (task->id == task_id) {
                            task_coord = task->coord;
                            break;
                        }
                    }

                    std::cout << "Task " << task_id << " at " << task_coord;
                    first = false;
                }
                std::cout << "]";
            }
            else {
                std::cout << "No tasks in queue";
            }
            std::cout << " | Current Position: " << robot->get_coord();
            if (robotExpectedPosition.count(robot->id)) {
                std::cout << " | Expected Position: " << robotExpectedPosition.at(robot->id);
            }
            std::cout << " | Energy: " << robot->get_energy();
            std::cout << " | Status: " << robot->get_status();
            std::cout << std::endl;
        }
        std::cout << "==============================\n" << std::endl;
    }
};

#endif