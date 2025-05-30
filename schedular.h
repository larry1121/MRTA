#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <queue>
#include <map>
#include <deque>
#include <set>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <tuple>
#include <cmath>
#include <iostream>
#include <iomanip>

// 알고리즘 선택
#define USE_MINMIN
// #define USE_SUFFERAGE
// #define USE_OLB

class Scheduler
{
public:
    // 메인 3개 메서드
    void on_info_updated(const std::set<Coord>& observed_coords,
                         const std::set<Coord>& updated_coords,
                         const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                         const std::vector<std::vector<OBJECT>>& known_object_map,
                         const std::vector<std::shared_ptr<TASK>>& active_tasks,
                         const std::vector<std::shared_ptr<ROBOT>>& robots);

    bool on_task_reached(const std::set<Coord>& observed_coords,
                         const std::set<Coord>& updated_coords,
                         const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                         const std::vector<std::vector<OBJECT>>& known_object_map,
                         const std::vector<std::shared_ptr<TASK>>& active_tasks,
                         const std::vector<std::shared_ptr<ROBOT>>& robots,
                         const ROBOT& robot,
                         const TASK& task);

    ROBOT::ACTION idle_action(const std::set<Coord>& observed_coords,
                              const std::set<Coord>& updated_coords,
                              const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                              const std::vector<std::vector<OBJECT>>& known_object_map,
                              const std::vector<std::shared_ptr<TASK>>& active_tasks,
                              const std::vector<std::shared_ptr<ROBOT>>& robots,
                              const ROBOT& robot);

    void set_tile_parameters(int size, int range);
    void set_optimization_parameters(int dist_thresh, double high_weight, double mid_weight, double cand_thresh, int pause_start, int resume_time);
    void reset_scheduler();

    bool is_map_fully_revealed(const std::vector<std::vector<OBJECT>>& known_object_map) const;
    void perform_task_assignment(
        const std::vector<std::shared_ptr<ROBOT>>& robots,
        const std::vector<std::shared_ptr<TASK>>& active_tasks,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        const std::vector<std::vector<OBJECT>>& known_object_map);



    // ----------------- DRONE EXPLORATION LOGIC -----------------
    void update_tile_info(const std::vector<std::vector<OBJECT>>& known_object_map);
    bool is_exploration_time() const;
    std::vector<Coord> plan_path(const Coord& start, const Coord& goal,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map, ROBOT::TYPE type,
        const std::vector<std::vector<OBJECT>>& known_object_map);





private:
    // ----------------- Drone 전용 변수 및 함수 -----------------
    struct TileInfo { Coord center; int unseen_cells = 0; };
    int tile_size = 5;
    int tile_range = 2;
    int map_size = -1, tile_rows = 0, tile_cols = 0;
    std::vector<std::vector<TileInfo>> tiles;
    std::unordered_map<int, std::deque<Coord>> drone_paths;
    std::unordered_map<int, Coord> drone_targets;
    std::set<std::pair<int, int>> assigned_targets;
    int current_time = 0;
    int distance_threshold = 0;
    double high_priority_weight = 80.0;
    double mid_priority_weight = 60.0;
    double candidate_threshold = 0.005;
    int exploration_pause_start = 100, exploration_resume = 950;
    void init_tiles(const std::vector<std::vector<OBJECT>>& known_object_map);
    static bool coord_equal(const Coord& a, const Coord& b);
    ROBOT::ACTION get_direction(const Coord& from, const Coord& to);

    // ----------------- Task Robot 및 공통 변수/함수 -----------------
    struct PathInfo {
        std::vector<ROBOT::ACTION> actions;
        int cost;
        std::vector<Coord> coordinates;
        PathInfo() : cost(std::numeric_limits<int>::max()) {}
        PathInfo(std::vector<ROBOT::ACTION> acts, int c, std::vector<Coord> coords)
            : actions(std::move(acts)), cost(c), coordinates(std::move(coords)) {}
    };

    std::map<int, std::map<Coord, PathInfo>> path_cache;
    std::map<int, std::map<int, int>> task_total_costs;
    std::map<int, int> robot_target_task_id;
    std::map<int, PathInfo> robot_current_paths;
    std::map<int, int> robotToTask;
    bool initial_map_revealed_and_assigned = false;
    bool needs_reassignment = false;
    int last_assignment_time = 0;
    std::set<int> newly_discovered_tasks, newly_completed_tasks, newly_exhausted_robots;
    std::map<int, std::queue<int>> robotTaskQueue;
    std::map<int, int> robotCurrentTaskEndTime;
    std::map<int, Coord> robotExpectedPosition;

    // Helper to convert action to coordinate change
    Coord action_to_delta(ROBOT::ACTION action) {
        switch (action) {
            case ROBOT::ACTION::UP:    return {0, 1};
            case ROBOT::ACTION::DOWN:  return {0, -1};
            case ROBOT::ACTION::LEFT:  return {-1, 0};
            case ROBOT::ACTION::RIGHT: return {1, 0};
            case ROBOT::ACTION::HOLD:  return {0, 0};
            default:                   return {0, 0};
        }
    }

    // Dijkstra for robots
    int dijkstra(const Coord& start,
                 const Coord& goal,
                 const ROBOT& robot,
                 const int task_cost_for_robot,
                 const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                 const std::vector<std::vector<OBJECT>>& known_object_map,
                 int map_size,
                 std::vector<ROBOT::ACTION>& out_path_actions,
                 std::vector<Coord>& out_path_coords);

    // 태스크 할당, 경로 관리 등 (기존)
    void checkForNewTasks(const std::vector<std::shared_ptr<TASK>>& active_tasks);
    void checkForCompletedTasks(const std::vector<std::shared_ptr<TASK>>& active_tasks);
    void checkForExhaustedRobots(const std::vector<std::shared_ptr<ROBOT>>& robots);
    void checkForMapChanges(const std::set<Coord>& updated_coords);
    bool shouldTriggerReassignment(const std::set<Coord>& updated_coords,
        const std::vector<std::shared_ptr<TASK>>& active_tasks,
        const std::vector<std::shared_ptr<ROBOT>>& robots) const;

    void updateRobotPosition(int robotId, const Coord& newPosition);
    void recalculateCostsForRobot(int robotId,
                                 const std::vector<std::shared_ptr<ROBOT>>& robots,
                                 const std::vector<std::shared_ptr<TASK>>& active_tasks,
                                 const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                                 const std::vector<std::vector<OBJECT>>& known_object_map);

    int calculateTaskCompletionTime(int robotId, int taskId,
                                  const std::vector<std::shared_ptr<TASK>>& active_tasks,
                                  const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                                  const std::vector<std::vector<OBJECT>>& known_object_map);

    bool isTaskAlreadyAssigned(int taskId) const;

    void performMinMinAssignment(const std::vector<std::shared_ptr<ROBOT>>& robots,
                                const std::vector<std::shared_ptr<TASK>>& active_tasks,
                                const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                                const std::vector<std::vector<OBJECT>>& known_object_map);
    void performSufferageAssignment(const std::vector<std::shared_ptr<ROBOT>>& robots,
                                   const std::vector<std::shared_ptr<TASK>>& active_tasks,
                                   const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                                   const std::vector<std::vector<OBJECT>>& known_object_map);
    void performOLBAssignment(const std::vector<std::shared_ptr<ROBOT>>& robots,
                             const std::vector<std::shared_ptr<TASK>>& active_tasks,
                             const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                             const std::vector<std::vector<OBJECT>>& known_object_map);

    // Path cache for robots (경로 재사용)
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
            : actions(acts), coordinates(coords), path_cost(pc), task_cost(tc), task_coord(tcrd) {}
    };
    std::map<int, std::queue<TaskPathInfo>> robot_path_cache;
    void updatePathCache(int robot_id,
                        const std::vector<std::shared_ptr<TASK>>& active_tasks,
                        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
                        const std::vector<std::vector<OBJECT>>& known_object_map,
                        const std::vector<std::shared_ptr<ROBOT>>& robots);
    void clearPathCache(int robot_id);
    bool isPathCacheValid(int robot_id, const Coord& current_pos) const;

public:
    static inline int calculate_move_cost(const Coord& c1, const Coord& c2, ROBOT::TYPE r_type,
        const std::vector<std::vector<std::vector<int>>>& cost_map)
    {
        if (c1 == c2) return 0;
        int type_idx = static_cast<int>(r_type);
        int cost1 = cost_map[c1.x][c1.y][type_idx];
        int cost2 = cost_map[c2.x][c2.y][type_idx];
        if (cost1 == INFINITE || cost2 == INFINITE) {
            return INFINITE;
        }
        return (cost1 + cost2) / 2;
    }

    void print_task_total_costs_table(const std::vector<std::shared_ptr<ROBOT>>& all_robots,
        const std::vector<std::shared_ptr<TASK>>& all_tasks) const;
};

#endif // SCHEDULER_H_
