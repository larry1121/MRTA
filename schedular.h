#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <deque>
#include <unordered_map>
#include <vector>
#include <set>

class Scheduler
{
public:
    void on_info_updated(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const std::vector<std::vector<std::vector<int>>> &known_cost_map,
                         const std::vector<std::vector<OBJECT>> &known_object_map,
                         const std::vector<std::shared_ptr<TASK>> &active_tasks,
                         const std::vector<std::shared_ptr<ROBOT>> &robots);

    bool on_task_reached(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const std::vector<std::vector<std::vector<int>>> &known_cost_map,
                         const std::vector<std::vector<OBJECT>> &known_object_map,
                         const std::vector<std::shared_ptr<TASK>> &active_tasks,
                         const std::vector<std::shared_ptr<ROBOT>> &robots,
                         const ROBOT &robot,
                         const TASK &task);

    ROBOT::ACTION idle_action(const set<Coord> &observed_coords,
                              const set<Coord> &updated_coords,
                              const std::vector<std::vector<std::vector<int>>> &known_cost_map,
                              const std::vector<std::vector<OBJECT>> &known_object_map,
                              const std::vector<std::shared_ptr<TASK>> &active_tasks,
                              const std::vector<std::shared_ptr<ROBOT>> &robots,
                              const ROBOT &robot);

    void update_tile_info(const std::vector<std::vector<OBJECT>> &known_object_map);
    bool is_exploration_time() const;
    std::vector<Coord> plan_path(const Coord &start, const Coord &goal, const std::vector<std::vector<std::vector<int>>> &known_cost_map, ROBOT::TYPE type, const std::vector<std::vector<OBJECT>> &known_object_map);

    void set_tile_parameters(int size, int range);
    void set_optimization_parameters(int dist_thresh, double high_weight, double mid_weight,
                                     double cand_thresh, int pause_start, int resume_time);
    void reset_scheduler();

private:
    struct TileInfo
    {
        Coord center;
        int unseen_cells = 0;
    };

    int tile_size = 5;
    int tile_range = 2;
    int map_size = -1;
    int tile_rows = 0, tile_cols = 0;
    std::vector<std::vector<TileInfo>> tiles;
    std::unordered_map<int, std::deque<Coord>> drone_paths;
    std::unordered_map<int, Coord> drone_targets;
    std::set<std::pair<int, int>> assigned_targets;
    int current_time = 0;

    // 최적화 파라미터들
    int distance_threshold = 1;        // 재할당 거리 임계값 (최고 성능 설정)
    double high_priority_weight = 2.0; // 70% 이상 미탐색 가중치
    double mid_priority_weight = 1.5;  // 40% 이상 미탐색 가중치
    double candidate_threshold = 0.3;  // 후보 선택 기준
    int exploration_pause_start = 500; // 탐색 중단 시작 (최고 성능: 12.57개)
    int exploration_resume = 1250;     // 탐색 재개 (최고 성능 설정)

    void init_tiles(const std::vector<std::vector<OBJECT>> &known_object_map);
    static bool coord_equal(const Coord &a, const Coord &b);
    ROBOT::ACTION get_direction(const Coord &from, const Coord &to);
};

#endif // SCHEDULER_H_
