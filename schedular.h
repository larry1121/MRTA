<<<<<<< HEAD
#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <map>
#include <queue>
#include <algorithm>
#include <unordered_set>

struct RobotTaskPair {
    int robot_id;
    int task_id;
    int distance;
    int energy_cost;
    double efficiency;  // 효율성 점수 (낮을수록 좋음)
};

// 탐색 영역 관리를 위한 구조체
struct ExplorationTarget {
    Coord coord;
    int last_visited_time;
    double priority;
};

class Scheduler
{
private:
    // 로봇-작업 할당 정보
    map<int, int> robot_to_task;  // 로봇 ID -> 작업 ID
    map<int, int> task_to_robot;  // 작업 ID -> 로봇 ID
    
    // 탐색 영역 관리
    vector<ExplorationTarget> exploration_targets;
    map<int, Coord> robot_exploration_targets;  // 로봇 ID -> 탐색 목표 좌표
    int current_time = 0;  // 탐색 우선순위 계산용 시간
    
    // 이미 방문한 좌표 추적
    unordered_set<int> visited_coords;  // x * 1000 + y 형태로 좌표 해시
    
    // 거리 계산 함수
    int calculate_distance(const Coord& from, const Coord& to) const;
    
    // 작업을 수행할 수 있는지 확인하는 함수
    bool can_robot_complete_task(const ROBOT& robot, const TASK& task, 
                               const vector<vector<vector<int>>>& known_cost_map) const;
    
    // 로봇에게 가장 가까운 작업을 찾는 함수
    shared_ptr<TASK> find_nearest_task(const ROBOT& robot, 
                                     const vector<shared_ptr<TASK>>& active_tasks,
                                     const vector<vector<vector<int>>>& known_cost_map) const;
    
    // 로봇이 작업으로 이동하기 위한 방향을 결정하는 함수
    ROBOT::ACTION decide_move_direction(const ROBOT& robot, const Coord& target_coord,
                                      const vector<vector<OBJECT>>& known_object_map,
                                      const vector<vector<vector<int>>>& known_cost_map) const;
    
    // 로봇의 탐색 목표를 설정하는 함수
    void assign_exploration_target(const ROBOT& robot, 
                                  const vector<vector<OBJECT>>& known_object_map,
                                  const vector<vector<vector<int>>>& known_cost_map);
    
    // 탐색 목표 좌표 업데이트
    void update_exploration_targets(const vector<vector<OBJECT>>& known_object_map);
    
    // 좌표 해시 생성
    int coord_to_hash(const Coord& coord) const {
        return coord.x * 1000 + coord.y;
    }

public:
    Scheduler() {}
    
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
};

#endif /* SCHEDULER_H_ */
=======
﻿#ifndef SCHEDULAR_H_
#define SCHEDULAR_H_

#include "simulator.h"

#include <queue>
#include <map>
#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>

/* ───────────────────────────────
 *  Coord 비교 / 해시
 * ─────────────────────────────── */
struct CoordCompare {
    bool operator()(const Coord& a, const Coord& b) const {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    }
};
struct CoordHash {
    std::size_t operator()(const Coord& c) const {
        return (std::size_t(c.x) << 20) ^ std::size_t(c.y & 0xFFFFF);
    }
};
struct CoordEq {
    bool operator()(const Coord& a, const Coord& b) const {
        return a.x == b.x && a.y == b.y;
    }
};

/* ───────────────────────────────
 *  Scheduler
 * ─────────────────────────────── */
class Scheduler
{
private:
    /* 로봇 id → 남은 경로                          */
    std::map<int, std::queue<Coord>> robot_paths;

    /* 로봇 id → 할당된 작업 id                     */
    std::map<int, int>               assigned_tasks;

    /* 과거에 이동을 시도했다가 실패한 셀           */
    std::unordered_set<Coord, CoordHash, CoordEq> blocked_cells;

    /* 직전 step 에 보냈던 “from / to” 좌표 기록    */
    std::unordered_map<int, Coord> last_from;   // key = robot id
    std::unordered_map<int, Coord> last_to;

    /* ---------- 경로 계산 ---------- */
    std::queue<Coord> astar_path(const Coord& start, const Coord& goal,
        const std::vector<std::vector<std::vector<int>>>& cost_map,
        ROBOT::TYPE type);

    std::queue<Coord> bfs_to_unknown(const Coord& start,
        const std::vector<std::vector<OBJECT>>& known_obj);

    /* ---------- 보조 ---------- */
    static std::queue<Coord> vec2q(const std::vector<Coord>& v);

    static bool is_passable(const Coord& c,
        const std::vector<std::vector<std::vector<int>>>& cost_map,
        ROBOT::TYPE type);

    bool passable(const Coord& c,
        const std::vector<std::vector<OBJECT>>& known_obj,
        ROBOT::TYPE type) const;

public:
    /* ---------- 필수 콜백 ---------- */
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
        const ROBOT& robot, const TASK& task);

    ROBOT::ACTION idle_action(const std::set<Coord>& observed_coords,
        const std::set<Coord>& updated_coords,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        const std::vector<std::vector<OBJECT>>& known_object_map,
        const std::vector<std::shared_ptr<TASK>>& active_tasks,
        const std::vector<std::shared_ptr<ROBOT>>& robots,
        const ROBOT& robot);
};

#endif /* SCHEDULAR_H_ */
>>>>>>> parent of a7b030c (RANDOM MOVE 일단 돌아가긴 함)
