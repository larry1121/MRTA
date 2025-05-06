#ifndef SCHEDULAR_H_
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
