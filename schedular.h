#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"

// ... 기존 코드 ...
#include <map>
#include <set>

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

    ROBOT::ACTION get_next_action_toward(const Coord& start, const Coord& goal, const vector<vector<OBJECT>>& known_object_map);

private:
    std::map<int, Coord> drone_targets;
    std::map<int, bool> drone_reverse;
    std::map<int, bool> drone_map_fully_revealed; // 드론별 맵 완전 탐색 여부
    std::map<int, int> drone_last_task_id;        // 드론별 마지막으로 찾은 task id
    int last_tick = 0;
    int map_size = 0;
    int tick = 0;
    std::set<Coord> revealed_coords; // 전체 밝혀진 좌표
    void update_drone_target(const shared_ptr<ROBOT>& drone, int map_size, const vector<vector<OBJECT>>& known_object_map);
    bool is_map_fully_revealed(const vector<vector<OBJECT>>& known_object_map);
};


#endif /* SCHEDULER_H_ */
