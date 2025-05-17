#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

class Scheduler
{
public:
    Scheduler();
    void on_info_updated(const set<Coord>&, const set<Coord>&,
        const std::vector<std::vector<std::vector<int>>>&,
        const std::vector<std::vector<OBJECT>>&,
        const std::vector<std::shared_ptr<TASK>>&,
        const std::vector<std::shared_ptr<ROBOT>>&);

    bool on_task_reached(const set<Coord>&, const set<Coord>&,
        const std::vector<std::vector<std::vector<int>>>&,
        const std::vector<std::vector<OBJECT>>&,
        const std::vector<std::shared_ptr<TASK>>&,
        const std::vector<std::shared_ptr<ROBOT>>&,
        const ROBOT&, const TASK&);

    ROBOT::ACTION idle_action(const set<Coord>&, const set<Coord>&,
        const std::vector<std::vector<std::vector<int>>>&,
        const std::vector<std::vector<OBJECT>>&,
        const std::vector<std::shared_ptr<TASK>>&,
        const std::vector<std::shared_ptr<ROBOT>>&,
        const ROBOT&);

private:
    int map_width_ = 0, map_height_ = 0;
    std::map<int, std::vector<Coord>> drone_path_;
    std::map<int, size_t> path_idx_;
    std::map<int, Coord> drone_target_;

    std::vector<Coord> find_targets(const std::vector<std::vector<OBJECT>>&,
        const std::vector<std::shared_ptr<TASK>>&);
    std::vector<Coord> find_frontiers(const std::vector<std::vector<OBJECT>>&);
    std::vector<Coord> find_path_astar(const Coord&, const Coord&, const std::vector<std::vector<OBJECT>>&, const ROBOT&);
    ROBOT::ACTION action_to(const Coord&, const Coord&);
};

#endif
