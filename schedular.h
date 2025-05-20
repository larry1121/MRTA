#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <deque>
#include <unordered_map>
#include <set>
#include <string>
#include <memory> // For std::shared_ptr

class Scheduler {
public:
    Scheduler() : learned_uniform_drone_cost_(-1) {} // Initialize member

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

private:
    int map_size = -1;
    std::unordered_map<int, std::deque<Coord>> drone_paths;
    std::unordered_map<int, Coord> drone_targets;
    std::set<Coord> cell_reserved;
    int learned_uniform_drone_cost_; // Learned uniform cost for drones on the current map

    void init_scheduler_state(const std::vector<std::vector<OBJECT>>& known_object_map,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map);
    std::vector<Coord> plan_path(const Coord& start, const Coord& goal,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        ROBOT::TYPE type,
        const std::vector<std::vector<OBJECT>>& known_object_map);
    static bool coord_equal(const Coord& a, const Coord& b);
    ROBOT::ACTION get_direction(const Coord& from, const Coord& to);

    int count_observable_unknowns(const Coord& pos,
        const std::vector<std::vector<OBJECT>>& known_map) const;

    long long calculate_path_energy(const std::vector<Coord>& path,
        const Coord& start_pos,
        ROBOT::TYPE robot_type,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map) const;
};

#endif // SCHEDULER_H_