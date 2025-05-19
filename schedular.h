#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <deque>
#include <unordered_map>
#include <vector>

class Scheduler
{
public:
    void on_info_updated(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        const std::vector<std::vector<OBJECT>>& known_object_map,
        const std::vector<std::shared_ptr<TASK>>& active_tasks,
        const std::vector<std::shared_ptr<ROBOT>>& robots);

    bool on_task_reached(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        const std::vector<std::vector<OBJECT>>& known_object_map,
        const std::vector<std::shared_ptr<TASK>>& active_tasks,
        const std::vector<std::shared_ptr<ROBOT>>& robots,
        const ROBOT& robot,
        const TASK& task);

    ROBOT::ACTION idle_action(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        const std::vector<std::vector<OBJECT>>& known_object_map,
        const std::vector<std::shared_ptr<TASK>>& active_tasks,
        const std::vector<std::shared_ptr<ROBOT>>& robots,
        const ROBOT& robot);

private:
    struct TileInfo {
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

    void init_tiles(const std::vector<std::vector<OBJECT>>& known_object_map);
    void update_tile_info(const std::vector<std::vector<OBJECT>>& known_object_map);
    std::vector<Coord> plan_path(const Coord& start, const Coord& goal, const std::vector<std::vector<std::vector<int>>>& known_cost_map, ROBOT::TYPE type, const std::vector<std::vector<OBJECT>>& known_object_map);
    static bool coord_equal(const Coord& a, const Coord& b);
    ROBOT::ACTION get_direction(const Coord& from, const Coord& to);
};

#endif // SCHEDULER_H_
