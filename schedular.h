// schedular.h
#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <limits>

class Scheduler
{
public:
    Scheduler();

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

private:
    int map_width_ = 0, map_height_ = 0;

    // ��к� ���� path
    std::map<int, std::vector<Coord>> drone_path_;
    std::map<int, size_t> path_idx_;

    // �� ��п� �Ҵ�� ����Ƽ��
    std::map<int, Coord> drone_target_frontier_;

    // ����Ƽ�� �ĺ� ���
    std::vector<Coord> frontiers_;
    std::set<Coord> assigned_frontiers_; // ���� �ٸ� ����� Ÿ���� ���� ����Ƽ��

    // �ʼ� ��ƿ
    std::vector<Coord> find_frontiers(const std::vector<std::vector<OBJECT>>& known_object_map);
    std::vector<Coord> find_path_bfs(const Coord& start, const Coord& goal, const std::vector<std::vector<OBJECT>>& known_object_map);
    ROBOT::ACTION action_to(const Coord& from, const Coord& to);
};

#endif
