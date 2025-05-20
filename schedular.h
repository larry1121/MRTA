#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h" // simulator.h ���� (ROBOT, TASK, Coord, OBJECT, ACTION, TYPE �� ����)
#include <vector>
#include <deque>
#include <unordered_map>
#include <set>
#include <string> // printf ��� std::string ���� �α� �� �ʿ��� �� ���� (����� �̻��)
#include <memory> // For std::shared_ptr

// ���� ���� (simulator.h�� �̹� ���ǵǾ� �ִٸ� �ߺ��� �� ������, ������ ���� ���)
// class ROBOT;
// class TASK;
// struct Coord;
// enum class OBJECT : int;
// namespace ROBOT { // ROBOT Ŭ���� ���ο� enum�� ���ǵǾ� �����Ƿ�, ���ӽ����̽� ����� ������
//     enum class TYPE;
//     enum class ACTION;
// }

class Scheduler {
public:
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

    // Helper methods
    void init_map_size(const std::vector<std::vector<OBJECT>>& known_object_map);
    std::vector<Coord> plan_path(const Coord& start, const Coord& goal,
        const std::vector<std::vector<std::vector<int>>>& known_cost_map,
        ROBOT::TYPE type, // Corrected: ROBOT::TYPE
        const std::vector<std::vector<OBJECT>>& known_object_map);
    static bool coord_equal(const Coord& a, const Coord& b);
    ROBOT::ACTION get_direction(const Coord& from, const Coord& to); // Corrected: ROBOT::ACTION

    int count_observable_unknowns(const Coord& pos,
        const std::vector<std::vector<OBJECT>>& known_map) const;

    long long calculate_path_energy(const std::vector<Coord>& path,
        const Coord& start_pos,
        ROBOT::TYPE robot_type, // Corrected: ROBOT::TYPE
        const std::vector<std::vector<std::vector<int>>>& known_cost_map) const;
};

#endif // SCHEDULER_H_