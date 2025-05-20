#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h" // simulator.h 포함 (ROBOT, TASK, Coord, OBJECT, ACTION, TYPE 등 정의)
#include <vector>
#include <deque>
#include <unordered_map>
#include <set>
#include <string> // printf 대신 std::string 관련 로깅 시 필요할 수 있음 (현재는 미사용)
#include <memory> // For std::shared_ptr

// 전방 선언 (simulator.h에 이미 정의되어 있다면 중복될 수 있으나, 안전을 위해 명시)
// class ROBOT;
// class TASK;
// struct Coord;
// enum class OBJECT : int;
// namespace ROBOT { // ROBOT 클래스 내부에 enum이 정의되어 있으므로, 네임스페이스 방식은 부적절
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