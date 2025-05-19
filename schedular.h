#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <queue>
#include <map>
#include <vector>
#include <limits>
#include <memory>
#include <unordered_set>
#include <unordered_map>

using namespace std;

struct CoordCompare {
    bool operator()(const Coord& a, const Coord& b) const {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    }
};

class Scheduler {
private:
    map<int, queue<Coord>> robot_paths;   // robot id → 현재 이동 경로
    std::unordered_map<int, int> robotToTask; // robot id → task id

    queue<Coord> bfs_find_path(const Coord& start, const Coord& goal,
                               const vector<vector<vector<int>>>& known_cost_map,
                               ROBOT::TYPE type);

    queue<Coord> bfs_find_unobserved(const Coord& start,
                                     const set<Coord>& observed_coords,
                                     const vector<vector<OBJECT>>& known_object_map);

public:
    static std::unordered_set<int> lastTaskIds;

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

    void assign_tasks_min_min(
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots
    );
};

#endif /* SCHEDULER_H_ */
