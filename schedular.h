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
