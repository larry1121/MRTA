#include "schedular.h"
#include <cstdlib> // rand()

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    // 랜덤 스케줄러는 매 tick 아무것도 하지 않음
}

bool Scheduler::on_task_reached(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots,
                                const ROBOT &robot,
                                const TASK &task)
{
    // DRONE은 task를 수행할 수 없으므로 예외 처리
    if (robot.type == ROBOT::TYPE::DRONE) {
        return false;
    }

    // 그냥 무조건 task를 수행하도록 한다 (나중에 에너지 체크 넣을 예정)
    return true;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    // 0: UP, 1: DOWN, 2: LEFT, 3: RIGHT, 4: HOLD 중 랜덤 선택
    int dir = rand() % 5;
    return static_cast<ROBOT::ACTION>(dir);
}
