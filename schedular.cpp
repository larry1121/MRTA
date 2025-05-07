#include "schedular.h"
#include <cstdlib> // rand()
#include <cmath>   // abs
#include <limits>  // numeric_limits

// 두 좌표 사이의 맨해튼 거리 계산
int Scheduler::calculate_distance(const Coord& from, const Coord& to) const {
    return abs(from.x - to.x) + abs(from.y - to.y);
}

// 로봇이 작업을 완료할 수 있는지 확인
bool Scheduler::can_robot_complete_task(const ROBOT& robot, const TASK& task, 
                                        const vector<vector<vector<int>>>& known_cost_map) const {
    // 드론은 작업을 수행할 수 없음
    if (robot.type == ROBOT::TYPE::DRONE) {
        return false;
    }
    
    // 작업 비용 확인
    int task_cost = task.get_cost(robot.type);
    if (task_cost >= numeric_limits<int>::max()) {
        return false;  // 무한대 비용은 수행 불가
    }
    
    // 현재 에너지로 작업을 완료할 수 있는지 확인
    if (robot.get_energy() < task_cost) {
        return false;
    }
    
    return true;
}

// 로봇에게 가장 가까운 작업 찾기
shared_ptr<TASK> Scheduler::find_nearest_task(const ROBOT& robot, 
                                             const vector<shared_ptr<TASK>>& active_tasks,
                                             const vector<vector<vector<int>>>& known_cost_map) const {
    shared_ptr<TASK> nearest_task = nullptr;
    int min_distance = numeric_limits<int>::max();
    
    for (const auto& task : active_tasks) {
        // 이미 다른 로봇에게 할당된 작업은 건너뜀
        if (task->get_assigned_robot_id() != -1 && task->get_assigned_robot_id() != robot.id) {
            continue;
        }
        
        // 완료된 작업은 건너뜀
        if (task->is_done()) {
            continue;
        }
        
        // 로봇이 수행할 수 없는 작업은 건너뜀
        if (!can_robot_complete_task(robot, *task, known_cost_map)) {
            continue;
        }
        
        int distance = calculate_distance(robot.get_coord(), task->coord);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_task = task;
        }
    }
    
    return nearest_task;
}

// 로봇이 목표 지점으로 이동하기 위한 방향 결정
ROBOT::ACTION Scheduler::decide_move_direction(const ROBOT& robot, const Coord& target_coord,
                                              const vector<vector<OBJECT>>& known_object_map,
                                              const vector<vector<vector<int>>>& known_cost_map) const {
    // 현재 위치와 목표 위치가 같으면 대기
    if (robot.get_coord().x == target_coord.x && robot.get_coord().y == target_coord.y) {
        return ROBOT::ACTION::HOLD;
    }
    
    // x축 이동 우선
    if (robot.get_coord().x < target_coord.x) {
        return ROBOT::ACTION::RIGHT;
    } else if (robot.get_coord().x > target_coord.x) {
        return ROBOT::ACTION::LEFT;
    }
    
    // x축이 같다면 y축 이동
    if (robot.get_coord().y < target_coord.y) {
        return ROBOT::ACTION::UP;
    } else if (robot.get_coord().y > target_coord.y) {
        return ROBOT::ACTION::DOWN;
    }
    
    // 기본 대기
    return ROBOT::ACTION::HOLD;
}

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    // 각 로봇에 대해 가장 가까운 작업 할당
    for (const auto& robot_ptr : robots) {
        // 이미 작업 중이거나 이동 중인 로봇은 건너뜀
        if (robot_ptr->get_status() == ROBOT::STATUS::WORKING || 
            robot_ptr->get_status() == ROBOT::STATUS::MOVING) {
            continue;
        }
        
        // 에너지가 없는 로봇은 건너뜀
        if (robot_ptr->get_energy() <= 0) {
            continue;
        }
        
        // 이미 작업이 할당된 로봇은 건너뜀
        if (robot_to_task.find(robot_ptr->id) != robot_to_task.end()) {
            // 할당된 작업이 완료되었거나 더 이상 존재하지 않으면 할당 취소
            bool task_exists = false;
            for (const auto& task : active_tasks) {
                if (task->id == robot_to_task[robot_ptr->id] && !task->is_done()) {
                    task_exists = true;
                    break;
                }
            }
            
            if (!task_exists) {
                int task_id = robot_to_task[robot_ptr->id];
                robot_to_task.erase(robot_ptr->id);
                task_to_robot.erase(task_id);
            } else {
                continue;  // 유효한 작업이 이미 할당되어 있으면 건너뜀
            }
        }
        
        // 가장 가까운 작업 찾기
        shared_ptr<TASK> nearest_task = find_nearest_task(*robot_ptr, active_tasks, known_cost_map);
        
        // 적합한 작업이 있으면 할당
        if (nearest_task) {
            robot_to_task[robot_ptr->id] = nearest_task->id;
            task_to_robot[nearest_task->id] = robot_ptr->id;
        }
    }
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
    // DRONE은 task를 수행할 수 없음
    if (robot.type == ROBOT::TYPE::DRONE) {
        return false;
    }
    
    // 작업을 완료할 수 있는지 확인
    if (!can_robot_complete_task(robot, task, known_cost_map)) {
        return false;
    }
    
    // 에너지 체크: 작업 수행에 필요한 에너지가 현재 로봇 에너지보다 많으면 실행하지 않음
    int task_cost = task.get_cost(robot.type);
    if (robot.get_energy() < task_cost) {
        return false;
    }
    
    // 작업 수행 시작
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
    // 에너지가 없는 로봇은 대기
    if (robot.get_energy() <= 0) {
        return ROBOT::ACTION::HOLD;
    }
    
    // 로봇에게 할당된 작업이 있으면 해당 작업 위치로 이동
    if (robot_to_task.find(robot.id) != robot_to_task.end()) {
        int task_id = robot_to_task[robot.id];
        
        // 할당된 작업 찾기
        for (const auto& task : active_tasks) {
            if (task->id == task_id && !task->is_done()) {
                // 작업 위치로 이동하기 위한 방향 결정
                return decide_move_direction(robot, task->coord, known_object_map, known_cost_map);
            }
        }
        
        // 작업을 찾지 못하면 할당 취소
        robot_to_task.erase(robot.id);
        task_to_robot.erase(task_id);
    }
    
    // 할당된 작업이 없으면 가까운 작업 찾기
    shared_ptr<TASK> nearest_task = find_nearest_task(robot, active_tasks, known_cost_map);
    
    if (nearest_task) {
        // 새 작업 할당
        robot_to_task[robot.id] = nearest_task->id;
        task_to_robot[nearest_task->id] = robot.id;
        
        // 작업 위치로 이동
        return decide_move_direction(robot, nearest_task->coord, known_object_map, known_cost_map);
    }
    
    // 드론은 탐색 범위가 넓으므로 맵 탐색에 활용
    if (robot.type == ROBOT::TYPE::DRONE) {
        // 무작위 방향으로 이동
        int dir = rand() % 4;  // UP, DOWN, LEFT, RIGHT 중 하나
        return static_cast<ROBOT::ACTION>(dir);
    }
    
    // 기본적으로 대기
    return ROBOT::ACTION::HOLD;
}
