<<<<<<< HEAD
#include "schedular.h"
#include <cstdlib> // rand()
#include <cmath>   // abs
#include <limits>  // numeric_limits
#include <random>  // 난수 생성

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

// 로봇에게 가장 적합한 작업 찾기 (효율성 고려)
shared_ptr<TASK> Scheduler::find_nearest_task(const ROBOT& robot, 
                                             const vector<shared_ptr<TASK>>& active_tasks,
                                             const vector<vector<vector<int>>>& known_cost_map) const {
    shared_ptr<TASK> best_task = nullptr;
    double best_efficiency = numeric_limits<double>::max();
    
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
        int task_cost = task->get_cost(robot.type);
        
        // 거리와 작업 비용을 고려한 효율성 계산
        // 거리가 가까울수록, 에너지 소모가 적을수록 효율적
        double efficiency = (distance * 0.7) + (task_cost * 0.3);
        
        if (efficiency < best_efficiency) {
            best_efficiency = efficiency;
            best_task = task;
        }
    }
    
    return best_task;
}

// 탐색 목표 좌표 업데이트
void Scheduler::update_exploration_targets(const vector<vector<OBJECT>>& known_object_map) {
    // 맵 크기 확인
    int width = known_object_map.size();
    if (width == 0) return;
    int height = known_object_map[0].size();
    
    // 기존 탐색 목표 우선순위 업데이트
    for (auto& target : exploration_targets) {
        // 시간에 따른 우선순위 증가 (오래 방문하지 않은 곳 우선)
        target.priority = (current_time - target.last_visited_time) * 0.1;
    }
    
    // 맵 전체에서 미탐색 영역이나 미발견 작업 위치를 탐색 목표로 추가
    int grid_size = 5; // 탐색 그리드 크기
    for (int x = 0; x < width; x += grid_size) {
        for (int y = 0; y < height; y += grid_size) {
            Coord center = {x, y};
            
            // 이미 목표로 설정된 위치는 건너뜀
            bool already_target = false;
            for (const auto& target : exploration_targets) {
                if (target.coord.x == center.x && target.coord.y == center.y) {
                    already_target = true;
                    break;
                }
            }
            
            if (!already_target) {
                // 벽이 아닌 경우에만 추가
                if (x < width && y < height && known_object_map[x][y] != OBJECT::WALL) {
                    ExplorationTarget new_target = {
                        center,
                        current_time,
                        1.0  // 기본 우선순위
                    };
                    exploration_targets.push_back(new_target);
                }
            }
        }
    }
    
    // 우선순위에 따라 정렬
    sort(exploration_targets.begin(), exploration_targets.end(),
         [](const ExplorationTarget& a, const ExplorationTarget& b) {
             return a.priority > b.priority;
         });
}

// 로봇에게 탐색 목표 할당
void Scheduler::assign_exploration_target(const ROBOT& robot, 
                                          const vector<vector<OBJECT>>& known_object_map,
                                          const vector<vector<vector<int>>>& known_cost_map) {
    // 이미 목표가 있으면 그대로 유지
    if (robot_exploration_targets.find(robot.id) != robot_exploration_targets.end()) {
        Coord target = robot_exploration_targets[robot.id];
        // 목표 근처에 도달했으면 목표 삭제하고 새로 할당
        if (calculate_distance(robot.get_coord(), target) <= 2) {
            robot_exploration_targets.erase(robot.id);
        } else {
            return;  // 목표가 있고 아직 도달하지 않았으면 그대로 유지
        }
    }
    
    // 새로운 탐색 목표 찾기
    if (exploration_targets.empty()) {
        update_exploration_targets(known_object_map);
    }
    
    if (!exploration_targets.empty()) {
        // 드론은 먼 거리 탐색, 다른 로봇은 가까운 거리 탐색
        if (robot.type == ROBOT::TYPE::DRONE) {
            // 드론을 위한 랜덤 목표 선택 (탐색 다양성 위해)
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> dis(0, min(10, (int)exploration_targets.size() - 1));
            int index = dis(gen);
            robot_exploration_targets[robot.id] = exploration_targets[index].coord;
            
            // 방문 시간 업데이트
            exploration_targets[index].last_visited_time = current_time;
        } else {
            // 휠/캐터필러는 가장 가까운 탐색 목표 선택
            int best_distance = numeric_limits<int>::max();
            int best_index = -1;
            
            for (int i = 0; i < exploration_targets.size(); i++) {
                int distance = calculate_distance(robot.get_coord(), exploration_targets[i].coord);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_index = i;
                }
            }
            
            if (best_index != -1) {
                robot_exploration_targets[robot.id] = exploration_targets[best_index].coord;
                exploration_targets[best_index].last_visited_time = current_time;
            }
        }
    } else {
        // 탐색 목표가 없으면 랜덤 이동
        int width = known_object_map.size();
        int height = (width > 0) ? known_object_map[0].size() : 0;
        
        // 유효한 좌표 생성
        Coord random_coord;
        for (int attempts = 0; attempts < 10; attempts++) {
            random_coord.x = rand() % width;
            random_coord.y = rand() % height;
            
            // 벽이 아니면 목표로 설정
            if (known_object_map[random_coord.x][random_coord.y] != OBJECT::WALL) {
                robot_exploration_targets[robot.id] = random_coord;
                break;
            }
        }
    }
}

// 로봇이 목표 지점으로 이동하기 위한 방향 결정
ROBOT::ACTION Scheduler::decide_move_direction(const ROBOT& robot, const Coord& target_coord,
                                              const vector<vector<OBJECT>>& known_object_map,
                                              const vector<vector<vector<int>>>& known_cost_map) const {
    // 현재 위치와 목표 위치가 같으면 대기
    if (robot.get_coord().x == target_coord.x && robot.get_coord().y == target_coord.y) {
        return ROBOT::ACTION::HOLD;
    }
    
    int x = robot.get_coord().x;
    int y = robot.get_coord().y;
    int target_x = target_coord.x;
    int target_y = target_coord.y;
    
    // 맵 크기 확인
    int max_x = known_object_map.size() - 1;
    int max_y = known_object_map[0].size() - 1;
    
    // 벽을 확인하고 가능한 이동 방향을 저장
    bool can_move_right = (x < max_x) && (known_object_map[x+1][y] != OBJECT::WALL);
    bool can_move_left = (x > 0) && (known_object_map[x-1][y] != OBJECT::WALL);
    bool can_move_up = (y < max_y) && (known_object_map[x][y+1] != OBJECT::WALL);
    bool can_move_down = (y > 0) && (known_object_map[x][y-1] != OBJECT::WALL);
    
    // 목표 방향 계산
    int dx = target_x - x;
    int dy = target_y - y;
    
    // 맨해튼 거리로 각 이동 방향의 효율성 계산
    int dist_right = (can_move_right) ? abs(target_x - (x+1)) + abs(target_y - y) : INT_MAX;
    int dist_left = (can_move_left) ? abs(target_x - (x-1)) + abs(target_y - y) : INT_MAX;
    int dist_up = (can_move_up) ? abs(target_x - x) + abs(target_y - (y+1)) : INT_MAX;
    int dist_down = (can_move_down) ? abs(target_x - x) + abs(target_y - (y-1)) : INT_MAX;
    
    // 이동 비용도 고려 (로봇 타입에 따라 다름)
    if (can_move_right && x+1 <= max_x) {
        int cost = (known_cost_map[x+1][y][static_cast<int>(robot.type)] > 0) ? 
                   known_cost_map[x+1][y][static_cast<int>(robot.type)] : 50;
        dist_right += cost * 0.1;  // 비용을 가중치를 적용해 거리에 추가
    }
    
    if (can_move_left && x-1 >= 0) {
        int cost = (known_cost_map[x-1][y][static_cast<int>(robot.type)] > 0) ? 
                   known_cost_map[x-1][y][static_cast<int>(robot.type)] : 50;
        dist_left += cost * 0.1;
    }
    
    if (can_move_up && y+1 <= max_y) {
        int cost = (known_cost_map[x][y+1][static_cast<int>(robot.type)] > 0) ? 
                   known_cost_map[x][y+1][static_cast<int>(robot.type)] : 50;
        dist_up += cost * 0.1;
    }
    
    if (can_move_down && y-1 >= 0) {
        int cost = (known_cost_map[x][y-1][static_cast<int>(robot.type)] > 0) ? 
                   known_cost_map[x][y-1][static_cast<int>(robot.type)] : 50;
        dist_down += cost * 0.1;
    }
    
    // 가장 효율적인 이동 방향 선택
    int min_dist = min({dist_right, dist_left, dist_up, dist_down});
    
    if (min_dist == INT_MAX) {
        // 모든 방향이 막혔으면 대기
        return ROBOT::ACTION::HOLD;
    }
    
    // 같은 거리의 경우 x축, y축 중 목표에 더 가까워지는 방향 우선
    if (abs(dx) > abs(dy)) {
        // x 방향이 더 멀면 x 먼저 이동
        if (dx > 0 && can_move_right) return ROBOT::ACTION::RIGHT;
        if (dx < 0 && can_move_left) return ROBOT::ACTION::LEFT;
        if (dy > 0 && can_move_up) return ROBOT::ACTION::UP;
        if (dy < 0 && can_move_down) return ROBOT::ACTION::DOWN;
    } else {
        // y 방향이 더 멀면 y 먼저 이동
        if (dy > 0 && can_move_up) return ROBOT::ACTION::UP;
        if (dy < 0 && can_move_down) return ROBOT::ACTION::DOWN;
        if (dx > 0 && can_move_right) return ROBOT::ACTION::RIGHT;
        if (dx < 0 && can_move_left) return ROBOT::ACTION::LEFT;
    }
    
    // 최소 거리 기준 선택
    if (min_dist == dist_right && can_move_right) {
        return ROBOT::ACTION::RIGHT;
    } else if (min_dist == dist_left && can_move_left) {
        return ROBOT::ACTION::LEFT;
    } else if (min_dist == dist_up && can_move_up) {
        return ROBOT::ACTION::UP;
    } else if (min_dist == dist_down && can_move_down) {
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
    // 매 호출마다 시간 증가
    current_time++;
    
    // 방문한 좌표 추적
    for (const auto& coord : observed_coords) {
        visited_coords.insert(coord_to_hash(coord));
    }
    
    // 탐색 목표 업데이트 (주기적으로)
    if (current_time % 10 == 0) {
        update_exploration_targets(known_object_map);
    }
    
    // 모든 로봇-작업 쌍 후보를 저장할 벡터
    vector<RobotTaskPair> candidates;
    
    // 기존 할당 상태 확인 및 정리
    for (auto it = robot_to_task.begin(); it != robot_to_task.end();) {
        int robot_id = it->first;
        int task_id = it->second;
        
        // 로봇 찾기
        shared_ptr<ROBOT> robot_ptr = nullptr;
        for (const auto& r : robots) {
            if (r->id == robot_id) {
                robot_ptr = r;
                break;
            }
        }
        
        // 작업 찾기
        shared_ptr<TASK> task_ptr = nullptr;
        for (const auto& t : active_tasks) {
            if (t->id == task_id) {
                task_ptr = t;
                break;
            }
        }
        
        // 로봇이 없거나, 에너지가 없거나, 작업 중이거나, 작업이 없거나, 작업이 완료된 경우 할당 취소
        if (!robot_ptr || robot_ptr->get_energy() <= 0 || 
            robot_ptr->get_status() == ROBOT::STATUS::WORKING || 
            robot_ptr->get_status() == ROBOT::STATUS::EXHAUSTED ||
            !task_ptr || task_ptr->is_done()) {
            
            // 작업에 할당된 로봇 정보도 삭제
            task_to_robot.erase(task_id);
            it = robot_to_task.erase(it);
        } else {
            ++it;
        }
    }
    
    // 새로운 할당을 위한 로봇-작업 쌍 생성
    for (const auto& robot_ptr : robots) {
        // 에너지가 없거나 작업 중이거나 이미 작업이 할당된 로봇은 건너뜀
        if (robot_ptr->get_energy() <= 0 || 
            robot_ptr->get_status() == ROBOT::STATUS::WORKING || 
            robot_ptr->get_status() == ROBOT::STATUS::MOVING ||
            robot_ptr->get_status() == ROBOT::STATUS::EXHAUSTED ||
            robot_to_task.find(robot_ptr->id) != robot_to_task.end()) {
            continue;
        }
        
        // 드론은 작업을 수행할 수 없으므로 탐색 목표만 할당
        if (robot_ptr->type == ROBOT::TYPE::DRONE) {
            assign_exploration_target(*robot_ptr, known_object_map, known_cost_map);
            continue;
        }
        
        // 각 작업에 대한 평가
        for (const auto& task_ptr : active_tasks) {
            // 이미 완료되었거나 다른 로봇에게 할당된 작업은 건너뜀
            if (task_ptr->is_done() || 
                (task_to_robot.find(task_ptr->id) != task_to_robot.end() && 
                 task_to_robot[task_ptr->id] != robot_ptr->id)) {
                continue;
            }
            
            // 작업을 수행할 수 있는지 확인
            if (!can_robot_complete_task(*robot_ptr, *task_ptr, known_cost_map)) {
                continue;
            }
            
            // 로봇-작업 간 거리 계산
            int distance = calculate_distance(robot_ptr->get_coord(), task_ptr->coord);
            int energy_cost = task_ptr->get_cost(robot_ptr->type);
            
            // 효율성 계산 (거리 50%, 에너지 비용 30%, 로봇 타입 20%)
            double efficiency_score;
            
            // 로봇 타입에 따른 가중치 (캐터필러는 높은 비용 작업에 적합, 휠은 낮은 비용 작업에 적합)
            if (robot_ptr->type == ROBOT::TYPE::CATERPILLAR) {
                // 캐터필러는 높은 비용 작업에 효율적
                efficiency_score = (distance * 0.5) + (energy_cost * 0.3) - (energy_cost * 0.2);
            } else {
                // 휠은 낮은 비용 작업에 효율적
                efficiency_score = (distance * 0.5) + (energy_cost * 0.3) + (energy_cost * 0.2);
            }
            
            // 후보에 추가
            candidates.push_back({robot_ptr->id, task_ptr->id, distance, energy_cost, efficiency_score});
        }
    }
    
    // 효율성 점수로 정렬 (낮을수록 좋음)
    sort(candidates.begin(), candidates.end(), [](const RobotTaskPair& a, const RobotTaskPair& b) {
        return a.efficiency < b.efficiency;
    });
    
    // 할당 - 가장 효율적인 로봇-작업 쌍부터 할당
    for (const auto& candidate : candidates) {
        // 이미 다른 작업이 할당된 로봇이면 건너뜀
        if (robot_to_task.find(candidate.robot_id) != robot_to_task.end()) {
            continue;
        }
        
        // 이미 다른 로봇에게 할당된 작업이면 건너뜀
        if (task_to_robot.find(candidate.task_id) != task_to_robot.end()) {
            continue;
        }
        
        // 로봇에게 작업 할당
        robot_to_task[candidate.robot_id] = candidate.task_id;
        task_to_robot[candidate.task_id] = candidate.robot_id;
    }
    
    // 작업이 할당되지 않은 로봇에게 탐색 목표 할당
    for (const auto& robot_ptr : robots) {
        if (robot_ptr->get_energy() <= 0 || 
            robot_ptr->get_status() == ROBOT::STATUS::WORKING || 
            robot_ptr->get_status() == ROBOT::STATUS::MOVING ||
            robot_ptr->get_status() == ROBOT::STATUS::EXHAUSTED) {
            continue;
        }
        
        // 작업이 할당되지 않은 로봇에게 탐색 목표 할당
        if (robot_to_task.find(robot_ptr->id) == robot_to_task.end()) {
            assign_exploration_target(*robot_ptr, known_object_map, known_cost_map);
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
    // 작업을 완료할 수 있는지 검사 (이미 에너지 검증 포함)
    if (!can_robot_complete_task(robot, task, known_cost_map)) {
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
    
    // 이동 중이거나 작업 중인 로봇은 대기
    if (robot.get_status() == ROBOT::STATUS::MOVING || 
        robot.get_status() == ROBOT::STATUS::WORKING) {
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
        task_to_robot.erase(task_id);
        robot_to_task.erase(robot.id);
    }
    
    // 탐색 목표가 있으면 해당 위치로 이동
    if (robot_exploration_targets.find(robot.id) != robot_exploration_targets.end()) {
        Coord target = robot_exploration_targets[robot.id];
        
        // 목표 근처에 도달했으면 목표 제거
        if (calculate_distance(robot.get_coord(), target) <= 2) {
            robot_exploration_targets.erase(robot.id);
            // 새 목표 할당
            assign_exploration_target(robot, known_object_map, known_cost_map);
            if (robot_exploration_targets.find(robot.id) != robot_exploration_targets.end()) {
                target = robot_exploration_targets[robot.id];
            }
        }
        
        return decide_move_direction(robot, target, known_object_map, known_cost_map);
    }
    
    // 탐색 목표도 없으면 새 목표 할당
    assign_exploration_target(robot, known_object_map, known_cost_map);
    if (robot_exploration_targets.find(robot.id) != robot_exploration_targets.end()) {
        return decide_move_direction(robot, robot_exploration_targets[robot.id], known_object_map, known_cost_map);
    }
    
    // 어떤 목표도 없으면 효율적인 랜덤 이동
    vector<ROBOT::ACTION> possible_actions;
    int x = robot.get_coord().x;
    int y = robot.get_coord().y;
    int width = known_object_map.size();
    int height = (width > 0) ? known_object_map[0].size() : 0;
    
    // 각 방향에 대해 벽이 없으면 이동 가능 리스트에 추가
    if (x + 1 < width && known_object_map[x + 1][y] != OBJECT::WALL)
        possible_actions.push_back(ROBOT::ACTION::RIGHT);
    if (x - 1 >= 0 && known_object_map[x - 1][y] != OBJECT::WALL)
        possible_actions.push_back(ROBOT::ACTION::LEFT);
    if (y + 1 < height && known_object_map[x][y + 1] != OBJECT::WALL)
        possible_actions.push_back(ROBOT::ACTION::UP);
    if (y - 1 >= 0 && known_object_map[x][y - 1] != OBJECT::WALL)
        possible_actions.push_back(ROBOT::ACTION::DOWN);
    
    // 가능한 이동이 있으면 랜덤으로 선택
    if (!possible_actions.empty()) {
        return possible_actions[rand() % possible_actions.size()];
    }
    
    // 기본적으로 대기
    return ROBOT::ACTION::HOLD;
}
=======
﻿/*
 *  schedular.cpp ― MRTA Scheduler
 */
#include "schedular.h"
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>

 /* =================================================
  *  보조
  * ================================================= */
bool Scheduler::is_passable(const Coord& c,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    ROBOT::TYPE type)
{
    const auto& layer = cost_map[static_cast<int>(type)];
    int N = (int)layer.size(), M = (int)layer[0].size();
    if (c.x < 0 || c.y < 0 || c.x >= N || c.y >= M) return false;
    return layer[c.x][c.y] != std::numeric_limits<int>::max();
}
std::queue<Coord> Scheduler::vec2q(const std::vector<Coord>& v)
{
    std::queue<Coord> q;
    for (auto& c : v) q.push(c);
    return q;
}

/* ---------------- 지도 밖 / 벽 / 차단 ---------------- */
bool Scheduler::passable(const Coord& c,
    const std::vector<std::vector<OBJECT>>& known_obj,
    ROBOT::TYPE type) const
{
    if (blocked_cells.count(c)) return false;

    int N = (int)known_obj.size();
    int M = (int)known_obj[0].size();
    if (c.x < 0 || c.y < 0 || c.x >= N || c.y >= M) return false;

    // 드론은 벽 통과 가능
    if (type == ROBOT::TYPE::DRONE)
        return true;

    return known_obj[c.x][c.y] != OBJECT::WALL;
}


/* =================================================
 *  A* (4-dir, 가중치 + blocked_cells 반영)
 * ================================================= */
std::queue<Coord> Scheduler::astar_path(const Coord& start,
    const Coord& goal,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    ROBOT::TYPE type)
{
    std::queue<Coord> empty;
    if (start == goal) return empty;

    const auto& layer = cost_map[static_cast<int>(type)];
    int N = (int)layer.size(), M = (int)layer[0].size();
    const int dx[4] = { 0,0,-1,1 }, dy[4] = { 1,-1,0,0 };
    constexpr int INF = std::numeric_limits<int>::max();

    struct Node {
        Coord c; int g; int f;
        bool operator>(const Node& o) const { return f > o.f; }
    };
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::unordered_map<Coord, int, CoordHash, CoordEq> gscore;
    std::unordered_map<Coord, Coord, CoordHash, CoordEq> parent;
    auto H = [&](int x, int y) { return std::abs(x - goal.x) + std::abs(y - goal.y); };

    gscore[start] = 0;
    pq.push({ start,0,H(start.x,start.y) });

    while (!pq.empty()) {
        Node cur = pq.top(); pq.pop();
        if (cur.c == goal) {
            std::vector<Coord> rev;
            for (Coord p = goal; p != start; p = parent[p]) rev.push_back(p);
            std::reverse(rev.begin(), rev.end());
            return vec2q(rev);
        }
        for (int d = 0; d < 4; ++d) {
            int nx = cur.c.x + dx[d], ny = cur.c.y + dy[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= M) continue;
            if (layer[nx][ny] == INF) continue;
            Coord nxt(nx, ny);
            if (blocked_cells.count(nxt)) continue;

            int tg = cur.g + layer[nx][ny];
            auto it = gscore.find(nxt);
            if (it == gscore.end() || tg < it->second) {
                gscore[nxt] = tg; parent[nxt] = cur.c;
                pq.push({ nxt, tg, tg + H(nx, ny) });
            }
        }
    }
    return empty;  /* goal unreachable */
}

/* =================================================
 *  BFS ― 가장 가까운 UNKNOWN (blocked 반영)
 * ================================================= */
std::queue<Coord> Scheduler::bfs_to_unknown(const Coord& start,
    const std::vector<std::vector<OBJECT>>& known_obj)
{
    std::queue<Coord> empty;

    int N = (int)known_obj.size(), M = (int)known_obj[0].size();
    const int dx[4] = { 0,0,-1,1 }, dy[4] = { 1,-1,0,0 };

    std::map<Coord, Coord, CoordCompare> parent;
    std::set<Coord, CoordCompare>        vis;
    std::queue<Coord>                    q;

    if (!passable(start, known_obj, ROBOT::TYPE::DRONE)) return empty;
    q.push(start); vis.insert(start);

    while (!q.empty()) {
        Coord cur = q.front(); q.pop();
        if (known_obj[cur.x][cur.y] == OBJECT::UNKNOWN) {
            std::vector<Coord> rev;
            for (Coord p = cur; p != start; p = parent[p]) rev.push_back(p);
            std::reverse(rev.begin(), rev.end());
            return vec2q(rev);
        }
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            Coord nxt(nx, ny);
            if (!passable(nxt, known_obj, ROBOT::TYPE::DRONE)) continue;
            if (vis.insert(nxt).second) { parent[nxt] = cur; q.push(nxt); }
        }
    }
    return empty;
}

/* =================================================
 *  정보 갱신 콜백
 * ================================================= */
void Scheduler::on_info_updated(const std::set<Coord>& /*obs*/,
    const std::set<Coord>& /*upd*/,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    const std::vector<std::vector<OBJECT>>& known_obj,
    const std::vector<std::shared_ptr<TASK>>& active_tasks,
    const std::vector<std::shared_ptr<ROBOT>>& robots)
{
    /* 완료된 작업 클리어 */
    for (auto it = assigned_tasks.begin(); it != assigned_tasks.end();) {
        int rid = it->first, tid = it->second;
        auto tp = std::find_if(active_tasks.begin(), active_tasks.end(),
            [&](auto& t) { return t->id == tid; });
        if (tp == active_tasks.end() || (*tp)->is_done()) {
            robot_paths[rid] = {};
            it = assigned_tasks.erase(it);
        }
        else ++it;
    }

    /* IDLE 로봇 스케줄 */
    for (auto& r : robots) {
        if (r->get_status() != ROBOT::STATUS::IDLE) continue;

        /* --- 드론 : 지도 탐색 --- */
        if (r->type == ROBOT::TYPE::DRONE) {
            if (robot_paths[r->id].empty())
                robot_paths[r->id] = bfs_to_unknown(r->get_coord(), known_obj);
            continue;
        }

        /* --- 지상 로봇 : 작업 할당 --- */
        if (assigned_tasks.count(r->id)) continue;

        int best = std::numeric_limits<int>::max();
        std::shared_ptr<TASK> best_task;
        std::queue<Coord>     best_path;

        for (const auto& t : active_tasks) {
            if (t->is_done() || t->get_assigned_robot_id() != -1) continue;

            auto path = astar_path(r->get_coord(), t->coord, cost_map, r->type);
            if (path.empty()) continue;

            int score = (int)path.size() + t->get_cost(r->type);
            if (score < best && r->get_energy() >= score) {
                best = score; best_task = t; best_path = path;
            }
        }
        if (best_task) {
            assigned_tasks[r->id] = best_task->id;
            robot_paths[r->id] = best_path;
        }
    }
}

/* =================================================
 *  작업 수행 가능 여부
 * ================================================= */
bool Scheduler::on_task_reached(const std::set<Coord>&,
    const std::set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>&,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&,
    const ROBOT& robot, const TASK& task)
{
    if (robot.type == ROBOT::TYPE::DRONE) return false;
    return robot.get_energy() >= task.get_cost(robot.type);
}

/* =================================================
 *  IDLE 상태 행동 결정
 * ================================================= */
ROBOT::ACTION Scheduler::idle_action(const std::set<Coord>& observed,
    const std::set<Coord>& updated,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    const std::vector<std::vector<OBJECT>>& known_obj,
    const std::vector<std::shared_ptr<TASK>>& tasks,
    const std::vector<std::shared_ptr<ROBOT>>& robots,
    const ROBOT& robot)
{
    // 경로 비었을 경우 경로 재계산
    if (robot_paths[robot.id].empty()) {
        if (robot.type == ROBOT::TYPE::DRONE) {
            robot_paths[robot.id] = bfs_to_unknown(robot.get_coord(), known_obj);
        }
        else {
            on_info_updated(observed, updated, cost_map, known_obj, tasks, robots);
        }
    }

    // 여전히 경로 없음 → HOLD
    if (robot_paths[robot.id].empty()) return ROBOT::ACTION::HOLD;

    Coord next = robot_paths[robot.id].front();

    // 다음 칸이 통행 불가면: 경로 무효화 후 HOLD (다음 tick에 재계산)
    if (!passable(next, known_obj, robot.type)) {
        blocked_cells.insert(next);
        robot_paths[robot.id] = {};
        return ROBOT::ACTION::HOLD;
    }

    robot_paths[robot.id].pop();  // 다음 칸 유효 → 이동

    int dx = next.x - robot.get_coord().x;
    int dy = next.y - robot.get_coord().y;
    if (std::abs(dx) + std::abs(dy) != 1) return ROBOT::ACTION::HOLD;

    if (dx == 1) return ROBOT::ACTION::DOWN;
    if (dx == -1) return ROBOT::ACTION::UP;
    if (dy == 1) return ROBOT::ACTION::RIGHT;
    if (dy == -1) return ROBOT::ACTION::LEFT;
    return ROBOT::ACTION::HOLD;
}
>>>>>>> parent of a7b030c (RANDOM MOVE 일단 돌아가긴 함)
