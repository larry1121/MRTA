// schedular.cpp
#include "schedular.h"
#include <queue>
#include <limits>
#include <cstdlib>
#include <cmath>

// Manhattan Distance
static int manhattan(const Coord& a, const Coord& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// 주변이 UNKNOWN인 EMPTY/TASK cell이 프론티어
std::vector<Coord> Scheduler::find_frontiers(const std::vector<std::vector<OBJECT>>& known_object_map) {
    std::vector<Coord> result;
    int h = known_object_map.size(), w = known_object_map[0].size();
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            if (known_object_map[x][y] == OBJECT::EMPTY || known_object_map[x][y] == OBJECT::TASK) {
                static const int dx[4] = { 0, 0, -1, 1 }, dy[4] = { -1, 1, 0, 0 };
                for (int d = 0; d < 4; ++d) {
                    int nx = x + dx[d], ny = y + dy[d];
                    if (nx >= 0 && ny >= 0 && nx < w && ny < h && known_object_map[nx][ny] == OBJECT::UNKNOWN) {
                        result.push_back({ x, y });
                        break;
                    }
                }
            }
        }
    }
    return result;
}

// BFS로 경로 찾기
std::vector<Coord> Scheduler::find_path_bfs(const Coord& start, const Coord& goal, const std::vector<std::vector<OBJECT>>& known_object_map) {
    if (start == goal) return { start };
    int h = known_object_map.size(), w = known_object_map[0].size();
    std::queue<Coord> q;
    std::map<Coord, Coord> parent;
    std::set<Coord> visited;
    q.push(start);
    visited.insert(start);

    static const int dx[4] = { 0, 0, -1, 1 }, dy[4] = { -1, 1, 0, 0 };
    while (!q.empty()) {
        Coord cur = q.front(); q.pop();
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            Coord next{ nx, ny };
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (known_object_map[nx][ny] == OBJECT::WALL || known_object_map[nx][ny] == OBJECT::UNKNOWN) continue;
            if (visited.count(next)) continue;
            parent[next] = cur;
            if (next == goal) {
                // reconstruct path
                std::vector<Coord> path;
                for (Coord p = goal; p != start; p = parent[p]) path.push_back(p);
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            q.push(next);
            visited.insert(next);
        }
    }
    return {};
}

ROBOT::ACTION Scheduler::action_to(const Coord& from, const Coord& to) {
    if (from.x < to.x) return ROBOT::ACTION::RIGHT;
    if (from.x > to.x) return ROBOT::ACTION::LEFT;
    if (from.y < to.y) return ROBOT::ACTION::UP;    // 시뮬레이터에서 UP=Y증가
    if (from.y > to.y) return ROBOT::ACTION::DOWN;  // 시뮬레이터에서 DOWN=Y감소
    return ROBOT::ACTION::HOLD;
}

Scheduler::Scheduler() {}

void Scheduler::on_info_updated(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots) {
    if (known_object_map.empty() || known_object_map[0].empty()) return;
    map_width_ = known_object_map.size();
    map_height_ = known_object_map[0].size();

    // 프론티어 갱신, 할당된 것 제외
    frontiers_ = find_frontiers(known_object_map);
    assigned_frontiers_.clear();
    for (auto& pair : drone_target_frontier_)
        assigned_frontiers_.insert(pair.second);

    // 각 드론의 path 및 target 갱신 필요성 체크
    for (auto& robot_ptr : robots) {
        if (robot_ptr->type != ROBOT::TYPE::DRONE) continue;
        int id = robot_ptr->id;
        const Coord& pos = robot_ptr->get_coord();

        // 현재 path가 없다면, 혹은 target frontier에 도달했거나 더이상 유효하지 않다면 새로 할당
        bool need_new_target = false;
        if (drone_path_[id].empty() || path_idx_[id] >= drone_path_[id].size())
            need_new_target = true;
        else if (drone_target_frontier_.count(id) == 0)
            need_new_target = true;
        else {
            // 만약 프론티어가 사라졌으면 (더이상 frontier가 아님) 재할당 필요
            Coord tgt = drone_target_frontier_[id];
            auto it = std::find(frontiers_.begin(), frontiers_.end(), tgt);
            if (it == frontiers_.end()) need_new_target = true;
        }

        if (need_new_target) {
            // 아직 할당되지 않은, 가장 가까운 frontier를 찾음
            int min_dist = std::numeric_limits<int>::max();
            Coord best_f = { -1, -1 };
            for (const auto& f : frontiers_) {
                if (assigned_frontiers_.count(f)) continue;
                int dist = manhattan(pos, f);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_f = f;
                }
            }
            if (best_f.x != -1) {
                // 경로 계산
                std::vector<Coord> path = find_path_bfs(pos, best_f, known_object_map);
                if (!path.empty()) {
                    drone_path_[id] = path;
                    path_idx_[id] = 1; // path[0]==현재위치
                    drone_target_frontier_[id] = best_f;
                    assigned_frontiers_.insert(best_f);
                }
                else {
                    drone_path_[id].clear();
                    path_idx_[id] = 0;
                    drone_target_frontier_.erase(id);
                }
            }
            else {
                // 할당 가능한 프론티어 없음
                drone_path_[id].clear();
                path_idx_[id] = 0;
                drone_target_frontier_.erase(id);
            }
        }
    }
}

bool Scheduler::on_task_reached(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot,
    const TASK& task) {
    // 드론은 작업 안함, 나머지는 작업 수행
    return robot.type != ROBOT::TYPE::DRONE;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot) {
    if (robot.type != ROBOT::TYPE::DRONE) return ROBOT::ACTION::HOLD;
    int id = robot.id;
    const Coord& pos = robot.get_coord();
    if (drone_path_.count(id) && !drone_path_[id].empty() && path_idx_[id] < drone_path_[id].size()) {
        Coord next = drone_path_[id][path_idx_[id]];
        // 바로 옆이 아니면 경로가 잘못된 것임(새로계획 필요)
        if (manhattan(pos, next) != 1) {
            drone_path_[id].clear(); path_idx_[id] = 0;
            drone_target_frontier_.erase(id);
            return ROBOT::ACTION::HOLD;
        }
        ++path_idx_[id];
        return action_to(pos, next);
    }
    return ROBOT::ACTION::HOLD;
}
