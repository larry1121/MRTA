#include "schedular.h"
#include <limits>
#include <queue>
#include <algorithm>
#include <cstdio>
#include <set>

bool Scheduler::coord_equal(const Coord& a, const Coord& b) {
    return a.x == b.x && a.y == b.y;
}

void Scheduler::init_tiles(const std::vector<std::vector<OBJECT>>& known_object_map) {
    if (map_size != -1) return;
    map_size = static_cast<int>(known_object_map.size());
}

std::vector<Coord> Scheduler::plan_path(const Coord& start, const Coord& goal,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map, ROBOT::TYPE type,
    const std::vector<std::vector<OBJECT>>& known_object_map) {
    typedef std::pair<int, Coord> PQItem;
    std::vector<std::vector<int>> dist(map_size, std::vector<int>(map_size, std::numeric_limits<int>::max() / 4));
    std::vector<std::vector<Coord>> prev(map_size, std::vector<Coord>(map_size, Coord(-1, -1)));
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    dist[start.x][start.y] = 0;
    pq.push(std::make_pair(0, start));
    static const int dx[4] = { 0,0,-1,1 }, dy[4] = { 1,-1,0,0 };
    while (!pq.empty()) {
        int cost = pq.top().first;
        Coord cur = pq.top().second;
        pq.pop();
        if (coord_equal(cur, goal)) break;
        for (int dir = 0; dir < 4; ++dir) {
            int nx = cur.x + dx[dir], ny = cur.y + dy[dir];
            if (nx < 0 || ny < 0 || nx >= map_size || ny >= map_size) continue;
            if (known_object_map[nx][ny] == OBJECT::WALL) continue;
            int ncost = known_cost_map[nx][ny][static_cast<int>(type)];
            if (ncost == -1) ncost = 10000; // 미확인 영역 진입 강제 허용
            if (ncost >= std::numeric_limits<int>::max() / 2) continue;
            int alt = cost + ncost;
            if (alt < dist[nx][ny]) {
                dist[nx][ny] = alt;
                prev[nx][ny] = cur;
                pq.push(std::make_pair(alt, Coord(nx, ny)));
            }
        }
    }
    std::vector<Coord> path;
    Coord p = goal;
    if (prev[p.x][p.y].x == -1 && prev[p.x][p.y].y == -1) return path;
    while (!coord_equal(p, start)) {
        if (known_object_map[p.x][p.y] == OBJECT::WALL) return std::vector<Coord>();
        path.push_back(p);
        p = prev[p.x][p.y];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

ROBOT::ACTION Scheduler::get_direction(const Coord& from, const Coord& to) {
    int dx = to.x - from.x, dy = to.y - from.y;
    if (dx == 0 && dy == 1) return ROBOT::ACTION::UP;
    if (dx == 0 && dy == -1) return ROBOT::ACTION::DOWN;
    if (dx == -1 && dy == 0) return ROBOT::ACTION::LEFT;
    if (dx == 1 && dy == 0) return ROBOT::ACTION::RIGHT;
    return ROBOT::ACTION::HOLD;
}

void Scheduler::on_info_updated(const set<Coord>&,
    const set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>& known_cost_map,
    const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>& robots)
{
    init_tiles(known_object_map);

    // 드론 목표 칸 예약(중복 방지)
    cell_reserved.clear();
    for (std::unordered_map<int, Coord>::iterator it = drone_targets.begin(); it != drone_targets.end(); ++it) {
        cell_reserved.insert(it->second);
    }

    for (size_t k = 0; k < robots.size(); ++k) {
        const auto& robot_ptr = robots[k];
        if (robot_ptr->type != ROBOT::TYPE::DRONE) continue;
        int rid = robot_ptr->id;
        Coord drone_pos = robot_ptr->get_coord();

        if (robot_ptr->get_energy() <= 0) {
            drone_paths[rid].clear();
            drone_targets[rid] = drone_pos;
            continue;
        }
        if (drone_paths.count(rid) && !drone_paths[rid].empty())
            continue;

        std::vector<Coord> candidate_targets;
        // "자신이 도달 가능한 모든 known 셀(벽X) 중, 관측범위 내 unknown이 1개라도 있는 칸"
        for (int x = 0; x < map_size; ++x) {
            for (int y = 0; y < map_size; ++y) {
                if (known_object_map[x][y] == OBJECT::WALL) continue;
                if (cell_reserved.count(Coord(x, y))) continue;
                bool has_unknown = false;
                for (int dx = -2; dx <= 2 && !has_unknown; ++dx) {
                    for (int dy = -2; dy <= 2 && !has_unknown; ++dy) {
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || nx >= map_size || ny < 0 || ny >= map_size) continue;
                        if (known_object_map[nx][ny] == OBJECT::UNKNOWN) has_unknown = true;
                    }
                }
                if (!has_unknown) continue;
                candidate_targets.push_back(Coord(x, y));
            }
        }

        std::vector<Coord> best_path;
        Coord best_target = drone_pos;
        int min_path_len = std::numeric_limits<int>::max();

        for (size_t i = 0; i < candidate_targets.size(); ++i) {
            std::vector<Coord> path = plan_path(drone_pos, candidate_targets[i], known_cost_map, ROBOT::TYPE::DRONE, known_object_map);
            if (path.empty()) continue;
            if ((int)path.size() < min_path_len) {
                min_path_len = (int)path.size();
                best_target = candidate_targets[i];
                best_path = path;
            }
        }

        if (!best_path.empty()) {
            drone_paths[rid] = std::deque<Coord>(best_path.begin(), best_path.end());
            drone_targets[rid] = best_target;
            cell_reserved.insert(best_target);
            printf("[DEBUG] Drone %d: path=%zu, target=(%d,%d)\n", rid, best_path.size(), best_target.x, best_target.y);
        }
        else {
            drone_paths[rid].clear();
            drone_targets[rid] = drone_pos;
            printf("[DEBUG] Drone %d: NO path (stay)\n", rid);
        }
    }
}

bool Scheduler::on_task_reached(const set<Coord>&,
    const set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>&,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&,
    const ROBOT& robot,
    const TASK&)
{
    return false; // 드론은 작업하지 않음
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>&,
    const set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&,
    const ROBOT& robot)
{
    if (robot.type != ROBOT::TYPE::DRONE) return ROBOT::ACTION::HOLD;
    int rid = robot.id;
    Coord cur = robot.get_coord();
    if (drone_paths.count(rid) && !drone_paths[rid].empty()) {
        Coord next = drone_paths[rid].front();
        if (coord_equal(cur, next)) drone_paths[rid].pop_front();
        if (!drone_paths[rid].empty()) {
            Coord next2 = drone_paths[rid].front();
            if (known_object_map[next2.x][next2.y] == OBJECT::WALL)
                return ROBOT::ACTION::HOLD;
            return get_direction(cur, next2);
        }
    }
    return ROBOT::ACTION::HOLD;
}
