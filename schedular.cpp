#include "schedular.h"
#include <queue>
#include <limits>
#include <cmath>
#include <random>

// 맨해튼 거리 휴리스틱
static int manhattan(const Coord& a, const Coord& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// 프론티어: known과 UNKNOWN의 경계(EMPTY/TASK && 인접 UNKNOWN)
std::vector<Coord> Scheduler::find_frontiers(const std::vector<std::vector<OBJECT>>& known_object_map) {
    std::vector<Coord> result;
    int w = known_object_map.size(), h = known_object_map[0].size();
    for (int x = 0; x < w; ++x)
        for (int y = 0; y < h; ++y) {
            if (known_object_map[x][y] == OBJECT::EMPTY || known_object_map[x][y] == OBJECT::TASK) {
                static const int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 };
                for (int d = 0; d < 4; ++d) {
                    int nx = x + dx[d], ny = y + dy[d];
                    if (nx >= 0 && ny >= 0 && nx < w && ny < h && known_object_map[nx][ny] == OBJECT::UNKNOWN) {
                        result.push_back(Coord{ x, y });
                        break;
                    }
                }
            }
        }
    return result;
}

// 프론티어, 태스크주변, EMPTY cell 배회
std::vector<Coord> Scheduler::find_targets(const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>& active_tasks) {
    std::vector<Coord> candidates = find_frontiers(known_object_map);
    const int MIN_FRONTIER_THRESHOLD = 10;
    int w = known_object_map.size(), h = known_object_map[0].size();
    if ((int)candidates.size() > MIN_FRONTIER_THRESHOLD) return candidates;

    // 태스크 주변 known cell도 타겟 추가
    for (size_t i = 0; i < active_tasks.size(); ++i) {
        Coord c = active_tasks[i]->coord;
        for (int dx = -2; dx <= 2; ++dx) for (int dy = -2; dy <= 2; ++dy) {
            int nx = c.x + dx, ny = c.y + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (known_object_map[nx][ny] == OBJECT::EMPTY) {
                if (std::find(candidates.begin(), candidates.end(), Coord{ nx, ny }) == candidates.end())
                    candidates.push_back(Coord{ nx, ny });
            }
        }
    }
    // 아무것도 없으면 전체 EMPTY cell 배회
    if (candidates.empty()) {
        for (int x = 0; x < w; ++x) for (int y = 0; y < h; ++y)
            if (known_object_map[x][y] == OBJECT::EMPTY)
                candidates.push_back(Coord{ x, y });
    }
    return candidates;
}

// A* 최단경로 (cost=1, 휴리스틱=맨해튼)
std::vector<Coord> Scheduler::find_path_astar(const Coord& start, const Coord& goal, const std::vector<std::vector<OBJECT>>& map, const ROBOT& robot) {
    if (start == goal) return std::vector<Coord>(1, start);
    int w = map.size(), h = map[0].size();
    typedef std::pair<int, Coord> PQElem;
    std::priority_queue<PQElem, std::vector<PQElem>, std::greater<PQElem> > open;
    std::map<Coord, int> gscore;
    std::map<Coord, Coord> parent;
    open.push(PQElem(manhattan(start, goal), start));
    gscore[start] = 0;
    static const int dx[] = { 0, 0, 1, -1 }, dy[] = { 1, -1, 0, 0 };
    while (!open.empty()) {
        PQElem top_pair = open.top(); open.pop();
        int score = top_pair.first;
        Coord cur = top_pair.second;
        if (cur == goal) {
            std::vector<Coord> path;
            for (Coord p = goal; p != start; p = parent[p]) path.push_back(p);
            path.push_back(start); std::reverse(path.begin(), path.end());
            return path;
        }
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d], ny = cur.y + dy[d];
            Coord nxt{ nx, ny };
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (map[nx][ny] == OBJECT::WALL || map[nx][ny] == OBJECT::UNKNOWN) continue;
            int tentative = gscore[cur] + 1;
            if (!gscore.count(nxt) || tentative < gscore[nxt]) {
                gscore[nxt] = tentative;
                parent[nxt] = cur;
                int fscore = tentative + manhattan(nxt, goal);
                open.push(PQElem(fscore, nxt));
            }
        }
    }
    return std::vector<Coord>();
}

ROBOT::ACTION Scheduler::action_to(const Coord& from, const Coord& to) {
    if (from.x < to.x) return ROBOT::ACTION::RIGHT;
    if (from.x > to.x) return ROBOT::ACTION::LEFT;
    if (from.y < to.y) return ROBOT::ACTION::UP;
    if (from.y > to.y) return ROBOT::ACTION::DOWN;
    return ROBOT::ACTION::HOLD;
}

Scheduler::Scheduler() {}

void Scheduler::on_info_updated(const set<Coord>&, const set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>& known_object_map,
    const std::vector<std::shared_ptr<TASK>>& active_tasks,
    const std::vector<std::shared_ptr<ROBOT>>& robots) {
    if (known_object_map.empty() || known_object_map[0].empty()) return;
    map_width_ = known_object_map.size(); map_height_ = known_object_map[0].size();
    std::vector<Coord> targets = find_targets(known_object_map, active_tasks);
    std::set<Coord> assigned;
    for (std::map<int, Coord>::iterator it = drone_target_.begin(); it != drone_target_.end(); ++it)
        assigned.insert(it->second);
    for (size_t i = 0; i < robots.size(); ++i) {
        const std::shared_ptr<ROBOT>& rp = robots[i];
        if (rp->type != ROBOT::TYPE::DRONE) continue;
        int id = rp->id;
        Coord pos = rp->get_coord();
        bool need_new = drone_path_[id].empty() || path_idx_[id] >= drone_path_[id].size();
        if (!need_new && drone_target_.count(id)) {
            if (std::find(targets.begin(), targets.end(), drone_target_[id]) == targets.end())
                need_new = true;
        }
        if (need_new) {
            int best = 1 << 30; Coord tgt = { -1, -1 };
            for (size_t t = 0; t < targets.size(); ++t) {
                if (!assigned.count(targets[t])) {
                    int d = manhattan(pos, targets[t]);
                    if (d < best) { best = d; tgt = targets[t]; }
                }
            }
            // 그래도 없으면 랜덤 배정
            if (tgt.x == -1 && !targets.empty()) {
                std::random_device rd; std::mt19937 gen(rd());
                int idx = std::uniform_int_distribution<int>(0, (int)targets.size() - 1)(gen);
                tgt = targets[idx];
            }
            if (tgt.x != -1) {
                std::vector<Coord> path = find_path_astar(pos, tgt, known_object_map, *rp);
                if (!path.empty()) {
                    drone_path_[id] = path; path_idx_[id] = 1;
                    drone_target_[id] = tgt; assigned.insert(tgt);
                }
                else { drone_path_[id].clear(); path_idx_[id] = 0; drone_target_.erase(id); }
            }
            else { drone_path_[id].clear(); path_idx_[id] = 0; drone_target_.erase(id); }
        }
    }
}

bool Scheduler::on_task_reached(const set<Coord>&, const set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>&,
    const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&,
    const ROBOT& robot, const TASK&) {
    return robot.type != ROBOT::TYPE::DRONE;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>&, const set<Coord>&,
    const std::vector<std::vector<std::vector<int>>>&,
    const std::vector<std::vector<OBJECT>>& known_object_map, const std::vector<std::shared_ptr<TASK>>&,
    const std::vector<std::shared_ptr<ROBOT>>&, const ROBOT& robot) {
    if (robot.type != ROBOT::TYPE::DRONE) return ROBOT::ACTION::HOLD;
    int id = robot.id; Coord pos = robot.get_coord();
    if (drone_path_.count(id) && !drone_path_[id].empty() && path_idx_[id] < drone_path_[id].size()) {
        Coord next = drone_path_[id][path_idx_[id]];
        if (manhattan(pos, next) != 1) { drone_path_[id].clear(); path_idx_[id] = 0; drone_target_.erase(id); return ROBOT::ACTION::HOLD; }
        ++path_idx_[id]; return action_to(pos, next);
    }
    // 주변으로라도 이동(배회)
    static const int dx[] = { 0,0,1,-1 }, dy[] = { 1,-1,0,0 };
    for (int d = 0; d < 4; ++d) {
        int nx = pos.x + dx[d], ny = pos.y + dy[d];
        if (nx >= 0 && ny >= 0 && nx < map_width_ && ny < map_height_) {
            if (known_object_map[nx][ny] != OBJECT::WALL && known_object_map[nx][ny] != OBJECT::UNKNOWN)
                return action_to(pos, Coord{ nx,ny });
        }
    }
    return ROBOT::ACTION::HOLD;
}
