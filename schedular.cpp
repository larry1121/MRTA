/*
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
