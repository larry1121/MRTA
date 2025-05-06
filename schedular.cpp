/* ────────────────────────────────────────────────
 *  schedular.cpp  ―  MRTA 과제용 스케줄러 구현
 * ──────────────────────────────────────────────── */
#include "schedular.h"

#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>        // std::abs

 /* =================================================
  *  보조 (정적) 함수
  * ================================================= */
bool Scheduler::is_passable(const Coord& c,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    ROBOT::TYPE type)
{
    const auto& layer = cost_map[static_cast<int>(type)];
    const int   N = static_cast<int>(layer.size());
    const int   M = static_cast<int>(layer[0].size());

    if (c.x < 0 || c.y < 0 || c.x >= N || c.y >= M) return false;
    return layer[c.x][c.y] != std::numeric_limits<int>::max();   /* INF == WALL */
}

std::queue<Coord> Scheduler::vec2q(const std::vector<Coord>& v)
{
    std::queue<Coord> q;
    for (const auto& c : v) q.push(c);
    return q;
}

/* =================================================
 *  A*  (4-방향, 가중치·벽 반영)
 * ================================================= */
std::queue<Coord> Scheduler::astar_path(const Coord& start,
    const Coord& goal,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    ROBOT::TYPE type)
{
    std::queue<Coord> empty;
    if (start.x == goal.x && start.y == goal.y) return empty;

    const auto& layer = cost_map[static_cast<int>(type)];
    const int   N = static_cast<int>(layer.size());
    const int   M = static_cast<int>(layer[0].size());
    const int   dx[4] = { 0, 0,-1, 1 };
    const int   dy[4] = { 1,-1, 0, 0 };
    const int   INF = std::numeric_limits<int>::max();

    struct Node
    {
        Coord c; int g; int f;
        bool operator>(const Node& o) const { return f > o.f; }
    };
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::unordered_map<Coord, int, CoordHash, CoordEq>  g_score;
    std::unordered_map<Coord, Coord, CoordHash, CoordEq>  parent;

    auto H = [&](int x, int y) { return std::abs(x - goal.x) + std::abs(y - goal.y); };

    g_score[start] = 0;
    pq.push({ start, 0, H(start.x,start.y) });

    while (!pq.empty())
    {
        Node cur = pq.top(); pq.pop();
        if (cur.c.x == goal.x && cur.c.y == goal.y)
        {
            std::vector<Coord> rev;
            for (Coord p = goal; !(p.x == start.x && p.y == start.y); p = parent[p])
                rev.push_back(p);
            std::reverse(rev.begin(), rev.end());
            return vec2q(rev);
        }

        for (int d = 0; d < 4; ++d)
        {
            int nx = cur.c.x + dx[d];
            int ny = cur.c.y + dy[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= M)  continue;
            if (layer[nx][ny] == INF)                    continue;   /* WALL */

            Coord nxt(nx, ny);
            int   tentative_g = cur.g + layer[nx][ny];

            auto it = g_score.find(nxt);
            if (it == g_score.end() || tentative_g < it->second)
            {
                g_score[nxt] = tentative_g;
                parent[nxt] = cur.c;
                pq.push({ nxt, tentative_g, tentative_g + H(nx,ny) });
            }
        }
    }
    return empty;          /* goal unreachable */
}

/* =================================================
 *  BFS  –  가장 가까운 UNKNOWN 칸
 * ================================================= */
std::queue<Coord> Scheduler::bfs_to_unknown(const Coord& start,
    const std::vector<std::vector<OBJECT>>& known_obj)
{
    std::queue<Coord> empty;
    const int N = static_cast<int>(known_obj.size());
    const int M = static_cast<int>(known_obj[0].size());
    const int dx[4] = { 0, 0,-1, 1 };
    const int dy[4] = { 1,-1, 0, 0 };

    if (known_obj[start.x][start.y] == OBJECT::WALL) return empty;

    std::map<Coord, Coord, CoordCompare> parent;
    std::set<Coord, CoordCompare>       vis;
    std::queue<Coord>                  q;

    q.push(start); vis.insert(start);

    while (!q.empty())
    {
        Coord cur = q.front(); q.pop();

        if (known_obj[cur.x][cur.y] == OBJECT::UNKNOWN)
        {
            std::vector<Coord> rev;
            for (Coord p = cur; !(p.x == start.x && p.y == start.y); p = parent[p])
                rev.push_back(p);
            std::reverse(rev.begin(), rev.end());
            return vec2q(rev);
        }

        for (int d = 0; d < 4; ++d)
        {
            int nx = cur.x + dx[d];
            int ny = cur.y + dy[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= M)                    continue;
            if (known_obj[nx][ny] == OBJECT::WALL)                         continue;

            Coord nxt(nx, ny);
            if (vis.insert(nxt).second)
            {
                parent[nxt] = cur;
                q.push(nxt);
            }
        }
    }
    return empty;          /* UNKNOWN 이 없음 or 접근 불가 */
}

/* =================================================
 *  정보 갱신 시 스케줄링
 * ================================================= */
void Scheduler::on_info_updated(const std::set<Coord>& /*observed*/,
    const std::set<Coord>& /*updated*/,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    const std::vector<std::vector<OBJECT>>& known_obj,
    const std::vector<std::shared_ptr<TASK>>& active_tasks,
    const std::vector<std::shared_ptr<ROBOT>>& robots)
{
    /* 완료된 작업 정리 */
    for (auto it = assigned_tasks.begin(); it != assigned_tasks.end(); )
    {
        int rid = it->first;
        int tid = it->second;

        auto tp = std::find_if(active_tasks.begin(), active_tasks.end(),
            [&](const std::shared_ptr<TASK>& t) { return t->id == tid; });
        if (tp == active_tasks.end() || (*tp)->is_done())
        {
            robot_paths[rid] = std::queue<Coord>();
            it = assigned_tasks.erase(it);
        }
        else ++it;
    }

    /* IDLE 로봇 스케줄 */
    for (const auto& r : robots)
    {
        if (r->get_status() != ROBOT::STATUS::IDLE) continue;

        /* ── DRONE : 탐색 위주 ───────────────── */
        if (r->type == ROBOT::TYPE::DRONE)
        {
            if (robot_paths[r->id].empty())
                robot_paths[r->id] = bfs_to_unknown(r->get_coord(), known_obj);
            continue;
        }

        /* ── 지상 로봇 : 작업 할당 ────────────── */
        if (assigned_tasks.count(r->id)) continue;           /* 이미 작업 있음 */

        int best_score = std::numeric_limits<int>::max();
        std::shared_ptr<TASK> best_task;
        std::queue<Coord>     best_path;

        for (const auto& t : active_tasks)
        {
            if (t->is_done() || t->get_assigned_robot_id() != -1) continue;

            std::queue<Coord> path = astar_path(r->get_coord(),
                t->coord,
                cost_map,
                r->type);
            if (path.empty()) continue;                      /* 도달 불가 */

            int score = static_cast<int>(path.size()) + t->get_cost(r->type);
            if (score < best_score && r->get_energy() >= score)
            {
                best_score = score;
                best_task = t;
                best_path = path;
            }
        }
        if (best_task)
        {
            assigned_tasks[r->id] = best_task->id;   /* 우리 내부 기록 */
            robot_paths[r->id] = best_path;
        }
    }
}

/* =================================================
 *  작업 위치 도달 후 수행 여부
 * ================================================= */
bool Scheduler::on_task_reached(const std::set<Coord>& /*observed*/,
    const std::set<Coord>& /*updated*/,
    const std::vector<std::vector<std::vector<int>>>& /*known_cost*/,
    const std::vector<std::vector<OBJECT>>& /*known_obj*/,
    const std::vector<std::shared_ptr<TASK>>& /*tasks*/,
    const std::vector<std::shared_ptr<ROBOT>>& /*robots*/,
    const ROBOT& robot,
    const TASK& task)
{
    if (robot.type == ROBOT::TYPE::DRONE) return false;
    return robot.get_energy() >= task.get_cost(robot.type);
}

/* =================================================
 *  IDLE 로봇의 다음 행동
 * ================================================= */
ROBOT::ACTION Scheduler::idle_action(const std::set<Coord>& observed,
    const std::set<Coord>& updated,
    const std::vector<std::vector<std::vector<int>>>& cost_map,
    const std::vector<std::vector<OBJECT>>& known_obj,
    const std::vector<std::shared_ptr<TASK>>& tasks,
    const std::vector<std::shared_ptr<ROBOT>>& robots,
    const ROBOT& robot)
{
    /* 경로가 없으면 재계산 시도 */
    if (robot_paths[robot.id].empty())
        on_info_updated(observed, updated, cost_map, known_obj, tasks, robots);

    /* 여전히 없으면 HOLD */
    if (robot_paths[robot.id].empty()) return ROBOT::ACTION::HOLD;

    Coord next = robot_paths[robot.id].front();

    /* 새로운 정보로 '벽' 이 되었다면 경로 폐기 후 HOLD */
    if (!is_passable(next, cost_map, robot.type))
    {
        robot_paths[robot.id] = std::queue<Coord>();   /* 큐 비움 */
        return ROBOT::ACTION::HOLD;                    /* 다음 tick 에 재계산 */
    }

    robot_paths[robot.id].pop();                       /* 정상 → 소비 */

    int dx = next.x - robot.get_coord().x;
    int dy = next.y - robot.get_coord().y;
    if (std::abs(dx) + std::abs(dy) != 1) return ROBOT::ACTION::HOLD;

    if (dx == 1) return ROBOT::ACTION::DOWN;
    if (dx == -1) return ROBOT::ACTION::UP;
    if (dy == 1) return ROBOT::ACTION::RIGHT;
    if (dy == -1) return ROBOT::ACTION::LEFT;
    return ROBOT::ACTION::HOLD;
}
