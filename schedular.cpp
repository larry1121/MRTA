#include "schedular.h"
#include <limits>
#include <queue>
#include <algorithm>
#include <cstdio>
#include <tuple>
#include <cmath>

bool Scheduler::coord_equal(const Coord &a, const Coord &b)
{
    return a.x == b.x && a.y == b.y;
}

void Scheduler::init_tiles(const std::vector<std::vector<OBJECT>> &known_object_map)
{
    if (map_size != -1)
        return;
    map_size = static_cast<int>(known_object_map.size());
    tile_rows = (map_size + tile_size - 1) / tile_size;
    tile_cols = (map_size + tile_size - 1) / tile_size;
    tiles.resize(tile_rows, std::vector<TileInfo>(tile_cols));
    for (int i = 0; i < tile_rows; ++i)
    {
        for (int j = 0; j < tile_cols; ++j)
        {
            int cx = j * tile_size + tile_range;
            int cy = i * tile_size + tile_range;
            if (cx >= map_size)
                cx = map_size - 1;
            if (cy >= map_size)
                cy = map_size - 1;
            // 경계 내 가장 가까운 벽이 아닌 위치 찾기
            if (known_object_map[cx][cy] == OBJECT::WALL)
            {
                bool found = false;
                for (int dist = 1; dist <= tile_range + 1 && !found; ++dist)
                {
                    for (int dx = -dist; dx <= dist && !found; ++dx)
                    {
                        for (int dy = -dist; dy <= dist && !found; ++dy)
                        {
                            int nx = cx + dx, ny = cy + dy;
                            if (nx >= 0 && nx < map_size && ny >= 0 && ny < map_size && known_object_map[nx][ny] != OBJECT::WALL)
                            {
                                cx = nx;
                                cy = ny;
                                found = true;
                            }
                        }
                    }
                }
            }
            tiles[i][j].center = Coord(cx, cy);
        }
    }
}

void Scheduler::update_tile_info(const std::vector<std::vector<OBJECT>> &known_object_map)
{
    for (int i = 0; i < tile_rows; ++i)
    {
        for (int j = 0; j < tile_cols; ++j)
        {
            int ux = tiles[i][j].center.x - tile_range;
            int uy = tiles[i][j].center.y - tile_range;
            int unseen = 0;
            for (int dx = 0; dx < tile_size; ++dx)
            {
                for (int dy = 0; dy < tile_size; ++dy)
                {
                    int x = ux + dx, y = uy + dy;
                    if (x >= 0 && x < map_size && y >= 0 && y < map_size)
                    {
                        if (known_object_map[x][y] == OBJECT::UNKNOWN)
                            unseen++;
                    }
                }
            }
            tiles[i][j].unseen_cells = unseen;
        }
    }
}

std::vector<Coord> Scheduler::plan_path(const Coord &start, const Coord &goal,
                                        const std::vector<std::vector<std::vector<int>>> &known_cost_map, ROBOT::TYPE type,
                                        const std::vector<std::vector<OBJECT>> &known_object_map)
{
    typedef std::pair<int, Coord> PQItem;
    std::vector<std::vector<int>> dist(map_size, std::vector<int>(map_size, std::numeric_limits<int>::max() / 4));
    std::vector<std::vector<Coord>> prev(map_size, std::vector<Coord>(map_size, Coord(-1, -1)));
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    dist[start.x][start.y] = 0;
    pq.push(std::make_pair(0, start));
    static const int dx[4] = {0, 0, -1, 1}, dy[4] = {1, -1, 0, 0};
    while (!pq.empty())
    {
        int cost = pq.top().first;
        Coord cur = pq.top().second;
        pq.pop();
        if (coord_equal(cur, goal))
            break;
        for (int dir = 0; dir < 4; ++dir)
        {
            int nx = cur.x + dx[dir], ny = cur.y + dy[dir];
            if (nx < 0 || ny < 0 || nx >= map_size || ny >= map_size)
                continue;
            if (known_object_map[nx][ny] == OBJECT::WALL)
                continue;
            int ncost = known_cost_map[nx][ny][static_cast<int>(type)];
            if (ncost == -1)
                ncost = 10000; // 예외 처리 필요!
            if (ncost >= std::numeric_limits<int>::max() / 2)
                continue;
            int alt = cost + ncost;
            if (alt < dist[nx][ny])
            {
                dist[nx][ny] = alt;
                prev[nx][ny] = cur;
                pq.push(std::make_pair(alt, Coord(nx, ny)));
            }
        }
    }
    std::vector<Coord> path;
    Coord p = goal;
    if (prev[p.x][p.y].x == -1 && prev[p.x][p.y].y == -1)
        return path;
    while (!coord_equal(p, start))
    {
        if (known_object_map[p.x][p.y] == OBJECT::WALL)
            return std::vector<Coord>();
        path.push_back(p);
        p = prev[p.x][p.y];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

ROBOT::ACTION Scheduler::get_direction(const Coord &from, const Coord &to)
{
    int dx = to.x - from.x, dy = to.y - from.y;
    if (dx == 0 && dy == 1)
        return ROBOT::ACTION::UP;
    if (dx == 0 && dy == -1)
        return ROBOT::ACTION::DOWN;
    if (dx == -1 && dy == 0)
        return ROBOT::ACTION::LEFT;
    if (dx == 1 && dy == 0)
        return ROBOT::ACTION::RIGHT;
    return ROBOT::ACTION::HOLD;
}

void Scheduler::on_info_updated(const set<Coord> &,
                                const set<Coord> &,
                                const std::vector<std::vector<std::vector<int>>> &known_cost_map,
                                const std::vector<std::vector<OBJECT>> &known_object_map,
                                const std::vector<std::shared_ptr<TASK>> &,
                                const std::vector<std::shared_ptr<ROBOT>> &robots)
{
    // 매번 호출될 때마다 시간 증가 (on_info_updated는 매 틱마다 호출됨)
    current_time++;

    init_tiles(known_object_map);
    update_tile_info(known_object_map);

    // 탐색 시간이 아니면 드론들을 정지시킴
    if (!is_exploration_time())
    {
        for (const auto &robot_ptr : robots)
        {
            if (robot_ptr->type == ROBOT::TYPE::DRONE)
            {
                int rid = robot_ptr->id;
                drone_paths[rid].clear();
                drone_targets[rid] = robot_ptr->get_coord();
                printf("[DEBUG] Drone %d: PAUSED (time=%d)\n", rid, current_time);
            }
        }
        return;
    }

    // 이전 할당 정보 초기화
    assigned_targets.clear();

    int n_drones = 0;
    for (const auto &r : robots)
        if (r->type == ROBOT::TYPE::DRONE)
            ++n_drones;

    // 드론들을 ID 순으로 정렬하여 일관된 순서로 타겟 할당
    std::vector<std::shared_ptr<ROBOT>> drone_robots;
    for (const auto &robot_ptr : robots)
    {
        if (robot_ptr->type == ROBOT::TYPE::DRONE)
            drone_robots.push_back(robot_ptr);
    }
    std::sort(drone_robots.begin(), drone_robots.end(),
              [](const std::shared_ptr<ROBOT> &a, const std::shared_ptr<ROBOT> &b)
              {
                  return a->id < b->id;
              });

    for (const auto &robot_ptr : drone_robots)
    {
        int rid = robot_ptr->id;
        Coord drone_pos = robot_ptr->get_coord();

        // 현재 드론이 이동 중이고 목표에 가까이 있다면 새로운 타겟을 찾지 않음
        if (robot_ptr->get_status() == ROBOT::STATUS::MOVING &&
            drone_targets.count(rid) &&
            !drone_paths[rid].empty())
        {
            Coord current_target = drone_targets[rid];
            int dist_to_target = abs(drone_pos.x - current_target.x) + abs(drone_pos.y - current_target.y);
            if (dist_to_target > 3) // 목표까지 거리가 3보다 크면 계속 이동
            {
                assigned_targets.insert({current_target.x, current_target.y});
                continue;
            }
        }

        // 항상 새로운 타일 중심 좌표 탐색(== '가장 효율적인 미지 영역' 우선 탐색)
        double best_score = -1e9;
        Coord best_center = drone_pos;
        std::vector<Coord> best_path;

        // 모든 타일을 거리 순으로 정렬하여 탐색
        std::vector<std::tuple<double, int, int>> tile_candidates;

        for (int i = 0; i < tile_rows; ++i)
        {
            for (int j = 0; j < tile_cols; ++j)
            {
                if (tiles[i][j].unseen_cells == 0)
                    continue;

                Coord target = tiles[i][j].center;

                // 이미 다른 드론이 할당받은 타겟인지 확인
                if (assigned_targets.count({target.x, target.y}))
                    continue;

                if (known_object_map[target.x][target.y] == OBJECT::WALL)
                    continue;

                // 맨하탄 거리 계산
                double distance = abs(drone_pos.x - target.x) + abs(drone_pos.y - target.y);
                double score = static_cast<double>(tiles[i][j].unseen_cells) / (distance + 1);

                tile_candidates.push_back(std::make_tuple(score, i, j));
            }
        }

        // 점수 순으로 정렬 (높은 점수가 먼저)
        std::sort(tile_candidates.begin(), tile_candidates.end(),
                  [](const auto &a, const auto &b)
                  {
                      return std::get<0>(a) > std::get<0>(b);
                  });

        // 상위 후보들 중에서 실제 경로를 계산하여 최적 선택
        for (const auto &candidate : tile_candidates)
        {
            int i = std::get<1>(candidate);
            int j = std::get<2>(candidate);
            double candidate_score = std::get<0>(candidate);

            if (candidate_score <= best_score * 0.5) // 점수가 현재 최고의 절반 이하면 중단
                break;

            Coord target = tiles[i][j].center;
            auto path = plan_path(drone_pos, target, known_cost_map, ROBOT::TYPE::DRONE, known_object_map);
            if (path.empty())
                continue;

            // 실제 경로 길이를 고려한 최종 점수
            double final_score = static_cast<double>(tiles[i][j].unseen_cells) / (path.size() + 1);
            if (final_score > best_score)
            {
                best_score = final_score;
                best_center = target;
                best_path = path;
            }
        }

        // 아무 좌표도 찾지 못한 경우 자리에서만 머물러서 관측
        if (best_path.empty() && known_object_map[drone_pos.x][drone_pos.y] == OBJECT::UNKNOWN)
        {
            best_path.push_back(drone_pos);
            best_center = drone_pos;
            best_score = 1.0;
        }

        if (!best_path.empty())
        {
            drone_paths[rid] = std::deque<Coord>(best_path.begin(), best_path.end());
            drone_targets[rid] = best_center;
            assigned_targets.insert({best_center.x, best_center.y}); // 타겟 할당 기록
            printf("[DEBUG] Drone %d: path=%zu, target=(%d,%d), score=%.3f (time=%d)\n",
                   rid, best_path.size(), best_center.x, best_center.y, best_score, current_time);
        }
        else
        {
            drone_paths[rid].clear();
            drone_targets[rid] = drone_pos;
            printf("[DEBUG] Drone %d: NO path (stay) (time=%d)\n", rid, current_time);
        }
    }
}

bool Scheduler::on_task_reached(const set<Coord> &,
                                const set<Coord> &,
                                const std::vector<std::vector<std::vector<int>>> &,
                                const std::vector<std::vector<OBJECT>> &,
                                const std::vector<std::shared_ptr<TASK>> &,
                                const std::vector<std::shared_ptr<ROBOT>> &,
                                const ROBOT &robot,
                                const TASK &)
{
    return false; // 예측 오류 처리
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &,
                                     const set<Coord> &,
                                     const std::vector<std::vector<std::vector<int>>> &,
                                     const std::vector<std::vector<OBJECT>> &known_object_map,
                                     const std::vector<std::shared_ptr<TASK>> &,
                                     const std::vector<std::shared_ptr<ROBOT>> &,
                                     const ROBOT &robot)
{
    if (robot.type != ROBOT::TYPE::DRONE)
        return ROBOT::ACTION::HOLD;

    // 탐색 시간이 아니면 정지
    if (!is_exploration_time())
        return ROBOT::ACTION::HOLD;

    int rid = robot.id;
    Coord cur = robot.get_coord();
    if (drone_paths.count(rid) && !drone_paths[rid].empty())
    {
        Coord next = drone_paths[rid].front();
        if (coord_equal(cur, next))
            drone_paths[rid].pop_front();
        if (!drone_paths[rid].empty())
        {
            Coord next2 = drone_paths[rid].front();
            if (known_object_map[next2.x][next2.y] == OBJECT::WALL)
                return ROBOT::ACTION::HOLD;
            return get_direction(cur, next2);
        }
    }
    return ROBOT::ACTION::HOLD;
}

bool Scheduler::is_exploration_time() const
{
    // 0~750틱과 1250틱 이후에만 탐색
    return (current_time <= 750) || (current_time >= 1250);
}
