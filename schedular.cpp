#include "schedular.h"
#include <limits>
#include <queue>
#include <algorithm>
#include <cstdio>

bool Scheduler::coord_equal(const Coord& a, const Coord& b) {
    return a.x == b.x && a.y == b.y;
}

void Scheduler::init_tiles(const std::vector<std::vector<OBJECT>>& known_object_map) {
    if (map_size != -1) return;
    map_size = static_cast<int>(known_object_map.size());
    tile_rows = (map_size + tile_size - 1) / tile_size;
    tile_cols = (map_size + tile_size - 1) / tile_size;
    tiles.resize(tile_rows, std::vector<TileInfo>(tile_cols));
    for (int i = 0; i < tile_rows; ++i) {
        for (int j = 0; j < tile_cols; ++j) {
            int cx = j * tile_size + tile_range;
            int cy = i * tile_size + tile_range;
            if (cx >= map_size) cx = map_size - 1;
            if (cy >= map_size) cy = map_size - 1;
            // 중심이 벽이면 주변 빈 칸 찾기
            if (known_object_map[cx][cy] == OBJECT::WALL) {
                bool found = false;
                for (int dist = 1; dist <= tile_range + 1 && !found; ++dist) {
                    for (int dx = -dist; dx <= dist && !found; ++dx) {
                        for (int dy = -dist; dy <= dist && !found; ++dy) {
                            int nx = cx + dx, ny = cy + dy;
                            if (nx >= 0 && nx < map_size && ny >= 0 && ny < map_size
                                && known_object_map[nx][ny] != OBJECT::WALL) {
                                cx = nx; cy = ny; found = true;
                            }
                        }
                    }
                }
            }
            tiles[i][j].center = Coord(cx, cy);
        }
    }
}

void Scheduler::update_tile_info(const std::vector<std::vector<OBJECT>>& known_object_map) {
    for (int i = 0; i < tile_rows; ++i) {
        for (int j = 0; j < tile_cols; ++j) {
            int ux = tiles[i][j].center.x - tile_range;
            int uy = tiles[i][j].center.y - tile_range;
            int unseen = 0;
            for (int dx = 0; dx < tile_size; ++dx) {
                for (int dy = 0; dy < tile_size; ++dy) {
                    int x = ux + dx, y = uy + dy;
                    if (x >= 0 && x < map_size && y >= 0 && y < map_size) {
                        if (known_object_map[x][y] == OBJECT::UNKNOWN)
                            unseen++;
                    }
                }
            }
            tiles[i][j].unseen_cells = unseen;
        }
    }
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
            if (ncost == -1) ncost = 10000; // 미확인 영역 진입 강제 허용!
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
    update_tile_info(known_object_map);

    int n_drones = 0;
    for (const auto& r : robots) if (r->type == ROBOT::TYPE::DRONE) ++n_drones;

    for (const auto& robot_ptr : robots) {
        if (robot_ptr->type != ROBOT::TYPE::DRONE) continue;
        int rid = robot_ptr->id;
        Coord drone_pos = robot_ptr->get_coord();

        // 항상 다음 타일 중심 목표 선정(== '남은 에너지 모두 소모' 적극 탐색)
        double best_score = -1e9;
        Coord best_center = drone_pos;
        std::vector<Coord> best_path;

        for (int i = 0; i < tile_rows; ++i) {
            std::vector<int> js(tile_cols);
            for (int jj = 0; jj < tile_cols; ++jj) js[jj] = jj;
            if (i % 2) std::reverse(js.begin(), js.end());
            for (int jj : js) {
                int j = (jj + rid) % tile_cols; // 각 드론에 열 분배
                if (tiles[i][j].unseen_cells == 0) continue;
                Coord target = tiles[i][j].center;
                if (known_object_map[target.x][target.y] == OBJECT::WALL) continue;
                auto path = plan_path(drone_pos, target, known_cost_map, ROBOT::TYPE::DRONE, known_object_map);
                if (path.empty()) continue;
                // 적극적으로 '아직 안 밝혀진 셀 많은 타일'을 우선!
                double score = static_cast<double>(tiles[i][j].unseen_cells) / (path.size() + 1);
                if (score > best_score) {
                    best_score = score;
                    best_center = target;
                    best_path = path;
                }
            }
        }
        // 아무 목표도 없으면 내 자리에라도 머무르며 관측
        if (best_path.empty() && known_object_map[drone_pos.x][drone_pos.y] == OBJECT::UNKNOWN) {
            best_path.push_back(drone_pos);
            best_center = drone_pos;
            best_score = 1.0;
        }
        if (!best_path.empty()) {
            drone_paths[rid] = std::deque<Coord>(best_path.begin(), best_path.end());
            drone_targets[rid] = best_center;
            printf("[DEBUG] Drone %d: path=%zu, target=(%d,%d)\n", rid, best_path.size(), best_center.x, best_center.y);
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
