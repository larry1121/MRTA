#include "schedular.h"
#include <cstdlib> // rand()
#include <queue>
#include <cmath>

// 4방향 이동 벡터와 ACTION 매핑
const int dx[4] = {0, 0, -1, 1};
const int dy[4] = {1, -1, 0, 0};
const ROBOT::ACTION actions[4] = {
    ROBOT::ACTION::UP, ROBOT::ACTION::DOWN, ROBOT::ACTION::LEFT, ROBOT::ACTION::RIGHT
};

void Scheduler::update_drone_target(const shared_ptr<ROBOT>& drone, int map_size, const vector<vector<OBJECT>>& known_object_map) {
    int id = drone->id;
    Coord cur = drone->get_coord();
    bool& reverse = drone_reverse[id];

    int next_x = cur.x + (reverse ? -1 : 1);
    int next_y = cur.y;

    if (next_x < 0 || next_x >= map_size || known_object_map[next_x][next_y] == OBJECT::WALL) {
        next_x = cur.x;
        next_y = cur.y + 1;
        reverse = !reverse;
    }
    if (next_y >= map_size || known_object_map[next_x][next_y] == OBJECT::WALL) {
        drone_targets[id] = cur; // HOLD
    } else {
        drone_targets[id] = Coord(next_x, next_y);
    }
}

bool Scheduler::is_map_fully_revealed(const vector<vector<OBJECT>>& known_object_map) {
    int N = known_object_map.size();
    for (int x = 0; x < N; ++x)
        for (int y = 0; y < N; ++y)
            if (known_object_map[x][y] == OBJECT::UNKNOWN)
                return false;
    return true;
}

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    // tick 계산 (known_object_map 크기 변하지 않으므로, tick을 증가시켜 사용)
    ++tick;
    if (map_size == 0) map_size = known_object_map.size();

    // 드론별로 맵 완전 탐색 여부 체크
    bool fully_revealed = is_map_fully_revealed(known_object_map);

    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            int id = robot->id;
            // 0~1/4시점: 맵 완전 탐색하면 정지
            if (tick <= map_size * 5) { // 예: 20*5=100, 실제로는 TIME_MAX/4
                if (fully_revealed) drone_map_fully_revealed[id] = true;
                else drone_map_fully_revealed[id] = false;
            }
            // 1/4~3/4시점: 새 task가 생기면 그 위치를 밝힐 때까지만 움직임
            else if (tick > map_size * 5 && tick <= map_size * 15) {
                // 새로 생긴 task가 있는지 확인
                for (const auto& t : active_tasks) {
                    if (!t->is_done() && t->get_assigned_robot_id() == -1) {
                        // 해당 task 좌표가 아직 밝혀지지 않았으면 움직임
                        if (known_object_map[t->coord.x][t->coord.y] == OBJECT::UNKNOWN) {
                            drone_map_fully_revealed[id] = false;
                            break;
                        } else {
                            drone_map_fully_revealed[id] = true;
                        }
                    }
                }
            } else {
                // 3/4 이후에는 기본적으로 정지
                drone_map_fully_revealed[id] = true;
            }

            // 타겟 갱신
            if (!drone_map_fully_revealed[id] && (drone_targets.count(id) == 0 || robot->get_coord() == drone_targets[id])) {
                update_drone_target(robot, map_size, known_object_map);
            }
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
    // DRONE은 task를 수행할 수 없으므로 예외 처리
    if (robot.type == ROBOT::TYPE::DRONE) {
        return false;
    }

    // 그냥 무조건 task를 수행하도록 한다 (나중에 에너지 체크 넣을 예정)
    return true;
}

ROBOT::ACTION Scheduler::get_next_action_toward(const Coord& start, const Coord& goal, const vector<vector<OBJECT>>& known_object_map) {
    int N = known_object_map.size();
    if (start == goal) return ROBOT::ACTION::HOLD;

    // BFS
    vector<vector<bool>> visited(N, vector<bool>(N, false));
    vector<vector<Coord>> parent(N, vector<Coord>(N, {-1, -1}));
    queue<Coord> q;
    q.push(start);
    visited[start.x][start.y] = true;

    while (!q.empty()) {
        Coord cur = q.front(); q.pop();
        if (cur == goal) break;
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d];
            int ny = cur.y + dy[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;
            if (known_object_map[nx][ny] == OBJECT::WALL) continue;
            if (visited[nx][ny]) continue;
            visited[nx][ny] = true;
            parent[nx][ny] = cur;
            q.push({nx, ny});
        }
    }

    // goal까지 도달 못하면 HOLD
    if (!visited[goal.x][goal.y]) return ROBOT::ACTION::HOLD;

    // goal에서부터 parent를 따라가서 start 바로 다음 칸을 찾음
    Coord cur = goal;
    Coord prev = goal;
    while (parent[cur.x][cur.y] != start) {
        cur = parent[cur.x][cur.y];
        if (cur.x == -1) return ROBOT::ACTION::HOLD; // 경로 없음
    }
    prev = cur;

    // 방향 결정
    for (int d = 0; d < 4; ++d) {
        if (start.x + dx[d] == prev.x && start.y + dy[d] == prev.y)
            return actions[d];
    }
    return ROBOT::ACTION::HOLD;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    if (robot.type == ROBOT::TYPE::DRONE) {
        int id = robot.id;
        if (drone_map_fully_revealed[id]) return ROBOT::ACTION::HOLD;

        Coord cur = robot.get_coord();
        Coord target = drone_targets[id];

        if (cur == target) return ROBOT::ACTION::HOLD;

        // 벽 우회 BFS로 한 칸 이동
        return get_next_action_toward(cur, target, known_object_map);
    }

    int dir = rand() % 5;
    return static_cast<ROBOT::ACTION>(dir);
}
