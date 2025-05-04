#include "schedular.h"
#include <algorithm>

queue<Coord> Scheduler::bfs_find_path(const Coord& start, const Coord& goal,
    const vector<vector<vector<int>>>& known_cost_map,
    ROBOT::TYPE type) {
    queue<Coord> path;
    if (start == goal) return path;

    int dx[4] = { 0, 0, -1, 1 };
    int dy[4] = { 1, -1, 0, 0 };
    map<Coord, Coord, CoordCompare> parent;
    set<Coord, CoordCompare> visited;
    queue<Coord> q;

    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        Coord current = q.front(); q.pop();
        if (current == goal) {
            vector<Coord> rev_path;
            Coord temp = goal;
            while (temp != start) {
                rev_path.push_back(temp);
                temp = parent[temp];
            }
            reverse(rev_path.begin(), rev_path.end());
            for (auto& c : rev_path) path.push(c);
            return path;
        }
        for (int dir = 0; dir < 4; dir++) {
            int nx = current.x + dx[dir];
            int ny = current.y + dy[dir];
            if (nx < 0 || ny < 0 || nx >= (int)known_cost_map[0].size() || ny >= (int)known_cost_map[0][0].size()) continue;
            Coord next(nx, ny);
            if (visited.find(next) == visited.end()) {
                visited.insert(next);
                parent[next] = current;
                q.push(next);
            }
        }
    }
    return path;
}

queue<Coord> Scheduler::bfs_find_unobserved(const Coord& start,
    const set<Coord>& observed_coords,
    const vector<vector<OBJECT>>& known_object_map) {
    queue<Coord> path;
    int dx[4] = { 0, 0, -1, 1 };
    int dy[4] = { 1, -1, 0, 0 };
    map<Coord, Coord, CoordCompare> parent;
    set<Coord, CoordCompare> visited;
    queue<Coord> q;

    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        Coord current = q.front(); q.pop();
        if (observed_coords.find(current) == observed_coords.end()) {
            vector<Coord> rev_path;
            Coord temp = current;
            while (temp != start) {
                rev_path.push_back(temp);
                temp = parent[temp];
            }
            reverse(rev_path.begin(), rev_path.end());
            for (auto& c : rev_path) path.push(c);
            return path;
        }
        for (int dir = 0; dir < 4; dir++) {
            int nx = current.x + dx[dir];
            int ny = current.y + dy[dir];
            if (nx < 0 || ny < 0 || nx >= (int)known_object_map.size() || ny >= (int)known_object_map[0].size()) continue;
            Coord next(nx, ny);
            if (visited.find(next) == visited.end()) {
                visited.insert(next);
                parent[next] = current;
                q.push(next);
            }
        }
    }
    return path;
}

void Scheduler::on_info_updated(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots) {
    for (auto& robot : robots) {
        if (robot->get_status() != ROBOT::STATUS::IDLE) continue;

        if (robot->type == ROBOT::TYPE::DRONE) {
            if (robot_paths[robot->id].empty()) {
                robot_paths[robot->id] = bfs_find_unobserved(robot->get_coord(), observed_coords, known_object_map);
            }
        }
        else {
            if (assigned_tasks.find(robot->id) != assigned_tasks.end()) continue;

            int best_cost = 1e9;
            shared_ptr<TASK> best_task = nullptr;
            for (auto& task : active_tasks) {
                if (task->is_done() || task->get_assigned_robot_id() != -1) continue;
                int cost = known_cost_map[static_cast<int>(robot->type)][task->coord.x][task->coord.y];
                if (cost < best_cost) {
                    best_cost = cost;
                    best_task = task;
                }
            }
            if (best_task) {
                assigned_tasks[robot->id] = best_task->id;
                robot_paths[robot->id] = bfs_find_path(robot->get_coord(), best_task->coord, known_cost_map, robot->type);
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
    return robot.type != ROBOT::TYPE::DRONE && robot.get_energy() > task.get_cost(robot.type);
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot) {
    if (!robot_paths[robot.id].empty()) {
        Coord next = robot_paths[robot.id].front();
        Coord now = robot.get_coord();

        int dx = next.x - now.x;
        int dy = next.y - now.y;

        if (abs(dx) + abs(dy) != 1) {
            robot_paths[robot.id] = queue<Coord>();
            return ROBOT::ACTION::HOLD;
        }

        robot_paths[robot.id].pop();
        if (dx == 1) return ROBOT::ACTION::DOWN;
        if (dx == -1) return ROBOT::ACTION::UP;
        if (dy == 1) return ROBOT::ACTION::RIGHT;
        if (dy == -1) return ROBOT::ACTION::LEFT;
    }
    return ROBOT::ACTION::HOLD;
}
