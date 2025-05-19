#include "schedular.h"
#include <cstdlib> // rand()
#include <vector>
#include <limits>
#include <map>
#include <memory>
#include <cstdio>
#include <unordered_set> // Ensure it's included here too for the definition
#include <unordered_map> // For robotToTask definition and usage
#include <algorithm> // Required for std::find_if
#include <tuple>     // Required for std::tuple (Sufferage)
// Define INF if not already defined globally, or use std::numeric_limits directly
// const int INF = std::numeric_limits<int>::max(); // It's better to use std::numeric_limits directly

// To enable Sufferage algorithm, define USE_SUFFERAGE before this point
// For example, uncomment the next line or define it in schedular.h or via compiler flags
// #define USE_SUFFERAGE

// Define and initialize the static member
std::unordered_set<int> Scheduler::lastTaskIds;

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    lastTaskIds.clear();
    for (const auto& task_ptr : active_tasks) {
        if (task_ptr) {
            lastTaskIds.insert(task_ptr->id);
        }
    }
    
    // Remove completed/disappeared tasks from robotToTask
    std::vector<int> robots_with_completed_tasks;
    for (auto const& [robot_id, task_id] : this->robotToTask) {
        if (lastTaskIds.find(task_id) == lastTaskIds.end()) {
            // This task is no longer in active_tasks, so it's considered completed or gone.
            robots_with_completed_tasks.push_back(robot_id);
        }
    }
    for (int robot_id : robots_with_completed_tasks) {
        this->robotToTask.erase(robot_id);
    }
    // Decision to call assign_tasks_min_min can be made here based on game state.
    // For example, if there are unassigned tasks and available robots.
    bool has_unassigned_tasks = false;
    for (const auto& task_ptr : active_tasks) {
        bool assigned = false;
        for (const auto& entry : this->robotToTask) {
            if (entry.second == task_ptr->id) {
                assigned = true;
                break;
            }
        }
        if (!assigned) {
            has_unassigned_tasks = true;
            break;
        }
    }
    bool has_available_robots = false;
    for (const auto& robot_ptr : robots) {
        if (this->robotToTask.find(robot_ptr->id) == this->robotToTask.end()) {
            has_available_robots = true;
            break;
        }
    }
    if (has_unassigned_tasks && has_available_robots) {
        assign_tasks_min_min(active_tasks, robots);
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

    // 로봇이 가진 에너지가 task 수행하는데 필요한 에너지보다 크면 실행
    if (robot.get_energy()>task.get_cost(robot.type)){
        if (robotToTask[robot.id]==task.id){
    return true;
        }
    }
    return false;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    if (robot.id == 0){
        if(rand()%5 ==0){
            if(robot.get_coord().x<2){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().x<4 && robot.get_coord().y!=2 &&robot.get_coord().x!=2){
                return static_cast<ROBOT::ACTION>(2);
            }
            else if(robot.get_coord().x>7){
                return static_cast<ROBOT::ACTION>(2);
            }
            else if(robot.get_coord().x>=4&&robot.get_coord().x!=7&&robot.get_coord().y != 17){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().y<2){
                return static_cast<ROBOT::ACTION>(0);
            }
            else if(robot.get_coord().y>17){
                return static_cast<ROBOT::ACTION>(1);
            }
            else if(robot.get_coord().x==2 && robot.get_coord().y>2){
                return static_cast<ROBOT::ACTION>(1);
            }
            else if(robot.get_coord().x==2 && robot.get_coord().y==2){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().y==2 && robot.get_coord().x<7){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().y==2 && robot.get_coord().x==7){
                return static_cast<ROBOT::ACTION>(0);
            }
            else if(robot.get_coord().x==7 && robot.get_coord().y>2 && robot.get_coord().y !=17){
                return static_cast<ROBOT::ACTION>(0);
            }
            else if(robot.get_coord().x==7 && robot.get_coord().y==17){
                return static_cast<ROBOT::ACTION>(2);
            }
            else if(robot.get_coord().y==17 && robot.get_coord().x>2){
                return static_cast<ROBOT::ACTION>(2);
            }
        }
    }
    if (robot.id == 3){
        if(rand() % 2 == 0){
            if(robot.get_coord().x<12){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().x<14 &&robot.get_coord().y!=2 &&robot.get_coord().x!=12){
                return static_cast<ROBOT::ACTION>(2);
            }
            else if(robot.get_coord().x>17){
                return static_cast<ROBOT::ACTION>(2);
            }
            else if(robot.get_coord().x>=14&&robot.get_coord().x!=17&&robot.get_coord().y != 17){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().y<2){
                return static_cast<ROBOT::ACTION>(0);
            }
            else if(robot.get_coord().y>17){
                return static_cast<ROBOT::ACTION>(1);
            }
            else if(robot.get_coord().x==12 && robot.get_coord().y>2){
                return static_cast<ROBOT::ACTION>(1);
            }
            else if(robot.get_coord().x==12 && robot.get_coord().y==2){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().y==2 && robot.get_coord().x<17){
                return static_cast<ROBOT::ACTION>(3);
            }
            else if(robot.get_coord().y==2 && robot.get_coord().x==17){
                return static_cast<ROBOT::ACTION>(0);
            }
            else if(robot.get_coord().x==17 && robot.get_coord().y>2 && robot.get_coord().y !=17){
                return static_cast<ROBOT::ACTION>(0);
            }
            else if(robot.get_coord().x==17 && robot.get_coord().y==17){
                return static_cast<ROBOT::ACTION>(2);
            }
            else if(robot.get_coord().y==17 && robot.get_coord().x>12){
                return static_cast<ROBOT::ACTION>(2);
            }
        }
    }
    
    // 0: UP, 1: DOWN, 2: LEFT, 3: RIGHT, 4: HOLD 중 랜덤 선택
    return static_cast<ROBOT::ACTION>(4);
}

void Scheduler::assign_tasks_min_min(
    const std::vector<std::shared_ptr<TASK>>& active_tasks,
    const std::vector<std::shared_ptr<ROBOT>>& robots) {

#ifdef USE_SUFFERAGE
    // SUFFERAGE ALGORITHM IMPLEMENTATION
    if (active_tasks.empty() || robots.empty()) {
        return;
    }

    std::vector<std::shared_ptr<TASK>> tasks_to_schedule;
    for (const auto& task_ptr : active_tasks) {
        if (!task_ptr) continue;
        bool is_persistently_assigned = false;
        for (const auto& assignment : this->robotToTask) {
            if (assignment.second == task_ptr->id) {
                is_persistently_assigned = true;
                break;
            }
        }
        if (!is_persistently_assigned) {
            tasks_to_schedule.push_back(task_ptr);
        }
    }

    if (tasks_to_schedule.empty()) {
        return;
    }

    std::vector<bool> task_assigned_this_sufferage_run(tasks_to_schedule.size(), false);
    int num_tasks_assigned_in_sufferage = 0;

    while (num_tasks_assigned_in_sufferage < tasks_to_schedule.size()) {
        int overall_best_task_idx_in_sched = -1;
        int overall_best_robot_idx_in_list = -1;
        double max_sufferage_value = -1.0;

        std::vector<std::tuple<double, int, int, int>> task_sufferages_info;
        // tuple: {sufferage_value, task_idx_in_tasks_to_schedule, robot_idx_for_1st_min_cost, 1st_min_cost}

        for (int i = 0; i < tasks_to_schedule.size(); ++i) {
            if (task_assigned_this_sufferage_run[i]) {
                continue;
            }

            const auto& current_task = tasks_to_schedule[i];
            if (!current_task) continue;

            std::vector<std::pair<int, int>> robot_costs_for_task; // pair: {cost, robot_idx_in_robots_list}

            for (int j = 0; j < robots.size(); ++j) {
                const auto& current_robot = robots[j];
                if (!current_robot) continue;

                // Check if robot is already persistently busy with any task (including one assigned in a previous Sufferage iteration)
                if (this->robotToTask.count(current_robot->id)) {
                    continue;
                }
                // Cost: (task_id * 3 + robot_id * 2) % 15 + 5. Using IDs for stability.
                int cost = (current_task->id * 3 + current_robot->id * 2) % 15 + 5;
                robot_costs_for_task.push_back({cost, j});
            }

            if (robot_costs_for_task.empty()) {
                continue; // No available robot for this task in this iteration
            }

            std::sort(robot_costs_for_task.begin(), robot_costs_for_task.end());

            double current_sufferage = 0.0;
            int first_min_cost = robot_costs_for_task[0].first;
            int best_robot_for_this_task_idx = robot_costs_for_task[0].second;

            if (robot_costs_for_task.size() >= 2) {
                int second_min_cost = robot_costs_for_task[1].first;
                current_sufferage = static_cast<double>(second_min_cost - first_min_cost);
            } else { // Only one robot available, prioritize highly
                current_sufferage = std::numeric_limits<double>::max() / 2.0; // High value
            }
            task_sufferages_info.emplace_back(current_sufferage, i, best_robot_for_this_task_idx, first_min_cost);
        }

        if (task_sufferages_info.empty()) {
            break;
        }

        // Find task with max sufferage from the collected info
        for(const auto& ts_info : task_sufferages_info){
            if(std::get<0>(ts_info) > max_sufferage_value){
                max_sufferage_value = std::get<0>(ts_info);
                overall_best_task_idx_in_sched = std::get<1>(ts_info);
                overall_best_robot_idx_in_list = std::get<2>(ts_info);
            }
        }
        
        if (overall_best_task_idx_in_sched != -1 && overall_best_robot_idx_in_list != -1) {
            const auto& assigned_task = tasks_to_schedule[overall_best_task_idx_in_sched];
            const auto& assigned_robot = robots[overall_best_robot_idx_in_list];

            if(assigned_task && assigned_robot){
                this->robotToTask[assigned_robot->id] = assigned_task->id;
                task_assigned_this_sufferage_run[overall_best_task_idx_in_sched] = true;
                num_tasks_assigned_in_sufferage++;
            }
        } else {
            break; // No task could be assigned in this iteration
        }
    }

#else // Original MIN-MIN LOGIC (from user's current file state)
    if (active_tasks.empty() || robots.empty()) {
        return; // No tasks to assign or no robots to assign to
    }

    int num_tasks = active_tasks.size(); // Use actual size
    int num_robots = robots.size();    // Use actual size

    vector<vector<int>> cost_matrix(num_tasks, vector<int>(num_robots));
    for (int i = 0; i < num_tasks; ++i) {
        for (int j = 0; j < num_robots; ++j) {
            cost_matrix[i][j] = (i * 3 + j * 2) % 15 + 5;
        }
    }

    vector<bool> task_assigned_status(num_tasks, false);
    vector<int> robot_current_completion_time(num_robots, 0);

    int tasks_remaining_to_assign = num_tasks;

    while (tasks_remaining_to_assign > 0) {
        int best_task_idx = -1;
        int best_robot_idx = -1;
        int min_completion_time = std::numeric_limits<int>::max();

        for (int t_idx = 0; t_idx < num_tasks; ++t_idx) {
            if (task_assigned_status[t_idx]) {
                continue;
            }
            // Skip if this task is already assigned to a robot in robotToTask by *another* robot
            // (This check might be more complex depending on desired re-assignment logic)
            bool already_globally_assigned = false;
            if(active_tasks[t_idx]){
                for(const auto& entry : this->robotToTask){
                    if(entry.second == active_tasks[t_idx]->id){
                        if (best_robot_idx != -1 && robots[best_robot_idx] && entry.first == robots[best_robot_idx]->id) {
                           // This task is assigned to the robot we are currently considering as best. This is fine.
                        } else {
                            already_globally_assigned = true; // Assigned to a DIFFERENT robot
                        }
                        break;
                    }
                }
            }
            if (already_globally_assigned) {
                continue;
            }

            int task_specific_best_robot_idx = -1;
            int task_specific_min_completion_time = std::numeric_limits<int>::max();

            for (int r_idx = 0; r_idx < num_robots; ++r_idx) {
                 if (!robots[r_idx]) continue;
                // Only consider robots that don't have a persistent task or are assigned *this* current task t_idx
                bool robot_busy_elsewhere = false;
                if (this->robotToTask.count(robots[r_idx]->id)) {
                    if (active_tasks[t_idx] && this->robotToTask.at(robots[r_idx]->id) != active_tasks[t_idx]->id) {
                        robot_busy_elsewhere = true;
                    }
                }
                if (robot_busy_elsewhere) {
                    continue;
                }

                int current_task_cost_on_robot = cost_matrix[t_idx][r_idx];
                int completion_time = robot_current_completion_time[r_idx] + current_task_cost_on_robot;

                if (completion_time < task_specific_min_completion_time) {
                    task_specific_min_completion_time = completion_time;
                    task_specific_best_robot_idx = r_idx;
                }
            }

            if (task_specific_best_robot_idx != -1 && task_specific_min_completion_time < min_completion_time) {
                min_completion_time = task_specific_min_completion_time;
                best_task_idx = t_idx;
                best_robot_idx = task_specific_best_robot_idx;
            }
        }

        if (best_task_idx == -1 || best_robot_idx == -1) {
            break;
        }

        task_assigned_status[best_task_idx] = true;
        robot_current_completion_time[best_robot_idx] = min_completion_time;
        
        if (robots[best_robot_idx] && active_tasks[best_task_idx]) {
             this->robotToTask[robots[best_robot_idx]->id] = active_tasks[best_task_idx]->id;
        }

        tasks_remaining_to_assign--;
    }
#endif
}
