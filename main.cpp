#include "simulator.h"
#include "schedular.h"
#include <fstream>
#include <vector>
#include <string>
#include <tuple>
#include <chrono>
#include <iomanip>
#include <sstream>

struct Hyperparameters
{
    int cluster_dist;
    int energy_margin;
    int max_cluster_size;
    int drone_pause;
    int drone_resume;
    int caterpillar_cost;
    int wheel_cost;
};

int main()
{
    constexpr int MAP_SIZE = 20;
    constexpr int NUM_ROBOT = 6;
    constexpr int NUM_MAX_TASKS = 16;
    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    constexpr int ROBOT_ENERGY = TIME_MAX * 6;
    constexpr int NUM_RUNS_PER_PARAM = 3;

    std::vector<Hyperparameters> param_sets;
    std::vector<int> cluster_dists = {1000, 1200};
    std::vector<int> energy_margins = {6};
    std::vector<int> max_cluster_sizes = {3};
    std::vector<int> drone_pauses = {100};
    std::vector<int> drone_resumes = {550};
    std::vector<int> caterpillar_costs = {250};
    std::vector<int> wheel_costs = {400, 450, 500};

    for (int cat_cost : caterpillar_costs)
    {
        for (int wheel_cost : wheel_costs)
        {
            for (int cd : cluster_dists)
            {
                for (int em : energy_margins)
                {
                    for (int mcs : max_cluster_sizes)
                    {
                        for (int dp : drone_pauses)
                        {
                            for (int dr : drone_resumes)
                            {
                                param_sets.push_back({cd, em, mcs, dp, dr, cat_cost, wheel_cost});
                            }
                        }
                    }
                }
            }
        }
    }

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");
    std::string filename = "results_" + ss.str() + ".csv";

    std::ofstream results_file(filename);
    std::stringstream header;
    header << "CaterpillarCost,WheelCost,ClusterDist,EnergyMargin,MaxClusterSize,DronePause,DroneResume,";
    for (int i = 1; i <= NUM_RUNS_PER_PARAM; ++i)
    {
        header << "Run" << i << "_Tasks,";
    }
    header << "AvgCompletedTasks\n";
    results_file << header.str();

    double best_avg_completed_tasks = -1.0;
    Hyperparameters best_params;

    int set_count = 0;
    for (const auto &params : param_sets)
    {
        set_count++;
        std::cout << "\n--- Running Simulation Set " << set_count << "/" << param_sets.size() << " ---\n";
        std::cout << "Params: CC=" << params.caterpillar_cost << " WC=" << params.wheel_cost
                  << " CD=" << params.cluster_dist << " EM=" << params.energy_margin
                  << " MCS=" << params.max_cluster_size << " DP=" << params.drone_pause
                  << " DR=" << params.drone_resume << std::endl;

        long long total_completed_tasks_for_param = 0;
        std::vector<int> run_results;

        for (int i = 0; i < NUM_RUNS_PER_PARAM; ++i)
        {
            std::cout << "  Run " << i + 1 << "/" << NUM_RUNS_PER_PARAM << "...";
            srand(static_cast<unsigned int>(time(NULL)) + set_count * 1000 + i);

            MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
            int time = -1;
            auto &robots = map.get_robots();
            auto &known_cost_map = map.get_known_cost_map();
            auto &known_object_map = map.get_known_object_map();
            auto &active_tasks = map.get_active_tasks();
            Scheduler scheduler;
            Scheduler::set_unknown_costs(params.caterpillar_cost, params.wheel_cost);
            scheduler.set_hyperparameters(params.cluster_dist, params.energy_margin, params.max_cluster_size, params.drone_pause, params.drone_resume);
            TASKDISPATCHER taskdispatcher(map, TIME_MAX);

            set<Coord> observed_coords;
            set<Coord> updated_coords;

            while (++time < TIME_MAX &&
                   robots.size() != map.get_exhausted_robot_num() &&
                   map.num_total_task != map.get_completed_task_num())
            {
                taskdispatcher.try_dispatch(time);
                observed_coords = map.observed_coord_by_robot();
                updated_coords = map.update_coords(observed_coords);

                scheduler.on_info_updated(observed_coords,
                                          updated_coords,
                                          known_cost_map,
                                          known_object_map,
                                          active_tasks,
                                          robots);

                for (auto robot : robots)
                {
                    auto &status = robot->get_status();
                    if (status == ROBOT::STATUS::IDLE)
                    {
                        auto coord = robot->get_coord();
                        bool do_task = false;
                        weak_ptr<TASK> task;
                        if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK))
                        {
                            task = map.task_at(coord);
                            do_task = scheduler.on_task_reached(observed_coords,
                                                                updated_coords,
                                                                known_cost_map,
                                                                known_object_map,
                                                                active_tasks,
                                                                robots,
                                                                *robot,
                                                                *(task.lock()));
                        }

                        if (do_task)
                        {
                            robot->start_working(task);
                        }
                        else
                        {
                            ROBOT::ACTION action = scheduler.idle_action(observed_coords,
                                                                         updated_coords,
                                                                         known_cost_map,
                                                                         known_object_map,
                                                                         active_tasks,
                                                                         robots,
                                                                         *robot);
                            robot->start_moving(action);
                        }
                    }

                    if (status == ROBOT::STATUS::MOVING)
                    {
                        robot->move();
                    }
                    else if (status == ROBOT::STATUS::WORKING)
                    {
                        robot->work();
                    }
                }
            }
            int completed_tasks = map.get_completed_task_num();
            total_completed_tasks_for_param += completed_tasks;
            run_results.push_back(completed_tasks);
            std::cout << " completed tasks: " << completed_tasks << std::endl;
        }

        double avg_completed_tasks = static_cast<double>(total_completed_tasks_for_param) / NUM_RUNS_PER_PARAM;

        results_file << params.caterpillar_cost << "," << params.wheel_cost << "," << params.cluster_dist << "," << params.energy_margin << "," << params.max_cluster_size
                     << "," << params.drone_pause << "," << params.drone_resume;
        for (int result : run_results)
        {
            results_file << "," << result;
        }
        results_file << "," << avg_completed_tasks << "\n";

        if (avg_completed_tasks > best_avg_completed_tasks)
        {
            best_avg_completed_tasks = avg_completed_tasks;
            best_params = params;
        }
        std::cout << "Average completed tasks for this set: " << avg_completed_tasks << std::endl;
    }

    results_file.close();

    cout << "\n\n==================== Best Parameters Found ====================\n";
    cout << "Caterpillar Cost: " << best_params.caterpillar_cost << "\n";
    cout << "Wheel Cost: " << best_params.wheel_cost << "\n";
    cout << "Cluster Distance Threshold: " << best_params.cluster_dist << "\n";
    cout << "Energy Margin Percent: " << best_params.energy_margin << "\n";
    cout << "Max Cluster Size: " << best_params.max_cluster_size << "\n";
    cout << "Drone Exploration Pause Start: " << best_params.drone_pause << "\n";
    cout << "Drone Exploration Resume: " << best_params.drone_resume << "\n";
    cout << "------------------------------------------------------------\n";
    cout << "Best Average Completed Tasks: " << best_avg_completed_tasks << "\n";
    cout << "=============================================================\n";

    return 0;
}

// #include "simulator.h"
// #include "schedular.h"
//
// int main()
//{
//     constexpr int MAP_SIZE = 20;
//     constexpr int NUM_ROBOT = 6;
//     constexpr int NUM_MAX_TASKS = 16;
//     constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
//     constexpr int WALL_DENSITY = 20;
//     constexpr int TIME_MAX = MAP_SIZE * 100;
//     constexpr int ROBOT_ENERGY = TIME_MAX * 6;
//
//     constexpr int NUM_RUNS = 5;          // �� �� �� ������
//     long long sum_active = 0;
//     long long sum_completed = 0;
//
//     for (int run = 0; run < NUM_RUNS; ++run)
//     {
//         /* ���� �� ȸ���� �õ� ���� �������������������������������������������������������������� */
//         srand(static_cast<unsigned int>(time(NULL)) + run * 12345);
//
//         /* ���� �⺻ ���� ��� ���� �������������������������������������������������������������� */
//         MAP        map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS,
//             NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
//         Scheduler  scheduler;
//         TASKDISPATCHER dispatcher(map, TIME_MAX);
//
//         set<Coord> observed_coords;
//         set<Coord> updated_coords;
//         TIMER      timer;
//
//         int time = -1;
//         auto& robots = map.get_robots();
//         auto& known_cost_map = map.get_known_cost_map();
//         auto& known_object_map = map.get_known_object_map();
//         auto& active_tasks = map.get_active_tasks();
//
//         /* ���� ���� �ùķ��̼� ���� �������������������������������������������������������������� */
//         while (++time < TIME_MAX &&
//             robots.size() != map.get_exhausted_robot_num() &&
//             map.num_total_task != map.get_completed_task_num())
//         {
//             dispatcher.try_dispatch(time);
//
//             observed_coords = map.observed_coord_by_robot();
//             updated_coords = map.update_coords(observed_coords);
//
//             /* �����ٷ� ȣ�� */
//             scheduler.on_info_updated(observed_coords, updated_coords,
//                 known_cost_map, known_object_map,
//                 active_tasks, robots);
//
//             /* �κ� ���� */
//             for (auto robot : robots)
//             {
//                 if (robot->get_status() == ROBOT::STATUS::IDLE)
//                 {
//                     auto coord = robot->get_coord();
//                     bool do_task = false;
//                     weak_ptr<TASK> task;
//
//                     if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK))
//                     {
//                         task = map.task_at(coord);
//                         do_task = scheduler.on_task_reached(observed_coords, updated_coords,
//                             known_cost_map, known_object_map,
//                             active_tasks, robots,
//                             *robot, *(task.lock()));
//                     }
//
//                     if (do_task)
//                         robot->start_working(task);
//                     else
//                     {
//                         auto act = scheduler.idle_action(observed_coords, updated_coords,
//                             known_cost_map, known_object_map,
//                             active_tasks, robots, *robot);
//                         robot->start_moving(act);
//                     }
//                 }
//
//                 /* ���� */
//                 if (robot->get_status() == ROBOT::STATUS::MOVING)  robot->move();
//                 else if (robot->get_status() == ROBOT::STATUS::WORKING) robot->work();
//             }
//         } // while(time)
//
//         /* ���� ȸ�� ��� ���� ������������������������������������������������������������������������ */
//         int active_cnt = static_cast<int>(map.get_active_tasks().size());
//         int completed_cnt = map.get_completed_task_num();
//
//         std::cout << "\n[RUN " << run << "] finished @ time " << time
//             << "  |  active=" << active_cnt
//             << "  completed=" << completed_cnt << "\n";
//
//         sum_active += active_cnt;
//         sum_completed += completed_cnt;
//     } // for(run)
//
//     /* ���� ��� ��� ���������������������������������������������������������������������������������������� */
//     std::cout << "\n==================== SUMMARY ====================\n";
//     std::cout << "Runs : " << NUM_RUNS << '\n';
//     std::cout << "Average ACTIVE    tasks left : "
//         << (static_cast<double>(sum_active) / NUM_RUNS) << '\n';
//     std::cout << "Average COMPLETED tasks      : "
//         << (static_cast<double>(sum_completed) / NUM_RUNS) << '\n';
//     std::cout << "=================================================\n";
//
//
//     return 0;
// }

// #include "simulator.h"
// #include "schedular.h"
// #include <vector>
// #include <algorithm> // std::sort
// #include <numeric>   // �� �̰� �߰�!!!
// #include <iostream>
//
// int main()
//{
//     constexpr int NUM_RUNS = 3;
//     constexpr int MAP_SIZE = 20;
//     constexpr int NUM_ROBOT = 6;
//     constexpr int NUM_MAX_TASKS = 16;
//     constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
//     constexpr int WALL_DENSITY = 20;
//     constexpr int ROBOT_ENERGY = MAP_SIZE * 100 * 6;
//
//     std::vector<int> active_tasks_at_end;
//     std::vector<int> completed_tasks_at_end;
//
//     for (int run = 0; run < NUM_RUNS; ++run) {
//         set<Coord> observed_coords;
//         set<Coord> updated_coords;
//         srand(static_cast<unsigned int>(time(NULL)) + run); // �ٸ� seed��
//
//         TIMER timer;
//         MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
//         int time = -1;
//         auto& robots = map.get_robots();
//         auto& known_cost_map = map.get_known_cost_map();
//         auto& known_object_map = map.get_known_object_map();
//         auto& active_tasks = map.get_active_tasks();
//         Scheduler scheduler;
//         TASKDISPATCHER taskdispatcher(map, MAP_SIZE * 100);
//
//         constexpr int TIME_MAX = MAP_SIZE * 100;
//
//         while (++time < TIME_MAX &&
//             robots.size() != map.get_exhausted_robot_num() &&
//             map.num_total_task != map.get_completed_task_num())
//         {
//             taskdispatcher.try_dispatch(time);
//             observed_coords = map.observed_coord_by_robot();
//             updated_coords = map.update_coords(observed_coords);
//
//             timer.start();
//             scheduler.on_info_updated(observed_coords,
//                 updated_coords,
//                 known_cost_map,
//                 known_object_map,
//                 active_tasks,
//                 robots);
//             timer.stop();
//
//             for (auto robot : robots) {
//                 auto& status = robot->get_status();
//                 if (status == ROBOT::STATUS::IDLE) {
//                     auto coord = robot->get_coord();
//                     bool do_task = false;
//                     weak_ptr<TASK> task;
//                     if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK)) {
//                         task = map.task_at(coord);
//                         timer.start();
//                         do_task = scheduler.on_task_reached(observed_coords,
//                             updated_coords,
//                             known_cost_map,
//                             known_object_map,
//                             active_tasks,
//                             robots,
//                             *robot,
//                             *(task.lock()));
//                         timer.stop();
//                     }
//
//                     if (do_task) {
//                         robot->start_working(task);
//                     }
//                     else {
//                         timer.start();
//                         ROBOT::ACTION action = scheduler.idle_action(observed_coords,
//                             updated_coords,
//                             known_cost_map,
//                             known_object_map,
//                             active_tasks,
//                             robots,
//                             *robot);
//                         timer.stop();
//                         robot->start_moving(action);
//                     }
//                 }
//
//                 if (status == ROBOT::STATUS::MOVING) {
//                     robot->move();
//                 }
//                 else if (status == ROBOT::STATUS::WORKING) {
//                     robot->work();
//                 }
//             }
//         }
//
//         // [���⼭ �� ȸ���� ����� ���]
//         active_tasks_at_end.push_back(map.get_active_tasks().size());
//         completed_tasks_at_end.push_back(map.get_completed_task_num());
//         std::cout << "[Run " << run + 1 << "] "
//             << "Active: " << map.get_active_tasks().size()
//             << ", Completed: " << map.get_completed_task_num()
//             << "\n";
//             cout << endl;
//             map.print_robot_summary();
//     }
//
//     // === [��� ȸ�� ���� �� ���] ===
//     // ���
//     // ���
//     double mean_active = std::accumulate(active_tasks_at_end.begin(),
//         active_tasks_at_end.end(), 0.0) / NUM_RUNS;
//     double mean_completed = std::accumulate(completed_tasks_at_end.begin(),
//         completed_tasks_at_end.end(), 0.0) / NUM_RUNS;
//     // �߾Ӱ�
//     std::vector<int> active_sorted = active_tasks_at_end,
//         completed_sorted = completed_tasks_at_end;
//     std::sort(active_sorted.begin(), active_sorted.end());
//     std::sort(completed_sorted.begin(), completed_sorted.end());
//     int median_active = active_sorted[NUM_RUNS / 2];
//     int median_completed = completed_sorted[NUM_RUNS / 2];
//
//     // ���� NEW : best / worst ������������������������������������������������������������������������������������
//     int best_active = active_sorted.front();           // ���� �۾� ���� ���� ���� run
//     int worst_active = active_sorted.back();            // ���� �۾� ���� ���� ���� run
//     int best_completed = completed_sorted.back();         // �Ϸ� �۾��� ���� ���� run
//     int worst_completed = completed_sorted.front();        // �Ϸ� �۾��� ���� ���� run
//     // ��������������������������������������������������������������������������������������������������������������������������������
//
//     std::cout << "=== Simulation " << NUM_RUNS << "-run Results ===\n";
//     std::cout << "Active tasks at end : mean=" << mean_active
//         << "  median=" << median_active
//         << "  best=" << best_active       // �߰�
//         << "  worst=" << worst_active      // �߰�
//         << '\n';
//     std::cout << "Completed tasks     : mean=" << mean_completed
//         << "  median=" << median_completed
//         << "  best=" << best_completed    // �߰�
//         << "  worst=" << worst_completed   // �߰�
//         << '\n';
//
//     return 0;
// }