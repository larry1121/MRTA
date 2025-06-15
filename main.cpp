#include "simulator.h"
#include "schedular.h"

int main()
{
    constexpr int MAP_SIZE = 20;
    constexpr int NUM_ROBOT = 6;
    constexpr int NUM_MAX_TASKS = 16;
    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    constexpr int ROBOT_ENERGY = TIME_MAX * 6;
    set<Coord> observed_coords;
    set<Coord> updated_coords;

    srand(static_cast<unsigned int>(time(NULL)));

    TIMER timer;
    MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
    int time = -1;
    auto &robots = map.get_robots();
    auto &known_cost_map = map.get_known_cost_map();
    auto &known_object_map = map.get_known_object_map();
    auto &active_tasks = map.get_active_tasks();
    Scheduler scheduler;
    TASKDISPATCHER taskdispatcher(map, TIME_MAX);

#ifdef VERBOSE
    for (int i = 0; i < ROBOT::NUM_ROBOT_TYPE; ++i)
        map.print_cost_map(static_cast<ROBOT::TYPE>(i));
#endif // VERBOSE

    while (++time < TIME_MAX &&
           robots.size() != map.get_exhausted_robot_num() &&
           map.num_total_task != map.get_completed_task_num())
    {
        taskdispatcher.try_dispatch(time);
        observed_coords = map.observed_coord_by_robot();
        updated_coords = map.update_coords(observed_coords);
#ifdef VERBOSE
        cout << "Time : " << time << endl;
        map.print_object_map();
        map.print_robot_summary();
        map.print_task_summary();
#endif // VERBOSE

        timer.start();
        scheduler.on_info_updated(observed_coords,
                                  updated_coords,
                                  known_cost_map,
                                  known_object_map,
                                  active_tasks,
                                  robots);
        timer.stop();

        // Print robot task queues for debugging
        scheduler.print_robot_task_queues(robots, active_tasks);

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
                    timer.start();
                    do_task = scheduler.on_task_reached(observed_coords,
                                                        updated_coords,
                                                        known_cost_map,
                                                        known_object_map,
                                                        active_tasks,
                                                        robots,
                                                        *robot,
                                                        *(task.lock()));
                    timer.stop();
                }

                if (do_task)
                {
                    robot->start_working(task);
                }
                else
                {
                    timer.start();
                    ROBOT::ACTION action = scheduler.idle_action(observed_coords,
                                                                 updated_coords,
                                                                 known_cost_map,
                                                                 known_object_map,
                                                                 active_tasks,
                                                                 robots,
                                                                 *robot);
                    timer.stop();
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

    cout << endl;
    map.print_robot_summary();
    map.print_task_summary();
    size_t unit;
    string units[] = {"ns", "us", "ms", "s"};
    double count = static_cast<double>(timer.time_elapsed.count());
    for (unit = 0; unit < 4 && count >= 1e3; ++unit)
        count /= 1e3;
    cout << "Algorithm time : " << count << units[unit] << endl;
}


//#include "simulator.h"
//#include "schedular.h"
//
//int main()
//{
//    constexpr int MAP_SIZE = 20;
//    constexpr int NUM_ROBOT = 6;
//    constexpr int NUM_MAX_TASKS = 16;
//    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
//    constexpr int WALL_DENSITY = 20;
//    constexpr int TIME_MAX = MAP_SIZE * 100;
//    constexpr int ROBOT_ENERGY = TIME_MAX * 6;
//
//    constexpr int NUM_RUNS = 5;          // �� �� �� ������
//    long long sum_active = 0;
//    long long sum_completed = 0;
//
//    for (int run = 0; run < NUM_RUNS; ++run)
//    {
//        /* ���� �� ȸ���� �õ� ���� �������������������������������������������������������������� */
//        srand(static_cast<unsigned int>(time(NULL)) + run * 12345);
//
//        /* ���� �⺻ ���� ��� ���� �������������������������������������������������������������� */
//        MAP        map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS,
//            NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
//        Scheduler  scheduler;
//        TASKDISPATCHER dispatcher(map, TIME_MAX);
//
//        set<Coord> observed_coords;
//        set<Coord> updated_coords;
//        TIMER      timer;
//
//        int time = -1;
//        auto& robots = map.get_robots();
//        auto& known_cost_map = map.get_known_cost_map();
//        auto& known_object_map = map.get_known_object_map();
//        auto& active_tasks = map.get_active_tasks();
//
//        /* ���� ���� �ùķ��̼� ���� �������������������������������������������������������������� */
//        while (++time < TIME_MAX &&
//            robots.size() != map.get_exhausted_robot_num() &&
//            map.num_total_task != map.get_completed_task_num())
//        {
//            dispatcher.try_dispatch(time);
//
//            observed_coords = map.observed_coord_by_robot();
//            updated_coords = map.update_coords(observed_coords);
//
//            /* �����ٷ� ȣ�� */
//            scheduler.on_info_updated(observed_coords, updated_coords,
//                known_cost_map, known_object_map,
//                active_tasks, robots);
//
//            /* �κ� ���� */
//            for (auto robot : robots)
//            {
//                if (robot->get_status() == ROBOT::STATUS::IDLE)
//                {
//                    auto coord = robot->get_coord();
//                    bool do_task = false;
//                    weak_ptr<TASK> task;
//
//                    if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK))
//                    {
//                        task = map.task_at(coord);
//                        do_task = scheduler.on_task_reached(observed_coords, updated_coords,
//                            known_cost_map, known_object_map,
//                            active_tasks, robots,
//                            *robot, *(task.lock()));
//                    }
//
//                    if (do_task)
//                        robot->start_working(task);
//                    else
//                    {
//                        auto act = scheduler.idle_action(observed_coords, updated_coords,
//                            known_cost_map, known_object_map,
//                            active_tasks, robots, *robot);
//                        robot->start_moving(act);
//                    }
//                }
//
//                /* ���� */
//                if (robot->get_status() == ROBOT::STATUS::MOVING)  robot->move();
//                else if (robot->get_status() == ROBOT::STATUS::WORKING) robot->work();
//            }
//        } // while(time)
//
//        /* ���� ȸ�� ��� ���� ������������������������������������������������������������������������ */
//        int active_cnt = static_cast<int>(map.get_active_tasks().size());
//        int completed_cnt = map.get_completed_task_num();
//
//        std::cout << "\n[RUN " << run << "] finished @ time " << time
//            << "  |  active=" << active_cnt
//            << "  completed=" << completed_cnt << "\n";
//
//        sum_active += active_cnt;
//        sum_completed += completed_cnt;
//    } // for(run)
//
//    /* ���� ��� ��� ���������������������������������������������������������������������������������������� */
//    std::cout << "\n==================== SUMMARY ====================\n";
//    std::cout << "Runs : " << NUM_RUNS << '\n';
//    std::cout << "Average ACTIVE    tasks left : "
//        << (static_cast<double>(sum_active) / NUM_RUNS) << '\n';
//    std::cout << "Average COMPLETED tasks      : "
//        << (static_cast<double>(sum_completed) / NUM_RUNS) << '\n';
//    std::cout << "=================================================\n";
//
//
//    return 0;
//}



//#include "simulator.h"
//#include "schedular.h"
//#include <vector>
//#include <algorithm> // std::sort
//#include <numeric>   // �� �̰� �߰�!!!
//#include <iostream>
//
//int main()
//{
//    constexpr int NUM_RUNS = 10;
//    constexpr int MAP_SIZE = 20;
//    constexpr int NUM_ROBOT = 6;
//    constexpr int NUM_MAX_TASKS = 16;
//    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
//    constexpr int WALL_DENSITY = 20;
//    constexpr int ROBOT_ENERGY = MAP_SIZE * 100 * 6;
//
//    std::vector<int> active_tasks_at_end;
//    std::vector<int> completed_tasks_at_end;
//
//    for (int run = 0; run < NUM_RUNS; ++run) {
//        set<Coord> observed_coords;
//        set<Coord> updated_coords;
//        srand(static_cast<unsigned int>(time(NULL)) + run); // �ٸ� seed��
//
//        TIMER timer;
//        MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
//        int time = -1;
//        auto& robots = map.get_robots();
//        auto& known_cost_map = map.get_known_cost_map();
//        auto& known_object_map = map.get_known_object_map();
//        auto& active_tasks = map.get_active_tasks();
//        Scheduler scheduler;
//        TASKDISPATCHER taskdispatcher(map, MAP_SIZE * 100);
//
//        constexpr int TIME_MAX = MAP_SIZE * 100;
//
//        while (++time < TIME_MAX &&
//            robots.size() != map.get_exhausted_robot_num() &&
//            map.num_total_task != map.get_completed_task_num())
//        {
//            taskdispatcher.try_dispatch(time);
//            observed_coords = map.observed_coord_by_robot();
//            updated_coords = map.update_coords(observed_coords);
//
//            timer.start();
//            scheduler.on_info_updated(observed_coords,
//                updated_coords,
//                known_cost_map,
//                known_object_map,
//                active_tasks,
//                robots);
//            timer.stop();
//
//            for (auto robot : robots) {
//                auto& status = robot->get_status();
//                if (status == ROBOT::STATUS::IDLE) {
//                    auto coord = robot->get_coord();
//                    bool do_task = false;
//                    weak_ptr<TASK> task;
//                    if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK)) {
//                        task = map.task_at(coord);
//                        timer.start();
//                        do_task = scheduler.on_task_reached(observed_coords,
//                            updated_coords,
//                            known_cost_map,
//                            known_object_map,
//                            active_tasks,
//                            robots,
//                            *robot,
//                            *(task.lock()));
//                        timer.stop();
//                    }
//
//                    if (do_task) {
//                        robot->start_working(task);
//                    }
//                    else {
//                        timer.start();
//                        ROBOT::ACTION action = scheduler.idle_action(observed_coords,
//                            updated_coords,
//                            known_cost_map,
//                            known_object_map,
//                            active_tasks,
//                            robots,
//                            *robot);
//                        timer.stop();
//                        robot->start_moving(action);
//                    }
//                }
//
//                if (status == ROBOT::STATUS::MOVING) {
//                    robot->move();
//                }
//                else if (status == ROBOT::STATUS::WORKING) {
//                    robot->work();
//                }
//            }
//        }
//
//        // [���⼭ �� ȸ���� ����� ���]
//        active_tasks_at_end.push_back(map.get_active_tasks().size());
//        completed_tasks_at_end.push_back(map.get_completed_task_num());
//        std::cout << "[Run " << run + 1 << "] "
//            << "Active: " << map.get_active_tasks().size()
//            << ", Completed: " << map.get_completed_task_num()
//            << "\n";
//            cout << endl;
//            map.print_robot_summary();
//    }
//
//    // === [��� ȸ�� ���� �� ���] ===
//    // ���
//    // ���
//    double mean_active = std::accumulate(active_tasks_at_end.begin(),
//        active_tasks_at_end.end(), 0.0) / NUM_RUNS;
//    double mean_completed = std::accumulate(completed_tasks_at_end.begin(),
//        completed_tasks_at_end.end(), 0.0) / NUM_RUNS;
//    // �߾Ӱ�
//    std::vector<int> active_sorted = active_tasks_at_end,
//        completed_sorted = completed_tasks_at_end;
//    std::sort(active_sorted.begin(), active_sorted.end());
//    std::sort(completed_sorted.begin(), completed_sorted.end());
//    int median_active = active_sorted[NUM_RUNS / 2];
//    int median_completed = completed_sorted[NUM_RUNS / 2];
//
//    // ���� NEW : best / worst ������������������������������������������������������������������������������������
//    int best_active = active_sorted.front();           // ���� �۾� ���� ���� ���� run
//    int worst_active = active_sorted.back();            // ���� �۾� ���� ���� ���� run
//    int best_completed = completed_sorted.back();         // �Ϸ� �۾��� ���� ���� run
//    int worst_completed = completed_sorted.front();        // �Ϸ� �۾��� ���� ���� run
//    // ��������������������������������������������������������������������������������������������������������������������������������
//
//    std::cout << "=== Simulation " << NUM_RUNS << "-run Results ===\n";
//    std::cout << "Active tasks at end : mean=" << mean_active
//        << "  median=" << median_active
//        << "  best=" << best_active       // �߰�
//        << "  worst=" << worst_active      // �߰�
//        << '\n';
//    std::cout << "Completed tasks     : mean=" << mean_completed
//        << "  median=" << median_completed
//        << "  best=" << best_completed    // �߰�
//        << "  worst=" << worst_completed   // �߰�
//        << '\n';
//
//    return 0;
//} 