#include "simulator.h"
#include "schedular.h"
#include <vector>
#include <numeric>
#include <iomanip>

int main()
{
    constexpr int MAP_SIZE = 20;
    constexpr int NUM_ROBOT = 6;
    constexpr int NUM_MAX_TASKS = 16;
    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    constexpr int ROBOT_ENERGY = TIME_MAX * 6;
    constexpr int NUM_RUNS = 100;

    std::vector<int> discovered_tasks_per_run;

    cout << "Running " << NUM_RUNS << " simulations..." << endl;

    for (int run = 0; run < NUM_RUNS; ++run)
    {
        // Set different seed for each run
        srand(static_cast<unsigned int>(time(NULL)) + run);

        set<Coord> observed_coords;
        set<Coord> updated_coords;

        TIMER timer;
        MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
        int time = -1;
        auto &robots = map.get_robots();
        auto &known_cost_map = map.get_known_cost_map();
        auto &known_object_map = map.get_known_object_map();
        auto &active_tasks = map.get_active_tasks();
        Scheduler scheduler;
        TASKDISPATCHER taskdispatcher(map, TIME_MAX);

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

        // Record the number of discovered tasks for this run
        int discovered_tasks = static_cast<int>(active_tasks.size()) + map.get_completed_task_num();
        discovered_tasks_per_run.push_back(discovered_tasks);

        // Print progress every 10 runs
        if ((run + 1) % 10 == 0)
        {
            cout << "Completed " << (run + 1) << "/" << NUM_RUNS << " runs" << endl;
        }
    }

    // Calculate statistics
    double sum = std::accumulate(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end(), 0.0);
    double average = sum / NUM_RUNS;

    int min_tasks = *std::min_element(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end());
    int max_tasks = *std::max_element(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end());

    // Count how many runs found all 16 tasks
    int perfect_runs = std::count(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end(), NUM_MAX_TASKS);

    cout << "\n=== SIMULATION RESULTS ===" << endl;
    cout << "Total simulations: " << NUM_RUNS << endl;
    cout << "Maximum possible tasks: " << NUM_MAX_TASKS << endl;
    cout << "Average discovered tasks: " << std::fixed << std::setprecision(2) << average << endl;
    cout << "Minimum discovered tasks: " << min_tasks << endl;
    cout << "Maximum discovered tasks: " << max_tasks << endl;
    cout << "Runs that found all tasks: " << perfect_runs << "/" << NUM_RUNS
         << " (" << std::fixed << std::setprecision(1) << (100.0 * perfect_runs / NUM_RUNS) << "%)" << endl;

    return 0;
}