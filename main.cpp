#include "simulator.h"
#include "schedular.h"
#include <vector>
#include <numeric>
#include <iomanip>
#include <climits>

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
    std::vector<std::vector<int>> drone_energy_per_run; // 각 실행에서 드론들의 남은 에너지

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

        // Record remaining energy for each drone
        std::vector<int> drone_energies;
        for (const auto &robot : robots)
        {
            if (robot->type == ROBOT::TYPE::DRONE)
            {
                drone_energies.push_back(robot->get_energy());
            }
        }
        drone_energy_per_run.push_back(drone_energies);

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

    // Calculate drone energy statistics
    int num_drones = drone_energy_per_run.empty() ? 0 : static_cast<int>(drone_energy_per_run[0].size());
    std::vector<double> avg_drone_energy(num_drones, 0.0);
    std::vector<int> min_drone_energy(num_drones, INT_MAX);
    std::vector<int> max_drone_energy(num_drones, 0);

    if (num_drones > 0)
    {
        for (int drone_idx = 0; drone_idx < num_drones; ++drone_idx)
        {
            double total_energy = 0.0;
            for (int run = 0; run < NUM_RUNS; ++run)
            {
                int energy = drone_energy_per_run[run][drone_idx];
                total_energy += energy;
                min_drone_energy[drone_idx] = std::min(min_drone_energy[drone_idx], energy);
                max_drone_energy[drone_idx] = std::max(max_drone_energy[drone_idx], energy);
            }
            avg_drone_energy[drone_idx] = total_energy / NUM_RUNS;
        }
    }

    cout << "\n=== SIMULATION RESULTS ===" << endl;
    cout << "Total simulations: " << NUM_RUNS << endl;
    cout << "Maximum possible tasks: " << NUM_MAX_TASKS << endl;
    cout << "Average discovered tasks: " << std::fixed << std::setprecision(2) << average << endl;
    cout << "Minimum discovered tasks: " << min_tasks << endl;
    cout << "Maximum discovered tasks: " << max_tasks << endl;
    cout << "Runs that found all tasks: " << perfect_runs << "/" << NUM_RUNS
         << " (" << std::fixed << std::setprecision(1) << (100.0 * perfect_runs / NUM_RUNS) << "%)" << endl;

    // Display drone energy statistics
    if (num_drones > 0)
    {
        cout << "\n=== DRONE ENERGY STATISTICS ===" << endl;
        cout << "Initial energy per drone: " << ROBOT_ENERGY << endl;

        for (int drone_idx = 0; drone_idx < num_drones; ++drone_idx)
        {
            cout << "Drone " << drone_idx << ":" << endl;
            cout << "  Average remaining energy: " << std::fixed << std::setprecision(1) << avg_drone_energy[drone_idx] << endl;
            cout << "  Min remaining energy: " << min_drone_energy[drone_idx] << endl;
            cout << "  Max remaining energy: " << max_drone_energy[drone_idx] << endl;
            cout << "  Average energy used: " << std::fixed << std::setprecision(1)
                 << (ROBOT_ENERGY - avg_drone_energy[drone_idx]) << endl;
            cout << "  Energy usage rate: " << std::fixed << std::setprecision(1)
                 << (100.0 * (ROBOT_ENERGY - avg_drone_energy[drone_idx]) / ROBOT_ENERGY) << "%" << endl;
        }

        // Overall drone energy statistics
        double total_avg_energy = 0.0;
        for (int drone_idx = 0; drone_idx < num_drones; ++drone_idx)
        {
            total_avg_energy += avg_drone_energy[drone_idx];
        }
        double overall_avg_energy = total_avg_energy / num_drones;

        cout << "\n=== OVERALL DRONE ENERGY ===" << endl;
        cout << "Average remaining energy (all drones): " << std::fixed << std::setprecision(1) << overall_avg_energy << endl;
        cout << "Average energy used (all drones): " << std::fixed << std::setprecision(1)
             << (ROBOT_ENERGY - overall_avg_energy) << endl;
        cout << "Overall energy usage rate: " << std::fixed << std::setprecision(1)
             << (100.0 * (ROBOT_ENERGY - overall_avg_energy) / ROBOT_ENERGY) << "%" << endl;
    }

    return 0;
}