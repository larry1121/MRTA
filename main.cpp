#include "simulator.h"
#include "schedular.h"
#include <vector>
#include <numeric>
#include <iomanip>
#include <climits>
#include <algorithm>

using namespace std;

struct OptimizationConfig
{
    int distance_threshold;
    double high_priority_weight;
    double mid_priority_weight;
    double candidate_threshold;
    int pause_start;
    int resume_time;
    double avg_tasks;
    int max_tasks;
    int perfect_runs;

    string to_string() const
    {
        return "dist:" + std::to_string(distance_threshold) +
               " hw:" + std::to_string(high_priority_weight) +
               " mw:" + std::to_string(mid_priority_weight) +
               " ct:" + std::to_string(candidate_threshold) +
               " ps:" + std::to_string(pause_start) +
               " rt:" + std::to_string(resume_time);
    }
};

// 실험 모드 설정
constexpr bool EXPERIMENT_MODE = false; // true로 설정하면 파라미터 최적화 실험 실행

int main()
{
    constexpr int MAP_SIZE = 20;
    constexpr int NUM_ROBOT = 6;
    constexpr int NUM_MAX_TASKS = 16;
    constexpr int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    constexpr int ROBOT_ENERGY = TIME_MAX * 6;

    // 베스트 타일 설정
    constexpr int BEST_TILE_SIZE = 5;
    constexpr int BEST_TILE_RANGE = 2;

    if (EXPERIMENT_MODE)
    {
        // 파라미터 최적화 실험
        constexpr int RUNS_PER_CONFIG = 100;

        vector<OptimizationConfig> configs = {
            // distance_threshold, high_weight, mid_weight, candidate_threshold, pause_start, resume_time

            // 기본 및 현재 설정
            {1, 2.0, 1.5, 0.3, 750, 1250}, // 기본 설정
            {2, 2.0, 1.5, 0.3, 750, 1250}, // 현재 설정
            {3, 2.0, 1.5, 0.3, 750, 1250}, // 더 관대한 거리

            // 가중치 변화
            {2, 2.5, 1.8, 0.3, 750, 1250}, // 더 높은 가중치
            {2, 1.8, 1.3, 0.3, 750, 1250}, // 더 낮은 가중치
            {2, 3.0, 2.0, 0.3, 750, 1250}, // 매우 높은 가중치
            {2, 1.5, 1.2, 0.3, 750, 1250}, // 매우 낮은 가중치
            {1, 3.0, 2.2, 0.3, 750, 1250}, // 거리1 + 높은 가중치
            {3, 1.5, 1.2, 0.3, 750, 1250}, // 거리3 + 낮은 가중치

            // 후보 선택 기준 변화
            {2, 2.0, 1.5, 0.2, 750, 1250}, // 더 엄격한 후보 선택
            {2, 2.0, 1.5, 0.4, 750, 1250}, // 더 관대한 후보 선택
            {2, 2.0, 1.5, 0.1, 750, 1250}, // 매우 엄격한 후보 선택
            {2, 2.0, 1.5, 0.5, 750, 1250}, // 매우 관대한 후보 선택
            {1, 2.5, 1.8, 0.1, 750, 1250}, // 공격적 + 엄격한 선택
            {3, 1.8, 1.3, 0.5, 750, 1250}, // 보수적 + 관대한 선택

            // 시간 구간 변화
            {2, 2.0, 1.5, 0.3, 700, 1200}, // 더 짧은 중단 시간
            {2, 2.0, 1.5, 0.3, 800, 1300}, // 더 긴 중단 시간
            {2, 2.0, 1.5, 0.3, 750, 1150}, // 더 빠른 재개
            {2, 2.0, 1.5, 0.3, 650, 1100}, // 매우 짧은 중단 + 빠른 재개
            {2, 2.0, 1.5, 0.3, 850, 1400}, // 매우 긴 중단 + 늦은 재개
            {2, 2.0, 1.5, 0.3, 600, 1250}, // 매우 빠른 중단 (최고 성능)
            {2, 2.0, 1.5, 0.3, 750, 1000}, // 매우 빠른 재개

            // 복합 최적화 설정
            {1, 2.5, 1.8, 0.2, 700, 1200},  // 공격적 설정
            {3, 1.8, 1.3, 0.4, 800, 1300},  // 보수적 설정
            {1, 3.0, 2.0, 0.1, 650, 1150},  // 매우 공격적
            {3, 1.5, 1.2, 0.5, 850, 1350},  // 매우 보수적
            {1, 2.2, 1.6, 0.25, 720, 1180}, // 균형 공격적
            {2, 1.9, 1.4, 0.35, 780, 1280}, // 균형 보수적
            {1, 2.8, 2.1, 0.15, 680, 1120}, // 극공격적
            {2, 2.3, 1.7, 0.25, 730, 1220}, // 중간 공격적

            // 최고 성능 기반 추가 실험
            {2, 2.0, 1.5, 0.2, 600, 1250}, // 최고 성능 + 엄격한 후보
            {2, 2.0, 1.5, 0.4, 600, 1250}, // 최고 성능 + 관대한 후보
            {1, 2.0, 1.5, 0.3, 600, 1250}, // 최고 성능 + 거리1
            {3, 2.0, 1.5, 0.3, 600, 1250}, // 최고 성능 + 거리3
            {2, 2.5, 1.8, 0.3, 600, 1250}, // 최고 성능 + 높은 가중치
            {2, 1.8, 1.3, 0.3, 600, 1250}, // 최고 성능 + 낮은 가중치
        };

        cout << "Running parameter optimization experiment..." << endl;
        cout << "Testing " << configs.size() << " configurations with "
             << RUNS_PER_CONFIG << " runs each..." << endl
             << endl;

        for (size_t config_idx = 0; config_idx < configs.size(); ++config_idx)
        {
            auto &config = configs[config_idx];

            cout << "Testing config " << (config_idx + 1) << "/" << configs.size()
                 << " - " << config.to_string() << endl;

            vector<int> discovered_tasks_per_run;

            for (int run = 0; run < RUNS_PER_CONFIG; ++run)
            {
                srand(static_cast<unsigned int>(time(NULL)) + static_cast<unsigned int>(config_idx * 1000 + run));

                set<Coord> observed_coords;
                set<Coord> updated_coords;

                MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
                int time = -1;
                auto &robots = map.get_robots();
                auto &known_cost_map = map.get_known_cost_map();
                auto &known_object_map = map.get_known_object_map();
                auto &active_tasks = map.get_active_tasks();
                Scheduler scheduler;

                scheduler.set_tile_parameters(BEST_TILE_SIZE, BEST_TILE_RANGE);
                scheduler.set_optimization_parameters(
                    config.distance_threshold, config.high_priority_weight, config.mid_priority_weight,
                    config.candidate_threshold, config.pause_start, config.resume_time);

                TASKDISPATCHER taskdispatcher(map, TIME_MAX);

                while (++time < TIME_MAX &&
                       static_cast<int>(robots.size()) != map.get_exhausted_robot_num() &&
                       map.num_total_task != map.get_completed_task_num())
                {
                    taskdispatcher.try_dispatch(time);
                    observed_coords = map.observed_coord_by_robot();
                    updated_coords = map.update_coords(observed_coords);

                    scheduler.on_info_updated(observed_coords, updated_coords, known_cost_map,
                                              known_object_map, active_tasks, robots);

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
                                do_task = scheduler.on_task_reached(observed_coords, updated_coords,
                                                                    known_cost_map, known_object_map,
                                                                    active_tasks, robots, *robot, *(task.lock()));
                            }

                            if (do_task)
                            {
                                robot->start_working(task);
                            }
                            else
                            {
                                ROBOT::ACTION action = scheduler.idle_action(observed_coords, updated_coords,
                                                                             known_cost_map, known_object_map,
                                                                             active_tasks, robots, *robot);
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

                int discovered_tasks = static_cast<int>(active_tasks.size()) + map.get_completed_task_num();
                discovered_tasks_per_run.push_back(discovered_tasks);
            }

            // Calculate statistics
            double sum = std::accumulate(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end(), 0.0);
            config.avg_tasks = sum / RUNS_PER_CONFIG;
            config.max_tasks = *std::max_element(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end());
            config.perfect_runs = static_cast<int>(std::count(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end(), NUM_MAX_TASKS));

            cout << "  Average: " << fixed << setprecision(2) << config.avg_tasks
                 << ", Max: " << config.max_tasks
                 << ", Perfect: " << config.perfect_runs << "/" << RUNS_PER_CONFIG << endl;
        }

        // Sort by performance
        std::sort(configs.begin(), configs.end(), [](const OptimizationConfig &a, const OptimizationConfig &b)
                  {
            if (a.perfect_runs != b.perfect_runs) return a.perfect_runs > b.perfect_runs;
            if (std::abs(a.avg_tasks - b.avg_tasks) > 0.01) return a.avg_tasks > b.avg_tasks;
            return a.max_tasks > b.max_tasks; });

        cout << "\n=== PARAMETER OPTIMIZATION RESULTS ===" << endl;
        cout << "Rank | Avg Tasks | Max | Perfect | Configuration" << endl;
        cout << "-----|-----------|-----|---------|-------------" << endl;

        for (size_t i = 0; i < std::min(static_cast<size_t>(8), configs.size()); ++i)
        {
            const auto &config = configs[i];
            cout << setw(4) << (i + 1) << " | "
                 << setw(9) << fixed << setprecision(2) << config.avg_tasks << " | "
                 << setw(3) << config.max_tasks << " | "
                 << setw(7) << config.perfect_runs << "/" << RUNS_PER_CONFIG << " | "
                 << config.to_string() << endl;
        }

        cout << "\n=== BEST PARAMETERS ===" << endl;
        const auto &best = configs[0];
        cout << "Distance threshold: " << best.distance_threshold << endl;
        cout << "High priority weight: " << best.high_priority_weight << endl;
        cout << "Mid priority weight: " << best.mid_priority_weight << endl;
        cout << "Candidate threshold: " << best.candidate_threshold << endl;
        cout << "Pause start: " << best.pause_start << endl;
        cout << "Resume time: " << best.resume_time << endl;
        cout << "Average tasks: " << fixed << setprecision(2) << best.avg_tasks << endl;
        cout << "Perfect runs: " << best.perfect_runs << "/" << RUNS_PER_CONFIG << endl;
    }
    else
    {
        // 일반 실행 모드 (기존 코드)
        constexpr int NUM_RUNS = 100;

        vector<int> discovered_tasks_per_run;
        vector<vector<int>> drone_energy_per_run;

        cout << "Running " << NUM_RUNS << " simulations with optimized settings..." << endl;
        cout << "Tile size: " << BEST_TILE_SIZE << ", Tile range: " << BEST_TILE_RANGE << endl
             << endl;

        for (int run = 0; run < NUM_RUNS; ++run)
        {
            srand(static_cast<unsigned int>(time(NULL)) + static_cast<unsigned int>(run));

            set<Coord> observed_coords;
            set<Coord> updated_coords;

            MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
            int time = -1;
            auto &robots = map.get_robots();
            auto &known_cost_map = map.get_known_cost_map();
            auto &known_object_map = map.get_known_object_map();
            auto &active_tasks = map.get_active_tasks();
            Scheduler scheduler;

            scheduler.set_tile_parameters(BEST_TILE_SIZE, BEST_TILE_RANGE);

            TASKDISPATCHER taskdispatcher(map, TIME_MAX);

            while (++time < TIME_MAX &&
                   static_cast<int>(robots.size()) != map.get_exhausted_robot_num() &&
                   map.num_total_task != map.get_completed_task_num())
            {
                taskdispatcher.try_dispatch(time);
                observed_coords = map.observed_coord_by_robot();
                updated_coords = map.update_coords(observed_coords);

                scheduler.on_info_updated(observed_coords, updated_coords, known_cost_map,
                                          known_object_map, active_tasks, robots);

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
                            do_task = scheduler.on_task_reached(observed_coords, updated_coords,
                                                                known_cost_map, known_object_map,
                                                                active_tasks, robots, *robot, *(task.lock()));
                        }

                        if (do_task)
                        {
                            robot->start_working(task);
                        }
                        else
                        {
                            ROBOT::ACTION action = scheduler.idle_action(observed_coords, updated_coords,
                                                                         known_cost_map, known_object_map,
                                                                         active_tasks, robots, *robot);
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

            int discovered_tasks = static_cast<int>(active_tasks.size()) + map.get_completed_task_num();
            discovered_tasks_per_run.push_back(discovered_tasks);

            vector<int> drone_energies;
            for (const auto &robot : robots)
            {
                if (robot->type == ROBOT::TYPE::DRONE)
                {
                    drone_energies.push_back(robot->get_energy());
                }
            }
            drone_energy_per_run.push_back(drone_energies);

            if ((run + 1) % 10 == 0)
            {
                cout << "Completed " << (run + 1) << "/" << NUM_RUNS << " runs" << endl;
            }
        }

        // 기존 통계 출력 코드...
        double sum = std::accumulate(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end(), 0.0);
        double average = sum / NUM_RUNS;

        int min_tasks = *std::min_element(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end());
        int max_tasks = *std::max_element(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end());
        int perfect_runs = std::count(discovered_tasks_per_run.begin(), discovered_tasks_per_run.end(), NUM_MAX_TASKS);

        cout << "\n=== OPTIMIZED SIMULATION RESULTS ===" << endl;
        cout << "Configuration: Tile size " << BEST_TILE_SIZE << ", Range " << BEST_TILE_RANGE << endl;
        cout << "Total simulations: " << NUM_RUNS << endl;
        cout << "Maximum possible tasks: " << NUM_MAX_TASKS << endl;
        cout << "Average discovered tasks: " << fixed << setprecision(2) << average << endl;
        cout << "Minimum discovered tasks: " << min_tasks << endl;
        cout << "Maximum discovered tasks: " << max_tasks << endl;
        cout << "Runs that found all tasks: " << perfect_runs << "/" << NUM_RUNS
             << " (" << fixed << setprecision(1) << (100.0 * perfect_runs / NUM_RUNS) << "%)" << endl;
    }

    return 0;
}