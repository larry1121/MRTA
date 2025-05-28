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
        constexpr int RUNS_PER_CONFIG = 50;

        vector<OptimizationConfig> configs = {
            // distance_threshold, high_weight, mid_weight, candidate_threshold, pause_start, resume_time

            // === 극한 설정 기반 Perfect runs 향상 실험 ===

            // 1. 현재 극한 설정 기반 미세 조정
            {0, 20.0, 15.0, 0.02, 200, 1100}, // 현재 극한 설정 (기준점)
            {0, 20.0, 15.0, 0.02, 190, 1100}, // 10틱 더 빠른 중단
            {0, 20.0, 15.0, 0.02, 180, 1100}, // 20틱 더 빠른 중단
            {0, 20.0, 15.0, 0.02, 170, 1100}, // 30틱 더 빠른 중단
            {0, 20.0, 15.0, 0.02, 210, 1100}, // 10틱 더 늦은 중단
            {0, 20.0, 15.0, 0.02, 220, 1100}, // 20틱 더 늦은 중단

            // 2. 극한 가중치 + 초엄격 선택 조합
            {0, 25.0, 20.0, 0.02, 200, 1100}, // 더 높은 가중치
            {0, 30.0, 25.0, 0.02, 200, 1100}, // 매우 높은 가중치
            {0, 40.0, 30.0, 0.02, 200, 1100}, // 극한 가중치
            {0, 50.0, 40.0, 0.02, 200, 1100}, // 초극한 가중치
            {0, 60.0, 50.0, 0.02, 200, 1100}, // 최극한 가중치

            // 3. 후보 선택 기준 극한화
            {0, 20.0, 15.0, 0.01, 200, 1100},  // 더 엄격한 선택
            {0, 20.0, 15.0, 0.005, 200, 1100}, // 극도로 엄격한 선택
            {0, 20.0, 15.0, 0.001, 200, 1100}, // 초극한 엄격한 선택
            {0, 20.0, 15.0, 0.03, 200, 1100},  // 약간 관대한 선택
            {0, 20.0, 15.0, 0.05, 200, 1100},  // 관대한 선택

            // 4. 극한 빠른 중단 실험
            {0, 20.0, 15.0, 0.02, 150, 1100}, // 매우 빠른 중단
            {0, 20.0, 15.0, 0.02, 100, 1100}, // 극도로 빠른 중단
            {0, 20.0, 15.0, 0.02, 50, 1100},  // 초극한 빠른 중단
            {0, 20.0, 15.0, 0.02, 25, 1100},  // 거의 즉시 중단
            {0, 20.0, 15.0, 0.02, 75, 1100},  // 매우 이른 중단

            // 5. 재개 시간 최적화
            {0, 20.0, 15.0, 0.02, 200, 1050}, // 50틱 더 빠른 재개
            {0, 20.0, 15.0, 0.02, 200, 1000}, // 100틱 더 빠른 재개
            {0, 20.0, 15.0, 0.02, 200, 950},  // 150틱 더 빠른 재개
            {0, 20.0, 15.0, 0.02, 200, 900},  // 200틱 더 빠른 재개
            {0, 20.0, 15.0, 0.02, 200, 1150}, // 50틱 더 늦은 재개

            // 6. 비대칭 극한 가중치 실험
            {0, 100.0, 1.0, 0.02, 200, 1100},  // 높은 우선순위만 극강화
            {0, 1.0, 100.0, 0.02, 200, 1100},  // 중간 우선순위만 극강화
            {0, 80.0, 1.0, 0.01, 200, 1100},   // 극비대칭 + 엄격
            {0, 1.0, 80.0, 0.01, 200, 1100},   // 반대 극비대칭 + 엄격
            {0, 150.0, 1.0, 0.005, 200, 1100}, // 초극한 비대칭

            // 7. 복합 극한 조합
            {0, 50.0, 40.0, 0.01, 150, 1000}, // 높은 가중치 + 빠른 시간
            {0, 80.0, 60.0, 0.005, 100, 950}, // 극한 가중치 + 극빠른 시간
            {0, 100.0, 80.0, 0.001, 50, 900}, // 최극한 조합
            {0, 40.0, 30.0, 0.02, 180, 1050}, // 균형 극한 조합
            {0, 60.0, 50.0, 0.01, 170, 1000}, // 고성능 조합

            // 8. 매우 짧은 중단 구간
            {0, 20.0, 15.0, 0.02, 150, 250},  // 100틱만 중단
            {0, 20.0, 15.0, 0.02, 100, 200},  // 100틱만 중단 (더 빠름)
            {0, 30.0, 25.0, 0.01, 150, 250},  // 높은 가중치 + 짧은 중단
            {0, 50.0, 40.0, 0.005, 100, 200}, // 극한 가중치 + 짧은 중단
            {0, 80.0, 60.0, 0.001, 75, 175},  // 최극한 + 짧은 중단

            // 9. 중단 없는 연속 탐색
            {0, 20.0, 15.0, 0.02, 2000, 2001},    // 중단 없음
            {0, 50.0, 40.0, 0.01, 2000, 2001},    // 높은 가중치 + 중단 없음
            {0, 100.0, 80.0, 0.005, 2000, 2001},  // 극한 가중치 + 중단 없음
            {0, 150.0, 100.0, 0.001, 2000, 2001}, // 최극한 + 중단 없음
            {0, 80.0, 1.0, 0.01, 2000, 2001},     // 비대칭 + 중단 없음

            // 10. 혁신적 시간 조합
            {0, 20.0, 15.0, 0.02, 50, 1500},   // 매우 이른 중단 + 늦은 재개
            {0, 30.0, 25.0, 0.01, 25, 1600},   // 극이른 중단 + 매우 늦은 재개
            {0, 50.0, 40.0, 0.005, 75, 1400},  // 높은 가중치 + 긴 재탐색
            {0, 80.0, 60.0, 0.001, 100, 1300}, // 극한 가중치 + 긴 재탐색
            {0, 100.0, 1.0, 0.01, 50, 1700},   // 비대칭 + 최대 재탐색

            // 11. 정밀 미세 조정
            {0, 22.0, 17.0, 0.02, 200, 1100},  // 미세 가중치 상향
            {0, 18.0, 13.0, 0.02, 200, 1100},  // 미세 가중치 하향
            {0, 20.0, 15.0, 0.018, 200, 1100}, // 미세 후보 조정
            {0, 20.0, 15.0, 0.022, 200, 1100}, // 미세 후보 조정
            {0, 20.0, 15.0, 0.02, 195, 1095},  // 모든 값 미세 조정

            // 12. 극한 실험
            {0, 200.0, 150.0, 0.001, 25, 1800},  // 모든 값 극한
            {0, 300.0, 1.0, 0.0001, 10, 1900},   // 초극한 비대칭
            {0, 500.0, 400.0, 0.0005, 50, 1750}, // 최극한 조합
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