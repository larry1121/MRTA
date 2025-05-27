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

            // === 16개 완전 발견을 위한 혁신적 실험 ===

            // 1. 극한 빠른 중단 + 긴 재탐색 시간
            {1, 2.0, 1.5, 0.3, 400, 1250}, // 매우 이른 중단
            {1, 2.0, 1.5, 0.3, 450, 1250}, // 이른 중단
            {1, 2.0, 1.5, 0.3, 350, 1250}, // 극도로 이른 중단
            {1, 2.0, 1.5, 0.3, 300, 1250}, // 초극한 이른 중단

            // 2. 거리 0 (매틱 재할당) + 다양한 조합
            {0, 2.0, 1.5, 0.3, 400, 1250}, // 거리0 + 빠른 중단
            {0, 2.0, 1.5, 0.3, 350, 1250}, // 거리0 + 매우 빠른 중단
            {0, 3.0, 2.5, 0.2, 400, 1250}, // 거리0 + 높은 가중치
            {0, 4.0, 3.0, 0.1, 350, 1250}, // 거리0 + 극한 가중치

            // 3. 중단 없는 연속 탐색 (에너지 한계까지)
            {1, 2.0, 1.5, 0.3, 2000, 2001}, // 중단 없음
            {0, 2.0, 1.5, 0.3, 2000, 2001}, // 거리0 + 중단 없음
            {1, 3.0, 2.5, 0.2, 2000, 2001}, // 높은 가중치 + 중단 없음
            {0, 4.0, 3.0, 0.1, 2000, 2001}, // 극한 설정 + 중단 없음

            // 4. 매우 짧은 중단 구간 (빠른 재시작)
            {1, 2.0, 1.5, 0.3, 400, 500}, // 100틱만 중단
            {0, 2.0, 1.5, 0.3, 400, 500}, // 거리0 + 짧은 중단
            {1, 3.0, 2.5, 0.2, 350, 450}, // 높은 가중치 + 짧은 중단
            {0, 4.0, 3.0, 0.1, 300, 400}, // 극한 설정 + 짧은 중단

            // 5. 극한 가중치 실험 (미탐색 영역 절대 우선)
            {1, 10.0, 8.0, 0.3, 400, 1250}, // 극한 높은 가중치
            {0, 10.0, 8.0, 0.1, 350, 1250}, // 거리0 + 극한 가중치
            {1, 5.0, 4.0, 0.2, 400, 1250},  // 매우 높은 가중치
            {0, 8.0, 6.0, 0.15, 300, 1250}, // 거리0 + 매우 높은 가중치

            // 6. 후보 선택 극한 실험
            {1, 2.0, 1.5, 0.05, 400, 1250}, // 극도로 엄격한 선택
            {0, 2.0, 1.5, 0.05, 350, 1250}, // 거리0 + 극엄격
            {1, 2.0, 1.5, 0.9, 400, 1250},  // 극도로 관대한 선택
            {0, 2.0, 1.5, 0.9, 350, 1250},  // 거리0 + 극관대

            // 7. 시간 구간 혁신적 조합
            {1, 2.0, 1.5, 0.3, 200, 1250}, // 매우 이른 중단
            {1, 2.0, 1.5, 0.3, 400, 1500}, // 늦은 재개
            {1, 2.0, 1.5, 0.3, 200, 1500}, // 이른 중단 + 늦은 재개
            {0, 3.0, 2.5, 0.2, 200, 1500}, // 거리0 + 긴 재탐색

            // 8. 비대칭 극한 가중치
            {1, 15.0, 1.0, 0.3, 400, 1250}, // 높은 우선순위만 극강화
            {1, 1.0, 15.0, 0.3, 400, 1250}, // 중간 우선순위만 극강화
            {0, 20.0, 1.0, 0.1, 350, 1250}, // 거리0 + 극비대칭
            {0, 1.0, 20.0, 0.1, 350, 1250}, // 거리0 + 반대 극비대칭

            // 9. 현재 최고 성능 미세 조정
            {1, 2.0, 1.5, 0.3, 500, 1250}, // 현재 최고 (기준)
            {1, 2.0, 1.5, 0.3, 480, 1250}, // 20틱 더 빠름
            {1, 2.0, 1.5, 0.3, 460, 1250}, // 40틱 더 빠름
            {1, 2.0, 1.5, 0.3, 520, 1250}, // 20틱 더 늦음

            // 10. 복합 극한 실험
            {0, 10.0, 8.0, 0.05, 200, 1500},  // 모든 극한 조합
            {0, 15.0, 12.0, 0.02, 150, 1600}, // 초극한 조합
            {1, 8.0, 6.0, 0.1, 250, 1400},    // 높은 성능 조합
            {0, 5.0, 4.0, 0.3, 300, 1300},    // 균형 극한 조합

            // 11. 에너지 효율성 극대화
            {1, 2.0, 1.5, 0.3, 100, 1250}, // 매우 이른 절약
            {0, 2.0, 1.5, 0.3, 100, 1250}, // 거리0 + 이른 절약
            {1, 3.0, 2.5, 0.2, 150, 1250}, // 높은 가중치 + 절약
            {0, 4.0, 3.0, 0.1, 100, 1250}, // 극한 + 최대 절약

            // 12. 재탐색 시간 최적화
            {1, 2.0, 1.5, 0.3, 400, 1100}, // 빠른 재개
            {1, 2.0, 1.5, 0.3, 400, 1000}, // 매우 빠른 재개
            {0, 3.0, 2.5, 0.2, 350, 1000}, // 거리0 + 빠른 재개
            {1, 2.0, 1.5, 0.3, 400, 900},  // 극빠른 재개
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