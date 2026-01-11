#ifndef _FILO2_NEIGHBORACCEPTANCE_HPP_
#define _FILO2_NEIGHBORACCEPTANCE_HPP_

#include <cmath>
#include <random>

#include "../solution/Solution.hpp"

namespace cobra {

#ifdef TIMELIMIT
    // Time-based simulated annealing helper class.
    class TimeBasedSimulatedAnnealing {

    public:
        TimeBasedSimulatedAnnealing(float initial_temperature_, float final_temperature_, std::mt19937 &rand_engine_, int time_ms)
            : rand_engine(rand_engine_), uniform_dist(0.0f, 1.0f) {

            initial_temperature = initial_temperature_;
            final_temperature = final_temperature_;
            temp_ratio = final_temperature / initial_temperature;
            period = time_ms;
        }


        bool accept(double solution_cost, const cobra::Solution &neighbor, int elapsed_time_ms) {
            const double temperature = initial_temperature * std::pow(temp_ratio, elapsed_time_ms / static_cast<double>(period));
            return neighbor.get_cost() < solution_cost - temperature * std::log(uniform_dist(rand_engine));
        }

        double get_temperature(int elapsed_time_ms) const {
            return initial_temperature * std::pow(temp_ratio, static_cast<double>(elapsed_time_ms) / static_cast<double>(period));
        }

        void decrease_temperature() { }

    private:
        double initial_temperature;
        double final_temperature;
        double temp_ratio;
        int period;

        std::mt19937 &rand_engine;
        std::uniform_real_distribution<double> uniform_dist;
    };
#else
    // Simulated annealing helper class.
    class SimulatedAnnealing {
    public:
        SimulatedAnnealing(double initial_temperature_, double final_temperature_, std::mt19937 &rand_engine_, int max_iter)
            : rand_engine(rand_engine_), uniform_dist(0.0, 1.0) {

            initial_temperature = initial_temperature_;
            final_temperature = final_temperature_;
            period = max_iter;

            temperature = initial_temperature;
            factor = std::pow(final_temperature / initial_temperature, 1.0 / static_cast<double>(period));
        }

        void decrease_temperature() {
            temperature *= factor;
        }

        bool accept(const double reference_solution_cost, const Solution &neighbor) {
            return neighbor.get_cost() < reference_solution_cost - temperature * std::log(uniform_dist(rand_engine));
        }

        double get_temperature() const {
            return temperature;
        }

    private:
        double initial_temperature;
        double final_temperature;
        double temperature;
        int period;

        std::mt19937 &rand_engine;
        std::uniform_real_distribution<double> uniform_dist;

        double factor;
    };
#endif

}  // namespace cobra

#endif