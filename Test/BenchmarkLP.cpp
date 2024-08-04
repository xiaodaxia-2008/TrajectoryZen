#include <catch.hpp>
#include <TrajectoryZen/TrajectoryToppra/LinearProgram.h>
#include <random>

TEST_CASE("BenchmarkLP", "[BenchmarkLP]")
{
    using namespace tz::lp;

    Objective2D obj{.a = 1, .b = 0.5};

    std::vector<Constraint2D> constraints{
        Constraint2D(6, -1, 36),
        Constraint2D(8, 7, 56),
        Constraint2D(-9, 5, 5),
        Constraint2D(-1, -1, -1),
        Constraint2D{.a = 1, .b = 0, .c = 10},
        Constraint2D{.a = -1, .b = 0, .c = 0},
        Constraint2D{.a = 0, .b = 1, .c = 10},
        Constraint2D{.a = 0, .b = -1, .c = 0},
    };

    auto sol = LP2D(obj, constraints);
    CHECK(sol.x == 0);
    CHECK(sol.y == 1);
    CHECK(sol.status == Status::OPTIMAL);
    BENCHMARK("BenchmarkLP") { LP2D(obj, constraints); };
}