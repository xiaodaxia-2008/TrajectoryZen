#include <TrajectoryZen/TrajectoryToppra/LinearProgram.h>

#include <matplot/matplot.h>
#include <spdlog/spdlog.h>

#include <ranges>


int main()
{
    using namespace tz::lp;

    auto construct_constraint = [](const std::array<double, 2> &pa,
                                   const std::array<double, 2> &pb) {
        auto a = pa[1] - pb[1];
        auto b = pb[0] - pa[0];
        auto c = pb[0] * pa[1] - pa[0] * pb[1];

        return Constraint2D{.a = a, .b = b, .c = c};
    };


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
    // sol.x = 0, sol.y = 1

    spdlog::info("minimize ({:.2})*x + ({:.2})*y", obj.a, obj.b);
    spdlog::info("sol: {}, {}, staus: {}", sol.x, sol.y,
                 std::to_underlying(sol.status));
    matplot::hold(true);
    matplot::plot(std::vector{sol.x}, std::vector{sol.y}, "*");

    std::vector<double> xs = matplot::linspace(-5, 15, 100);
    for (auto &[a, b, c] :
         constraints | std::views::take(constraints.size() - 4)) {
        matplot::plot(xs, matplot::transform(
                              xs, [&](double x) { return (c - a * x) / b; }));
        spdlog::info("({:.2})*x + ({:.2})*y <= {:.2}", a, b, c);
    }

    double c = obj.a * sol.x + obj.b * sol.y;
    matplot::plot(xs,
                  matplot::transform(
                      xs, [&](double x) { return (c - obj.a * x) / obj.b; }),
                  "g-")
        ->line_width(2.);


    matplot::xlim({-5., 15.});
    matplot::ylim({-5., 15.});
    matplot::grid(true);
    matplot::axis("equal");
    matplot::show();

    spdlog::info("exit");

    return 0;
}