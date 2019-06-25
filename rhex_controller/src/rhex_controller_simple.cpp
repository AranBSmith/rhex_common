#include <iostream>
#include <rhex_controller/rhex_controller_simple.hpp>

using namespace rhex_controller;

int main()
{
    RhexControllerSimple controller({1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}, {});

    for (double t = 0.0; t <= 5.0; t += 0.1) {
        auto angles = controller.pos(t);

        std::cout << "angles: " ;
        for (int i = 0; i < angles.size(); i++) {
                std::cout << angles.at(i) << ' ';
        }
        std::cout << std::endl;
    }
    return 0;
}
