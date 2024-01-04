//
// Created by ShenGuoHao on 2023/11/27.
//

#include "matplotlibcpp.h"
#include <vector>
#include <cmath>

namespace plt = matplotlibcpp;

int main() {
    std::vector<double> x = {1, 2, 3, 4};
    std::vector<double> y = {1, 4, 9, 16};

    plt::plot(x, y);
    plt::show();

    return 0;
}
