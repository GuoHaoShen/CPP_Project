#include <matplotlibcpp.h>
#include <cmath>

namespace plt = matplotlibcpp;

// rosenbrock函数
double func_rosenbrock(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j){
    double a = 1;
    double b = 100;

    x.push_back(i);
    y.push_back(j);
    z.push_back(pow(a-i, 2) + b * pow(j - pow(i, 2), 2));
}

int main() {

    //绘制rosenbrock函数
    std::vector<std::vector<double>> x1, y1, z1;
    for (double i = -5; i <= 5;  i += 1) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -5; j <= 5; j += 1) {
            func_rosenbrock(x_row, y_row, z_row, i, j);
        }
        x1.push_back(x_row);
        y1.push_back(y_row);
        z1.push_back(z_row);
    }

    plt::contour(x1, y1, z1);

    // 添加额外的线
    std::vector<double> lineX = {1.0, 2.0, 3.0};
    std::vector<double> lineY = {1.0, 2.0, 3.0};
    plt::plot(lineX, lineY, "r-");  // 添加红色线

    // 添加额外的点
    std::vector<double> pointX = {2.0};
    std::vector<double> pointY = {2.0};
    plt::scatter(pointX, pointY, 50, {{"color", "g"}});  // 添加绿色圆点

    plt::show();

    return 0;
}
