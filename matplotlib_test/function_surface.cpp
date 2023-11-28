//
// Created by ShenGuoHao on 2023/11/27.
//


#include <matplotlibcpp.h>
#include <cmath>

namespace plt = matplotlibcpp;

// z = sin(sqrt(x^2 + y^2))
void func(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j) {
    x.push_back(i);
    y.push_back(j);
    z.push_back(::std::sin(::std::hypot(i, j)));
}

// 椭球面
void func_ellipsoid(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j){
    double a = 1.0; // x轴半轴长度
    double b = 2.0; // y轴半轴长度
    double c = 3.0; // z轴半轴长度
    double theta = 2 * M_PI * i / 100;
    double phi = M_PI * j /100;
    x.push_back(a * sin(phi) * cos(theta));
    y.push_back(b * sin(phi) * sin(theta));
    z.push_back(c * cos(phi));
}

// 单页双曲面
void func_hyperboloid(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j){
    double a = 1.0; // x轴半轴长度
    double b = 1.0; // y轴半轴长度
    double c = 1.0; // z轴半轴长度  
    double u = 1.0 * i / 100;
    double v = 2 * M_PI * j / 100;
    x.push_back(a * cosh(u) * cos(v));
    y.push_back(b * cosh(u) * sin(v));
    z.push_back(c * sinh(u));
}

// 双曲抛物面
void func_paraboloids(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j){
    double a = 1.0;
    double b = 1.0;
    x.push_back(i);
    y.push_back(j);
    z.push_back((i*i)/(a*a) - (j*j)/(b*b));
}

// rosenbrock函数
double func_rosenbrock(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j){
    double a = 1;
    double b = 100;

    x.push_back(i);
    y.push_back(j);
    z.push_back(pow(a-i, 2) + b * pow(j - pow(i, 2), 2));
}


int main() {

    // //绘制复杂函数
    // std::vector<std::vector<double>> x1, y1, z1;
    // for (double i = -5; i <= 5;  i += 0.25) {
    //     std::vector<double> x_row, y_row, z_row;
    //     for (double j = -5; j <= 5; j += 0.25) {
    //         func(x_row, y_row, z_row, i, j);
    //     }
    //     x1.push_back(x_row);
    //     y1.push_back(y_row);
    //     z1.push_back(z_row);
    // }
    // plt::plot_surface(x1, y1, z1);

    // // 绘制椭球面
    // std::vector<std::vector<double>> x2, y2, z2;
    // for (double i = 0; i <= 100;  i ++) {
    //     std::vector<double> x_row, y_row, z_row;
    //     for (double j = 0; j <= 100; j ++) {
    //         func_ellipsoid(x_row, y_row, z_row, i, j);
    //     }
    //     x2.push_back(x_row);
    //     y2.push_back(y_row);
    //     z2.push_back(z_row);
    // }
    // plt::plot_surface(x2, y2, z2);

    // // 绘制单页双曲面
    // std::vector<std::vector<double>> x3, y3, z3;
    // for (double i = 0; i <= 100;  i ++) {
    //     std::vector<double> x_row, y_row, z_row;
    //     for (double j = 0; j <= 100; j ++) {
    //         func_hyperboloid(x_row, y_row, z_row, i, j);
    //     }
    //     x3.push_back(x_row);
    //     y3.push_back(y_row);
    //     z3.push_back(z_row);
    // }
    // plt::plot_surface(x3, y3, z3);

    // //绘制双曲抛物面
    // std::vector<std::vector<double>> x4, y4, z4;
    // for (double i = -5; i <= 5;  i += 0.25) {
    //     std::vector<double> x_row, y_row, z_row;
    //     for (double j = -5; j <= 5; j += 0.25) {
    //         func_paraboloids(x_row, y_row, z_row, i, j);
    //     }
    //     x4.push_back(x_row);
    //     y4.push_back(y_row);
    //     z4.push_back(z_row);
    // }
    // plt::plot_surface(x4, y4, z4);

    //绘制rosenbrock函数
    std::vector<std::vector<double>> x5, y5, z5;
    for (double i = -5; i <= 5;  i += 0.25) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -5; j <= 5; j += 0.25) {
            func_rosenbrock(x_row, y_row, z_row, i, j);
        }
        x5.push_back(x_row);
        y5.push_back(y_row);
        z5.push_back(z_row);
    }
    plt::plot_surface(x5, y5, z5);

    // 添加额外的线
    std::vector<double> lineX = {1.0, 5.0, 2.0};
    std::vector<double> lineY = {1.0, 2.0, 3.0};
    plt::plot(lineX, lineY, "r-");  // 添加红色线

    std::vector<double> x = {1.0, 2.0, 3.0};
    std::vector<double> y = {2.0, 3.0, 4.0};
    std::vector<double> z = {3.0, 4.0, 5.0};

    plt::plot3(x, y, z);  // 画一条红色的3维线

    // 添加额外的点
    std::vector<double> pointX = {2.0};
    std::vector<double> pointY = {2.0};
    plt::scatter(pointX, pointY, 50, {{"color", "g"}});  // 添加绿色圆点

    plt::show();
}
