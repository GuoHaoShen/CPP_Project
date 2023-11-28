//
// Created by ShenGuoHao on 2023/11/27.
//

#include "matplotlibcpp.h"
#include <vector>
#include <cmath>
#include <random>
#include <chrono>

namespace plt = matplotlibcpp;
using namespace std;
 
/**
 * @brief rosenbrock函数
 *
 * @param x vector<double>类型 为函数自变量，向量长度为自变量数量（函数维数）
 * @result 函数值
 */
double rosenbrockFunc(const vector<double> x){
    int num = x.size()/2;
    double res = 0.0;
    for (int i = 0; i < num; i++)
    {
        res += 100 * pow(pow(x[2*i], 2) - x[2*i+1], 2) + pow(x[2*i] - 1 , 2);
    }
    return res;
}

/**
 * @brief 求rosenbrock函数的梯度
 *
 * @param x vector<double>类型 为函数自变量，向量长度为自变量数量（函数维数）
 * @result 梯度
 */
vector<double> rosenbrockGradient(const vector<double> x){
    int num = x.size();
    vector<double> res;

    double der1 = -400*(x[1] - pow(x[0], 2)) - 2*(1 - x[0]);
    res.push_back(der1);

    for (int i = 1; i < num-1; i++)
    {
        double derm = -400*(x[i+1] - pow(x[i], 2)) * x[i] + 200*(x[i] - pow(x[i-1], 2)) - 2*(1 - x[i]);
        res.push_back(derm);
    }
    
    double dern = 200 * (x[num-1] - pow(x[num-2], 2));
    res.push_back(dern);
    return res;
}

/**
 * @brief 向量数乘 
*/
vector<double> multipliVector(const vector<double> x, double t)
{
    vector<double> res;
    for(int i = 0; i < x.size(); ++i)
    {
        res.push_back(x[i] * t);
    }
    return res;
}

/**
 * @brief 向量的和
*/
vector<double> addVector(const vector<double> x1, const vector<double> x2)
{
    vector<double> res;
    for(int i = 0; i < x1.size(); ++i){
        res.push_back(x1[i] + x2[i]); 
    }
    return res;
}

/**
 * @brief 向量点积
*/
double proVector(const vector<double> x1, const vector<double> x2)
{
    double res = 0.0;
    for(int i = 0; i < x1.size(); ++i){
        res += x1[i] * x2[i]; 
    }
    return res;
}

/**
 * @brief Armijo条件
 *
 * @param x vector<double>类型 为函数自变量，向量长度为自变量数量（函数维数）
 * @param grad vector<double>类型 函数梯度
 * @result 符合armijo准则的步长
 */
double armjio(const vector<double> x, const vector<double> grad){
    double alpha = 1.0;
    double c = 0.02;      // 0~1的一个参数
    double beta = 0.4;

    vector<double> d = multipliVector(grad, -1);

    while (rosenbrockFunc(addVector(x, multipliVector(d, alpha))) > rosenbrockFunc(x) + proVector(d, multipliVector(grad, c*alpha)))
    {
        alpha *= beta;
    }
    return alpha;
}

/**
 * @brief 生成随机数vector
 *
 * @param size 需要的vector长度
 * @param min 随机数下限，默认为0
 * @param max 随机数上限，默认为1
 * @result std::vector<double>类型随机vector 
 */
std::vector<double> generateRandomVector(int size, double min=0.0, double max=1.0) {
    std::vector<double> randomVector;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);

    for (int i = 0; i < size; ++i) {
        randomVector.push_back(dis(gen));
    }

    return randomVector;
}

/**
 * @brief 用于可视化的函数
 */
double func_rosenbrock(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z, double i, double j){
    double a = 1;
    double b = 100;

    x.push_back(i);
    y.push_back(j);
    z.push_back(pow(a-i, 2) + b * pow(j - pow(i, 2), 2) + 100);
}

int main(){
    std::vector<double> x0 = generateRandomVector(2, 0, 2);       //初始化函数值
    // std::vector<double> x0 = {1.3, 1.2, 0.8};

    double error_range = 1e-4;      // 梯度阈值，小于该值认为已经完成
    double interation_num = 0;          // 记录迭代次数
    double interation_max = 5000;              

    vector<double> grad = rosenbrockGradient(x0);


    // matplotlib可视化
    std::vector<double> lineX;
    std::vector<double> lineY;

    cout << "-----------------------------" << endl;
    cout <<"初始x0值：";
        for(int i = 0; i < x0.size(); ++i) cout << x0[i] << " " ; cout << endl;
    cout << "初始函数值: " << rosenbrockFunc(x0) << endl;
    cout <<"初始梯度值：";
        for(int i = 0; i < x0.size(); ++i) cout << grad[i] << " " ; cout << endl;

    // 获取迭代开始执行的时间点
    auto start = std::chrono::high_resolution_clock::now();

    while (sqrt(proVector(grad, grad)) > error_range)
    {
        double alpha = armjio(x0, grad);                    // 使用armijo准则计算步长
        vector<double> d = multipliVector(grad, -1);        // 计算负梯度
        // x0 = addVector(x0, multipliVector(d, alpha));       // 更新x0
        x0 = addVector(x0, multipliVector(d, 0.0001));       // 更新x0
        grad = rosenbrockGradient(x0);                      // 更新梯度

        interation_num++;
        if (interation_num > interation_max)
        {
            cout << "超出迭代次数，退出迭代  interation_num = " << interation_num << endl;
            break;
        }

        // matplotlib添加额外的线
        lineX.push_back(x0[0]);
        lineY.push_back(x0[1]);

        cout << "-----------------------------" << endl;
        cout << "alpha值：" << alpha << endl;
        cout <<"负梯度值：" ;
             for(int i = 0; i < d.size(); ++i) cout << d[i] << " " ; cout << endl;
        cout <<"迭代x0值：" ;  
            for(int i = 0; i < x0.size(); ++i) cout << x0[i] << " " ; cout << endl;
        cout <<"梯度值：";
            for(int i = 0; i < grad.size(); ++i) cout << grad[i] << " " ; cout << endl;
    }

    // 获取程序结束执行的时间点
    auto end = std::chrono::high_resolution_clock::now();

    // 计算程序执行时间并输出
    std::chrono::duration<double> duration = end - start;

    cout << "-----------------------------" << endl;
    std::cout << "迭代执行时间: " << duration.count() << " 秒" << std::endl;
    cout << "迭代次数: " << interation_num << endl;
    cout << "迭代精度: " << sqrt(proVector(grad, grad)) << endl;
    cout << "令function最小的x0值：";
    for(int i = 0; i < x0.size(); ++i) cout << x0[i] << " ";
    cout << endl;
    cout << "最小值: " << rosenbrockFunc(x0) << endl;

    //绘制rosenbrock函数
    std::vector<std::vector<double>> contourX, contourY, contourZ;
    for (double i = 0; i <= 2;  i += 0.1) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = 0; j <= 2; j += 0.1) {
            func_rosenbrock(x_row, y_row, z_row, i, j);
        }
        contourX.push_back(x_row);
        contourY.push_back(y_row);
        contourZ.push_back(z_row);
    }
    plt::plot_surface(contourX, contourY, contourZ);
    plt::plot(lineX, lineY, "r-4");  // 添加红色线
    // cout <<"lineX：" ;  
    //     for(int i = 0; i < lineX.size(); ++i) cout << lineX[i] << " " ; cout << endl;
    // cout <<"lineY：";
    //     for(int i = 0; i < lineY.size(); ++i) cout << lineY[i] << " " ; cout << endl;
    plt::show();
    
}