//
// Created by ShenGuoHao on 2023/11/27.
//

#include "matplotlibcpp.h"
#include <vector>
#include <cmath>

using namespace std;

/**
 * @brief rosenbrock函数
 *
 * @param x vector<double>类型 为函数自变量，向量长度为自变量数量（函数维数）
 * @result 函数值
 */
double rosenbrockFunc(const vector<double> x){
    size_t num = x.size()/2;
    double res = 0.0;
    for (size_t i = 0; i < num; i++)
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
    size_t num = x.size()/2;
    vector<double> res;
    for (size_t i = 0; i < num; i++)
    {
        double per1 = 400 * (pow(x[2*i], 3) - x[2*i]*x[2*i+1]) + 2 * (x[2*i] - 1);
        double per2 = -200 * (pow(x[2*i], 2) - pow(x[2*i+1], 2));
        res.push_back(per1);
        res.push_back(per2);
    }
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
 * @param grad 函数梯度
 * @result 
 */
double armjio(const vector<double> x, const vector<double> grad){

}