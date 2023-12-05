#include <gtest/gtest.h>
#include <gtest/gtest-param-test.h>


int add(int a, int b) {
  return a + b;
}

// 定义一个参数化测试类
class AddParameterizedTest : public testing::TestWithParam<std::tuple<int, int, int>> {
};

// 使用TEST_P宏定义参数化测试
TEST_P(AddParameterizedTest, AddTest) {
  int a = std::get<0>(GetParam());
  int b = std::get<1>(GetParam());
  int expected_sum = std::get<2>(GetParam());
  int result = add(a, b);
  ASSERT_EQ(result, expected_sum);
}

// 使用INSTANTIATE_TEST_CASE_P宏实例化参数化测试
INSTANTIATE_TEST_CASE_P(AddTestInstantiation, AddParameterizedTest, testing::Values(
    std::make_tuple(1, 1, 2),
    std::make_tuple(2, 3, 5),
    std::make_tuple(-1, 1, 0)
));
