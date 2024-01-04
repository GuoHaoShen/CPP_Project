#include <gtest/gtest.h>

// 定义一个名为MyTest的测试用例
TEST(MyTest, FirstTest) {
    // 在测试用例中编写测试点
    int result = 2 + 2;
    // 使用gtest的断言来验证测试点
    ASSERT_EQ(result, 4);
}

TEST(MyTest, SecondTest) {
    // 另一个测试点
    bool condition = true;
    ASSERT_TRUE(condition);
}

int main(int argc, char **argv) {
    // 初始化gtest
    ::testing::InitGoogleTest(&argc, argv);
    // 运行所有的测试用例
    return RUN_ALL_TESTS();
}
