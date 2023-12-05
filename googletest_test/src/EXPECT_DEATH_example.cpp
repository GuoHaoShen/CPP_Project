#include <gtest/gtest.h>

void myFunction() {
    // 假设这个函数会导致程序终止
    // 例如调用exit或抛出异常
    // 这里我们假设调用exit(1)
    throw std::runtime_error("抛出一个错误");
    exit(1);
}

// 使用EXPECT_DEATH宏定义死亡测试
TEST(MyDeathTest, Test) {
    EXPECT_DEATH(myFunction(), "");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
