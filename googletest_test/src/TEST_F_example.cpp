#include <gtest/gtest.h>

// 定义一个测试夹具
class MyTestFixture : public ::testing::Test {
protected:
    void SetUp() override {
        // 在每个测试用例运行前执行的初始化代码
    }

    void TearDown() override {
        // 在每个测试用例运行后执行的清理代码
    }

    int sharedValue; // 夹具中的成员变量
};

// 使用TEST_F宏定义测试用例，并指定测试夹具的名称
TEST_F(MyTestFixture, Test1) {
    // 在测试用例中可以使用夹具中的成员变量和方法
    sharedValue = 100;
    EXPECT_EQ(sharedValue, 100);
}

TEST_F(MyTestFixture, Test2) {
    // 另一个测试用例也可以使用夹具中的成员变量和方法
    sharedValue = 200;
    EXPECT_EQ(sharedValue, 200);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
