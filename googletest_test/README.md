GoogleTest
------
Google Test（简称gtest）是Google开发的一个C++单元测试框架，用于编写和运行C++的单元测试。它提供了丰富的断言（assertion）来验证代码的行为，支持测试夹具（test fixtures）和参数化测试（parameterized tests），并且能够生成详细的测试报告。

本模块为学习gtest过程中使用的示例代码

文件结构
----------

.
├── CMakeLists.txt
├── README.md
├── build/                          # 编译文件
└── example.cpp            

生成报告解析
----------
在运行后gtest会生成报告
以下为example中报告的解析
```
[==========] Running 2 tests from 2 test suites.：这表示一共有2个测试套件中的2个测试被运行。
[----------] Global test environment set-up.：这表示全局测试环境的设置已经完成。
[----------] 1 test from AddTest：这表示接下来是来自名为AddTest的测试套件中的1个测试。
[ RUN      ] AddTest.PositiveNumbers：这表示正在运行名为PositiveNumbers的测试用例。
[       OK ] AddTest.PositiveNumbers (0 ms)：这表示PositiveNumbers测试用例成功通过了，并且用时0毫秒。
[----------] 1 test from AddTest (0 ms total)：这表示AddTest测试套件中的测试已经全部运行完毧，总共用时0毫秒。
[----------] 1 test from AddTest1：这表示接下来是来自名为AddTest1的测试套件中的1个测试。
[ RUN      ] AddTest1.PositiveNumbers：这表示正在运行名为PositiveNumbers的测试用例。
            /mnt/d/Project/CPP_Project/googletest_test/example.cpp:12: Failure：这表示测试用例在example.cpp文件的第12行出现了失败。
            Expected equality of these values:：这表示接下来是期望的值。
            add(3, 3)：这表示调用add函数传入参数3和3。
            Which is: 6：这表示add函数返回的结果是6。
            4：这表示实际的期望值是4。
[  FAILED  ] AddTest1.PositiveNumbers (0 ms)：这表示PositiveNumbers测试用例运行失败，并且用时0毫秒。
[----------] 1 test from AddTest1 (0 ms total)：这表示AddTest1测试套件中的测试已经全部运行完毧，总共用时0毫秒。
[----------] Global ... test environment tear-down：这表示全局测试环境的清理已经完成。
[==========] 2 tests from 2 test suites ran. (0 ms total)：这表示一共有2个测试套件中的2个测试被运行，总共用时0毫秒。
[  PASSED  ] 1 test.：这表示有1个测试通过了。
[  FAILED  ] 1 test, listed below:：这表示有1个测试失败了。
1 FAILED TEST：这表示一共有1个测试失败了。

```

常用方法
----------
* EXPECT_EQ
    
    用于验证两个值是否相等。如果两个值相等，测试将通过；如果不相等，测试将失败并显示实际值和期望值。
    ```
    //基本语法
    EXPECT_EQ(expected, actual);
    ```
    其中，expected是你期望的值，而actual是实际的值。这个宏会比较expected和actual，如果它们相等，测试通过；如果不相等，测试失败并显示详细信息。

* EXPECT_NE: 
  
    用于验证两个值不相等。
    ```
    EXPECT_NE(value1, value2);
    ```
* EXPECT_TRUE / EXPECT_FALSE: 
  
    用于验证条件是否为真或为假。
    ```
    EXPECT_TRUE(condition);
    EXPECT_FALSE(condition);
    ```
* ASSERT_EQ / ASSERT_NE / ASSERT_TRUE / ASSERT_FALSE: 
    
    与上述的EXPECT宏类似，不同之处在于如果断言失败，ASSERT宏将导致测试立即终止，而EXPECT宏只会导致当前测试失败。

* 参数化测试

    参数化测试是一种在单元测试中使用不同输入参数运行相同测试逻辑的方法。这种方法可以帮助你减少重复的测试代码，同时覆盖多种输入情况，提高测试的全面性。
    在gtest中，参数化测试可以通过TEST_P宏来实现。

* Death Tests

    死亡测试（Death Tests）是一种用于验证代码是否会导致程序终止的测试方法。它用于验证代码是否会抛出异常、调用exit或abort等导致程序终止的行为。在gtest中，可以使用EXPECT_DEATH和ASSERT_DEATH宏来编写死亡测试。

参考资料
----------
ChatGpt3.5