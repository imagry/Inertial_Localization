#include <gtest/gtest.h>

TEST(SampleTest, BasicAssertions) {
  // Expect equality
  EXPECT_EQ(2 + 2, 4);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
