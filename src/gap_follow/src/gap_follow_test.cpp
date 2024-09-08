#include <gtest/gtest.h>

#include <gap_follow/gap_follow.hpp>  // assuming the function is in this header file
using namespace gap_follow;
TEST(GapFollowTest, EmptyRanges) {
  GapFollow gap_follow;
  std::vector<float> ranges;
  std::pair<int, int> gap;
  int drivable_angle_idx = 0;

  EXPECT_THROW(gap_follow.find_max_gap(ranges, gap, drivable_angle_idx),
               std::invalid_argument);
}

TEST(GapFollowTest, DrivableAngleIdxOutOfRange) {
  GapFollow gap_follow;
  std::vector<float> ranges = {1.0, 2.0, 3.0};
  std::pair<int, int> gap;
  int drivable_angle_idx = 2;

  EXPECT_THROW(gap_follow.find_max_gap(ranges, gap, drivable_angle_idx),
               std::invalid_argument);
}

TEST(GapFollowTest, SingleElementRange) {
  GapFollow gap_follow;
  std::vector<float> ranges = {1.0};
  std::pair<int, int> gap;
  int drivable_angle_idx = 0;

  gap_follow.find_max_gap(ranges, gap, drivable_angle_idx);
  EXPECT_EQ(gap.first, 0);
  EXPECT_EQ(gap.second, 0);
}

TEST(GapFollowTest, MultipleElementsRangeMaxGapAtBeginning) {
  GapFollow gap_follow;
  std::vector<float> ranges = {9.0, 0, 1.0, 2.0, 3.0, 0, 1.4, 4.4};
  std::pair<int, int> gap;
  int drivable_angle_idx = 0;

  gap_follow.find_max_gap(ranges, gap, drivable_angle_idx);
  EXPECT_EQ(gap.first, 2);
  EXPECT_EQ(gap.second, 5);
}

TEST(GapFollowTest, MultipleElementsRangeMaxGapAtEnd) {
  GapFollow gap_follow;
  std::vector<float> ranges = {1.0, 2.0, 3.0, 0.5};
  std::pair<int, int> gap;
  int drivable_angle_idx = 0;

  gap_follow.find_max_gap(ranges, gap, drivable_angle_idx);
  EXPECT_EQ(gap.first, 0);
  EXPECT_EQ(gap.second, 3);
}

TEST(GapFollowTest, MultipleElementsRangeMaxGapInMiddle) {
  GapFollow gap_follow;
  std::vector<float> ranges = {0, 0.5, 2.0, 0, 3.0};
  std::pair<int, int> gap;
  int drivable_angle_idx = 0;

  gap_follow.find_max_gap(ranges, gap, drivable_angle_idx);
  EXPECT_EQ(gap.first, 1);
  EXPECT_EQ(gap.second, 3);
}

TEST(GapFollowTest, FindBestPointSingleMid) {
  GapFollow gap_follow;
  std::vector<float> rdata = {1.0, 2.0, 2.04, 1.95, 5.0};
  std::pair<int, int> gap = {1, 4};

  int best_point = gap_follow.find_best_point(rdata, gap);

  EXPECT_EQ(best_point, 2);
}
TEST(GapFollowTest, FindBestPointDoubleMid) {
  GapFollow gap_follow;
  std::vector<float> rdata = {1.0, 2.0, 2.04, 1.95, 1.0,  1.1,
                              2.0, 2.0, 2.01, 2.03, 2.02, 1.0};
  std::pair<int, int> gap = {1, 11};

  int best_point = gap_follow.find_best_point(rdata, gap);

  EXPECT_EQ(best_point, 8);
}
TEST(GapFollowTest, FindBestPointDoubleMidFlipped) {
  GapFollow gap_follow;
  std::vector<float> rdata = {1.0, 2.0, 2.0, 2.01, 2.03, 2.02,
                              1.0, 1.1, 2.0, 2.04, 1.95, 1.0};
  std::pair<int, int> gap = {1, 11};

  int best_point = gap_follow.find_best_point(rdata, gap);

  EXPECT_EQ(best_point, 3);
}
TEST(GapFollowTest, FindBestPoint) {
  GapFollow gap_follow;
  std::vector<float> rdata = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::pair<int, int> gap = {1, 4};

  int best_point = gap_follow.find_best_point(rdata, gap);

  EXPECT_EQ(best_point, 3);
}

TEST(GapFollowTest, FindBestPoint_EmptyRdata) {
  GapFollow gap_follow;
  std::vector<float> rdata;
  std::pair<int, int> gap = {1, 4};

  EXPECT_THROW(gap_follow.find_best_point(rdata, gap), std::invalid_argument);
}

TEST(GapFollowTest, FindBestPoint_GapIndicesOutOfRange) {
  GapFollow gap_follow;
  std::vector<float> rdata = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::pair<int, int> gap = {6, 7};

  EXPECT_THROW(gap_follow.find_best_point(rdata, gap), std::invalid_argument);
}

TEST(GapFollowTest, FindBestPoint_MultipleMaxValues) {
  GapFollow gap_follow;
  std::vector<float> rdata = {1.0, 5.0, 3.0, 5.0, 2.0};
  std::pair<int, int> gap = {1, 4};

  int best_point = gap_follow.find_best_point(rdata, gap);

  // Either 1 or 3 could be the best point, so we just check that it's one of
  // them
  EXPECT_TRUE(best_point == 1 || best_point == 3);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}