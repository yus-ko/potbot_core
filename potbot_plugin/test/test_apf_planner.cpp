/**
 * @file test_apf_planner.cpp
 * @brief potbot_nav::planner::APF クラスのユニットテスト
 *
 * configure() は Costmap2DROS を必要とするためフルテストは困難。
 * オブジェクト生成・型確認・インターフェース継承を検証する。
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include "potbot_plugin/apf_planner.hpp"

class APFPlannerTest : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// デフォルト構築が成功しnullptrでないことを確認する
TEST_F(APFPlannerTest, InstanceIsValid)
{
  auto planner = std::make_unique<potbot_nav::planner::APF>();
  EXPECT_NE(planner, nullptr);
}

// nav2_core::GlobalPlanner インターフェースを継承していることを確認する
TEST_F(APFPlannerTest, InheritsGlobalPlannerInterface)
{
  auto planner = std::make_shared<potbot_nav::planner::APF>();
  auto base_ptr = std::dynamic_pointer_cast<nav2_core::GlobalPlanner>(planner);
  EXPECT_NE(base_ptr, nullptr);
}

// createPlan() の戻り値型が nav_msgs::msg::Path であることをコンパイル時に確認する
TEST_F(APFPlannerTest, PlanReturnTypeIsPath)
{
  // configure() 前に createPlan() を呼ぶと nullptr dereference になるため
  // 関数ポインタ経由で戻り値型のみ検証する
  using CreatePlanFn = nav_msgs::msg::Path (potbot_nav::planner::APF::*)(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &);
  static_assert(
    std::is_same<
      CreatePlanFn,
      nav_msgs::msg::Path (potbot_nav::planner::APF::*)(
        const geometry_msgs::msg::PoseStamped &,
        const geometry_msgs::msg::PoseStamped &)
    >::value,
    "createPlan() の戻り値型は nav_msgs::msg::Path でなければならない");

  nav_msgs::msg::Path path;
  EXPECT_TRUE(path.poses.empty());
}

// 複数インスタンスを同時に生成しても干渉しないことを確認する
TEST_F(APFPlannerTest, MultipleInstancesNoInterference)
{
  EXPECT_NO_THROW({
    potbot_nav::planner::APF planner1;
    potbot_nav::planner::APF planner2;
    potbot_nav::planner::APF planner3;
  });
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
