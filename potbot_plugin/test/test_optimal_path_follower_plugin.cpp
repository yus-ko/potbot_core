/**
 * @file test_optimal_path_follower_plugin.cpp
 * @brief potbot_nav::controller::OptimalPathFollower クラスのユニットテスト
 *
 * Nav2 Controller プラグインとして実装された OptimalPathFollower のテスト。
 * configure() は Costmap2DROS を必要とするためフルテストは困難であり、
 * オブジェクト生成・型確認・configure前の安全性を検証する。
 *
 * 注意: setPlan() / cleanup() / activate() / deactivate() は
 * configure() 後に生成される global_pub_ を使用するため、
 * configure 前に呼び出すとクラッシュする。これらは対象外とする。
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "potbot_plugin/optimal_path_follower.hpp"

// ============================================================
// テストフィクスチャ
// ============================================================

class OptimalPathFollowerPluginTest : public ::testing::Test
{
public:
  // rclcpp の初期化・終了はテストスイート単位で一度だけ行う
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// ============================================================
// テスト 1: デフォルトコンストラクタが例外なく成功する
// ============================================================

TEST_F(OptimalPathFollowerPluginTest, ConstructorNoThrow)
{
  // OptimalPathFollower オブジェクトをデフォルト構築できることを確認する
  EXPECT_NO_THROW({
    potbot_nav::controller::OptimalPathFollower controller;
  });
}

// ============================================================
// テスト 2: デフォルト構築後のオブジェクトが有効なインスタンスである
// ============================================================

TEST_F(OptimalPathFollowerPluginTest, InstanceIsValid)
{
  // ヒープ上でのオブジェクト生成が成功し、nullptr でないことを確認する
  auto controller = std::make_unique<potbot_nav::controller::OptimalPathFollower>();
  EXPECT_NE(controller, nullptr);
}

// ============================================================
// テスト 3: Controller インターフェースを継承していることの型確認
// ============================================================

TEST_F(OptimalPathFollowerPluginTest, InheritsControllerInterface)
{
  // OptimalPathFollower が nav2_core::Controller の派生クラスであることを確認する
  // dynamic_cast が成功すれば継承関係が正しい
  auto controller = std::make_shared<potbot_nav::controller::OptimalPathFollower>();
  auto base_ptr = std::dynamic_pointer_cast<nav2_core::Controller>(controller);
  EXPECT_NE(base_ptr, nullptr);
}

// ============================================================
// テスト 4: setSpeedLimit(絶対値) が configure前でもクラッシュしない
// ============================================================

TEST_F(OptimalPathFollowerPluginTest, SetSpeedLimitAbsolute)
{
  // setSpeedLimit() の実装は引数を無視する no-op であるため
  // configure 前でも安全に呼び出せることを確認する
  potbot_nav::controller::OptimalPathFollower controller;
  EXPECT_NO_THROW({
    controller.setSpeedLimit(0.5, false);
  });
}

// ============================================================
// テスト 5: setSpeedLimit(パーセント) が configure前でもクラッシュしない
// ============================================================

TEST_F(OptimalPathFollowerPluginTest, SetSpeedLimitPercentage)
{
  // setSpeedLimit() は percentage フラグを受け取るが、実装は no-op であるため
  // configure 前でも安全に呼び出せることを確認する
  potbot_nav::controller::OptimalPathFollower controller;
  EXPECT_NO_THROW({
    controller.setSpeedLimit(50.0, true);
  });
}

// ============================================================
// テスト 6: 複数インスタンスを同時に生成できる
// ============================================================

TEST_F(OptimalPathFollowerPluginTest, MultipleInstancesCanBeCreated)
{
  // 複数の OptimalPathFollower オブジェクトを同時に生成しても干渉しないことを確認する
  EXPECT_NO_THROW({
    potbot_nav::controller::OptimalPathFollower controller1;
    potbot_nav::controller::OptimalPathFollower controller2;
    potbot_nav::controller::OptimalPathFollower controller3;
  });
}

// ============================================================
// メインエントリポイント
// ============================================================

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
