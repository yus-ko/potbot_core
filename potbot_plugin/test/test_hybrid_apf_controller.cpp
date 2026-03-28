// Copyright 2024 potbot
/**
 * @file test_hybrid_apf_controller.cpp
 * @brief potbot_nav::controller::HybridApfController クラスのユニットテスト
 *
 * Nav2 Controller プラグインとして実装された HybridApfController のテスト。
 * configure() は Costmap2DROS を必要とするためフルテストは困難であり、
 * オブジェクト生成・型確認・configure前の安全性を検証する。
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "potbot_plugin/hybrid_apf_controller.hpp"

// ============================================================
// テストフィクスチャ
// ============================================================

class HybridApfControllerTest : public ::testing::Test
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

TEST_F(HybridApfControllerTest, Constructor)
{
    EXPECT_NO_THROW(
    {
        potbot_nav::controller::HybridApfController c;
    });
}

// ============================================================
// テスト 2: ヒープ上でのオブジェクト生成が成功し nullptr でない
// ============================================================

TEST_F(HybridApfControllerTest, InstanceIsValid)
{
    auto c = std::make_unique<potbot_nav::controller::HybridApfController>();
    EXPECT_NE(c, nullptr);
}

// ============================================================
// テスト 3: nav2_core::Controller にキャストできること
// ============================================================

TEST_F(HybridApfControllerTest, IsControllerInterface)
{
    potbot_nav::controller::HybridApfController c;
    nav2_core::Controller* p = &c;
    EXPECT_NE(p, nullptr);
}

// ============================================================
// テスト 4: setSpeedLimit() が configure前でもクラッシュしない
// ============================================================

TEST_F(HybridApfControllerTest, SetSpeedLimitNoThrow)
{
    potbot_nav::controller::HybridApfController c;
    EXPECT_NO_THROW(c.setSpeedLimit(0.5, false));
    EXPECT_NO_THROW(c.setSpeedLimit(50.0, true));
}

// ============================================================
// テスト 5: 複数インスタンスを同時に生成できる
// ============================================================

TEST_F(HybridApfControllerTest, MultipleInstancesCanBeCreated)
{
    EXPECT_NO_THROW(
    {
        potbot_nav::controller::HybridApfController c1;
        potbot_nav::controller::HybridApfController c2;
        potbot_nav::controller::HybridApfController c3;
    });
}

// ============================================================
// テスト 6: shared_ptr でのキャストが成功する
// ============================================================

TEST_F(HybridApfControllerTest, SharedPtrCastToController)
{
    auto ctrl = std::make_shared<potbot_nav::controller::HybridApfController>();
    auto base = std::dynamic_pointer_cast<nav2_core::Controller>(ctrl);
    EXPECT_NE(base, nullptr);
}

// ============================================================
// メインエントリポイント
// ============================================================

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
