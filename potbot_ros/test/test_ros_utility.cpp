#include <gtest/gtest.h>
#include <cmath>
#include <potbot_ros/utility.hpp>

// ============================================================
// get_point(double x, y, z) テスト
// ============================================================

TEST(ROSUtilityTest, GetPointFromXYZ)
{
    auto pt = potbot_lib::utility::get_point(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(pt.x, 1.0);
    EXPECT_DOUBLE_EQ(pt.y, 2.0);
    EXPECT_DOUBLE_EQ(pt.z, 3.0);
}

TEST(ROSUtilityTest, GetPointDefaultZero)
{
    auto pt = potbot_lib::utility::get_point();
    EXPECT_DOUBLE_EQ(pt.x, 0.0);
    EXPECT_DOUBLE_EQ(pt.y, 0.0);
    EXPECT_DOUBLE_EQ(pt.z, 0.0);
}

// ============================================================
// get_point(const Point& p) / get_point(const geometry_msgs::msg::Point& p) 往復テスト
// ============================================================

TEST(ROSUtilityTest, GetPointFromPotbotPoint)
{
    potbot_lib::Point p(1.5, 2.5, 3.5);
    auto msg_pt = potbot_lib::utility::get_point(p);
    EXPECT_DOUBLE_EQ(msg_pt.x, 1.5);
    EXPECT_DOUBLE_EQ(msg_pt.y, 2.5);
    EXPECT_DOUBLE_EQ(msg_pt.z, 3.5);
}

TEST(ROSUtilityTest, GetPotbotPointFromMsgPoint)
{
    geometry_msgs::msg::Point msg_pt;
    msg_pt.x = 4.0;
    msg_pt.y = 5.0;
    msg_pt.z = 6.0;
    potbot_lib::Point p = potbot_lib::utility::get_point(msg_pt);
    EXPECT_DOUBLE_EQ(p.x, 4.0);
    EXPECT_DOUBLE_EQ(p.y, 5.0);
    EXPECT_DOUBLE_EQ(p.z, 6.0);
}

TEST(ROSUtilityTest, GetPointRoundTrip)
{
    potbot_lib::Point original(7.0, 8.0, 9.0);
    auto msg_pt = potbot_lib::utility::get_point(original);
    potbot_lib::Point restored = potbot_lib::utility::get_point(msg_pt);
    EXPECT_DOUBLE_EQ(restored.x, original.x);
    EXPECT_DOUBLE_EQ(restored.y, original.y);
    EXPECT_DOUBLE_EQ(restored.z, original.z);
}

// ============================================================
// get_quat / get_rpy 往復テスト
// ============================================================

TEST(ROSUtilityTest, GetQuatFromRPY)
{
    auto q = potbot_lib::utility::get_quat(0.0, 0.0, 0.0);
    // ゼロ角の場合、w=1, x=y=z=0
    EXPECT_NEAR(q.w, 1.0, 1e-9);
    EXPECT_NEAR(q.x, 0.0, 1e-9);
    EXPECT_NEAR(q.y, 0.0, 1e-9);
    EXPECT_NEAR(q.z, 0.0, 1e-9);
}

TEST(ROSUtilityTest, GetRPYRoundTrip)
{
    double roll_in = 0.1, pitch_in = 0.2, yaw_in = 0.3;
    auto q = potbot_lib::utility::get_quat(roll_in, pitch_in, yaw_in);
    double roll_out, pitch_out, yaw_out;
    potbot_lib::utility::get_rpy(q, roll_out, pitch_out, yaw_out);
    EXPECT_NEAR(roll_out, roll_in, 1e-9);
    EXPECT_NEAR(pitch_out, pitch_in, 1e-9);
    EXPECT_NEAR(yaw_out, yaw_in, 1e-9);
}

TEST(ROSUtilityTest, GetRPYYawOnly)
{
    double yaw_in = M_PI / 4.0;
    auto q = potbot_lib::utility::get_quat(0.0, 0.0, yaw_in);
    double roll_out, pitch_out, yaw_out;
    potbot_lib::utility::get_rpy(q, roll_out, pitch_out, yaw_out);
    EXPECT_NEAR(roll_out, 0.0, 1e-9);
    EXPECT_NEAR(pitch_out, 0.0, 1e-9);
    EXPECT_NEAR(yaw_out, yaw_in, 1e-9);
}

// ============================================================
// get_pose テスト
// ============================================================

TEST(ROSUtilityTest, GetPoseFromXYZRPY)
{
    auto pose = potbot_lib::utility::get_pose(1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 2.0);
    EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
    double roll, pitch, yaw;
    potbot_lib::utility::get_rpy(pose.orientation, roll, pitch, yaw);
    EXPECT_NEAR(yaw, M_PI / 2.0, 1e-9);
}

TEST(ROSUtilityTest, GetPoseFromPotbotPose)
{
    potbot_lib::Pose p;
    p.position.x = 1.0;
    p.position.y = 2.0;
    p.position.z = 0.0;
    p.rotation.x = 0.0;
    p.rotation.y = 0.0;
    p.rotation.z = 0.5;
    auto msg_pose = potbot_lib::utility::get_pose(p);
    EXPECT_DOUBLE_EQ(msg_pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(msg_pose.position.y, 2.0);
}

TEST(ROSUtilityTest, GetPotbotPoseFromMsgPose)
{
    auto msg_pose = potbot_lib::utility::get_pose(3.0, 4.0, 0.0, 0.0, 0.0, 0.0);
    potbot_lib::Pose p = potbot_lib::utility::get_pose(msg_pose);
    EXPECT_DOUBLE_EQ(p.position.x, 3.0);
    EXPECT_DOUBLE_EQ(p.position.y, 4.0);
}

TEST(ROSUtilityTest, GetPoseRoundTrip)
{
    potbot_lib::Pose original;
    original.position.x = 5.0;
    original.position.y = 6.0;
    original.position.z = 0.0;
    original.rotation.x = 0.0;
    original.rotation.y = 0.0;
    original.rotation.z = 1.0;

    auto msg_pose = potbot_lib::utility::get_pose(original);
    potbot_lib::Pose restored = potbot_lib::utility::get_pose(msg_pose);

    EXPECT_DOUBLE_EQ(restored.position.x, original.position.x);
    EXPECT_DOUBLE_EQ(restored.position.y, original.position.y);
    EXPECT_NEAR(restored.rotation.z, original.rotation.z, 1e-9);
}

// ============================================================
// get_distance テスト
// ============================================================

TEST(ROSUtilityTest, GetDistanceKnownValue)
{
    geometry_msgs::msg::Point p1;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    geometry_msgs::msg::Point p2;
    p2.x = 3.0; p2.y = 4.0; p2.z = 0.0;
    double dist = potbot_lib::utility::get_distance(p1, p2);
    EXPECT_DOUBLE_EQ(dist, 5.0);
}

TEST(ROSUtilityTest, GetDistanceSamePoint)
{
    geometry_msgs::msg::Point p1;
    p1.x = 1.0; p1.y = 1.0; p1.z = 1.0;
    double dist = potbot_lib::utility::get_distance(p1, p1);
    EXPECT_DOUBLE_EQ(dist, 0.0);
}

TEST(ROSUtilityTest, GetDistance3D)
{
    geometry_msgs::msg::Point p1;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    geometry_msgs::msg::Point p2;
    p2.x = 1.0; p2.y = 1.0; p2.z = 1.0;
    double dist = potbot_lib::utility::get_distance(p1, p2);
    EXPECT_NEAR(dist, std::sqrt(3.0), 1e-9);
}

// ============================================================
// get_map_index / get_map_coordinate テスト
// ============================================================

nav_msgs::msg::MapMetaData make_test_map_info()
{
    nav_msgs::msg::MapMetaData info;
    info.resolution = 0.05f;
    info.width = 100;
    info.height = 100;
    info.origin.position.x = 0.0;
    info.origin.position.y = 0.0;
    return info;
}

TEST(ROSUtilityTest, GetMapIndexOrigin)
{
    auto info = make_test_map_info();
    int idx = potbot_lib::utility::get_map_index(0.0, 0.0, info);
    EXPECT_EQ(idx, 0);
}

TEST(ROSUtilityTest, GetMapIndexKnownCell)
{
    auto info = make_test_map_info();
    // resolution は float32 型なので double との混算で精度誤差が生じる
    // get_map_coordinate で取得したセル中心座標を使うことで往復整合性を確認する
    // インデックス5のセル座標を取得し、それをget_map_indexに渡す
    auto coord = potbot_lib::utility::get_map_coordinate(5, info);
    // 小数点の中心にわずかに寄せることで境界誤差を回避する
    int idx = potbot_lib::utility::get_map_index(coord.x + 1e-6, coord.y + 1e-6, info);
    EXPECT_EQ(idx, 5);
}

TEST(ROSUtilityTest, GetMapIndexRowTwo)
{
    auto info = make_test_map_info();
    // インデックス200 (x=0, y=2) のセル座標を取得して往復確認
    auto coord = potbot_lib::utility::get_map_coordinate(200, info);
    int idx = potbot_lib::utility::get_map_index(coord.x + 1e-6, coord.y + 1e-6, info);
    EXPECT_EQ(idx, 200);
}

TEST(ROSUtilityTest, GetMapCoordinateOrigin)
{
    auto info = make_test_map_info();
    auto pt = potbot_lib::utility::get_map_coordinate(0, info);
    EXPECT_DOUBLE_EQ(pt.x, 0.0);
    EXPECT_DOUBLE_EQ(pt.y, 0.0);
}

TEST(ROSUtilityTest, GetMapCoordinateCell1)
{
    auto info = make_test_map_info();
    // インデックス1 → グリッド(1, 0) → 座標 (0.05, 0.0)
    auto pt = potbot_lib::utility::get_map_coordinate(1, info);
    EXPECT_NEAR(pt.x, 0.05, 1e-6);
    EXPECT_NEAR(pt.y, 0.0, 1e-6);
}

TEST(ROSUtilityTest, GetMapCoordinateRow2)
{
    auto info = make_test_map_info();
    // インデックス100 → グリッド(0, 1) → 座標 (0.0, 0.05)
    auto pt = potbot_lib::utility::get_map_coordinate(100, info);
    EXPECT_NEAR(pt.x, 0.0, 1e-6);
    EXPECT_NEAR(pt.y, 0.05, 1e-6);
}

// ============================================================
// get_path / to_msg 往復テスト
// ============================================================

TEST(ROSUtilityTest, GetPathRoundTrip)
{
    std::vector<potbot_lib::Pose> original(3);
    original[0].position.x = 0.0; original[0].position.y = 0.0;
    original[1].position.x = 1.0; original[1].position.y = 0.0;
    original[2].position.x = 2.0; original[2].position.y = 1.0;

    auto msg_path = potbot_lib::utility::get_path(original);
    auto restored = potbot_lib::utility::get_path(msg_path);

    ASSERT_EQ(restored.size(), original.size());
    for (size_t i = 0; i < original.size(); i++)
    {
        EXPECT_DOUBLE_EQ(restored[i].position.x, original[i].position.x);
        EXPECT_DOUBLE_EQ(restored[i].position.y, original[i].position.y);
    }
}

TEST(ROSUtilityTest, ToMsgPosesRoundTrip)
{
    std::vector<potbot_lib::Pose> poses(2);
    poses[0].position.x = 1.0; poses[0].position.y = 2.0;
    poses[1].position.x = 3.0; poses[1].position.y = 4.0;

    std::vector<geometry_msgs::msg::PoseStamped> msg_poses;
    potbot_lib::utility::to_msg(poses, msg_poses);

    ASSERT_EQ(msg_poses.size(), poses.size());
    EXPECT_DOUBLE_EQ(msg_poses[0].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(msg_poses[1].pose.position.y, 4.0);
}

// ============================================================
// color::get_msg テスト
// ============================================================

TEST(ROSUtilityTest, ColorGetMsgRed)
{
    auto c = potbot_lib::color::get_msg(potbot_lib::color::RED);
    EXPECT_FLOAT_EQ(c.r, 1.0f);
    EXPECT_FLOAT_EQ(c.g, 0.0f);
    EXPECT_FLOAT_EQ(c.b, 0.0f);
    EXPECT_FLOAT_EQ(c.a, 1.0f);
}

TEST(ROSUtilityTest, ColorGetMsgGreen)
{
    auto c = potbot_lib::color::get_msg(potbot_lib::color::GREEN);
    EXPECT_FLOAT_EQ(c.r, 0.0f);
    EXPECT_FLOAT_EQ(c.g, 1.0f);
    EXPECT_FLOAT_EQ(c.b, 0.0f);
    EXPECT_FLOAT_EQ(c.a, 1.0f);
}

TEST(ROSUtilityTest, ColorGetMsgBlue)
{
    auto c = potbot_lib::color::get_msg(potbot_lib::color::BLUE);
    EXPECT_FLOAT_EQ(c.r, 0.0f);
    EXPECT_FLOAT_EQ(c.g, 0.0f);
    EXPECT_FLOAT_EQ(c.b, 1.0f);
    EXPECT_FLOAT_EQ(c.a, 1.0f);
}

TEST(ROSUtilityTest, ColorGetMsgYellow)
{
    auto c = potbot_lib::color::get_msg(potbot_lib::color::YELLOW);
    EXPECT_FLOAT_EQ(c.r, 1.0f);
    EXPECT_FLOAT_EQ(c.g, 1.0f);
    EXPECT_FLOAT_EQ(c.b, 0.0f);
    EXPECT_FLOAT_EQ(c.a, 1.0f);
}

TEST(ROSUtilityTest, ColorGetMsgWhite)
{
    auto c = potbot_lib::color::get_msg(potbot_lib::color::WHITE);
    EXPECT_FLOAT_EQ(c.r, 1.0f);
    EXPECT_FLOAT_EQ(c.g, 1.0f);
    EXPECT_FLOAT_EQ(c.b, 1.0f);
    EXPECT_FLOAT_EQ(c.a, 1.0f);
}

TEST(ROSUtilityTest, ColorGetMsgModulo)
{
    // color_id % 8 で折り返すので color_id=8 は RED と同じになる
    auto c_red = potbot_lib::color::get_msg(potbot_lib::color::RED);
    auto c_mod = potbot_lib::color::get_msg(8);
    EXPECT_FLOAT_EQ(c_mod.r, c_red.r);
    EXPECT_FLOAT_EQ(c_mod.g, c_red.g);
    EXPECT_FLOAT_EQ(c_mod.b, c_red.b);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
