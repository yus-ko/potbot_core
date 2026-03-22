# M1: T-004 potbot_plugin APF プランナーテスト仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-004 |
| マイルストーン | M1 |
| タイトル | potbot_plugin APF プランナーテストを追加 |
| コミット | `761269f` |
| ステータス | 完了 |

### 目的

`potbot_nav::planner::APF` クラスに対して rclcpp ベースのユニットテストを新規作成し、Nav2 GlobalPlanner プラグインとしてのオブジェクト生成・インターフェース継承を自動的に検証できる体制を整える。

### 背景

`potbot_plugin` の APF プランナーは `nav2_core::GlobalPlanner` を継承した Nav2 プラグインであり、完全なテスト（`configure()` + `createPlan()` の実行）は `Costmap2DROS` の初期化を必要とするため困難である。M1 では rclcpp の初期化のみで実行可能な範囲（オブジェクト生成・インターフェース継承確認・複数インスタンス同時生成）のテストを追加した。

---

## 2. 実装内容

- `potbot_plugin/test/test_apf_planner.cpp` を新規作成（4 テストケース）
- `potbot_plugin/CMakeLists.txt` に `test_apf_planner` のテストターゲットを追加
- rclcpp の初期化・終了はテストスイート単位（`SetUpTestSuite` / `TearDownTestSuite`）で一度だけ行う

---

## 3. テストケース一覧

### テストフィクスチャ: `APFPlannerTest`

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `InstanceIsValid` | `std::make_unique<potbot_nav::planner::APF>()` が成功し nullptr でないこと |
| 2 | `InheritsGlobalPlannerInterface` | `dynamic_pointer_cast<nav2_core::GlobalPlanner>` が成功し、継承関係が正しいこと |
| 3 | `PlanReturnTypeIsPath` | `createPlan()` の戻り値型が `nav_msgs::msg::Path` であることをコンパイル時に確認すること |
| 4 | `MultipleInstancesNoInterference` | 3 つの APF インスタンスを同時に生成しても例外が発生しないこと |

**合計: 4 件**

---

## 4. テスト制限事項

- `configure()` は `Costmap2DROS`・`tf2_ros::Buffer`・ノード等を必要とするため、テスト対象外とする
- `createPlan()` は `configure()` 後でなければ nullptr 参照が発生するため、テスト対象外とする
- テストは rclcpp の基本的な初期化のみで実行可能な範囲に限定する

---

## 5. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_plugin --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_plugin

# このファイルのみ実行
ctest --test-dir build/potbot_plugin -R test_apf_planner
```

---

## 6. 対応ファイル

- `potbot_plugin/test/test_apf_planner.cpp` — テスト本体（新規作成）
- `potbot_plugin/CMakeLists.txt` — テストターゲット追加
