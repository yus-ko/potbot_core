# M1: T-005 potbot_plugin OptimalPathFollower プラグインテスト仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-005 |
| マイルストーン | M1 |
| タイトル | potbot_plugin OptimalPathFollower プラグインテストを追加 |
| コミット | `aaae985` |
| ステータス | 完了 |

### 目的

`potbot_nav::controller::OptimalPathFollower` クラスに対して rclcpp ベースのユニットテストを新規作成し、Nav2 Controller プラグインとしてのオブジェクト生成・インターフェース継承・configure 前の安全性を自動的に検証できる体制を整える。

### 背景

`potbot_plugin` の OptimalPathFollower コントローラーは `nav2_core::Controller` を継承した Nav2 プラグインである。T-004 の APF プランナー同様、`configure()` の完全なテストは困難であるため、configure 前に安全に呼び出せる範囲のテストを追加した。なお `setPlan()` / `cleanup()` / `activate()` / `deactivate()` は configure() 後に生成される `global_pub_` を使用するため対象外とした。

---

## 2. 実装内容

- `potbot_plugin/test/test_optimal_path_follower_plugin.cpp` を新規作成（6 テストケース）
- `potbot_plugin/CMakeLists.txt` に `test_optimal_path_follower_plugin` のテストターゲットを追加
- rclcpp の初期化・終了はテストスイート単位（`SetUpTestSuite` / `TearDownTestSuite`）で一度だけ行う

---

## 3. テストケース一覧

### テストフィクスチャ: `OptimalPathFollowerPluginTest`

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `ConstructorNoThrow` | デフォルトコンストラクタが例外なく成功すること |
| 2 | `InstanceIsValid` | `std::make_unique<...>()` が成功し nullptr でないこと |
| 3 | `InheritsControllerInterface` | `dynamic_pointer_cast<nav2_core::Controller>` が成功し、継承関係が正しいこと |
| 4 | `SetSpeedLimitAbsolute` | `setSpeedLimit(0.5, false)`（絶対値指定）が configure 前でもクラッシュしないこと |
| 5 | `SetSpeedLimitPercentage` | `setSpeedLimit(50.0, true)`（パーセント指定）が configure 前でもクラッシュしないこと |
| 6 | `MultipleInstancesCanBeCreated` | 3 つのコントローラーインスタンスを同時に生成しても例外が発生しないこと |

**合計: 6 件**

---

## 4. テスト制限事項

- `configure()` は `Costmap2DROS`・`tf2_ros::Buffer`・ノード等を必要とするため、テスト対象外とする
- `setPlan()` / `cleanup()` / `activate()` / `deactivate()` は configure 後に生成される `global_pub_` を使用するため、テスト対象外とする
- `computeVelocityCommands()` は configure 後のみ安全に呼び出せるため、テスト対象外とする
- `setSpeedLimit()` の実装は no-op であるため、configure 前でも安全に呼び出せることを確認する

---

## 5. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_plugin --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_plugin

# このファイルのみ実行
ctest --test-dir build/potbot_plugin -R test_optimal_path_follower_plugin
```

---

## 6. 対応ファイル

- `potbot_plugin/test/test_optimal_path_follower_plugin.cpp` — テスト本体（新規作成）
- `potbot_plugin/CMakeLists.txt` — テストターゲット追加
