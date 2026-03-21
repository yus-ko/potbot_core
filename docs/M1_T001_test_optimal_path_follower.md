# M1: T-001 test_optimal_path_follower 仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| マイルストーン | M1 |
| タイトル | test_optimal_path_follower を追加 |
| コミット | `b0e5087` |
| ステータス | 完了 |

### 目的

M0 のスコープ外だった `OptimalPathFollower` クラスに対して Google Test (gtest) ベースのユニットテストを新規作成し、経路追従制御器の動作を自動的に検証できる体制を整える。

### 背景

`OptimalPathFollower` はロボットの経路追従制御を行うクラスであり、全探索法（all_search）と勾配法（gradient）の2つの最適化手法を実装している。M0 では依存関係の複雑さから対象外とされていたが、M1 にて重点的にテストを整備した。

---

## 2. 実装内容

- `potbot_lib/test/test_optimal_path_follower.cpp` を新規作成（16 テストケース）
- `make_path()` ヘルパー関数で始点(0,0)からゴールへの Pose リストを生成するユーティリティを実装
- `potbot_lib/CMakeLists.txt` に `test_optimal_path_follower` のテストターゲットを追加

---

## 3. テストケース一覧

### テストスイート: `OptimalPathFollowerTest`

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `Constructor` | デフォルト構築が成功すること |
| 2 | `InitialState` | 初期状態で v=0, omega=0, x=0, y=0 であること |
| 3 | `EmptyPathReachedTarget` | setTargetPath() 未呼び出し時に reachedTarget()=true であること |
| 4 | `SetTargetPath` | Pose リストをセット後、ゴールが遠い場合に reachedTarget()=false であること |
| 5 | `SetMarginAndLimit` | setMargin / setLimit を呼んでも例外が発生しないこと |
| 6 | `SettersNoThrow` | 各種セッター（最適化手法・時間増分・速度増分・学習率等）が例外なく動作すること |
| 7 | `CalculateCommandAllSearch` | all_search 法で calculateCommand() 後に v, omega が制限範囲内であること |
| 8 | `CalculateCommandGradient` | gradient 法で calculateCommand() 後に v, omega が制限範囲内であること |
| 9 | `GettersAfterCalculate` | calculateCommand() 後にゲッター（getPlans/getSplitPath/getBestPath/getBestPlan/getBestCmd）が正常動作すること |
| 10 | `ReachedTargetWhenClose` | ロボットがゴール近傍（0.01m）にいるとき reachedTarget()=true であること |
| 11 | `NotReachedTargetWhenFar` | ロボットがゴールから遠い（5m）とき reachedTarget()=false であること |
| 12 | `YDirectionTarget` | Y 軸方向目標（0, 1.0）に対して何らかの制御コマンドが出力されること |
| 13 | `DiagonalTarget` | 斜め方向目標（1.0, 1.0）に対して何らかの制御コマンドが出力されること |
| 14 | `ZeroLimitProducesZeroVelocity` | setLimit(0,0,0,0) 設定後は calculateCommand() で v=omega=0 になること |
| 15 | `ConvergenceSimulation` | ロボット状態を更新しながら reachedTarget() まで最大 500 ステップで収束すること |
| 16 | `GetSplitPathNonEmpty` | setTargetPath() + calculateCommand() 後に getSplitPath() が空でないこと |

**合計: 16 件**

---

## 4. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_lib

# このファイルのみ実行
ctest --test-dir build/potbot_lib -R test_optimal_path_follower
```

---

## 5. 対応ファイル

- `potbot_lib/test/test_optimal_path_follower.cpp` — テスト本体（新規作成）
- `potbot_lib/CMakeLists.txt` — テストターゲット追加
