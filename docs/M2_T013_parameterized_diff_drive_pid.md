# T-013 チケット仕様書 — DiffDriveAgent・PID の TEST_P パラメータ化テスト

| 項目 | 内容 |
|---|---|
| チケット番号 | T-013 |
| マイルストーン | M2 |
| タイトル | DiffDriveAgent・PID の TEST_P パラメータ化テストを追加 |
| コミット | `f248d19` |
| ステータス | 完了 |

---

## 1. 実装内容

`DiffDriveAgent` および `PID` クラスに対して Google Test の `TEST_P` マクロを使用したパラメータ化テストを追加した。

- `DiffDriveAgent`: 速度・角速度・タイムステップの組み合わせによる `update()` の数値検証、距離・角度計算の多方向検証
- `PID`: 速度リミットクランプ、到達判定、ゲイン収束特性、`initPID()` 状態リセットの各観点をパラメータ化

各テストは `INSTANTIATE_TEST_SUITE_P` で具体値を注入し、テストロジックは1つの `TEST_P` 関数として記述する Navigation 2 スタイルを採用した。

---

## 2. テストケース一覧

### 2.1 test_diff_drive_agent_param.cpp（合計 16 件）

#### DiffDriveUpdateTest / UpdatePosition（5件）

`DiffDriveAgent::update()` を各パラメータセットで1回呼び出し、`x`・`y`・`yaw` が期待値と一致することを `EXPECT_NEAR(, 1e-9)` で検証する。

| # | v | omega | dt | expected_x | expected_y | expected_yaw | 説明 |
|---|---|---|---|---|---|---|---|
| 0 | 1.0 | 0.0 | 0.1 | 0.1 | 0.0 | 0.0 | 直進（omega=0, yaw=0） |
| 1 | 0.0 | π | 0.1 | 0.0 | 0.0 | π×0.1 | 旋回のみ（v=0） |
| 2 | -1.0 | 0.0 | 0.1 | -0.1 | 0.0 | 0.0 | 後退（v=-1.0） |
| 3 | 1.0 | 0.0 | 0.02 | 0.02 | 0.0 | 0.0 | デフォルト dt=0.02s で直進 |
| 4 | 1.0 | -π/2 | 0.1 | cos(-π/20)×0.1 | sin(-π/20)×0.1 | -π/20 | 右旋回しながら前進 |

#### DiffDriveDistanceTest / GetDistanceToPoint（5件）

`DiffDriveAgent::getDistance(Point)` が期待値（hypot 計算値）と一致することを検証する。

| # | robot (x, y) | target (x, y) | expected_distance | 説明 |
|---|---|---|---|---|
| 0 | (0, 0) | (3, 4) | 5.0 | 3-4-5 直角三角形 |
| 1 | (0, 0) | (0, 0) | 0.0 | 同一点（距離ゼロ） |
| 2 | (-1, -1) | (2, 3) | 5.0 | 負座標 |
| 3 | (0, 0) | (1, 0) | 1.0 | X 軸方向のみ |
| 4 | (0, 0) | (0, 1) | 1.0 | Y 軸方向のみ |

#### DiffDriveAngleTest / GetAngleToPoint（6件）

`DiffDriveAgent::getAngle(Point)` が `atan2` の期待値と一致することを検証する。

| # | robot (x, y) | target (x, y) | expected_angle | 説明 |
|---|---|---|---|---|
| 0 | (0, 0) | (1, 0) | 0.0 | +X 方向 |
| 1 | (0, 0) | (0, 1) | π/2 | +Y 方向 |
| 2 | (0, 0) | (-1, 0) | π | -X 方向 |
| 3 | (0, 0) | (0, -1) | -π/2 | -Y 方向 |
| 4 | (0, 0) | (1, 1) | π/4 | 45度方向 |
| 5 | (1, 1) | (2, 2) | π/4 | オフセット付き |

---

### 2.2 test_pid_param.cpp（合計 21 件）

#### PIDApplyLimitTest / ApplyLimitClamps（7件）

`PID::applyLimit()` が `v`・`omega` を指定上限・下限にクランプすることを検証する。

| # | max_linear | max_angular | input_v | input_omega | expected_v | expected_omega | 説明 |
|---|---|---|---|---|---|---|---|
| 0 | 0.5 | π | 1.0 | 0.0 | 0.5 | 0.0 | v が上限を超える |
| 1 | 0.5 | π | -1.0 | 0.0 | -0.5 | 0.0 | v が下限を超える（負値） |
| 2 | 1.0 | 1.0 | 0.0 | 5.0 | 0.0 | 1.0 | omega が上限を超える |
| 3 | 1.0 | 1.0 | 0.0 | -5.0 | 0.0 | -1.0 | omega が下限を超える（負値） |
| 4 | 1.0 | 2.0 | 0.5 | 1.0 | 0.5 | 1.0 | リミット内：変化なし |
| 5 | 1.0 | π | 0.0 | 0.0 | 0.0 | 0.0 | ゼロ：変化なし |
| 6 | 0.3 | 0.5 | 2.0 | 3.0 | 0.3 | 0.5 | v と omega 両方超過 |

#### PIDReachedTargetTest / ReachedTargetCheck（5件）

`PID::reachedTarget()` がマージン内外で正しい真偽値を返すことを検証する。ロボットは原点（デフォルト）。

| # | target (x, y) | margin_dist | margin_angle | expected | 説明 |
|---|---|---|---|---|---|
| 0 | (0, 0) | 0.03 | 0.1 | true | ロボット=ゴール |
| 1 | (10, 10) | 0.03 | 0.1 | false | マージン外 |
| 2 | (0.02, 0) | 0.03 | 0.1 | true | マージン内 |
| 3 | (1, 0) | 0.03 | 0.1 | false | マージン超過 |
| 4 | (0.5, 0) | 1.0 | π | true | 大きなマージン |

#### PIDGainConvergenceTest / ConvergesWithGain（5件）

PID ゲイン組み合わせごとに `calculateCommand()` + `update()` ループを実行し、目標に収束することを検証する。

| # | gain_p | gain_i | gain_d | target_x | max_steps | expect_converge | 説明 |
|---|---|---|---|---|---|---|---|
| 0 | 5.0 | 0.0 | 0.0 | 0.5 | 2000 | true | 高 P ゲイン |
| 1 | 1.0 | 0.0 | 0.0 | 0.5 | 5000 | true | 低 P ゲイン |
| 2 | 3.0 | 0.5 | 0.0 | 0.5 | 5000 | true | P+I ゲイン |
| 3 | 3.0 | 0.0 | 0.001 | 0.5 | 2000 | true | P+D ゲイン |
| 4 | 3.0 | 0.5 | 0.001 | 0.5 | 5000 | true | P+I+D ゲイン |

#### PIDInitResetTest / InitPIDResetsToStop（4件）

`initPID()` 呼び出し後にプロセス状態が `PROCESS_STOP` になることを検証する。

| # | target (x, y) | steps_before_init | 説明 |
|---|---|---|---|
| 0 | (1, 0) | 1 | 1ステップ後にリセット |
| 1 | (0, 1) | 5 | 5ステップ後にリセット |
| 2 | (2, 2) | 10 | 10ステップ後にリセット |
| 3 | (-1, 0) | 3 | 3ステップ後にリセット |

---

## 3. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# DiffDriveAgent パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_diff_drive_agent_param

# PID パラメータ化テストのみ実行
ctest --test-dir build/potbot_lib -R test_pid_param

# 全テスト実行
ctest --test-dir build/potbot_lib
```

---

## 4. 対応ファイル

- `potbot_lib/test/test_diff_drive_agent_param.cpp` — DiffDriveAgent パラメータ化テスト
- `potbot_lib/test/test_pid_param.cpp` — PID パラメータ化テスト
- `potbot_lib/CMakeLists.txt` — テスト登録設定
