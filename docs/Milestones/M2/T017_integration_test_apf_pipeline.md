# T-017 チケット仕様書 — APF パイプライン統合テスト

| 項目 | 内容 |
|---|---|
| チケット番号 | T-017 |
| マイルストーン | M2 |
| タイトル | APF パイプライン統合テストを追加（APFPathPlanner + OptimalPathFollower + DiffDriveAgent 連携） |
| コミット | `832fad4` |
| ステータス | 完了 |

---

## 1. 実装内容

`potbot_lib/test/integration/` ディレクトリに APF ナビゲーションパイプライン全体の統合テストを作成した。

Navigation 2 の統合テストパターン（複数コンポーネントの協調動作検証）を踏襲し、以下の3コンポーネントが連携して動作することを end-to-end で検証する：

```
ArtificialPotentialField → APFPathPlanner → OptimalPathFollower（DiffDriveAgent を継承）
```

各テストでは実際の制御ループ（最大200ステップ）を実行し、ゴール到達またはゴールへの近接を確認する。

---

## 2. テストケース一覧

### 2.1 integration/test_apf_pipeline.cpp（合計 3 件）

#### APFPipelineTest / FullNavigationCycle

障害物なし環境でのフルナビゲーションサイクルを検証する。

| 設定項目 | 値 |
|---|---|
| グリッド | 41×41、解像度 0.05m、origin (0,0) |
| 引力ゲイン（wa） | 1.0 |
| 斥力ゲイン（wr） | 0.0（障害物なし） |
| dtr | 10.0 |
| ロボット初期位置 | (-0.9, 0.0) |
| ゴール | (0.9, 0.0) |
| OptimalPathFollower 速度制限 | v: [-0.3, 0.3]、omega: [-1.5, 1.5] |
| 最適化手法 | all_search |
| 最大ステップ数 | 200 |

検証内容:
- `planner.createPath()` が true を返す
- 生成経路の点数が 1 より大きい
- 200ステップ以内にゴールに到達（`reachedTarget() == true`）
- ゴールまでの距離が 0.5 m 未満

---

#### APFPipelineTest / NavigationWithObstacle

障害物回避を含むフルパイプラインを検証する。

| 設定項目 | 値 |
|---|---|
| グリッド | 41×41、解像度 0.05m、origin (0,0) |
| 引力ゲイン（wa） | 1.0 |
| 斥力ゲイン（wr） | 5.0 |
| dtr | 0.3 |
| ロボット初期位置 | (-0.8, 0.0) |
| ゴール | (0.8, 0.0) |
| 障害物位置 | (0.0, 0.1)（ロボット-ゴール中間） |
| 最適化手法 | all_search |
| 最大ステップ数 | 200 |

検証内容:
- `planner.createPath()` が true を返す
- 生成経路が空でない
- 200ステップ後のゴールまでの距離が 1.5 m 未満（障害物回避により迂回するため許容誤差は広め）

---

#### APFPipelineTest / MultipleReplanningCycles

ゴールを変えながら3サイクルの経路計画→追従を繰り返す。各サイクルで独立したAPFインスタンスを使用する。

| サイクル | ロボット初期位置 | ゴール | 初期 yaw | 到達要件 |
|---|---|---|---|---|
| 1 | (-0.5, 0.0) | (0.5, 0.0) | 0.0 | `reachedTarget() == true` |
| 2 | (0.0, -0.5) | (0.0, 0.5) | π/2 | `reachedTarget() == true` |
| 3 | (-0.5, -0.5) | (0.5, 0.5) | π/4 | ゴールまでの距離 < 1.0 m |

全サイクル共通設定:
- グリッド: 41×41、解像度 0.05m
- wa=1.0、wr=0.0（障害物なし）、dtr=10.0
- OptimalPathFollower: `all_search`、v: [-0.3, 0.3]、omega: [-1.5, 1.5]
- 最大ステップ数: 200

---

## 3. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# APF パイプライン統合テストのみ実行
ctest --test-dir build/potbot_lib -R test_apf_pipeline

# 全テスト実行
ctest --test-dir build/potbot_lib
```

---

## 4. 対応ファイル

- `potbot_lib/test/integration/test_apf_pipeline.cpp` — APF パイプライン統合テスト
- `potbot_lib/test/integration/CMakeLists.txt` — 統合テスト用ビルド設定
- `potbot_lib/CMakeLists.txt` — テスト登録設定
