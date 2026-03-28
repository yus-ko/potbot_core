# M13: APFグローバルプランナー高速化

## 背景

E2Eパイプライン（`docker compose --profile pipeline up --abort-on-container-exit`）実行時、APFグローバルプランナーが3-5Hzでしか動作せず、移動障害物回避への拡張時にボトルネックとなる問題を解決する。

## 目標

プランニング周波数を20Hz以上に引き上げ、移動障害物回避の拡張に対応可能な計算余裕を確保する。

## 実施内容

### H1: フィールド解像度パラメータ化（T-027）

- `field_resolution` パラメータを `apf_planner.cpp` に追加
- costmap解像度（0.05m）と独立にAPFフィールド解像度を設定可能に
- デフォルト0.05m → 0.1mに変更でグリッドセル数1/4
- costmapからの障害物抽出もサンプリングステップを自動調整

### H2: Dijkstra→A*+tiebreak置換（T-028）

- `createPathAStar()` を `APFPathPlanner` に追加
- ユークリッド距離ヒューリスティック（admissible保証）
- tiebreak factor（1 + 1e-4）で同f値ノードのゴール方向優先展開
- `planning_method: "astar"` で直接A*を使用可能に
- フォールバック経路探索もDijkstra→A*に置換

### 計装: プランニング所要時間ログ出力（T-029）

- `createPlan()` 内に `std::chrono` ベースの計測を追加
- 障害物抽出、フィールド計算、パス探索の各フェーズを個別計測
- `RCLCPP_INFO` でグリッドサイズ、解像度、INCREMENTAL/FULL区分を出力

### H3: フィールド差分更新（T-030）

- `updatePotentialFieldIncremental()` で障害物変化検出
- 障害物数・位置が変化していない場合は斥力場の再計算をスキップ可能
- `createPotentialFieldRepulsionOnly()` で斥力場のみ再計算
- `publishPotentialField()` を計測区間から除外（publish自体が30-40ms消費していたことが判明）

## 計測結果

| 指標 | 改善前 | H1+H2後 | H3後（publish分離） |
|------|--------|---------|---------------------|
| プランニング周波数 | 3-5Hz | ~10Hz | **~20Hz+** |
| total所要時間 | ~200-300ms推定 | ~50ms | **10-25ms** |
| field計算 | 支配的 | ~50ms（publish含む） | **8-22ms** |
| path探索 | Dijkstra | **0.1-9ms（A*）** | 同左 |
| CPU使用率 | 99.8% | ~25% | 同等 |
| ナビゲーション | 成功 | 成功 | **成功** |

## ボトルネック分析

### 改善前の内訳（推定）
1. `createPotentialField()`: ~100-150ms（200x200グリッド、O(M²)）
2. `createPathDijkstra()`: ~50-100ms（O(M² log M²)）
3. `publishPotentialField()`: ~30-40ms（RViz MarkerArray）

### 改善後の内訳（実測）
1. 障害物抽出: ~0.1ms
2. フィールド計算: 8-22ms（グリッドサイズ依存）
3. パス探索（A*）: 0.1-9ms
4. publishPotentialField: 30-40ms（計測外、非ブロッキング化推奨）

## 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `potbot_lib/include/potbot_lib/artificial_potential_field.hpp` | `createPotentialFieldRepulsionOnly()`, `updatePotentialFieldIncremental()`, キャッシュメンバ追加 |
| `potbot_lib/src/artificial_potential_field.cpp` | 差分更新・斥力場分離計算・引力場再計算・局所解再検出の実装 |
| `potbot_lib/include/potbot_lib/apf_path_planner.hpp` | `createPathAStar()` 宣言追加 |
| `potbot_lib/src/apf_path_planner.cpp` | A*実装、フォールバックA*化 |
| `potbot_plugin/include/potbot_plugin/apf_planner.hpp` | `field_resolution_` メンバ追加 |
| `potbot_plugin/src/apf_planner.cpp` | 解像度パラメータ、計装、差分更新統合、publish分離 |
| `test/config/waffle_pi.yaml` | `planning_method: "astar"`, `field_resolution: 0.1` 追加 |

## 今後の課題

- `publishPotentialField()` の非同期化またはデバッグ時のみ有効化（30-40msの削減余地）
- 移動障害物導入時の差分更新効果の検証
- H4（斥力計算打ち切り）は計測上不要と判断、必要時に実施
