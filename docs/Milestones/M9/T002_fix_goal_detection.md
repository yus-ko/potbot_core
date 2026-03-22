# M9 T-002: ゴール到達判定修正とcreatePathWithWeight改善

## チケット概要

- **マイルストーン**: M9 (createPathWithWeightをNav2パイプラインに統合)
- **チケット番号**: T-002
- **目的**: `createPathWithWeight()` でゴールに到達できない問題を修正し、可視化の座標系ミスマッチを解消する

## 修正したバグの詳細

### バグA: Jスケール不一致（局所解脱出が機能しない）

**場所**: `potbot_lib/src/apf_path_planner.cpp` の `createPathWithWeight()` 局所解脱出ブロック

**問題**: `J_min` は `J_min_pre`（ポテンシャル値、例: 50〜1000+）で初期化されるが、
J は正規化比率（0〜1）のため、常に `J <= J_min` が true になり、最初のセルが無条件選択される。

```cpp
// 修正前（バグ）
double J_min = J_min_pre;  // 大きな数値

// J は常に 0〜1 の範囲（正規化済み）
j1 = wu * PotentialValue / sum;  // 0〜1
J = j1 + j2;                     // 0〜1

if (J <= J_min)  // 常にtrue → 最初のセルを選ぶだけで脱出になっていない
```

**修正後**:
```cpp
// バグ修正: スケール統一
J_min = std::numeric_limits<double>::infinity();
// → 初回も正しく最良Jセルを選択できる
```

---

### バグB: Dijkstraフォールバックなし（壁越えゴールに到達不能）

**場所**: `createPathWithWeight()` の冒頭

**問題**: `createPath()` (M8) はDijkstraを最初に試みて失敗時のみ勾配降下にフォールバックするが、
`createPathWithWeight()` はこのパターンを持たないため、ゴールが壁の向こうにある場合に経路生成が失敗する。

**修正後**:
```cpp
bool APFPathPlanner::createPathWithWeight(double init_robot_pose)
{
    // まずDijkstra法で経路を生成する（createPath()と同様のパターン）
    if (createPathDijkstra(init_robot_pose)) {
        return true;
    }
    // Dijkstraが失敗した場合はweighted勾配降下法にフォールバック
    path_.clear();
    // ... 既存のweightedアプローチ
```

---

### バグC: no_progress_count未実装（ループ検出なし）

**場所**: `createPathWithWeight()` のメインループ

**問題**: `createPath()` に実装されているno_progress検出（バグ5修正）が
`createPathWithWeight()` には存在せず、ゴール方向に進まない場合のループ防止が機能しない。

**修正後**:
```cpp
Point goal = apf_->getGoal();
double prev_dist_to_goal = std::numeric_limits<double>::infinity();
int no_progress_count = 0;
const int no_progress_limit = 50;

// ループ末尾で:
double current_dist_to_goal = sqrt(pow(px - goal.x, 2) + pow(py - goal.y, 2));
if (current_dist_to_goal >= prev_dist_to_goal) no_progress_count++;
else no_progress_count = 0;
prev_dist_to_goal = current_dist_to_goal;
if (no_progress_count >= no_progress_limit) break;
```

---

### バグD: 可視化の座標系ミスマッチ

**場所**: `test/scripts/analyze_rosbag.py` / `test/scripts/rosbag_record_wrapper.sh`

**問題**: ロボット軌跡を `/odom`（odom座標系）から描画していたが、
ゴール位置は `/test/goal_pose`（map座標系）であり、座標系が異なるため
視覚的に「ゴールに届いていない」ように見えていた。
また `/amcl_pose` がrosbagに記録されていなかった。

**修正内容**:
1. `rosbag_record_wrapper.sh` に `/amcl_pose` を記録トピックとして追加
2. `analyze_rosbag.py` で `/amcl_pose` データを読み込み
3. `/amcl_pose` がある場合はmap座標系で軌跡描画（ゴールと同じ座標系）
4. `/amcl_pose` がない場合は `/odom` にフォールバック（後方互換性確保）

## E2Eテスト結果

- `planning_method: "weighted"` + `weight_potential: 1.0` + `weight_pose: 0.0`
- ゴール: (-1.25, 3.5)（壁の向こう側）
- 結果: `ゴール到達成功！` / 終了コード 0
- AMCL軌跡（map座標系）がゴール位置に到達していることを `navigation_result.png` で確認済み

## 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `potbot_lib/src/apf_path_planner.cpp` | バグA・B・Cの修正 |
| `test/scripts/rosbag_record_wrapper.sh` | `/amcl_pose` を記録トピックに追加 |
| `test/scripts/analyze_rosbag.py` | `/amcl_pose` 優先の軌跡描画対応 |
| `docs/Milestones/M9/T002_fix_goal_detection.md` | 本ドキュメント（新規作成） |
