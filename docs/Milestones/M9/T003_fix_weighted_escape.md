# M9 T-003: createPathWithWeightのweighted脱出機構修正

## チケット概要

- **マイルストーン**: M9 (createPathWithWeightをNav2パイプラインに統合)
- **チケット番号**: T-003
- **目的**: `createPathWithWeight()` をDijkstraフォールバックなしで局所解を回避できるよう修正する

## 背景

T-002でDijkstraフォールバックを追加することでE2Eテストは通過したが、
「createPathWithWeightによるweighted探索のみで局所解を回避できることがこのリポジトリの独自性」
という要件によりDijkstraフォールバックを削除し、weighted脱出機構自体を正しく機能させる。

## 修正したバグの詳細

### バグA: break_flagが局所解脱出ループを1回で終了させていた

**場所**: `potbot_lib/src/apf_path_planner.cpp` の else ブロック（局所解脱出）

**問題**: 外側ループ `for (i=0; i<100; i++)` の最初の反復で利用可能なセルが見つかると
`break_flag = true` → `if (break_flag) break;` で即終了していた。
ランダム重み(`wu`, `w_theta`)やrange拡大（1→最大11）は実行されず、
常にrange=1の最初のセルが選択されるため、壁回りの局所解を脱出できなかった。

```cpp
// 修正前（バグ）
bool break_flag = false;
for (size_t i = 0; i < 100; i++) {
    // ...
    if (J <= J_min) {
        J_min = J; pf_idx_min = idx;
        break_flag = true;   // ← 最初のセルで即終了
    }
    if (break_flag) break;   // ← 100回試行のうち1回目で完了
    wu = random; w_theta = 1-wu;  // ← 実行されない
    random_range = random*10+1;   // ← 実行されない
}
```

**修正後**:
```cpp
size_t best_idx = SIZE_MAX;  // 全100回試行での最良セルインデックス
for (size_t i = 0; i < 100; i++) {
    // ...
    if (J < J_min) {
        J_min = J;
        best_idx = idx;  // break_flagなしでグローバル最小Jを追跡
    }
    // 全100回実行するため次の反復のランダム化を常に行う
    wu = random; w_theta = 1-wu;
    random_range = random*10+1;
}
if (best_idx != SIZE_MAX) {
    solving_local_minimum = false;
    pf_idx_min = best_idx;
}
```

**効果**: 100回の試行で探索範囲が最大range=11（21×21セル、約1m×1m）まで拡大し、
ランダムな`wu`/`w_theta`の組み合わせで最良の脱出方向を見つける。

---

### バグB: Dijkstraフォールバック（T-002で追加→本チケットで削除）

**問題**: `createPathWithWeight()` 冒頭でDijkstraを呼び出すため、
weighted探索が実行される機会がなかった。

**修正**: Dijkstraフォールバックブロックを完全に削除。
`createPathWithWeight()` はweighted勾配降下法のみで経路生成する。

---

### バグC: search_indexes空時のbreak

**問題**: 探索範囲内にセルがない場合に `break` でループ終了していたため、
残りの試行（ランダム範囲拡大）が実行されなかった。

**修正**:
```cpp
// 修正前
if (search_indexes.empty()) break;

// 修正後
if (search_indexes.empty()) {
    wu = random; w_theta = 1-wu;
    random_range = random*10+1;
    continue;  // 次のランダム範囲で再試行
}
```

---

### バグD: no_progress_countが壁迂回を妨害

**問題**: `no_progress_count >= 50` でループ強制終了するため、
壁を迂回するために一時的にゴールから遠ざかる動きが必要な場合に打ち切られた。

**修正**: `no_progress_count` 関連の変数宣言とチェックブロックを削除。
`max_path_length_`（20m）と重複検出で終了条件を担保する。

## E2Eテスト結果

- `planning_method: "weighted"` + `weight_potential: 1.0` + `weight_pose: 0.0`
- 開始位置: (-2.0, -0.5)
- ゴール: (-1.25, 3.5)（壁の向こう側）
- 結果: `ゴール到達成功！` / 終了コード 0
- Dijkstraフォールバックなし（weighted探索のみ）

## 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `potbot_lib/src/apf_path_planner.cpp` | バグA・B・C・Dの修正 |
| `docs/Milestones/M9/T003_fix_weighted_escape.md` | 本ドキュメント（新規作成） |
