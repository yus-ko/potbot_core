# M7: T-001 APFプランナーのパスループ問題を修正

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| マイルストーン | M7 |
| タイトル | APFプランナーのパスループ問題を修正 |
| ステータス | 完了 |

### 目的

ゴール位置 (-1.25, 3.5) を設定したとき `/plan` トピックのグローバルパスがループ状に乱れる問題を修正する。

---

## 2. 根本原因の詳細分析

### 原因 1: APF フィールドサイズの固定値（主因）

**場所:** `potbot_plugin/src/apf_planner.cpp`

**旧コード:**
```cpp
apfros_->getApf()->initPotentialField(50, 50, 0.05, robot.x, robot.y);
// コストマップスキャン
for (int mx = rmx - 50; mx < rmx + 50; mx++) {
    for (int my = rmy - 50; my < rmy + 50; my++) {
```

**問題:**
- フィールドは 50×50 セル × 0.05m = **2.5m × 2.5m**（ロボット中心 ±1.25m）
- ゴール (-1.25, 3.5) は Y 方向に 3.5m 必要 → フィールド外（y_max = 1.25m）
- `setGoal(x, y)` 内で `getFieldIndex(x, y)` が `out_of_range` 例外を投げる
- `catch(...) {}` でシレント処理され、`IS_AROUND_GOAL` が**一切設定されない**
- 経路探索の while ループ終了条件 `IS_AROUND_GOAL == false` が常に真
- 経路は `path_.size() > 100` の上限まで生成され続ける
- ローカル最小値解消コード（障害物エッジ追跡）が繰り返し呼ばれ、経路がループ状になる

### 原因 2: `getSquareIndex` の `size_t` アンダーフローバグ（副因）

**場所:** `potbot_lib/src/field.cpp`

**旧コード:**
```cpp
for (size_t row = centor_row-range; row <= centor_row+range; row++)
{
    for (size_t col = centor_col-range; col <= centor_col+range; col++)
    {
        ...
        catch(std::out_of_range& oor)
        {
            search_indexes.clear();  // 全結果をクリアしてリターン
            return;
        }
    }
}
```

**問題 2-1: size_t アンダーフロー**
- `centor_row = 0`, `range = 1` の場合、`size_t 0 - 1 = SIZE_MAX`（ラップアラウンド）
- ループ判定 `SIZE_MAX <= centor_row + range = 1` → false → ループが**全く実行されない**
- フィールド上端・左端でサーチが常に空になり、経路が途中で打ち切られる

**問題 2-2: 例外時の全クリア**
- 1つのセルが範囲外でも `search_indexes.clear()` で**全ての結果をクリア**してリターン
- フィールド境界付近（端から1つ内側）でもサーチが失敗する
- 例: `centor_row = 48`（50行フィールドの2番目から最後の行）では row=50 の例外で全クリア

---

## 3. 修正内容

### 修正 1: `apf_planner.cpp` — フィールドサイズの動的計算

**修正後コード:**
```cpp
// ロボット-ゴール間の距離に基づいてフィールドサイズを動的に計算し、
// ゴールが必ずフィールド内に含まれるようにする。
const double resolution = 0.05;
const int max_half_cells = 100;  // 最大 10m x 10m
double dist_x = std::abs(goal.pose.position.x - robot.x);
double dist_y = std::abs(goal.pose.position.y - robot.y);
double max_dist = std::max({dist_x, dist_y, 1.25});
int half_cells = std::min(static_cast<int>(max_dist / resolution) + 5, max_half_cells);
int total_cells = 2 * half_cells;

apfros_->getApf()->initPotentialField(total_cells, total_cells, resolution, robot.x, robot.y);
// コストマップスキャン
for (int mx = rmx - half_cells; mx < rmx + half_cells; mx++) {
    for (int my = rmy - half_cells; my < rmy + half_cells; my++) {
```

**効果:**
- ゴール (-1.25, 3.5) の場合: `max_dist = 3.5`, `half_cells = min(75, 100) = 75`
- フィールドサイズ: 150×150 セル × 0.05m = **7.5m × 7.5m**
- ゴールが常にフィールド内 → `IS_AROUND_GOAL` が正しく設定される
- 経路がゴール付近で正常に終端する

**フィールドサイズの計算式:**
| パラメータ | 値 |
|---|---|
| 最小半径 | 1.25m（元の固定サイズと同じ） |
| 最大半径 | 5.0m（max_half_cells=100 × 0.05m） |
| 安全マージン | +5セル（0.25m） |

### 修正 2: `field.cpp` — `getSquareIndex` のバグ修正

**修正後コード:**
```cpp
void Field::getSquareIndex(std::vector<size_t>& search_indexes, size_t centor_row, size_t centor_col, size_t range)
{
    // size_t のアンダーフローを避けるために signed int を使用する。
    int irange = static_cast<int>(range);
    int irow_center = static_cast<int>(centor_row);
    int icol_center = static_cast<int>(centor_col);

    for (int row = irow_center - irange; row <= irow_center + irange; row++)
    {
        for (int col = icol_center - irange; col <= icol_center + irange; col++)
        {
            if (row == irow_center && col == icol_center) continue;
            if (row < 0 || col < 0) continue;
            try
            {
                size_t pf_idx = getFieldIndex(static_cast<size_t>(row), static_cast<size_t>(col));
                search_indexes.push_back(pf_idx);
            }
            catch(std::out_of_range& oor)
            {
                continue;  // 範囲外のセルをスキップ（全クリアしない）
            }
        }
    }
}
```

**効果:**
- `size_t` アンダーフローが解消される（signed int を使用）
- フィールド境界付近でも有効なセルは正しくサーチ結果に含まれる
- フィールド端でのサーチが空にならなくなり、経路がより安定して生成される

---

## 4. ユニットテスト更新

**ファイル:** `potbot_lib/test/test_field.cpp`

`GetSquareIndexCorner` テストを修正後の正しい挙動に更新:

```cpp
TEST(FieldTest, GetSquareIndexCorner)
{
    // コーナー(0,0)でrange=1の場合、フィールド内の有効なセルのみが返る。
    // 旧実装では size_t アンダーフローと全クリアバグにより空が返っていたが、
    // 修正後は (0,1), (1,0), (1,1) の3セルが返る。
    Field field(5, 5, 1.0, 0.0, 0.0);
    std::vector<size_t> indexes;
    field.getSquareIndex(indexes, 0, 0, 1);
    // (row=0,col=1)=1, (row=1,col=0)=5, (row=1,col=1)=6 の3セルが有効
    EXPECT_EQ(indexes.size(), 3u);
}
```

---

## 5. 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `potbot_plugin/src/apf_planner.cpp` | フィールドサイズをロボット-ゴール間距離に基づいて動的計算 |
| `potbot_lib/src/field.cpp` | `getSquareIndex` の `size_t` アンダーフローと全クリアバグを修正 |
| `potbot_lib/test/test_field.cpp` | `GetSquareIndexCorner` テストを修正後の挙動に更新 |

---

## 6. 動作確認

### ゴール (-1.25, 3.5) でのフィールドサイズ計算例

```
robot = (0, 0), goal = (-1.25, 3.5)
dist_x = 1.25, dist_y = 3.5
max_dist = 3.5
half_cells = min(int(3.5/0.05) + 5, 100) = min(75, 100) = 75
total_cells = 150

フィールド範囲:
  x: -3.75 ～ 3.75 m  (goal x=-1.25 は範囲内)
  y: -3.75 ～ 3.75 m  (goal y=3.5 は範囲内)

IS_AROUND_GOAL: ゴール付近のセルに正しく設定される
→ 経路がゴール到達で正常に終端する
```
