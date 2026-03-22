# M8: T-001 Dijkstra経路計画でAPF局所解問題を解消

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| マイルストーン | M8 |
| タイトル | Dijkstra経路計画でAPF局所解問題を解消 |
| ステータス | 実装中 |

### 目的

APF の勾配降下法が持つ構造的な局所解問題を、Dijkstra 法を `createPathDijkstra()` として実装することで根本的に解消する。`createPath()` の主要経路探索手法を Dijkstra 法に切り替え、ゴールが壁の向こう側にある場合でも確実に経路を生成できるようにする。

---

## 2. 根本原因の詳細分析

### 問題 1: APF 勾配降下法の局所解

**場所:** `potbot_lib/src/apf_path_planner.cpp` — `createPath()` メソッド

**現在の動作:**

`createPath()` は各ステップで近傍セルのうちポテンシャルが最も低いセルを選択して移動する（勾配降下法）。APF のポテンシャルは以下の合成である：

- **引力ポテンシャル**: ゴール方向に単調減少
- **斥力ポテンシャル**: 障害物から離れるほど減少

凹型障害物・コの字型通路・狭い廊下の入口などでは、引力と斥力が釣り合う「局所最小値」がゴール以外の場所に生じる。勾配降下法はこの点で停滞し、経路探索が終了しない。

**問題のケース例:**
```
ロボット(S)           ゴール(G)
     S    [壁壁壁]    G
          [  入口 ]
```
- `S` から見て `G` への引力は強いが、壁の斥力で直進できない
- 入口方向のポテンシャルが局所最小値となり、そこで停滞する

### 問題 2: 壁エッジ追跡機構の脆弱性

**場所:** `potbot_lib/src/apf_path_planner.cpp` — `createPath()` の `no_progress_count` ロジック

**現在のコード（概要）:**
```cpp
// no_progress_count が閾値を超えたら壁エッジ追跡モードへ切り替え
if (no_progress_count > threshold) {
    // エッジ追跡処理
}
```

**問題:**
- `no_progress_count` の閾値が小さく、複雑形状の障害物では脱出できない
- エッジ追跡が正しい方向に収束する保証がなく、経路がループする可能性がある
- 複数の局所最小値が存在する場合、最初の局所解から抜け出しても次の局所解に捕まる

### 問題 3: `no_progress_count` が小さすぎる問題

狭い通路や複雑な形状の障害物群では、局所解から脱出するために多くのエッジ追跡ステップが必要になる。しかし `no_progress_count` の閾値が小さいと、真に局所解に捕まっていない段階でもエッジ追跡に移行し、誤った方向へ経路が延伸してしまう。

---

## 3. 解決策の設計

### Dijkstra 法の採用

Dijkstra 法はグラフ上の最短経路問題を**完全性を保証して**解くアルゴリズムである。スタートから全ノードへの最短コストを順次確定していくため、局所最小値に捕まることがない。

APF フィールドをグラフとみなし、以下のように辺コストを設計する：

**辺コスト関数:**
```
edge_cost = phys_dist * (1 + repulsion_cost)
```

| 変数 | 説明 |
|---|---|
| `phys_dist` | 隣接セル間の物理距離（直進=解像度、斜め=解像度×√2） |
| `repulsion_cost` | 移動先セルの APF 斥力ポテンシャル値 |

この設計により：
- 障害物から遠いセル（斥力小）は低コストで通過できる
- 障害物に近いセル（斥力大）は高コストとなり、安全なルートが優先される
- 障害物セル自体は `std::numeric_limits<double>::infinity()` として通過不可

### `createPathDijkstra()` の実装概要

**場所:** `potbot_lib/src/apf_path_planner.cpp`（新規メソッド）

```
1. スタートセルとゴールセルのインデックスを取得
2. 優先度付きキュー（min-heap）でコストが最小のセルから順に処理
3. 各セルから8近傍（直進4 + 斜め4）へ edge_cost を計算して展開
4. ゴールセルに到達したら親ポインタを逆にたどって経路を復元
5. 経路をセル座標列（x, y）に変換して返す
```

**宣言（ヘッダーへの追加）:**
```cpp
// potbot_lib/include/potbot_lib/apf_path_planner.h
void createPathDijkstra();
```

### `createPath()` でDijkstraを優先、失敗時フォールバック

`createPath()` のロジックを以下のように変更する：

```
1. createPathDijkstra() を試みる
2. Dijkstra が経路を返した場合 → その経路を採用
3. Dijkstra が失敗した場合（ゴール到達不可）→ 従来の APF 勾配降下法にフォールバック
```

従来の APF 勾配降下法はフォールバックとして残すことで、後方互換性と安全性を確保する。

---

## 4. 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `potbot_lib/include/potbot_lib/apf_path_planner.h` | `createPathDijkstra()` メソッドの宣言を追加 |
| `potbot_lib/src/apf_path_planner.cpp` | `createPathDijkstra()` の実装、`createPath()` の呼び出しロジックをDijkstra優先に変更 |

---

## 5. 動作確認方法

### ゴールが壁の向こう側にあるケースでの確認

```bash
# Docker環境を起動
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test/
docker-compose up gazebo domain_bridge potbot

# RViz でゴールを壁の向こう側に設定して /plan を確認
ros2 topic echo /plan
```

**期待する結果:**
- 従来: 経路が生成されないか、ループ状になる
- 修正後: 壁を迂回する正しい経路が生成される

### 単体テストによる確認

```bash
cd /home/rtx3090/potbot/ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/potbot_lib
```

**期待する結果:** 既存の全テストがパスすること

### 辺コスト関数の計算例

```
解像度 (resolution) = 0.05 m

直進移動（上下左右）:
  phys_dist = 0.05
  斥力なし (repulsion_cost = 0): edge_cost = 0.05 * (1 + 0) = 0.05
  斥力あり (repulsion_cost = 2): edge_cost = 0.05 * (1 + 2) = 0.15

斜め移動:
  phys_dist = 0.05 * √2 ≈ 0.0707
  斥力なし (repulsion_cost = 0): edge_cost ≈ 0.0707
  斥力あり (repulsion_cost = 2): edge_cost ≈ 0.212

障害物セル:
  edge_cost = infinity（通過不可）
```
