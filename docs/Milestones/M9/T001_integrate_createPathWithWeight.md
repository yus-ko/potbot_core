# M9: T-001 createPathWithWeight の Nav2 統合

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| マイルストーン | M9 |
| タイトル | createPathWithWeight の Nav2 統合 |
| ステータス | 実装中 |

### 目的

`APFPathPlanner::createPathWithWeight()` を Nav2 プランナープラグイン（`potbot_nav::planner::APF`）から呼び出せるようにし、YAML パラメータファイルで経路計画手法を切り替えられるようにする。

---

## 2. 問題の詳細

### 問題 1: `setParams()` が呼び出されていない

**場所:** `potbot_plugin/src/apf_planner.cpp` — `configure()` および `createPlan()`

`APFPathPlanner::setParams(double maxp, size_t sr, double wpot, double wpos)` は以下の4フィールドを設定する：

| フィールド | 意味 | デフォルト値 |
|---|---|---|
| `max_path_length_` | 経路の最大長さ [m] | 6.0 |
| `path_search_range_` | 局所探索の範囲（セル数） | 1 |
| `path_weight_potential_` | 局所解脱出時のポテンシャル重み | 0.0 |
| `path_weight_pose_` | 局所解脱出時の姿勢差分重み | 1.0 |

現状の `apf_planner.cpp` では `setParams()` が一度も呼ばれていないため、`createPathWithWeight()` を呼び出しても重みパラメータがデフォルト値のままになる。`weight_potential=0.0` のデフォルト値ではポテンシャル項が完全に無効化され、局所解脱出の挙動が意図した通りにならない。

### 問題 2: `planning_method` が実装されていない

**場所:** `potbot_plugin/src/apf_planner.cpp` — `createPlan()`

現状の `createPlan()` は `planner->createPath()` を固定で呼び出している。`createPath()` の内部では Dijkstra 法を試み、失敗時に APF 勾配降下法にフォールバックするが、`createPathWithWeight()` は呼び出し経路に存在しない。Nav2 の YAML パラメータから手法を切り替える手段がない。

### 問題 3: YAML 設定が存在しない

**場所:** `test/config/waffle_pi.yaml` — `planner_server.GridBased`

現在の設定は以下のみ：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "potbot_nav::planner::APF"
```

`planning_method`・`weight_potential`・`weight_pose`・`path_search_range`・`max_path_length` の各パラメータエントリが存在しないため、`configure()` でパラメータを宣言・取得しても実行時にデフォルト値しか取れない。

---

## 3. createPathWithWeight のアルゴリズム説明

`createPathWithWeight()` は APF ポテンシャルフィールド上で勾配降下法を基本としながら、局所解を検出した場合に重み付き評価関数で脱出を試みるアルゴリズムである。

### 通常モード（勾配降下）

局所解に陥っていないと判定されている間は、探索範囲内の未訪問セルのうちポテンシャル値 `P` が最小のセルへ移動する。

```
移動先 = argmin { P(idx) | idx ∈ 探索範囲, 未訪問 }
```

### 局所解検出

1ステップの探索で探索範囲内のすべての未訪問セルのポテンシャルが現在位置以上になった場合、`solving_local_minimum = true` に設定され脱出モードへ移行する。

### 局所解脱出モード（重み付き探索）

局所解状態では、最大100回のランダム範囲探索を行い、以下のコスト関数 `J` が最小のセルを選択する：

```
J = wu * (P / (P + Δθ)) + w_theta * (Δθ / (P + Δθ))
```

| 変数 | 意味 |
|---|---|
| `wu` | `weight_potential`（ポテンシャル重み） |
| `w_theta` | `weight_pose`（姿勢差分重み） |
| `P` | 移動候補セルの APF ポテンシャル値 |
| `Δθ` | 前ステップの進行方向と候補セル方向の角度差（絶対値） |

この評価関数により、局所解脱出時はポテンシャルの低減だけでなく進行方向の連続性も考慮した経路選択が行われる。`solving_local_minimum` は `J ≤ J_min` を満たすセルが見つかった時点で `false` に戻り、通常モードへ復帰する。

---

## 4. 解決策の設計

### 4.1 apf_planner.hpp への追加

`APF` クラスのプライベートメンバに以下を追加する：

```cpp
std::string planning_method_;   // "dijkstra" or "weight"
double max_path_length_;
int path_search_range_;
double weight_potential_;
double weight_pose_;
```

### 4.2 apf_planner.cpp の configure() への追加

`configure()` 内で以下のパラメータを宣言・取得する：

```cpp
nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".planning_method", rclcpp::ParameterValue(std::string("dijkstra")));
nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_path_length", rclcpp::ParameterValue(6.0));
nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".path_search_range", rclcpp::ParameterValue(1));
nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".weight_potential", rclcpp::ParameterValue(1.0));
nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".weight_pose", rclcpp::ParameterValue(1.0));

node_->get_parameter(name_ + ".planning_method", planning_method_);
node_->get_parameter(name_ + ".max_path_length", max_path_length_);
node_->get_parameter(name_ + ".path_search_range", path_search_range_);
node_->get_parameter(name_ + ".weight_potential", weight_potential_);
node_->get_parameter(name_ + ".weight_pose", weight_pose_);
```

### 4.3 apf_planner.cpp の createPlan() への変更

`planner->createPath()` の呼び出し前に `setParams()` を追加し、`planning_method_` に応じて呼び出しメソッドを分岐する：

```
planner->setParams(max_path_length_, path_search_range_, weight_potential_, weight_pose_)

if planning_method_ == "weight":
    planner->createPathWithWeight()
else:
    planner->createPath()   // Dijkstra優先（M8実装）
```

### 4.4 waffle_pi.yaml への追加

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "potbot_nav::planner::APF"
      planning_method: "weight"   # "dijkstra" or "weight"
      max_path_length: 6.0
      path_search_range: 1
      weight_potential: 1.0
      weight_pose: 1.0
```

---

## 5. 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `potbot_plugin/include/potbot_plugin/apf_planner.hpp` | `planning_method_`・`max_path_length_`・`path_search_range_`・`weight_potential_`・`weight_pose_` メンバ変数を追加 |
| `potbot_plugin/src/apf_planner.cpp` | `configure()` にパラメータ宣言・取得を追加、`createPlan()` で `setParams()` 呼び出しと `planning_method_` による分岐を追加 |
| `test/config/waffle_pi.yaml` | `GridBased` プラグインに `planning_method`・重み・範囲パラメータを追記 |

---

## 6. 動作確認方法

### docker compose パイプラインでの確認

```bash
# Docker環境を起動
cd /home/rtx3090/potbot/ros2_ws/src/potbot_core/test/
docker-compose up gazebo domain_bridge potbot

# planning_method: weight 設定で /plan トピックを確認
ros2 topic echo /plan
```

**期待する結果:**
- `planning_method: weight` 設定時: `createPathWithWeight()` が呼ばれ、局所解に陥る場面でも重み付き探索で脱出した経路が生成される
- `planning_method: dijkstra` 設定時: M8 実装の Dijkstra 優先経路が生成される

### パラメータ反映の確認

```bash
# ノード起動後にパラメータ値を確認
ros2 param get /planner_server GridBased.planning_method
ros2 param get /planner_server GridBased.weight_potential
ros2 param get /planner_server GridBased.weight_pose
```

**期待する結果:** `waffle_pi.yaml` に記載した値が返ること
