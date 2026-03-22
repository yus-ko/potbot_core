# M9 マイルストーン概要仕様書 — createPathWithWeight Nav2統合

| 項目 | 内容 |
|---|---|
| マイルストーン | M9 |
| タイトル | createPathWithWeight Nav2統合 |
| ステータス | 実装中 |
| 完了日 | 2026-03-22 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M9 は `APFPathPlanner::createPathWithWeight()` を Nav2 パイプラインで実際に使用できるよう統合するマイルストーンである。

M8 で Dijkstra 法による経路計画（`createPathDijkstra()`）を実装し、`createPath()` の主経路として組み込んだ。一方、`createPathWithWeight()` は APF ポテンシャル勾配降下法をベースとしながら局所解検出時に重み付き探索で脱出を試みるアルゴリズムとして実装済みであるが、Nav2 プランナープラグイン（`APF` クラス）から呼び出されていない状態であった。

本マイルストーンでは `planning_method` パラメータを導入し、`dijkstra`・`weight` の2方式を YAML で切り替えられるようにすることを目的とする。

---

## 2. 問題の背景

### createPathWithWeight が Nav2 パイプラインに接続されていなかった

`potbot_plugin/src/apf_planner.cpp` の `createPlan()` は `planner->createPath()` を呼び出しており、`createPathWithWeight()` は呼び出し経路に存在しない。さらに以下の3点が未接続の原因となっていた。

**問題 1: `setParams()` が呼び出されていない**

`APFPathPlanner::setParams()` は `max_path_length_`・`path_search_range_`・`path_weight_potential_`・`path_weight_pose_` の4パラメータを設定するメソッドである。`apf_planner.cpp` の `configure()` および `createPlan()` でこのメソッドが呼び出されておらず、`createPathWithWeight()` で使用される重みパラメータがデフォルト値（`weight_potential=0.0`, `weight_pose=1.0`）のままとなっていた。

**問題 2: `planning_method` パラメータが実装されていない**

`createPath()`・`createPathWithWeight()`・`createPathDijkstra()` の3メソッドが存在するが、どの手法を使うかを外部から切り替える仕組みがない。Nav2 の YAML パラメータファイルから手法を選択できない。

**問題 3: YAML 設定が存在しない**

`test/config/waffle_pi.yaml` の `planner_server` セクションに `GridBased` プラグインの設定はあるが、`planning_method`・`weight_potential`・`weight_pose` などのパラメータエントリが存在しない。

---

## 3. 解決策の概要

`apf_planner.cpp` の `configure()` に `planning_method`・`weight_potential`・`weight_pose`・`path_search_range`・`max_path_length` の各パラメータを宣言・取得するコードを追加する。`createPlan()` では取得した `planning_method` の値に応じて `planner->createPath()` または `planner->createPathWithWeight()` を呼び分ける。あわせて `planner->setParams()` を呼び出してパラメータを反映させる。

`waffle_pi.yaml` には `GridBased` プラグインのサブキーとしてこれらのパラメータを追記する。

---

## 4. チケット一覧

### T-001: createPathWithWeight の Nav2 統合

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| タイトル | createPathWithWeight の Nav2 統合 |
| ステータス | 実装中 |

詳細は [T001_integrate_createPathWithWeight.md](T001_integrate_createPathWithWeight.md) を参照。

---

## 5. 変更対象ファイル

| ファイル | 変更種別 | 内容 |
|---|---|---|
| `potbot_plugin/include/potbot_plugin/apf_planner.hpp` | 修正 | `planning_method_` など新規メンバ変数を追加 |
| `potbot_plugin/src/apf_planner.cpp` | 修正 | `configure()` にパラメータ宣言・取得を追加、`createPlan()` で `setParams()` 呼び出しと `planning_method` 分岐を追加 |
| `test/config/waffle_pi.yaml` | 修正 | `GridBased` プラグインに `planning_method`・重み・範囲パラメータを追記 |

---

## 6. 完了基準

| # | 完了基準 | 状態 |
|---|---|---|
| 1 | `planning_method: weight` を YAML に設定すると `createPathWithWeight()` が呼ばれること | 実装中 |
| 2 | `planning_method: dijkstra` を YAML に設定すると `createPathDijkstra()` を優先する従来の `createPath()` が呼ばれること | 実装中 |
| 3 | `weight_potential`・`weight_pose` の値が `setParams()` 経由で反映されること | 実装中 |
| 4 | docker compose パイプラインで経路が正常に生成されること | 実装中 |
