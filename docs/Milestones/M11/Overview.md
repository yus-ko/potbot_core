# M11 マイルストーン概要仕様書 — ハイブリッド A* + APF waypointコントローラー実装

| 項目 | 内容 |
|---|---|
| マイルストーン | M11 |
| タイトル | ハイブリッド A* + APF waypointコントローラー実装 |
| ステータス | 実装完了 |
| 完了日 | 2026-03-28 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

APFナビゲーションの局所解問題を解決するため、「グローバルパスのwaypoint切り替え + APF斥力」ハイブリッドコントローラーを実装する。

従来の `ArtificialPotentialField` はグリッドベースのポテンシャルフィールドを事前計算するため、リアルタイムの局所制御への適用が困難であった。また、グローバルプランナーが生成したwaypointを引力ターゲットとして使用しないため、局所解に陥った場合に脱出できない問題があった。

本マイルストーンでは以下を実現する：
- グローバルパスのwaypointをAPF引力ターゲットとして使用
- waypointに到達したら次インデックスへ自動進行
- APF斥力でリアルタイム障害物回避
- Nav2 Controller プラグインとして統合

---

## 2. 問題の背景

### 局所解問題

APFアルゴリズムは障害物の手前で局所解（ポテンシャルの谷）に陥ることがある。既存の `ArtificialPotentialField::createPotentialField()` はグリッド全体を計算するバッチ処理であり、毎制御ループで呼び出すことは計算コストの観点から現実的でない。

### 解析的力計算の不在

従来実装にはグリッドを使わない解析的なAPF力ベクトル計算メソッドが存在せず、リアルタイム制御への応用が困難であった。

---

## 3. 解決策の概要

1. `ArtificialPotentialField::getForce()` を追加し、グリッド非依存の解析的APF力ベクトルをリアルタイムで計算できるようにする。
2. `ApfWaypointController`（potbot_lib）を新規実装し、グローバルパスのwaypoint追従とAPF斥力回避を組み合わせた制御器を提供する。
3. `HybridApfController`（potbot_plugin）を Nav2 Controller プラグインとして実装し、Nav2パイプラインに統合する。

---

## 4. チケット一覧

### T-001: ArtificialPotentialField::getForce() 追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| タイトル | ArtificialPotentialFieldにgetForceメソッドを追加 |
| ステータス | 完了 |

詳細は [T-001_getForce.md](T-001_getForce.md) を参照。

---

### T-002: ApfWaypointController クラス実装 (potbot_lib)

| 項目 | 内容 |
|---|---|
| チケット番号 | T-002 |
| タイトル | ApfWaypointControllerクラスをpotbot_libに追加 |
| ステータス | 完了 |

詳細は [T-002_apf_waypoint_controller.md](T-002_apf_waypoint_controller.md) を参照。

---

### T-003: テスト追加 (potbot_lib)

| 項目 | 内容 |
|---|---|
| チケット番号 | T-003 |
| タイトル | ApfWaypointControllerのgtestを追加 |
| ステータス | 完了 |

詳細は [T-003_lib_tests.md](T-003_lib_tests.md) を参照。

---

### T-004: HybridApfController Nav2 Controllerプラグイン実装

| 項目 | 内容 |
|---|---|
| チケット番号 | T-004 |
| タイトル | HybridApfControllerをNav2プラグインとして追加 |
| ステータス | 完了 |

詳細は [T-004_hybrid_apf_controller_plugin.md](T-004_hybrid_apf_controller_plugin.md) を参照。

---

### T-005: potbot_plugin テスト追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-005 |
| タイトル | HybridApfControllerのgtestを追加 |
| ステータス | 完了 |

---

## 5. 変更対象ファイル

| ファイル | 変更種別 | 内容 |
|---|---|---|
| `potbot_lib/include/potbot_lib/artificial_potential_field.hpp` | 修正 | `getForce()` メソッドを追加 |
| `potbot_lib/src/artificial_potential_field.cpp` | 修正 | `getForce()` の実装を追加 |
| `potbot_lib/include/potbot_lib/apf_waypoint_controller.hpp` | 新規 | `ApfWaypointController` クラス定義 |
| `potbot_lib/src/apf_waypoint_controller.cpp` | 新規 | `ApfWaypointController` 実装 |
| `potbot_lib/CMakeLists.txt` | 修正 | `apf_waypoint_controller.cpp` とテストを追加 |
| `potbot_lib/test/test_apf_waypoint_controller.cpp` | 新規 | gtest 9ケース |
| `potbot_plugin/include/potbot_plugin/hybrid_apf_controller.hpp` | 新規 | `HybridApfController` クラス定義 |
| `potbot_plugin/src/hybrid_apf_controller.cpp` | 新規 | `HybridApfController` 実装 |
| `potbot_plugin/plugin_nav2_core.xml` | 修正 | `HybridApfController` のプラグイン登録を追加 |
| `potbot_plugin/CMakeLists.txt` | 修正 | `hybrid_apf_controller.cpp` とテストを追加 |
| `potbot_plugin/test/test_hybrid_apf_controller.cpp` | 新規 | gtest 6ケース |

---

## 6. 完了基準

| # | 完了基準 | 状態 |
|---|---|---|
| 1 | `getForce()` が障害物なしのとき引力方向の力を返すこと | 完了 |
| 2 | `getForce()` が障害物ありのとき斥力を加算すること | 完了 |
| 3 | `ApfWaypointController::computeCommand()` が速度制限内の出力を返すこと | 完了 |
| 4 | waypoint追従が機能し、500ステップ以内にゴール到達すること | 完了 |
| 5 | `HybridApfController` が `nav2_core::Controller` として登録されること | 完了 |
| 6 | potbot_lib の全テストがパスすること | 完了 |
| 7 | potbot_plugin のビルドが成功すること | 完了 |
