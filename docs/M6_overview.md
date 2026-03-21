# M6 マイルストーン概要仕様書 — rosbag2 記録・解析パイプライン

| 項目 | 内容 |
|---|---|
| マイルストーン | M6 |
| タイトル | rosbag2 記録・解析パイプライン |
| ステータス | 完了 |
| 完了日 | 2026-03-22 |
| 担当 | claude |

---

## 1. マイルストーン概要・目的

M6 は potbot_core プロジェクトにおけるテスト結果の記録・可視化基盤を構築するマイルストーンである。

M5 で整備した E2E テストパイプラインを基盤として、以下の3点を達成する：

1. **Docker インフラ変更**（T-001）: Dockerfile に rosbags・matplotlib・numpy をインストールし、docker-compose.yml に rosbag-record サービスと rosbag-analysis サービスを追加する。
2. **Python 解析スクリプト作成**（T-002）: rosbags ライブラリで rosbag2 データを読み込み、移動軌跡と速度プロファイルを3パネル図として PNG に可視化する解析スクリプトを新規作成する。
3. **パイプライン統合**（T-003）: `run_navigation_test.sh` を修正し、rosbag2 記録の開始・停止・解析実行を一連のパイプラインとして統合する。

---

## 2. 背景

### M5 からの継続

M5 では Gazebo + Nav2 による E2E テストパイプラインを構築し、ナビゲーションの成否を自動検証できる体制を整えた。しかし以下の課題が残されていた：

- テスト実行時のロボット動作データ（軌跡・速度・角速度）を記録する仕組みがなかった。ナビゲーションの成否のみが判定可能であり、どのような経路を通ったか・どのような速度プロファイルであったかを事後確認できなかった。
- ナビゲーションの品質を定量的に評価する手段がなかった。たとえば経路の滑らかさや速度の変動を可視化してデバッグに活用することができなかった。

### rosbag2 記録・解析の必要性

- rosbag2 による記録を導入することで、テスト実行中のトピックデータ（/odom, /cmd_vel, /scan, /tf, /tf_static）を永続化できる。
- rosbags Python モジュールによるオフライン解析で、記録データを直接読み込み matplotlib で可視化できる。ROS 環境なしでもデータ分析が可能になる。
- 記録・解析をシェルスクリプトに統合することで、テスト実行→記録→解析→結果出力の一連のワークフローを自動化できる。

---

## 3. スコープ

### 対象ディレクトリ

| ディレクトリ | 対象 | 理由 |
|---|---|---|
| `test/` | **対象** | Docker 環境・解析スクリプト・シェルスクリプト・README すべてが test/ 配下 |
| `potbot_lib` | 対象外 | C++ コード変更なし |
| `potbot_ros` | 対象外 | C++ コード変更なし |
| `potbot_plugin` | 対象外 | C++ コード変更なし |
| `potbot_behavior_tree` | 対象外 | 変更なし |
| `potbot_msgs` | 対象外 | 変更なし |

### 変更対象ファイル

| ファイル | 変更種別 | 役割 |
|---|---|---|
| `test/Dockerfile` | 修正 | rosbags・matplotlib・numpy のインストール追加 |
| `test/docker-compose.yml` | 修正 | rosbag-record・rosbag-analysis サービス追加 |
| `test/.gitignore` | 修正 | `results/` ディレクトリを除外対象に追加 |
| `test/analyze_rosbag.py` | 新規 | rosbag2 データの可視化スクリプト |
| `test/run_navigation_test.sh` | 修正 | rosbag2 記録・停止・解析のパイプライン統合 |
| `test/README.md` | 修正 | rosbag 記録・解析セクション追記 |

---

## 4. チケット一覧

### T-001: Docker インフラ変更

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| タイトル | Dockerfile と docker-compose.yml に rosbag2 記録・解析基盤を追加 |
| コミット | `dfcd798` |
| ステータス | 完了 |

**実装内容:**

- `test/Dockerfile` に `pip3 install rosbags matplotlib numpy` を追加
- `test/docker-compose.yml` に `rosbag-record` サービスを追加（Domain 1 で /odom /cmd_vel /scan /tf /tf_static を記録）
- `test/docker-compose.yml` に `rosbag-analysis` サービスを追加（analysis プロファイル、Python 解析スクリプトを実行）
- `test/.gitignore` に `results/` を追加

---

### T-002: Python 解析スクリプト作成

| 項目 | 内容 |
|---|---|
| チケット番号 | T-002 |
| タイトル | rosbag2 データの可視化解析スクリプトを作成 |
| コミット | `9aaa663` |
| ステータス | 完了 |

**実装内容:**

- `test/analyze_rosbag.py` を新規作成
- rosbags ライブラリで rosbag2（sqlite3 形式）を読み込み
- /odom から (x, y) 座標を抽出し XY 軌跡をプロット
- /cmd_vel から linear.x, angular.z を抽出し時系列グラフを生成
- matplotlib で3パネル図（軌跡、線速度、角速度）を PNG 保存
- コマンドライン引数: `--bag-path`, `--output-dir`

---

### T-003: パイプライン統合

| 項目 | 内容 |
|---|---|
| チケット番号 | T-003 |
| タイトル | rosbag2 記録・解析をナビゲーションテストパイプラインに統合 |
| コミット | `49406f2` |
| ステータス | 完了 |

**実装内容:**

- `test/run_navigation_test.sh` を修正: rosbag2 記録のバックグラウンド開始→ナビゲーション実行→記録停止→解析実行→結果出力の一連のフローを実装
- `test/README.md` に rosbag 記録と解析のセクションを追加

---

## 5. 完了基準

M6 を完了とみなすための基準は以下の通りである。すべての項目が達成済みであることを確認した。

| # | 完了基準 | 状態 |
|---|---|---|
| 1 | Dockerfile に rosbags・matplotlib・numpy がインストールされること | 完了 |
| 2 | docker-compose.yml に rosbag-record サービスが追加されていること | 完了 |
| 3 | docker-compose.yml に rosbag-analysis サービスが追加されていること | 完了 |
| 4 | analyze_rosbag.py が存在し、3パネルの可視化を生成できること | 完了 |
| 5 | run_navigation_test.sh が rosbag 記録・停止・解析を一連のパイプラインとして実行すること | 完了 |
| 6 | README.md に rosbag 記録・解析の説明が追記されていること | 完了 |

---

## 6. 関連ファイル一覧

### Docker インフラ（T-001）

```
test/
├── Dockerfile                            # rosbags・matplotlib・numpy インストール追加
├── docker-compose.yml                    # rosbag-record・rosbag-analysis サービス追加
└── .gitignore                            # results/ を除外対象に追加
```

### Python 解析スクリプト（T-002）

```
test/
└── analyze_rosbag.py                     # rosbag2 データ可視化スクリプト（3パネル図生成）
```

### パイプライン統合（T-003）

```
test/
├── run_navigation_test.sh                # rosbag2 記録・停止・解析のパイプライン統合
└── README.md                             # rosbag 記録・解析セクション追記
```

### 本仕様書

```
docs/
└── M6_overview.md                        # 本ファイル（M6マイルストーン概要仕様書）
```
