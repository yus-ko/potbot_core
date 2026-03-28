# M12: 仮想障害物 + 渦巻き力による局所解回避

## 背景

APF経路計画は局所解に陥りやすく、従来の脱出手法（ランダム探索・壁沿い走行）では障害物背後・狭路・谷間の全パターンで失敗していた。そのため、ほぼ全てのケースでDijkstraフォールバックが使用され、APFの独自価値が活きていなかった。

## 解決策

2つの手法を組み合わせてAPFの局所解問題を解消する:

1. **仮想障害物**: 局所解検出時にそのセルに仮想障害物を配置し、斥力でロボットを押し出す。ポテンシャル場を再計算して勾配降下を再開する。
2. **渦巻き力**: 斥力ベクトルをゴール方向に回転させ、障害物周囲を回り込む力を常時発生させる。

## 処理フロー

```
createPath()
  ├─ escape_method による分岐
  │   ├─ "virtual_obstacle_vortex" → createPathWithVirtualObstacle()
  │   ├─ "random_weighted" → createPathWithWeight() (既存)
  │   └─ "wall_following" → createPathWallFollowing() (既存)
  └─ 失敗時 → createPathDijkstra() (最終フォールバック)
```

## 新規パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `escape_method` | string | `"virtual_obstacle_vortex"` | 局所解脱出戦略 |
| `vortex_angle` | double | `0.785` (π/4) | 渦巻き力の回転角度 |
| `virtual_obstacle_lifetime` | int | `1` | 仮想障害物の生存ステップ数 |
| `max_escape_attempts` | int | `3` | 最大再試行回数 |

## チケット一覧

| チケット | タイトル | 状態 |
|---|---|---|
| [T-020](T-020_grid_info_extension.md) | GridInfo拡張 | 完了 |
| [T-021](T-021_virtual_obstacle_vortex_apf.md) | APF仮想障害物 + 渦巻き力 | 完了 |
| [T-022](T-022_escape_method_planner.md) | Planner新脱出メソッド + ディスパッチ | 完了 |
| [T-023](T-023_plugin_params.md) | Plugin新ROSパラメータ | 完了 |
| [T-024](T-024_local_minima_tests.md) | 局所解回避テスト | 完了 |
| [T-025](T-025_existing_test_update.md) | 既存テスト修正 | 完了 |
| [T-026](T-026_documentation.md) | ドキュメント | 完了 |

## 変更ファイル一覧

| ファイル | 変更種別 |
|---|---|
| `potbot_lib/include/potbot_lib/field.hpp` | 修正 |
| `potbot_lib/include/potbot_lib/artificial_potential_field.hpp` | 修正 |
| `potbot_lib/src/artificial_potential_field.cpp` | 修正 |
| `potbot_lib/include/potbot_lib/apf_path_planner.hpp` | 修正 |
| `potbot_lib/src/apf_path_planner.cpp` | 修正 |
| `potbot_plugin/include/potbot_plugin/apf_planner.hpp` | 修正 |
| `potbot_plugin/src/apf_planner.cpp` | 修正 |
| `potbot_lib/test/test_apf_local_minima.cpp` | 新規 |
| `potbot_lib/test/test_apf_path_planner.cpp` | 修正 |
| `potbot_lib/CMakeLists.txt` | 修正 |
