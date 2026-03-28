# T-023: Plugin新ROSパラメータ

## 概要
Nav2プラグイン`APF`に新しいROSパラメータを追加し、仮想障害物+渦巻き力の設定をYAMLから行えるようにする。

## 追加パラメータ
| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `escape_method` | string | `"virtual_obstacle_vortex"` | 局所解脱出戦略 |
| `vortex_angle` | double | `0.785` | 渦巻き力の回転角度(rad) |
| `virtual_obstacle_lifetime` | int | `1` | 仮想障害物の生存ステップ数 |
| `max_escape_attempts` | int | `3` | 仮想障害物の最大再試行回数 |

## YAML設定例
```yaml
planner_server:
  ros__parameters:
    GridBased.escape_method: "virtual_obstacle_vortex"
    GridBased.vortex_angle: 0.785
    GridBased.virtual_obstacle_lifetime: 1
    GridBased.max_escape_attempts: 3
```
