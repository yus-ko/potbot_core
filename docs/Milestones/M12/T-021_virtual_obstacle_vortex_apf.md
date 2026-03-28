# T-021: APF仮想障害物 + 渦巻き力

## 概要
`ArtificialPotentialField`クラスに仮想障害物管理機能と渦巻き力（斥力ベクトル回転）を追加。

## 仮想障害物
- `VirtualObstacle`構造体: `{x, y, lifetime}`
- `addVirtualObstacle(x, y, lifetime)`: 仮想障害物を追加しグリッドフラグを設定
- `clearVirtualObstacles()`: 全仮想障害物を除去
- `decrementVirtualObstacleLifetimes()`: lifetime減算、0以下で自動除去
- `createPotentialField()`: 実障害物と同様に斥力計算に参加

## 渦巻き力
- `vortex_angle_`パラメータ（デフォルト0.0）
- `getForce()`内で斥力合計ベクトルをゴール方向に回転
- 回転方向: ゴール角度と斥力角度の差分で自動判定
- `vortex_angle_==0`の場合は回転なし（既存互換）

## 回転ロジック
```
goal_angle = atan2(goal_y - robot_y, goal_x - robot_x)
rep_angle = atan2(f_rep_y, f_rep_x)
angle_diff = normalize(goal_angle - rep_angle)
sign = (angle_diff >= 0) ? +1 : -1
theta = sign * vortex_angle_
[f_rep_x', f_rep_y'] = R(theta) * [f_rep_x, f_rep_y]
```
