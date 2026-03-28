# T-022: Planner新脱出メソッド + ディスパッチ

## 概要
`APFPathPlanner`に仮想障害物による局所解脱出メソッドを追加し、`createPath()`をescape_methodベースのディスパッチに改修。

## 新メソッド
### `createPathWithVirtualObstacle()`
1. 勾配降下で最小ポテンシャルセルへ移動
2. 局所解検出（近傍に低ポテンシャルなし）→ 仮想障害物を配置
3. `createPotentialField()`でポテンシャル場を再計算
4. 勾配降下を再開（最大`max_escape_attempts_`回まで）
5. ゴール到達で`true`、失敗で`false`を返す

### `createPath()`ディスパッチ
```
"virtual_obstacle_vortex" → createPathWithVirtualObstacle()
"random_weighted"         → createPathWithWeight()
"wall_following"          → createPathWallFollowing()
失敗時                     → createPathDijkstra()
```

### `createPathWallFollowing()`
既存の勾配降下+壁沿い走行ロジックを抽出（旧`createPath()`本体）。

## 新パラメータ
- `escape_method_`: 脱出戦略（デフォルト: "virtual_obstacle_vortex"）
- `max_escape_attempts_`: 最大再試行回数（デフォルト: 3）
- `virtual_obstacle_lifetime_`: 仮想障害物の生存ステップ数（デフォルト: 1）
