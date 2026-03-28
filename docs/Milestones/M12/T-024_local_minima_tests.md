# T-024: 局所解回避テスト

## 概要
`test_apf_local_minima.cpp`を新規作成し、仮想障害物+渦巻き力による局所解回避を3シナリオで検証。

## テストシナリオ
1. **BehindSingleObstacle**: 障害物背後のゴールへの回り込み
2. **NarrowPassage**: 2障害物間の狭路通過
3. **ValleyBetweenObstacles**: 3障害物の谷間からの脱出
4. **ZeroVortexAngleNoCrash**: vortex_angle=0でクラッシュしないことを確認
5. **CreatePathDispatchVirtualObstacle**: createPath()ディスパッチの動作確認
6. **DijkstraFallback**: max_escape_attempts=0でDijkstraフォールバック確認

## 共通設定
- `vortex_angle`: π/4
- `escape_method`: "virtual_obstacle_vortex"
- `max_escape_attempts`: 3
- `virtual_obstacle_lifetime`: 1
