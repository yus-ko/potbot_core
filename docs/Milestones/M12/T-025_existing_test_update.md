# T-025: 既存テスト修正

## 概要
`createPath()`の内部動作変更に伴い、既存テストの名称とコメントを修正。

## 変更内容
- `test_apf_path_planner.cpp`
  - `CreatePathUsesDijkstraFirst` → `CreatePathProducesValidPath`にリネーム
  - コメントを「Dijkstra優先」から「適切な手法にディスパッチ」に修正
  - テストロジック自体は変更なし（createPath()は最終的にパスを生成する）
