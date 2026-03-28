# T-020: GridInfo拡張

## 概要
`IS_VIRTUAL_OBSTACLE`をGridInfo enumに追加し、仮想障害物をグリッド上で識別可能にする。

## 変更内容
- `potbot_lib/include/potbot_lib/field.hpp`
  - `GridInfo` enumに`IS_VIRTUAL_OBSTACLE = 8`を追加
  - `FieldGrid.states`のデフォルト初期化を8要素→9要素に拡張
