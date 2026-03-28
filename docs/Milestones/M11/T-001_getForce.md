# T-001: ArtificialPotentialField::getForce() 追加

| 項目 | 内容 |
|---|---|
| チケット番号 | T-001 |
| タイトル | ArtificialPotentialFieldにgetForceメソッドを追加 |
| マイルストーン | M11 |
| ステータス | 完了 |

---

## 概要

`ArtificialPotentialField` クラスにグリッド非依存の解析的APF力ベクトル計算メソッド `getForce()` を追加する。

既存の `createPotentialField()` はグリッド全体のポテンシャルを事前計算するバッチ処理であり、リアルタイム制御ループには不向きである。`getForce()` は任意のロボット位置・ターゲット位置に対して引力と斥力を直接計算し、制御に使用できる力ベクトルを返す。

---

## 変更ファイル

- `potbot_lib/include/potbot_lib/artificial_potential_field.hpp`
- `potbot_lib/src/artificial_potential_field.cpp`

---

## インターフェース

```cpp
void getForce(double rx, double ry,
              double target_x, double target_y,
              double& fx, double& fy) const;
```

### 引数

| 引数 | 型 | 説明 |
|---|---|---|
| `rx` | `double` | ロボットのx位置 [m] |
| `ry` | `double` | ロボットのy位置 [m] |
| `target_x` | `double` | 引力ターゲット（waypoint）のx位置 [m] |
| `target_y` | `double` | 引力ターゲット（waypoint）のy位置 [m] |
| `fx` | `double&` | 出力: x方向合力 [N相当] |
| `fy` | `double&` | 出力: y方向合力 [N相当] |

---

## 数式

### 引力

```
F_att_x = weight_attraction_field_ * (target_x - rx)
F_att_y = weight_attraction_field_ * (target_y - ry)
```

### 斥力（各障害物 i に対して）

```
d_i = sqrt((rx - ox_i)^2 + (ry - oy_i)^2)
if d_i <= distance_threshold_repulsion_field_:
    coeff = weight_repulsion_field_ * (1/(d_i+eps) - 1/(dth+eps)) / (d_i+eps)^3
    F_rep_i_x = coeff * (rx - ox_i)
    F_rep_i_y = coeff * (ry - oy_i)
```

### 合力

```
fx = F_att_x + sum(F_rep_i_x)
fy = F_att_y + sum(F_rep_i_y)
```

`eps = 1e-100`（ゼロ除算防止）

---

## 使用する既存メンバ変数

| メンバ変数 | 型 | 用途 |
|---|---|---|
| `weight_attraction_field_` | `double` | 引力重み |
| `weight_repulsion_field_` | `double` | 斥力重み |
| `distance_threshold_repulsion_field_` | `double` | 斥力有効距離 [m] |
| `obstacles_` | `std::vector<Point>` | 登録済み障害物リスト |
