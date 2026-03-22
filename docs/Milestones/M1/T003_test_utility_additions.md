# M1: T-003 test_utility 追加テスト仕様書

## 1. チケット概要

| 項目 | 内容 |
|---|---|
| チケット番号 | T-003 |
| マイルストーン | M1 |
| タイトル | test_utility に未テスト関数のテストを追加 |
| コミット | `e04773a` |
| ステータス | 完了 |

### 目的

M0 で作成した `test_utility.cpp` にカバーされていなかった関数群（`get_vec`・`get_index`・`vec_to_path`・`bezier` 高レベル関数・`Pose::to_affine`・`Point::to_rotation`・`is_containing`）のテストを追加し、`utility.hpp` の全関数を網羅的に検証する。

### 背景

M0 の `test_utility.cpp` は `Point`・`Pose` の構造体演算と基本的な utility 関数（`combination`・`get_rotate_matrix`・`contains`・`find_closest_vector`）をカバーしていた。しかし `get_vec`・`get_index`・`vec_to_path`・`bezier`（高レベル）・`Pose::to_affine`・`Point::to_rotation`・`is_containing` などの関数はテストされていなかったため、M1 にて追加した。

---

## 2. 実装内容

- `potbot_lib/test/test_utility.cpp` に 22 件のテストケースを追加（M0: 27件 → M1: 49件）
- 追加テスト対象: `utility::get_vec(Point)`、`utility::get_vec(Pose)`、`utility::get_index`、`utility::vec_to_path`、`utility::bezier`（高レベル）、`Pose::to_affine`、`Point::to_rotation`、`utility::is_containing`

---

## 3. テストケース一覧（追加分）

### テストスイート: `UtilityGetVecTest`（`utility::get_vec` テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 1 | `GetVecPointEmptyInput` | Point の空ベクター入力で結果が空になること |
| 2 | `GetVecPointSingleElement` | Point 1 個の入力で Eigen::Vector3d が 1 個返ること |
| 3 | `GetVecPointSize` | Point 3 個の入力で結果サイズが 3 になること |
| 4 | `GetVecPoseEmptyInput` | Pose の空ベクター入力で結果が空になること |
| 5 | `GetVecPoseSize` | Pose 2 個の入力で結果サイズが 2 になること |
| 6 | `GetVecPoseTranslation` | Pose の `get_vec()` が Affine3d の translation に正しい値を返すこと |

### テストスイート: `UtilityGetIndexTest`（`utility::get_index` テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 7 | `GetIndexFound` | 存在する Vector2d のインデックスが正しく返ること |
| 8 | `GetIndexNotFound` | 存在しない Vector2d に対して -1 が返ること |
| 9 | `GetIndexFirstElement` | 先頭要素のインデックスが 0 を返ること |

### テストスイート: `UtilityVecToPathTest`（`utility::vec_to_path` テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 10 | `VecToPathEmptyInput` | 空ベクター入力で path が空になること |
| 11 | `VecToPathSingleElement` | 1 要素入力で path に 1 点が追加され、x・y が正しく設定されること |

### テストスイート: `UtilityBezierHighLevelTest`（`utility::bezier` 高レベル関数テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 12 | `BezierHighLevelEmptyPath` | 空の path 入力で false が返ること |
| 13 | `BezierHighLevelSinglePoint` | 1 点の path 入力で false が返ること |
| 14 | `BezierHighLevelMultiplePoints` | 3 点以上の path 入力で true が返り、補間後の path が空でないこと |

### テストスイート: `PoseAffineTest`（`Pose::to_affine` テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 15 | `ToAffineTranslation` | `Pose(1,2,3,...)` の `to_affine()` で translation が (1,2,3) になること |
| 16 | `ToAffineIdentityRotation` | ゼロ回転の Pose で `to_affine()` の回転成分が単位行列になること |

### テストスイート: `PointRotationTest`（`Point::to_rotation` テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 17 | `ToRotationOrthogonal` | `Point(0, 0, π/4)` の `to_rotation()` が直交行列（R×R.T ≈ I）であること |
| 18 | `ToRotationDeterminantOne` | `to_rotation()` の行列式が 1 であること（回転行列の条件） |

### テストスイート: `UtilityIsContainingTest`（`utility::is_containing` テスト）

| # | テストケース名 | テスト内容 |
|---|---|---|
| 19 | `IsContainingTrue` | `is_containing(3, {1,2,3,4})` が true を返すこと |
| 20 | `IsContainingFalse` | `is_containing(5, {1,2,3,4})` が false を返すこと |
| 21 | `IsContainingEqualsContains` | `is_containing()` と `contains()` が同じ結果を返すこと |

### テストスイート: `UtilityGetVecAffineTest`

| # | テストケース名 | テスト内容 |
|---|---|---|
| 22 | `GetVecPoseAffineTranslation` | 複数の Pose を `get_vec()` で変換して Affine3d の translation が正しいこと |

**追加テスト合計: 22 件（M0 の 27 件と合わせて計 49 件）**

---

## 4. ビルド・実行コマンド

```bash
# テスト付きビルド（ワークスペースルートで実行）
cd ros2_ws
colcon build --packages-select potbot_lib --cmake-args -DBUILD_TESTING=ON

# 全テスト実行
ctest --test-dir build/potbot_lib

# このファイルのみ実行
ctest --test-dir build/potbot_lib -R test_utility
```

---

## 5. 対応ファイル

- `potbot_lib/test/test_utility.cpp` — テスト本体（追加）
- `potbot_lib/CMakeLists.txt` — 変更なし（M0 で設定済み）
