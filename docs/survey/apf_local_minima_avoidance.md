# ポテンシャル法（APF）における局所解回避手法 サーベイ

**作成日:** 2026-03-28
**調査方法:** Web調査（論文・技術記事）

---

## 背景：APFと局所解問題

Khatib（1986）が提案したAPF（Artificial Potential Field）は、目標への引力とオブジェクトへの斥力を組み合わせたシンプルかつリアルタイム性に優れた経路計画手法である。しかし、引力と斥力がキャンセルし合う点（局所解）にロボットが捕捉される問題は、APFの本質的な欠陥として広く知られている。

主な発生条件：
- 障害物がロボットとゴールの間に挟まれた配置
- 障害物が密集した狭路（corridor）環境
- ゴールが届かない問題（GNRON: Goal Non-Reachable with Obstacle Nearby）

---

## 手法一覧

### 1. ランダム摂動法（Random Perturbation / Random Walk）

**原理:** 局所解に捕捉された際にランダム方向の力を一時的に付加する。Barraquand & Latombe（1991）が初期の提案。

**効果:**
- 単純な閉塞環境では有効だが収束保証なし（確率的）
- Wall-Followingとの比較でパス長が13〜48%長くなる傾向
- 複数ロボット実験（2024）ではHybrid APF + Wall-FollowerがランダムA渉単独を上回る成功率

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | なし（確率的） |
| 計算コスト | 極低 |
| 実装難易度 | 極低 |
| 動的環境対応 | 高 |

**主な文献:**
- Barraquand, J. & Latombe, J.C. (1991). "Robot Motion Planning: A Distributed Representation Approach." *International Journal of Robotics Research*, 10(6): 628–649.
- Ren, C. et al. (2024). "Improved artificial potential field method based on robot local path information." *International Journal of Advanced Robotic Systems*. https://journals.sagepub.com/doi/10.1177/17298806241278172

---

### 2. 仮想障害物・仮想ターゲット法（Virtual Obstacles / Virtual Targets）

**原理:** 局所解周辺に仮想的な障害物（Virtual Obstacle）や中間目標点（Virtual Subgoal）を配置して補助力場を生成する。

**効果（定量）:**
- **S-APFM（Subarea-APF Model）**: イテレーション数53.88%削減、エネルギー消費53.87%削減
- **PSO-APFハイブリッド**: パス長18%短縮、障害物回避効率90%向上、成功率85%向上（動的環境）
- **VAPF + RL（MDPI Machines, 2025）**: 局所解発生が有意に減少、ドローンナビゲーション成功率が大幅向上

**主な文献:**
- Kim, D.H. & Shin, S. (2003). "Artificial potential field based path planning for mobile robots using a virtual obstacle concept." *IEEE/ASME International Conference on Advanced Intelligent Mechatronics*, pp. 735–740. https://ieeexplore.ieee.org/document/1225434
- Xi, W. et al. (2025). "Path planning of mobile robot based on improved PRM and APF." *Measurement and Control*. https://journals.sagepub.com/doi/10.1177/00202940241291282
- Park, S. et al. (2025). "Addressing Local Minima in Path Planning for Drones with Reinforcement Learning-Based Vortex Artificial Potential Fields." *MDPI Machines*, 13(7): 600. https://www.mdpi.com/2075-1702/13/7/600
- Sezgin, G.K. et al. (2024). "Path Planning Based on Artificial Potential Field with an Enhanced Virtual Hill Algorithm." *Applied Sciences*, 14(18): 8292. https://www.mdpi.com/2076-3417/14/18/8292

---

### 3. 調和ポテンシャル（Harmonic Potential Fields）

**原理:** ポテンシャル関数がラプラス方程式 ∇²φ = 0 を満たす場合、内部に局所最小値を持たない（調和関数の最大・最小値定理）。これを利用して局所解フリーな経路計画を実現する。

**効果:**
- **理論的に局所解ゼロが保証**（設定空間内の内部極値は存在しない）
- 全出発点に共通のポテンシャルマップを再利用可能
- グリッドの離散化誤差により実装上は疑似局所解が発生する可能性あり

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | **理論的保証あり** |
| 計算コスト | 高（オフラインO(N²〜N³)） |
| 実装難易度 | 高 |
| 動的環境対応 | 低（再計算が必要） |

**主な文献:**
- Kim, J. & Khosla, P. (1992). "Real-Time Obstacle Avoidance Using Harmonic Potential Functions." *IEEE Transactions on Robotics and Automation*, 8(3): 338–349. http://vigir.missouri.edu/~gdesouza/Research/MobileRobotics/Harmonic%20Functions.pdf
- Connolly, C.I. et al. (1990). "Path planning using Laplace's equation." *ICRA 1990*.
- Gomez, J.V. et al. (2021). "Real-Time Path Planning Based on Harmonic Functions under a Proper Generalized Decomposition-Based Framework." *MDPI Sensors*, 21(12): 3943. https://pmc.ncbi.nlm.nih.gov/articles/PMC8228859/
- Vlantis, P. et al. (2023). "Robot Navigation in Complex Workspaces Employing Harmonic Maps and Adaptive Artificial Potential Fields." *MDPI Sensors*, 23(9): 4464. https://www.mdpi.com/1424-8220/23/9/4464

---

### 4. ナビゲーション関数（Navigation Functions: Rimon-Koditschek型）

**原理:** Rimon & Koditschek（1992）が提案。障害物を球体にモデル化し、ゴール位置のみを大域的最小値とする解析的ポテンシャル関数を構築する。パラメータκを適切に選択することで局所解ゼロが数学的に証明される。

```
φ(q) = γ_d(q) / (γ_d(q)^κ + β(q))^(1/κ)
```

**効果:**
- 球形世界において**ほぼすべての初期位置からゴール到達が数学的に保証**（測度ゼロの例外を除く）
- ランタイムの計算コストは標準APFと同等
- 障害物が球形（またはそれに変換可能）である前提条件が厳しい

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | **数学的保証あり（球形世界）** |
| 計算コスト | 低（解析的） |
| 実装難易度 | 高 |
| 動的環境対応 | 困難 |

**主な文献:**
- Rimon, E. & Koditschek, D.E. (1992). "Exact Robot Navigation Using Artificial Potential Functions." *IEEE Transactions on Robotics and Automation*, 8(5): 501–518. https://www.semanticscholar.org/paper/Exact-robot-navigation-using-artificial-potential-Rimon-Koditschek/2e365698de9f727cc5be3bcfb6329d71896ca67d
- Arslan & Koditschek (2018). "Sensor-Based Reactive Navigation in Unknown Convex Sphere Worlds." https://omurarslan.github.io/assets/publications/arslan_koditschek_IJRR2018.pdf

---

### 5. ハイブリッドプランナー（A* / RRT* + APF）

**原理:** A*等のグローバルプランナーが生成した経由点（waypoint）をAPFの引力ターゲットとして使用する。局所解に捕捉される前にwaypointを切り替えることで局所解を回避する。Nav2のGlobalPlanner+Controller構成と一致する。

**効果（定量）:**
- **G-APF（グローバルパス誘導APF）**: パス角度コーナー40.8%削減、振動回数81.8%削減、パス長4.3%短縮
- **RRT* + APF**: 従来RRT*比で反復回数95%削減、シミュレーション時間93.98%削減
- **APF + DQN**: 成功率98.17%（DQN単独: 94.00%）

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | なし（実用的に大幅改善） |
| 計算コスト | 中（二層構造） |
| 実装難易度 | 中 |
| Nav2との親和性 | **高（既存構造と一致）** |

**主な文献:**
- Chen (2023). "Global path guided vehicle obstacle avoidance path planning with APF method." *IET Cyber-Systems and Robotics*. https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/csy2.12082
- PMC (2024). "Research on Autonomous Vehicle Path Planning Based on Improved RRT* Algorithm and APF Method." https://pmc.ncbi.nlm.nih.gov/articles/PMC11207524/
- PMC (2025). "Path Planning Trends for Autonomous Mobile Robot Navigation: A Review." https://pmc.ncbi.nlm.nih.gov/articles/PMC11861809/

---

### 6. Tangent Bug / Wall Following との組み合わせ

**原理:** 局所解検出時に障害物輪郭追従（Wall Following）モードへ切り替える。TangentBugはKamon, Rimon & Rivlin（1998）が提案した距離センサ対応の改良版。2モード（ゴール方向移動・輪郭追従）を切り替えながら動作する。

**効果（定量）:**
- **APF + WF（ルールベース）**: 6〜10台ロボット環境で成功率60〜100%（arxiv:2409.10332）
- **APF + WF（学習ベース, ViT使用）**: ルールベース比で成功率35〜48%改善、Swapシナリオで100%
- Bugアルゴリズムは**解が存在すれば必ずゴール到達が数学的に保証**（完全アルゴリズム）

**主な文献:**
- Kamon, I., Rimon, E., & Rivlin, E. (1998). "TangentBug: A Range-Sensor-Based Navigation Algorithm." *International Journal of Robotics Research*, 17(9), 934-953. https://journals.sagepub.com/doi/10.1177/027836499801700903
- Mohamed & El-Metwally (2011). "An improved Tangent Bug method integrated with APF for multi-robot path planning." https://ieeexplore.ieee.org/document/5946136/
- arxiv:2409.10332 (2024). "Escaping Local Minima: Hybrid APF with Wall-Follower for Decentralized Multi-Robot Navigation." https://arxiv.org/html/2409.10332v1
- Yun & Tan. "A wall-following method for escaping local minima in potential field based motion planning." https://www.semanticscholar.org/paper/A-wall-following-method-for-escaping-local-minima-Yun-Tan/822e12b32db727bb9a8dc9492d08dafc140b6152

---

### 7. Vector Field Histogram（VFH / VFH+）

**原理:** ヒストグラムグリッドで方向ごとの障害物密度（Polar Obstacle Density）を評価し、密度が低い「谷（valley）」を選択する。APFの力のバランスに依存しないため構造的に局所解が起きにくい。

**効果:**
- APF（平均到達時間50.78秒）に対してVFH系が有意に優位
- VFH+Tでトラップ回避メカニズムを追加

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | 構造的に低リスク |
| 計算コスト | 低〜中 |
| 実装難易度 | 中 |

**主な文献:**
- Borenstein, J. & Koren, Y. (1991). "The vector field histogram—fast obstacle avoidance for mobile robots." *IEEE Transactions on Robotics and Automation*, 7(3), 278–288. https://ieeexplore.ieee.org/document/88137/
- Ulrich, I. & Borenstein, J. (1998). "VFH+: Reliable obstacle avoidance for fast mobile robots." https://www.semanticscholar.org/paper/VFH%2B:-reliable-obstacle-avoidance-for-fast-mobile-Ulrich-Borenstein/0650ddea1f769fe6b5c92f5806d973605293dbd1
- VFH+T (2024). https://www.sciencedirect.com/science/article/pii/S2590123024008806

---

### 8. 回転ポテンシャルフィールド（Rotational Potential Fields / VAPF）

**原理:** APFの対称性を破ることで局所解を回避する。斥力場に接線方向（タンジェント）の力成分を追加（VAPF）、または改良された斥力ポテンシャル関数を使用する（Ge & Cui 2000）。

**効果（定量）:**
- **APF + WF ハイブリッド（TurtleBot4実機実験）**: 最大30台で成功率60〜100%、対称対向衝突シナリオで100%
- 比較ベースライン（ORCA、バニラAPF、RPF単体）は非凸環境でほぼ完全に失敗

**主な文献:**
- Ge, S.S. & Cui, Y.J. (2000). "New potential functions for mobile robot path planning." *IEEE Transactions on Robotics and Automation*, 16(5), 615–620. https://ieeexplore.ieee.org/document/880813/
- VORPF (2025). "Virtual Obstacle-based Rotational Potential Field for UAV Swarm to Escape Local Minima." https://ieeexplore.ieee.org/document/11213480/
- RL-based VAPF (2025). https://www.mdpi.com/2075-1702/13/7/600

---

### 9. DWA（Dynamic Window Approach）との組み合わせ

**原理:** 速度空間サンプリングで多方向を同時評価するDWAで局所解脱出を補完。Fox, Burgard & Thrun（1997）が提案。ROS 2 Nav2の`nav2_dwb_controller`として標準実装済み。

**効果（定量）:**
- 改良DWA: 軌跡滑らかさ47.9%改善、走行時間37.3%短縮
- A* + DWA統合: 経路長1.2%短縮、計算時間20.1%削減

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | なし（補完） |
| 計算コスト | 中 |
| Nav2との親和性 | **最高（標準実装済み）** |

**主な文献:**
- APF + DWA (IEEE ICUS 2019). https://ieeexplore.ieee.org/document/8996014/
- 改良DWA（フォーメーション, 2024). https://www.sciencedirect.com/science/article/pii/S2772662224000754
- A* + DWA 農業環境（2025). https://www.sciencedirect.com/science/article/pii/S2214317325000630

---

### 10. 強化学習との組み合わせ（RL + APF）

**原理:** APFの局所解問題をデータ駆動的に解決。VAPFとTD3を組み合わせた階層制御、またはAPFを状態表現・報酬整形に活用するアプローチ。

**効果（定量）:**
- TD3ベース改良: 成功率93.1%、衝突率6.8%（2025年）
- VAPF+RLで純粋RLと比較して収束速度・安定性で優位

**評価:**
| 項目 | 内容 |
|------|------|
| 局所解フリー保証 | なし（学習済み） |
| 計算コスト | 高（GPU推論） |
| 実装難易度 | 高 |

**主な文献:**
- VAPF + RL (MDPI Machines 2025). https://www.mdpi.com/2075-1702/13/7/600
- Variable-Direction APF + TD3 (MDPI Mathematics 2025). https://www.mdpi.com/2227-7390/13/14/2312
- APF + DRL（屋内火災避難, 2025). https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/tje2.70074

---

## 総合比較表

| 手法 | 局所解フリー保証 | 計算コスト | 実装難易度 | 動的環境 | potbot_core適合性 |
|------|:--------------:|:--------:|:--------:|:------:|:----------------:|
| ランダム摂動 | なし | 極低 | 極低 | 高 | 補助的 |
| 仮想障害物/ターゲット | なし（改善） | 低〜中 | 中 | 中 | 実用的 |
| VAPF（渦巻き） | なし（根本改善） | 低〜中 | 中 | 高 | **高** |
| 調和ポテンシャル | **理論保証** | 高 | 高 | 低 | 静的環境向き |
| ナビゲーション関数 | **理論保証** | 低 | 高 | 低 | 制約大 |
| A*/RRT* ハイブリッド | なし（大幅改善） | 中 | 中 | 中 | **高** |
| Wall Following/Bug | 完全性保証 | 低 | 低〜中 | 高 | **高** |
| VFH / VFH+ | 構造的に低リスク | 低〜中 | 中 | 高 | 高 |
| DWA + APF | なし（補完） | 中 | 低 | 高 | **最高** |
| RL + APF | なし（学習済み） | 高 | 高 | 高 | 研究寄り |

---

## potbot_core への実装推奨順位

1. **DWA + APF**（最短路）— `nav2_dwb_controller` と APF GlobalPlanner の組み合わせで即試験可能
2. **VAPF（渦巻き）**— `potbot_lib/ArtificialPotentialField` の斥力関数に接線成分を追加するだけ
3. **ハイブリッド（A* + APF waypoint切り替え）**— `ApfPathPlanner` にサブゴール切り替えロジックを追加（**M11で実装予定**）
4. **Wall Following (Nav2 Recovery Behavior)**— ビヘイビアツリー拡張として既存の `potbot_behavior_tree` に追加可能
5. **調和ポテンシャル**— 中長期的・静的マップが確定した環境向け
