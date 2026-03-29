#include <potbot_lib/apf_path_planner.hpp>
#include <queue>
#include <climits>

namespace potbot_lib{

    namespace path_planner{

        // void get_search_index_APF(Field& field, std::vector<size_t>, )
        APFPathPlanner::APFPathPlanner(ArtificialPotentialField *apf) : apf_(NULL)
        {
            std::random_device rd;
            random_engine_ = new std::default_random_engine(rd());
            random_generator_double_ = new std::uniform_real_distribution<double>(0,1.0);
            apf_ = apf;
        }

        void APFPathPlanner::setParams(double maxp, size_t sr, double wpot, double wpos)
        {
            max_path_length_ = maxp;
            path_search_range_ = sr;
            path_weight_potential_ = wpot;
            path_weight_pose_ = wpos;
        }

        void APFPathPlanner::setParams(double maxp, size_t sr, double wpot, double wpos,
                                       const std::string& escape_method, int max_escape_attempts,
                                       int virtual_obstacle_lifetime)
        {
            setParams(maxp, sr, wpot, wpos);
            escape_method_ = escape_method;
            max_escape_attempts_ = max_escape_attempts;
            virtual_obstacle_lifetime_ = virtual_obstacle_lifetime;
        }

        /**
         * @brief Dijkstra法による経路計画
         *
         * APFの勾配降下法では局所解に陥る問題をDijkstra法で解消する。
         * 各グリッドの斥力コストをエッジコストに組み込み、障害物を避けながら
         * グローバル最適経路を探索する。
         *
         * @param init_robot_pose ロボットの初期姿勢（現バージョンでは未使用）
         * @return true  経路が見つかった場合
         * @return false ロボット位置が見つからない、またはゴールに到達できない場合
         */
        bool APFPathPlanner::createPathDijkstra(double init_robot_pose)
        {
            path_.clear();
            std::vector<potential::FieldGrid>* field_values = apf_->getValues();
            const size_t N = field_values->size();

            // ロボット開始インデックスを検索
            size_t start_idx = SIZE_MAX;
            for (const auto& v : *field_values) {
                if (v.states[potential::GridInfo::IS_ROBOT]) {
                    start_idx = v.index;
                    break;
                }
            }
            if (start_idx == SIZE_MAX) return false;

            // Dijkstra法による最短経路探索
            // エッジコスト = 物理距離 × (1 + 斥力コスト) とすることで障害物付近を回避する
            std::vector<double> dist(N, std::numeric_limits<double>::infinity());
            std::vector<size_t> prev(N, SIZE_MAX);
            using P = std::pair<double, size_t>;
            std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

            dist[start_idx] = 0.0;
            pq.push({0.0, start_idx});
            size_t goal_idx = SIZE_MAX;

            while (!pq.empty()) {
                auto [d, u] = pq.top();
                pq.pop();
                // 古いエントリはスキップ
                if (d > dist[u]) continue;

                // ゴール周辺グリッドに到達したら終了
                if ((*field_values)[u].states[potential::GridInfo::IS_AROUND_GOAL]) {
                    goal_idx = u;
                    break;
                }

                // 隣接グリッドへの遷移コストを計算
                std::vector<size_t> neighbors;
                apf_->getSquareIndex(neighbors, (*field_values)[u].row, (*field_values)[u].col, 1);

                for (size_t v : neighbors) {
                    double dx = (*field_values)[v].x - (*field_values)[u].x;
                    double dy = (*field_values)[v].y - (*field_values)[u].y;
                    double phys_dist = std::sqrt(dx * dx + dy * dy);
                    // 障害物グリッドは非常に高いコストを付与（壁を飛び越える経路を防止）
                    double obstacle_penalty = 0.0;
                    if ((*field_values)[v].states[potential::GridInfo::IS_OBSTACLE]) {
                        obstacle_penalty = 1000.0;
                    }
                    // 斥力コストをペナルティとして加算することで障害物付近を回避
                    double repulsion_cost = (*field_values)[v].repulsion;
                    double edge_cost = phys_dist * (1.0 + repulsion_cost) + obstacle_penalty;
                    double new_dist = d + edge_cost;

                    if (new_dist < dist[v]) {
                        dist[v] = new_dist;
                        prev[v] = u;
                        pq.push({new_dist, v});
                    }
                }
            }

            if (goal_idx == SIZE_MAX) return false;

            // パスを復元（ゴール→スタートの逆順をreverse）
            std::vector<Pose> rev_path;
            for (size_t cur = goal_idx; cur != SIZE_MAX; cur = prev[cur]) {
                rev_path.push_back(Pose{(*field_values)[cur].x, (*field_values)[cur].y});
            }
            std::reverse(rev_path.begin(), rev_path.end());
            path_ = rev_path;
            return !path_.empty();
        }

        /**
         * @brief A*法による経路計画（Dijkstraの高速化版）
         *
         * ユークリッド距離ヒューリスティック + tiebreakで探索ノード数を削減。
         * admissible保証を維持しつつ、同コストノードをゴール方向優先で展開する。
         */
        bool APFPathPlanner::createPathAStar(double init_robot_pose)
        {
            path_.clear();
            std::vector<potential::FieldGrid>* field_values = apf_->getValues();
            const size_t N = field_values->size();

            // ロボット開始インデックスを検索
            size_t start_idx = SIZE_MAX;
            for (const auto& v : *field_values) {
                if (v.states[potential::GridInfo::IS_ROBOT]) {
                    start_idx = v.index;
                    break;
                }
            }
            if (start_idx == SIZE_MAX) return false;

            // ゴール座標を取得（ヒューリスティック計算用）
            double goal_x = 0, goal_y = 0;
            for (const auto& v : *field_values) {
                if (v.states[potential::GridInfo::IS_GOAL]) {
                    goal_x = v.x;
                    goal_y = v.y;
                    break;
                }
            }

            // ヒューリスティック関数: ユークリッド距離（admissible）
            auto heuristic = [&](size_t idx) -> double {
                double dx = (*field_values)[idx].x - goal_x;
                double dy = (*field_values)[idx].y - goal_y;
                return std::sqrt(dx * dx + dy * dy);
            };

            // tiebreak用の微小係数（同f値ノードでゴールに近い方を優先）
            constexpr double tiebreak_factor = 1.0 + 1e-4;

            std::vector<double> g_cost(N, std::numeric_limits<double>::infinity());
            std::vector<size_t> prev(N, SIZE_MAX);
            // {f_cost, g_cost, index} — f値が同じ場合にg値（実コスト）が大きい方を優先（ゴールに近い）
            using Entry = std::tuple<double, double, size_t>;
            std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

            g_cost[start_idx] = 0.0;
            double h_start = heuristic(start_idx) * tiebreak_factor;
            open.push({h_start, 0.0, start_idx});
            size_t goal_idx = SIZE_MAX;

            while (!open.empty()) {
                auto [f, g, u] = open.top();
                open.pop();
                // 古いエントリはスキップ
                if (g > g_cost[u]) continue;

                // ゴール周辺グリッドに到達したら終了
                if ((*field_values)[u].states[potential::GridInfo::IS_AROUND_GOAL]) {
                    goal_idx = u;
                    break;
                }

                // 隣接グリッドへの遷移コストを計算
                std::vector<size_t> neighbors;
                apf_->getSquareIndex(neighbors, (*field_values)[u].row, (*field_values)[u].col, 1);

                const int u_row = (*field_values)[u].row;
                const int u_col = (*field_values)[u].col;

                for (size_t v : neighbors) {
                    // 障害物グリッドはスキップ
                    if ((*field_values)[v].states[potential::GridInfo::IS_OBSTACLE]) continue;

                    const int v_row = (*field_values)[v].row;
                    const int v_col = (*field_values)[v].col;
                    const int dr = v_row - u_row;
                    const int dc = v_col - u_col;

                    // 斜め移動時、隣接する直交セルが障害物なら遷移を禁止
                    // （壁の角を斜めにすり抜ける経路を防止）
                    if (dr != 0 && dc != 0) {
                        const int adj_r = u_row + dr;
                        const int adj_c = u_col + dc;
                        const auto& hdr = apf_->getHeader();
                        const int rows = static_cast<int>(hdr.rows);
                        const int cols = static_cast<int>(hdr.cols);
                        // 行方向の隣接セル (u_row+dr, u_col) が障害物か
                        if (adj_r >= 0 && adj_r < rows && u_col >= 0 && u_col < cols) {
                            size_t idx_r = static_cast<size_t>(adj_r) * hdr.cols + static_cast<size_t>(u_col);
                            if ((*field_values)[idx_r].states[potential::GridInfo::IS_OBSTACLE]) continue;
                        }
                        // 列方向の隣接セル (u_row, u_col+dc) が障害物か
                        if (u_row >= 0 && u_row < rows && adj_c >= 0 && adj_c < cols) {
                            size_t idx_c = static_cast<size_t>(u_row) * hdr.cols + static_cast<size_t>(adj_c);
                            if ((*field_values)[idx_c].states[potential::GridInfo::IS_OBSTACLE]) continue;
                        }
                    }

                    double dx = (*field_values)[v].x - (*field_values)[u].x;
                    double dy = (*field_values)[v].y - (*field_values)[u].y;
                    double phys_dist = std::sqrt(dx * dx + dy * dy);
                    // 斥力コストをペナルティとして加算することで障害物付近を回避
                    double repulsion_cost = (*field_values)[v].repulsion;
                    double edge_cost = phys_dist * (1.0 + repulsion_cost);
                    double new_g = g + edge_cost;

                    if (new_g < g_cost[v]) {
                        g_cost[v] = new_g;
                        prev[v] = u;
                        double new_f = new_g + heuristic(v) * tiebreak_factor;
                        open.push({new_f, new_g, v});
                    }
                }
            }

            if (goal_idx == SIZE_MAX) return false;

            // パスを復元（ゴール→スタートの逆順をreverse）
            std::vector<Pose> rev_path;
            for (size_t cur = goal_idx; cur != SIZE_MAX; cur = prev[cur]) {
                rev_path.push_back(Pose{(*field_values)[cur].x, (*field_values)[cur].y});
            }
            std::reverse(rev_path.begin(), rev_path.end());
            path_ = rev_path;
            return !path_.empty();
        }

        bool APFPathPlanner::createPathWithWeight(double init_robot_pose)
        {
            path_.clear();
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre, J_min_tmp;
            double P_min;
            double path_length  = 0;
            size_t range        = path_search_range_;
            double weight_potential = path_weight_potential_;
            double weight_pose = path_weight_pose_;
            std::vector<potential::FieldGrid>* field_values;
            field_values = apf_->getValues();
            double scale = std::pow(10, 3);

            for (auto value : (*field_values))
            {
                if (value.states[potential::GridInfo::IS_ROBOT])
                {
                    pf_idx_min  = value.index;
                    center_x    = value.x;
                    center_y    = value.y;
                    J_min_pre   = value.value;
                    P_min       = value.value;
                    center_row  = value.row;
                    center_col  = value.col;
                    break;
                }
            }
            path_.push_back(Pose{center_x, center_y});
            (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

            // sortRepulsionEdges();

            double theta_pre = init_robot_pose;
            double x_pre     = path_.back().position.x;
            double y_pre     = path_.back().position.y;

            bool solving_local_minimum = false;
            bool change_weight = false;
            // J_min_pre = 1e10;

            while ((*field_values)[pf_idx_min].states[potential::GridInfo::IS_AROUND_GOAL] == false &&
                    path_length <= max_path_length_)
            {
                //経路補間に時間がかかってしまうため制御点(path.size())の数に上限を設ける
                if (path_.size() > 300) break;
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                apf_->getSquareIndex(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue = (*field_values)[idx].value;
                        if (PotentialValue < P_min) 
                        {
                            solving_local_minimum       = false;
                            P_min                       = PotentialValue;
                            pf_idx_min                  = idx;
                        }
                    }
                    // if (solving_local_minimum == true)
                    // {
                    //     change_weight = true;
                    //     //j_minの保存
                    //     //姿勢のround
                    //     J_min_tmp = J_min;
                    //     continue;
                    // }
                    // else
                    // {
                    //     // J_min = J_min_tmp;
                    //     change_weight = false;
                    // }
                    
                }
                else
                {
                    // スケール不一致を解消するためinfで初期化し、最良Jセルを正しく選択する
                    J_min = std::numeric_limits<double>::infinity();
                    double j1,j2;
                    int random_range = static_cast<int>(range);
                    double wu               = weight_potential;
                    double w_theta          = weight_pose;
                    size_t best_idx = SIZE_MAX;  // 全100回試行での最良セルインデックス
                    size_t best_idx_no_los = SIZE_MAX;  // LOSなしフォールバック用
                    double J_min_no_los = std::numeric_limits<double>::infinity();

                    const auto field_header = apf_->getHeader();
                    const size_t field_cols = field_header.cols;
                    const size_t field_rows = field_header.rows;

                    auto isLOSClear = [&](size_t r0, size_t c0, size_t r1, size_t c1) -> bool {
                        int dr = std::abs((int)r1 - (int)r0);
                        int dc = std::abs((int)c1 - (int)c0);
                        int sr = ((int)r1 >= (int)r0) ? 1 : -1;
                        int sc = ((int)c1 >= (int)c0) ? 1 : -1;
                        int err = dr - dc;
                        int r = (int)r0, c = (int)c0;
                        while (true) {
                            if (r < 0 || c < 0 || (size_t)r >= field_rows || (size_t)c >= field_cols) return false;
                            size_t fidx = (size_t)r * field_cols + (size_t)c;
                            if ((*field_values)[fidx].states[potential::GridInfo::IS_OBSTACLE]) return false;
                            if (r == (int)r1 && c == (int)c1) break;
                            int e2 = 2 * err;
                            if (e2 > -dc) { err -= dc; r += sr; }
                            if (e2 < dr) { err += dr; c += sc; }
                        }
                        return true;
                    };

                    for (size_t i = 0; i < 100; i++)
                    {
                        apf_->getSquareIndex(search_indexes, center_row, center_col, random_range);
                        if (search_indexes.empty())
                        {
                            // 探索範囲が空の場合は次のランダム範囲を試す
                            wu          = (*random_generator_double_)((*random_engine_));
                            w_theta     = 1.0 - wu;
                            random_range = (*random_generator_double_)((*random_engine_)) * 30 + 1;
                            continue;
                        }

                        for (auto idx : search_indexes)
                        {
                            if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] == true) continue;
                            // 障害物セル自体には経路点を置かない
                            if ((*field_values)[idx].states[potential::GridInfo::IS_OBSTACLE] == true) continue;

                            double PotentialValue   = (*field_values)[idx].value;
                            double x                = (*field_values)[idx].x;
                            double y                = (*field_values)[idx].y;

                            double theta            = atan2(y-y_pre,x-x_pre);
                            double posediff         = abs(theta - theta_pre);
                                posediff         = std::floor(posediff * scale) / scale;

                            double sum;
                            sum = PotentialValue+posediff;
                            j1               = wu*PotentialValue/sum;
                            j2               = w_theta*posediff/sum;
                            double J                = j1 + j2;

                            // break_flagを使わず全100回を通じてグローバル最小Jを追跡
                            // LOSが通る候補を優先、なければLOSなし候補をフォールバック
                            if (J < J_min && isLOSClear(center_row, center_col,
                                                         (*field_values)[idx].row,
                                                         (*field_values)[idx].col))
                            {
                                J_min       = J;
                                best_idx    = idx;
                            }
                            if (J < J_min_no_los)
                            {
                                J_min_no_los = J;
                                best_idx_no_los = idx;
                            }
                        }
                        // 全100回実行するため次の反復のランダム化を常に行う
                        wu          = (*random_generator_double_)((*random_engine_));
                        w_theta     = 1.0 - wu;
                        random_range = (*random_generator_double_)((*random_engine_)) * 30 + 1;
                    }
                    // LOS候補が見つかった場合はそれを、なければフォールバック候補を使用
                    // どちらも見つからない場合は探索打ち切り
                    size_t chosen = (best_idx != SIZE_MAX) ? best_idx : best_idx_no_los;
                    if (chosen != SIZE_MAX)
                    {
                        solving_local_minimum = false;
                        pf_idx_min = chosen;
                    }
                    else break;
                }
                
                double px   = (*field_values)[pf_idx_min].x;
                double py   = (*field_values)[pf_idx_min].y;
                path_length += sqrt(pow(px - path_.back().position.x,2) + pow(py - path_.back().position.y,2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                J_min_pre   = J_min;
                P_min = (*field_values)[pf_idx_min].value;
                x_pre       = px;
                y_pre       = py;
                theta_pre   = atan2(py-path_.back().position.y,px-path_.back().position.x);
                //経路点が2連続で同じ場所の場合処理を終わらせる
                Pose p{px,py};
                // バグ1: path_.size() < 2 のときpath_.end()[-2]は未定義動作になるためガードを追加
                if (path_.size() >= 2 && p == path_.end()[-1] && p == path_.end()[-2]) break;

                path_.push_back(Pose{px, py});
                (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

            }
            return true;
        }

        bool APFPathPlanner::createPathWithVirtualObstacle(double init_robot_pose)
        {
            path_.clear();
            apf_->clearVirtualObstacles();

            std::vector<potential::FieldGrid>* field_values = apf_->getValues();

            // ロボット開始位置を取得
            size_t pf_idx_min = 0;
            size_t center_row = 0, center_col = 0;
            double center_x = 0, center_y = 0;
            double P_min = 0;
            bool found_robot = false;
            for (const auto& value : *field_values)
            {
                if (value.states[potential::GridInfo::IS_ROBOT])
                {
                    pf_idx_min = value.index;
                    center_x = value.x;
                    center_y = value.y;
                    center_row = value.row;
                    center_col = value.col;
                    P_min = value.value;
                    found_robot = true;
                    break;
                }
            }
            if (!found_robot) return false;

            path_.push_back(Pose{center_x, center_y});
            (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

            double path_length = 0;
            int escape_count = 0;
            size_t range = path_search_range_;

            while ((*field_values)[pf_idx_min].states[potential::GridInfo::IS_AROUND_GOAL] == false &&
                    path_length <= max_path_length_)
            {
                if (path_.size() > 300) break;

                // 近傍探索で最小ポテンシャルセルを検索
                std::vector<size_t> search_indexes;
                apf_->getSquareIndex(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                bool found_lower = false;
                double best_potential = P_min;
                size_t best_idx = pf_idx_min;

                for (auto idx : search_indexes)
                {
                    if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH]) continue;
                    if ((*field_values)[idx].states[potential::GridInfo::IS_OBSTACLE]) continue;

                    double potential_val = (*field_values)[idx].value;
                    if (potential_val < best_potential)
                    {
                        found_lower = true;
                        best_potential = potential_val;
                        best_idx = idx;
                    }
                }

                if (found_lower)
                {
                    // 勾配降下: より低いポテンシャルへ移動
                    pf_idx_min = best_idx;
                    double px = (*field_values)[pf_idx_min].x;
                    double py = (*field_values)[pf_idx_min].y;
                    path_length += sqrt(pow(px - path_.back().position.x, 2) + pow(py - path_.back().position.y, 2));
                    center_row = (*field_values)[pf_idx_min].row;
                    center_col = (*field_values)[pf_idx_min].col;
                    P_min = (*field_values)[pf_idx_min].value;

                    Pose p{px, py};
                    if (path_.size() >= 2 && p == path_.end()[-1] && p == path_.end()[-2]) break;
                    path_.push_back(p);
                    (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                }
                else
                {
                    // 局所解検出: 仮想障害物を配置して再計算
                    if (escape_count >= max_escape_attempts_) break;

                    apf_->addVirtualObstacle(
                        (*field_values)[pf_idx_min].x,
                        (*field_values)[pf_idx_min].y,
                        virtual_obstacle_lifetime_);

                    // ポテンシャル場全体を再計算
                    apf_->createPotentialField();
                    field_values = apf_->getValues();

                    // 既存パスのIS_PLANNED_PATHフラグを再設定
                    for (const auto& pose : path_)
                    {
                        try
                        {
                            size_t idx = apf_->getFieldIndex(pose.position.x, pose.position.y);
                            (*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                        }
                        catch(...){}
                    }

                    // 現在セルのポテンシャル値を更新
                    P_min = (*field_values)[pf_idx_min].value;

                    escape_count++;
                    apf_->decrementVirtualObstacleLifetimes();
                }
            }

            apf_->clearVirtualObstacles();

            // ゴール到達判定
            if (!path_.empty() && (*field_values)[pf_idx_min].states[potential::GridInfo::IS_AROUND_GOAL])
            {
                return true;
            }
            return false;
        }

        bool APFPathPlanner::createPath(double init_robot_pose)
        {
            bool success = false;

            if (escape_method_ == "virtual_obstacle_vortex")
            {
                success = createPathWithVirtualObstacle(init_robot_pose);
            }
            else if (escape_method_ == "random_weighted")
            {
                success = createPathWithWeight(init_robot_pose);
            }
            else if (escape_method_ == "wall_following")
            {
                success = createPathWallFollowing(init_robot_pose);
            }

            if (!success)
            {
                // 最終フォールバック: A*（Dijkstraより高速）
                success = createPathAStar(init_robot_pose);
            }

            return success;
        }

        bool APFPathPlanner::createPathWallFollowing(double init_robot_pose)
        {
            path_.clear();
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre;
            double path_length  = 0;
            size_t range        = path_search_range_;
            std::vector<potential::FieldGrid>* field_values;
            field_values = apf_->getValues();

            // バグ5: ゴール方向への進捗がない連続ステップ数を追跡してループを防ぐ
            double prev_dist_to_goal = std::numeric_limits<double>::infinity();
            int no_progress_count = 0;
            const int no_progress_limit = 50;

            double distance_threshold_repulsion_field = apf_->getDistanceThresholdRepulsionField();

            Point robot = apf_->getRobot();
            Point goal = apf_->getGoal();
            potbot_lib::potential::FieldGrid robot_grid = apf_->getValue(robot.x,robot.y);
            pf_idx_min  = robot_grid.index;
            center_x    = robot_grid.x;
            center_y    = robot_grid.y;
            J_min_pre   = robot_grid.value;
            center_row  = robot_grid.row;
            center_col  = robot_grid.col;

            bool goal_inside_repulsion = false;
            try
            {
                potbot_lib::potential::FieldGrid goal_grid = apf_->getValue(goal.x,goal.y);
                if (goal_grid.states[potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
                {
                    goal_inside_repulsion = true;
                }
            }
            catch (...){}
            
            path_.push_back(Pose{center_x, center_y});
            (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

            sortRepulsionEdges();

            double theta_pre = init_robot_pose;
            double x_pre     = path_.back().position.x;
            double y_pre     = path_.back().position.y;

            bool solving_local_minimum = false;
            bool change_weight = false;
            // バグ3: elseブランチ（ローカル最小値解消）から来たか判別するフラグ
            bool came_from_local_minimum_escape = false;
            while ((*field_values)[pf_idx_min].states[potential::GridInfo::IS_AROUND_GOAL] == false &&
                    path_length <= max_path_length_)
            {
                //経路補間に時間がかかってしまうため制御点(path.size())の数に上限を設ける
                if (path_.size() > 100) break;
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                apf_->getSquareIndex(search_indexes, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue   = (*field_values)[idx].value;
                        double x                = (*field_values)[idx].x;
                        double y                = (*field_values)[idx].y;
                        
                        double theta = atan2(y-y_pre,x-x_pre);
                        double posediff         = abs(theta - theta_pre);

                        double wu               = 1.0;
                        double w_theta          = 0;
                        if (change_weight == true)
                        {
                            wu                  = 0.0;
                            w_theta             = 1.0;
                        }
                        double J                = wu*PotentialValue + w_theta*posediff;

                        if (J < J_min) 
                        {
                            solving_local_minimum       = false;
                            J_min                       = J;
                            pf_idx_min                  = idx;
                        }
                    }
                    
                }
                else
                {

                    std::vector<potential::FieldGrid> edges_clockwise, edges_counterclockwise;
                    getRepulsionEdges(edges_clockwise, edges_counterclockwise, center_row, center_col);

                    // バグ2: エッジが空の場合は未定義動作を防ぐためbreakする
                    if (edges_clockwise.empty() && edges_counterclockwise.empty()) break;




                    // path.clear();
                    // for (size_t i = 0; i < edges_clockwise.size(); i++)
                    // {
                    //     path.push_back({edges_clockwise[i].x, edges_clockwise[i].y});
                    //     (*field_values)[edges_clockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                    // }
                    // return;

                    // path.clear();
                    // for (size_t i = 0; i < edges_counterclockwise.size(); i++)
                    // {
                    //     path.push_back({edges_counterclockwise[i].x, edges_counterclockwise[i].y});
                    //     (*field_values)[edges_counterclockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                    // }
                    // return;




                    // double distance_to_goal_clockwise = std::numeric_limits<double>::infinity();
                    size_t path_length_clockwise;
                    for (path_length_clockwise = 0;path_length_clockwise < edges_clockwise.size()/2; path_length_clockwise++)
                    {
                        if (getSmallerPotentialIndex(edges_clockwise[path_length_clockwise].index, J_min) != edges_clockwise[path_length_clockwise].index)
                        {
                            break;
                        }
                    }

                    size_t path_length_counterclockwise;
                    for (path_length_counterclockwise = 0;path_length_counterclockwise < edges_counterclockwise.size()/2; path_length_counterclockwise++)
                    {
                        if (getSmallerPotentialIndex(edges_counterclockwise[path_length_counterclockwise].index, J_min) != edges_counterclockwise[path_length_counterclockwise].index)
                        {
                            break;
                        }
                    }

                    // バグ2: 逃走できなかった場合（エッジを辿り切れなかった）はbreakする
                    if (path_length_clockwise == edges_clockwise.size()/2 && path_length_counterclockwise == edges_counterclockwise.size()/2) break;

                    if(path_length_clockwise < path_length_counterclockwise)
                    {
                        // バグ4: エッジ追加時にpath_lengthを累積する
                        for (size_t i = 0; i < path_length_clockwise; i++)
                        {
                            double ex = edges_clockwise[i].x;
                            double ey = edges_clockwise[i].y;
                            path_length += sqrt(pow(ex - path_.back().position.x, 2) + pow(ey - path_.back().position.y, 2));
                            path_.push_back(Pose{ex, ey});
                            (*field_values)[edges_clockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                            if (path_length > max_path_length_ || path_.size() > 100) break;
                        }
                        pf_idx_min = edges_clockwise[path_length_clockwise].index;
                    }
                    else
                    {
                        // バグ4: エッジ追加時にpath_lengthを累積する
                        for (size_t i = 0; i < path_length_counterclockwise; i++)
                        {
                            double ex = edges_counterclockwise[i].x;
                            double ey = edges_counterclockwise[i].y;
                            path_length += sqrt(pow(ex - path_.back().position.x, 2) + pow(ey - path_.back().position.y, 2));
                            path_.push_back(Pose{ex, ey});
                            (*field_values)[edges_counterclockwise[i].index].states[potential::GridInfo::IS_PLANNED_PATH] = true;
                            if (path_length > max_path_length_ || path_.size() > 100) break;
                        }
                        pf_idx_min = edges_counterclockwise[path_length_counterclockwise].index;
                    }

                    // バグ3: elseブランチから来たことをフラグで記録し、J_min_preを逃走点のポテンシャル値で更新する
                    came_from_local_minimum_escape = true;
                    J_min_pre = (*field_values)[pf_idx_min].value;

                    solving_local_minimum   = false;
                }

                double px   = (*field_values)[pf_idx_min].x;
                double py   = (*field_values)[pf_idx_min].y;
                path_length += sqrt(pow(px - path_.back().position.x,2) + pow(py - path_.back().position.y,2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                // バグ3: elseブランチから来た場合はJ_min_preを上書きしない（elseブランチ内で逃走点のポテンシャルを設定済み）
                if (!came_from_local_minimum_escape) {
                    J_min_pre   = J_min;
                }
                came_from_local_minimum_escape = false;
                x_pre       = px;
                y_pre       = py;
                theta_pre   = atan2(py-path_.back().position.y,px-path_.back().position.x);
                //経路点が2連続で同じ場所の場合処理を終わらせる
                Pose p{px,py};
                // バグ1: path_.size() < 2 のときpath_.end()[-2]は未定義動作になるためガードを追加
                if (path_.size() >= 2 && p == path_.end()[-1] && p == path_.end()[-2]) break;

                if ((p.position - path_.back().position).norm()>0.1)
                {
                    // ROS_INFO("over distance");
                    break;
                }

                if (goal_inside_repulsion && (goal-p.position).norm()<distance_threshold_repulsion_field+0.1)
                {
                    break;
                }

                path_.push_back(Pose{px, py});
                (*field_values)[pf_idx_min].states[potential::GridInfo::IS_PLANNED_PATH] = true;

                // バグ5: ゴールに近づかない連続ステップが続く場合はbreakしてループを防ぐ
                {
                    double current_dist_to_goal = sqrt(pow(px - goal.x, 2) + pow(py - goal.y, 2));
                    if (current_dist_to_goal >= prev_dist_to_goal) {
                        no_progress_count++;
                    } else {
                        no_progress_count = 0;
                    }
                    prev_dist_to_goal = current_dist_to_goal;
                    if (no_progress_count >= no_progress_limit) break;
                }

            }
            return true;
        }

        size_t APFPathPlanner::getSmallerPotentialIndex(size_t centor_index, double potential_value)
        {
            std::vector<potential::FieldGrid>* field_values;
            field_values = apf_->getValues();

            size_t centor_row = (*field_values)[centor_index].row;
            size_t centor_col = (*field_values)[centor_index].col;

            if(potential_value < 0)
            {
                potential_value = 0;
            }
            
            std::vector<size_t> search_indexes;
            apf_->getSquareIndex(search_indexes, centor_row, centor_col, 1);
            if (search_indexes.empty()) return centor_index;

            double distance_from_centor_to_goal = sqrt(pow((*field_values)[centor_index].x - apf_->getGoal().x,2)+pow((*field_values)[centor_index].y - apf_->getGoal().y,2));

            for (auto idx : search_indexes)
            {
                if ((*field_values)[idx].value < potential_value) 
                {
                    double distance_from_idx_to_goal = sqrt(pow((*field_values)[idx].x - apf_->getGoal().x,2)+pow((*field_values)[idx].y - apf_->getGoal().y,2));
                    if (distance_from_idx_to_goal < distance_from_centor_to_goal)
                    {
                        return idx;
                    }
                }
            }

            return centor_index;

        }

        void APFPathPlanner::sortRepulsionEdges()
        {
            loop_edges_.clear();
            
            potential::Field edge_field;
            apf_->infoFilter(edge_field, potential::GridInfo::IS_REPULSION_FIELD_EDGE);
            std::vector<potential::FieldGrid>* edge_values = edge_field.getValues();
            if((*edge_values).empty()) return;

            float next_grid_distance = sqrt(2.0) * apf_->getHeader().resolution;
            // next_grid_distance+=0.2;
            float scale = std::pow(10, 3);
            next_grid_distance =  std::ceil(next_grid_distance * scale) / scale;

            std::vector<potential::FieldGrid> sorted_values;
            sorted_values.push_back((*edge_values)[0]);
            (*edge_values).erase((*edge_values).begin());
            while (!(*edge_values).empty())
            {
                float x = sorted_values.back().x;
                float y = sorted_values.back().y;
                float min_distance = std::numeric_limits<float>::infinity();
                size_t min_index;
                for (size_t index = 0; index < (*edge_values).size(); index++)
                {
                    float distance = sqrt(pow(x - (*edge_values)[index].x, 2) + pow(y - (*edge_values)[index].y, 2));
                    if (distance <  min_distance)
                    {
                        min_distance = distance;
                        min_index = index;
                    }
                }

                if (min_distance > next_grid_distance)
                {
                    loop_edges_.push_back(sorted_values);
                    sorted_values.clear();
                }
                sorted_values.push_back((*edge_values)[min_index]);
                (*edge_values).erase((*edge_values).begin() + min_index);
            }
            loop_edges_.push_back(sorted_values);

            // std::vector<std::vector<potbot_lib::potential::FieldGrid>> merged;
            // for (const auto& points : loop_edges_) 
            // {
            //     if (merged.empty()) 
            //     {
            //         merged.push_back(points);
            //     } 
            //     else 
            //     {
            //         float s_x = merged.back().back().x;
            //         float s_y = merged.back().back().y;
            //         float e_x = merged.back().front().x;
            //         float e_y = merged.back().front().y;

            //         float distance1 = sqrt(pow(s_x-points.front().x,2)+pow(s_y-points.front().y,2));
            //         float distance2 = sqrt(pow(e_x-points.front().x,2)+pow(e_y-points.front().y,2));
            //         float distance3 = sqrt(pow(s_x-points.back().x,2)+pow(s_y-points.back().y,2));
            //         float distance4 = sqrt(pow(e_x-points.back().x,2)+pow(e_y-points.back().y,2));

            //         if (distance1 <= next_grid_distance || distance2 <= next_grid_distance || distance3 <= next_grid_distance || distance4 <= next_grid_distance) 
            //         {
            //             for (const auto& p:points)
            //             {
            //                 merged.back().push_back(p);
            //             }
            //         }
            //         else
            //         { 
            //             merged.push_back(points);
            //         }
            //     }
            // }
            // loop_edges_ = merged;

        }

        void APFPathPlanner::getRepulsionEdges(std::vector<potential::FieldGrid>& edges_clockwise, std::vector<potential::FieldGrid>& edges_counterclockwise, size_t row_centor, size_t col_centor)
        {
            // auto loops = loop_edges_[0];
            // edges_clockwise.push_back(loops[0]);
            // for (size_t i = 1; i < loops.size(); i++)
            // {
            //     double xi = loops[i].x;
            //     double yi = loops[i].y;
            //     double xi_1 = loops[i-1].x;
            //     double yi_1 = loops[i-1].y;
            //     double distance = sqrt(pow(xi-xi_1,2)+pow(yi-yi_1,2));
            //     double ref = sqrt(0.11*0.11+0.11*0.11);
            //     if (distance <= ref)
            //     {
            //         edges_clockwise.push_back(loops[i]);
            //     }
            //     else
            //     {
            //         break;
            //     }
                
            // }
            // return;
            

            size_t start_index = apf_->getFieldIndex(row_centor, col_centor);

            float min_distance = std::numeric_limits<float>::infinity();
            float start_x = apf_->getValue(start_index).x;
            float start_y = apf_->getValue(start_index).y;
            // std::vector<potential::FieldGrid> near_edges;
            for (const auto& loops : loop_edges_)
            {
                for (const auto& value : loops)
                {
                    float distance = sqrt(pow(start_x - value.x,2) + pow(start_y - value.y,2));
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        start_index = value.index;
                        // near_edges = loops;
                    }
                }
            }

            for (const auto& loops : loop_edges_)
            {
                size_t shift_num = 0;
                for (const auto& value : loops)
                {
                    if(value.index == start_index)
                    {   
                        // edges_counterclockwise.push_back(loops[0]);
                        // for (size_t i = 1; i < loops.size(); i++)
                        // {
                        //     double xi = loops[i].x;
                        //     double yi = loops[i].y;
                        //     double xi_1 = loops[i-1].x;
                        //     double yi_1 = loops[i-1].y;
                        //     double distance = sqrt(pow(xi-xi_1,2)+pow(yi-yi_1,2));
                        //     double ref = sqrt(0.11*0.11+0.11*0.11);
                        //     if (distance <= ref)
                        //     {
                        //         edges_counterclockwise.push_back(loops[i]);
                        //     }
                        //     else
                        //     {
                        //         break;
                        //     }
                            
                        // }
                        

                        edges_counterclockwise  = loops;
                        std::rotate(edges_counterclockwise.begin(), edges_counterclockwise.begin() + shift_num, edges_counterclockwise.end());
                        edges_clockwise         = edges_counterclockwise;
                        std::reverse(edges_clockwise.begin(), edges_clockwise.end());
                        std::rotate(edges_clockwise.rbegin(), edges_clockwise.rbegin() + 1, edges_clockwise.rend());
                        return;
                    }
                    shift_num++;
                }
            }
        }

        bool APFPathPlanner::bezier()
        {
            // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/
            
            const std::vector<Pose> &path_control = path_;
            std::vector<Pose> path_interpolated;

            if (path_control.size() < 2) return false;

            double x_min = path_control[0].position.x;
            double x_max = path_control[0].position.x;
            double y_min = path_control[0].position.y;
            double y_max = path_control[0].position.y;
            for(const auto& point : path_control)
            {
                double x = point.position.x;
                double y = point.position.y;
                if (x < x_min) x_min = x;
                if (x > x_max) x_max = x;
                if (y < y_min) y_min = y;
                if (y > y_max) y_max = y;
            }

            double n = path_control.size();

            int bezier_idx = 0;
            double inc = 1.0/double(n*10);
            double interpolate_distance_threshold = sqrt(pow(path_control[0].position.x - path_control[1].position.x,2) + pow(path_control[0].position.y - path_control[1].position.y,2));
            for (double t = 0.0; t <= 1.0; t += inc)
            {
                double x = 0;
                double y = 0;
                for (double i = 0.0; i <= n-1.0; i++)
                {
                    double a = combination(n-1.0,i);
                    double b = pow(t,i);
                    double c = pow(1.0-t,n-i-1.0);
                    double x_inc = a * b * c * path_control[size_t(i)].position.x;
                    double y_inc = combination(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * path_control[size_t(i)].position.y;
                    if (std::isnan(x_inc) || std::isinf(x_inc))
                    {
                        x_inc = 0;
                    }
                    if (std::isnan(y_inc) || std::isinf(y_inc))
                    {
                        y_inc = 0;
                    }
                    
                    x += x_inc;
                    y += y_inc;
                }
                // if (x > x_max || x < x_min || y > y_max || y < y_min)
                // {
                //     path_interpolated = path_control;
                //     return;
                // }
                if (path_interpolated.size() > 1)
                {
                    double distance = sqrt(pow(x - path_interpolated.back().position.x,2) + pow(y - path_interpolated.back().position.y,2));
                    if (distance > interpolate_distance_threshold) break;
                }
                
                path_interpolated.push_back({x,y});
            }
            
            path_ = path_interpolated;
            return true;
        }

        void APFPathPlanner::getPath(std::vector<Pose>& path)
        {
            path = path_;
        }

        double APFPathPlanner::combination(double n, double r)
        {
            double top = 1.0;
            double bottom = 1.0;

            for(double i = 0.0; i < r; i++)
            {
                top *= n-i;
            }

            for(double i = 0.0; i < r; i++)
            {
                bottom *= i+1.0;
            }
            
            double ans = top/bottom;
            if (std::isnan(ans))
            {
                ans = 0;
            }
            if (std::isinf(ans))
            {
                ans = 1e100;
            }
            return ans;
        }
    }
}