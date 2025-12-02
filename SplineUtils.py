import re
from math import atan2, degrees, sqrt, floor
from copy import deepcopy
from BasicElements import Pos, Rot, Line, Transform
from EaseUtils import parse_easing_func, calculate_adaptive_multiplier


def _spline_catmull_rom_interpolate(p0, p1, p2, p3, t):
    """
    Catmull-Rom スプライン補間（1次元）
    t (0.0-1.0) に応じて p1 と p2 の間の値を計算する。
    """
    t2 = t * t
    t3 = t2 * t
    return 0.5 * ( (2 * p1) +
                   (-p0 + p2) * t +
                   (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
                   (-p0 + 3*p1 - 3*p2 + p3) * t3 )


def _spline_calculate_look_at(cam_pos, target_pos):
    """
    カメラの位置 (cam_pos) からターゲットの位置 (target_pos) を
    見つめるための回転 (Rot) オブジェクトを計算する。
    """
    tx, ty, tz = target_pos.unpack()
    cam_x, cam_y, cam_z = cam_pos.unpack()
    delta_x = tx - cam_x
    delta_y = ty - cam_y
    delta_z = tz - cam_z
    horizontal_dist = sqrt(delta_x**2 + delta_z**2)
    if horizontal_dist < 1e-6:
        horizontal_dist = 1e-6 
    ry = degrees(atan2(delta_x, delta_z))
    rx = -degrees(atan2(delta_y, horizontal_dist))
    rz = 0
    return Rot(rx, ry, rz)


def _spline_get_arc_length_lut(control_points, num_segments, samples_per_segment=100, p0_guide=None, p3_guide=None):
    """
    【距離基準のためのパス1】
    スプライン軌道の総距離を計算し、
    (距離, グローバルt) のルックアップテーブル(LUT)を作成する。
    p0_guide, p3_guide が指定されている場合は、両端の接線計算に使用する。
    """
    lut = [(0.0, 0.0)] # (distance, global_t)
    total_distance = 0.0
    last_pos = control_points[0] # 最初の制御点
    for i in range(num_segments):
        # このセグメント(i -> i+1)の制御点を取得
        p1 = control_points[i]
        p2 = control_points[i + 1]
        # p0 (始点の接線用) の決定
        if i > 0:
            p0 = control_points[i - 1]
        elif p0_guide is not None:
            p0 = p0_guide # ガイドを使用
        else:
            p0 = p1 # フォールバック
        # p3 (終点の接線用) の決定
        if i < (num_segments - 1):
            p3 = control_points[i + 2]
        elif p3_guide is not None:
            p3 = p3_guide # ガイドを使用
        else:
            p3 = p2 # フォールバック
        for j in range(1, samples_per_segment + 1):
            local_t = j / samples_per_segment
            global_t = i + local_t # グローバルなスプラインパラメータ t
            # Catmull-Rom で現在の座標を計算
            x = _spline_catmull_rom_interpolate(p0.x, p1.x, p2.x, p3.x, local_t)
            y = _spline_catmull_rom_interpolate(p0.y, p1.y, p2.y, p3.y, local_t)
            z = _spline_catmull_rom_interpolate(p0.z, p1.z, p2.z, p3.z, local_t)
            # 1サンプル前の座標からの距離を計算
            dx = x - last_pos.x
            dy = y - last_pos.y
            dz = z - last_pos.z
            dist = sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += dist
            lut.append((total_distance, global_t))
            last_pos = Pos(x, y, z)
    return lut, total_distance


def _spline_get_t_for_distance(lut, target_distance):
    """
    【距離基準のためのパス2】
    LUT を参照し、target_distance (目標距離) に
    最も近いグローバルなスプラインパラメータ t を逆引きする。
    """
    # 線形探索 (LUTはソート済み)
    for i in range(1, len(lut)):
        dist_prev, t_prev = lut[i-1]
        dist_curr, t_curr = lut[i]
        if dist_curr >= target_distance:
            # ターゲット距離を跨ぐ [prev, curr] のペアを発見
            dist_segment = dist_curr - dist_prev
            if dist_segment < 1e-6:
                return t_prev # ゼロ除算を回避
            # 2点間で線形補間して t を求める
            ratio = (target_distance - dist_prev) / dist_segment
            t = t_prev + (t_curr - t_prev) * ratio
            return t
    # ターゲット距離が総距離を超えた場合、最後の t を返す
    return lut[-1][1]


def _spline_unwrap_angles(angles):
    """
    ロール(rz)の「遠回り問題」を解消するため、角度リストをアンラップする。
    例: [350, 10, 20] -> [350, 370, 380]
    """
    unwrapped = []
    if not angles:
        return unwrapped
    unwrapped.append(angles[0])
    last_angle = angles[0]
    for i in range(1, len(angles)):
        current_angle = angles[i]
        diff = current_angle - last_angle
        if diff > 180:
            current_angle -= 360 # 10 -> 350 へのジャンプを補正
        elif diff < -180:
            current_angle += 360 # 350 -> 10 へのジャンプを補正
        unwrapped.append(current_angle)
        last_angle = current_angle
    return unwrapped


def _spline_parse_points(text, logger):
    """
    splineコマンド文字列から制御点(q_...)とイージング、range、sync情報を抽出するヘルパー
    """
    param_text = re.split('[,_]', text, 1)[-1]
    params_list = re.split('[,]', param_text) 
    control_point_strings = [] 
    target_pos_params = []     
    ease_name_str = None
    cnct_flag = False
    sync_flag = False
    range_start = 0.0
    range_end = 1.0
    for param in params_list:
        param_stripped = param.strip()
        if not param_stripped:
            continue
        if param_stripped.startswith('q_'):
            control_point_strings.append(param_stripped)
        elif param_stripped.lower() == 'cnct':
            cnct_flag = True
        elif param_stripped.lower() == 'sync':
            sync_flag = True
        elif param_stripped.lower().startswith('r'):
            # Range解析 r0-0.5, r0.0-1.0 など
            try:
                range_parts = param_stripped[1:].split('-')
                if len(range_parts) == 2:
                    range_start = float(range_parts[0])
                    range_end = float(range_parts[1])
            except ValueError:
                if logger: logger.log(f'! spline: rangeパラメータの解析に失敗しました: {param_stripped} !')
        elif param_stripped[0].isalpha(): 
            if ease_name_str is None:
                ease_name_str = param_stripped
        else:
            target_pos_params.append(param_stripped)
    # イージング解析
    ease_func, dx, dy, easetype_name = (None, 6, 6, None)
    if ease_name_str:
        ease_func, dx, dy, easetype_name = parse_easing_func(ease_name_str, logger, log_prefix='spline ')
    if ease_func is None:
        ease_func = lambda t: t
    # 座標解析
    control_points_pos = [] 
    control_points_fov = [] 
    control_points_roll_raw = []
    for i, point_str in enumerate(control_point_strings):
        try:
            parts = point_str.split('_')
            if len(parts) >= 8:
                px = float(parts[1])
                py = float(parts[2])
                pz = float(parts[3])
                rz = float(parts[6])
                fov = float(parts[7])
                control_points_pos.append(Pos(px, py, pz))
                control_points_fov.append(fov)
                control_points_roll_raw.append(rz)
            else:
                raise ValueError("q_ 形式のデータが短すぎます (8要素必要)")
        except Exception as e:
            if logger: logger.log(f'! spline: 座標 {i+1} ({point_str}) の解析に失敗: {e} !')
            return None
    return {
        'pos': control_points_pos,
        'fov': control_points_fov,
        'roll': control_points_roll_raw,
        'ease_func': ease_func,
        'ease_name': easetype_name,
        'dx': dx, 'dy': dy,
        'target_params': target_pos_params,
        'cnct': cnct_flag,
        'sync': sync_flag,
        'range_start': range_start,
        'range_end': range_end
    }


def spline(self, text, dur, next_text=None, next_dur=None): 
    """
    Catmull-Rom スプライン補間を行う。(距離基準 / ターゲット追従モード専用)
    q_... 形式の座標 (8要素: px,py,pz,rx,ry,rz,fov) を補間する。
    """
    self.logger.log(f'spline コマンド解析開始: "{text}"')
    current_data = _spline_parse_points(text, self.logger)
    if current_data is None:
        return
    control_points_pos = current_data['pos']
    control_points_fov = current_data['fov']
    control_points_roll_raw = current_data['roll']
    cnct = current_data['cnct']
    sync = current_data['sync']
    range_start = current_data['range_start']
    range_end = current_data['range_end']
    # --- Look-Behind: 過去の制御点の結合 ---
    prev_guide_pos = None
    prev_guide_fov = None
    prev_guide_roll = None
    if cnct:
        if hasattr(self, 'last_spline_points') and self.last_spline_points:
            # 始点省略: 前回の終点を始点として追加
            last_end_pos = self.last_spline_points[-1]
            last_end_fov = self.last_spline_fov[-1]
            last_end_roll = self.last_spline_roll[-1]
            control_points_pos.insert(0, last_end_pos)
            control_points_fov.insert(0, last_end_fov)
            control_points_roll_raw.insert(0, last_end_roll)
            self.logger.log(f'spline: cnctを検出。前回の終点を始点として使用します。')
            # ガイド点 (前回の後ろから2番目) を取得
            if len(self.last_spline_points) >= 2:
                prev_guide_pos = self.last_spline_points[-2]
                prev_guide_fov = self.last_spline_fov[-2]
                prev_guide_roll = self.last_spline_roll[-2]
                self.logger.log(f'spline: 前回の軌道を考慮して滑らかに接続します。')
        else:
            self.logger.log(f'! spline: cnctが指定されましたが、前回のspline情報がありません !')
    # --- Look-Ahead: 次の制御点の取得 ---
    next_guide_pos = None
    next_guide_fov = None
    next_guide_roll = None
    next_total_distance = 0.0
    # Next Sync Data (Simulation/Calc)
    next_sync_target_vel = None
    if next_text and 'spline' in next_text:
        # 次のコマンドを簡易解析
        next_data = _spline_parse_points(next_text, None) # ログは出さない
        if next_data and next_data['pos']:
            # 次が cnct あり -> 始点が省略されているので、最初の点(pos[0])が実質2点目(ガイド)
            # 次が cnct なし -> 最初の点(pos[0])が始点(ガイド)
            next_guide_pos = next_data['pos'][0]
            next_guide_fov = next_data['fov'][0]
            next_guide_roll = next_data['roll'][0]
            # --- Next Velocity Calculation for Sync ---
            if sync and next_dur and next_dur > 0:
                # 1. 次の制御点を構築
                next_points = next_data['pos'][:] # Copy
                if next_data['cnct']:
                    # 結合する場合、始点は現在の終点
                    next_points.insert(0, control_points_pos[-1])
                # 2. 次の総距離を計算
                # ガイド点は一旦無視して距離計算 (近似)
                _, next_total_distance = _spline_get_arc_length_lut(next_points, len(next_points)-1)
                # 3. v_next の決定
                if next_data['sync']:
                    # 次もSyncなら平均速度をターゲットにする
                    next_sync_target_vel = next_total_distance / next_dur
                else:
                    # 次がSyncでない(EaseInなど)なら、イージングの傾きから初速をシミュレーション
                    # v(t) = (TotalDist / TotalDur) * Ease'(t)
                    # Ease'(0) を数値微分で求める
                    next_ease = next_data['ease_func']
                    dt = 0.001
                    slope = (next_ease(dt) - next_ease(0)) / dt
                    next_sync_target_vel = (next_total_distance / next_dur) * slope
            self.logger.log(f'spline: 次のsplineコマンド({next_guide_pos})を参照して終点を滑らかにします。')
    # データを保存 (次回の cnct / sync 用)
    self.last_spline_points = deepcopy(control_points_pos)
    self.last_spline_fov = deepcopy(control_points_fov)
    self.last_spline_roll = deepcopy(control_points_roll_raw)
    # 5. 制御点の数をチェック
    if len(control_points_pos) < 2:
        self.logger.log(f"! spline: 軌道には最低2点の制御点が必要です (現在 {len(control_points_pos)} 点) !")
        return
    control_points_roll = _spline_unwrap_angles(control_points_roll_raw)
    # ガイドのRollもアンラップ処理に合わせて調整が必要
    if prev_guide_roll is not None:
        diff = control_points_roll[0] - prev_guide_roll
        while diff > 180: prev_guide_roll += 360; diff -= 360
        while diff < -180: prev_guide_roll -= 360; diff += 360
    if next_guide_roll is not None:
         diff = next_guide_roll - control_points_roll[-1]
         while diff > 180: next_guide_roll -= 360; diff -= 360
         while diff < -180: next_guide_roll += 360; diff += 360
    num_segments = len(control_points_pos) - 1
    # 3. ターゲット座標の決定 (既存ロジック)
    target_pos_params = current_data['target_params']
    target_pos_start = None 
    target_pos_end = None   
    try:
        if len(target_pos_params) >= 6:
            tx1 = float(eval(target_pos_params[0]))
            ty1 = float(eval(target_pos_params[1]))
            tz1 = float(eval(target_pos_params[2]))
            tx2 = float(eval(target_pos_params[3]))
            ty2 = float(eval(target_pos_params[4]))
            tz2 = float(eval(target_pos_params[5]))
            target_pos_start = Pos(tx1, ty1, tz1)
            target_pos_end = Pos(tx2, ty2, tz2)
            self.logger.log(f'spline: [移動注視点モード] で実行。')
        elif len(target_pos_params) >= 3:
            tx = float(eval(target_pos_params[0]))
            ty = float(eval(target_pos_params[1]))
            tz = float(eval(target_pos_params[2]))
            target_pos_start = Pos(tx, ty, tz)
            self.logger.log(f'spline: [静的注視点モード] で実行。注視点: ({tx}, {ty}, {tz})')
        else:
            target_pos_start = Pos(0, 1.6, 0)
            self.logger.log(f'spline: [デフォルト注視点モード] で実行。')
    except Exception as e:
        self.logger.log(f'! spline: ターゲット座標の解析に失敗: {e} !')
        target_pos_start = Pos(0, 1.6, 0)
    # 6. 【距離基準のためのパス1】 軌道の総距離とLUTを事前計算
    self.logger.log('spline: 軌道の総距離を事前計算中...')
    # ガイド情報をPos型に変換して渡す
    pg_pos = prev_guide_pos
    ng_pos = next_guide_pos
    pg_fov = Pos(prev_guide_fov, 0, 0) if prev_guide_fov is not None else None
    ng_fov = Pos(next_guide_fov, 0, 0) if next_guide_fov is not None else None
    pg_roll = Pos(prev_guide_roll, 0, 0) if prev_guide_roll is not None else None
    ng_roll = Pos(next_guide_roll, 0, 0) if next_guide_roll is not None else None
    control_points_roll_pos = [Pos(r, 0, 0) for r in control_points_roll]
    control_points_fov_pos = [Pos(f, 0, 0) for f in control_points_fov] 
    pos_lut, total_distance = _spline_get_arc_length_lut(control_points_pos, num_segments, p0_guide=pg_pos, p3_guide=ng_pos)
    fov_lut, total_fov_change_pseudo = _spline_get_arc_length_lut(control_points_fov_pos, num_segments, p0_guide=pg_fov, p3_guide=ng_fov)
    roll_lut, total_roll_change_pseudo = _spline_get_arc_length_lut(control_points_roll_pos, num_segments, p0_guide=pg_roll, p3_guide=ng_roll)
    self.logger.log(f'spline: 軌道総距離: {total_distance:.2f} m')
    # --- Sync & Time Logic ---
    rate_func = None
    if sync:
        self.logger.log('spline: syncモード有効。速度同期計算を行います。')
        CUT_THRESHOLD = 0.1 # カット検出の閾値
        v_prev = 0.0
        v_curr = total_distance / dur if dur > 0 else 0
        v_next = 0.0
        # 前回の速度 (実測値: self.lines の最後を参照)
        # カット検出: 前回の終点と今回の始点が離れていたらv_prevを採用しない
        valid_prev = False
        if len(self.lines) > 0:
            last_line = self.lines[-1]
            last_pos = last_line.end.pos
            curr_start_pos = control_points_pos[0]
            # 距離チェック
            dx = last_pos.x - curr_start_pos.x
            dy = last_pos.y - curr_start_pos.y
            dz = last_pos.z - curr_start_pos.z
            dist_diff = sqrt(dx*dx + dy*dy + dz*dz)
            if dist_diff < CUT_THRESHOLD:
                if last_line.duration > 0:
                    dx_l = last_line.end.pos.x - last_line.start.pos.x
                    dy_l = last_line.end.pos.y - last_line.start.pos.y
                    dz_l = last_line.end.pos.z - last_line.start.pos.z
                    dist_last = sqrt(dx_l*dx_l + dy_l*dy_l + dz_l*dz_l)
                    v_prev = dist_last / last_line.duration
                    valid_prev = True
            else:
                 self.logger.log(f'spline: sync - 前回の終点と距離があるため({dist_diff:.2f}m)、速度を引き継ぎません。')
        if not valid_prev:
            v_prev = v_curr # 連続していない場合は現在の速度で開始
        # 次回の速度 (計算済みターゲット)
        # カット検出: 今回の終点と次回の始点が離れていたらv_nextを採用しない
        valid_next = False
        if next_sync_target_vel is not None and next_guide_pos is not None:
            curr_end_pos = control_points_pos[-1]
            # 距離チェック
            dx = curr_end_pos.x - next_guide_pos.x
            dy = curr_end_pos.y - next_guide_pos.y
            dz = curr_end_pos.z - next_guide_pos.z
            dist_diff = sqrt(dx*dx + dy*dy + dz*dz)
            if dist_diff < CUT_THRESHOLD:
                v_next = next_sync_target_vel
                valid_next = True
            else:
                self.logger.log(f'spline: sync - 次回の始点と距離があるため({dist_diff:.2f}m)、速度を合わせません。')
        if not valid_next:
             v_next = v_curr # 連続していない場合は現在の速度で終了
        ratio_in = 1.0
        ratio_out = 1.0
        if v_curr > 0.0001:
            ratio_in = v_prev / v_curr
            ratio_out = v_next / v_curr
        self.logger.log(f'spline: sync - Vin: {v_prev:.2f}, Vcurr: {v_curr:.2f}, Vnext: {v_next:.2f}')
        self.logger.log(f'spline: sync - Ratio In: {ratio_in:.2f}, Ratio Out: {ratio_out:.2f}')
        # エルミート補間による時間関数
        # T(t) = (t^3 - 2t^2 + t) * R_in + (-2t^3 + 3t^2) + (t^3 - t^2) * R_out
        def hermite_time(t):
            t2 = t * t
            t3 = t2 * t
            term1 = (t3 - 2*t2 + t) * ratio_in
            term2 = (-2*t3 + 3*t2)
            term3 = (t3 - t2) * ratio_out
            return term1 + term2 + term3
        rate_func = hermite_time
    else:
        # 通常のイージング
        ease_func_base = current_data['ease_func']
        dx_base = current_data['dx']
        dy_base = current_data['dy']
        name_base = current_data['ease_name']
        if name_base == 'Drift':
             rate_func = lambda t: ease_func_base(t, dx_base, dy_base)
        else:
             rate_func = ease_func_base
    # Range適用関数
    def final_rate_func(t):
        # 0 <= t <= 1 の時間を range_start <= T <= range_end にマップ
        mapped_t = range_start + t * (range_end - range_start)
        # BaseのRateを計算 (Sync or Ease)
        base_rate = rate_func(mapped_t)
        # 正規化: range_start時点のRateを0、range_end時点のRateを1にする
        start_rate = rate_func(range_start)
        end_rate = rate_func(range_end)
        denom = end_rate - start_rate
        if abs(denom) < 0.0001:
            return 0 # 範囲がつぶれている場合
        return (base_rate - start_rate) / denom
    # 7. アダプティブ解像度と時間分割
    base_p = 10 
    p = base_p
    # SyncやRangeが入ると解析的予測が難しいので、とりあえず固定か、単純なeaseチェックのみ
    if not sync and current_data['ease_func'] and dur > 0:
        # イージング時の解像度調整 (既存ロジック活用)
        res_multiplier = calculate_adaptive_multiplier(current_data['ease_func'], dur, current_data['dx'], current_data['dy'], self.logger)
        p = base_p / res_multiplier
        self.logger.log(f'spline p: {p} (base_p: {base_p} / mult: {res_multiplier})')
    span = 1 / 30
    spans = []
    init_dur = dur
    while dur > 0:
        min_span = min(span, dur)
        if dur - min_span < 0.01:
            min_span = dur
        spans.append(min_span)
        dur -= min_span
    span_size = len(spans)
    init_dur = sum(spans)
    if init_dur == 0:
        self.logger.log('spline: 期間が0のため、最後の制御点にスナップします。')
        endPos = control_points_pos[-1]
        endFov = control_points_fov[-1]
        endRoll = control_points_roll[-1]
        end_target = target_pos_end if target_pos_end else target_pos_start
        endRot = _spline_calculate_look_at(endPos, end_target)
        endRot.z = endRoll
        self.lastTransform = Transform(endPos, endRot, endFov)
        return
    # 8. メインループ
    for i in range(span_size):
        new_line = Line(spans[i])
        new_line.visibleDict = deepcopy(self.visibleObject.state)
        t_end = sum(spans[:(i+1)]) / init_dur
        if t_end > 1: t_end = 1
        # (A) Sync / Range / Ease が適用された Rate
        rate = final_rate_func(t_end)
        # 9. 【距離基準のためのパス2】 rate を「距離」に変換し、t を逆引き
        # (位置)
        target_distance = total_distance * rate
        global_spline_t_pos = _spline_get_t_for_distance(pos_lut, target_distance)
        # (FOV)
        target_fov_change = total_fov_change_pseudo * rate
        global_spline_t_fov = _spline_get_t_for_distance(fov_lut, target_fov_change)
        # (Roll)
        target_roll_change = total_roll_change_pseudo * rate
        global_spline_t_roll = _spline_get_t_for_distance(roll_lut, target_roll_change)
        # 10. グローバル t を「セグメント」と「ローカル t」に分解
        # (位置)
        segment_index_pos = min(floor(global_spline_t_pos), num_segments - 1)
        segment_t_pos = global_spline_t_pos - segment_index_pos 
        # (FOV)
        segment_index_fov = min(floor(global_spline_t_fov), num_segments - 1)
        segment_t_fov = global_spline_t_fov - segment_index_fov 
        # (Roll)
        segment_index_roll = min(floor(global_spline_t_roll), num_segments - 1)
        segment_t_roll = global_spline_t_roll - segment_index_roll 
        # 11. 補間に必要な4点を取得 (ガイド対応版)
        def get_p_points(points, idx, g_p0, g_p3):
            p1 = points[idx]
            p2 = points[idx+1]
            p0 = points[idx-1] if idx > 0 else (g_p0 if g_p0 is not None else p1)
            p3 = points[idx+2] if idx < (len(points)-2) else (g_p3 if g_p3 is not None else p2)
            return p0, p1, p2, p3
        # (位置)
        p0_pos, p1_pos, p2_pos, p3_pos = get_p_points(control_points_pos, segment_index_pos, pg_pos, ng_pos)
        # (FOV)
        p0_fov, p1_fov, p2_fov, p3_fov = get_p_points(control_points_fov, segment_index_fov, pg_fov.x if pg_fov else None, ng_fov.x if ng_fov else None)
        # (Roll)
        p0_roll, p1_roll, p2_roll, p3_roll = get_p_points(control_points_roll, segment_index_roll, pg_roll.x if pg_roll else None, ng_roll.x if ng_roll else None)
        # 12. Catmull-Rom 補間を実行
        # 位置(Pos)
        end_px = _spline_catmull_rom_interpolate(p0_pos.x, p1_pos.x, p2_pos.x, p3_pos.x, segment_t_pos)
        end_py = _spline_catmull_rom_interpolate(p0_pos.y, p1_pos.y, p2_pos.y, p3_pos.y, segment_t_pos)
        end_pz = _spline_catmull_rom_interpolate(p0_pos.z, p1_pos.z, p2_pos.z, p3_pos.z, segment_t_pos)
        endPos = Pos(end_px, end_py, end_pz)
        # FOV (1D補間)
        endFov = _spline_catmull_rom_interpolate(p0_fov, p1_fov, p2_fov, p3_fov, segment_t_fov)
        # Roll (1D補間)
        endRoll = _spline_catmull_rom_interpolate(p0_roll, p1_roll, p2_roll, p3_roll, segment_t_roll)
        # 13. 回転を計算 (注視点の計算は「時間基準」の 'rate' を使う)
        current_target_pos = None
        if target_pos_end is None:
            # モード1a または デフォルト
            current_target_pos = target_pos_start
        else:
            # モード1b: 移動注視点
            tx = target_pos_start.x + (target_pos_end.x - target_pos_start.x) * rate
            ty = target_pos_start.y + (target_pos_end.y - target_pos_start.y) * rate
            tz = target_pos_start.z + (target_pos_end.z - target_pos_start.z) * rate
            current_target_pos = Pos(tx, ty, tz)
        endRot = _spline_calculate_look_at(endPos, current_target_pos)
        endRot.z = endRoll
        # 14. new_line の設定
        new_line.end = Transform(endPos, endRot, endFov)
        if i == 0:
            # 始点もRange/Syncの影響を受けるため再計算が必要だが、
            # 厳密には前のフレームの最後と一致すべき。
            # ここでは簡易的に前回のLastTransformまたは初期位置を使う
            if cnct and range_start == 0.0:
                 new_line.start = deepcopy(self.lastTransform)
            else:
                 # 0地点の計算 (Rate=0の地点)
                 # ただし range_start > 0 の場合はジャンプする可能性がある
                 new_line.start = deepcopy(self.lastTransform) 
                 # 注意: Rangeで時間を飛ばした場合、Start位置はジャンプするのが正しい挙動
        else:
            new_line.start = deepcopy(self.lastTransform)
        # 15. 終点スナップ処理 (Rangeの最後が1.0でない場合はスナップしないほうが良いが、誤差吸収のため入れておく)
        if i == span_size - 1 and range_end == 1.0:
            endPos = control_points_pos[-1]
            endFov = control_points_fov[-1]
            endRoll = control_points_roll[-1]
            endRot = None
            if target_pos_end is None:
                endRot = _spline_calculate_look_at(endPos, target_pos_start)
            else:
                endRot = _spline_calculate_look_at(endPos, target_pos_end)
            endRot.z = endRoll
            new_line.end = Transform(endPos, endRot, endFov)
        self.lines.append(new_line)
        self.lastTransform = new_line.end
        self.logger.log(new_line.start)
