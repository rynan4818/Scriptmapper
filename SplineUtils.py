import re
from math import atan2, pi, sin, cos, degrees, sqrt, floor
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


def _spline_bspline_interpolate(p0, p1, p2, p3, t):
    """
    一様3次Bスプライン補間（1次元）
    制御点を通らず、滑らかに近似（ショートカット）する。
    """
    t2 = t * t
    t3 = t2 * t
    # 基底関数
    val = (p0 * ((1-t)**3) +
           p1 * (3*t3 - 6*t2 + 4) +
           p2 * (-3*t3 + 3*t2 + 3*t + 1) +
           p3 * t3) / 6.0
    return val


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


def _spline_get_arc_length_lut(control_points, num_segments, interpolate_func, samples_per_segment=100, p0_guide=None, p3_guide=None):
    """
    【距離基準のためのパス1】
    スプライン軌道の総距離を計算し、LUTを作成する。
    interpolate_func: 使用する補間関数
    """
    lut = [(0.0, 0.0)] # (distance, global_t)
    total_distance = 0.0
    # 最初の位置を計算（t=0）
    # ループ外で初期位置を計算する際は、ガイド点ロジックと同様にp0, p1, p2, p3を特定する必要がある
    p1_start = control_points[0]
    p2_start = control_points[1]
    p0_start = p0_guide if p0_guide is not None else p1_start
    p3_start = control_points[2] if num_segments > 1 else (p3_guide if p3_guide is not None else p2_start)
    start_x = interpolate_func(p0_start.x, p1_start.x, p2_start.x, p3_start.x, 0.0)
    start_y = interpolate_func(p0_start.y, p1_start.y, p2_start.y, p3_start.y, 0.0)
    start_z = interpolate_func(p0_start.z, p1_start.z, p2_start.z, p3_start.z, 0.0)
    last_pos = Pos(start_x, start_y, start_z)
    for i in range(num_segments):
        p1 = control_points[i]
        p2 = control_points[i + 1]
        # ガイド点考慮
        if i > 0: p0 = control_points[i - 1]
        elif p0_guide is not None: p0 = p0_guide
        else: p0 = p1
        if i < (num_segments - 1): p3 = control_points[i + 2]
        elif p3_guide is not None: p3 = p3_guide
        else: p3 = p2
        for j in range(1, samples_per_segment + 1):
            local_t = j / samples_per_segment
            global_t = i + local_t 
            x = interpolate_func(p0.x, p1.x, p2.x, p3.x, local_t)
            y = interpolate_func(p0.y, p1.y, p2.y, p3.y, local_t)
            z = interpolate_func(p0.z, p1.z, p2.z, p3.z, local_t)
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
    for i in range(1, len(lut)):
        dist_prev, t_prev = lut[i-1]
        dist_curr, t_curr = lut[i]
        if dist_curr >= target_distance:
            dist_segment = dist_curr - dist_prev
            if dist_segment < 1e-6:
                return t_prev
            ratio = (target_distance - dist_prev) / dist_segment
            t = t_prev + (t_curr - t_prev) * ratio
            return t
    return lut[-1][1]


def _spline_unwrap_angles(angles):
    """
    ロール(rz)の「遠回り問題」を解消するため、角度リストをアンラップする。
    """
    unwrapped = []
    if not angles:
        return unwrapped
    unwrapped.append(angles[0])
    last_angle = angles[0]
    for i in range(1, len(angles)):
        current_angle = angles[i]
        diff = current_angle - last_angle
        if diff > 180: current_angle -= 360
        elif diff < -180: current_angle += 360
        unwrapped.append(current_angle)
        last_angle = current_angle
    return unwrapped


def _spline_parse_points(text, logger):
    """
    spline/bsplineコマンド文字列から制御点(q_...)とイージング、range、sync、bspline情報を抽出するヘルパー
    """
    param_text = re.split('[,_]', text, 1)[-1]
    params_list = re.split('[,]', param_text) 
    control_point_strings = [] 
    target_pos_params = []     
    ease_name_str = None
    cnct_flag = False
    sync_flag = False
    bspline_flag = text.strip().startswith('bspline')
    range_start = 0.0
    range_end = 1.0
    for param in params_list:
        param_stripped = param.strip()
        if not param_stripped: continue
        lower_param = param_stripped.lower()
        if param_stripped.startswith('q_'):
            control_point_strings.append(param_stripped)
        elif lower_param == 'cnct':
            cnct_flag = True
        elif lower_param == 'sync':
            sync_flag = True
        elif lower_param.startswith('r'):
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
    ease_func, dx, dy, easetype_name = (None, 6, 6, None)
    if ease_name_str:
        ease_func, dx, dy, easetype_name = parse_easing_func(ease_name_str, logger, log_prefix='spline ')
    if ease_func is None:
        ease_func = lambda t: t
    control_points_pos = [] 
    control_points_fov = [] 
    control_points_roll_raw = []
    for i, point_str in enumerate(control_point_strings):
        try:
            parts = point_str.split('_')
            if len(parts) >= 8:
                px, py, pz = float(parts[1]), float(parts[2]), float(parts[3])
                rz, fov = float(parts[6]), float(parts[7])
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
        'bspline': bspline_flag, 
        'range_start': range_start,
        'range_end': range_end
    }


def spline(self, text, dur, next_text=None, next_dur=None): 
    """
    Catmull-Rom / B-Spline 補間を行う。
    q_... 形式の座標 (8要素: px,py,pz,rx,ry,rz,fov) を補間する。
    """
    self.logger.log(f'spline コマンド解析開始: "{text}"')
    current_data = _spline_parse_points(text, self.logger)
    if current_data is None:
        self.logger.log('! spline コマンドの解析に失敗したため、プログラムを終了します !')
        input()
        exit()
    control_points_pos = current_data['pos']
    control_points_fov = current_data['fov']
    control_points_roll_raw = current_data['roll']
    cnct = current_data['cnct']
    sync = current_data['sync']
    use_bspline = current_data['bspline']
    range_start = current_data['range_start']
    range_end = current_data['range_end']
    # 補間関数の決定
    interp_func = _spline_bspline_interpolate if use_bspline else _spline_catmull_rom_interpolate
    if use_bspline:
        self.logger.log('spline: B-Splineモード (滑らか優先/制御点近似) で実行します。')
    # --- Look-Behind: 過去の制御点の結合 ---
    prev_guide_pos = None
    prev_guide_fov = None
    prev_guide_roll = None
    if cnct:
        if hasattr(self, 'last_spline_points') and self.last_spline_points:
            last_end_pos = self.last_spline_points[-1]
            last_end_fov = self.last_spline_fov[-1]
            last_end_roll = self.last_spline_roll[-1]
            control_points_pos.insert(0, last_end_pos)
            control_points_fov.insert(0, last_end_fov)
            control_points_roll_raw.insert(0, last_end_roll)
            self.logger.log(f'spline: cnctを検出。前回の終点を始点として使用します。')
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
    next_sync_target_vel = None
    if next_text and 'spline' in next_text:
        next_data = _spline_parse_points(next_text, None) 
        if next_data and next_data['pos']:
            if next_data['cnct']:
                next_guide_pos = next_data['pos'][0]
                next_guide_fov = next_data['fov'][0]
                next_guide_roll = next_data['roll'][0]
                self.logger.log(f'spline: 次のsplineコマンド(cnctあり)を参照して終点を滑らかにします。')
            # Next Velocity Calculation for Sync
            if sync and next_dur and next_dur > 0:
                next_points = next_data['pos'][:] 
                if next_data['cnct']:
                    next_points.insert(0, control_points_pos[-1])
                # 次の区間がB-Splineかどうかで距離計算の補間関数を変える
                next_interp_func = _spline_bspline_interpolate if next_data['bspline'] else _spline_catmull_rom_interpolate
                _, next_total_distance = _spline_get_arc_length_lut(next_points, len(next_points)-1, next_interp_func)
                if next_data['sync']:
                    next_sync_target_vel = next_total_distance / next_dur
                else:
                    next_ease = next_data['ease_func']
                    dt = 0.001
                    slope = (next_ease(dt) - next_ease(0)) / dt
                    next_sync_target_vel = (next_total_distance / next_dur) * slope
    # データ保存
    self.last_spline_points = deepcopy(control_points_pos)
    self.last_spline_fov = deepcopy(control_points_fov)
    self.last_spline_roll = deepcopy(control_points_roll_raw)
    if len(control_points_pos) < 2:
        self.logger.log(f"! spline: 軌道には最低2点の制御点が必要です !")
        return
    control_points_roll = _spline_unwrap_angles(control_points_roll_raw)
    # ガイドのRoll補正
    if prev_guide_roll is not None:
        diff = control_points_roll[0] - prev_guide_roll
        while diff > 180: prev_guide_roll += 360; diff -= 360
        while diff < -180: prev_guide_roll -= 360; diff += 360
    if next_guide_roll is not None:
         diff = next_guide_roll - control_points_roll[-1]
         while diff > 180: next_guide_roll -= 360; diff -= 360
         while diff < -180: next_guide_roll += 360; diff += 360
    num_segments = len(control_points_pos) - 1
    # ターゲット座標決定
    target_pos_params = current_data['target_params']
    target_pos_start = None 
    target_pos_end = None   
    try:
        if len(target_pos_params) >= 6:
            tx1, ty1, tz1 = float(eval(target_pos_params[0])), float(eval(target_pos_params[1])), float(eval(target_pos_params[2]))
            tx2, ty2, tz2 = float(eval(target_pos_params[3])), float(eval(target_pos_params[4])), float(eval(target_pos_params[5]))
            target_pos_start = Pos(tx1, ty1, tz1)
            target_pos_end = Pos(tx2, ty2, tz2)
            self.logger.log(f'spline: [移動注視点モード] で実行。')
        elif len(target_pos_params) >= 3:
            tx, ty, tz = float(eval(target_pos_params[0])), float(eval(target_pos_params[1])), float(eval(target_pos_params[2]))
            target_pos_start = Pos(tx, ty, tz)
            self.logger.log(f'spline: [静的注視点モード] で実行。注視点: ({tx}, {ty}, {tz})')
        else:
            target_pos_start = Pos(0, 1.6, 0)
            self.logger.log(f'spline: [デフォルト注視点モード] で実行。')
    except Exception as e:
        self.logger.log(f'! spline: ターゲット座標の解析に失敗: {e} !')
        target_pos_start = Pos(0, 1.6, 0)
    # LUT作成 (interp_funcを渡す)
    self.logger.log('spline: 軌道総距離を事前計算中...')
    pg_pos, ng_pos = prev_guide_pos, next_guide_pos
    pg_fov = Pos(prev_guide_fov, 0, 0) if prev_guide_fov is not None else None
    ng_fov = Pos(next_guide_fov, 0, 0) if next_guide_fov is not None else None
    pg_roll = Pos(prev_guide_roll, 0, 0) if prev_guide_roll is not None else None
    ng_roll = Pos(next_guide_roll, 0, 0) if next_guide_roll is not None else None
    control_points_roll_pos = [Pos(r, 0, 0) for r in control_points_roll]
    control_points_fov_pos = [Pos(f, 0, 0) for f in control_points_fov] 
    pos_lut, total_distance = _spline_get_arc_length_lut(control_points_pos, num_segments, interp_func, p0_guide=pg_pos, p3_guide=ng_pos)
    fov_lut, total_fov_change_pseudo = _spline_get_arc_length_lut(control_points_fov_pos, num_segments, interp_func, p0_guide=pg_fov, p3_guide=ng_fov)
    roll_lut, total_roll_change_pseudo = _spline_get_arc_length_lut(control_points_roll_pos, num_segments, interp_func, p0_guide=pg_roll, p3_guide=ng_roll)
    self.logger.log(f'spline: 軌道総距離: {total_distance:.2f} m')
    # Sync & Rate Func
    rate_func = None
    if sync:
        self.logger.log('spline: syncモード有効。')
        CUT_THRESHOLD = 0.1 
        v_prev, v_curr, v_next = 0.0, (total_distance / dur if dur > 0 else 0), 0.0
        valid_prev = False
        if len(self.lines) > 0:
            last_line = self.lines[-1]
            last_pos = last_line.end.pos
            # --- カット検出用始点の計算 ---
            # B-Splineの場合、制御点と実際の始点がズレるため、interp_funcを使って正確な始点を求める
            if cnct and range_start == 0.0:
                 # cnct時は前回のLastTransformにいるはず
                 curr_start_real = self.lastTransform.pos
            else:
                 # cnctなし、またはrange途中開始の場合、正確な位置を計算する
                 # ガイド点などのパラメータ準備
                 p1_s = control_points_pos[0]
                 p2_s = control_points_pos[1]
                 p0_s = prev_guide_pos if prev_guide_pos is not None else p1_s
                 p3_s = control_points_pos[2] if num_segments > 1 else (next_guide_pos if next_guide_pos is not None else p2_s)
                 sx = interp_func(p0_s.x, p1_s.x, p2_s.x, p3_s.x, 0.0)
                 sy = interp_func(p0_s.y, p1_s.y, p2_s.y, p3_s.y, 0.0)
                 sz = interp_func(p0_s.z, p1_s.z, p2_s.z, p3_s.z, 0.0)
                 curr_start_real = Pos(sx, sy, sz)
            dx, dy, dz = last_pos.x - curr_start_real.x, last_pos.y - curr_start_real.y, last_pos.z - curr_start_real.z
            dist_diff = sqrt(dx*dx + dy*dy + dz*dz)
            if dist_diff < CUT_THRESHOLD:
                if last_line.duration > 0:
                    dx_l, dy_l, dz_l = last_line.end.pos.x - last_line.start.pos.x, last_line.end.pos.y - last_line.start.pos.y, last_line.end.pos.z - last_line.start.pos.z
                    v_prev = sqrt(dx_l*dx_l + dy_l*dy_l + dz_l*dz_l) / last_line.duration
                    valid_prev = True
            else:
                 self.logger.log(f'spline: sync - 前回の終点と距離があるため({dist_diff:.2f}m)、速度を引き継ぎません。')
        if not valid_prev: v_prev = v_curr 
        valid_next = False
        if next_sync_target_vel is not None and next_guide_pos is not None:
            # 終点側も同様に、B-Splineだと制御点とズレる可能性があるが、
            # LUT計算で総距離を出しているので平均速度アプローチではそこまで厳密でなくても良いが、
            # カット検出においては実際の終点座標が重要。
            # ここでは control_points_pos[-1] を使っているが、B-Splineの場合はズレる。
            # ただし、next_guide_pos も次の始点制御点なので、B-Spline同士なら制御点距離でも近似できる。
            # 厳密にするなら t=1.0 の座標を計算すべきだが、現状のままでもある程度機能する。
            curr_end_pos = control_points_pos[-1]
            dx, dy, dz = curr_end_pos.x - next_guide_pos.x, curr_end_pos.y - next_guide_pos.y, curr_end_pos.z - next_guide_pos.z
            if sqrt(dx*dx + dy*dy + dz*dz) < CUT_THRESHOLD:
                v_next = next_sync_target_vel
                valid_next = True
            else:
                self.logger.log(f'spline: sync - 次回の始点と距離があるため、速度を合わせません。')
        if not valid_next: v_next = v_curr
        ratio_in = v_prev / v_curr if v_curr > 0.0001 else 1.0
        ratio_out = v_next / v_curr if v_curr > 0.0001 else 1.0
        self.logger.log(f'spline: sync - Ratio In: {ratio_in:.2f}, Ratio Out: {ratio_out:.2f}')
        def hermite_time(t):
            t2, t3 = t*t, t*t*t
            return (t3 - 2*t2 + t) * ratio_in + (-2*t3 + 3*t2) + (t3 - t2) * ratio_out
        rate_func = hermite_time
    else:
        ease_func_base = current_data['ease_func']
        if current_data['ease_name'] == 'Drift':
             rate_func = lambda t: ease_func_base(t, current_data['dx'], current_data['dy'])
        else:
             rate_func = ease_func_base
    def final_rate_func(t):
        mapped_t = range_start + t * (range_end - range_start)
        base_rate = rate_func(mapped_t)
        start_rate = rate_func(range_start)
        end_rate = rate_func(range_end)
        denom = end_rate - start_rate
        if abs(denom) < 0.0001: return 0
        return (base_rate - start_rate) / denom
    # 解像度計算
    base_p = 10 
    p = base_p
    if not sync and current_data['ease_func'] and dur > 0:
        res_multiplier = calculate_adaptive_multiplier(current_data['ease_func'], dur, current_data['dx'], current_data['dy'], self.logger)
        p = base_p / res_multiplier
        self.logger.log(f'spline p: {p} (base_p: {base_p} / mult: {res_multiplier})')
    span = 1 / 30
    spans = []
    init_dur = dur
    while dur > 0:
        min_span = min(span, dur)
        if dur - min_span < 0.01: min_span = dur
        spans.append(min_span)
        dur -= min_span
    span_size = len(spans)
    init_dur = sum(spans)
    # 座標計算の共通関数 (interp_funcを使用)
    def get_transform_at_t(t_val):
        rate = final_rate_func(t_val)
        target_distance = total_distance * rate
        global_spline_t_pos = _spline_get_t_for_distance(pos_lut, target_distance)
        target_fov_change = total_fov_change_pseudo * rate
        global_spline_t_fov = _spline_get_t_for_distance(fov_lut, target_fov_change)
        target_roll_change = total_roll_change_pseudo * rate
        global_spline_t_roll = _spline_get_t_for_distance(roll_lut, target_roll_change)
        def get_interp_val(points, global_t, p0_g, p3_g):
            seg_idx = min(floor(global_t), num_segments - 1)
            seg_t = global_t - seg_idx
            p1, p2 = points[seg_idx], points[seg_idx+1]
            p0 = points[seg_idx-1] if seg_idx > 0 else (p0_g if p0_g is not None else p1)
            p3 = points[seg_idx+2] if seg_idx < (len(points)-2) else (p3_g if p3_g is not None else p2)
            if isinstance(p1, Pos):
                return interp_func(p0.x, p1.x, p2.x, p3.x, seg_t), \
                       interp_func(p0.y, p1.y, p2.y, p3.y, seg_t), \
                       interp_func(p0.z, p1.z, p2.z, p3.z, seg_t)
            else:
                return interp_func(p0, p1, p2, p3, seg_t)
        px, py, pz = get_interp_val(control_points_pos, global_spline_t_pos, pg_pos, ng_pos)
        fov = get_interp_val(control_points_fov, global_spline_t_fov, pg_fov.x if pg_fov else None, ng_fov.x if ng_fov else None)
        roll = get_interp_val(control_points_roll, global_spline_t_roll, pg_roll.x if pg_roll else None, ng_roll.x if ng_roll else None)
        current_target_pos = None
        if target_pos_end is None:
            current_target_pos = target_pos_start
        else:
            tx = target_pos_start.x + (target_pos_end.x - target_pos_start.x) * rate
            ty = target_pos_start.y + (target_pos_end.y - target_pos_start.y) * rate
            tz = target_pos_start.z + (target_pos_end.z - target_pos_start.z) * rate
            current_target_pos = Pos(tx, ty, tz)
        rot_obj = _spline_calculate_look_at(Pos(px, py, pz), current_target_pos)
        rot_obj.z = roll
        return Transform(Pos(px, py, pz), rot_obj, fov)
    # 0秒の場合の処理
    if init_dur == 0:
        self.logger.log('spline: 期間が0のため、Duration 0のラインを生成します。')
        endTrans = get_transform_at_t(1.0)
        new_line = Line(0)
        new_line.visibleDict = deepcopy(self.visibleObject.state)
        # 始点も正しく計算
        if cnct and range_start == 0.0: new_line.start = deepcopy(self.lastTransform)
        else: new_line.start = get_transform_at_t(0.0)
        new_line.end = endTrans
        self.lines.append(new_line)
        self.lastTransform = new_line.end
        return
    # メインループ
    for i in range(span_size):
        new_line = Line(spans[i])
        new_line.visibleDict = deepcopy(self.visibleObject.state)
        t_end = sum(spans[:(i+1)]) / init_dur
        if t_end > 1: t_end = 1
        endTrans = get_transform_at_t(t_end)
        new_line.end = endTrans
        if i == 0:
            # 始点補正: cnctが無い場合はt=0から計算
            if cnct and range_start == 0.0:
                 new_line.start = deepcopy(self.lastTransform)
            else:
                 new_line.start = get_transform_at_t(0.0)
        else:
            new_line.start = deepcopy(self.lastTransform)
        self.lines.append(new_line)
        self.lastTransform = new_line.end
        self.logger.log(new_line.start)
