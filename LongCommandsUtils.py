import os
import json
import re

from random import random
from math import atan2, pi, sin, cos, degrees, sqrt, floor
from copy import deepcopy
from BasicElements import Pos, Rot, Line, Transform
from shake_data import SHAKE_LIST
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


def _spline_get_arc_length_lut(control_points, num_segments, samples_per_segment=100):
    """
    【距離基準のためのパス1】
    スプライン軌道の総距離を計算し、
    (距離, グローバルt) のルックアップテーブル(LUT)を作成する。
    """
    lut = [(0.0, 0.0)] # (distance, global_t)
    total_distance = 0.0
    last_pos = control_points[0] # 最初の制御点
    for i in range(num_segments):
        # このセグメント(i -> i+1)の制御点を取得
        p1 = control_points[i]
        p2 = control_points[i + 1]
        p0 = control_points[i - 1] if i > 0 else p1
        p3 = control_points[i + 2] if i < (num_segments - 1) else p2
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
            dist = sqrt(dx*dx + dy*dy + dz*dz) # math. を削除
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


def get_shake_value(shake_data, frame):
    if not shake_data:
        return 0
    first_frame_number = shake_data[0][0]
    last_frame_number = shake_data[-1][0]
    loop_duration = last_frame_number - first_frame_number
    if loop_duration > 0:
        frame = ((frame - first_frame_number) % loop_duration) + first_frame_number
    elif frame > last_frame_number:
        return shake_data[-1][1]
    for i in range(len(shake_data) - 1):
        if shake_data[i][0] <= frame < shake_data[i+1][0]:
            t = (frame - shake_data[i][0]) / (shake_data[i+1][0] - shake_data[i][0])
            return shake_data[i][1] * (1 - t) + shake_data[i+1][1] * t
    return shake_data[-1][1]


def spline(self, text, dur): 
    """
    Catmull-Rom スプライン補間を行う。(距離基準 / ターゲット追従モード専用)
    q_... 形式の座標 (8要素: px,py,pz,rx,ry,rz,fov) を補間する。
    モード 1a (静的注視点): SPLINE, tx,ty,tz, q_..., q_...
    モード 1b (移動注視点): SPLINE, tx1,ty1,tz1,tx2,ty2,tz2, q_..., q_...
    モード 2 (デフォルト): SPLINE, q_..., q_... (注視点は 0, 1.6, 0)
    """
    self.logger.log(f'spline コマンド解析開始: "{text}"')
    param_text = re.split('[,_]', text, 1)[-1]
    params_list = re.split('[,]', param_text) 
    control_point_strings = [] 
    target_pos_params = []     
    ease_name_str = None       
    # 1. パラメータを「q_形式」「数値」「イージング名」に分類
    for param in params_list:
        param_stripped = param.strip()
        if not param_stripped:
            continue
        if param_stripped.startswith('q_'):
            control_point_strings.append(param_stripped)
        elif param_stripped[0].isalpha(): 
            if ease_name_str is None:
                ease_name_str = param_stripped
        else:
            target_pos_params.append(param_stripped)
    # 2. イージング関数を取得
    ease_func, dx, dy, easetype_name = (None, 6, 6, None)
    if ease_name_str:
        ease_func, dx, dy, easetype_name = parse_easing_func(ease_name_str, self.logger, log_prefix='spline ')
    is_linear = ease_func is None
    if is_linear:
        ease_func = lambda t: t
    # 3. ターゲット座標の決定
    target_pos_start = None 
    target_pos_end = None   
    try:
        if len(target_pos_params) >= 6:
            # モード1b: 移動注視点
            tx1 = float(eval(target_pos_params[0]))
            ty1 = float(eval(target_pos_params[1]))
            tz1 = float(eval(target_pos_params[2]))
            tx2 = float(eval(target_pos_params[3]))
            ty2 = float(eval(target_pos_params[4]))
            tz2 = float(eval(target_pos_params[5]))
            target_pos_start = Pos(tx1, ty1, tz1)
            target_pos_end = Pos(tx2, ty2, tz2)
            self.logger.log(f'spline: [移動注視点モード] で実行。({tx1},{ty1},{tz1}) -> ({tx2},{ty2},{tz2})')
        elif len(target_pos_params) >= 3:
            # モード1a: 静的注視点
            tx = float(eval(target_pos_params[0]))
            ty = float(eval(target_pos_params[1]))
            tz = float(eval(target_pos_params[2]))
            target_pos_start = Pos(tx, ty, tz)
            self.logger.log(f'spline: [静的注視点モード] で実行。注視点: ({tx}, {ty}, {tz})')
        else:
            # デフォルトの静的注視点を使用
            target_pos_start = Pos(0, 1.6, 0)
            self.logger.log(f'spline: [デフォルト注視点モード] で実行。注視点: (0, 1.6, 0)')
    except Exception as e:
        self.logger.log(f'! spline: ターゲット座標の解析に失敗 ({target_pos_params}): {e} !')
        target_pos_start = Pos(0, 1.6, 0) # デフォルトにフォールバック
        self.logger.log(f'spline: [デフォルト注視点モード] にフォールバックします。')
    # 4. 座標データを解析
    control_points_pos = [] 
    control_points_fov = [] 
    for i, point_str in enumerate(control_point_strings):
        try:
            parts = point_str.split('_')
            if len(parts) >= 8:
                px = float(parts[1])
                py = float(parts[2])
                pz = float(parts[3])
                fov = float(parts[7])
                control_points_pos.append(Pos(px, py, pz))
                control_points_fov.append(fov)
            else:
                raise ValueError("q_ 形式のデータが短すぎます (8要素必要)")
        except Exception as e:
            self.logger.log(f'! spline: 座標 {i+1} ({point_str}) の解析に失敗: {e} !')
            return
    # 5. 制御点の数をチェック
    if len(control_points_pos) < 2:
        self.logger.log(f"! spline: 軌道には最低2点の制御点が必要です (現在 {len(control_points_pos)} 点) !")
        return
    num_segments = len(control_points_pos) - 1
    # 6. 【距離基準のためのパス1】 軌道の総距離とLUTを事前計算
    self.logger.log('spline: 軌道の総距離を事前計算中...')
    # (FOVも同様のLUTを作成)
    control_points_fov_pos = [Pos(f, 0, 0) for f in control_points_fov] # 1DデータをPosとして扱う
    pos_lut, total_distance = _spline_get_arc_length_lut(control_points_pos, num_segments)
    fov_lut, total_fov_change_pseudo = _spline_get_arc_length_lut(control_points_fov_pos, num_segments)
    self.logger.log(f'spline: 軌道総距離: {total_distance:.2f} m')
    # 7. アダプティブ解像度と時間分割
    base_p = 10 
    p = base_p
    if not is_linear and dur > 0:
        res_multiplier = calculate_adaptive_multiplier(ease_func, dur, dx, dy, self.logger)
        p = base_p / res_multiplier
        self.logger.log(f'spline p: {p} (base_p: {base_p} / mult: {res_multiplier})')
    # (総距離に基づいて span を決定するロジックに変更可能だが、既存のロジックも機能する)
    span = max(1/30, dur/max(36, 36 * (len(control_points_pos) / 4)))
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
        end_target = target_pos_end if target_pos_end else target_pos_start
        endRot = _spline_calculate_look_at(endPos, end_target)
        self.lastTransform = Transform(endPos, endRot, endFov)
        return
    # 8. メインループ
    for i in range(span_size):
        new_line = Line(spans[i])
        new_line.visibleDict = deepcopy(self.visibleObject.state)
        t_end = sum(spans[:(i+1)]) / init_dur
        if t_end > 1: t_end = 1
        # (A) イージングされた「時間の進捗率」 (0.0 -> 1.0)
        rate = ease_func(t_end, dx, dy) if easetype_name == 'Drift' else ease_func(t_end)
        # 9. 【距離基準のためのパス2】 rate を「距離」に変換し、t を逆引き
        # (位置)
        target_distance = total_distance * rate
        global_spline_t_pos = _spline_get_t_for_distance(pos_lut, target_distance)
        # (FOV)
        target_fov_change = total_fov_change_pseudo * rate
        global_spline_t_fov = _spline_get_t_for_distance(fov_lut, target_fov_change)
        # 10. グローバル t を「セグメント」と「ローカル t」に分解
        # (位置)
        segment_index_pos = min(floor(global_spline_t_pos), num_segments - 1)
        segment_t_pos = global_spline_t_pos - segment_index_pos 
        # (FOV)
        segment_index_fov = min(floor(global_spline_t_fov), num_segments - 1)
        segment_t_fov = global_spline_t_fov - segment_index_fov 
        # 11. 補間に必要な4点を取得
        # (位置)
        p1_pos = control_points_pos[segment_index_pos]
        p2_pos = control_points_pos[segment_index_pos + 1]
        p0_pos = control_points_pos[segment_index_pos - 1] if segment_index_pos > 0 else p1_pos
        p3_pos = control_points_pos[segment_index_pos + 2] if segment_index_pos < (num_segments - 1) else p2_pos
        # (FOV)
        p1_fov = control_points_fov[segment_index_fov]
        p2_fov = control_points_fov[segment_index_fov + 1]
        p0_fov = control_points_fov[segment_index_fov - 1] if segment_index_fov > 0 else p1_fov
        p3_fov = control_points_fov[segment_index_fov + 2] if segment_index_fov < (num_segments - 1) else p2_fov
        # 12. Catmull-Rom 補間を実行
        # 位置(Pos)
        end_px = _spline_catmull_rom_interpolate(p0_pos.x, p1_pos.x, p2_pos.x, p3_pos.x, segment_t_pos)
        end_py = _spline_catmull_rom_interpolate(p0_pos.y, p1_pos.y, p2_pos.y, p3_pos.y, segment_t_pos)
        end_pz = _spline_catmull_rom_interpolate(p0_pos.z, p1_pos.z, p2_pos.z, p3_pos.z, segment_t_pos)
        endPos = Pos(end_px, end_py, end_pz)
        # FOV (1D補間)
        endFov = _spline_catmull_rom_interpolate(p0_fov, p1_fov, p2_fov, p3_fov, segment_t_fov)
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
        # 14. new_line の設定
        new_line.end = Transform(endPos, endRot, endFov)
        if i == 0:
            startPos = control_points_pos[0]
            startFov = control_points_fov[0]
            startRot = _spline_calculate_look_at(startPos, target_pos_start)
            new_line.start = Transform(startPos, startRot, startFov)
        else:
            new_line.start = deepcopy(self.lastTransform)
        # 15. 終点スナップ処理
        if i == span_size - 1:
            endPos = control_points_pos[-1]
            endFov = control_points_fov[-1]
            endRot = None
            if target_pos_end is None:
                endRot = _spline_calculate_look_at(endPos, target_pos_start)
            else:
                endRot = _spline_calculate_look_at(endPos, target_pos_end)
            new_line.end = Transform(endPos, endRot, endFov)
        self.lines.append(new_line)
        self.lastTransform = new_line.end
        self.logger.log(new_line.start)


def rvib(self, dur, text, line):
    param_str = text[4:]
    if param_str and (param_str[0] == '_' or param_str[0] == ','):
        param_str = param_str[1:]
    params_list = [p.strip() for p in param_str.split(',') if p.strip()]
    numeric_params_list = params_list
    ease_func = None
    dx = 6
    dy = 6
    easetype_name = None
    if params_list:
        last_param_str = params_list[-1]
        u_text = last_param_str.upper()
        found_ease_obj = False
        if any([c.isalpha() for c in u_text]):
            ease_func, dx, dy, easetype_name = parse_easing_func(u_text, self.logger, log_prefix='rvib ')
            if ease_func:
                found_ease_obj = True
        if found_ease_obj:
            numeric_params_list = params_list[:-1]
    if ease_func is None:
        ease_func = lambda t: t
    if not numeric_params_list:
        self.logger.log(f'! rvib: シェイク名が指定されていません !')
        self.lines.append(line)
        return
    shake_name = numeric_params_list[0].upper()
    strength = float(numeric_params_list[1]) if len(numeric_params_list) > 1 and numeric_params_list[1] else 1.0
    speed = float(numeric_params_list[2]) if len(numeric_params_list) > 2 and numeric_params_list[2] else 1.0
    pos_scale = float(numeric_params_list[3]) if len(numeric_params_list) > 3 and numeric_params_list[3] else 1.0
    rot_scale = float(numeric_params_list[4]) if len(numeric_params_list) > 4 and numeric_params_list[4] else 1.0
    if shake_name not in SHAKE_LIST:
        self.logger.log(f'! rvib: 手ブレ名 "{shake_name}" が見つかりません !')
        self.lines.append(line)
        return
    shake_info = SHAKE_LIST[shake_name]
    shake_fps = shake_info[1]
    shake_data = shake_info[2]
    ixp, iyp, izp = line.start.pos.unpack()
    ixr, iyr, izr = line.start.rot.unpack()
    lxp, lyp, lzp = line.end.pos.unpack()
    lxr, lyr, lzr = line.end.rot.unpack()
    iFOV = line.start.fov
    lFOV = line.end.fov
    spans = []
    span = 1 / 60
    temp_dur = dur
    while temp_dur > 0:
        min_span = min(span, temp_dur)
        if temp_dur - min_span < 0.01:
            min_span = temp_dur
        spans.append(min_span)
        temp_dur -= min_span
    span_size = len(spans)
    total_duration = sum(spans)
    if total_duration == 0:
        self.lines.append(line)
        return
    last_rate = 0.0
    for i in range(span_size):
        new_line = Line(spans[i])
        new_line.visibleDict = deepcopy(line.visibleDict)
        t_start = sum(spans[:i]) / total_duration
        t_end = sum(spans[:(i+1)]) / total_duration
        if t_end > 1: t_end = 1
        rate_start = 0.0
        rate_end = 0.0
        if easetype_name == 'Drift':
            rate_start = ease_func(t_start, dx, dy)
            rate_end = ease_func(t_end, dx, dy)
        else:
            rate_start = ease_func(t_start)
            rate_end = ease_func(t_end)
        delta_t = t_end - t_start
        delta_rate = rate_end - last_rate
        speed_multiplier = delta_rate / delta_t if delta_t > 1e-6 else 0
        last_rate = rate_end
        frame = t_end * total_duration * shake_fps * speed
        dx_s = get_shake_value(shake_data.get(('location', 0)), frame) * strength * pos_scale * speed_multiplier
        dy_s = get_shake_value(shake_data.get(('location', 1)), frame) * strength * pos_scale * speed_multiplier
        dz_s = get_shake_value(shake_data.get(('location', 2)), frame) * strength * pos_scale * speed_multiplier
        drx_s = get_shake_value(shake_data.get(('rotation_euler', 0)), frame) * strength * rot_scale * 180 / pi * speed_multiplier
        dry_s = get_shake_value(shake_data.get(('rotation_euler', 1)), frame) * strength * rot_scale * 180 / pi * speed_multiplier
        drz_s = get_shake_value(shake_data.get(('rotation_euler', 2)), frame) * strength * rot_scale * 180 / pi * speed_multiplier
        if i == 0:
            new_line.start.pos = Pos(ixp, iyp, izp)
            new_line.start.rot = Rot(ixr, iyr, izr)
            new_line.start.fov = iFOV
        else:
            new_line.start = deepcopy(self.lastTransform)
        px2 = ixp + (lxp - ixp) * rate_end
        py2 = iyp + (lyp - iyp) * rate_end
        pz2 = izp + (lzp - izp) * rate_end
        rx2 = ixr + (lxr - ixr) * rate_end
        ry2 = iyr + (lyr - iyr) * rate_end
        rz2 = izr + (lzr - izr) * rate_end
        fov2 = iFOV + (lFOV - iFOV) * rate_end
        new_line.end.pos = Pos(px2 + dx_s, py2 + dy_s, pz2 + dz_s)
        new_line.end.rot = Rot(rx2 + drx_s, ry2 + dry_s, rz2 + drz_s)
        new_line.end.fov = fov2
        if i == span_size - 1:
            new_line.end.pos = Pos(lxp, lyp, lzp)
            new_line.end.rot = Rot(lxr, lyr, lzr)
            new_line.end.fov = lFOV
        self.lastTransform = new_line.end
        self.lines.append(new_line)


def rotate(self, text, dur):
    params_list = []
    if text:
        params_list = text.split(',')
    numeric_params_list = []
    special_params_list = []
    found_ease_name = False
    for param in params_list:
        param_stripped = param.strip()
        if not found_ease_name and any([c.isalpha() for c in param_stripped]):
            found_ease_name = True
        if found_ease_name:
            special_params_list.append(param_stripped)
        else:
            numeric_params_list.append(param)
    numeric_text = ','.join(numeric_params_list)
    if any([c.isalpha() for c in numeric_text]): 
        self.logger.log(
            f'数値パラメータ {numeric_text} に英字を確認。セキュリティの問題上、プログラムを強制終了します。')
        input()
        exit()
    ease_name_str = None
    if len(special_params_list) > 0:
        ease_name_str = special_params_list[0].strip()
    easefunc = None 
    dx = 6 
    dy = 6 
    easetype_name = None
    if ease_name_str and ease_name_str != '': 
        easefunc, dx, dy, easetype_name = parse_easing_func(ease_name_str, self.logger, log_prefix='rotate ')
    # def_value
    r1 = 3
    h1 = 3
    a = 1
    o = 1.0
    s1 = 0
    j = 0
    w = 360
    if len(numeric_text) > 0:
        param = [eval(i) for i in numeric_text.split(',') if i.strip()]
        if len(param) > 0:
            r1 = param[0]
        if len(param) > 1:
            h1 = param[1]
        if len(param) > 2:
            a = param[2]
        if len(param) > 3:
            o = param[3]
        if len(param) > 4:
            s1 = param[4]
        if len(param) > 5:
            j = param[5]
        if len(param) > 6:
            w = param[6]
        if len(param) > 7:
            r2 = param[7]
        else:
            r2 = r1
        if len(param) > 8:
            h2 = param[8]
        else:
            h2 = h1
        if len(param) > 9:
            s2 = param[9]
        else:
            s2 = s1
    is_linear = False
    if easefunc is None:
        easefunc = lambda t: t 
        is_linear = True
        self.logger.log('rotate で線形補間を使用します。')
    else:
        self.logger.log(f'rotate でイージング {easetype_name} を使用します。')
    self.logger.log(
        f'パラメータ r1:{r1} h1:{h1} a:{a} o:{o} s1:{s1} j:{j} w:{w} r2:{r2} h2:{h2} s2:{s2}')
    base_p = 5 if w < 360 else 10
    p = base_p
    if not is_linear and dur > 0:
        res_multiplier = calculate_adaptive_multiplier(easefunc, dur, dx, dy, self.logger)
        
        p = base_p / res_multiplier 
        
        self.logger.log(f'p: {p} (base_p: {base_p} / mult: {res_multiplier})')
    else:
        self.logger.log(f'線形補間。基本解像度を使用。 p: {p}')
    span = max(1/30, dur/abs(w/p))
    spans = []
    init_dur = dur
    while dur > 0:
        min_span = min(span, dur)
        if dur - min_span < 0.01:
            min_span = dur
        spans.append(min_span)
        dur -= min_span
    span_size = len(spans)
    rate_start = 0.0 
    theta = 2*pi * rate_start * (w/360) - 1/2*pi + j*2*pi/360
    r = r1 + (r2-r1) * rate_start
    h = h1 + (h2-h1) * rate_start
    s = s1 + (s2-s1) * rate_start
    angle = atan2(h-a, r)
    px = round(r*cos(theta), 3)
    pz = round(r*sin(theta)+o, 3)
    rx = degrees(angle)
    ry = -degrees(theta)+270
    self.lastTransform = Transform(Pos(px, h, pz), Rot(rx, ry, s), self.fov)
    for i in range(span_size):
        new_line = Line(spans[i])
        new_line.visibleDict = deepcopy(self.visibleObject.state)
        new_line.start = deepcopy(self.lastTransform)
        t = sum(spans[:(i+1)]) / init_dur
        if t > 1:
            t = 1
        rate = 0.0
        if easetype_name == 'Drift':
            rate = easefunc(t, dx, dy) 
        else:
            rate = easefunc(t)
        theta = 2*pi * rate * (w/360) - 1/2*pi + j*2*pi/360
        r = r1 + (r2-r1) * rate
        h = h1 + (h2-h1) * rate
        s = s1 + (s2-s1) * rate
        angle = atan2(h-a, r)
        px = round(r*cos(theta), 3)
        pz = round(r*sin(theta)+o, 3)
        rx = degrees(angle)
        ry = -degrees(theta)+270
        endPos = Pos(px, h, pz)
        endRot = Rot(rx, ry, s)
        new_line.end = Transform(endPos, endRot, self.fov)
        self.logger.log(new_line.start)
        self.lines.append(new_line)
        self.lastTransform = new_line.end


def rot(self, dur, text, line):
    self.logger.log(f'rot コマンド解析開始: "{text}"')
    params_list = re.split('[_,]',text[3:]) 
    numeric_params_list = []
    special_params_list = []
    found_ease_name = False
    for param in params_list:
        param_stripped = param.strip()
        if not found_ease_name and any([c.isalpha() for c in param_stripped]):
            found_ease_name = True
        if found_ease_name:
            special_params_list.append(param_stripped)
        else:
            numeric_params_list.append(param)
    ease_func = None
    dx = 6
    dy = 6
    easetype_name = None
    if found_ease_name:
        ease_name_str = special_params_list[0]
        ease_func, dx, dy, easetype_name = parse_easing_func(ease_name_str, self.logger, log_prefix='rot ')
    is_linear = ease_func is None
    if is_linear:
        ease_func = lambda t: t
    n = None
    o = 0
    for param in numeric_params_list:
        if param.strip() == '':
            continue
        try:
            if n is None:
                n = float(eval(param))
                assert(n != 0)
            else:
                o = float(eval(param))
        except:
            self.logger.log(f'! rotの後の数値が不正です !')
            self.logger.log(f'rot: False としますが、意図しない演出になっています。')
            self.lines.append(line)
            self.logger.log(line.start)
            self.logger.log(line.end)
            return
    ixp, iyp, izp = line.start.pos.unpack()
    ixr, iyr, izr = line.start.rot.unpack()
    lxp, lyp, lzp = line.end.pos.unpack()
    lxr, lyr, lzr = line.end.rot.unpack()
    ixr = (ixr + 180) % 360 - 180
    lxr = (lxr + 180) % 360 - 180
    iFOV = line.start.fov
    lFOV = line.end.fov
    ir = sqrt(ixp**2+izp**2)
    lr = sqrt(lxp**2+lzp**2)
    itheta = atan2(izp, ixp) % (2*pi)
    ltheta = atan2(lzp, lxp) % (2*pi)
    dtheta = 0.0 
    if n is None:
        self.logger.log(f'! rotのnパラメータが指定されていません !')
        n = 0
    if n > 0:
        if ltheta > itheta:
            dtheta = ltheta-itheta + 2*pi*(n-1)
        else:
            dtheta = ltheta-itheta + 2*pi*n
    elif n < 0:
        if ltheta > itheta:
            dtheta = ltheta-itheta + 2*pi*n
        else:
            dtheta = ltheta-itheta + 2*pi*(n+1)
    else:
        self.logger.log(f'! rotのnパラメータが 0 です !')
        self.logger.log(f'rot: False としますが、意図しない演出になっています。')
        self.lines.append(line)
        self.logger.log(line.start)
        self.logger.log(line.end)
        return
    base_p = 5 if dtheta < 2*pi else 10
    p = base_p
    if not is_linear and dur > 0:
        res_multiplier = calculate_adaptive_multiplier(ease_func, dur, dx, dy, self.logger)
        p = base_p / res_multiplier
        self.logger.log(f'rot p: {p} (base_p: {base_p} / mult: {res_multiplier})')
    else:
        self.logger.log(f'rot 線形補間。基本解像度を使用。 p: {p}')
    span = max(1/30, dur/degrees(abs(dtheta)/p))
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
        self.lines.append(line)
        return
    self.lastTransform = line.start 
    for i in range(span_size):
        new_line = Line(spans[i])
        # new_line.visibleDict = deepcopy(self.visibleObject.state)
        new_line.visibleDict = deepcopy(line.visibleDict)
        new_line.start = deepcopy(self.lastTransform)
        t_end = sum(spans[:(i+1)]) / init_dur
        if t_end > 1: t_end = 1
        rate_end = 0.0
        if easetype_name == 'Drift':
            rate_end = ease_func(t_end, dx, dy)
        else:
            rate_end = ease_func(t_end)
        theta2 = itheta + dtheta * rate_end
        r2 = ir + (lr-ir) * rate_end
        h2 = iyp + (lyp-iyp) * rate_end
        s2 = izr + (lzr-izr) * rate_end
        fov2 = iFOV + (lFOV-iFOV) * rate_end
        rx2 = ixr + (lxr-ixr) * rate_end
        px = round(r2*cos(theta2), 3)
        pz = round(r2*sin(theta2)+o, 3)
        ry = -degrees(theta2)+270
        new_line.end.pos = Pos(px, h2, pz)
        new_line.end.rot = Rot(rx2, ry, s2)
        new_line.end.fov = fov2
        self.logger.log(new_line.start)
        self.lines.append(new_line)
        self.lastTransform = new_line.end


def vibro(self, dur, param):
    steps = []
    bpm = self.bpm
    span = max(1/30, param*60/bpm)
    while dur > 0:
        steps.append(min(span, dur))
        dur -= span
        span *= (0.9 + random()*0.2)
    ans = []
    for step in steps:
        new_line = Line(step)
        new_line.visibleDict = deepcopy(self.visibleObject.state)
        new_line.start = deepcopy(self.lastTransform)
        new_line.end = deepcopy(self.lastTransform)
        dx = round(random()/6, 3)-1/12
        dy = round(random()/6, 3)-1/12
        dz = round(random()/6, 3)-1/12
        new_line.end.pos.x += dx
        new_line.end.pos.y += dy
        new_line.end.pos.z += dz
        self.lastTransform = new_line.end
        ans.append(new_line)
        self.logger.log(new_line.start)
        self.lines.append(new_line)


def vib(self, dur, text, line):
    param_str = text[3:]
    params_list = param_str.split(',')
    numeric_param_str = params_list[0]
    special_params_list = params_list[1:]
    ease_func = None
    dx = 6
    dy = 6
    easetype_name = None
    if len(special_params_list) > 0:
        ease_name_str = special_params_list[0].strip()
        if ease_name_str:
            ease_func, dx, dy, easetype_name = parse_easing_func(ease_name_str, self.logger, log_prefix='vib ')
    is_linear = ease_func is None
    if is_linear:
        ease_func = lambda t: t
    try:
        param = float(eval(numeric_param_str))
    except:
        self.logger.log(f'! vibの後の数値が不正です !')
        self.logger.log(f'vib: False としますが、意図しない演出になっています。')
        self.lines.append(line)
        self.logger.log(line.start)
        self.logger.log(line.end)
        return
    ixp, iyp, izp = line.start.pos.unpack()
    ixr, iyr, izr = line.start.rot.unpack()
    lxp, lyp, lzp = line.end.pos.unpack()
    lxr, lyr, lzr = line.end.rot.unpack()
    iyr = iyr % 360
    lyr = lyr % 360
    iyr = iyr if abs(lyr-iyr) < 180 else (iyr+180) % 360 - 180
    lyr = lyr if abs(lyr-iyr) < 180 else (lyr+180) % 360 - 180
    iFOV = line.start.fov
    lFOV = line.end.fov
    spans = []
    bpm = self.bpm
    span = max(1/30, param*60/bpm) 
    init_dur = dur
    if not is_linear and init_dur > 0:
        res_multiplier = calculate_adaptive_multiplier(ease_func, init_dur, dx, dy, self.logger)
        if res_multiplier > 1:
            span = span / res_multiplier
            self.logger.log(f'vib span 調整 (/{res_multiplier:.2f}) -> {span}')
    while dur > 0:
        min_span = min(span, dur)
        if dur - min_span < 0.01:
            min_span = dur
        spans.append(min_span)
        dur -= min_span
    span_size = len(spans)
    init_dur = sum(spans) 
    if init_dur == 0:
        self.lines.append(line)
        return
    last_rate = 0.0
    for i in range(span_size):
        new_line = Line(spans[i])
        # new_line.visibleDict = deepcopy(self.visibleObject.state)
        new_line.visibleDict = deepcopy(line.visibleDict)
        new_line.start = deepcopy(self.lastTransform)
        if i == 0:
            new_line.start.pos = Pos(ixp, iyp, izp)
            new_line.start.rot = Rot(ixr, iyr, izr)
            new_line.start.fov = iFOV
        t_start = sum(spans[:i]) / init_dur
        t_end = sum(spans[:(i+1)]) / init_dur
        if t_end > 1: t_end = 1
        rate_start = 0.0
        rate_end = 0.0
        if easetype_name == 'Drift':
            rate_start = ease_func(t_start, dx, dy)
            rate_end = ease_func(t_end, dx, dy)
        else:
            rate_start = ease_func(t_start)
            rate_end = ease_func(t_end)
        delta_t = t_end - t_start
        delta_rate = rate_end - last_rate
        speed_multiplier = delta_rate / delta_t if delta_t > 1e-6 else 0
        last_rate = rate_end
        dx_r = (round(random()/6, 3)-1/12) * speed_multiplier
        dy_r = (round(random()/6, 3)-1/12) * speed_multiplier
        dz_r = (round(random()/6, 3)-1/12) * speed_multiplier
        px2 = ixp + (lxp-ixp) * rate_end
        py2 = iyp + (lyp-iyp) * rate_end
        pz2 = izp + (lzp-izp) * rate_end
        rx2 = ixr + (lxr-ixr) * rate_end
        ry2 = iyr + (lyr-iyr) * rate_end
        rz2 = izr + (lzr-izr) * rate_end
        fov2 = iFOV + (lFOV-iFOV) * rate_end
        new_line.end.pos = Pos(px2+dx_r, py2+dy_r, pz2+dz_r)
        new_line.end.rot = Rot(rx2, ry2, rz2)
        new_line.end.fov = fov2
        if i == span_size-1:
            new_line.end.pos = Pos(lxp, lyp, lzp)
            new_line.end.rot = Rot(lxr, lyr, lzr)
            new_line.end.fov = lFOV
        self.lastTransform = new_line.end
        self.logger.log(new_line.start)
        self.lines.append(new_line)


def script(self, text, dur):
    path_dir = self.path_obj.parent
    script_dir = os.path.join(path_dir, 'script')
    if not os.path.exists(script_dir):
        self.logger.log('! script フォルダが見つかりません。プログラムを終了します !')
        input()
        exit()
    script_file = text[7:] + '.json'
    script_path = os.path.join(script_dir, script_file)
    if not os.path.exists(script_path):
        self.logger.log(f'! {script_file} が見つかりません。プログラムを終了します !')
        input()
        exit()
    f = open(script_path, 'rb')
    j = json.load(f)
    if 'Movements' in j:
        sum_dur = 0
        movements = j['Movements']
        for m in movements:
            sum_dur += m['Duration']
            if 'Delay' in m and m['Delay'] > 0:
                self.logger.log(f'! {script_file} にDelay設定があります !')
                self.logger.log(f'Delayは非対応のため意図しない演出になっています。')
            if 'EaseTransition' in m and m['EaseTransition']:
                self.logger.log(f'! {script_file} にEaseTransition設定があります !')
                self.logger.log(f'EaseTransitionは非対応のため意図しない演出になっています。')
        ratio_dur = dur / sum_dur
        script_lines = []
        for m in movements:
            new_line = Line(m['Duration'] * ratio_dur)
            new_line.visibleDict = deepcopy(self.visibleObject.state)
            v = m['StartPos']
            new_line.start.pos = Pos(v['x'], v['y'], v['z'])
            new_line.start.fov = v['FOV']
            v = m['StartRot']
            new_line.start.rot = Rot(v['x'], v['y'], v['z'])
            if 'StartHeadOffset' in m:
                v = m['StartHeadOffset']
                new_line.startHeadOffset = Pos(v['x'], v['y'], v['z'])
            v = m['EndPos']
            new_line.end.pos = Pos(v['x'], v['y'], v['z'])
            new_line.end.fov = v['FOV']
            v = m['EndRot']
            new_line.end.rot = Rot(v['x'], v['y'], v['z'])
            if 'EndHeadOffset' in m:
                v = m['EndHeadOffset']
                new_line.endHeadOffset = Pos(v['x'], v['y'], v['z'])
            if 'TurnToHead' in m:
                new_line.turnToHead = m['TurnToHead']
            if 'TurnToHeadHorizontal' in m:
                new_line.turnToHeadHorizontal = m['TurnToHeadHorizontal']
            script_lines.append(new_line)
        size = len(script_lines)
        for i in range(size):
            l = script_lines[i]
            if l.duration < 0.017:  #間隔を1/60sec以上にする
                if i+1 < size and l.end == script_lines[i+1].start:
                    script_lines[i+1].start = l.start
                    script_lines[i+1].duration += l.duration
                    l.duration = 0
        for s in script_lines:
            if s.duration == 0:
                continue
            if s.duration < 0.01:
                if self.lines[-1].end == s.start:
                    self.lines[-1].end = s.end
                    self.lines[-1].duration += s.duration
                    continue
            self.logger.log(s.start)
            self.lines.append(s)
