from copy import deepcopy
from math import cos, pi, sin, sqrt, ceil, pow
import re

from BasicElements import Pos, Rot, Line, Transform

# Reference: https://easings.net/

easetypes = ['InSine',
             'OutSine',
             'InOutSine',
             'InCubic',
             'OutCubic',
             'InOutCubic',
             'InQuint',
             'OutQuint',
             'InOutQuint',
             'InCirc',
             'OutCirc',
             'InOutCirc',
             'InElastic',
             'OutElastic',
             'InOutElastic',
             'InQuad',
             'OutQuad',
             'InOutQuad',
             'InQuart',
             'OutQuart',
             'InOutQuart',
             'InExpo',
             'OutExpo',
             'InOutExpo',
             'InBack',
             'OutBack',
             'InOutBack',
             'InBounce',
             'OutBounce',
             'InOutBounce',
             'Drift',
             ]


def make_in_pow(p):
    def InPow(t):
        return pow(t, p)
    InPow.__name__ = f'In{p}'
    return InPow


def make_out_pow(p):
    def OutPow(t):
        return 1 - pow(1 - t, p)
    OutPow.__name__ = f'Out{p}'
    return OutPow


def make_in_out_pow(p):
    def InOutPow(t):
        return pow(2, p - 1) * pow(t, p) if t < 0.5 else 1 - pow(-2 * t + 2, p) / 2
    InOutPow.__name__ = f'InOut{p}'
    return InOutPow


def InSine(t):
    return 1 - cos((t * pi) / 2)


def OutSine(t):
    return sin((t * pi) / 2)


def InOutSine(t):
    return -(cos(pi * t) - 1) / 2


def InCubic(t):
    return t * t * t


def OutCubic(t):
    return 1 - pow(1 - t, 3)


def InOutCubic(t):
    return 4 * t * t * t if t < 0.5 else 1 - pow(-2 * t + 2, 3) / 2


def InQuint(t):
    return t * t * t * t * t


def OutQuint(t):
    return 1 - pow(1 - t, 5)


def InOutQuint(t):
    return 16 * t * t * t * t * t if t < 0.5 else 1 - pow(-2 * t + 2, 5) / 2


def InCirc(t):
    return 1 - sqrt(1 - pow(t, 2))


def OutCirc(t):
    return sqrt(1 - pow(t - 1, 2))


def InOutCirc(t):
    return (1 - sqrt(1 - pow(2 * t, 2))) / 2 if t < 0.5 else (sqrt(1 - pow(-2 * t + 2, 2)) + 1) / 2


def InElastic(t):
    c4 = (2 * pi) / 3
    if t == 0:
        return 0
    elif t == 1:
        return 1
    else:
        return -pow(2, 10 * t - 10) * sin((t * 10 - 10.75) * c4)


def OutElastic(t):
    c4 = (2 * pi) / 3
    if t == 0:
        return 0
    elif t == 1:
        return 1
    else:
        return pow(2, -10 * t) * sin((t * 10 - 0.75) * c4) + 1


def InOutElastic(t):
    c5 = (2 * pi) / 4.5
    if t == 0:
        return 0
    elif t == 1:
        return 1
    elif t < 0.5:
        return -(pow(2, 20 * t - 10) * sin((20 * t - 11.125) * c5)) / 2
    else:
        return (pow(2, -20 * t + 10) * sin((20 * t - 11.125) * c5)) / 2 + 1


def InQuad(t):
    return t * t


def OutQuad(t):
    return 1 - (1 - t) * (1 - t)


def InOutQuad(t):
    return 2 * t * t if t < 0.5 else 1 - pow(-2 * t + 2, 2) / 2


def InQuart(t):
    return t * t * t * t


def OutQuart(t):
    return 1 - pow(1 - t, 4)


def InOutQuart(t):
    return 8 * t * t * t * t if t < 0.5 else 1 - pow(-2 * t + 2, 4) / 2


def InExpo(t):
    return 0 if t == 0 else pow(2, 10 * t - 10)


def OutExpo(t):
    return 1 if t == 1 else 1 - pow(2, -10 * t)


def InOutExpo(t):
    if t == 0:
        return 0
    elif t == 1:
        return 1
    elif t < 0.5:
        return pow(2, 20 * t - 10) / 2
    else:
        return (2 - pow(2, -20 * t + 10)) / 2


def InBack(t):
    c1 = 1.70158
    c3 = c1 + 1
    return c3 * t * t * t - c1 * t * t


def OutBack(t):
    c1 = 1.70158
    c3 = c1 + 1
    return 1 + c3 * pow(t - 1, 3) + c1 * pow(t - 1, 2)


def InOutBack(t):
    c1 = 1.70158
    c2 = c1 * 1.525
    return (pow(2 * t, 2) * ((c2 + 1) * 2 * t - c2)) / 2 if t < 0.5 else (pow(2 * t - 2, 2) * ((c2 + 1) * (t * 2 - 2) + c2) + 2) / 2


def InBounce(t):
    return 1 - OutBounce(1 - t)


def OutBounce(t):
    n1 = 7.5625
    d1 = 2.75
    if t < (1 / d1):
        return n1 * t * t
    elif t < (2 / d1):
        return n1 * (t - 1.5 / d1)**2 + 0.75
    elif t < (2.5 / d1):
        return n1 * (t - 2.25 / d1)**2 + 0.9375
    else:
        return n1 * (t - 2.625 / d1)**2 + 0.984375


def InOutBounce(t):
    return (1 - OutBounce(1 - 2 * t)) / 2 if t < 0.5 else (1 + OutBounce(2 * t - 1)) / 2


def Drift(t, x=6, y=6):
    x /= 10
    y /= 10
    if x == 1 and y == 1:
        return t
    if x > y:
        hy = 0
        hx = (x-y)/(1-y)
    else:
        hx = 0
        hy = (y-x)/(1-x)
    if t >= x:
        return (1-t)/(1-x)*y + (t-x)/(1-x)
    else:
        ng = 0
        ok = 1
        while abs(ng-ok) > 1e-10:
            mid = ng + ok
            mid /= 2
            criteria = 3*(1-mid)*mid*mid*hx + mid*mid*mid*x
            if criteria >= t:
                ok = mid
            else:
                ng = mid
        return 3*(1-ok)*ok*ok*hy + ok*ok*ok*y


def calculate_adaptive_multiplier(easefunc, dur, dx, dy, logger):
    if easefunc is None or dur <= 0:
        return 1.0
    sim_step = 1/30 
    num_steps = ceil(dur / sim_step)
    if num_steps == 0:
        num_steps = 1
    linear_delta_rate = 1.0 / num_steps
    max_delta_rate = 0.0
    last_rate = 0.0
    for i in range(1, num_steps + 1):
        t = i / num_steps
        current_rate = 0.0
        if easefunc.__name__ == 'Drift':
            current_rate = easefunc(t, dx, dy) 
        else:
            current_rate = easefunc(t)
        delta_rate = current_rate - last_rate
        if delta_rate > max_delta_rate:
            max_delta_rate = delta_rate
        last_rate = current_rate
    res_multiplier = 1.0
    if linear_delta_rate > 0 and max_delta_rate > linear_delta_rate:
        res_multiplier = max_delta_rate / linear_delta_rate
        logger.log(f'イージング最大速度に基づき解像度を調整 (x{res_multiplier:.2f})')
    else:
        logger.log(f'イージングは低速なため基本解像度を使用。')
    return res_multiplier


def interpolate(start, end, rate):
    if rate >= 1.0:
        return end
    if rate <= 0.0:
        return start
    return start*(1-rate) + end*rate


def ease(self, dur, text : str, line):
    u_text = text.upper().strip()
    dx = 6
    dy = 6
    easefunc = None
    if u_text == 'EASE':
        self.logger.log(f'easeコマンドを検出')
        self.logger.log(f'有効なeasing関数名が指定されていないため、easeInOutCubic（CameraPlus デフォルト）を返します')
        easefunc = InOutCubic
    elif u_text.startswith('EASE'):
        easefunc, dx, dy, _ = parse_easing_func(u_text[4:], self.logger, log_prefix='easeコマンド ')
    else:
        easefunc, dx, dy, _ = parse_easing_func(u_text, self.logger, log_prefix='easeコマンド ')
    print(dx, dy)
    if easefunc is None:
        self.logger.log(f'! 有効なeaseコマンドを検出できません !')
        self.logger.log(f'EaseTransition: False としますが、意図しない演出になっています。')
        self.lines.append(line)
        self.logger.log(line.start)
        self.logger.log(line.end)
        return
    span = max(1/30, dur/36)
    spans = []
    init_dur = dur
    while dur > 0:
        min_span = min(span, dur)
        if dur - min_span < 0.01:
            min_span = dur
        spans.append(min_span)
        dur -= min_span
    span_size = len(spans)
    ixp, iyp, izp = line.start.pos.unpack()
    ixr, iyr, izr = line.start.rot.unpack()
    lxp, lyp, lzp = line.end.pos.unpack()
    lxr, lyr, lzr = line.end.rot.unpack()
    lxr = ixr + (lxr - ixr + 180) % 360 - 180
    lyr = iyr + (lyr - iyr + 180) % 360 - 180
    lzr = izr + (lzr - izr + 180) % 360 - 180
    iFOV = line.start.fov
    lFOV = line.end.fov

    self.lastTransform = line.start
    for i in range(span_size):
        new_line = Line(spans[i])
        new_line.visibleDict = deepcopy(line.visibleDict)
        if i == span_size - 1:
            t = 1.0
        else:
            t = min(1, sum(spans[:(i+1)])/init_dur)
        if easefunc != Drift:
            rate = easefunc(t)
        else:
            rate = Drift(t,dx,dy)
        
        if t >= 1.0:
            rate = 1.0
            
        new_line.start = deepcopy(self.lastTransform)
        endPos = Pos(
            interpolate(ixp, lxp, rate),
            interpolate(iyp, lyp, rate),
            interpolate(izp, lzp, rate),
        )
        endRot = Rot(
            interpolate(ixr, lxr, rate),
            interpolate(iyr, lyr, rate),
            interpolate(izr, lzr, rate),
        )
        fov = interpolate(iFOV, lFOV, rate)
        new_line.end = Transform(endPos, endRot, fov)
        self.logger.log(new_line.start)
        self.lines.append(new_line)
        self.lastTransform = new_line.end


def parse_easing_func(text : str, logger, log_prefix=''):
    if not text:
        return None, 6, 6, None
    u_text = text.upper().strip()
    dx = 6
    dy = 6
    easetype_name = None
    easefunc = None
    if u_text.startswith('DRIFT'):
        split_params = u_text.split('_')
        if len(split_params) > 2:
            dx = float(split_params[1])
            dy = float(split_params[2])
        u_text = 'DRIFT' 
    if u_text[:2] == 'IO':
        u_text = 'INOUT' + u_text[2:]
    if (u_text and u_text[0] == 'I') and (len(u_text) == 1 or u_text[1] != 'N'):
        u_text = 'IN' + u_text[1:]
    if (u_text and u_text[0] == 'O') and (len(u_text) == 1 or u_text[1] != 'U'):
        u_text = 'OUT' + u_text[1:]
    found_ease = False
    for easetype in easetypes:
        u_easetype = easetype.upper()
        if u_text.startswith(u_easetype):
            if logger:
                logger.log(f'{log_prefix}ease関数 {easetype} を検出')
            try:
                easefunc = globals()[easetype] 
            except KeyError:
                if logger:
                    logger.log(f'{log_prefix}! {easetype} の関数実体が見つかりません !')
                easefunc = None
            easetype_name = easetype
            found_ease = True
            break
    if not found_ease:
        match = re.match(r'^(INOUT|IN|OUT)(\d+\.?\d*)$', u_text)
        if match:
            prefix = match.group(1)
            param_str = match.group(2)
            try:
                p = float(param_str)
                if p < 1:
                    if logger:
                        logger.log(f'{log_prefix}! イージングの数値パラメータは 1以上が必要です: "{text}" !')
                    return None, dx, dy, None
                if prefix == "IN":
                    easefunc = make_in_pow(p)
                    easetype_name = f'In{p}'
                elif prefix == "OUT":
                    easefunc = make_out_pow(p)
                    easetype_name = f'Out{p}'
                elif prefix == "INOUT":
                    easefunc = make_in_out_pow(p)
                    easetype_name = f'InOut{p}'
                if logger:
                    logger.log(f'{log_prefix}数値指定のease関数 {easetype_name} を検出')
                found_ease = True
            except ValueError:
                pass
    if not found_ease:
        if logger and text:
            logger.log(f'{log_prefix}! 有効なease関数名 "{text}" を検出できませんでした !')
        return None, dx, dy, None
    return easefunc, dx, dy, easetype_name
