from copy import deepcopy
from math import atan2, cos, degrees, pi, sin, sqrt
from random import random as rd

from utils import create_template


def random(r):
    theta = rd()*2*pi
    phi = rd()/4*pi
    angle = atan2(r*sin(phi)-1.5, r)
    pos = {'x': round(r*cos(phi)*cos(theta), 1),
           'y': round(r*sin(phi), 1),
           'z': round(r*cos(phi)*sin(theta), 1)}
    spin = rd()*20-10
    rot = {'x': int(degrees(angle)),
           'y': -int(degrees(theta))+270,
           'z': spin}
    return pos, rot


def center(r):
    if r >= 0:
        pos = {'x': 0,
               'y': 1.5,
               'z': r}
        rot = {'x': 0,
               'y': 180,
               'z': 0}
    if r < 0:
        pos = {'x': 0,
               'y': 1.5+abs(r),
               'z': r}
        rot = {'x': 45,
               'y': 0,
               'z': 0}
    return pos, rot


def side(r):
    pos = {'x': r,
           'y': 1.5,
           'z': 0}
    if r >= 0:
        rot = {'x': 0,
               'y': -90,
               'z': 0}
    else:
        rot = {'x': 0,
               'y': 90,
               'z': 0}
    return pos, rot


def top(h):
    pos = {'x': 0,
           'y': h,
           'z': h/10}
    rot = {'x': 90,
           'y': 0,
           'z': 0}
    return pos, rot


def diagb(r):
    pos = {'x': r/1.4,
           'y': 3.0,
           'z': -abs(r)/1.4}
    angle = degrees(atan2(1.5, abs(r)))
    if r >= 0:
        rot = {'x': angle,
               'y': -45,
               'z': 0}
    if r < 0:
        rot = {'x': angle,
               'y': 45,
               'z': 0}
    return pos, rot


def diagf(r):
    pos = {'x': r/1.4,
           'y': 3.0,
           'z': abs(r)/1.4}
    angle = degrees(atan2(1.5, abs(r)))
    if r >= 0:
        rot = {'x': angle,
               'y': -135,
               'z': 0}
    if r < 0:
        rot = {'x': angle,
               'y': 135,
               'z': 0}
    return pos, rot


def mirror(last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    pos['x'] *= -1
    rot['y'] *= -1
    return pos, rot


def zoom(scale, last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    pos['x'] /= scale
    pos['y'] -= 1.5
    pos['y'] /= scale
    pos['y'] += 1.5
    pos['z'] /= scale
    return pos, rot


def spin(r, last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    rot['z'] += r
    return pos, rot


def screw(scale, last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    angle = 20*(1-scale)
    pos['x'] /= scale
    pos['y'] -= 1.5
    pos['y'] /= scale
    pos['y'] += 1.5
    pos['z'] /= scale
    rot['z'] += angle
    return pos, rot


def slide(r, last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    pos['x'] += r
    return pos, rot


def shift(r, last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    pos['y'] += r
    return pos, rot


def push(r, last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    pos['z'] += r
    return pos, rot


def stop(last_pos_rot):
    pos, rot = deepcopy(last_pos_rot)
    return pos, rot


def original(command):
    cx = float(command['px'])
    cy = float(command['py'])
    cz = float(command['pz'])
    pos = {'x': cx, 'y': cy, 'z': cz}
    if command['lookat'].lower() == 'true':
        theta = atan2(cz, cx)
        theta = -int(degrees(theta))+270
        r = sqrt(cx**2+cz**2)
        angle = int(degrees(atan2(cy-1.5, r)))
        rot = {'x': angle, 'y': theta, 'z': 0}
    else:
        rot = {'x': float(command['rx']), 'y': float(
            command['ry']), 'z': float(command['rz'])}
    return pos, rot


def rotate(dur, text):
    log_text = ''
    # def_value
    r = 3
    h = 3
    a = 1
    o = 1.0
    s = 0
    if len(text) > 6:
        param = [eval(i) for i in text[6:].split(',')]
        if len(param) > 0:
            r = param[0]
        if len(param) > 1:
            h = param[1]
        if len(param) > 2:
            a = param[2]
        if len(param) > 3:
            o = param[3]
        if len(param) > 4:
            s = param[4]
    log_text += f'パラメータ r:{r} h:{h} a:{a} o:{o} s:{s}\n'
    span = max(1/30, dur/36)
    spans = []
    while dur > 0.001:
        min_span = min(span, dur)
        spans.append(min_span)
        dur -= min_span
    span_size = len(spans)
    ans = []
    for i in range(span_size):
        new_line = create_template()
        theta = 2*pi*i/span_size - 1/2*pi
        next_theta = 2*pi*(i+1)/span_size - 1/2*pi
        angle = atan2(h-a, r)
        px = round(r*cos(theta), 3)
        pz = round(r*sin(theta)+o, 3)
        rx = degrees(angle)
        ry = -degrees(theta)+270
        new_line['StartPos'] = {'x': px, 'y': h, 'z': pz}
        new_line['StartRot'] = {'x': rx, 'y': ry, 'z': s}
        px = round(r*cos(next_theta), 3)
        pz = round(r*sin(next_theta)+o, 3)
        rx = degrees(angle)
        ry = -degrees(next_theta)+270
        new_line['EndPos'] = {'x': px, 'y': h, 'z': pz}
        new_line['EndRot'] = {'x': rx, 'y': ry, 'z': s}
        new_line['Duration'] = spans[i]
        log_text += f'start POS{new_line["StartPos"]} ROT{new_line["StartRot"]}\n'
        log_text += f'end POS{new_line["EndPos"]} ROT{new_line["EndRot"]}\n'
        ans.append(new_line)
    return ans, log_text


def vibro(dur, bpm, param, last_pos_rot):
    log_text = ''
    steps = []
    pos, rot = deepcopy(last_pos_rot)
    span = max(1/30, param*60/bpm)
    while dur > 0:
        steps.append(min(span, dur))
        dur -= span
        span *= (0.9 + rd()*0.2)
    ans = []
    for s in steps:
        new_line = create_template()
        new_line['StartPos'] = pos
        new_line['StartRot'] = rot
        e_pos, e_rot = deepcopy((pos, rot))
        dx = round(rd()/6, 3)-1/12
        dy = round(rd()/6, 3)-1/12
        dz = round(rd()/6, 3)-1/12
        e_pos['x'] += dx
        e_pos['y'] += dy
        e_pos['z'] += dz
        new_line['EndPos'] = e_pos
        new_line['EndRot'] = e_rot
        new_line['Duration'] = s
        ans.append(new_line)
        pos, rot = (e_pos, e_rot)
        log_text += f'start POS{new_line["StartPos"]} ROT{new_line["StartRot"]}\n'
        log_text += f'end POS{new_line["EndPos"]} ROT{new_line["EndRot"]}\n'
    return ans, log_text
