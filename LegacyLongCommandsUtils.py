"""Original vibration sampling when no additional easing option is specified.

The calculation order, floating-point subdivision loop, and random draws are
kept from hibit-at/Scriptmapper 1a706b2. Do not replace the loop with ceil or
merge the remainder before sampling: both alter existing camera movements.
Output duration correction belongs to ScriptMapper.render_json. Empty
intervals retain the fork's handling without division by zero.
"""

from random import random
from copy import deepcopy
from BasicElements import Pos, Rot, Line


def vib(self, dur, text, line):
    sampling_duration = getattr(line, 'legacy_sampling_duration', dur)
    if dur <= 0 and sampling_duration <= 0:
        self.lines.append(line)
        return
    try:
        param = float(eval(text[3:]))
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
    dx, dy, dz = 0, 0, 0
    spans = []
    bpm = self.bpm
    span = max(1/30, param*60/bpm)
    output_duration = max(0, dur)
    dur = sampling_duration
    if dur <= 0:
        dur = output_duration
    while dur > 0:
        min_span = min(span, dur)
        spans.append(min_span)
        dur -= min_span
    # Only the sample count follows legacy timing. Keep corrected elapsed time
    # (including BPM changes and offset) in the generated camera script.
    spans = [output_duration/len(spans)]*len(spans)
    span_size = len(spans)
    for i in range(span_size):
        new_line = Line(spans[i])
        # new_line.visibleDict = deepcopy(self.visibleObject.state)
        new_line.visibleDict = deepcopy(line.visibleDict)
        new_line.start = deepcopy(self.lastTransform)
        if i == 0:
            new_line.start.pos = Pos(ixp, iyp, izp)
            new_line.start.rot = Rot(ixr, iyr, izr)
            new_line.start.fov = iFOV
        dx = round(random()/6, 3)-1/12
        dy = round(random()/6, 3)-1/12
        dz = round(random()/6, 3)-1/12
        px2 = ixp + (lxp-ixp)*(i+1)/span_size
        py2 = iyp + (lyp-iyp)*(i+1)/span_size
        pz2 = izp + (lzp-izp)*(i+1)/span_size
        rx2 = ixr + (lxr-ixr)*(i+1)/span_size
        ry2 = iyr + (lyr-iyr)*(i+1)/span_size
        rz2 = izr + (lzr-izr)*(i+1)/span_size
        fov2 = iFOV + (lFOV-iFOV)*(i+1)/span_size
        new_line.end.pos = Pos(px2+dx, py2+dy, pz2+dz)
        new_line.end.rot = Rot(rx2, ry2, rz2)
        new_line.end.fov = fov2
        if i == span_size-1:
            new_line.end.pos = Pos(lxp, lyp, lzp)
            new_line.end.rot = Rot(lxr, lyr, lzr)
            new_line.end.fov = lFOV
        self.lastTransform = new_line.end
        self.logger.log(new_line.start)
        self.lines.append(new_line)
