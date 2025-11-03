from copy import deepcopy
import os
import csv
import json
import pathlib
import shutil
import datetime
from random import seed
from math import ceil

from BasicElements import Bookmark, Line, Transform, Logger, VisibleObject, Pos, Rot
from GeneralUtils import format_time, manual_process
from BookmarkUtils import copy_process, fill_process, raw_process
from CommandUtils import long_command, parse_command
from EnvCommandUtils import env_command
from EaseUtils import ease
from LongCommandsUtils import rot, vib, rvib


class ScriptMapper:
    def __init__(self):
        # system
        self.file_path = None
        self.path_obj = None
        self.logger = None
        self.manual = {}
        # env
        self.bpm = 0
        self.bpmchanges = []
        self.fov = 60
        self.seed = seed
        self.seed(0)
        self.height = 1.5
        self.turnToHead = False
        self.turnToHeadHorizontal = False
        self.visibleObject = VisibleObject()
        self.offset = 0
        # bookmarks
        self.dummyend_grid = 0
        self.raw_b = []
        self.filled_b = []
        self.copied_b = []
        # lines
        self.lines = []
        self.lastTransform = Transform()
        self.lastLine = None
        # output
        self.output = None

    def set_file_path(self, file_path):
        self.file_path = file_path
        self.path_obj = pathlib.Path(file_path)
        path_dir = self.path_obj.parent
        logger = Logger(path_dir)
        self.logger = logger

    def confirm_WIP(self):
        isWIP = self.path_obj.parent.parent
        if isWIP.name != 'CustomWIPLevels':
            self.logger.log('WIPフォルダの下にありません。プログラムを終了します。')
            input()
            exit()
        self.logger.log('WIPフォルダの下にあることを確認\n')

    def check_bpm(self):
        path_dir = self.path_obj.parent
        info_path = os.path.join(path_dir, 'info.dat')
        if not os.path.exists(info_path):
            self.logger.log('info.dat が見つかりません。プログラムを終了します。')
            input()
            exit()
        
        f_info = open(info_path, 'rb')
        j_info = json.load(f_info)
        
        bpm = 0
        if '_beatsPerMinute' in j_info:
            bpm = j_info['_beatsPerMinute'] # V2
            self.logger.log('V2形式のデフォルトBPMを検出')
        elif 'audio' in j_info and 'bpm' in j_info['audio']:
            bpm = j_info['audio']['bpm'] # V4
            self.logger.log('V4形式のデフォルトBPMを検出')
        else:
            self.logger.log('info.dat にBPM情報が見つかりません。プログラムを終了します。')
            f_info.close()
            input()
            exit()

        self.bpm = bpm
        self.logger.log(f'デフォルトbpmを計測 {self.bpm} \n')

        f_beatmap = open(self.file_path, 'rb')
        j = json.load(f_beatmap)

        if '_events' in j:
            bpmChanges = j['_events'] # V2.5
            for b in bpmChanges:
                if b['_type'] == 100:
                    if self.bpmchanges == []:
                        self.logger.log('V2.5のBPM Eventsを検出')
                    self.logger.log(f'grid : {b["_time"]:6.2f} (BPM : {b["_floatValue"]})')
                    self.bpmchanges.append({
                        'grid': b['_time'],
                        'bpm': b['_floatValue']})
        if self.bpmchanges != []:
            self.logger.log('')
            f_info.close()
            f_beatmap.close()
            return
            
        if '_customData' in j:
            if '_BPMChanges' not in j['_customData']:
                pass
            else:
                self.logger.log('CustomDataのBPMChangesを検出')
                bpmChanges = j['_customData']['_BPMChanges']
                for b in bpmChanges:
                    self.logger.log(f'time : {b["_time"] * 60 / self.bpm:6.2f} (BPM : {b["_BPM"]})')
                    self.bpmchanges.append({
                        'time': b['_time'] * 60 / self.bpm,
                        'bpm': b['_BPM'],
                        'perbar': b['_beatsPerBar']})
                self.logger.log('')
                f_info.close()
                f_beatmap.close()
                return

        if 'bpmEvents' in j:
            self.logger.log('V3.0のBPM Eventsを検出')
            bpmChanges = j['bpmEvents'] # V3
            for b in bpmChanges:
                self.logger.log(f'grid : {b["b"]:6.2f} (BPM : {b["m"]})')
                self.bpmchanges.append({
                    'grid': b['b'],
                    'bpm': b['m']})
            self.logger.log('')
            f_info.close()
            f_beatmap.close()
            return

        self.logger.log('V2/V3系BPM変更が見つかりません。V4 (AudioData.dat) をチェックします。\n')

        audio_data_filename = None
        if 'audio' in j_info and 'audioDataFilename' in j_info['audio']:
            audio_data_filename = j_info['audio']['audioDataFilename']
        
        audio_data_path = None
        if audio_data_filename:
            audio_data_path = os.path.join(path_dir, audio_data_filename)

        if audio_data_path and os.path.exists(audio_data_path):
            self.logger.log(f'バージョン検出：V4 (AudioData.dat を検出)\n')
            try:
                f_audio = open(audio_data_path, 'r', encoding='utf-8')
                j_audio = json.load(f_audio)
                
                song_freq = j_audio['songFrequency']
                bpm_data = j_audio['bpmData']
                
                self.logger.log(f'AudioData.dat からBPM Eventsを読み込みます (Sampling Frequency: {song_freq} Hz)')

                if bpm_data and bpm_data[0]['sb'] > 0:
                    self.logger.log(f'grid : {0.0:6.2f} (BPM : {self.bpm}) - (デフォルト)')
                    self.bpmchanges.append({
                        'grid': 0.0,
                        'bpm': self.bpm
                    })

                for b in bpm_data:
                    si = b['si']
                    ei = b['ei']
                    sb = b['sb']
                    eb = b['eb']
                    
                    segment_beats = eb - sb
                    segment_samples = ei - si
                    
                    if segment_samples == 0 or segment_beats == 0:
                        self.logger.log(f'grid : {sb:6.2f} (BPM : N/A) - Duration 0')
                        continue

                    segment_seconds = segment_samples / song_freq
                    segment_bpm = (segment_beats * 60) / segment_seconds
                    
                    self.logger.log(f'grid : {sb:6.2f} (BPM : {segment_bpm:6.2f})')
                    self.bpmchanges.append({
                        'grid': sb,
                        'bpm': segment_bpm
                    })
                
                f_audio.close()
                self.logger.log('')

            except Exception as e:
                self.logger.log(f'AudioData.dat の読み込みまたは処理に失敗しました: {e}')
                input()
                exit()
        else:
            self.logger.log('BPM変更は見つかりませんでした。\n')

        f_info.close()
        f_beatmap.close()

    def make_manual_commands(self):
        path_dir = self.path_obj.parent
        input_path = os.path.join(path_dir, 'input.csv')
        if os.path.exists(input_path):
            self.logger.log('input.csv を確認しました。オリジナルコマンドを追加します。')
            data = csv.DictReader(open(input_path, 'r', encoding='utf-8-sig'))
            manual_process(self, data)
        else:
            self.logger.log('input.csv が見つからないため、オリジナルコマンドは追加されません。\n')

    def make_raw_b(self):
        dummyend_grid = 100
        f = open(self.path_obj, 'r')
        j = json.load(f)
        notes = None
        
        if '_notes' in j:
            self.logger.log('バージョン検出：V2 譜面\n')
        else:
            self.logger.log('バージョン検出：V3 譜面\n')
        
        if '_notes' in j:
            notes = j['_notes']
        elif 'colorNotes' in j:
            notes = j['colorNotes']  # V3
        if len(notes) > 0:
            if '_time' in notes[-1]:
                dummyend_grid = notes[-1]['_time']+100
            elif 'b' in notes[-1]:
                dummyend_grid = notes[-1]['b'] + 100  # V3
        
        bookmarks = []
        
        if '_customData' in j.keys() and '_bookmarks' in j['_customData']:
            self.logger.log('V2 (CustomData) ブックマークを検出。')
            bookmarks = j['_customData']['_bookmarks']
        elif 'customData' in j.keys() and 'bookmarks' in j['customData']:
            self.logger.log('V3 ブックマークを検出。')
            bookmarks = j['customData']['bookmarks']  # V3
        elif '_bookmarks' in j.keys():
            self.logger.log('V2 (Legacy) ブックマークを検出。')
            bookmarks = j['_bookmarks']
        
        if not bookmarks:
            self.logger.log(f'V2/V3 ブックマークが見つからないか空です。V4形式のブックマークを探します。\n')
            path_dir = self.path_obj.parent
            current_filename = self.path_obj.name
            info_path = path_dir / 'info.dat'
            v4_name_segment = None

            try:
                f_info = open(info_path, 'r', encoding='utf-8')
                j_info = json.load(f_info)
                if 'difficultyBeatmaps' in j_info:
                    for beatmap in j_info['difficultyBeatmaps']:
                        if beatmap.get('beatmapDataFilename') == current_filename:
                            characteristic = beatmap.get('characteristic')
                            difficulty = beatmap.get('difficulty')
                            if characteristic and difficulty:
                                v4_name_segment = f"{characteristic}{difficulty}"
                                self.logger.log(f'info.dat から V4 ブックマーク名セグメントを生成: {v4_name_segment}')
                                break
                f_info.close()
            except Exception as e:
                self.logger.log(f'info.dat の読み込み中にエラーが発生しました: {e}')
            
            if v4_name_segment:
                v4_bookmark_path = path_dir / 'Bookmarks' / f'ChroMapper.{v4_name_segment}.bookmarks.dat'
                
                if os.path.exists(v4_bookmark_path):
                    self.logger.log(f'バージョン検出：V4 (ChroMapper) ブックマーク\n')
                    self.logger.log(f'V4ブックマークファイルを読み込みます: {v4_bookmark_path}')
                    try:
                        f_v4 = open(v4_bookmark_path, 'r', encoding='utf-8')
                        j_v4 = json.load(f_v4)
                        bookmarks = j_v4['bookmarks']
                        f_v4.close()
                    except Exception as e:
                        self.logger.log(f'V4ブックマークファイルの読み込みに失敗しました: {e}')
                        self.logger.log('プログラムを終了します。')
                        input()
                        exit()
                else:
                    self.logger.log(f'V4ブックマークファイルが見つかりませんでした: {v4_bookmark_path}')
            else:
                self.logger.log('info.dat 内に現ファイルに一致する V4 ビートマップ定義が見つかりませんでした。')
        else:
             self.logger.log(f'V2/V3 ブックマークを {len(bookmarks)} 件検出しました。')

        if len(bookmarks) == 0:
            self.logger.log('この譜面にはブックマークが含まれていません。プログラムを終了します。')
            exit()
        else:
            if '_time' in bookmarks[-1]:
                dummyend_grid = max(
                    dummyend_grid, bookmarks[-1]['_time'] + 100)
            elif 'b' in bookmarks[-1]:
                dummyend_grid = max(
                    dummyend_grid, bookmarks[-1]['b'] + 100)  # V3
            elif 'beat' in bookmarks[-1]:
                dummyend_grid = max(
                    dummyend_grid, bookmarks[-1]['beat'] + 100) # V4

        bookmarks.append({'_time': dummyend_grid, '_name': 'dummyend'})
        self.dummyend_grid = dummyend_grid
        self.logger.log(f'ダミーエンドをグリッド {dummyend_grid} に設定。')
        for b in bookmarks:
            raw_process(self, b)

    def make_filled_b(self):
        size = len(self.raw_b)
        for i in range(size-1):
            fill_process(self, i)
        self.filled_b.append(Bookmark(self.dummyend_grid, 'stop'))

    def make_copied_b(self):
        size = len(self.filled_b)
        for i in range(size-1):
            copy_process(self, i)
        self.copied_b.append(Bookmark(self.dummyend_grid, 'stop'))

    def calc_duration(self):
        command_b = []
        for b in self.copied_b:
            if b.text[0] == '#':
                continue
            command_b.append(b)
        size = len(command_b)
        for i in range(size-1):
            start = command_b[i].grid
            end = command_b[i+1].grid
            current_bpm = self.bpm
            bpmchange_grid = 0
            virtual_time = 0
            start_time = 0
            end_time = 0
            start_check = True
            v1_format = False
            for change in self.bpmchanges:
                if 'grid' not in change:
                    dur_grid = end - start
                    command_b[i].duration = dur_grid*60/self.bpm
                    v1_format = True
                    break
                if start_check and change['grid'] >= start:
                    start_time = virtual_time + (start - bpmchange_grid) * 60 / current_bpm
                    start_check = False
                if change['grid'] >= end:
                    break
                virtual_time += (change['grid'] - bpmchange_grid) * 60 / current_bpm
                current_bpm = change['bpm']
                bpmchange_grid = change['grid']
            if start_check:
                start_time = virtual_time + (start - bpmchange_grid) * 60 / current_bpm
            if not v1_format:
                end_time = virtual_time + (end - bpmchange_grid) * 60 / current_bpm
                command_b[i].duration = end_time - start_time

    def show_bookmarks(self) -> None:
        self.logger.log('\nfill,copyの処理を完了しました。最終的なブックマークは以下になります。')
        self.logger.log('          grid (   time  ) : script')
        cnt = 1
        sum_time = 0
        for b in self.copied_b:
            grid = b.grid
            virtual_time = grid * 60 / self.bpm
            virtual_grid = 0
            span_end = 0
            current_bpm = self.bpm
            for change in self.bpmchanges:
                if 'time' not in change:  # V2.5以降
                    break
                span = change['time'] - span_end
                span_end = change['time']
                if span > virtual_time:
                    break
                interval = 60 / current_bpm
                virtual_grid += ceil((span / interval) - 0.001)
                virtual_time -= span
                current_bpm = change['bpm']
            interval = 60 / current_bpm
            virtual_grid += virtual_time / 60 * current_bpm
            if b.text[0] == "#":
                self.logger.log(f'    env',
                                f'{virtual_grid:6.2f}             : {b.text}')
            else:
                self.logger.log(
                    f'{str(cnt).rjust(3)}番目',
                    f'{virtual_grid:6.2f} ({format_time(sum_time)}) : {b.text}')
                cnt += 1
            sum_time += b.duration

    def parse_bookmarks(self):
        cnt = 0
        sum_time = 0
        for b in self.copied_b:
            if b.text[0] == '#':
                self.logger.log('\n環境コマンドを検出')
                env_command(self, b.text[1:])
                continue
            cnt += 1
            text = b.text
            dur = b.duration
            self.logger.log(f'\n{cnt}番目のスクリプトを確認中...')
            self.logger.log(text)
            self.logger.log(f'grid : {b.grid:6.2f} ({format_time(sum_time)})')
            self.logger.log(f'duration : {dur:.2f}')
            sum_time += dur
            # if long command -> process -> skip
            if long_command(self, text, dur):
                continue
            # normal command
            parse = text.split(',')
            if len(parse) == 1:
                parse.append('stop')
                parse.append('False')
            elif len(parse) == 2:
                parse.append('False')
            # new_line = Line(dur, self.visibleObject.state)
            new_line = Line(dur)
            if self.offset > 0:
                new_line.duration = max(0, new_line.duration - self.offset)
                self.logger.log(
                    f'offset コマンドにより、この箇所は {new_line.duration} 秒に短縮されます。')
                self.offset = 0
            new_line.visibleDict = deepcopy(self.visibleObject.state)
            new_line.turnToHead = self.turnToHead
            new_line.turnToHeadHorizontal = self.turnToHeadHorizontal
            # start
            start_command = parse[0]
            self.logger.log(f'start : {start_command}')
            parse_command(self, new_line.start, start_command)
            self.lastTransform = new_line.start
            # end
            end_command = parse[1]
            if end_command == 'next':
                new_line.isNext = True
                new_line.end = Transform(Pos(0, 0, 0), Rot(0, 0, 0), 0)
                self.logger.log('next コマンドを検出。次のスクリプトの開始位置に合わせます。')
                self.logger.log('全スクリプトを変換後再計算するため、下のログには仮パラメータが出力されます。')
            else:
                self.logger.log(f'end : {end_command}')
                parse_command(self, new_line.end, end_command)
                self.lastTransform = new_line.end
            # transition
            transition_command = parse[2]
            self.logger.log(f'transition : {transition_command}')
            if transition_command != 'False':
                if transition_command[:3].lower() == 'rot':
                    new_line.rot = ','.join(parse[2:])
                    self.logger.log(
                        f'rot に文字列を確認しましたが、回転の処理は、next の後に行う必要があるため、後で再計算します。')
                elif transition_command[:3].lower() == 'vib':
                    new_line.vib = ','.join(parse[2:])
                    self.logger.log(
                        f'vib に文字列を確認しましたが、vibroの処理は、next の後に行う必要があるため、後で再計算します。')
                elif transition_command[:4].lower() == 'rvib':
                    new_line.rvib = ','.join(parse[2:])
                    self.logger.log(
                        f'rvib に文字列を確認しましたが、rvibの処理は、next の後に行う必要があるため、後で再計算します。')
                else:
                    # ease(self, dur, ease_command, new_line)
                    new_line.ease = transition_command
                    self.logger.log(
                        f'（工事中）easeTransition に文字列を確認しましたが、イージングの処理は、next の後に行う必要があるため、後で再計算します。')
            self.lines.append(new_line)
            self.logger.log(f'start {new_line.start}')
            self.logger.log(f'end {new_line.end}')

    def next_calc(self):
        lines = self.lines
        size = len(lines)
        for i in range(size-1):
            line = lines[i]
            if line.isNext:
                next_line = lines[i+1]
                line.end = next_line.start

    def ease_calc(self):
        self.logger.log('\nイージングの処理が臨時的にここにログに出されます。後で直します。')
        original = deepcopy(self.lines)
        self.lines = []
        for org in original:
            if org.ease != '':
                ease(self, org.duration, org.ease, org)
            else:
                self.lines.append(org)

    def rot_calc(self):
        self.logger.log('\nrotの処理が臨時的にここにログに出されます。')
        original = deepcopy(self.lines)
        self.lines = []
        for org in original:
            if org.rot != '':
                rot(self, org.duration, org.rot, org)
            else:
                self.lines.append(org)

    def vib_calc(self):
        self.logger.log('\nvibの処理が臨時的にここにログに出されます。')
        original = deepcopy(self.lines)
        self.lines = []
        for org in original:
            if org.vib != '':
                vib(self, org.duration, org.vib, org)
            else:
                self.lines.append(org)

    def rvib_calc(self):
        self.logger.log('\nrvibの処理が臨時的にここにログに出されます。')
        original = deepcopy(self.lines)
        self.lines = []
        for org in original:
            if hasattr(org, 'rvib') and org.rvib != '':
                rvib(self, org.duration, org.rvib, org)
            else:
                self.lines.append(org)

    def render_json(self):
        template = {}
        template['ActiveInPauseMenu'] = True
        template['TurnToHeadUseCameraSetting'] = False
        template['Movements'] = []
        duration_error = 0
        total_duration_error = 0
        total_duration_error_count = 0
        rounding_error = 0.0
        lines_count = len(self.lines)
        for i in range(lines_count):
            line = self.lines[i]
            if i < lines_count - 1 and line.duration < 0.01:
                duration_error += line.duration
                total_duration_error += line.duration
                total_duration_error_count += 1
                continue
            line.duration += duration_error
            duration_error = 0
            if line.duration < 0.01:
                line.duration = 0.01
            movement = {}
            movement['StartPos'] = {'x': line.start.pos.x,
                                    'y': line.start.pos.y,
                                    'z': line.start.pos.z,
                                    'FOV': line.start.fov}
            movement['StartRot'] = {'x': line.start.rot.x,
                                    'y': line.start.rot.y,
                                    'z': line.start.rot.z}
            movement['StartHeadOffset'] = {'x': line.startHeadOffset.x,
                                           'y': line.startHeadOffset.y,
                                           'z': line.startHeadOffset.z}
            movement['EndPos'] = {'x': line.end.pos.x,
                                  'y': line.end.pos.y,
                                  'z': line.end.pos.z,
                                  'FOV': line.end.fov}
            movement['EndRot'] = {'x': line.end.rot.x,
                                  'y': line.end.rot.y,
                                  'z': line.end.rot.z}
            movement['EndHeadOffset'] = {'x': line.endHeadOffset.x,
                                         'y': line.endHeadOffset.y,
                                         'z': line.endHeadOffset.z}
            movement['TurnToHead'] = line.turnToHead
            movement['TurnToHeadHorizontal'] = line.turnToHeadHorizontal

            effective_duration = line.duration + rounding_error
            rounded_duration = round(effective_duration, 3)
            rounding_error = effective_duration - rounded_duration
            movement['Duration'] = rounded_duration

            movement['Delay'] = 0
            movement['EaseTransition'] = False
            movement['VisibleObject'] = {}
            for key, value in line.visibleDict.items():
                movement['VisibleObject'][key] = value
            template['Movements'].append(movement)
        self.output = template
        if total_duration_error_count > 0:
            self.logger.log(f'\ndurationが0.01秒未満のmovementが{total_duration_error_count} 回ありました。合計の補正時間は{total_duration_error:.20f} 秒です')
        self.logger.log('\nソフト内部でのjsonデータの作成に成功しました。\n')

    def create_file(self):
        custom_map = self.path_obj.parent.name
        not_wip_folder = os.path.join(
            self.path_obj.parents[2], 'CustomLevels', custom_map)
        if os.path.exists(not_wip_folder):
            self.logger.log('カスタムマップに同名のフォルダを確認。こちらにもSongScript.jsonを作成します。\n')
            not_wip_target = os.path.join(not_wip_folder, 'SongScript.json')
            json.dump(self.output, open(not_wip_target, 'w'), indent=4)
            self.logger.log(not_wip_target)
        path_dir = self.path_obj.parent
        target_path = os.path.join(path_dir, 'SongScript.json')
        self.logger.log(target_path)
        json.dump(self.output, open(target_path, 'w'), indent=4)
        self.logger.log('\nファイルの書き出しを正常に完了しました。')
        # create log history
        log_path = os.path.join(path_dir, 'log_latest.txt')
        now = str(datetime.datetime.now()).replace(':', '_')[:19]
        log_folder_path = os.path.join(path_dir, 'logs')
        if not os.path.exists(log_folder_path):
            os.mkdir(log_folder_path)
        copy_path = os.path.join(log_folder_path, f'log_{now}.txt')
        shutil.copyfile(log_path, copy_path)
