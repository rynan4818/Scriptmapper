import sys
import time
import json
from ScriptMapperClass import ScriptMapper


def mapping(file_path):
    t_start = time.perf_counter()
    mapper = ScriptMapper()
    mapper.set_file_path(file_path)

    mapper.logger.log(f"[Time] Init: {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # WIP の下にあるか確認
    mapper.confirm_WIP()

    # BPM計測
    mapper.check_bpm()

    # オリジナルコマンドを追加
    mapper.make_manual_commands()

    mapper.logger.log(f"[Time] Pre-process: {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # bookmarkの抽出（raw_b）
    mapper.make_raw_b()

    mapper.logger.log(f"[Time] make_raw_b (JSON Load): {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # fillの処理（filled_b）
    mapper.make_filled_b()

    # copyの処理（copied_b）
    mapper.make_copied_b()

    # durationを計算
    mapper.calc_duration()

    # 最終的なブックマーク
    mapper.show_bookmarks()

    mapper.logger.log(f"[Time] Process Bookmarks: {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # ブックマークをパース
    mapper.parse_bookmarks()

    mapper.logger.log(f"[Time] parse_bookmarks: {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # nextの計算
    mapper.next_calc()

    # easeの処理
    mapper.ease_calc()

    # rotの処理
    mapper.rot_calc()

    # vibの処理
    mapper.vib_calc()

    # rvibの処理
    mapper.rvib_calc()

    mapper.logger.log(f"[Time] Calc transforms: {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # レンダリング
    mapper.render_json()

    mapper.logger.log(f"[Time] render_json (Dict creation): {(time.perf_counter() - t_start) * 1000:.0f} ms")
    t_start = time.perf_counter()

    # CPythonとIronPythonの比較用：文字列化の時間を計測
    _ = json.dumps(mapper.output, ensure_ascii=False)
    mapper.logger.log(f"[Time] json.dumps (String creation): {(time.perf_counter() - t_start) * 1000:.0f} ms")

    # ファイルの書き出し
    mapper.create_file()


if __name__ == "__main__":
    # ファイルパスの取得
    if len(sys.argv) == 1:
        print('Script Mapper は単独では開けません。譜面の dat ファイルをドラッグ＆ドロップしてください。')
        input()
        exit()
    file_path = sys.argv[1]
    mapping(file_path)
