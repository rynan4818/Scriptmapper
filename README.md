# Script Mapper

このツールは、以下のように マッピングソフト 上でブックマークに命令（スクリプト）を書いていくことにより、同じタイミングでカメラが動いてくれるようなカメラスクリプトを作成します。

![fig1](https://user-images.githubusercontent.com/43929933/124432656-d4d87100-ddac-11eb-810c-150ec4d64882.png)

# 実行方法
Scriptmapper.exe を WIP フォルダ直下にコピーしたカスタムマップフォルダに入れてください。

![fig4](https://user-images.githubusercontent.com/43929933/124432683-dace5200-ddac-11eb-8a8e-b63a431fed8a.png)

本ツールは、ブックマークの編集中にカスタムマップの譜面データを変更してしまう危険性を考慮し、**CustomWIPLevels 直下でないとエラーを起こす**ように設定しています。お手数ですが、必ずマップを CustomWIPLevels にコピーしてから作業を行ってください。

ブックマークでスクリプトを記入した dat ファイルを、Scriptmapper.exe のアイコンにドラッグ＆ドロップしてください。

![fig5](https://user-images.githubusercontent.com/43929933/124432690-dc981580-ddac-11eb-82af-f44095647660.png)

同じフォルダに `SongScript.json` が出力されていれば成功です。後は、`camerascript.cfg` の設定を `songSpecificScript=True`に指定すれば、その譜面をプレイする時には、マッピングした通りにカメラが動作してくれるはずです！

また、同時に `logs` というフォルダの中に log が生成されます。生成に失敗した場合、また成功しても、意図したカメラの動きになっていない場合、log を見れば原因がわかるかもしれません。例えば、あるコマンドを検出した後に log が途切れていれば、その後のコマンドのパラメータの設定に失敗した可能性が高いです。

![fig6](https://user-images.githubusercontent.com/43929933/124432709-e02b9c80-ddac-11eb-961a-9613fc0f6da8.png)

# スクリプトの設定方法

- スクリプトは「,」（カンマ）によって startコマンドとendコマンドに区切られます。
- スクリプトの持続期間は、そのブックマーク位置から、次のブックマーク位置までです。
- カメラの位置・角度は、startコマンドの位置からendコマンドの位置まで連続的に変化します。
- endコマンドを省略した場合には自動的に`stop`が入ります。（つまり、その区間のカメラはstartコマンドの位置で静止します）

ブックマークは、マッピングソフト上で**bキーを打つ**ことで設定できます。

# プリセットコマンド　基本編
スクリプトの先頭に以下の文字が書かれていた場合、プリセットコマンドとして認識されます。以下、位置に関わるパラメータは**メートル基準**、角度に関わるパラメータは**0～360度基準**になります（ラジアンではありません）。位置や前後左右はアバター（＝プレイヤー）が立っている位置・向いている向きを基準とします。

|名前|説明|パラメータ|例|
|---|---|---|---|
|`center`|真正面からアバターを見る、または真後ろから見下ろす位置にカメラを置きます。|前方向位置|`center1`|
|`top`|アバターの真上やや前方にカメラを置きます。まっすぐ見下ろします。|高さ|`top3`|
|`side`|真横にカメラを置きます。高さは 1.5 m（ほぼ顔と同じ位置）。|右手方向位置|`side3`|
|`diagf`|斜め前にカメラを置きます。高さは 3 m（見下ろす位置）。|右手前方向位置|`diagf3`|
|`diagb`|斜め後ろにカメラを置きます。高さは 3 m（見下ろす位置）。|右手後方向位置|`diagb3`|
|`random`|ランダムな座標にカメラを置きます。|半径|`random3`|

※当ツールのパラメータは、位置・角度ともに、基本的にUnityの座標系で**正になる**ように設定しております。

※`center`のカメラ位置は、アバターより前方ならば目線の高さに、後方ならば見下ろす位置になります

# オリジナルコマンド
デフォルトコマンドで表現が難しい場合は、自分でパラメータを指定したオリジナルコマンドを作成します。オリジナルコマンドは以下のような csv ファイルを用意し「`input.csv`」と名付けます。（それ以外の名前ではプログラムが認識しません）。

![fig2](https://user-images.githubusercontent.com/43929933/124432665-d6a23480-ddac-11eb-9735-7a414c847bab.png)

ヘッダー（1行目）には以下の列名を記入してください。

|列名|機能|
|---|---|
|`label`|コマンドとして認識される名前。これと同じ名前をスクリプトに記入することで、コマンドとして機能します。|
|`px`|カメラの x 座標|
|`py`|カメラの y 座標|
|`pz`|カメラの z 座標|
|`lookat`|ここを `true` にした場合、自動的にアバターを向く角度を指定します。以降に記入する角度のパラメータは無視されます。|
|`rx`|カメラの x 角度|
|`ry`|カメラの y 角度|
|`rz`|カメラの z 角度|

---

※これより下の内容は発展的な内容になります。

# プリセットコマンド　中級編

以下のコマンドは、デフォルトコマンドですが、直前の座標に操作を加えるものです。

|名前|説明|パラメータ|例|
|---|---|---|---|
|`stop`|直前のスクリプトが動作し終わった位置でカメラを静止します。|-|`stop`|
|`mirror`|直前の座標を左右方向に鏡像反転させます。|-|`mirror`|
|`zoom`|直前の座標を縮小します（つまりアバターに近づく）。縮小の中心は(0,1.5,0)です（おおよそアバターの顔の位置）。|縮小倍率|`zoom2`|
|`spin`|カメラの z 方向角度（ロール）を変化させます。|反時計周り角度|`spin-40`|
|`slide`|カメラの左右方向の位置を移動させます。|右方向位置|`slide.5`|
|`shift`|カメラの上下方向の位置を移動させます。|上方向位置|`shift.5`|
|`push`|カメラの前後方向の位置を移動させます。|前方向位置|`push1`|
|`screw`|spinとzoomをあわせたような動きをします。|縮小倍率|`spin5/4`|

※パラメータの指定はマイナス`-40`、小数点開始`.5`、分数`5/4`でも可能です。

※`stop`の後にコマンドを続けることも可能です。例えば、`stop,shift1`では、直前のスクリプトが動作し終わった**後の位置から更に** スムーズに1 m上昇するような動きを、次のスクリプト開始まで続けます。

# プリセットコマンド　上級編

以下のコマンドは単一の座標を返すのではなく、一定の連続した動きを記述するコマンドです。

## rotate

カメラがアバターを中心に回転します。後に続く文字列（から、スラッシュで区切られた 2つの文字列）をそれぞれ半径、高さ（ともにメートル）と認識します。また、半径は高さを考慮しない水平投影面での半径を示します。

ex) `rotate4,3` （半径 4m、高さ 3m の円軌道）

## vibro
カメラがランダムに移動します。パラメータは周期となり、短い周期を指定すれば振動、長い周期を指定すれば手ブレのような動きになります。

ex) `vibro1/6`

---

※これより下の内容は更にマニアックになります

# 特殊コマンド

以下はスクリプトそのものを対象とする、発展的な特殊コマンドです。ある程度ツールの操作に慣れた上で、操作を省略するためのものです。

## copy

スクリプトをある範囲で一括コピーします。マップの展開が同じ部分で、以前書いたブックマークを使い回す時に便利です。例えば、100グリッド目で`copy40`と書いたとします。その次のブックマークが120だった場合、コピーを**貼り付ける**範囲は100～120になります。この長さは20です。そしてコピーの**元となる**範囲は40から始まり、同じ長さを持つ部分になります。つまり、40～60の部分が100～120の範囲に転写されます。この時、終点となる60や120丁度のグリッドは転写範囲に**含まれない**ことに注意してください。※

ex) `copy40`

※開始地点から「終了地点までの」範囲ではなく、「次の範囲の開始地点の直前まで」と考えます。例えば、64グリッド目から更に32グリッド分を含む範囲は95グリッドまでですが、64+32=96グリッド目の「直前まで」と考えた方が計算としてはわかりやすいです。

## fill

スクリプトを反復複製します。例えば、`random3`をある区間何回も繰り返す場合、それらをすべて書くのは面倒です。そのような場合、例えば`fill1/4,random3`と書くことで、同じ記述を反復することができます。つまり、以下の画像はプログラム上では同じものとして認識されます。

![fig3](https://user-images.githubusercontent.com/43929933/124432671-d86bf800-ddac-11eb-927c-23caf4f5bdb2.png)

fill の後に続く文字列をスパン、カンマで区切られた後の文字列をパターンとして認識します。つまり`fill1/4,random3`は次のブックマークまで、マッパー上のグリッドの 1/4 間隔で`random3`というブックマークを記入し続けます。

ex) `fill1/4,random3,zoom2`（グリッド 1/4 間隔で`random3,zoom2`を記入し続ける）

ex) `fill2,mirror`（グリッド 2 間隔で`mirror`を記入し続ける）

## copyとfillの併用

copy範囲に他のスクリプトを含めることはできません（そうなった場合、挟まれたコマンドがcopyの終点になってしまいます）。しかし、fill内にcopyを含めることはできます。

ex) `fill2,copy40` fillの終点まで、グリッド40~42の内容が反復されることになります
