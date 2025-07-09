シミュレーションソフト「SCONE」を用いた, 人工筋による歩行支援効果を検証するためのコード

動かすためには[SCONE](https://scone.software/doku.php?id=install)が必要

ScriptControllerとして人工筋による介入を行う

介入手順として

1. 制御の基準とする側の脚 (右脚or左足) の踵接地を検出
2. 踵が接地してから'delay time'分待機
3. 'delay time'分の待機が終了後, 'contraction time'分人工筋を収縮
4. 'contraction time'中は定義した計算式によって力を計算
5. 力の計算と同時に, 人工筋の取り付け位置として指定した二点を結んだ直線と, 支援対象の関節 (今回は股関節) 間のモーメントアームを計算
6. 計算した力とモーメントアームの積によって生み出されるモーメントを計算し, 'add external moment'によって支援対象の関節に加える
