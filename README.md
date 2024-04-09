# rovi_industrial

本パッケージは、ロボットコントローラからのリクエスト処理、およびロボットコントローラへのレスポンス処理を行う。  
ROSドライバが存在するロボットでも、周辺機器を操作するためのインタフェースはSocketを用いることが多い。将来的にはROSレベルでの完全なインタフェースが標準化されるかもしれないが、当面はSocketを使ったインタフェースが汎用性が高い。  
本パッケージもSocket Serverとして設計されており、ロボット側をClientとすることを前提としている。

## Parameter
- Configuration(/config/rsocket)  
以下パラメータによる

|name|type|description|
|:----|:----|:----|
|protocol|{fanuc,melfa...}|通信プロトコル選択|
|port|Int|ソケットサーバのポート番号|
|source_frame_id|String|マスターデータのフレームID|
|target_frame_id|String|シーン中のマスターに一致するフレームID|
|base_frame_id|String|source->target変換の基準座標系。X2の返答として返す座標(OK\n(x,y,z,a,b,c))は、source->target変換をこの座標系に変換している。|
|update_frame_id|String|本frame_idが指定されていた時は、X1にて座標を受け取った場合、その値にてframeを更新する。座標のストリーミングドライバがないロボット用の救済機能|

## Topic
### To subscribe
  
|name|type|description|
|:----|:----|:----|
|response/cleare|Bool|リセット要求に対する応答|
|response/capture|Bool|キャプチャ要求に対する応答|
|response/solve|Bool|ソルブ要求に対する応答|
|response/recipe_load|Bool|レシピ選択要求に対する応答|
|response/function_load|Bool|ファンクション選択要求に対する応答|

### To publish

|name|type|description|
|:----|:----|:----|
|request/clear|Bool|リセット要求|
|request/capture|Bool|キャプチャ要求|
|request/solve|Bool|ソルブ要求|
|request/recipe_load|String|レシピ選択要求|
|request/function_load|String|ファンクション選択要求|

## TF

|name|description|
|:----|:----|
|waypoint[0..n]|socketがX2(...)などにて経由点情報を受信した場合、その点数に応じた"waypoint0..n"という名のframeをpublishする。基準フレームはパラメータwaypoint_frame_idで与えられたものとする|

## Internal Protocol

|format|description|
|:----|:----|
|X0()|リセット要求。()内のパラメータは無視される|
|X1(x,y,z,r,p,w)|キャプチャ要求。()内のパラメータが座標形式であれば、update_frame_idで指定したframeを更新する|
|X2([x,y,z,r,p,w;...;x,y,z,r,p,w])|ソルブ要求。()内のパラメータが座標シーケンス形式であれば、その点数に応じた"waypoint0..n"という名のframeをpublishする(TFの項を参照）。|
|X3(string)|レシピ変更要求。()内のパラメータが文字列であれば、その文字を/request/recipe_loadトピックに発行する|
