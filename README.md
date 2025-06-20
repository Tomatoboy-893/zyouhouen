本システムのゴール<br>
使用機材<br>
BME280搭載　温湿度・気圧センサモジュールhttps://www.switch-science.com/products/2323/<br>
raspberry-piと温湿度、気圧センサを組み合わせての統計データ取得からグラフ化。<br>
raspberry側（例外もある）で規制が入っている場合があるので、その場合は、raspberry-piの中のPythonで仮想環境を立ててからの実行で解決。<br>
・時間計測←一分間計測&5秒間ごとのデータ取得は導入済み<br>
↓<br>
それを応用すれば24時間など長時間のデータ取得は可能??<br>
↓<br>
バックグラウンドで実行したら、ssh接続無しで可能<br>
データ取得と、グラフ化は別々にしたほうがいい。<br>
・データ取得　data_logger.pyで可能←ラズパイ側<br>
・グラフ化 plot_bme_data.pyで可能←自身のPCでの実行<br>
・matplotは、ラズパイ側での規制が入ってるので、使えないので、一括してグラフ化までは行えない。<br>
<br>
新し取り組み<br>
・app.pyでデータを取得した後、index.htmlでその取得したデータをグラフ化、リアルタイムでの表示を実現（したい）<br>
↓<br>
pip install Flask #webフレームを作成するライブラリーのdownloadをしないと、web上で見ることができない<br>
app.pyと、index.htmlでのシステムでは、ラズパイと同じwifiに接続しないとみることができない。<br>
解決方法<p>
・wifiの設定変更<p>
・pvn接続
