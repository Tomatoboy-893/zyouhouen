本システムのゴール<br>
使用機材<br>
BME280搭載　温湿度・気圧センサモジュールhttps://www.switch-science.com/products/2323/<br>
raspberry-piと温湿度、気圧センサを組み合わせての統計データ取得からグラフ化。<br>
raspberry側（学校の北を使う場合等)で規制が入っている場合があるので、その場合は、raspberry-piの中のPythonで仮想空間を立ててからの実行で解決。<br>
・時間計測←一分間計測&5秒間ごとのデータ取得は導入済み<br>
↓<br>
それを応用すれば24時間など長時間のデータ取得は可能??<br>
↓<br>
バックグラウンドで実行したら、ssh接続無しで可能<br>
データ取得と、グラフ化は別々にしたほうがいい。<br>
・データ取得　data_logger.pyで可能←ラズパイ側<br>
・グラフ化 plot_bme_data.pyで可能←自身のPCでの実行<br>
・matplotは、ラズパイ側での規制が入ってるので、使えないので、一括してグラフ化までは行えない。<br>
bme280_complete.pyでもしかしたら一括できるかも？？<br>
↓<br>
pip install matplotlibを仮想環境で実行
