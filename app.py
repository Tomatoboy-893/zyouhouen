# app.py - 完璧版
from flask import Flask, render_template, jsonify
import time
from datetime import datetime
import collections
import threading
import logging
import json

# ログ設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# BME280センサー用のクラス
class BME280Sensor:
    def __init__(self):
        self.bus = None
        self.initialized = False
        self.digT = []
        self.digP = []
        self.digH = []
        self.t_fine = 0.0
        self.I2C_BUS = 1
        self.I2C_ADDR = 0x76
        self.last_error = None
        
    def init_bus(self):
        """I2Cバスを安全に初期化"""
        if self.bus is not None:
            return True
            
        try:
            from smbus2 import SMBus
            self.bus = SMBus(self.I2C_BUS)
            return True
        except ImportError:
            logger.warning("smbus2がインストールされていません。模擬モードで動作します。")
            return False
        except Exception as e:
            logger.error(f"I2Cバス初期化失敗: {e}")
            self.last_error = str(e)
            return False
    
    def write_reg(self, reg, data):
        """レジスタに書き込み"""
        if not self.init_bus():
            return False
        try:
            self.bus.write_byte_data(self.I2C_ADDR, reg, data)
            return True
        except Exception as e:
            logger.error(f"書き込み失敗 reg={hex(reg)}: {e}")
            return False
    
    def read_byte(self, reg, signed=False):
        """1バイト読み込み"""
        if not self.init_bus():
            return None
        try:
            value = self.bus.read_byte_data(self.I2C_ADDR, reg)
            if signed and value > 127:
                value -= 256
            return value
        except Exception as e:
            logger.error(f"読み込み失敗 reg={hex(reg)}: {e}")
            return None
    
    def read_word(self, reg, signed=False):
        """2バイト読み込み"""
        lsb = self.read_byte(reg)
        msb = self.read_byte(reg + 1)
        if lsb is None or msb is None:
            return None
        
        value = (msb << 8) | lsb
        if signed and value & 0x8000:
            value -= 65536
        return value
    
    def setup_sensor(self):
        """センサー設定"""
        # 湿度オーバーサンプリング x1
        if not self.write_reg(0xF2, 0x01): return False
        # 温度・気圧オーバーサンプリング x1, ノーマルモード
        if not self.write_reg(0xF4, 0x27): return False
        # スタンバイ 1000ms, フィルタOFF
        if not self.write_reg(0xF5, 0xA0): return False
        return True
    
    def read_calibration(self):
        """校正パラメータ読み込み"""
        # 温度校正
        self.digT = [
            self.read_word(0x88),           # T1
            self.read_word(0x8A, True),     # T2
            self.read_word(0x8C, True)      # T3
        ]
        if None in self.digT: return False
        
        # 気圧校正
        self.digP = [self.read_word(0x8E)]  # P1
        for i in range(1, 9):
            val = self.read_word(0x90 + (i-1)*2, True)
            if val is None: return False
            self.digP.append(val)
        
        # 湿度校正
        self.digH = []
        self.digH.append(self.read_byte(0xA1)) # H1
        calib_data = self.bus.read_i2c_block_data(self.I2C_ADDR, 0xE1, 7)
        self.digH.append((calib_data[1] << 8) | calib_data[0]) # H2
        self.digH.append(calib_data[2]) # H3
        self.digH.append((calib_data[3] << 4) | (calib_data[4] & 0x0F)) # H4
        self.digH.append((calib_data[5] << 4) | (calib_data[4] >> 4)) # H5
        self.digH.append(calib_data[6]) # H6
        if None in self.digH: return False
        
        return True
    
    def read_raw_data(self):
        """生データ読み込み"""
        if not self.init_bus():
            return None, None, None
        try:
            # 8バイト一括読み込み
            data = self.bus.read_i2c_block_data(self.I2C_ADDR, 0xF7, 8)
            pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            hum_raw = (data[6] << 8) | data[7]
            return temp_raw, pres_raw, hum_raw
        except Exception as e:
            logger.error(f"生データ読み込み失敗: {e}")
            return None, None, None

    def compensate_temp(self, raw_temp):
        """温度補正 (t_fine計算を含む)"""
        var1 = (raw_temp / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        var2 = ((raw_temp / 131072.0 - self.digT[0] / 8192.0) *
                (raw_temp / 131072.0 - self.digT[0] / 8192.0)) * self.digT[2]
        self.t_fine = var1 + var2
        temperature = self.t_fine / 5120.0
        return temperature

    def compensate_pressure(self, raw_pres):
        """気圧補正（データシート準拠版）"""
        if not self.digP or raw_pres is None or self.t_fine == 0: return None
        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.digP[5] / 32768.0
        var2 = var2 + var1 * self.digP[4] * 2.0
        var2 = (var2 / 4.0) + (self.digP[3] * 65536.0)
        var1 = (self.digP[2] * var1 * var1 / 524288.0 + self.digP[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.digP[0]
        if var1 == 0:
            return 0
        p = 1048576.0 - raw_pres
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self.digP[8] * p * p / 2147483648.0
        var2 = p * self.digP[7] / 32768.0
        p = p + (var1 + var2 + self.digP[6]) / 16.0
        return p / 100.0 # hPaに変換

    def compensate_humidity(self, raw_hum):
        """湿度補正（データシート準拠版）"""
        if not self.digH or raw_hum is None or self.t_fine == 0: return None
        v_x1_u32r = self.t_fine - 76800.0
        v_x1_u32r = (raw_hum - (self.digH[3] * 64.0 + self.digH[4] / 16384.0 * v_x1_u32r)) * \
                    (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * v_x1_u32r * \
                    (1.0 + self.digH[2] / 67108864.0 * v_x1_u32r)))
        humidity = v_x1_u32r * (1.0 - self.digH[0] * v_x1_u32r / 524288.0)
        return max(0.0, min(100.0, humidity))
    
    def initialize(self):
        """センサー初期化"""
        try:
            if not self.init_bus():
                return False
            
            if not self.setup_sensor():
                logger.error("センサー設定失敗")
                return False
            
            if not self.read_calibration():
                logger.error("校正パラメータ読み込み失敗")
                return False
            
            self.initialized = True
            logger.info("BME280センサー初期化完了")
            return True
            
        except Exception as e:
            logger.error(f"センサー初期化エラー: {e}")
            self.last_error = str(e)
            return False
    
    def read_data(self):
        """センサーデータ読み込み"""
        if not self.initialized:
            return None, None, None
        
        temp_raw, pres_raw, hum_raw = self.read_raw_data()
        if temp_raw is None or pres_raw is None or hum_raw is None:
            return None, None, None
        
        temperature = self.compensate_temp(temp_raw)
        pressure = self.compensate_pressure(pres_raw)
        humidity = self.compensate_humidity(hum_raw)
        
        return temperature, pressure, humidity

# グローバル変数
sensor = BME280Sensor()
data_history = collections.deque(maxlen=50)  # 最大50個のデータ
latest_data = None
data_lock = threading.Lock()
app_running = True

def data_collector():
    """バックグラウンドデータ収集"""
    global latest_data
    logger.info("データ収集開始")
    
    while app_running:
        try:
            temp, pres, hum = sensor.read_data()
            
            if temp is not None and pres is not None and hum is not None:
                data = {
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'temperature': round(temp, 1),
                    'pressure': round(pres, 1),
                    'humidity': round(hum, 1)
                }
                
                with data_lock:
                    data_history.append(data)
                    latest_data = data
                
                logger.debug(f"データ更新: {temp:.1f}°C, {pres:.1f}hPa, {hum:.1f}%")
            else:
                if sensor.initialized: # 初期化成功後に読み取れなくなった場合
                    logger.warning("センサーデータ読み込み失敗")
            
        except Exception as e:
            logger.error(f"データ収集エラー: {e}")
        
        time.sleep(5)  # 5秒間隔

# Flaskアプリ
app = Flask(__name__)

@app.route('/')
def index():
    """メインページ"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>BME280センサー</title>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; margin: 20px; background: #f4f7f9; color: #333; }
            .container { max-width: 800px; margin: 0 auto; background: white; padding: 25px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.08); }
            h1 { color: #1a237e; }
            .sensor-data { display: flex; flex-wrap: wrap; justify-content: space-around; margin: 25px 0; gap: 15px; }
            .data-box { text-align: center; padding: 20px; background: #e8eaf6; border-radius: 10px; min-width: 130px; flex-grow: 1; transition: transform 0.2s; }
            .data-box:hover { transform: translateY(-5px); }
            .value { font-size: 28px; font-weight: 600; color: #3f51b5; }
            .unit { font-size: 14px; color: #555; }
            .label { font-size: 16px; margin-top: 5px; color: #333; }
            .status { padding: 10px 15px; margin: 20px 0; border-radius: 8px; text-align: center; font-weight: 500;}
            .online { background: #e8f5e9; color: #2e7d32; border-left: 5px solid #4caf50; }
            .offline { background: #ffebee; color: #c62828; border-left: 5px solid #f44336; }
            .demo { background: #e3f2fd; color: #1565c0; border-left: 5px solid #2196f3; }
            .controls { text-align: center; margin-top: 20px; }
            button { padding: 10px 20px; margin: 5px; border: none; border-radius: 8px; cursor: pointer; font-size: 16px; transition: background-color 0.2s; }
            .btn-primary { background: #3f51b5; color: white; }
            .btn-primary:hover { background: #303f9f; }
            .btn-secondary { background: #9e9e9e; color: white; }
            .btn-secondary:hover { background: #757575; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🌡️ BME280センサーモニター</h1>
            <div id="status" class="status offline">接続中...</div>
            
            <div class="sensor-data">
                <div class="data-box">
                    <div class="value" id="temp">--</div>
                    <div class="unit">°C</div>
                    <div class="label">温度</div>
                </div>
                <div class="data-box">
                    <div class="value" id="press">--</div>
                    <div class="unit">hPa</div>
                    <div class="label">気圧</div>
                </div>
                <div class="data-box">
                    <div class="value" id="hum">--</div>
                    <div class="unit">%</div>
                    <div class="label">湿度</div>
                </div>
            </div>
            
            <div class="controls">
                <button class="btn-primary" onclick="updateData()">データ更新</button>
                <button class="btn-secondary" onclick="toggleAutoUpdate()">自動更新: <span id="auto-status">ON</span></button>
            </div>
            
            <div id="last-update" style="text-align: center; margin-top: 15px; color: #777; font-size: 14px;"></div>
        </div>

        <script>
            let autoUpdate = true;
            let updateInterval;
            
            function updateData() {
                fetch('/api/latest')
                    .then(response => {
                        if (!response.ok) {
                            throw new Error('Network response was not ok');
                        }
                        return response.json();
                    })
                    .then(data => {
                        const statusEl = document.getElementById('status');
                        
                        document.getElementById('temp').textContent = data.temperature !== undefined ? data.temperature : '--';
                        document.getElementById('press').textContent = data.pressure !== undefined ? data.pressure : '--';
                        document.getElementById('hum').textContent = data.humidity !== undefined ? data.humidity : '--';
                        document.getElementById('last-update').textContent = '最終更新: ' + (data.timestamp || '不明');

                        if (data.demo) {
                            statusEl.textContent = 'デモモードで動作中';
                            statusEl.className = 'status demo';
                        } else if (data.error) {
                            statusEl.textContent = 'エラー: ' + data.error;
                            statusEl.className = 'status offline';
                        } else {
                            statusEl.textContent = 'オンライン';
                            statusEl.className = 'status online';
                        }
                    })
                    .catch(error => {
                        console.error('エラー:', error);
                        const statusEl = document.getElementById('status');
                        statusEl.textContent = 'サーバー接続エラー';
                        statusEl.className = 'status offline';
                    });
            }
            
            function toggleAutoUpdate() {
                autoUpdate = !autoUpdate;
                document.getElementById('auto-status').textContent = autoUpdate ? 'ON' : 'OFF';
                
                if (autoUpdate) {
                    if (!updateInterval) {
                        updateInterval = setInterval(updateData, 5000);
                    }
                } else {
                    clearInterval(updateInterval);
                    updateInterval = null;
                }
            }
            
            // 初期化
            document.addEventListener('DOMContentLoaded', () => {
                updateData();
                updateInterval = setInterval(updateData, 5000);
            });
        </script>
    </body>
    </html>
    '''

@app.route('/api/latest')
def api_latest():
    """最新データAPI"""
    with data_lock:
        if latest_data:
            return jsonify(latest_data)
        else:
            # センサーが初期化されていない場合、デモデータを返す
            return jsonify({
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'temperature': 25.0,
                'pressure': 1013.2,
                'humidity': 55.0,
                'demo': True
            })

@app.route('/api/history')
def api_history():
    """履歴データAPI"""
    with data_lock:
        return jsonify(list(data_history))

@app.route('/api/status')
def api_status():
    """ステータスAPI"""
    with data_lock:
        return jsonify({
            'sensor_initialized': sensor.initialized,
            'data_count': len(data_history),
            'last_error': sensor.last_error,
            'app_start_time': app.config.get('START_TIME')
        })

def create_app():
    """アプリ初期化"""
    # センサー初期化
    if sensor.initialize():
        logger.info("✅ センサー初期化成功")
    else:
        logger.warning("⚠️ センサー初期化失敗 - デモモードで動作します")
    
    # データ収集スレッド開始
    collector_thread = threading.Thread(target=data_collector, daemon=True)
    collector_thread.start()
    
    app.config['START_TIME'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    return app

if __name__ == '__main__':
    try:
        app = create_app()
        logger.info("🚀 Flaskアプリ起動 (http://0.0.0.0:5000)")
        # Waitress を本番環境での使用に推奨
        # from waitress import serve
        # serve(app, host='0.0.0.0', port=5000)
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        logger.info("👋 アプリケーションを終了します")
        app_running = False
    except Exception as e:
        logger.critical(f"💥 起動エラー: {e}")
        app_running = False
