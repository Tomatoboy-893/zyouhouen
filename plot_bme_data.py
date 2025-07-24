# 必要なライブラリをインポート
import pandas as pd # type: ignore
import matplotlib.pyplot as plt # type: ignore
import matplotlib.dates as mdates # type: ignore
import sys

# --- 設定 ---
# 読み込むCSVファイル名
INPUT_CSV_FILE = 'bme280_log.csv'
# 保存するグラフの画像ファイル名
OUTPUT_IMAGE_FILE = 'bme280_graph.png'

def plot_sensor_data(csv_file):
    # CSVファイルの読み込み
    try:
        print(f"'{csv_file}' を読み込んでいます...")
        # timestamp列を日付時刻型としてパースし、インデックスに設定
        df = pd.read_csv(
            csv_file,
            parse_dates=['timestamp'],
            index_col='timestamp'
        )
        print("ファイルの読み込みが完了しました。")
    except FileNotFoundError:
        print(f"エラー: ファイル '{csv_file}' が見つかりません。", file=sys.stderr)
        print("BME280のデータ収集スクリプトを先に実行してください。", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"エラー: ファイルの読み込み中に問題が発生しました: {e}", file=sys.stderr)
        sys.exit(1)

    # データが空でないか確認
    if df.empty:
        print("警告: CSVファイルにデータが含まれていません。グラフは生成されません。")
        return

    print("グラフを生成しています...")

    # グラフの準備 (3つのグラフを縦に並べる)
    # figsizeで全体のサイズを、sharex=TrueでX軸(時間軸)を共有
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 10), sharex=True)

    # --- 1. 温度のグラフ ---
    axes[0].plot(df.index, df['temperature_c'], color='red', marker='.', linestyle='-')
    axes[0].set_title('Temperature Over Time')
    axes[0].set_ylabel('Temperature (°C)')
    axes[0].grid(True)

    # --- 2. 気圧のグラフ ---
    axes[1].plot(df.index, df['pressure_hpa'], color='blue', marker='.', linestyle='-')
    axes[1].set_title('Pressure Over Time')
    axes[1].set_ylabel('Pressure (hPa)')
    axes[1].grid(True)

    # --- 3. 湿度のグラフ ---
    axes[2].plot(df.index, df['humidity_percent'], color='green', marker='.', linestyle='-')
    axes[2].set_title('Humidity Over Time')
    axes[2].set_ylabel('Humidity (%)')
    axes[2].grid(True)

    # X軸のフォーマットを設定
    axes[2].set_xlabel('Time')
    # 日付と時刻が見やすいようにフォーマットを指定
    xfmt = mdates.DateFormatter('%Y-%m-%d\n%H:%M:%S')
    axes[2].xaxis.set_major_formatter(xfmt)
    fig.autofmt_xdate(rotation=45, ha='right') # ラベルが重ならないように自動調整

    # 全体のレイアウトを調整
    plt.tight_layout()

    # グラフを画像ファイルとして保存
    try:
        plt.savefig(OUTPUT_IMAGE_FILE, dpi=150)
        print(f"グラフを '{OUTPUT_IMAGE_FILE}' として保存しました。")
    except Exception as e:
        print(f"エラー: グラフの保存に失敗しました: {e}", file=sys.stderr)


    # グラフを表示
    plt.show()


if __name__ == '__main__':
    plot_sensor_data(INPUT_CSV_FILE)