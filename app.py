from flask import Flask, render_template
from flask_socketio import SocketIO
import pandas as pd
from datetime import datetime, timedelta
import time

app = Flask(__name__)
socketio = SocketIO(app)

# ฟังก์ชั่นเพื่อตรวจสอบสถานะ
def get_status(timestamp):
    try:
        # แปลง timestamp เป็น datetime
        timestamp_with_date = datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f")
    except ValueError:
        # หาก timestamp ไม่สามารถแปลงได้ ให้คืนค่าเป็นสถานะ 0 (ไฟสีแดง)
        return 0
    #current_time = datetime.now() + timedelta(hours=7)
    current_time = datetime.now()
    time_diff = current_time - timestamp_with_date

    # ตรวจสอบว่าเวลาห่างกันน้อยกว่า 10 นาทีหรือไม่
    if time_diff < timedelta(minutes=1):
        return 1  # สถานะ 1 (ไฟสีเขียว)
    else:
        return 0  # สถานะ 0 (ไฟสีแดง)

@app.route('/')
def index():
    return render_template('index.html')

# ฟังก์ชั่นในการอัปเดตข้อมูลแบบเรียลไทม์
# ฟังก์ชั่นในการอัปเดตข้อมูลแบบเรียลไทม์
@socketio.on('requestData')
def handle_request_data():
    csv_file = r'C:\Users\piyap\Dropbox\03AIT\IOT_project\server\ap_status.csv'
    data = pd.read_csv(csv_file)

    # ตรวจสอบว่ามีคอลัมน์ apID, timestamp, และ status
    if 'apID' in data.columns and 'timestamp' in data.columns:
        data['status'] = data['timestamp'].apply(get_status)
        ap_data = data[['apID', 'timestamp', 'status']].to_dict(orient='records')  # ส่งข้อมูลในลำดับที่ถูกต้อง

        # ส่งข้อมูลให้ลูกข่ายผ่าน WebSocket
        socketio.emit('updateData', ap_data)
    else:
        print("CSV format is incorrect, missing required columns.")

if __name__ == '__main__':
    socketio.run(app, debug=True)

