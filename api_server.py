import os
import json
import csv
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
from datetime import datetime
import pytz

# File paths
DATA_COLLECTION_FILE = 'data_collection.json'
AP_STATUS_FILE = 'ap_status.csv'

# Initialize the JSON and CSV files
def init_files():
    if not os.path.exists(DATA_COLLECTION_FILE):
        with open(DATA_COLLECTION_FILE, 'w') as f:
            json.dump([], f)

    if not os.path.exists(AP_STATUS_FILE):
        with open(AP_STATUS_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['apID', 'timestamp', 'status'])

# Update data in the CSV file
def update_ap_status(apID, new_timestamp, status):
    rows = []
    updated = False
    try:
        with open(AP_STATUS_FILE, 'r', newline='') as f:
            reader = csv.reader(f)
            headers = next(reader)
            for row in reader:
                if row[0] == apID:
                    row[1] = new_timestamp  # Update timestamp
                    row[2] = status         # Update status
                    updated = True
                rows.append(row)

        if updated:
            with open(AP_STATUS_FILE, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                writer.writerows(rows)
        return updated
    except IOError as e:
        print(f"Error reading/writing the file {AP_STATUS_FILE}: {e}")
        return False

# Save POST data to JSON file
def save_to_data_collection(data):
    if not os.path.exists(DATA_COLLECTION_FILE):
        with open(DATA_COLLECTION_FILE, 'w') as f:
            json.dump([], f)

    try:
        with open(DATA_COLLECTION_FILE, 'r+') as f:
            try:
                records = json.load(f)
                if not isinstance(records, list):
                    records = []
            except json.JSONDecodeError:
                records = []

            records.append(data)
            f.seek(0)
            json.dump(records, f, indent=4)
            f.truncate()
    except IOError as e:
        print(f"Error opening or writing to the file {DATA_COLLECTION_FILE}: {e}")

# Function to get the latest 6 entries from data_collection.json
def get_latest_data():
    try:
        with open(DATA_COLLECTION_FILE, 'r') as f:
            records = json.load(f)
            # Return the last 6 records or fewer if the file has less than 6 records
            return records[-6:] if len(records) > 6 else records
    except (IOError, json.JSONDecodeError) as e:
        print(f"Error reading the file {DATA_COLLECTION_FILE}: {e}")
        return []

# HTTP Server setup
class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self, status_code=200, content_type="application/json"):
        self.send_response(status_code)
        self.send_header("Content-type", content_type)
        self.end_headers()

    def do_GET(self):
        parsed_url = urlparse(self.path)
        query_params = parse_qs(parsed_url.query)

        if parsed_url.path == '/access_point':
            device_id = query_params.get("DeviceID", [None])[0]

            mapping_table = {
                "11568181": "1001",
                "9208885": "1002",
                "7390399": "1003",
                "14185269": "1004",
                "12357853": "1005",
                "15769261": "1006",
                "4984236": "1007",
            }

            apID = mapping_table.get(device_id)

            if apID:
                bangkok_tz = pytz.timezone('Asia/Bangkok')
                timestamp = datetime.now(bangkok_tz).strftime('%Y-%m-%d %H:%M:%S.%f')

                status = "Active"
                updated = update_ap_status(apID, timestamp, status)

                if updated:
                    response = {"status": "success", "message": "Device found", "apID": apID, "timestamp": timestamp}
                    self._set_headers(200)
                else:
                    response = {"status": "error", "message": "apID not found in CSV", "apID": apID}
                    self._set_headers(404)
            else:
                response = {"status": "error", "message": "DeviceID not found", "device_id": device_id}
                self._set_headers(404)

            self.wfile.write(json.dumps(response).encode('utf-8'))

        elif parsed_url.path == '/index':
            # Handle the /index endpoint
            data = get_latest_data()
            response = {"status": "success", "data": data}
            self._set_headers(200)
            self.wfile.write(json.dumps(response).encode('utf-8'))

        else:
            self._set_headers(404)
            self.wfile.write(b'{"error": "Endpoint not found"}')

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        try:
            data = json.loads(post_data.decode('utf-8'))
        except UnicodeDecodeError as e:
            print(f"Unicode decoding error: {e}")
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"error": "Invalid UTF-8 encoding"}')
            return

        bangkok_tz = pytz.timezone('Asia/Bangkok')
        serverstamp = datetime.now(bangkok_tz).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]
        data['serverstamp'] = serverstamp

        parsed_url = urlparse(self.path)
        if parsed_url.path == '/data':
            save_to_data_collection(data)
            self._set_headers(200)
            response = {"status": "success", "endpoint": "/data", "received": data}
            self.wfile.write(json.dumps(response).encode('utf-8'))

        elif parsed_url.path == '/access_point':
            if 'apID' in data and 'timestamp' in data and 'status' in data:
                save_to_ap_status(data)
                self._set_headers(200)
                response = {"status": "success", "endpoint": "/access_point", "received": data}
                self.wfile.write(json.dumps(response).encode('utf-8'))
            else:
                self._set_headers(400)
                self.wfile.write(b'{"error": "Missing required fields (apID, timestamp, status)"}')

        else:
            self._set_headers(404)
            self.wfile.write(b'{"error": "Endpoint not found"}')

# Run the server
def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=8080):
    init_files()
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f"Starting httpd server on port {port}")
    httpd.serve_forever()

if __name__ == "__main__":
    run()
