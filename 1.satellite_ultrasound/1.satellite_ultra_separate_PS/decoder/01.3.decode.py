from flask import Flask, request, jsonify
from waitress import serve
import logging, struct, threading
import os
from datetime import datetime, timezone
PORT = 5000

# For InfluxDB Client
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# **************************** IMEIs - S ****************************
IMEI_satellite_Lars                 = "301434061665020"
altitude_satellite_Lars_cm          = 10000

IMEI_satellite_ultra_VN1            = "301434062610970"
altitude_satellite_ultra_VN1_cm     = 0

IMEI_satellite_lidar_VN1            = "301434062610..."
altitude_satellite_lidar_VN1_cm     = 0

              # Dict values: [measurement_name, node_name,  region_tag, altitude_cm]
imei_to_node =  {
  IMEI_satellite_ultra_VN1 : ["Table1_satellite_2026", "satellite_ultra_VN1", "Vietnam", altitude_satellite_ultra_VN1_cm],
  IMEI_satellite_lidar_VN1 : ["Table1_satellite_2026", "satellite_lidar_VN1", "Vietnam", altitude_satellite_lidar_VN1_cm],
}
# **************************** IMEIs - E ****************************


# *************************************** InfluxDB - s ***********************************
# API TOKEN - Obtained from InfluxDB UI
os.environ["INFLUXDB_TOKEN"] = "FKeoUEiwE4VbbZN04ZS6s40Ez3HTaRFgtsE6tHh_S7tQXLm561bxn26U264tQsyLZkmwMNoUnqkqx883-jLchA=="
token = os.environ.get("INFLUXDB_TOKEN")
org = "qnunes"
bucket="qnunes_bucketpa1"
url = "http://localhost:8086"                 

write_client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# Compute water level from raw distance, build an InfluxDB point, and write it.
def write_to_influx(imei, parsed):
    if "sleep_time_s" not in parsed:
        return

    sleep_s     = parsed["sleep_time_s"]
    vcap_mv     = parsed["vcap_mv"]
    vpv_mv      = parsed["vpv_mv"]
    distance_cm = parsed["Distance_cm"]
    csq         = parsed["signal_quality"]

    # Decoding Water Levels from Measured Distance Data
    altitude_cm    = imei_to_node[imei][3]
    water_level_cm = altitude_cm + distance_cm

    point = (
        Point(imei_to_node[imei][0])
        .tag("Node", imei_to_node[imei][1])
        .tag("region", imei_to_node[imei][2])
        .time(datetime.now(timezone.utc),       WritePrecision.NS)
        .field("SAT sleeping time (s)",         sleep_s)
        .field("SAT capacitor voltage (V)",     float(vcap_mv) / 1000)
        .field("SAT solar_voltage(V)",          float(vpv_mv)  / 1000)
        .field("SAT WaterLevel (cm)",           water_level_cm)
        .field("SAT SignalQuality",             csq)
    )
    write_api.write(bucket=bucket, org="qnunes", record=point)
    logger.info("InfluxDB write OK | IMEI %s | WaterLevel=%dcm (altitude=%dcm - measured=%dcm)", imei, water_level_cm, altitude_cm, distance_cm)

# *************************************** InfluxDB - E ***********************************

# Binary frame: 10 bytes, 5 × little-endian uint16
# [0-1] sleep_time_s  [2-3] vcap_mv  [4-5] vpv_mv  [6-7] water_level_cm  [8-9] csq
FRAME_FORMAT = "<HHHHH"
FRAME_SIZE   = 10

# Unpack 10 raw bytes from the satellite message into named sensor readings.
# Returns a dict with raw_hex always present; sensor fields added if frame is complete.
def parse_payload(raw: bytes) -> dict:
    result = {"raw_hex": raw.hex()}
    if len(raw) >= FRAME_SIZE:
        sleep_s, vcap_mv, vpv_mv, distance_cm, csq = struct.unpack_from(FRAME_FORMAT, raw)
        result.update({
            "sleep_time_s":   sleep_s,
            "vcap_mv":        vcap_mv,
            "vpv_mv":         vpv_mv,
            "Distance_cm":    distance_cm,
            "signal_quality": csq,
        })
    return result

app = Flask(__name__)
# Handle an incoming POST from the RockBLOCK server.
# Validates the form fields, decodes the hex payload, returns HTTP 200 immediately,
# then writes to InfluxDB in a background thread.
@app.route("/iridium", methods=["POST"])
def receive_iridium():
    form = request.form
    imei = form.get("imei")

    if not form:
        return jsonify({"error": "No form data"}), 400
    if not imei:
        return jsonify({"error": "Missing imei"}), 400
    if imei not in imei_to_node:
        logger.warning("Unknown IMEI %s — message ignored.", imei)
        return jsonify({"error": "Unknown IMEI"}), 403

    device_name = imei_to_node[imei][1]
    parsed = {}
    payload_hex = form.get("data")
    payload_bytes = bytes.fromhex(payload_hex)
    if payload_hex:
        try:
            parsed = parse_payload(payload_bytes)
        except ValueError:
            return jsonify({"error": "Invalid hex payload"}), 400

    logger.info("IMEI %s (%s) | MOMSN %s | parsed: %s", imei, device_name, form.get("momsn"), parsed)

    # Return HTTP 200 immediately, then write to InfluxDB in the background
    threading.Thread(target=write_to_influx, args=(imei, parsed), daemon=True).start()

    return jsonify({"status": "OK", "parsed": parsed}), 200

if __name__ == "__main__":
    serve(app, host="0.0.0.0", port=PORT)