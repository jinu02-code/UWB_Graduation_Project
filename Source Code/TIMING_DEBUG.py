//삼각대에 앵커 하고 나서 맵 크기 바꾼거

from flask import Flask, request, jsonify, render_template
from datetime import datetime
import time
import math
import json
import requests
from pathlib import Path
from collections import defaultdict

app = Flask(__name__)

PERF_LOG = False

def perf_ms():
    return time.perf_counter() * 1000.0

def perf_log(stage, t0=None, extra=""):
    if not PERF_LOG:
        return
    now = perf_ms()
    if t0 is None:
        print(f"[PERF] {stage} at={now:.3f}ms {extra}")
    else:
        print(f"[PERF] {stage} elapsed={now - t0:.3f}ms {extra}")


# ============================================================
# 앵커 좌표 설정 [m]
#
# 기본 구조:
# A1 ───────── A4
# │            │
# │            │
# A2 ───────── A3
#
# 현재 설정: 실제 설치 공간 기준 약 3.24m × 3.24m 배치
#
# 기준 좌표계:
#   A1 = (0.00, 0.00)         # 좌상단 기준점
#   A4 = (3.24, 0.00)         # A1 → A4 거리: 약 3.24m
#   A2 = (0.00, 3.16)         # A1 → A2 거리: 약 3.16m
#   A3 = (3.16, 3.24)         # A2 → A3 거리: 약 3.16m, A4 → A3 거리: 약 3.24m
#
# 주요 앵커 간 거리:
#   A1 - A4 = 3.24m           # 상단 가로 거리
#   A1 - A2 = 3.16m           # 왼쪽 세로 거리
#   A2 - A3 = 3.16m           # 하단 가로 거리
#   A4 - A3 = 3.24m           # 오른쪽 세로 거리
#
# 대각선 거리:
#   A1 - A3 ≈ 4.53m
#   A2 - A4 ≈ 4.53m
#
# 주의:
# - 완전한 정사각형이 아니라 실제 설치 오차를 반영하여
#   A2는 y=3.16m, A3는 x=3.16m로 설정한다.
# - x축: A1 → A4 방향
# - y축: A1 → A2 방향
# - HTML 맵은 이 좌표값을 AWS /latest의 room 정보로 받아 자동 반영한다.
# ============================================================

ROOM_WIDTH_M = 3.24
ROOM_HEIGHT_M = 3.24
 

ANCHORS = {
    1: (0.00, 0.00),   # A1: 기준점, 좌상단 앵커
    4: (3.24, 0.00),   # A4: A1-A4 거리 약 3.24m, 우상단 앵커
    2: (0.00, 3.16),   # A2: A1-A2 거리 약 3.16m, 좌하단 앵커
    3: (3.16, 3.24),   # A3: A2-A3 거리 약 3.16m, A4-A3 거리 약 3.24m, 우하단 앵커
}  

ROOM_POLYGON = [ANCHORS[1], ANCHORS[4], ANCHORS[3], ANCHORS[2]]

MIN_VALID_DISTANCE_M = 0.02
MAX_VALID_DISTANCE_M = 20.0
MAX_ANCHOR_AGE_SEC = 1.5

# 같은 TAG-Anchor에서 매우 짧은 시간에 비현실적으로 튀는 거리값은 버림
DISTANCE_OUTLIER_WINDOW_SEC = 1.0
MAX_DISTANCE_JUMP_M = 2.5

latest = defaultdict(dict)
latest_position = {}
# 태그별 최신 배터리 상태 저장
# 거리값은 앵커별로 필요하지만, 배터리값은 TAG1/TAG2별 최신값 하나만 관리하면 됨
latest_battery = {}

DEVICE_ID = "raspberrypi_uwb_01"
LATEST_JSON_PATH = Path("latest.json")

# ============================================================
# API Gateway 설정
#
# API_POSITION_URL에는 API Gateway의 POST /position 주소를 넣어야 함.
# 예:
# API_POSITION_URL = "https://xxxx.execute-api.us-east-2.amazonaws.com/position"
# ============================================================

API_POSITION_URL = "https://ywfa8nwekc.execute-api.us-east-2.amazonaws.com/position"

API_POST_INTERVAL_SEC = 0.125
HISTORY_SAVE_INTERVAL_SEC = 10.0

last_api_post_time = 0.0
last_history_save_time = 0.0


def now_iso():
    return datetime.now().isoformat(timespec="seconds")


# ============================================================
# Battery handling policy
#
# TAG -> Anchor -> RPi 경로에서 가장 안정적으로 전달되는 값은 voltage[mV/V]이다.
# 따라서 Anchor가 보낸 battery_percent는 사용하지 않고, RPi가 voltage로 percent를 계산한다.
#
# Li-ion 1셀 단순 근사:
#   4.15 V 이상 = 100%
#   3.30 V 이하 = 0%
#   그 사이 = 선형 보간
#
# 갑자기 0.25 V 이상 튀는 값은 UWB 바이트 오염/잘못된 패킷으로 보고 latest_battery를 갱신하지 않는다.
# ============================================================
BATTERY_EMPTY_V = 3.30
BATTERY_FULL_V = 4.15
BATTERY_MIN_VALID_V = 3.20
BATTERY_MAX_VALID_V = 4.30
BATTERY_MAX_JUMP_V = 0.25


def estimate_battery_percent_from_voltage(voltage):
    """voltage[V]만 사용해서 배터리 잔량[%]을 계산한다."""
    try:
        v = float(voltage)
    except Exception:
        return None

    if math.isnan(v) or math.isinf(v):
        return None

    if v >= BATTERY_FULL_V:
        return 100
    if v <= BATTERY_EMPTY_V:
        return 0

    percent = round((v - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100.0)
    return int(min(max(percent, 0), 100))


def is_valid_tag_battery_voltage(voltage):
    """태그 Li-ion 1셀 기준으로 현실적인 전압 범위만 통과시킨다."""
    try:
        v = float(voltage)
    except Exception:
        return False

    if math.isnan(v) or math.isinf(v):
        return False

    return BATTERY_MIN_VALID_V <= v <= BATTERY_MAX_VALID_V


def battery_acceptance_reason(tag_id, voltage):
    """
    새 battery_voltage를 latest_battery에 저장해도 되는지 판단한다.
    반환: (accept: bool, reason: str)
    """
    if not is_valid_tag_battery_voltage(voltage):
        return False, "invalid_range"

    last = latest_battery.get(tag_id)
    if not last:
        return True, "first_valid"

    last_v = float(last.get("battery_voltage", 0.0))
    if abs(float(voltage) - last_v) > BATTERY_MAX_JUMP_V:
        return False, f"sudden_jump_from_{last_v:.3f}V"

    return True, "accepted"

def estimate_position_least_squares(distances):
    """
    distances: {anchor_id: distance_m}
    return: (x, y) or None

    A1~A4 거리값으로 least squares 기반 삼변측량 수행.
    """
    valid = {
        aid: float(d)
        for aid, d in distances.items()
        if aid in ANCHORS and d > 0
    }

    if len(valid) < 3:
        return None

    ref_id = sorted(valid.keys())[0]
    x1, y1 = ANCHORS[ref_id]
    d1 = valid[ref_id]

    A = []
    b = []

    for aid in sorted(valid.keys()):
        if aid == ref_id:
            continue

        xi, yi = ANCHORS[aid]
        di = valid[aid]

        A.append([2.0 * (xi - x1), 2.0 * (yi - y1)])
        b.append(d1**2 - di**2 + xi**2 - x1**2 + yi**2 - y1**2)

    a11 = sum(row[0] * row[0] for row in A)
    a12 = sum(row[0] * row[1] for row in A)
    a22 = sum(row[1] * row[1] for row in A)

    c1 = sum(A[i][0] * b[i] for i in range(len(A)))
    c2 = sum(A[i][1] * b[i] for i in range(len(A)))

    det = a11 * a22 - a12 * a12

    if abs(det) < 1e-9:
        return None

    x = (c1 * a22 - c2 * a12) / det
    y = (a11 * c2 - a12 * c1) / det

    return x, y


def point_in_polygon(x, y, polygon):
    """
    주어진 점 (x, y)이 polygon 내부 또는 경계에 있으면 True 반환.
    polygon: [(x1, y1), (x2, y2), ...]
    """
    inside = False
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        # 경계선 위에 있는 경우 내부로 처리
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) < 1e-9:
            if min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9 and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9:
                return True

        if (y1 > y) != (y2 > y):
            x_intersect = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < x_intersect:
                inside = not inside

    return inside


def compute_position_for_tag(tag_id):
    """
    tag_id별 최신 A1~A4 거리값을 모아 위치 계산.
    TAG1, TAG2 모두 같은 함수로 계산됨.
    """
    _t0_compute = perf_ms()
    if tag_id not in latest:
        return None

    current_time = time.time()
    distances = {}
    ages = {}

    for aid, item in latest[tag_id].items():
        age = current_time - item["received_unix"]
        ages[aid] = age

        if age <= MAX_ANCHOR_AGE_SEC:
            distances[aid] = item["avg_distance"]

    if len(distances) < 3:
        latest_position[tag_id] = {
            "available": False,
            "reason": "need at least 3 fresh anchors",
            "fresh_anchor_count": len(distances),
            "distances": distances,
            "ages": ages,
            "timestamp": now_iso(),
        }
        perf_log('compute_position_for_tag unavailable', _t0_compute, f'TAG{tag_id} fresh={len(distances)}')
        return None

    pos = estimate_position_least_squares(distances)

    if pos is None:
        latest_position[tag_id] = {
            "available": False,
            "reason": "least squares failed",
            "fresh_anchor_count": len(distances),
            "distances": distances,
            "ages": ages,
            "timestamp": now_iso(),
        }
        perf_log('compute_position_for_tag LS_FAIL', _t0_compute, f'TAG{tag_id}')
        return None

    x, y = pos

    x_clamped = min(max(x, 0.0), ROOM_WIDTH_M)
    y_clamped = min(max(y, 0.0), ROOM_HEIGHT_M)

    # 원본 추정 좌표(x, y)를 기준으로 방 이탈 여부를 판단한다.
    # 화면 표시는 x_clamped/y_clamped를 사용하지만, 이탈 판단은 clamping 전 좌표를 사용해야 한다.
    out_of_room = not point_in_polygon(x, y, ROOM_POLYGON)

    latest_position[tag_id] = {
        "available": True,
        "x": x,
        "y": y,
        "x_clamped": x_clamped,
        "y_clamped": y_clamped,
        "out_of_room": out_of_room,
        "fresh_anchor_count": len(distances),
        "distances": distances,
        "ages": ages,
        "timestamp": now_iso(),
    }

    perf_log('compute_position_for_tag OK', _t0_compute, f'TAG{tag_id} fresh={len(distances)}')
    return latest_position[tag_id]


def build_dynamodb_style_payload():
    """
    S3 latest.json / DynamoDB / IoT Core 확장까지 고려한 공통 payload 생성.

    - S3: latest.json 파일로 업로드해서 웹페이지가 records 배열을 읽으면 됨
    - DynamoDB: records 배열 안의 각 항목을 tag_id + timestamp 기준으로 한 줄씩 저장하면 됨
    - IoT Core: 이 payload 전체를 publish하거나 records를 개별 publish해도 됨
    """
    payload_timestamp = now_iso()

    payload = {
        "device_id": DEVICE_ID,
        "timestamp": payload_timestamp,
        "room": {
            "width_m": ROOM_WIDTH_M,
            "height_m": ROOM_HEIGHT_M,
            "anchors": {
                f"A{aid}": {
                    "anchor_id": aid,
                    "x": coord[0],
                    "y": coord[1],
                }
                for aid, coord in ANCHORS.items()
            },
        },
        "records": []
    }

    for tag_id in sorted(latest.keys()):
        compute_position_for_tag(tag_id)

        pos = latest_position.get(tag_id, {
            "available": False,
            "reason": "no position",
            "fresh_anchor_count": 0,
            "timestamp": payload_timestamp,
        })

        record = {
            "tag_id": f"TAG{tag_id}",
            "numeric_tag_id": tag_id,
            "timestamp": pos.get("timestamp", payload_timestamp),
            "device_id": DEVICE_ID,
            "available": bool(pos.get("available", False)),
            "fresh_anchor_count": int(pos.get("fresh_anchor_count", 0)),
            "room_width_m": ROOM_WIDTH_M,
            "room_height_m": ROOM_HEIGHT_M,
            "anchor_distances": {},
            "anchors": {},
        }

        battery = latest_battery.get(tag_id)
        if battery:
            record["battery_voltage"] = round(float(battery["battery_voltage"]), 3)
            record["battery_mV"] = int(round(float(battery["battery_voltage"]) * 1000.0))
            record["battery_voltage_mV"] = int(round(float(battery["battery_voltage"]) * 1000.0))
            record["battery_percent"] = int(battery["battery_percent"])
            record["battery_updated_at"] = battery["updated_at"]

        for aid, item in sorted(latest[tag_id].items()):
            age_sec = round(time.time() - item["received_unix"], 3)

            record["anchor_distances"][f"A{aid}"] = round(float(item["avg_distance"]), 3)
            record["anchors"][f"A{aid}"] = {
                "anchor_id": aid,
                "distance": round(float(item["distance"]), 3),
                "avg_distance": round(float(item["avg_distance"]), 3),
                "seq": int(item["seq"]),
                "age_sec": age_sec,
                "received_time": item["received_time"],
            }

        if pos.get("available"):
            record.update({
                "x": round(float(pos["x"]), 3),
                "y": round(float(pos["y"]), 3),
                "x_clamped": round(float(pos["x_clamped"]), 3),
                "y_clamped": round(float(pos["y_clamped"]), 3),
                "out_of_room": bool(pos.get("out_of_room", False)),
            })
        else:
            record["out_of_room"] = False
            record["reason"] = pos.get("reason", "unavailable")

        payload["records"].append(record)

    return payload


def save_latest_json():
    """
    Raspberry Pi 로컬 폴더에 latest.json 자동 저장.
    나중에 이 파일을 S3에 업로드하면 외부 웹페이지에서 그대로 읽을 수 있음.
    """
    _t0 = perf_ms()
    payload = build_dynamodb_style_payload()
    perf_log("build_dynamodb_style_payload", _t0)

    _t1 = perf_ms()
    with LATEST_JSON_PATH.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    perf_log("write_latest_json", _t1)

    perf_log("save_latest_json total", _t0)
    return payload


def post_payload_to_api(payload):
    """
    Raspberry Pi에서 계산한 위치 payload를 API Gateway의 POST /position으로 전송.
    """
    global last_api_post_time, last_history_save_time
    _t0_api = perf_ms()

    if not API_POSITION_URL or "https://ywfa8nwekc.execute-api.us-east-2.amazonaws.com/position:" in API_POSITION_URL:
        print("API POST skipped: API_POSITION_URL is not set")
        return

    now = time.time()

    if now - last_api_post_time < API_POST_INTERVAL_SEC:
        return

    save_history = False
    if now - last_history_save_time >= HISTORY_SAVE_INTERVAL_SEC:
        save_history = True
        last_history_save_time = now

    payload["save_history"] = save_history

    try:
        _t_req = perf_ms()
        r = requests.post(API_POSITION_URL, json=payload, timeout=2)
        perf_log("requests.post AWS /position", _t_req, f"status={r.status_code}")
        print(f"API POST status={r.status_code}, body={r.text[:160]}")
        last_api_post_time = now
    except Exception as e:
        perf_log("requests.post AWS /position FAILED", _t0_api, str(e))
        print(f"API POST failed: {e}")
    finally:
        perf_log("post_payload_to_api total", _t0_api)


@app.route("/uwb", methods=["POST"])
def receive_uwb():
    _t0_route = perf_ms()
    """
    앵커 ESP32가 HTTP POST로 거리값을 보내는 주소.
    예시 JSON:
    {
      "tag_id": 1,
      "anchor_id": 4,
      "seq": 107,
      "distance": 2.810,
      "avg_distance": 2.800
    }
    """
    _t_json = perf_ms()
    data = request.get_json(silent=True)
    perf_log('Flask request.get_json', _t_json)

    # DEBUG: Anchor가 실제로 battery 필드를 보내는지 확인용
    # 정상 예: {'tag_id':2, 'anchor_id':1, ..., 'battery_voltage':3.91, 'battery_percent':82}
    print(f"RAW /uwb data: {data}")

    if data is None:
        return jsonify({"ok": False, "error": "invalid json"}), 400

    try:
        tag_id = int(data["tag_id"])
        anchor_id = int(data["anchor_id"])
        seq = int(data["seq"])
        distance = float(data["distance"])
        avg_distance = float(data["avg_distance"])
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400

    # Anchor가 보낸 battery_voltage만 신뢰한다.
    # battery_percent는 받더라도 저장하지 않고 로그에만 남긴다.
    battery_voltage_raw = data.get("battery_voltage")
    battery_mV_raw = data.get("battery_mV", data.get("battery_mv", data.get("battery_voltage_mV")))
    battery_percent_raw = data.get("battery_percent")

    try:
        if battery_voltage_raw is not None:
            battery_voltage = float(battery_voltage_raw)
        elif battery_mV_raw is not None:
            battery_voltage = float(battery_mV_raw) / 1000.0
        else:
            battery_voltage = None
    except Exception:
        battery_voltage = None

    try:
        battery_percent_from_anchor = int(battery_percent_raw) if battery_percent_raw is not None else None
    except Exception:
        battery_percent_from_anchor = None

    battery_percent = None
    battery_accept = False
    battery_reject_reason = "no_voltage"

    if battery_voltage is not None:
        battery_accept, battery_reject_reason = battery_acceptance_reason(tag_id, battery_voltage)
        if battery_accept:
            battery_percent = estimate_battery_percent_from_voltage(battery_voltage)

    if (
        math.isnan(distance)
        or math.isnan(avg_distance)
        or math.isinf(distance)
        or math.isinf(avg_distance)
        or distance < MIN_VALID_DISTANCE_M
        or avg_distance < MIN_VALID_DISTANCE_M
        or distance > MAX_VALID_DISTANCE_M
        or avg_distance > MAX_VALID_DISTANCE_M
    ):
        print(f"DROP INVALID: TAG{tag_id},A{anchor_id},seq={seq},d={distance},avg={avg_distance}")
        return jsonify({"ok": False, "reason": "invalid distance"}), 200

    prev_item = latest.get(tag_id, {}).get(anchor_id)
    if prev_item:
        prev_age = time.time() - prev_item.get("received_unix", 0)
        prev_avg = float(prev_item.get("avg_distance", avg_distance))
        if prev_age <= DISTANCE_OUTLIER_WINDOW_SEC and abs(avg_distance - prev_avg) > MAX_DISTANCE_JUMP_M:
            print(
                f"DROP OUTLIER: TAG{tag_id},A{anchor_id},seq={seq},"
                f"avg={avg_distance:.3f},prev={prev_avg:.3f},age={prev_age:.3f}s"
            )
            return jsonify({"ok": False, "reason": "distance outlier filtered"}), 200

    item = {
        "tag_id": tag_id,
        "anchor_id": anchor_id,
        "seq": seq,
        "distance": distance,
        "avg_distance": avg_distance,
        "received_time": now_iso(),
        "received_unix": time.time(),
    }

    latest[tag_id][anchor_id] = item

    # 배터리값은 앵커별이 아니라 태그별 최신값으로 저장한다.
    # percent는 Anchor가 보낸 값을 쓰지 않고, RPi에서 voltage 기반으로 계산한 값을 저장한다.
    if battery_accept and battery_voltage is not None and battery_percent is not None:
        latest_battery[tag_id] = {
            "battery_voltage": battery_voltage,
            "battery_percent": battery_percent,
            "updated_at": now_iso(),
            "received_unix": time.time(),
            "source_anchor_id": anchor_id,
            "source_seq": seq,
        }

    _t_pos = perf_ms()
    compute_position_for_tag(tag_id)
    perf_log('receive_uwb compute_position_for_tag call', _t_pos)
    _t_save = perf_ms()
    payload = save_latest_json()
    perf_log('receive_uwb save_latest_json call', _t_save)
    _t_post = perf_ms()
    post_payload_to_api(payload)
    perf_log('receive_uwb post_payload_to_api call', _t_post)

    if battery_accept and battery_voltage is not None and battery_percent is not None:
        raw_pct_text = battery_percent_from_anchor if battery_percent_from_anchor is not None else "none"
        print(
            f"RX OK: TAG{tag_id},A{anchor_id},seq={seq},d={distance:.3f},avg={avg_distance:.3f},"
            f"bat_rx={battery_voltage:.3f}V/{int(round(battery_voltage*1000))}mV,{battery_percent}%"
            f"(rpi_calc,anchor_pct={raw_pct_text},reason={battery_reject_reason})"
        )
    elif tag_id in latest_battery:
        b = latest_battery[tag_id]
        print(
            f"RX OK: TAG{tag_id},A{anchor_id},seq={seq},d={distance:.3f},avg={avg_distance:.3f},"
            f"bat_rx=ignored({battery_voltage_raw},reason={battery_reject_reason}),"
            f"last={b['battery_voltage']:.3f}V/{int(round(b['battery_voltage']*1000))}mV,{b['battery_percent']}%,"
            f"src=A{b.get('source_anchor_id','?')},seq={b.get('source_seq','?')}"
        )
    else:
        print(
            f"RX OK: TAG{tag_id},A{anchor_id},seq={seq},d={distance:.3f},avg={avg_distance:.3f},"
            f"bat_rx=ignored({battery_voltage_raw},reason={battery_reject_reason}),last=none"
        )

    if tag_id in latest_position:
        p = latest_position[tag_id]
        print("=" * 60)
        print(f"TAG{tag_id} latest distances")

        for aid in [1, 2, 3, 4]:
            if aid in latest[tag_id]:
                item = latest[tag_id][aid]
                age = time.time() - item["received_unix"]
                print(f"A{aid}: {item['avg_distance']:.3f} m | seq={item['seq']} | age={age:.1f}s")
            else:
                print(f"A{aid}: no data")

        if p.get("available"):
            print(
                f"TAG{tag_id} position: "
                f"x={p['x']:.3f} m, y={p['y']:.3f} m, "
                f"x_clamped={p['x_clamped']:.3f} m, y_clamped={p['y_clamped']:.3f} m, "
                f"out_of_room={p.get('out_of_room', False)}"
            )
        else:
            print(f"TAG{tag_id} position unavailable: {p.get('reason')}")
        print("=" * 60)

    perf_log('receive_uwb TOTAL', _t0_route, f'TAG{tag_id},A{anchor_id},seq={seq}')
    return jsonify({"ok": True})


@app.route("/latest", methods=["GET"])
def get_latest():
    _t0_latest = perf_ms()
    """
    브라우저/HTML이 최신 거리값과 계산된 좌표를 확인하는 주소.
    TAG1, TAG2 모두 JSON으로 반환.
    """
    output = {}

    for tag_id in list(latest.keys()):
        compute_position_for_tag(tag_id)

        output[str(tag_id)] = {
            "anchors": {
                str(aid): {
                    **item,
                    "age_sec": round(time.time() - item["received_unix"], 3),
                }
                for aid, item in latest[tag_id].items()
            },
            "position": latest_position.get(tag_id, {"available": False}),
            "battery": latest_battery.get(tag_id),
        }

    perf_log('/latest TOTAL', _t0_latest)
    return jsonify(output)


@app.route("/latest-json", methods=["GET"])
def get_latest_json_payload():
    """
    latest.json과 같은 DynamoDB-style records 구조를 브라우저에서 바로 확인하는 주소.
    S3에 올릴 JSON 구조를 미리 확인할 때 사용.
    """
    payload = save_latest_json()
    return jsonify(payload)


@app.route("/", methods=["GET"])
def index():
    return render_template(
        "index.html",
        room_width=ROOM_WIDTH_M,
        room_height=ROOM_HEIGHT_M,
    )


if __name__ == "__main__":
    print("Starting UWB dual-tag receiver latest-position + battery mV/% + 0.125s API post v3")
    print("Open browser: http://<raspberry_pi_ip>:5000/")
    print("POST endpoint: http://<raspberry_pi_ip>:5000/uwb")
    app.run(host="0.0.0.0", port=5000, debug=False)
