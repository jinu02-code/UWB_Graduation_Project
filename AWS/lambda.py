import json
import os
import time
from decimal import Decimal
from datetime import datetime, timezone

import boto3

TABLE_NAME = os.environ.get("TABLE_NAME", "project1-02-ohio-uwb_position")
dynamodb = boto3.resource("dynamodb")
table = dynamodb.Table(TABLE_NAME)

def perf_ms():
    return time.perf_counter() * 1000.0

def log_perf(stage, t0=None, extra=""):
    now = perf_ms()
    if t0 is None:
        print(f"[PERF] {stage} at={now:.3f}ms {extra}")
    else:
        print(f"[PERF] {stage} elapsed={now - t0:.3f}ms {extra}")


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def response(status_code, body):
    return {
        "statusCode": status_code,
        "headers": {
            "Content-Type": "application/json",
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Headers": "Content-Type",
            "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
        },
        "body": json.dumps(body, ensure_ascii=False, default=str),
    }


def to_decimal(obj):
    if isinstance(obj, float):
        return Decimal(str(round(obj, 6)))
    if isinstance(obj, dict):
        return {k: to_decimal(v) for k, v in obj.items() if v is not None}
    if isinstance(obj, list):
        return [to_decimal(v) for v in obj]
    return obj


def save_position(event):
    _t0 = perf_ms()
    body = event.get("body") or "{}"
    _t_parse = perf_ms()
    data = json.loads(body, parse_float=Decimal)
    log_perf("Lambda json.loads", _t_parse, f"body_len={len(body)}")

    records = data.get("records", [])
    save_history = bool(data.get("save_history", False))
    updated_at = now_iso()

    # Raspberry Pi가 보낸 방 크기/앵커 좌표를 별도 LATEST 항목으로 저장
    # 같은 DynamoDB 테이블을 그대로 쓰고, tag_id="ROOM" / timestamp="LATEST"로 저장한다.
    room = data.get("room")
    if room:
        room_item = {
            "tag_id": "ROOM",
            "timestamp": "LATEST",
            "updated_at": updated_at,
            "device_id": data.get("device_id", "raspberrypi_uwb_01"),
            "room": room,
        }
        _t_put_room = perf_ms()
        table.put_item(Item=to_decimal(room_item))
        log_perf("DynamoDB put_item ROOM", _t_put_room)

    saved = []

    for record in records:
        if not record.get("available", False):
            continue

        tag_id = str(record.get("tag_id", "")).upper()
        if not tag_id:
            continue

        item = {
            "tag_id": tag_id,
            "timestamp": "LATEST",
            "updated_at": updated_at,
            "device_id": data.get("device_id", "raspberrypi_uwb_01"),
            "available": True,
            "x": record.get("x"),
            "y": record.get("y"),
            "x_clamped": record.get("x_clamped", record.get("x")),
            "y_clamped": record.get("y_clamped", record.get("y")),
            "out_of_room": record.get("out_of_room", False),
            "battery_voltage": record.get("battery_voltage"),
            "battery_percent": record.get("battery_percent"),
            "battery_updated_at": record.get("battery_updated_at"),
            "fresh_anchor_count": record.get("fresh_anchor_count", 0),
            "anchor_distances": record.get("anchor_distances", {}),
            "anchors": record.get("anchors", {}),
            "room_width_m": record.get("room_width_m"),
            "room_height_m": record.get("room_height_m"),
        }

        # fallback용으로 각 TAG LATEST에도 room을 같이 넣어둔다.
        if room:
            item["room"] = room

        _t_put_latest = perf_ms()
        table.put_item(Item=to_decimal(item))
        log_perf('DynamoDB put_item LATEST', _t_put_latest, tag_id)

        if save_history:
            history_item = dict(item)
            history_item["timestamp"] = updated_at
            _t_put_hist = perf_ms()
            table.put_item(Item=to_decimal(history_item))
            log_perf('DynamoDB put_item HISTORY', _t_put_hist, tag_id)

        saved.append(tag_id)

    log_perf('save_position TOTAL', _t0, f'saved={saved}')
    return response(200, {"ok": True, "saved": saved, "room_saved": bool(room)})


def get_latest():
    _t0 = perf_ms()
    records = []
    room = None

    # 방 크기/앵커 좌표 최신값 읽기
    _t_get_room = perf_ms()
    room_result = table.get_item(
        Key={
            "tag_id": "ROOM",
            "timestamp": "LATEST",
        }
    )
    log_perf("DynamoDB get_item ROOM", _t_get_room)

    room_item = room_result.get("Item")
    if room_item:
        room = room_item.get("room")

    for tag_id in ["TAG1", "TAG2"]:
        _t_get = perf_ms()
        result = table.get_item(
            Key={
                "tag_id": tag_id,
                "timestamp": "LATEST",
            }
        )
        log_perf("DynamoDB get_item LATEST", _t_get, tag_id)

        item = result.get("Item")
        if item:
            # ROOM 항목이 아직 없을 때를 대비한 fallback
            if room is None and item.get("room"):
                room = item.get("room")
            records.append(item)

    log_perf('get_latest TOTAL', _t0, f'count={len(records)}')
    return response(200, {"room": room, "records": records})


def get_history(event):
    _t0 = perf_ms()
    params = event.get("queryStringParameters") or {}
    tag_id = params.get("tag_id", "TAG1").upper()
    limit = int(params.get("limit", "50"))

    _t_query = perf_ms()
    result = table.query(
        KeyConditionExpression="tag_id = :tag_id",
        ExpressionAttributeValues={
            ":tag_id": tag_id,
        },
        ScanIndexForward=False,
        Limit=limit,
    )
    log_perf("DynamoDB query HISTORY", _t_query, f"{tag_id} limit={limit}")

    items = [
        item for item in result.get("Items", [])
        if item.get("timestamp") != "LATEST"
    ]

    log_perf('get_history TOTAL', _t0, f'{tag_id} count={len(items)}')
    return response(200, {"tag_id": tag_id, "records": items})


def lambda_handler(event, context):
    _t0 = perf_ms()
    method = event.get("requestContext", {}).get("http", {}).get("method", "")
    path = event.get("rawPath", "")

    if method == "OPTIONS":
        return response(200, {"ok": True})

    if method == "POST" and path.endswith("/position"):
        return save_position(event)

    if method == "GET" and path.endswith("/latest"):
        return get_latest()

    if method == "GET" and path.endswith("/history"):
        return get_history(event)

    return response(404, {"ok": False, "error": "not found", "path": path, "method": method})
