#!/usr/bin/env python3
"""MQTT -> Supabase bridge for the STM32 board.

The board publishes one JSON message per reading to a local Mosquitto broker
(topic drivemetrics/readings). This bridge subscribes, builds trips directly
in Supabase (append to a trip whose last point is within 30 min, else start a
new one), and writes route_points + harsh trip_events. It replaces both the
old `thingspeak-sync` and `build-trip` edge functions.

Message payload (ts is Unix epoch SECONDS):
    {"dev":"stm32-001","ts":1718086496,"pred":2,"conf":95.3,"lat":31.5156,
     "lon":74.4654,"spd":45.2,"gmax":2.156,"collision":0,"gpeak":0.0,"fix":1}

Setup:
    pip install paho-mqtt requests
    # start the broker (see tools/mosquitto.conf):
    mosquitto -c tools/mosquitto.conf -v
    # run the bridge:
    SUPABASE_SERVICE_KEY=<service_role_key> python3 tools/mqtt_bridge.py

Env vars:
    SUPABASE_URL          (default: the project URL below)
    SUPABASE_SERVICE_KEY  (required — service_role key; server-side only)
    MQTT_HOST             (default: localhost)
    MQTT_PORT             (default: 1883)
    MQTT_TOPIC            (default: drivemetrics/readings)
"""
import json
import math
import os
import sys
from datetime import datetime, timezone

import requests

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("Missing dependency: pip install paho-mqtt")

SUPABASE_URL = os.environ.get(
    "SUPABASE_URL", "https://gsgmyvdszgejcadgrvgs.supabase.co"
).rstrip("/")
SERVICE_KEY = os.environ.get("SUPABASE_SERVICE_KEY")
MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_TOPIC = os.environ.get("MQTT_TOPIC", "drivemetrics/readings")

TRIP_GAP_SEC = 30 * 60          # >30 min idle -> new trip
EPOCH_CUTOFF = 1577836800       # 2020-01-01; drop readings older than this (no GPS)

if not SERVICE_KEY:
    sys.exit("Set SUPABASE_SERVICE_KEY (the service_role key) in the environment.")

REST = SUPABASE_URL + "/rest/v1"
HEADERS = {
    "apikey": SERVICE_KEY,
    "Authorization": "Bearer " + SERVICE_KEY,
    "Content-Type": "application/json",
}
sess = requests.Session()
sess.headers.update(HEADERS)


# ── Supabase REST helpers ─────────────────────────────────────────────────────
def sb_get(table, params):
    r = sess.get(f"{REST}/{table}", params=params, timeout=20)
    r.raise_for_status()
    return r.json()


def sb_insert(table, row, return_rep=False):
    headers = {"Prefer": "resolution=ignore-duplicates"}
    if return_rep:
        headers["Prefer"] = "return=representation"
    r = sess.post(f"{REST}/{table}", json=row, headers=headers, timeout=20)
    r.raise_for_status()
    return r.json() if return_rep and r.content else None


def sb_patch(table, params, row):
    r = sess.patch(f"{REST}/{table}", params=params, json=row, timeout=20)
    r.raise_for_status()


def iso(ts_sec):
    return datetime.fromtimestamp(ts_sec, tz=timezone.utc).isoformat().replace(
        "+00:00", "Z"
    )


def haversine_km(a, b):
    R = 6371.0
    dlat = math.radians(b[0] - a[0])
    dlon = math.radians(b[1] - a[1])
    h = (
        math.sin(dlat / 2) ** 2
        + math.cos(math.radians(a[0])) * math.cos(math.radians(b[0]))
        * math.sin(dlon / 2) ** 2
    )
    return R * 2 * math.asin(math.sqrt(h))


# ── trip building ─────────────────────────────────────────────────────────────
def find_or_create_trip(dev, ts, recorded_at):
    """Return the id of a trip ending within 30 min of recorded_at, else a new one."""
    cutoff = iso(ts - TRIP_GAP_SEC)
    rows = sb_get(
        "trips",
        {
            "device_token": f"eq.{dev}",
            "end_time": f"gte.{cutoff}",
            "order": "end_time.desc",
            "limit": "1",
            "select": "id",
        },
    )
    if rows:
        return rows[0]["id"]
    created = sb_insert(
        "trips",
        {"device_token": dev, "start_time": recorded_at, "end_time": recorded_at},
        return_rep=True,
    )
    print(f"  + new trip {created[0]['id']}")
    return created[0]["id"]


def recompute_trip(trip_id, this_recorded_at):
    """Refresh end_time / speeds / distance from the trip's route points."""
    pts = sb_get(
        "route_points",
        {
            "trip_id": f"eq.{trip_id}",
            "order": "recorded_at.asc",
            "select": "latitude,longitude,speed_kmh,recorded_at",
        },
    )
    patch = {"end_time": this_recorded_at}
    if pts:
        patch["end_time"] = max(this_recorded_at, pts[-1]["recorded_at"])
        speeds = [p["speed_kmh"] for p in pts if p["speed_kmh"] is not None]
        if speeds:
            patch["max_speed_kmh"] = max(speeds)
            patch["avg_speed_kmh"] = sum(speeds) / len(speeds)
        dist = 0.0
        for i in range(1, len(pts)):
            dist += haversine_km(
                (pts[i - 1]["latitude"], pts[i - 1]["longitude"]),
                (pts[i]["latitude"], pts[i]["longitude"]),
            )
        patch["distance_km"] = round(dist, 3)
    sb_patch("trips", {"id": f"eq.{trip_id}"}, patch)


def handle(msg):
    m = json.loads(msg)
    ts = int(m["ts"])
    if ts < EPOCH_CUTOFF:
        print(f"  drop epoch reading ts={ts} (no GPS time)")
        return
    dev = m.get("dev", "unknown")
    recorded_at = iso(ts)

    trip_id = find_or_create_trip(dev, ts, recorded_at)

    lat, lon = m.get("lat", 0), m.get("lon", 0)
    if lat and lon:
        sb_insert(
            "route_points",
            {
                "trip_id": trip_id,
                "latitude": lat,
                "longitude": lon,
                "speed_kmh": m.get("spd"),
                "recorded_at": recorded_at,
            },
        )

    pred = int(m.get("pred", 0))
    if pred >= 2:  # harsh event (accel/turn/brake); 0=idle, 1=normal
        g = m.get("gpeak") if m.get("collision") else m.get("gmax")
        sb_insert(
            "trip_events",
            {
                "trip_id": trip_id,
                "event_label": pred,
                "confidence": m.get("conf"),
                "latitude": lat or None,
                "longitude": lon or None,
                "speed_kmh": m.get("spd"),
                "g_worst": g,
                "recorded_at": recorded_at,
            },
        )

    recompute_trip(trip_id, recorded_at)


# ── MQTT callbacks ────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc, *args):
    print(f"Connected to broker rc={rc}; subscribing to {MQTT_TOPIC}")
    client.subscribe(MQTT_TOPIC, qos=0)


def on_message(client, userdata, msg):
    try:
        print(f"{msg.topic}: {msg.payload.decode(errors='replace')[:120]}")
        handle(msg.payload.decode())
    except Exception as e:  # keep the bridge alive on a bad message
        print(f"  ERROR: {e}")


if __name__ == "__main__":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    print(f"Bridge: mqtt://{MQTT_HOST}:{MQTT_PORT}/{MQTT_TOPIC} -> {SUPABASE_URL}")
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_forever()
