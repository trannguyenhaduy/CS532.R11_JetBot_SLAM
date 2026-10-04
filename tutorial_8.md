## Overview

The robot lab has a **Hikvision PTZ camera** (pan / tilt / zoom) mounted on the ceiling at
`192.168.16.62`. It is used both as the JetBot fleet's overhead navigation view **and** as a
data-collection tool: you can drive it to any pan/tilt/zoom via the ISAPI HTTP API and pull
frames from its RTSP stream. This guide shows how to control it and grab images.

> ⚠️ **Shared device.** This camera is also the robot fleet's overhead nav camera. Always
> restore the calibrated **home pose** (azimuth 530, elevation 200, zoom 19) when you finish,
> or robot navigation and auto-docking break.

## Connection

| | |
|---|---|
| IP | `192.168.16.62` |
| Auth | HTTP **digest**, `admin` / `<PASSWORD>` |
| Control API | ISAPI, base `http://192.168.16.62/ISAPI/PTZCtrl/channels/1` |
| Video | RTSP `rtsp://admin:<PASSWORD>@192.168.16.62:554/Streaming/Channels/101` (1920×1080) |

Use `curl --digest` (or Python `requests` with `HTTPDigestAuth`) — plain basic auth is rejected.

Credentials are **not shown here** (this guide is public) — get the camera password from the lab admin. It is also stored privately in the AgentNet capability entry.


## Read the current position

```bash
curl -s --digest -u admin:<PASSWORD> \
  http://192.168.16.62/ISAPI/PTZCtrl/channels/1/status
```

Returns `<elevation>`, `<azimuth>`, `<absoluteZoom>` — all in **tenths** (azimuth 0–3600 = 0–360°,
elevation 0–900, zoom ×10 so `45` = 4.5×).

## Move to an absolute position

```bash
curl -s --digest -u admin:<PASSWORD> -X PUT \
  -H 'Content-Type: application/xml' \
  -d '<PTZData><AbsoluteHigh><elevation>120</elevation><azimuth>513</azimuth><absoluteZoom>45</absoluteZoom></AbsoluteHigh></PTZData>' \
  http://192.168.16.62/ISAPI/PTZCtrl/channels/1/absolute
```

For joystick-style motion use `PUT /continuous` with `<PTZData><pan>-100..100</pan>
<tilt>…</tilt><zoom>…</zoom></PTZData>` and send zeros to stop.

## Grab a frame

```bash
ffmpeg -y -rtsp_transport tcp \
  -i 'rtsp://admin:<PASSWORD>@192.168.16.62:554/Streaming/Channels/101' \
  -frames:v 1 -q:v 2 frame.jpg
```

Focus is set to **AUTO** so zoomed frames stay sharp; after a big zoom change wait ~4 s for the
move + refocus before grabbing.

## Framing tip (important)

It's a **ceiling** mount, so a *high* elevation value tilts the lens **down onto the near floor**
and you lose the people/robots across the room. To see the room, keep **elevation low (~110–140)**
and use **zoom** to reach further in. High-elevation + high-zoom = a close-up of the floor.

## Python helper

```python
import subprocess
CAM, AUTH = "192.168.16.62", "admin:<PASSWORD>"
ISAPI = f"http://{CAM}/ISAPI/PTZCtrl/channels/1"

def ptz_goto(az, el, zoom):
    body = (f"<PTZData><AbsoluteHigh><elevation>{el}</elevation>"
            f"<azimuth>{az}</azimuth><absoluteZoom>{zoom}</absoluteZoom></AbsoluteHigh></PTZData>")
    subprocess.run(["curl","-s","--digest","-u",AUTH,"-X","PUT",
        "-H","Content-Type: application/xml","-d",body, f"{ISAPI}/absolute"])

def grab(path):
    subprocess.run(["ffmpeg","-y","-rtsp_transport","tcp",
        "-i", f"rtsp://{AUTH}@{CAM}:554/Streaming/Channels/101","-frames:v","1", path])

ptz_goto(530, 200, 19)   # <- always end at home
```

## Worked example: person-counting dataset

A collector (`person_count/collect_ptz.py`) drives the camera through a set of presets — wide
whole-room, mid group, and low-elevation tele close-ups — grabbing several frames at each so people
appear at varied angles and scales, writes a `manifest.csv` (filename + az/el/zoom + timestamp), and
restores home on exit. Re-run it at different times of day for occupancy variety.

## See also

The camera-control recipe is also stored in **AgentNet** (capability *"Control the lab PTZ camera
(Hikvision ISAPI)"*) for agents to reuse. The JetBot fleet server additionally proxies it at
`POST /api/cameras/room/ptz`.
