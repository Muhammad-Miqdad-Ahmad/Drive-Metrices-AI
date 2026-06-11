#!/usr/bin/env python3
"""HTTP -> HTTPS relay for the STM32 board.

The board's WiFi module (ISM43362) cannot send TLS SNI, which Supabase
(behind Cloudflare) requires. This script accepts the board's plain-HTTP
requests on the local network and forwards them to Supabase over HTTPS.

Usage:
    pip install requests
    python3 relay.py

Then set RELAY_IP in Core/Inc/net_config.h to this machine's LAN IP
(find it with `ip addr` on Linux or `ipconfig` on Windows).
"""
import http.server

import requests

SUPABASE = "https://gsgmyvdszgejcadgrvgs.supabase.co"
PORT = 8787

FORWARD_HEADERS = {"apikey", "authorization", "content-type", "prefer"}


class Relay(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _forward(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b""
        headers = {k: v for k, v in self.headers.items()
                   if k.lower() in FORWARD_HEADERS}
        try:
            r = requests.request(self.command, SUPABASE + self.path,
                                 data=body, headers=headers, timeout=20)
            status, content = r.status_code, r.content
        except requests.RequestException as e:
            status, content = 502, str(e).encode()

        print(f"{self.command} {self.path} ({length} B) -> {status}")
        if status >= 300 and content:
            print(f"  response: {content[:300].decode(errors='replace')}")

        self.send_response(status)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(content)

    do_POST = _forward
    do_PATCH = _forward
    do_GET = _forward

    def log_message(self, *args):  # silence default per-request logging
        pass


if __name__ == "__main__":
    print(f"Relaying 0.0.0.0:{PORT} -> {SUPABASE}")
    http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Relay).serve_forever()
