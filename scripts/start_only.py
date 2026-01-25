#!/usr/bin/env python3
import urllib.request
import urllib.parse
from urllib.error import URLError, HTTPError

BASE = 'http://100.76.149.200'
FILE = 'shell.thr'

try:
    body = urllib.parse.urlencode({'file': FILE, 'clearing': '0'}).encode('utf-8')
    req = urllib.request.Request(BASE + '/api/pattern/start', data=body, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=4) as resp:
        print(resp.read().decode())
except (HTTPError, URLError, OSError) as exc:
    print(f"error {exc}")
