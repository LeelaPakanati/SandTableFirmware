#!/usr/bin/env python3
import urllib.request
import urllib.parse
from urllib.error import URLError, HTTPError

BASE = 'http://100.76.149.200'

try:
    body = urllib.parse.urlencode({'brightness': '10'}).encode('utf-8')
    req = urllib.request.Request(BASE + '/api/led/brightness', data=body, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=4) as resp:
        print('set', resp.read().decode())

    with urllib.request.urlopen(BASE + '/api/led/brightness', timeout=4) as resp:
        print('get', resp.read().decode())
except (HTTPError, URLError, OSError) as exc:
    print(f"error {exc}")
