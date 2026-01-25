#!/usr/bin/env python3
import urllib.request
from urllib.error import URLError, HTTPError

BASE = 'http://100.76.149.200'

try:
    with urllib.request.urlopen(BASE + '/api/status', timeout=4) as resp:
        print(resp.read().decode())
except (HTTPError, URLError, OSError) as exc:
    print(f"error {exc}")
