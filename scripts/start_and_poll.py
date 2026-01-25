#!/usr/bin/env python3
import json
import time
import urllib.request
import urllib.parse
from urllib.error import URLError, HTTPError

BASE = 'http://100.76.149.200'
FILE = 'shell.thr'

def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as resp:
        return resp.read().decode()


def post_form(path, data):
    body = urllib.parse.urlencode(data).encode('utf-8')
    req = urllib.request.Request(BASE + path, data=body, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=5) as resp:
        return resp.read().decode()


def safe(label, func):
    try:
        return func()
    except (HTTPError, URLError, OSError, ValueError) as exc:
        print(f"{label} error {exc}")
        return None


def main():
    status = safe('status', lambda: get('/api/status'))
    if status:
        print('status', status)

    stop = safe('stop', lambda: post_form('/api/pattern/stop', {}))
    if stop:
        print('stop', stop)

    time.sleep(0.3)

    start = safe('start', lambda: post_form('/api/pattern/start', {'file': FILE, 'clearing': '0'}))
    if start:
        print('start', start)

    last = None
    for i in range(6):
        status = safe('status', lambda: get('/api/status'))
        if status:
            print('status', status)
        pos = safe('position', lambda: get('/api/position'))
        if pos:
            try:
                data = json.loads(pos)
                cur = data.get('current')
                if not cur:
                    print('position none')
                else:
                    key = (round(cur['x'], 4), round(cur['y'], 4))
                    moved = last is not None and key != last
                    print('position', key, 'moved' if moved else 'same')
                    last = key
            except json.JSONDecodeError:
                print('position parse error')
        time.sleep(0.5)


if __name__ == '__main__':
    main()
