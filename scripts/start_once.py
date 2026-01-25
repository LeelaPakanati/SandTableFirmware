#!/usr/bin/env python3
import urllib.request
import urllib.parse
from urllib.error import URLError, HTTPError

BASE = 'http://100.76.149.200'
FILE = 'shell.thr'

def get(path):
    with urllib.request.urlopen(BASE + path, timeout=4) as resp:
        return resp.read().decode()


def post_form(path, data):
    body = urllib.parse.urlencode(data).encode('utf-8')
    req = urllib.request.Request(BASE + path, data=body, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=4) as resp:
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

    status2 = safe('status2', lambda: get('/api/status'))
    if status2:
        print('status2', status2)

    files = safe('files', lambda: get('/api/files'))
    if files:
        print('files', files)

    start = safe('start', lambda: post_form('/api/pattern/start', {'file': FILE, 'clearing': '0'}))
    if start:
        print('start', start)

    status3 = safe('status3', lambda: get('/api/status'))
    if status3:
        print('status3', status3)


if __name__ == '__main__':
    main()
