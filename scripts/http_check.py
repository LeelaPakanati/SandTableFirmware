#!/usr/bin/env python3
import json
import urllib.request
import urllib.parse

BASE = 'http://100.76.149.200'


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as resp:
        return resp.read().decode()


def post_form(path, data):
    body = urllib.parse.urlencode(data).encode('utf-8')
    req = urllib.request.Request(BASE + path, data=body, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=5) as resp:
        return resp.read().decode()


def main():
    print('status', get('/api/status'))
    print('start', post_form('/api/pattern/start', {'file': 'shell.thr', 'clearing': '0'}))
    print('status2', get('/api/status'))
    print('stop', post_form('/api/pattern/stop', {}))
    print('status3', get('/api/status'))


if __name__ == '__main__':
    main()
