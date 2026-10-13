"""ROS-free part of the dashboard: state store, command/reply matching and the HTTP server."""
import ipaddress
import itertools
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

from sorting_cell_dashboard.page import PAGE


class DashboardCore:
    """send_command(text) publishes on scene/command, send_start() on cell/start."""

    def __init__(self, send_command, send_start, use_plc=False, timeout=5.0):
        self._send_command = send_command
        self._send_start = send_start
        self.use_plc = use_plc
        self.timeout = timeout
        self._lock = threading.Lock()
        self._state = None
        self._phase = ''
        self._version = 0
        self._pending = {}
        self._ids = itertools.count(1)

    # ---- inputs from ROS
    def on_state(self, text):
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError:
            return
        if isinstance(data, dict):
            with self._lock:
                self._state = data
                self._version += 1

    def on_reply(self, text):
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError:
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            entry = self._pending.get(str(data.get('id', '')))
        if entry is not None:
            entry['result'] = {'ok': bool(data.get('ok')), 'message': str(data.get('message', ''))}
            entry['event'].set()

    def on_phase(self, text):
        with self._lock:
            if text != self._phase:
                self._phase = text
                self._version += 1

    # ---- actions from the browser
    def command(self, cmd):
        cmd = ' '.join(str(cmd).split())
        if not cmd or len(cmd) > 300 or '|' in cmd:
            return {'ok': False, 'message': 'invalid command'}
        rid = str(next(self._ids))
        entry = {'event': threading.Event(), 'result': None}
        with self._lock:
            self._pending[rid] = entry
        try:
            self._send_command(f'{rid}|{cmd}')
            if not entry['event'].wait(self.timeout):
                return {'ok': False, 'message': 'no reply from scene_server - is it running?'}
            return entry['result']
        finally:
            with self._lock:
                self._pending.pop(rid, None)

    def start(self):
        if self.use_plc:
            return {'ok': False, 'message': 'the PLC controls the start: use panel/start'}
        with self._lock:
            seq = (self._state or {}).get('seq', '')
        if seq == 'running':
            return {'ok': False, 'message': 'the arm is already moving'}
        if seq == 'fault':
            return {'ok': False, 'message': 'the sequencer could not plan this scene'}
        self._send_start()
        return {'ok': True, 'message': 'start sent'}

    def snapshot(self):
        with self._lock:
            return {'version': self._version, 'state': self._state, 'phase': self._phase,
                    'use_plc': self.use_plc}


def _host_allowed(host_header):
    """Only localhost or a literal IP address: stops other web sites reaching the page by DNS rebinding."""
    host = host_header.strip()
    if host.startswith('['):
        host = host[1:host.find(']')] if ']' in host else host
    elif host.count(':') == 1:
        host = host.split(':')[0]
    if host == 'localhost':
        return True
    try:
        ipaddress.ip_address(host)
        return True
    except ValueError:
        return False


def make_server(core, host, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype='application/json'):
            data = body.encode('utf-8') if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not _host_allowed(self.headers.get('Host', '')):
                return self._send(403, '{"error": "forbidden host"}')
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                return self._send(200, PAGE, 'text/html')
            if path == '/api/state':
                return self._send(200, json.dumps(core.snapshot()))
            return self._send(404, '{"error": "not found"}')

        def do_POST(self):
            if not _host_allowed(self.headers.get('Host', '')):
                return self._send(403, '{"error": "forbidden host"}')
            # A JSON content type cannot be sent cross-site without a CORS preflight, which this server never grants.
            if not self.headers.get('Content-Type', '').startswith('application/json'):
                return self._send(415, '{"error": "JSON only"}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                length = -1
            if length < 0 or length > 10000:
                return self._send(413, '{"error": "too large"}')
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                return self._send(400, '{"error": "bad JSON"}')
            path = self.path.split('?')[0]
            if path == '/api/command' and isinstance(body, dict):
                return self._send(200, json.dumps(core.command(body.get('cmd', ''))))
            if path == '/api/start':
                return self._send(200, json.dumps(core.start()))
            return self._send(404, '{"error": "not found"}')

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
