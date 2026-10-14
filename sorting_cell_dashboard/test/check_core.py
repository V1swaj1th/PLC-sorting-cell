"""Run: PYTHONPATH=sorting_cell_dashboard python3 sorting_cell_dashboard/test/check_core.py"""
import json
import sys
import threading
import urllib.error
import urllib.request

from sorting_cell_dashboard.core import DashboardCore, _host_allowed, make_server

passed = failed = 0


def check(what, ok):
    global passed, failed
    print(f'  [{"PASS" if ok else "FAIL"}] {what}')
    passed, failed = (passed + 1, failed) if ok else (passed, failed + 1)


sent, starts = [], []
core = None


def fake_scene_server(text):  # replies on another thread, like a ROS callback would
    sent.append(text)
    rid, cmd = text.split('|', 1)
    if cmd == 'silent':
        return
    ok = cmd != 'bad'
    threading.Timer(0.05, core.on_reply,
                    args=(f'{{id: {rid}, ok: {str(ok).lower()}, message: "{cmd} done"}}',)).start()


core = DashboardCore(fake_scene_server, lambda: starts.append(1), timeout=0.5)
r = core.command('random   all')
check('command gets its own reply',
      r == {'ok': True, 'message': 'random all done'} and sent[-1].endswith('|random all'))
check('a refused command reports ok = false', core.command('bad')['ok'] is False)
r = core.command('silent')
check('no reply -> clear timeout message', r['ok'] is False and 'scene_server' in r['message'])
check('empty or piped commands are rejected locally',
      core.command('  ')['ok'] is False and core.command('a|b')['ok'] is False)
results = []
threads = [threading.Thread(target=lambda i=i: results.append(core.command(f'cmd{i}'))) for i in range(20)]
[t.start() for t in threads]
[t.join() for t in threads]
check('20 concurrent commands each get their own reply',
      sorted(x['message'] for x in results) == sorted(f'cmd{i} done' for i in range(20)))
core.on_reply('{id: 999, ok: true, message: stray}')
core.on_reply('not: [yaml')
check('stray or broken replies are ignored; the next command still works',
      core.command('after')['message'] == 'after done')

core.on_state('revision: 3\nseq: ready\nobjects: []\nreach: {rows: ["0111"]}\n')
snap = core.snapshot()
check('state is stored and versioned', snap['state']['revision'] == 3 and snap['version'] == 1)
check('quoted reach rows stay text', snap['state']['reach']['rows'][0] == '0111')
check('start is sent when ready', core.start()['ok'] and len(starts) == 1)
core.on_state('revision: 3\nseq: running\n')
check('start refused while running', core.start()['ok'] is False and len(starts) == 1)
plc = DashboardCore(fake_scene_server, lambda: starts.append(1), use_plc=True)
check('start refused when the PLC controls the cell', plc.start()['ok'] is False and len(starts) == 1)

for host, ok in [('localhost:8765', True), ('127.0.0.1:8765', True), ('[::1]:8765', True), ('172.17.0.2:8765', True),
                 ('evil.example:8765', False), ('localhost.evil.example', False), ('', False)]:
    check(f'Host {host!r} allowed={ok}', _host_allowed(host) == ok)

core.on_state('revision: 4\nseq: ready\n')
srv = make_server(core, '127.0.0.1', 0)
threading.Thread(target=srv.serve_forever, daemon=True).start()
base = f'http://127.0.0.1:{srv.server_address[1]}'
page = urllib.request.urlopen(base + '/').read().decode()
check('GET / serves the page', '<canvas id="cv"' in page)
check('GET /api/state serves JSON',
      json.loads(urllib.request.urlopen(base + '/api/state').read())['state']['revision'] == 4)
req = urllib.request.Request(base + '/api/command', data=b'{"cmd": "random objects"}', method='POST',
                             headers={'Content-Type': 'application/json'})
check('POST /api/command round trip',
      json.loads(urllib.request.urlopen(req).read())['message'] == 'random objects done')
try:
    urllib.request.urlopen(urllib.request.Request(base + '/api/command', data=b'x', method='POST',
                                                  headers={'Content-Type': 'text/plain'}))
    check('non-JSON POST refused', False)
except urllib.error.HTTPError as e:
    check('non-JSON POST refused', e.code == 415)
srv.shutdown()
print(f'\n{passed} passed, {failed} failed')
sys.exit(0 if failed == 0 else 1)
