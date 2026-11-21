"""Bounded HTTP test runner executed on robot; no direct motor publisher."""
import argparse
import json
import os
import signal
import time
import urllib.request


def request(path, body=None):
    payload = None if body is None else json.dumps(body).encode('utf-8')
    req = urllib.request.Request('http://127.0.0.1:5010'+path, data=payload,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=2.) as response:
        return json.loads(response.read().decode('utf-8'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    parser.add_argument('--seconds', type=float, default=60.)
    args = parser.parse_args()
    os.makedirs(args.directory, exist_ok=True)
    started = False
    reason = 'NOT_STARTED'
    states = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal '+str(signum))
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    path = os.path.join(args.directory, 'telemetry.jsonl')
    with open(path, 'w') as log:
        def record(kind, value):
            row = dict(kind=kind, wall_time=time.time(), monotonic=time.monotonic(), data=value)
            log.write(json.dumps(row, allow_nan=False)+'\n')
            log.flush()
        try:
            status = request('/status')
            record('before', status)
            if status.get('mode') != 'live' or status.get('target') != 'person':
                raise RuntimeError('Requires live mode and person target')
            request('/navigation', {'enabled': False})
            time.sleep(.7)
            status = request('/status')
            nav = status.get('navigation', {})
            for key in ('scan_age_s', 'odom_age_s', 'monitor_age_s'):
                if nav.get(key) is None or not 0 <= nav[key] < .5:
                    raise RuntimeError('Input not fresh: '+key)
            if nav.get('localization_error') or not nav.get('map_memory', {}).get('persistent'):
                raise RuntimeError('Requires valid localization and persistent map')
            if status.get('camera_age_s') is None or not -.2 <= status['camera_age_s'] < 1.:
                raise RuntimeError('Camera not fresh')
            record('map_before', request('/map'))
            # Set before POST so a lost response still enters the stop path.
            started = True
            record('start_request', request('/map_goal', {'mode': 'search'}))
            # HTTP acknowledges publication, not navigation's asynchronous acceptance.
            acceptance_deadline = time.monotonic()+3.
            while time.monotonic() < acceptance_deadline:
                time.sleep(.2)
                status = request('/status')
                record('acceptance', status)
                nav = status.get('navigation', {})
                if nav.get('enabled') and nav.get('goal_mode') == 'search':
                    break
            else:
                raise RuntimeError('Search not accepted: '+str(nav.get('goal_error')))
            deadline = time.monotonic()+min(60., max(1., args.seconds))
            next_map = time.monotonic()+3.
            last_state = None
            while time.monotonic() < deadline:
                time.sleep(.5)
                status = request('/status')
                record('status', status)
                nav = status.get('navigation', {})
                state = nav.get('state', 'UNKNOWN')
                states[state] = states.get(state, 0)+1
                if state != last_state:
                    print('State: '+state, flush=True)
                    last_state = state
                if state in ('FOUND', 'STUCK', 'SEARCH_COMPLETE'):
                    reason = state
                    break
                if not nav.get('enabled') or nav.get('goal_mode') != 'search':
                    reason = 'REQUEST_REJECTED_OR_PAUSED'
                    break
                if time.monotonic() >= next_map:
                    record('map', request('/map'))
                    next_map = time.monotonic()+3.
            else:
                reason = 'TIME_LIMIT'
        except BaseException as exc:
            reason = type(exc).__name__+': '+str(exc)
            record('error', reason)
        finally:
            stop_ok = False
            if started:
                for attempt in range(3):
                    try:
                        record('pause_response', request('/navigation', {'enabled': False}))
                        time.sleep(.6)
                        status = request('/status')
                        record('after', status)
                        nav = status.get('navigation', {})
                        stop_ok = (nav.get('enabled') is False and
                                   all(abs(v) < .001 for v in nav.get('monitor_velocity', [1.])))
                        if stop_ok:
                            break
                    except Exception as exc:
                        record('stop_error', str(exc))
            summary = dict(reason=reason, started=started, stop_confirmed=stop_ok, states=states)
            record('summary', summary)
            with open(os.path.join(args.directory, 'summary.json'), 'w') as output:
                json.dump(summary, output, indent=2)
            print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
