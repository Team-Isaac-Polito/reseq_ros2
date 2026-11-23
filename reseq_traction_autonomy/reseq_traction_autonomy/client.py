"""Loopback task API client and command-line entry point."""

import argparse
import json
import os
import urllib.error
import urllib.request

URL = os.environ.get('RESEQ_TRACTION_URL', 'http://127.0.0.1:8765')


def call(tool, arguments=None):
    request = urllib.request.Request(
        URL.rstrip('/') + '/tools/' + tool,
        method='POST',
        data=json.dumps(arguments or {}, allow_nan=False).encode(),
        headers={'Content-Type': 'application/json'},
    )
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(exc.read().decode()) from exc
    if 'error' in result:
        raise RuntimeError(result['error'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tool')
    parser.add_argument('arguments', nargs='?', default='{}')
    args = parser.parse_args()
    print(json.dumps(call(args.tool, json.loads(args.arguments)), indent=2))


if __name__ == '__main__':
    main()
