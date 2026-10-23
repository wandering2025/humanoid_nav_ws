#!/usr/bin/env python3
"""Check the source snapshot, including documentation links and runtime hashes."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def public_files():
    tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode().split('\0')
    paths = [Path(p) for p in tracked if p]
    candidates = sorted({str(p) for path in paths for p in (path, *path.parents) if str(p) != '.'})
    raw = subprocess.check_output(['git', 'check-attr', '-z', '--stdin', 'export-ignore'],
                                  input=('\0'.join(candidates)+'\0').encode(), cwd=ROOT)
    values = raw.decode().rstrip('\0').split('\0')
    ignored = {Path(values[i]) for i in range(0, len(values), 3) if values[i+2] == 'set'}
    return {p for p in paths if not any(a in ignored for a in (p, *p.parents))}


def main():
    files = public_files()
    errors = []
    text_types = {'.md', '.py', '.sh', '.yaml', '.yml', '.json', '.txt', '.xml'}
    links_checked = 0
    for relative in sorted(files):
        path = ROOT / relative
        if not path.is_file():
            errors.append(f'Missing tracked file: {relative}')
            continue
        if path.suffix not in text_types:
            continue
        text = path.read_text()
        if re.search(r'[\u3400-\u9fff]', text):
            errors.append(f'Non-English text in public file: {relative}')
        if re.search(r'/home/[a-zA-Z0-9_.-]+/', text):
            errors.append(f'Host-specific home path: {relative}')
        if path.suffix == '.py':
            try:
                ast.parse(text, filename=str(relative))
            except SyntaxError as exc:
                errors.append(str(exc))
        if path.suffix == '.sh':
            result = subprocess.run(['bash', '-n', str(path)], capture_output=True, text=True)
            if result.returncode:
                errors.append(result.stderr)
        if path.suffix == '.md':
            targets = re.findall(r'!?\[[^\]]*\]\(([^\s)]+)(?:\s+[^)]*)?\)', text)
            targets += re.findall(r'(?:src|href)="([^"]+)"', text)
            for target in targets:
                uri = urlsplit(target)
                if uri.scheme or uri.netloc or not uri.path:
                    continue
                resolved = (path.parent / unquote(uri.path)).resolve()
                links_checked += 1
                if not resolved.is_relative_to(ROOT) or resolved.relative_to(ROOT) not in files:
                    errors.append(f'Link outside public snapshot: {relative} -> {target}')
    lock = json.loads((ROOT/'scripts/advanced/runtime-lock.json').read_text())
    for name, expected in lock['files'].items():
        if Path(name) not in files or hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            errors.append(f'Runtime contract mismatch: {name}')
    for name, evidence in json.loads((ROOT/'docs/validation/source-index.json').read_text())['media'].items():
        if hashlib.sha256((ROOT/'docs/media'/name).read_bytes()).hexdigest() != evidence['sha256']:
            errors.append(f'Media provenance mismatch: {name}')
    if errors:
        print('\n'.join(errors))
        return 1
    print(f'PASS: {len(files)} public files; {links_checked} local links; English text, syntax, runtime and media hashes.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
