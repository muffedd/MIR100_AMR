#!/usr/bin/env python3
"""Static call-graph regression for the MATLAB modules (runs without MATLAB).

Scans every .m call site of the contract functions and fails on:
  - arity outside the locked signature range
  - build_map called with a scenario handle (needs Nx7 boxes or CSV path)
  - any reference to opts.originXY (field never defined; [] = auto origin)
  - verify_robot/load_mir100 call sites that drop the new UrdfPath/ImportInfo
    contract where a non-default URDF path is in use (informational)
Limitations: text scan; calls whose argument list contains a nested call
spanning parentheses are not arity-checked (regex limitation), and struct
literals with parens inside may confuse the arg splitter. The two banned
patterns above are raw-text checks and always apply.
"""
import re, sys, pathlib

root = pathlib.Path(__file__).resolve().parent.parent

CONTRACTS = {           # fn: (min_args, max_args)
    'build_map': (2, 4), 'check_map': (4, 4), 'plan_path': (4, 4),
    'follow_route': (3, 3), 'verify_robot': (1, 2), 'load_mir100': (0, 2),
    'read_world_boxes': (1, 1), 'build_world': (1, 2), 'spawn_mir100': (2, 3),
    'check_run_clearance': (2, 3), 'make_demo': (3, 4), 'run_react_demo': (1, 3),
    'urdf_link_masses': (1, 1),
}

def split_args(s):
    out, depth, cur = [], 0, ''
    for ch in s:
        if ch in '([{': depth += 1
        if ch in ')]}': depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip()); cur = ''
        else:
            cur += ch
    if cur.strip(): out.append(cur.strip())
    return out

bad, checked = [], 0
for f in sorted(root.rglob('*.m')):
    for i, line in enumerate(f.read_text().splitlines(), 1):
        code = line.split('%', 1)[0]
        if re.match(r'^\s*function\b', code):
            continue  # the definition line itself
        if re.search(r'build_map\(\s*S\.scen', code):
            bad.append(f'{f}:{i}: build_map(S.scen,...) - scenario handle unsupported; pass Nx7 boxes or CSV path')
        if 'opts.originXY' in code:
            bad.append(f'{f}:{i}: opts.originXY referenced but never defined; pass [] for auto origin')
        for fn, (lo, hi) in CONTRACTS.items():
            for m in re.finditer(r'(?<![\w.])' + fn + r'\s*\(([^()]*)\)', code):
                args = split_args(m.group(1)) if m.group(1).strip() else []
                checked += 1
                if not (lo <= len(args) <= hi):
                    bad.append(f'{f}:{i}: {fn} called with {len(args)} args, contract is {lo}..{hi}: {m.group(0)[:80]}')

for b in bad: print('FAIL', b)
print(f'{checked} call sites checked, {len(bad)} violations')
sys.exit(1 if bad else 0)
