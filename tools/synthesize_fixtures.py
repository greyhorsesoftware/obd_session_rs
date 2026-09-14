#!/usr/bin/env python3
"""Replace captured-vehicle identifiers in mock_data/*.json with synthetic ones
(open-source pre-flight). Works INSIDE the frames, byte for byte, per ECU, so
every fixture keeps its exact ISO-TP structure, ECU interleaving and padding:

  * 0902 (VIN): the 17 chars → a synthetic VIN that keeps the WMI
    (positions 1–3) and the model-year char (position 10), so WMI routing and
    year_char in the goldens don't change.
  * 0904 (calibration ids): each printable run per ECU → "MOCKCAL<ecu><n>…",
    same length, same padding. A run that packs N 16-char ids back to back
    (count byte N) is split into N ids first.
  * vehicle_info.vin / calibration ids, when the file carries that block.
  * Every other reply in every mode is decoded per ECU; if the real VIN shows up
    anywhere else it is replaced there too. The whole file is then re-checked.

Idempotent: a file already synthetic is left alone.
"""
import json, glob, re, sys, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAMES = {"default": "DEFAULT", "bmw428i": "BMW428S", "gladiator": "GLADATR", "mustang50": "MSTNG50", "rangerover": "RNGRVR1"}
VIN_RE = re.compile(r"[A-Z0-9]{17}")   # loose on purpose: must also find already-synthetic VINs

def split(line):
    """(header tokens, pci tokens, data tokens) for one frame line."""
    t = line.split()
    h = 4 if t[0] == "18" else 1
    hdr, rest = t[:h], t[h:]
    pci = 2 if rest[0].startswith("1") else 1      # first frame carries a 2-byte PCI
    return hdr, rest[:pci], rest[pci:]

def is_frames(v):
    return isinstance(v, list) and v and all(isinstance(x, str) and " " in x for x in v)

def ecus(lines):
    """{header: [indexes into lines]} in file order."""
    g = {}
    for i, l in enumerate(lines):
        g.setdefault(" ".join(split(l)[0]), []).append(i)
    return g

def decode(lines, idxs):
    out = []
    for i in idxs:
        _, _, data = split(lines[i])
        out += [chr(int(b, 16)) if 32 <= int(b, 16) < 127 else "\x00" for b in data]
    return "".join(out)

def substitute(lines, idxs, old, new):
    """Replace ASCII run `old` with `new` (same length) inside one ECU's frames. Returns hit count."""
    assert len(old) == len(new), (old, new)
    if old == new:
        return 0
    text = decode(lines, idxs)
    at = text.find(old)
    if at < 0:
        return 0
    k = 0
    for i in idxs:
        hdr, pci, data = split(lines[i])
        nd = []
        for b in data:
            nd.append(f"{ord(new[k - at]):02X}" if at <= k < at + len(old) else b)
            k += 1
        lines[i] = " ".join(hdr + pci + nd)
    return 1 + substitute(lines, idxs, old, new)   # repeat if the run occurs again

def synth_vin(real, name):
    tail = NAMES.get(name, name.upper())[:7].ljust(7, "0")
    return real[:3] + "SYNTH0" + real[9] + tail

def raw(lines, idxs):
    return [int(b, 16) for i in idxs for b in split(lines[i])[2]]

def cal_runs(lines, idxs):
    """Printable runs after the `49 04 NN` prefix, packed 16-char ids split apart."""
    text, count = decode(lines, idxs), raw(lines, idxs)[2]
    runs = re.findall(r"[ -~]{4,}", text[3:])
    out = []
    for r in runs:
        if count > 1 and len(r) == 16 * count:
            out += [r[i:i + 16] for i in range(0, len(r), 16)]
        else:
            out.append(r)
    return out

def main():
    for f in sorted(glob.glob(os.path.join(ROOT, "mock_data", "*.json"))):
        name = os.path.basename(f)[:-5]
        orig = open(f).read()
        d = json.loads(orig)
        before = {(m, p): list(v) for m, t in d.items() if isinstance(t, dict) for p, v in t.items() if is_frames(v)}
        n9 = d.get("09", {})
        vin_lines = n9.get("02") or []
        real = None
        if vin_lines:
            for hdr, idxs in ecus(vin_lines).items():
                m = VIN_RE.search(decode(vin_lines, idxs))
                if m: real = m.group(0); break
        if not real:
            print(f"{name:<12} no VIN — skipped"); continue
        vin_done = "SYNTH" in real or "MOCK" in real
        new = real if vin_done else synth_vin(real, name)
        assert vin_done or (len(new) == 17 and not re.search(r"[IOQ]", new[3:])), new

        # 1. calibration ids in 0904, per ECU
        repl, real_cals = {}, []
        if n9.get("04"):
            lines = n9["04"]
            for e, (hdr, idxs) in enumerate(ecus(lines).items()):
                for j, r in enumerate(cal_runs(lines, idxs)):
                    if r in repl or r.startswith("MOCKCAL"): continue   # packed twice / already done
                    synth = f"MOCKCAL{e}{j}".ljust(len(r), "0")[:len(r)]
                    repl[r] = synth; real_cals.append(r)
                    assert substitute(lines, idxs, r, synth) >= 1, (name, hdr, r)

        # 2. the VIN: 0902 plus anywhere else it shows up (per ECU)
        hits = {}
        for mode, table in d.items():
            if not isinstance(table, dict): continue
            for pid, lines in table.items():
                if not is_frames(lines): continue
                for hdr, idxs in ecus(lines).items():
                    n = substitute(lines, idxs, real, new)
                    if n: hits[f"{mode}{pid}@{hdr}"] = n
        assert vin_done or any(k.startswith("0902") for k in hits), (name, hits)
        if vin_done and not repl:
            print(f"{name:<12} already synthetic — skipped"); continue

        # 3. vehicle_info block
        vi = d.get("vehicle_info")
        if isinstance(vi, dict):
            for k, v in list(vi.items()):
                if v == real: vi[k] = new
                elif isinstance(v, list): vi[k] = [repl.get(x, x) if isinstance(x, str) else x for x in v]
                elif isinstance(v, dict): vi[k] = {kk: repl.get(vv, vv) if isinstance(vv, str) else vv for kk, vv in v.items()}
        d.setdefault("_provenance", "captured-vehicle fixture; VIN and calibration ids replaced with synthetic values by "
                                    "tools/synthesize_fixtures.py (WMI + model-year char kept)")

        # 4. verify nothing real survives, in any ECU's decoded stream or in plain text
        for mode, table in d.items():
            if isinstance(table, dict):
                for pid, lines in table.items():
                    if is_frames(lines):
                        for hdr, idxs in ecus(lines).items():
                            t = decode(lines, idxs)
                            assert vin_done or real not in t, (name, mode, pid, hdr)
                            for r in real_cals: assert r not in t, (name, mode, pid, hdr, r)
        blob = json.dumps(d)
        assert vin_done or real not in blob
        assert not any(r in blob for r in real_cals)
        # 5. patch the original text line by line (keeps the file's own formatting)
        patched = orig
        for (m, p), old in before.items():
            for o, n in zip(old, d[m][p]):
                if o != n: patched = patched.replace(f'"{o}"', f'"{n}"')
        patched = patched.replace(f'"{real}"', f'"{new}"')
        for r, synth in repl.items(): patched = patched.replace(f'"{r}"', f'"{synth}"')
        if '"_provenance"' not in orig:
            head, brace, rest = patched.partition("{")
            patched = f'{head}{{\n  "_provenance" : {json.dumps(d["_provenance"])},{rest}'
        assert json.loads(patched) == d, f"{name}: textual patch drifted from the computed fixture"
        open(f, "w").write(patched)
        print(f"{name:<12} {real} → {new} · cal ids: {len(real_cals)} · VIN hits: {hits}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
