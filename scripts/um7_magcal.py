#!/usr/bin/env python3
"""UM7 magnetometer calibration (hard + soft iron, 2D) from the Pi's command line.

Stop the rover service first (this talks to the UM7 directly on its port):

    sudo systemctl stop ipex-rover
    python3 um7_magcal.py info                 check the UM7 answers; print its current calibration
    python3 um7_magcal.py calibrate            record while you spin the rover, fit, write (asks first)
    python3 um7_magcal.py verify               spin again; check the calibrated field is a centred circle
    python3 um7_magcal.py restore <result.json>   put back the calibration saved before a write
    sudo systemctl start ipex-rover

calibrate: turn the rover slowly and level through at least one full turn (two is better,
~30-60 s each), by hand is fine. It records the raw magnetometer, fits an ellipse to x/y,
and computes the offset (hard iron) and the matrix (soft iron) that make it a centred circle.
Before writing it checks how this UM7 applies its calibration (C*(raw-b) or C*raw-b) against
the processed data it recorded, and refuses if neither matches. The old values are saved, the
new ones are written and read back, and only then (after a second yes) committed to flash.
Z is left as it was (2D: the rover stays level). Files: ~/roxey_data/imu_cal/<date_time>/.

UM7 protocol: packets 's' 'n' 'p' <type> <address> [data, 4 bytes per register, big-endian]
<16-bit checksum = sum of all previous bytes, big-endian>. type bit7 = has data, bit6 = batch,
bits 5..2 = batch length. Registers: CREG_MAG_CAL1_1..3_3 = 15..23 (float, row-major),
CREG_MAG_BIAS_X..Z = 24..26 (float), DREG_MAG_RAW_XY/Z = 92/93 (int16 x|y, z|-),
DREG_MAG_PROC_X..Z = 105..107 (float), GET_FW_VERSION = 170, FLASH_COMMIT = 171.
"""
import argparse
import datetime
import json
import os
import select
import struct
import subprocess
import sys
import termios
import time
import tty

import numpy as np

PORT = '/dev/ttyAMA0'
HAS_DATA, IS_BATCH = 0x80, 0x40
REG_CAL, REG_BIAS, REG_RAW, REG_PROC = 15, 24, 92, 105
CMD_FW_VERSION, CMD_FLASH_COMMIT = 170, 171
ERRORS = {253: 'bad checksum', 254: 'unknown address', 255: 'invalid batch size'}
OUT_ROOT = os.path.expanduser('~/roxey_data/imu_cal')


# ----------------------------------------------------------------- serial + protocol

class UM7:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = b''

    def close(self):
        os.close(self.fd)

    def _send(self, address, data=b''):
        n = len(data) // 4
        ptype = (HAS_DATA if data else 0) | ((IS_BATCH | (n << 2)) if n > 1 else 0)
        pkt = b'snp' + bytes([ptype, address]) + data
        os.write(self.fd, pkt + struct.pack('>H', sum(pkt) & 0xFFFF))

    def _packet(self, timeout):
        """Next valid packet (address, type, data) or None on timeout."""
        end = time.time() + timeout
        while True:
            i = self.buf.find(b'snp')
            if i >= 0 and len(self.buf) >= i + 5:
                ptype, addr = self.buf[i + 3], self.buf[i + 4]
                n = ((ptype >> 2) & 0x0F if ptype & IS_BATCH else 1) if ptype & HAS_DATA else 0
                total = 5 + 4 * n + 2
                if len(self.buf) >= i + total:
                    pkt = self.buf[i:i + total]
                    self.buf = self.buf[i + total:]
                    if sum(pkt[:-2]) & 0xFFFF == struct.unpack('>H', pkt[-2:])[0]:
                        return addr, ptype, pkt[5:-2]
                    continue
            elif i < 0:
                self.buf = self.buf[-2:]
            left = end - time.time()
            if left <= 0:
                return None
            if select.select([self.fd], [], [], left)[0]:
                self.buf += os.read(self.fd, 4096)

    def _wait(self, address, want_data, timeout=0.5):
        end = time.time() + timeout
        while time.time() < end:
            p = self._packet(end - time.time())
            if p is None:
                return None
            addr, ptype, data = p
            if addr in ERRORS:
                raise RuntimeError(f'UM7 replied: {ERRORS[addr]}')
            if addr == address and (bool(data) == want_data):
                return data
        return None

    def read(self, address, n, tries=5):
        for _ in range(tries):
            ptype = (IS_BATCH | (n << 2)) if n > 1 else 0
            pkt = b'snp' + bytes([ptype, address])
            os.write(self.fd, pkt + struct.pack('>H', sum(pkt) & 0xFFFF))
            data = self._wait(address, True)
            if data and len(data) == 4 * n:
                return data
        raise RuntimeError(f'no reply reading register {address} (is the UM7 connected / service stopped?)')

    def write(self, address, data, tries=5):
        for _ in range(tries):
            self._send(address, data)
            if self._wait(address, False) is not None:
                return
        raise RuntimeError(f'no acknowledgement writing register {address}')

    def command(self, address, want_data=False, tries=5):
        for _ in range(tries):
            self._send(address)
            data = self._wait(address, want_data, timeout=1.0)
            if data is not None:
                return data
        raise RuntimeError(f'no reply to command {address}')

    # -- registers
    def calibration(self):
        vals = struct.unpack('>12f', self.read(REG_CAL, 12))
        return np.array(vals[:9]).reshape(3, 3), np.array(vals[9:])

    def set_calibration(self, C, b):
        self.write(REG_CAL, struct.pack('>12f', *np.asarray(C, float).ravel(), *np.asarray(b, float)))

    def raw_mag(self):
        x, y, z, _ = struct.unpack('>4h', self.read(REG_RAW, 2))
        return np.array([x, y, z], float)

    def proc_mag(self):
        return np.array(struct.unpack('>3f', self.read(REG_PROC, 3)))


# ----------------------------------------------------------------- recording + fit

def record(um7, seconds, label):
    print(f'\nRecording {label}. Turn the rover slowly and level, at least one full turn '
          f'(two is better). Ctrl+C when done (max {seconds} s).\n')
    t0 = time.time()
    rows, bins = [], set()
    try:
        while time.time() - t0 < seconds:
            raw, proc = um7.raw_mag(), um7.proc_mag()
            rows.append([time.time() - t0, *raw, *proc])
            xy = np.array([r[1:3] for r in rows])
            c = (xy.max(axis=0) + xy.min(axis=0)) / 2
            bins.add(int(np.degrees(np.arctan2(raw[1] - c[1], raw[0] - c[0])) // 10) % 36)
            sys.stdout.write(f'\r  {time.time() - t0:5.1f} s   {len(rows):4d} samples   '
                             f'turn coverage {10 * len(bins):3d}/360 deg   ')
            sys.stdout.flush()
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    print()
    a = np.array(rows)
    if 10 * len(bins) < 330:
        print(f'  WARNING: only {10 * len(bins)} of 360 deg covered; the fit needs a full turn.')
    return a


def fit_ellipse(x, y):
    """Least-squares conic fit. Returns centre c, matrix Q with (p-c)^T Q (p-c) = 1."""
    xm, ym, s = x.mean(), y.mean(), max(x.std(), y.std())
    u, v = (x - xm) / s, (y - ym) / s
    D = np.column_stack([u * u, u * v, v * v, u, v, np.ones_like(u)])
    A, B, Cc, Dd, E, F = np.linalg.svd(D, full_matrices=False)[2][-1]
    if B * B - 4 * A * Cc >= 0:
        raise RuntimeError('points do not form an ellipse (spin a full, level turn)')
    M = np.array([[A, B / 2], [B / 2, Cc]])
    c = -0.5 * np.linalg.solve(M, [Dd, E])
    k = c @ M @ c - F
    Q = M / k
    # undo normalisation
    return c * s + [xm, ym], Q / (s * s)


def circle_quality(xy):
    c = (xy.max(axis=0) + xy.min(axis=0)) / 2
    r = np.linalg.norm(xy - c, axis=1)
    R = r.mean()
    offset = np.linalg.norm(c)
    # worst-case heading error from an off-centre circle and from out-of-roundness
    err_off = np.degrees(np.arcsin(min(offset / R, 1.0)))
    err_round = np.degrees((r.max() - r.min()) / (r.max() + r.min()))
    return {'centre_offset_pct': 100 * offset / R, 'roundness_pct': 100 * r.std() / R,
            'heading_error_deg_approx': err_off + err_round}


def apply_model(model, C, b, raw):
    return (raw - b) @ C.T if model == 'C(raw-b)' else raw @ C.T - b


def which_model(C, b, raw, proc):
    """Which formula reproduces the UM7's processed output (allowing one overall scale)."""
    out = {}
    for model in ('C(raw-b)', 'C*raw-b'):
        pred = apply_model(model, C, b, raw)
        s = np.sum(pred * proc) / max(np.sum(pred * pred), 1e-12)
        out[model] = (np.median(np.linalg.norm(proc - s * pred, axis=1) / np.linalg.norm(proc, axis=1)), s)
    best = min(out, key=lambda m: out[m][0])
    return best, out


# ----------------------------------------------------------------- commands

def ask(q):
    return input(q + ' [y/N] ').strip().lower() == 'y'


def cmd_info(um7, _):
    fw = um7.command(CMD_FW_VERSION, want_data=True)
    print('UM7 firmware:', fw.decode(errors='replace'))
    C, b = um7.calibration()
    np.set_printoptions(precision=6, suppress=True)
    print('mag calibration matrix (CREG_MAG_CAL):\n', C)
    print('mag bias (CREG_MAG_BIAS):', b)
    print('raw mag now:', um7.raw_mag(), '  processed mag now:', um7.proc_mag())


def cmd_calibrate(um7, args):
    out = os.path.join(OUT_ROOT, datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    C_old, b_old = um7.calibration()
    a = record(um7, args.seconds, 'raw magnetometer')
    np.savetxt(os.path.join(out, 'samples.csv'), a, delimiter=',',
               header='t_s,raw_x,raw_y,raw_z,proc_x,proc_y,proc_z', comments='')
    raw, proc = a[:, 1:4], a[:, 4:7]
    if len(raw) < 50:
        sys.exit('Too few samples; nothing written.')

    model, fits = which_model(C_old, b_old, raw, proc)
    for m, (res, s) in fits.items():
        print(f'  calibration formula {m:9s}: matches processed data within {100 * res:5.1f} % (scale {s:.3g})')
    if fits[model][0] > 0.10:
        sys.exit('Neither formula matches how this UM7 processes its magnetometer; nothing written. '
                 f'Samples saved in {out}')
    scale = fits[model][1]

    c, Q = fit_ellipse(raw[:, 0], raw[:, 1])
    w, V = np.linalg.eigh(Q)
    axes = 1 / np.sqrt(w)
    R = np.sqrt(axes.prod())
    T = V @ np.diag(np.sqrt(w)) @ V.T * R            # ellipse -> circle of radius R (counts)
    # keep the processed x/y magnitude where it was (the filter only uses direction)
    g = np.median(np.linalg.norm(proc[:, :2], axis=1)) / R / scale
    C_new = np.zeros((3, 3))
    C_new[:2, :2] = g * T
    C_new[2, 2] = C_old[2, 2]
    if model == 'C(raw-b)':
        b_new = np.array([c[0], c[1], b_old[2]])
    else:
        b_new = np.array([*(C_new[:2, :2] @ c), b_old[2]])

    before = circle_quality(proc[:, :2])
    after = circle_quality(apply_model(model, C_new, b_new, raw)[:, :2] * scale)
    print(f'\nFit: centre ({c[0]:.1f}, {c[1]:.1f}) counts, axes {axes[0]:.1f} / {axes[1]:.1f} counts '
          f'(ratio {axes.max() / axes.min():.3f})')
    print(f'  before: centre offset {before["centre_offset_pct"]:5.1f} %  roundness {before["roundness_pct"]:4.1f} %  '
          f'heading error ~{before["heading_error_deg_approx"]:4.1f} deg')
    print(f'  after : centre offset {after["centre_offset_pct"]:5.1f} %  roundness {after["roundness_pct"]:4.1f} %  '
          f'heading error ~{after["heading_error_deg_approx"]:4.1f} deg  (predicted)')
    result = {'model': model, 'old': {'C': C_old.tolist(), 'b': b_old.tolist()},
              'new': {'C': C_new.tolist(), 'b': b_new.tolist()},
              'fit': {'centre': c.tolist(), 'axes': axes.tolist()}, 'before': before, 'after_predicted': after}
    path = os.path.join(out, 'result.json')
    with open(path, 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\nSaved {path} (old values included, for restore).')

    if not ask('Write the new calibration to the UM7 (RAM only, not yet flash)?'):
        print('Nothing written.')
        return
    um7.set_calibration(C_new, b_new)
    C_chk, b_chk = um7.calibration()
    if not (np.allclose(C_chk, C_new, rtol=1e-5, atol=1e-9) and np.allclose(b_chk, b_new, rtol=1e-5, atol=1e-6)):
        sys.exit('Read-back does not match what was written. Not committing; power-cycle the UM7 to drop it.')
    print('Written and read back OK (active now, lost at power-off until committed).')
    print('Tip: run "verify" first, then commit with "commit".' if not args.commit_now else '')
    if args.commit_now and ask('Commit to UM7 flash (survives power-off)?'):
        um7.command(CMD_FLASH_COMMIT)
        print('Committed to flash.')


def cmd_verify(um7, args):
    a = record(um7, args.seconds, 'calibrated magnetometer')
    q = circle_quality(a[:, 4:6])
    print(f'Calibrated field: centre offset {q["centre_offset_pct"]:.1f} %, roundness {q["roundness_pct"]:.1f} %, '
          f'heading error ~{q["heading_error_deg_approx"]:.1f} deg')
    print('Good: offset < 5 % and roundness < 3 % (roughly < 4 deg).')


def cmd_commit(um7, _):
    C, b = um7.calibration()
    np.set_printoptions(precision=6, suppress=True)
    print('Current (RAM) calibration:\n', C, '\nbias', b)
    if ask('Commit these to UM7 flash?'):
        um7.command(CMD_FLASH_COMMIT)
        print('Committed to flash.')


def cmd_restore(um7, args):
    old = json.load(open(args.file))['old']
    um7.set_calibration(old['C'], old['b'])
    print('Old calibration written (RAM).')
    if ask('Commit to UM7 flash?'):
        um7.command(CMD_FLASH_COMMIT)
        print('Committed to flash.')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--port', default=PORT)
    sub = p.add_subparsers(dest='cmd', required=True)
    sub.add_parser('info')
    c = sub.add_parser('calibrate')
    c.add_argument('--seconds', type=float, default=180)
    c.add_argument('--commit-now', action='store_true', help='offer to commit to flash right after writing')
    v = sub.add_parser('verify')
    v.add_argument('--seconds', type=float, default=120)
    sub.add_parser('commit')
    r = sub.add_parser('restore')
    r.add_argument('file')
    args = p.parse_args()

    active = subprocess.run(['systemctl', 'is-active', 'ipex-rover'], capture_output=True, text=True).stdout.strip()
    if active == 'active':
        sys.exit('ipex-rover is running and owns the UM7 port. Run: sudo systemctl stop ipex-rover')
    um7 = UM7(args.port)
    try:
        {'info': cmd_info, 'calibrate': cmd_calibrate, 'verify': cmd_verify,
         'commit': cmd_commit, 'restore': cmd_restore}[args.cmd](um7, args)
    finally:
        um7.close()


if __name__ == '__main__':
    main()
