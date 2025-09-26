#!/usr/bin/env python3
import sys
import argparse
from pathlib import Path
import struct
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

ALIGNMENT_BYTE = 0x33
CRC_POLY = 0x1011
CRC_SEED = 0x35

def crc16(poly: int, seed: int, buf: bytes) -> int:
    crc = seed
    for b in buf:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

def read_varint(b: bytes, pos: int):
    result = 0
    shift = 0
    while True:
        if pos >= len(b):
            raise EOFError
        byte = b[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            break
        shift += 7
    return result, pos

def decode_key(b: bytes, pos: int):
    key, pos = read_varint(b, pos)
    return key >> 3, key & 7, pos

def read_len_delim(b: bytes, pos: int):
    ln, pos = read_varint(b, pos)
    end = pos + ln
    if end > len(b):
        raise EOFError
    return b[pos:end], end

def parse_inner(payload_bytes: bytes):
    """Return (floats_dict, varints_dict) from a length-delimited inner message payload."""
    import struct as _st
    p = 0
    floats = {}
    varints = {}
    while p < len(payload_bytes):
        try:
            fnum, wtype, p2 = decode_key(payload_bytes, p)
        except Exception:
            break
        if wtype == 5:  # fixed32 -> float
            if p2 + 4 > len(payload_bytes): break
            raw = int.from_bytes(payload_bytes[p2:p2+4], "little")
            val = _st.unpack("<f", _st.pack("<I", raw))[0]
            floats[fnum] = val
            p = p2 + 4
        elif wtype == 0:  # varint (bool/int)
            v, p = read_varint(payload_bytes, p2)
            varints[fnum] = v
        elif wtype == 2:  # length-delimited (skip nested)
            _, p = read_len_delim(payload_bytes, p2)
        elif wtype == 1:  # fixed64 (skip)
            p = p2 + 8
        else:
            break
    return floats, varints

def parse_oneof(inner: bytes):
    p = 0
    tag, wtype, p = decode_key(inner, p)
    if wtype != 2:
        return None, None
    payload, _ = read_len_delim(inner, p)
    return tag, payload

def decode_file_strict(path: Path):
    raw = path.read_bytes()
    i = 0
    payloads = []
    bad_crc = 0

    while i < len(raw) - 4:
        idx = raw.find(bytes([ALIGNMENT_BYTE]), i)
        if idx == -1:
            break
        if idx + 2 >= len(raw):
            break
        length = raw[idx + 1]
        end = idx + 2 + length + 2
        if end > len(raw):
            break
        payload = raw[idx + 2: idx + 2 + length]
        rec_crc_lo = raw[idx + 2 + length]
        rec_crc_hi = raw[idx + 2 + length + 1]
        rec_crc = rec_crc_lo | (rec_crc_hi << 8)
        calc = crc16(CRC_POLY, CRC_SEED, raw[idx: idx + 2 + length])
        if calc == rec_crc:
            payloads.append(payload)
            i = end
        else:
            bad_crc += 1
            i = idx + 1

    imu = {"t": [], "ax": [], "ay": [], "az": [], "gx": [], "gy": [], "gz": []}
    est = {"t": [], "ax": [], "ay": [], "az": [],
           "vx": [], "vy": [], "vz": [],
           "x": [], "y": [], "z": [],
           "roll": [], "pitch": [], "yaw": []}
    pressure = {"t": [], "pressure": []}
    gnss = {"t": [], "lat": [], "lon": [], "alt": []}
    events = {"t": [],}  # dynamic boolean fields
    pyro = {"t": [],}    # dynamic boolean fields
    flight_state = {"t": [], "state": []}
    event_fields = set()
    pyro_fields = set()

    tags_count = {}

    for mb in payloads:
        pos = 0
        if pos < len(mb) and mb[pos] == 0x0D:
            t_raw = int.from_bytes(mb[pos + 1: pos + 5], "little")
            pos += 5
        else:
            try:
                fnum, wtype, pos = decode_key(mb, pos)
                if wtype == 5:
                    t_raw = int.from_bytes(mb[pos:pos+4], "little")
                    pos += 4
                else:
                    continue
            except Exception:
                continue

        if pos < len(mb) and mb[pos] == 0x10:
            _, pos = read_varint(mb, pos + 1)
        if pos >= len(mb) or mb[pos] != 0x1A:
            continue
        inner, _ = read_len_delim(mb, pos + 1)

        tag, payload = parse_oneof(inner)
        if tag is None:
            continue
        tags_count[tag] = tags_count.get(tag, 0) + 1

        floats, varints = parse_inner(payload)
        t = float(t_raw) / 1000.0  # your request: divide by 1000

        # Known: IMU (3), StateEstimate (4 or 17)
        if tag == 3:
            if all(k in floats for k in (1,2,3,4,5,6)):
                imu["t"].append(t)
                imu["ax"].append(floats[1]); imu["ay"].append(floats[2]); imu["az"].append(floats[3])
                imu["gx"].append(floats[4]); imu["gy"].append(floats[5]); imu["gz"].append(floats[6])
            continue
        if tag in (4, 17):
            est["t"].append(t)
            est["x"].append(floats.get(1, np.nan))
            est["y"].append(floats.get(2, np.nan))
            est["z"].append(floats.get(3, np.nan))
            est["vx"].append(floats.get(4, np.nan))
            est["vy"].append(floats.get(5, np.nan))
            est["vz"].append(floats.get(6, np.nan))
            est["ax"].append(floats.get(7, np.nan))
            est["ay"].append(floats.get(8, np.nan))
            est["az"].append(floats.get(9, np.nan))
            est["roll"].append(floats.get(10, np.nan))
            est["pitch"].append(floats.get(11, np.nan))
            est["yaw"].append(floats.get(12, np.nan))
            continue

        # Heuristics:
        # Pressure: exactly one float field present across many samples
        if len(floats) == 1 and not varints:
            key = next(iter(floats.keys()))
            pressure["t"].append(t)
            pressure["pressure"].append(floats[key])
            continue

        # GNSS: three floats with lat/lon ranges
        if len(floats) >= 2:
            fvals = list(floats.values())
            # try locate lat/lon
            cand = sorted(fvals)  # not used; we need ranges
            vals = list(floats.items())
            lat = None; lon = None; alt = None
            for k,v in floats.items():
                if -90.5 <= v <= 90.5 and lat is None:
                    lat = v
                elif -180.5 <= v <= 180.5 and lon is None:
                    lon = v
                else:
                    alt = v if alt is None else alt
            if lat is not None and lon is not None:
                gnss["t"].append(t)
                gnss["lat"].append(lat); gnss["lon"].append(lon); gnss["alt"].append(alt if alt is not None else np.nan)
                continue

        # Flight events: mostly boolean varints (1/0), multiple fields
        if varints and all(v in (0,1) for v in varints.values()):
            # Decide bucket size: if >=4 booleans, call it events; if <=3, call it pyro
            if len(varints) >= 4:
                events["t"].append(t)
                for k,v in varints.items():
                    name = f"event_{k}"
                    events.setdefault(name, []).append(v)
                    event_fields.add(name)
                continue
            else:
                pyro["t"].append(t)
                for k,v in varints.items():
                    name = f"pyro_{k}"
                    pyro.setdefault(name, []).append(v)
                    pyro_fields.add(name)
                continue
        # Flight state: non-boolean varints (e.g., enum)
        if varints and any(v not in (0,1) for v in varints.values()):
            # take first varint as state enum value
            key, val = next(iter(varints.items()))
            flight_state["t"].append(t)
            flight_state["state"].append(val)
            continue

        # If nothing matched, ignore this tag

    return imu, est, pressure, gnss, events, pyro, flight_state, tags_count, len(payloads), bad_crc

def list_and_choose_bin(base_dir: Path) -> Path:
    bins = sorted(base_dir.glob("*.bin"))
    if not bins:
        print(f"No .bin files found in {base_dir}")
        sys.exit(1)
    print("Select a .bin file:")
    for i, p in enumerate(bins, 1):
        print(f"  {i}) {p.name}")
    while True:
        try:
            sel = int(input("Enter number: "))
            if 1 <= sel <= len(bins):
                return bins[sel - 1]
        except Exception:
            pass
        print("Invalid selection. Try again.")


def plot_groups(out_prefix: Path, title: str):
    from matplotlib.ticker import ScalarFormatter
    # Load CSVs if they exist
    imu_csv = Path(str(out_prefix) + "_imu.csv")
    est_csv = Path(str(out_prefix) + "_est.csv")
    prs_csv = Path(str(out_prefix) + "_pressure.csv")
    gnss_csv = Path(str(out_prefix) + "_gnss.csv")
    evt_csv = Path(str(out_prefix) + "_events.csv")
    pyro_csv = Path(str(out_prefix) + "_pyro.csv")
    flight_csv = Path(str(out_prefix) + "_flight.csv")

    imu_df = pd.read_csv(imu_csv) if imu_csv.exists() else pd.DataFrame()
    est_df = pd.read_csv(est_csv) if est_csv.exists() else pd.DataFrame()
    prs_df = pd.read_csv(prs_csv) if prs_csv.exists() else pd.DataFrame()
    gnss_df = pd.read_csv(gnss_csv) if gnss_csv.exists() else pd.DataFrame()
    evt_df = pd.read_csv(evt_csv) if evt_csv.exists() else pd.DataFrame()
    pyro_df = pd.read_csv(pyro_csv) if pyro_csv.exists() else pd.DataFrame()
    flt_df = pd.read_csv(flight_csv) if flight_csv.exists() else pd.DataFrame()

    def fmt_axes(axs):
        from matplotlib.ticker import ScalarFormatter
        for ax in axs:
            ax.set_xlabel('Time [s]')
            fmt = ScalarFormatter(useOffset=False, useMathText=False)
            fmt.set_scientific(False)
            ax.xaxis.set_major_formatter(fmt)

    # Group 1: RAW (IMU accel, IMU gyro, Pressure)
    import matplotlib.pyplot as plt, numpy as np
    fig1, axs1 = plt.subplots(3, 1, figsize=(11, 10), sharex=False)
    axs1 = np.asarray(axs1).ravel()
    if not imu_df.empty and {'t','ax','ay','az'}.issubset(imu_df.columns):
        axs1[0].plot(imu_df['t'], imu_df['ax'], marker='.', linestyle='None', label='ax_imu')
        axs1[0].plot(imu_df['t'], imu_df['ay'], marker='.', linestyle='None', label='ay_imu')
        axs1[0].plot(imu_df['t'], imu_df['az'], marker='.', linestyle='None', label='az_imu')
    axs1[0].set_ylabel('IMU Accel'); 
    if axs1[0].lines: axs1[0].legend()

    if not imu_df.empty and {'t','gx','gy','gz'}.issubset(imu_df.columns):
        axs1[1].plot(imu_df['t'], imu_df['gx'], marker='.', linestyle='None', label='gx')
        axs1[1].plot(imu_df['t'], imu_df['gy'], marker='.', linestyle='None', label='gy')
        axs1[1].plot(imu_df['t'], imu_df['gz'], marker='.', linestyle='None', label='gz')
    axs1[1].set_ylabel('IMU Gyro'); 
    if axs1[1].lines: axs1[1].legend()

    if not prs_df.empty and {'t','pressure'}.issubset(prs_df.columns):
        axs1[2].plot(prs_df['t'], prs_df['pressure'], marker='.', linestyle='None', label='pressure')
    axs1[2].set_ylabel('Pressure');
    if axs1[2].lines: axs1[2].legend()
    fmt_axes(axs1)
    fig1.suptitle(title + " — RAW")
    fig1.tight_layout()
    fig1.savefig(Path(str(out_prefix) + "_raw.png"), dpi=160)

    # Group 2: STATE (Est accel, vel, pos, attitude)
    fig2, axs2 = plt.subplots(4, 1, figsize=(11, 13), sharex=False)
    axs2 = np.asarray(axs2).ravel()
    if not est_df.empty:
        if {'t','ax_est','ay_est','az_est'}.issubset(est_df.columns):
            axs2[0].plot(est_df['t'], est_df['ax_est'], marker='.', linestyle='None', label='ax_est')
            axs2[0].plot(est_df['t'], est_df['ay_est'], marker='.', linestyle='None', label='ay_est')
            axs2[0].plot(est_df['t'], est_df['az_est'], marker='.', linestyle='None', label='az_est')
        axs2[0].set_ylabel('Accel (state)'); axs2[0].text(0.995, 0.95, 'state', transform=axs2[0].transAxes, ha='right', va='top')
        if axs2[0].lines: axs2[0].legend()

        if {'t','vx','vy','vz'}.issubset(est_df.columns):
            axs2[1].plot(est_df['t'], est_df['vx'], marker='.', linestyle='None', label='vx')
            axs2[1].plot(est_df['t'], est_df['vy'], marker='.', linestyle='None', label='vy')
            axs2[1].plot(est_df['t'], est_df['vz'], marker='.', linestyle='None', label='vz')
        axs2[1].set_ylabel('Velocity (state)'); axs2[1].text(0.995, 0.95, 'state', transform=axs2[1].transAxes, ha='right', va='top')
        if axs2[1].lines: axs2[1].legend()

        if {'t','x','y','z'}.issubset(est_df.columns):
            axs2[2].plot(est_df['t'], est_df['x'], marker='.', linestyle='None', label='x')
            axs2[2].plot(est_df['t'], est_df['y'], marker='.', linestyle='None', label='y')
            axs2[2].plot(est_df['t'], est_df['z'], marker='.', linestyle='None', label='z')
        axs2[2].set_ylabel('Position (state)'); axs2[2].text(0.995, 0.95, 'state', transform=axs2[2].transAxes, ha='right', va='top')
        if axs2[2].lines: axs2[2].legend()

        if {'t','roll','pitch','yaw'}.issubset(est_df.columns):
            axs2[3].plot(est_df['t'], est_df['roll'], marker='.', linestyle='None', label='roll')
            axs2[3].plot(est_df['t'], est_df['pitch'], marker='.', linestyle='None', label='pitch')
            axs2[3].plot(est_df['t'], est_df['yaw'], marker='.', linestyle='None', label='yaw')
        axs2[3].set_ylabel('Attitude (state)'); axs2[3].text(0.995, 0.95, 'state', transform=axs2[3].transAxes, ha='right', va='top')
        if axs2[3].lines: axs2[3].legend()
    fmt_axes(axs2)
    fig2.suptitle(title + " — STATE")
    fig2.tight_layout()
    fig2.savefig(Path(str(out_prefix) + "_state.png"), dpi=160)

    # Group 3: FLIGHT (state enum + events)
    fig3, axs3 = plt.subplots(2, 1, figsize=(11, 8), sharex=False)
    axs3 = np.asarray(axs3).ravel()
    if not flt_df.empty and {'t','state'}.issubset(flt_df.columns):
        axs3[0].plot(flt_df['t'], flt_df['state'], marker='.', linestyle='None', label='flight_state')
    axs3[0].set_ylabel('Flight State'); 
    if axs3[0].lines: axs3[0].legend()

    if not evt_df.empty:
        for col in evt_df.columns:
            if col == 't': continue
            axs3[1].plot(evt_df['t'], evt_df[col], marker='.', linestyle='None', label=col)
    axs3[1].set_ylabel('Events'); 
    if axs3[1].lines: axs3[1].legend()
    fmt_axes(axs3)
    fig3.suptitle(title + " — FLIGHT")
    fig3.tight_layout()
    fig3.savefig(Path(str(out_prefix) + "_flight.png"), dpi=160)

    # Group 4: OTHER (GNSS lat/lon, GNSS alt, Pyro)
    fig4, axs4 = plt.subplots(3, 1, figsize=(11, 10), sharex=False)
    axs4 = np.asarray(axs4).ravel()
    if not gnss_df.empty and {'t','lat','lon'}.issubset(gnss_df.columns):
        axs4[0].plot(gnss_df['t'], gnss_df['lat'], marker='.', linestyle='None', label='lat')
        axs4[0].plot(gnss_df['t'], gnss_df['lon'], marker='.', linestyle='None', label='lon')
    axs4[0].set_ylabel('GNSS lat/lon'); 
    if axs4[0].lines: axs4[0].legend()

    if not gnss_df.empty and {'t','alt'}.issubset(gnss_df.columns):
        axs4[1].plot(gnss_df['t'], gnss_df['alt'], marker='.', linestyle='None', label='alt')
    axs4[1].set_ylabel('GNSS alt'); 
    if axs4[1].lines: axs4[1].legend()

    if not pyro_df.empty:
        for col in pyro_df.columns:
            if col == 't': continue
            axs4[2].plot(pyro_df['t'], pyro_df[col], marker='.', linestyle='None', label=col)
    axs4[2].set_ylabel('Pyro'); 
    if axs4[2].lines: axs4[2].legend()
    fmt_axes(axs4)
    fig4.suptitle(title + " — OTHER")
    fig4.tight_layout()
    fig4.savefig(Path(str(out_prefix) + "_other.png"), dpi=160)


def main():
    parser = argparse.ArgumentParser(description="Decode Fjalar flash logs into CSVs and plot multiple telemetry streams.")
    parser.add_argument("path", nargs="?", help="Binary file to decode; omit to choose from ./dashboard/data")
    parser.add_argument("--dir", default=None, help="Folder with .bin files (default ./dashboard/data)")
    args = parser.parse_args()

    cwd = Path.cwd()
    base_dir = Path(args.dir) if args.dir else (cwd / "dashboard" / "data")
    if not base_dir.exists():
        alt = cwd / "data"
        if alt.exists():
            base_dir = alt

    def list_and_choose_bin(base_dir: Path) -> Path:
        bins = sorted(base_dir.glob("*.bin"))
        if not bins:
            print(f"No .bin files found in {base_dir}")
            sys.exit(1)
        print("Select a .bin file:")
        for i, p in enumerate(bins, 1):
            print(f"  {i}) {p.name}")
        while True:
            try:
                sel = int(input("Enter number: "))
                if 1 <= sel <= len(bins):
                    return bins[sel - 1]
            except Exception:
                pass
            print("Invalid selection. Try again.")

    if args.path:
        src = Path(args.path)
        if src.is_dir():
            base_dir = src
            src = list_and_choose_bin(base_dir)
        elif not src.exists():
            cand = base_dir / args.path
            if cand.exists():
                src = cand
            else:
                print(f"File not found: {args.path}")
                sys.exit(1)
    else:
        src = list_and_choose_bin(base_dir)

    imu, est, pressure, gnss, events, pyro, flight_state, tags, ok_frames, bad_crc = decode_file_strict(src)
    print(f"Parsed frames: ok={ok_frames} bad_crc={bad_crc} tags={tags} imu={len(imu['t'])}")

    # Save CSVs (trim per-group to shortest length to avoid length mismatch)
    def to_df_trim(d, rename=None):
        keys = list(d.keys())
        L = min(len(d[k]) for k in keys if isinstance(d[k], list) and k != '')
        out = {k: (d[k][:L] if isinstance(d[k], list) else d[k]) for k in keys}
        df = pd.DataFrame(out)
        if rename: df = df.rename(columns=rename)
        return df

    out_prefix = base_dir / src.stem

    imu_df = to_df_trim(imu)
    imu_csv = Path(str(out_prefix) + "_imu.csv")
    imu_df.to_csv(imu_csv, index=False)

    est_df = to_df_trim(est, rename={"ax":"ax_est","ay":"ay_est","az":"az_est"})
    est_csv = Path(str(out_prefix) + "_est.csv")
    est_df.to_csv(est_csv, index=False)

    if pressure["t"]:
        prs_df = to_df_trim(pressure)
        prs_csv = Path(str(out_prefix) + "_pressure.csv")
        prs_df.to_csv(prs_csv, index=False)
    else:
        prs_csv = Path(str(out_prefix) + "_pressure.csv")

    if gnss["t"]:
        gnss_df = to_df_trim(gnss)
        gnss_csv = Path(str(out_prefix) + "_gnss.csv")
        gnss_df.to_csv(gnss_csv, index=False)
    else:
        gnss_csv = Path(str(out_prefix) + "_gnss.csv")

    if len(events) > 1:
        evt_df = pd.DataFrame(events).fillna(0)
        evt_csv = Path(str(out_prefix) + "_events.csv")
        evt_df.to_csv(evt_csv, index=False)
    else:
        evt_csv = Path(str(out_prefix) + "_events.csv")

    if len(pyro) > 1:
        pyro_df = pd.DataFrame(pyro).fillna(0)
        pyro_csv = Path(str(out_prefix) + "_pyro.csv")
        pyro_df.to_csv(pyro_csv, index=False)
    else:
        pyro_csv = Path(str(out_prefix) + "_pyro.csv")

    # Flight state CSV
    if flight_state["t"]:
        flt_df = pd.DataFrame(flight_state)
        flight_csv = Path(str(out_prefix) + "_flight.csv")
        flt_df.to_csv(flight_csv, index=False)
    else:
        flight_csv = Path(str(out_prefix) + "_flight.csv")

    # Generate grouped figures
    plot_groups(out_prefix, f"Decoded data from {src.name}")
    print("Saved:", imu_csv, est_csv, prs_csv, gnss_csv, evt_csv, pyro_csv, flight_csv,
          Path(str(out_prefix) + "_raw.png"), Path(str(out_prefix) + "_state.png"),
          Path(str(out_prefix) + "_flight.png"), Path(str(out_prefix) + "_other.png"))

if __name__ == "__main__":
    main()
