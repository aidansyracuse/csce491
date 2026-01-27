#!/usr/bin/env python3
import sys, os, math

# --- setup to import Waves from utils (same pattern as skeleton) ---
code_dir = os.path.split(os.path.abspath(sys.argv[0]))[0]
parent_dir = os.path.split(code_dir)[0]
python_utils_dir = os.path.join(parent_dir, "utils", "python_utils")
sys.path.append(python_utils_dir)
from waves import Waves

w = Waves()
w.loadText(sys.stdin.read())

def log(s):
    sys.stderr.write(str(s) + "\n")

def find_signal(names, *keys):
    low = [n.lower() for n in names]
    for key in keys:
        key = key.lower()
        for i, n in enumerate(low):
            if key in n:
                return names[i]
    return None

sig_names = list(w.signals())
clk_sig = find_signal(sig_names, "sclk", "clk", "clock")
mosi_sig = find_signal(sig_names, "mosi", "si", "mosi_m") or find_signal(sig_names, "mosi")
miso_sig = find_signal(sig_names, "miso", "so", "miso_m") or find_signal(sig_names, "miso")
cs_sig   = find_signal(sig_names, "cs", "ss", "ssel", "chipselect", "chip_select")
cpol_sig = find_signal(sig_names, "cpol")
cpha_sig = find_signal(sig_names, "cpha")

if clk_sig is None or (mosi_sig is None and miso_sig is None):
    log("ERROR: missing required signals (clk/mosi/miso)")
    sys.exit(0)

S = w.samples()
log("Signals: " + ", ".join(sig_names))
log("samples: {}".format(S))

# helper to get a signal value at a float time
def sig_at_time(sig, t):
    # w.signalAt(signal, time) is provided in the Waves utils
    return int(w.signalAt(sig, t))

# determine CS polarity robustly by sampling CS at the first clock edge found
cs_active_low = True
if cs_sig is not None:
    # try to find a clock edge (either posedge or negedge) and sample cs there
    first_clk_edge_t = None
    # start searching from the beginning
    start_t = w.data[0][0]
    (tpos, okpos) = w.nextEdge(clk_sig, start_t, posedge=True, negedge=False)
    (tneg, okneg) = w.nextEdge(clk_sig, start_t, posedge=False, negedge=True)
    # pick the earliest valid edge time
    cand = []
    if okpos:
        cand.append(tpos)
    if okneg:
        cand.append(tneg)
    if len(cand) == 0:
        # fallback: no clock edges found — assume active-low (common default)
        cs_active_low = True
        log("Warning: no clk edges found when determining CS polarity; assuming active-low")
    else:
        first_clk_edge_t = min(cand)
        try:
            cs_val_during_clk = int(w.signalAt(cs_sig, first_clk_edge_t))
            # if cs_val_during_clk == 0 then active is 0 -> active-low True
            cs_active_low = (cs_val_during_clk == 0)
            log("Sampled cs at first clk edge t={}: cs_val={} -> cs_active_low={}".format(first_clk_edge_t, cs_val_during_clk, cs_active_low))
        except Exception as e:
            cs_active_low = True
            log("Error sampling cs at clk edge: {}; assuming active-low".format(e))
else:
    log("no CS signal found; assuming CS always active")


def cs_is_active_time(t):
    if cs_sig is None:
        return True
    v = sig_at_time(cs_sig, t)
    return (v == 0) if cs_active_low else (v == 1)

# read CPOL/CPHA initial values if present (Part 3)
cpol = 0
cpha = 0
if cpol_sig is not None:
    try:
        cpol = sig_at_time(cpol_sig, w.data[0][0])
    except Exception:
        cpol = 0
if cpha_sig is not None:
    try:
        cpha = sig_at_time(cpha_sig, w.data[0][0])
    except Exception:
        cpha = 0
log("cpol={}, cpha={}".format(cpol, cpha))

# Determine which edge to sample on given CPOL/CPHA.
# For typical SPI:
# Mode 0: CPOL=0, CPHA=0 -> sample on rising (leading) edge
# Mode 1: CPOL=0, CPHA=1 -> sample on falling (trailing) edge
# Mode 2: CPOL=1, CPHA=0 -> sample on falling (leading) edge
# Mode 3: CPOL=1, CPHA=1 -> sample on rising (trailing) edge
#
# We will treat "leading edge" as the first edge after idle, and sampling
# edge depends on CPHA: if CPHA==0 sample on leading edge, else sample on trailing.
# For simplicity:
if cpol == 0 and cpha == 0:
    sample_posedge = True
    sample_negedge = False
elif cpol == 0 and cpha == 1:
    sample_posedge = False
    sample_negedge = True
elif cpol == 1 and cpha == 0:
    sample_posedge = False
    sample_negedge = True
else:  # cpol==1 and cpha==1
    sample_posedge = True
    sample_negedge = False

log("sampling on posedge={}, negedge={}".format(sample_posedge, sample_negedge))

# We'll scan through the waveform timewise using the Waves API to jump to edges.
transactions_pairs = []  # list of (mosi_byte, miso_byte) exchanges

t = w.data[0][0]  # start time
end_time = w.data[-1][0]

# Helper: find next time >= t where CS becomes active (or return None)
def find_next_cs_active(t0):
    t = t0
    # if already active at t0, return t0
    try:
        if cs_is_active_time(t):
            return t
    except Exception:
        pass
    # else advance by looking for edges on cs_sig
    if cs_sig is None:
        return t0
    cur_t = t
    while cur_t <= end_time:
        (nt, ok) = w.nextEdge(cs_sig, cur_t, posedge=True, negedge=True)
        if not ok:
            return None
        cur_t = nt
        try:
            if cs_is_active_time(cur_t):
                return cur_t
        except Exception:
            return None
        # move slightly forward to avoid finding same edge again
        cur_t = cur_t + 1e-9
    return None

# Helper: find next time when CS becomes inactive (return time or None)
def find_next_cs_inactive(t0):
    if cs_sig is None:
        return None
    cur_t = t0
    while cur_t <= end_time:
        (nt, ok) = w.nextEdge(cs_sig, cur_t, posedge=True, negedge=True)
        if not ok:
            return None
        cur_t = nt
        try:
            if not cs_is_active_time(cur_t):
                return cur_t
        except Exception:
            return None
        cur_t = cur_t + 1e-9
    return None

# Main loop: find next active CS window, then collect clock-synchronized bits while active
search_t = t
while True:
    start_active = find_next_cs_active(search_t)
    if start_active is None:
        break
    # determine end of this active window (time when cs becomes inactive)
    end_active = find_next_cs_inactive(start_active)
    if end_active is None:
        end_active = end_time + 1.0  # go until end if never deasserted

    # Within [start_active, end_active) collect bits on clock sampling edges
    # Start searching clock edges at start_active
    clk_search_t = start_active
    mosi_bits = []
    miso_bits = []
    while True:
        (edge_t, ok) = w.nextEdge(clk_sig, clk_search_t, posedge=sample_posedge, negedge=sample_negedge)
        if not ok:
            break
        # if found edge outside active window, stop
        if edge_t >= end_active:
            break
        # sample data slightly after the edge time (tiny epsilon) to ensure stable read
        sample_time = edge_t + 1e-9
        try:
            mbit = sig_at_time(mosi_sig, sample_time) if mosi_sig is not None else 0
        except Exception:
            mbit = 0
        try:
            sbit = sig_at_time(miso_sig, sample_time) if miso_sig is not None else 0
        except Exception:
            sbit = 0
        mosi_bits.append(int(mbit))
        miso_bits.append(int(sbit))
        # advance search time to just after this edge to find the next one
        clk_search_t = edge_t + 1e-9

    # convert bits into byte pairs (MSB first)
    nbytes = min(len(mosi_bits), len(miso_bits)) // 8
    for i in range(nbytes):
        mbyte = 0
        sbyte = 0
        for j in range(8):
            mbyte = (mbyte << 1) | mosi_bits[i*8 + j]
            sbyte = (sbyte << 1) | miso_bits[i*8 + j]
        transactions_pairs.append((mbyte, sbyte))

    # continue searching after end_active
    search_t = end_active + 1e-9
    if search_t > end_time:
        break

log("Found total exchanges: {}".format(len(transactions_pairs)))

# Interpret exchanges, supporting streaming transactions (Part 2)
out_lines = []
i = 0
L = len(transactions_pairs)
while i + 1 < L:
    cmd_mosi, cmd_miso = transactions_pairs[i]
    data_mosi, data_miso = transactions_pairs[i+1]

    cmd = cmd_mosi
    addr = (cmd >> 2) & 0x3f   # bits 7..2
    write_flag = (cmd >> 1) & 0x1
    stream_flag = cmd & 0x1

    if stream_flag == 0:
        # normal 2-exchange transaction
        if write_flag == 1:
            out_lines.append("WR {:02x} {:02x}".format(addr, data_mosi))
        else:
            out_lines.append("RD {:02x} {:02x}".format(addr, data_miso))
        i += 2
    else:
        # streaming transaction: data_mosi (second exchange MOSI) contains N
        N = data_mosi
        if N < 0:
            N = 0
        # collect next N exchanges starting at i+2 .. i+1+N
        values = []
        # ensure we don't run past available pairs
        max_avail = L - (i + 2)
        take = min(N, max_avail)
        for k in range(take):
            ex_mosi, ex_miso = transactions_pairs[i + 2 + k]
            if write_flag == 1:
                # write stream: master -> slave; take MOSI
                values.append("{:02x}".format(ex_mosi))
            else:
                # read stream: slave -> master; take MISO
                values.append("{:02x}".format(ex_miso))
        # Format output
        typ = "WR" if write_flag == 1 else "RD"
        if take == N:
            out_lines.append("{} STREAM {:02x} {}".format(typ, addr, " ".join(values)))
        else:
            # partial (shouldn't happen on valid tests) — still output what we have
            out_lines.append("{} STREAM {:02x} {}".format(typ, addr, " ".join(values)))
        # advance i past command, length, and the N exchanges we consumed
        i += 2 + take


# Print decoded lines (stdout only)
for l in out_lines:
    print(l)
