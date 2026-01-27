import sys, os

# --- setup to import Waves from utils (same pattern as skeleton) ---
code_dir = os.path.split(os.path.abspath(sys.argv[0]))[0]
parent_dir = os.path.split(code_dir)[0]
python_utils_dir = os.path.join(parent_dir, "utils", "python_utils")
sys.path.append(python_utils_dir)
from waves import Waves

w = Waves()
w.loadText(sys.stdin.read())

# convenience: stderr logger
def log(s):
    sys.stderr.write(s + "\n")

# Helper to find a signal by keywords (case-insensitive substring match)
def find_signal(names, *keys):
    low = [n.lower() for n in names]
    for key in keys:
        key = key.lower()
        for i, n in enumerate(low):
            if key in n:
                return names[i]
    return None

sig_names = w.signals()
# pick common signal name variants
clk_sig = find_signal(sig_names, "sclk", "clk", "clock")
mosi_sig = find_signal(sig_names, "mosi", "si", "mosi_m") or find_signal(sig_names, "mosi")
miso_sig = find_signal(sig_names, "miso", "so", "miso_m") or find_signal(sig_names, "miso")
cs_sig   = find_signal(sig_names, "cs", "ss", "ssel", "chipselect", "chip_select")

if clk_sig is None or (mosi_sig is None and miso_sig is None):
    log("ERROR: couldn't find required SPI signals (clk/mosi/miso) in input")
    sys.exit(0)

# robust accessor for a signal value at sample index (tries common method names)
def bit_at(sig, idx):
    # many lab Waves objects use signalAt(name, index)
    try:
        return int(w.signalAt(sig, idx))
    except Exception:
        pass
    try:
        return int(w.signal_at(sig, idx))
    except Exception:
        pass
    # some provide a dict-like mapping w.values[name][idx]
    try:
        return int(w.values[sig][idx])
    except Exception:
        pass
    # fallback: try calling w.signal(sig)[idx]
    try:
        s = w.signal(sig)
        return int(s[idx])
    except Exception:
        pass
    # if nothing works, raise
    raise RuntimeError("unable to access waveform samples for signal: {}".format(sig))

S = w.samples()

# Determine CS active polarity (if cs exists)
cs_active_low = True
if cs_sig is not None:
    # sample some values to determine idle level (majority)
    ones = 0
    samples_to_check = min(50, S)
    for i in range(samples_to_check):
        try:
            if bit_at(cs_sig, i):
                ones += 1
        except Exception:
            pass
    idle_high = (ones > samples_to_check/2)
    cs_active_low = idle_high  # if idle is high => active when low
else:
    # no CS provided -> treat as always active
    cs_active_low = None

# helper to check if CS is active at sample index
def cs_is_active(idx):
    if cs_sig is None:
        return True
    v = bit_at(cs_sig, idx)
    return (v == 0) if cs_active_low else (v == 1)

# assemble transactions by sampling on rising clock edges (CPOL=0, CPHA=0)
transactions = []

clk_prev = bit_at(clk_sig, 0)
collecting = False
mosi_bits = []
miso_bits = []

def flush_bytes():
    """Convert collected bit streams into pairs of bytes (MOSI_byte, MISO_byte) in order.
       Returns list of (mosi_byte, miso_byte). Clears buffers."""
    pairs = []
    # group bits into bytes in the order they were sampled (MSB first)
    # create byte lists from the bit arrays
    mb = mosi_bits[:]
    sb = miso_bits[:]
    # number of full bytes available is min(len(mb), len(sb)) // 8
    nbytes = min(len(mb), len(sb)) // 8
    for i in range(nbytes):
        mbyte = 0
        sbyte = 0
        for j in range(8):
            mbyte = (mbyte << 1) | mb[i*8 + j]
            sbyte = (sbyte << 1) | sb[i*8 + j]
        pairs.append((mbyte, sbyte))
    return pairs

# walk samples
for i in range(1, S):
    try:
        clk = bit_at(clk_sig, i)
    except Exception:
        # can't read clk at this sample, skip
        continue

    active = cs_is_active(i)
    # detect rising edge
    if clk_prev == 0 and clk == 1 and active:
        # sample MOSI / MISO on this edge
        try:
            m = bit_at(mosi_sig, i) if mosi_sig is not None else 0
        except Exception:
            m = 0
        try:
            s = bit_at(miso_sig, i) if miso_sig is not None else 0
        except Exception:
            s = 0
        mosi_bits.append(m)
        miso_bits.append(s)
    # detect end of frame: when CS goes inactive or when we reach last sample
    if cs_sig is not None:
        # if previously active and now inactive, flush
        prev_active = cs_is_active(i-1)
        if prev_active and not active:
            # flush collected bits into transactions
            pairs = flush_bytes()
            # remove used bits
            bytes_consumed = len(pairs)*8
            if bytes_consumed > 0:
                # consume
                mosi_bits = mosi_bits[bytes_consumed:]
                miso_bits = miso_bits[bytes_consumed:]
            # extend transactions with pairs
            for p in pairs:
                transactions.append(p)
            # reset bit buffers
            mosi_bits = []
            miso_bits = []
    clk_prev = clk

# at end, flush any remaining full bytes
pairs = flush_bytes()
for p in pairs:
    transactions.append(p)

# Now interpret transactions as two-exchange operations:
# For each pair (first_byte, second_byte):
#   if first_byte & 0x80 != 0 -> read: addr = first_byte & 0x7f, data = second_byte (from MISO)
#   else -> write: addr = first_byte & 0x7f, data = second_byte (from MOSI)
# Print lines exactly like "WR aa vv" or "RD aa vv" (lowercase hex, zero-padded)
out_lines = []
i = 0
while i + 1 <= len(transactions)-1:
    # take first pair as bytes: transactions is list of (mosi_byte, miso_byte)
    # but the transmission semantics: first byte is on MOSI (cmd), second exchange contains data:
    cmd_mosi, cmd_miso = transactions[i]
    data_mosi, data_miso = transactions[i+1]
    cmd = cmd_mosi  # address/command lives on MOSI for these boards
    if (cmd & 0x80) != 0:
        addr = cmd & 0x7f
        val = data_miso
        out_lines.append("RD {:02x} {:02x}".format(addr, val))
    else:
        addr = cmd & 0x7f
        val = data_mosi
        out_lines.append("WR {:02x} {:02x}".format(addr, val))
    i += 2

# print results to stdout (no extra blank lines)
for l in out_lines:
    print(l)
PY
