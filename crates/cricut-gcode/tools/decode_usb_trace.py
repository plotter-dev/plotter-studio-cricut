"""Decode a Design Space session captured as a USB trace export and print its commands.

The export is UTF-16 text with one group of lines per URB event (a table of fields followed by a
hex dump of the payload). The host's bytes are the bulk OUT payloads; the machine's are the bulk
IN payloads. The session key is the machine's `NewAES` reply (`0x30` + 48 bytes) decrypted with
the Explore Air 2's KEK, so the capture must include both directions.

Usage:
    python decode_usb_trace.py <trace.txt>             # commands, polls hidden, vectors summarised
    python decode_usb_trace.py <trace.txt> --all       # also every poll and every vector frame

Needs the `cryptography` package. The session key stays in memory and is never printed. This is a
read-only analysis tool: it never talks to a machine.
"""
import collections
import re
import sys

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

# The Explore Air 2 base buffer, as in `protocol.rs`; KEK[i] = base[63 - 2i].
BASE = bytes.fromhex(
    "204c63cd413760ad33e2142121165655728372dd44c351db15947136336078aa"
    "21d913388505788b568a3787112c2500011e64e570b231cb56c9868605401618"
)
KEK = bytes(BASE[63 - 2 * i] for i in range(32))

NAMES = {
    0x21: "BeginDraw", 0x22: "EndDraw", 0x30: "SetMachineState", 0x31: "Geometry",
    0xA1: "SpeedRegister", 0x76: "OrientBlade", 0x43: "MultiDrawVector",
    0x60: "MachineStatus", 0x73: "InterfaceState", 0x72: "SetLights", 0x71: "ClearButtons",
    0x68: "Abort", 0x84: "UnloadMat", 0xB9: "ReadPosition", 0xA0: "ReadRegister",
}
POLLS = {0x60, 0x73, 0xB9, 0x38}


def ecb_decrypt(data, key):
    return Cipher(algorithms.AES(key), modes.ECB()).decryptor().update(data)


def unescape(raw):
    """Undo the machine's byte stuffing: 0x5E 0xA2 is 0x24."""
    out = bytearray()
    i = 0
    while i < len(raw):
        if raw[i] == 0x5E and i + 1 < len(raw) and raw[i + 1] == 0xA2:
            out.append(0x24)
            i += 2
        else:
            out.append(raw[i])
            i += 1
    return bytes(out)


def events(path):
    """The trace's lines grouped by event number."""
    text = open(path, "rb").read().decode("utf-16")
    grouped = collections.OrderedDict()
    current = None
    for line in text.splitlines():
        m = re.match(r"^(\d+)\t0x", line)
        if m:
            current = int(m.group(1))
            grouped.setdefault(current, [])
        if current is not None and line.strip():
            grouped[current].append(line)
    return grouped


def chunks(grouped):
    """(event, 'tx' | 'rx', bytes) for each bulk transfer that carries data."""
    out = []
    for number, lines in grouped.items():
        text = "\n".join(lines)
        direction = re.search(r'Direction\t"(\w+)"', text)
        endpoint = re.search(r"EndpointAddress\t([^\t]*)", text)
        if not direction or not endpoint:
            continue
        if "(output)" in endpoint.group(1) and direction.group(1) == "Down":
            kind = "tx"
        elif "(input)" in endpoint.group(1) and direction.group(1) == "Up":
            kind = "rx"
        else:
            continue
        data = bytearray()
        for line in lines:
            m = re.match(r"^\t{6}[0-9a-f]{8}   ((?:[0-9a-f]{2} )+)", line)
            if m:
                data += bytes.fromhex(m.group(1).replace(" ", ""))
        if data:
            out.append((number, kind, bytes(data)))
    return out


def find_session_key(rx):
    u = unescape(rx)
    for i, byte in enumerate(u):
        if byte == 0x30 and i + 49 <= len(u):
            plain = ecb_decrypt(u[i + 1 : i + 49], KEK)
            if plain[0] == 1:
                return plain[1:33]
    return None


def split_frames(stream):
    """Host frames: `[len][op][payload]` plain, or `[0x80 | n][n encrypted bytes]`."""
    i = 0
    frames = []
    while i < len(stream):
        h = stream[i]
        n = 1 + (h if h < 0x80 else (128 if h == 0x80 else (h & 0x7F)))
        frames.append(stream[i : i + n])
        i += n
    return frames


def commands(stream, key):
    """(encrypted?, opcode, payload) for every host frame."""
    out = []
    for f in split_frames(stream):
        if f[0] >= 0x80 and (len(f) - 1) % 16 == 0:
            body = ecb_decrypt(f[1:], key)
            out.append((True, body[1], bytes(body[2 : 1 + body[0]])))
        else:
            out.append((False, f[1], bytes(f[2:])))
    return out


def main():
    path = sys.argv[1]
    show_all = "--all" in sys.argv[2:]
    ch = chunks(events(path))
    tx = b"".join(c[2] for c in ch if c[1] == "tx")
    rx = b"".join(c[2] for c in ch if c[1] == "rx")
    key = find_session_key(rx)
    if key is None:
        sys.exit("no NewAES reply in the machine's bytes: capture both directions")
    previous, repeats = None, 0
    rows = []
    for encrypted, op, payload in commands(tx, key):
        if not show_all and op in POLLS:
            continue
        item = (encrypted, op, len(payload) if op == 0x43 and not show_all else payload)
        if item == previous:
            repeats += 1
            continue
        if previous is not None:
            rows.append((previous, repeats))
        previous, repeats = item, 1
    if previous is not None:
        rows.append((previous, repeats))
    for (encrypted, op, payload), count in rows:
        shown = f"len={payload}" if isinstance(payload, int) else payload.hex()
        print(f"{'enc' if encrypted else 'pln'} {NAMES.get(op, f'op{op:02x}'):16s} {shown[:96]}"
              + (f"  x{count}" if count > 1 else ""))


if __name__ == "__main__":
    main()
