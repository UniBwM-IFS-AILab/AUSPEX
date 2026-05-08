#!/usr/bin/env bash
set -euo pipefail

# net_bench.sh — measure *real* latency, jitter, and throughput (client-side)
#
# Measures:
#  - Latency: ping avg (RTT avg)
#  - Jitter:  ping mdev, iperf3 UDP jitter, fping stddev (high-frequency)
#  - Throughput: iperf3 TCP (single stream + parallel streams)
#
# Output:
#  - Appends one CSV row per run with timestamp, distance, src_ip, dst_ip, and all metrics.
#
# Usage:
#   # On the destination node (server):
#   iperf3 -s
#
#   # On the source node (client):
#   ./net_bench.sh --target 10.8.0.16 --out results.csv --distance 25
#
# Optional:
#   ./net_bench.sh --target 10.8.0.16 --out results.csv --distance 25 --tcp-parallel 4 --udp-rate 20M
#   ./net_bench.sh --target 10.8.0.16 --out results.csv  # distance will be NA if not specified

TARGET=""
DISTANCE=""
OUTFILE="net_bench_results.csv"

PING_COUNT=50
PING_INTERVAL="0.2"     # seconds (<=0.2 needs sudo on many systems)
FPING_COUNT=500
FPING_PERIOD=10         # ms between probes for fping

UDP_RATE="20M"          # iperf3 UDP send rate
UDP_DURATION=15         # seconds
TCP_DURATION=15         # seconds
TCP_PARALLEL=4          # iperf3 -P parallel streams

TIMEOUT_SEC=25

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "ERROR: missing dependency: $1" >&2; exit 1; }
}

usage() {
  cat <<EOF
Usage:
  $0 --target <dst_ip> [--out <file.csv>] [options]

Required:
  --target <dst_ip>             Destination IP (must be reachable)

Output:
  --out <file.csv>              CSV file to append to (default: ${OUTFILE})
  --distance <value>            Distance in km or any unit (optional, for reference)

Ping (latency + jitter via mdev):
  --ping-count <n>              (default: ${PING_COUNT})
  --ping-interval <sec>         (default: ${PING_INTERVAL})

fping (high-frequency jitter via stddev):
  --fping-count <n>             (default: ${FPING_COUNT})
  --fping-period-ms <ms>        (default: ${FPING_PERIOD})

iperf3 UDP (jitter + loss + UDP throughput):
  --udp-rate <rate>             (default: ${UDP_RATE}) e.g. 5M, 20M, 100M
  --udp-duration <sec>          (default: ${UDP_DURATION})

iperf3 TCP (throughput):
  --tcp-duration <sec>          (default: ${TCP_DURATION})
  --tcp-parallel <n>            (default: ${TCP_PARALLEL})

Examples:
  iperf3 -s                      # run on destination first
  $0 --target 10.8.0.16 --out run1.csv
  $0 --target 10.8.0.16 --udp-rate 10M --tcp-parallel 8
EOF
}

# --- args ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    --target) TARGET="${2:-}"; shift 2;;
    --out) OUTFILE="${2:-}"; shift 2;;
    --distance) DISTANCE="${2:-}"; shift 2;;
    --ping-count) PING_COUNT="${2:-}"; shift 2;;
    --ping-interval) PING_INTERVAL="${2:-}"; shift 2;;
    --fping-count) FPING_COUNT="${2:-}"; shift 2;;
    --fping-period-ms) FPING_PERIOD="${2:-}"; shift 2;;
    --udp-rate) UDP_RATE="${2:-}"; shift 2;;
    --udp-duration) UDP_DURATION="${2:-}"; shift 2;;
    --tcp-duration) TCP_DURATION="${2:-}"; shift 2;;
    --tcp-parallel) TCP_PARALLEL="${2:-}"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1" >&2; usage; exit 1;;
  esac
done

[[ -n "$TARGET" ]] || { echo "ERROR: --target is required" >&2; usage; exit 1; }

# --- deps ---
need_cmd ip
need_cmd ping
need_cmd fping
need_cmd iperf3
need_cmd python3
need_cmd timeout

# --- helpers ---
csv_escape() {
  # Escape double-quotes for CSV and wrap in quotes
  local s="${1:-}"
  s="${s//\"/\"\"}"
  printf "\"%s\"" "$s"
}

get_src_ip() {
  # Prefer the actual route-selected source IP to TARGET
  local src
  src="$(ip route get "$TARGET" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')"
  if [[ -z "${src:-}" ]]; then
    src="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  echo "${src:-NA}"
}

iso_ts() {
  date -Iseconds
}

get_signal_strength() {
  # Use external get_signal_strength.sh script for consistent behavior across PC and Pi
  # Returns: For managed: "AP=XX:XX:XX:XX:XX:XX|sig=-XX"
  #          For mesh/IBSS: "MAC=peer1_mac|sig=XX|avg=YY;..."
  #          Or: "NA" if no connection
  local script_dir script_path
  
  # Find the get_signal_strength.sh script relative to this script's location
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  script_path="${script_dir}/get_signal_strength.sh"
  
  # If script exists, use it; otherwise return NA
  if [[ -x "$script_path" ]]; then
    "$script_path"
  else
    echo "NA"
  fi
}

run_ping() {
  # Returns: avg_ms mdev_ms
  local out avg mdev
  out="$(timeout "${TIMEOUT_SEC}" ping -n -q -c "${PING_COUNT}" -i "${PING_INTERVAL}" "${TARGET}" 2>&1 || true)"

  # Parse: rtt min/avg/max/mdev = a/b/c/d ms
  avg="$(echo "$out" | awk -F'=' '/rtt|round-trip/ {gsub(/ ms/,"",$2); split($2,a,"/"); gsub(/^[ \t]+/,"",a[2]); print a[2]; exit}')"
  mdev="$(echo "$out" | awk -F'=' '/rtt|round-trip/ {gsub(/ ms/,"",$2); split($2,a,"/"); gsub(/^[ \t]+/,"",a[4]); print a[4]; exit}')"

  [[ -n "${avg:-}" ]] || avg="NA"
  [[ -n "${mdev:-}" ]] || mdev="NA"
  echo "$avg" "$mdev"
}

run_fping() {
  # Returns: avg_ms stddev_ms  
  # Note: fping outputs per-packet results followed by a summary line
  # We extract avg from summary. For stddev, we calculate it from per-packet results.
  local out avg stddev

  out="$(timeout "${TIMEOUT_SEC}" fping -c "${FPING_COUNT}" -p "${FPING_PERIOD}" "${TARGET}" 2>&1 || true)"

  # fping summary format: "10.8.0.16 : xmt/rcv/%loss = N/N/N%, min/avg/max = X/Y/Z"
  # or sometimes with stddev: "... min/avg/max/stddev = X/Y/Z/W"
  
  # Try to extract from summary line with min/avg/max format
  avg="$(echo "$out" | awk '
    /xmt\/rcv\/%loss/ {
      # Extract avg from "min/avg/max = a/b/c" or "min/avg/max/stddev = a/b/c/d"
      line = $0
      idx = index(line, "min/avg/max")
      if (idx > 0) {
        # Get substring from min/avg/max onwards
        part = substr(line, idx)
        # Skip to the "=" and beyond
        sub(/^[^=]*=[ \t]*/, "", part)
        # Split the values by /
        split(part, vals, "/")
        if (vals[2] != "") {
          print vals[2]
          found = 1
        }
      }
    }
    END { if (!found) print "" }
  ')"
  
  # Try to extract stddev from summary line if it exists
  stddev="$(echo "$out" | awk '
    /xmt\/rcv\/%loss/ {
      # Check if stddev is in the summary: "min/avg/max/stddev = a/b/c/d"
      line = $0
      idx = index(line, "min/avg/max/stddev")
      if (idx > 0) {
        # Get substring from min/avg/max/stddev onwards
        part = substr(line, idx)
        # Skip to the "=" and beyond
        sub(/^[^=]*=[ \t]*/, "", part)
        # Split the values by /
        split(part, vals, "/")
        # stddev is the 4th value
        if (vals[4] != "") {
          print vals[4]
          found = 1
        }
      }
    }
    END { if (!found) print "" }
  ')"
  
  # If stddev not in summary, calculate it from per-packet results
  if [[ -z "${stddev:-}" ]]; then
    stddev="$(echo "$out" | awk '
      # Parse lines like: "10.8.0.16 : [N], 64 bytes, X.XX ms"
      /\[.*\], 64 bytes, [0-9.]+ ms/ {
        # Extract the time value
        line = $0
        idx = index(line, " ms")
        if (idx > 0) {
          # Find the start of the number (work backwards from " ms")
          num_end = idx - 1
          num_start = num_end
          while (num_start > 0 && (substr(line, num_start, 1) ~ /[0-9.]/)) {
            num_start--
          }
          if (num_start < num_end) {
            val = substr(line, num_start + 1, num_end - num_start)
            values[n] = val
            n++
          }
        }
      }
      END {
        if (n > 1) {
          # Calculate mean
          sum = 0
          for (i = 0; i < n; i++) sum += values[i]
          mean = sum / n
          
          # Calculate standard deviation
          var_sum = 0
          for (i = 0; i < n; i++) {
            diff = values[i] - mean
            var_sum += diff * diff
          }
          stddev = sqrt(var_sum / n)
          printf "%.3f", stddev
        }
      }
    ')"
  fi

  [[ -n "${avg:-}" ]] || avg="NA"
  [[ -n "${stddev:-}" ]] || stddev="NA"
  echo "$avg" "$stddev"
}


run_iperf3_json() {
  # $1 = mode: "tcp" or "udp"
  # $2 = parallel streams (tcp only; can be 1)
  # Emits JSON to stdout on success; returns non-zero on failure
  local mode="$1"
  local par="${2:-1}"
  local cmd=(iperf3 -c "$TARGET" -J)

  if [[ "$mode" == "udp" ]]; then
    cmd+=( -u -b "$UDP_RATE" -t "$UDP_DURATION" )
  else
    cmd+=( -t "$TCP_DURATION" )
    if [[ "$par" -gt 1 ]]; then
      cmd+=( -P "$par" )
    fi
  fi

  local err tmp
  err="$(mktemp)"
  tmp="$(mktemp)"
  # stdout -> tmp (JSON), stderr -> err (status/warnings)
  if ! timeout "${TIMEOUT_SEC}" "${cmd[@]}" >"$tmp" 2>"$err"; then
    echo "WARN: iperf3 ${mode} failed (exit code $?). stderr:" >&2
    sed 's/^/  /' "$err" >&2
    rm -f "$tmp" "$err"
    return 1
  fi

  # Ensure stdout actually looks like JSON
  if grep -q '^[[:space:]]*{' "$tmp"; then
    cat "$tmp"
    rm -f "$tmp" "$err"
    return 0
  else
    echo "WARN: iperf3 ${mode} produced no JSON on stdout. stderr:" >&2
    sed 's/^/  /' "$err" >&2
    rm -f "$tmp" "$err"
    return 1
  fi
}


parse_iperf3() {
  local mode="$1"
  local json_str="$2"
  
  python3 << EOFPY
import json, sys
mode = "$mode"
raw = """$json_str"""

if not raw or not raw.strip():
    print("NA NA" if mode=="tcp" else "NA NA NA")
    sys.exit(0)

try:
    data = json.loads(raw)
except Exception as e:
    print("NA NA" if mode=="tcp" else "NA NA NA")
    sys.exit(0)

def mbps(bps):
    return bps / 1e6

end = data.get("end", {}) or {}

if mode == "udp":
    s = end.get("sum", {}) or end.get("sum_received", {}) or {}
    bps = s.get("bits_per_second")
    jitter = s.get("jitter_ms")
    lostp = s.get("lost_percent")
    print(
        (f"{mbps(bps):.3f}" if isinstance(bps,(int,float)) else "NA"),
        (f"{jitter:.3f}" if isinstance(jitter,(int,float)) else "NA"),
        (f"{lostp:.3f}" if isinstance(lostp,(int,float)) else "NA"),
    )
else:
    s_recv = end.get("sum_received", {}) or {}
    s_sent = end.get("sum_sent", {}) or {}
    bps = s_recv.get("bits_per_second") or s_sent.get("bits_per_second")
    retrans = s_sent.get("retransmits")
    print(
        (f"{mbps(bps):.3f}" if isinstance(bps,(int,float)) else "NA"),
        (str(retrans) if retrans is not None else "NA"),
    )
EOFPY
}


# --- run all tests ---
TS="$(iso_ts)"
SRC_IP="$(get_src_ip)"
DST_IP="$TARGET"
SIGNAL="$(get_signal_strength)"

# ping
read -r PING_AVG_MS PING_MDEV_MS < <(run_ping)

# fping (high-frequency)
read -r FPING_AVG_MS FPING_STDDEV_MS < <(run_fping)

# iperf3 UDP (throughput + jitter + loss)
UDP_JSON=""
UDP_Mbps="NA"; UDP_Jitter="NA"; UDP_LostPct="NA"
if UDP_JSON="$(run_iperf3_json udp 1)"; then
  read -r UDP_Mbps UDP_Jitter UDP_LostPct < <(parse_iperf3 udp "$UDP_JSON")
fi


# iperf3 TCP single stream
TCP1_JSON=""
TCP1_Mbps="NA"; TCP1_Retr="NA"
if TCP1_JSON="$(run_iperf3_json tcp 1)"; then
  read -r TCP1_Mbps TCP1_Retr < <(parse_iperf3 tcp "$TCP1_JSON")
fi

# iperf3 TCP parallel
TCPP_JSON=""
TCPP_Mbps="NA"; TCPP_Retr="NA"
if [[ "$TCP_PARALLEL" -gt 1 ]]; then
  if TCPP_JSON="$(run_iperf3_json tcp "$TCP_PARALLEL")"; then
    read -r TCPP_Mbps TCPP_Retr < <(parse_iperf3 tcp "$TCPP_JSON")
  fi
fi

# --- write CSV ---
HEADER="timestamp,distance,signal_strength,src_ip,dst_ip,ping_avg_ms,ping_mdev_ms,fping_avg_ms,fping_stddev_ms,iperf3_udp_mbps,iperf3_udp_jitter_ms,iperf3_udp_lost_percent,iperf3_tcp_mbps,iperf3_tcp_retransmits,iperf3_tcp_parallel_streams,iperf3_tcp_parallel_mbps,iperf3_tcp_parallel_retransmits"

if [[ ! -f "$OUTFILE" || ! -s "$OUTFILE" ]]; then
  echo "$HEADER" > "$OUTFILE"
fi

{
  echo -n "$(csv_escape "$TS"),"
  echo -n "$(csv_escape "${DISTANCE:-NA}"),"
  echo -n "$(csv_escape "$SIGNAL"),"
  echo -n "$(csv_escape "$SRC_IP"),"
  echo -n "$(csv_escape "$DST_IP"),"
  echo -n "$(csv_escape "$PING_AVG_MS"),"
  echo -n "$(csv_escape "$PING_MDEV_MS"),"
  echo -n "$(csv_escape "$FPING_AVG_MS"),"
  echo -n "$(csv_escape "$FPING_STDDEV_MS"),"
  echo -n "$(csv_escape "$UDP_Mbps"),"
  echo -n "$(csv_escape "$UDP_Jitter"),"
  echo -n "$(csv_escape "$UDP_LostPct"),"
  echo -n "$(csv_escape "$TCP1_Mbps"),"
  echo -n "$(csv_escape "$TCP1_Retr"),"
  echo -n "$(csv_escape "$TCP_PARALLEL"),"
  echo -n "$(csv_escape "$TCPP_Mbps"),"
  echo    "$(csv_escape "$TCPP_Retr")"
} >> "$OUTFILE"

echo "Wrote results to: $OUTFILE"
echo "Run summary:"
echo "  from ${SRC_IP} -> ${DST_IP} @ ${TS}"
echo "  signal: ${SIGNAL}"
echo "  ping:   avg=${PING_AVG_MS} ms, mdev=${PING_MDEV_MS} ms"
echo "  fping:  avg=${FPING_AVG_MS} ms, stddev=${FPING_STDDEV_MS} ms"
echo "  iperf3 UDP:  ${UDP_Mbps} Mbit/s, jitter=${UDP_Jitter} ms, loss=${UDP_LostPct}%"
echo "  iperf3 TCP:  ${TCP1_Mbps} Mbit/s (retrans=${TCP1_Retr})"
echo "  iperf3 TCP P=${TCP_PARALLEL}: ${TCPP_Mbps} Mbit/s (retrans=${TCPP_Retr})"
