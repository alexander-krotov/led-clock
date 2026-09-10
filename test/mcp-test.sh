#!/usr/bin/env bash
#
# Basic smoke test for the led-clock MCP server.
#
# Fires a handful of JSON-RPC 2.0 requests at POST http://<host>/mcp with curl
# and checks the responses. Meant to be run from WSL Ubuntu (or any Linux with
# bash + curl). `jq` is used for response checks and pretty-printing if present;
# without it the checks fall back to plain substring matching.
#
# Usage:
#   bash test/mcp-test.sh                        # host defaults to led-clock.local:8080
#   bash test/mcp-test.sh 192.168.1.42:8080
#   MCP_HOST=led-clock.local:8080 bash test/mcp-test.sh
#   bash test/mcp-test.sh --set-offset 3         # also exercise set_ntp_config (offset)
#   bash test/mcp-test.sh --set-server fi.pool.ntp.org
#
# Exit status is 0 only if every check passes. --set-* options write persistent
# config on the device, so leave them off for a read-only run.

set -u

HOST="${MCP_HOST:-led-clock.local:8080}"
SET_OFFSET=""
SET_SERVER=""

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)
      sed -n '3,18p' "$0" | sed 's/^# \{0,1\}//; s/^#$//'
      exit 0
      ;;
    --set-offset) SET_OFFSET="${2:-}"; shift 2 ;;
    --set-server) SET_SERVER="${2:-}"; shift 2 ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *)  HOST="$1"; shift ;;
  esac
done

URL="http://${HOST}/mcp"

if command -v jq >/dev/null 2>&1; then
  HAVE_JQ=1
  pretty() { jq . 2>/dev/null || cat; }
else
  HAVE_JQ=0
  pretty() { cat; }
  echo "note: jq not found, using substring checks (apt install jq for better output)" >&2
fi

PASS=0
FAIL=0
ID=0

# rpc <description> <method> <params-json-or-empty> <expect>
#   expect:
#     result       -> HTTP 200 and a top-level "result" object
#     tool-ok      -> result present and result.isError is not true
#     tool-error   -> result present and result.isError is true
#     rpc-error    -> HTTP 200 and a top-level "error" object
#     http:<code>  -> HTTP status equals <code>, body not inspected
rpc() {
  local desc="$1" method="$2" params="$3" expect="$4"
  ID=$((ID + 1))

  local body
  if [ "$method" = "__RAW__" ]; then
    body="${params}"          # send params verbatim as the request body
  elif [ "$method" = "notifications/initialized" ]; then
    body="{\"jsonrpc\":\"2.0\",\"method\":\"${method}\"}"
  elif [ -n "$params" ]; then
    body="{\"jsonrpc\":\"2.0\",\"id\":${ID},\"method\":\"${method}\",\"params\":${params}}"
  else
    body="{\"jsonrpc\":\"2.0\",\"id\":${ID},\"method\":\"${method}\"}"
  fi

  echo "=============================================================="
  echo "TEST: ${desc}"
  echo "  -> ${method} ${params}"

  local resp rc status payload
  resp="$(curl -sS -m 10 -w $'\n%{http_code}' \
            -H 'Content-Type: application/json' \
            -H 'Accept: application/json' \
            -H 'Expect:' \
            -X POST --data "${body}" "${URL}" 2>&1)"
  rc=$?
  if [ $rc -ne 0 ]; then
    echo "  FAIL: curl error (rc=$rc): ${resp}"
    FAIL=$((FAIL + 1))
    return
  fi

  status="${resp##*$'\n'}"
  payload="${resp%$'\n'*}"

  echo "  HTTP ${status}"
  [ -n "$payload" ] && echo "$payload" | pretty | sed 's/^/  /'

  local ok=1 why=""
  case "$expect" in
    http:*)
      [ "$status" = "${expect#http:}" ] || { ok=0; why="expected HTTP ${expect#http:}, got ${status}"; }
      ;;
    result|tool-ok|tool-error|rpc-error)
      if [ "$status" != "200" ]; then
        ok=0; why="HTTP ${status}"
      elif [ "$HAVE_JQ" = "1" ]; then
        case "$expect" in
          result)
            echo "$payload" | jq -e '.result' >/dev/null 2>&1 || { ok=0; why="no .result"; } ;;
          tool-ok)
            echo "$payload" | jq -e '.result' >/dev/null 2>&1 || { ok=0; why="no .result"; }
            echo "$payload" | jq -e '.result.isError == true' >/dev/null 2>&1 && { ok=0; why="result.isError is true"; } ;;
          tool-error)
            echo "$payload" | jq -e '.result.isError == true' >/dev/null 2>&1 || { ok=0; why="result.isError not true"; } ;;
          rpc-error)
            echo "$payload" | jq -e '.error' >/dev/null 2>&1 || { ok=0; why="no .error"; } ;;
        esac
      else
        case "$expect" in
          result|tool-ok)   [[ "$payload" == *'"result"'* ]] || { ok=0; why="no result in body"; } ;;
          tool-error)       [[ "$payload" == *'"isError"'* ]] || { ok=0; why="no isError in body"; } ;;
          rpc-error)        [[ "$payload" == *'"error"'* ]]  || { ok=0; why="no error in body"; } ;;
        esac
      fi
      ;;
    *)
      ok=0; why="bad expect spec: ${expect}" ;;
  esac

  if [ "$ok" = "1" ]; then
    echo "  PASS"; PASS=$((PASS + 1))
  else
    echo "  FAIL: ${why}"; FAIL=$((FAIL + 1))
  fi
}

echo "MCP server: ${URL}"

rpc "initialize handshake" \
    "initialize" \
    '{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"mcp-test","version":"0"}}' \
    result

rpc "initialized notification (no id -> 202)" \
    "notifications/initialized" "" "http:202"

rpc "ping" "ping" "" result

rpc "tools/list" "tools/list" "" result

rpc "get_sensors" \
    "tools/call" '{"name":"get_sensors","arguments":{}}' tool-ok

rpc "get_ds3231_time" \
    "tools/call" '{"name":"get_ds3231_time","arguments":{}}' tool-ok

rpc "unknown tool -> isError" \
    "tools/call" '{"name":"does_not_exist","arguments":{}}' tool-error

rpc "unknown method -> JSON-RPC error" \
    "no/such/method" "" rpc-error

rpc "malformed JSON -> parse error" \
    "__RAW__" '{ this is not json' rpc-error

if [ -n "$SET_OFFSET" ]; then
  rpc "set_ntp_config utc_offset_hours=${SET_OFFSET}" \
      "tools/call" "{\"name\":\"set_ntp_config\",\"arguments\":{\"utc_offset_hours\":${SET_OFFSET}}}" \
      tool-ok
fi

if [ -n "$SET_SERVER" ]; then
  rpc "set_ntp_config ntp_server=${SET_SERVER}" \
      "tools/call" "{\"name\":\"set_ntp_config\",\"arguments\":{\"ntp_server\":\"${SET_SERVER}\"}}" \
      tool-ok
fi

echo "=============================================================="
echo "RESULT: ${PASS} passed, ${FAIL} failed"
[ "$FAIL" -eq 0 ]
