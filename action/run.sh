#!/usr/bin/env bash
# Finds videos matching SF_FILES, stabilizes each, writes a report and a job summary.
set -uo pipefail
shopt -s globstar nullglob extglob

out_dir="${SF_OUTPUT_DIR:-}"
suffix="${SF_SUFFIX:-_stabilized}"
preset="${SF_PRESET:-handheld}"
work="${RUNNER_TEMP:-/tmp}/steadyframe-run"
mkdir -p "$work"
list_file="$work/outputs.txt"
artifact_file="$work/artifact-paths.txt"
report="$work/report.json"
: > "$list_file"
: > "$artifact_file"

# Collect inputs, de-duplicated, excluding our own outputs.
declare -A seen=()
inputs=()
read -r -a patterns <<< "$(echo "${SF_FILES:-**/*.mp4}" | tr '\n' ' ')"
for pat in "${patterns[@]}"; do
  for f in $pat; do
    [ -f "$f" ] || continue
    f="${f#./}"
    case "$f" in
      *"$suffix".*) continue ;;
      *_compare.*) continue ;;
    esac
    if [ -n "$out_dir" ] && [[ "$f" == "${out_dir%/}/"* ]]; then continue; fi
    [ -n "${seen[$f]:-}" ] && continue
    seen[$f]=1
    inputs+=("$f")
  done
done

echo "Found ${#inputs[@]} video(s)."
common=(--preset "$preset" -y)
[ -n "${SF_CONFIG:-}" ] && common+=(--config "$SF_CONFIG")
extra=()
[ -n "${SF_ARGS:-}" ] && read -r -a extra <<< "$SF_ARGS"

outputs=()
failed=0
reports=()
i=0
for f in "${inputs[@]}"; do
  i=$((i + 1))
  dir="$(dirname "$f")"
  base="$(basename "$f")"
  stem="${base%.*}"
  if [ -n "$out_dir" ]; then
    dest_dir="${out_dir%/}/$dir"
    [ "$dir" = "." ] && dest_dir="${out_dir%/}"
  else
    dest_dir="$dir"
  fi
  mkdir -p "$dest_dir"
  out="$dest_dir/$stem$suffix.mp4"
  if [ "${SF_SKIP_EXISTING:-true}" = "true" ] && [ -f "$out" ]; then
    echo "::notice file=$f::Skipping, $out already exists"
    continue
  fi
  echo "::group::[$i/${#inputs[@]}] $f → $out"
  r="$work/report-$i.json"
  if steadyframe "$f" -o "$out" --report "$r" "${common[@]}" "${extra[@]}"; then
    outputs+=("$out")
    echo "$out" >> "$list_file"
    echo "$out" >> "$artifact_file"
    reports+=("$r")
    if [ "${SF_COMPARE:-none}" != "none" ]; then
      cmp="$dest_dir/${stem}_compare.mp4"
      if steadyframe "$f" -o "$cmp" --compare "$SF_COMPARE" "${common[@]}" "${extra[@]}" -q; then
        echo "$cmp" >> "$list_file"
        echo "$cmp" >> "$artifact_file"
      fi
    fi
  else
    failed=$((failed + 1))
    echo "::error file=$f::Stabilization failed"
    [ -f "$r" ] && reports+=("$r")
  fi
  echo "::endgroup::"
done

# Merge per-file reports.
if [ ${#reports[@]} -gt 0 ] && command -v jq >/dev/null; then
  jq -s '{tool: .[0].tool, version: .[0].version, preset: .[0].preset, config: .[0].config, results: [.[].results[]]}' "${reports[@]}" > "$report"
else
  echo '{"results": []}' > "$report"
fi
echo "$report" >> "$artifact_file"

# Job summary
if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    echo "### Steadyframe"
    echo
    if [ ${#reports[@]} -gt 0 ] && command -v jq >/dev/null; then
      echo "| Video | Shake before | Shake after | Reduction | Zoom | Time |"
      echo "|---|---|---|---|---|---|"
      jq -r '.results[] | if .error then "| \(.input) | failed: \(.error | gsub("[\\n|]"; " ")) | | | | |" else
        "| \(.input) | \(.stats.jitter_before.translation_px*10|round/10) px, \(.stats.jitter_before.rotation_deg*100|round/100)° | \(.stats.jitter_after.translation_px*100|round/100) px, \(.stats.jitter_after.rotation_deg*100|round/100)° | \(.stats.reduction_pct|round)% | \(.stats.mean_zoom*100|round/100)× | \(.seconds|round) s |" end' "$report"
    else
      echo "No videos were processed."
    fi
  } >> "$GITHUB_STEP_SUMMARY"
fi

{
  echo "count=${#outputs[@]}"
  echo "failed=$failed"
  echo "report=$report"
  echo "list-file=$list_file"
  echo "files<<SF_EOF"
  printf '%s\n' "${outputs[@]}"
  echo "SF_EOF"
  echo "artifact-paths<<SF_EOF"
  cat "$artifact_file"
  echo "SF_EOF"
} >> "${GITHUB_OUTPUT:-/dev/null}"

echo "Stabilized ${#outputs[@]} video(s), $failed failed."
if [ "$failed" -gt 0 ] && [ "${SF_FAIL_ON_ERROR:-true}" = "true" ]; then
  exit 1
fi
exit 0
