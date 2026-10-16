#!/usr/bin/env bash
# Puts a `steadyframe` binary on PATH: downloads a release build when one exists
# for this runner, otherwise builds from the action's source with cargo.
set -euo pipefail

repo="${SF_ACTION_REPO:-Perceus64/steadyframe}"
[ -z "$repo" ] && repo="Perceus64/steadyframe"
version="${SF_VERSION:-}"
bin_dir="${RUNNER_TEMP:-/tmp}/steadyframe-bin"
mkdir -p "$bin_dir"

case "${RUNNER_OS:-Linux}-${RUNNER_ARCH:-X64}" in
  Linux-X64)   target="x86_64-unknown-linux-gnu"; ext="tar.gz" ;;
  Linux-ARM64) target="aarch64-unknown-linux-gnu"; ext="tar.gz" ;;
  macOS-ARM64) target="aarch64-apple-darwin"; ext="tar.gz" ;;
  macOS-X64)   target="x86_64-apple-darwin"; ext="tar.gz" ;;
  Windows-X64) target="x86_64-pc-windows-msvc"; ext="zip" ;;
  *)           target=""; ext="" ;;
esac

build_from_source() {
  echo "Building steadyframe from source (first run takes a minute or two)…"
  if ! command -v cargo >/dev/null; then
    curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
    # shellcheck disable=SC1091
    source "$HOME/.cargo/env"
  fi
  cargo build --release --locked --manifest-path "$SF_ACTION_PATH/Cargo.toml" -p steadyframe-cli \
    || cargo build --release --manifest-path "$SF_ACTION_PATH/Cargo.toml" -p steadyframe-cli
  local exe="steadyframe"
  [ "${RUNNER_OS:-}" = "Windows" ] && exe="steadyframe.exe"
  cp "$SF_ACTION_PATH/target/release/$exe" "$bin_dir/"
}

download() {
  local tag="$1"
  [ -z "$target" ] && return 1
  local url
  if [ "$tag" = "latest" ]; then
    url="https://github.com/$repo/releases/latest/download/steadyframe-$target.$ext"
  else
    url="https://github.com/$repo/releases/download/$tag/steadyframe-$target.$ext"
  fi
  echo "Downloading $url"
  local tmp="${RUNNER_TEMP:-/tmp}/steadyframe-dl.$ext"
  curl -fsSL --retry 3 -o "$tmp" "$url" || return 1
  if [ "$ext" = "zip" ]; then
    unzip -o -q "$tmp" -d "$bin_dir"
  else
    tar -xzf "$tmp" -C "$bin_dir"
  fi
}

if [ "$version" = "source" ]; then
  build_from_source
elif [ -n "$version" ]; then
  download "$version" || { echo "::warning::No release binary for $version/$target; building from source."; build_from_source; }
elif [[ "${SF_ACTION_REF:-}" =~ ^v[0-9] ]]; then
  download "$SF_ACTION_REF" || build_from_source
else
  build_from_source
fi

echo "$bin_dir" >> "$GITHUB_PATH"
"$bin_dir/steadyframe" --version
