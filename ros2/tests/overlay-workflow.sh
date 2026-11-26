#!/usr/bin/env bash
set -euo pipefail

# The argument is an empty scratch directory; no real Docker or SSH calls are made.
scratch="${1:?Pass an empty scratch directory}"
[[ -d "$scratch" && -z "$(ls -A "$scratch")" ]] || {
  echo "Scratch directory must already exist and be empty." >&2; exit 1;
}
scratch="$(cd "$scratch" && pwd)"
repo="$(cd "$(dirname "$0")/.." && pwd)"
real_docker="$(command -v docker)"
mkdir -p "$scratch/work/src/demo" "$scratch/work/scripts" "$scratch/bin" "$scratch/remote"
cp "$repo/DockerFile" "$repo/.dockerignore" "$repo/astro.repos" \
  "$repo/docker-entrypoint.sh" "$repo/docker-compose.yml" "$repo/Makefile" "$scratch/work/"
cp "$repo/scripts/overlay.sh" "$scratch/work/scripts/"
printf '<package><name>demo</name></package>\n' > "$scratch/work/src/demo/package.xml"

cat > "$scratch/bin/ssh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
shift
cd "$MOCK_REMOTE"
export MOCK_REMOTE_CONTEXT=1
exec bash -c "$1"
EOF

cat > "$scratch/bin/docker" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
case "$1" in
  buildx)
    target= output= packages=
    while (($#)); do
      case "$1" in
        --target) target="$2"; shift ;;
        --output) output="$2"; shift ;;
        --build-arg) [[ "$2" != OVERLAY_PACKAGES=* ]] || packages="${2#*=}"; shift ;;
      esac
      shift
    done
    case "$target" in
      overlay-fingerprint)
        mkdir -p "${output#type=local,dest=}"
        printf 'mock package versions\n' > "${output#type=local,dest=}/packages.txt"
        ;;
      overlay-export)
        printf '%s\n' "$packages" >> "$MOCK_BUILDS"
        tar -cf "${output#type=tar,dest=}" -C "$MOCK_PAYLOAD" .
        ;;
    esac
    ;;
  save) printf image ;;
  load) cat >/dev/null ;;
  image)
    if [[ "$*" == *'json .RootFS'* ]]; then
      printf '%s\n' "${MOCK_IMAGE_CONTENT:-matching image content}"
    elif [[ "${MOCK_REMOTE_CONTEXT:-}" == 1 ]]; then
      printf 'sha256:%064d\n' 2
    else
      printf 'sha256:%064d\n' 1
    fi
    ;;
  create) printf '123456789abc\n' ;;
  cp) printf 'baseline install\n' > "$3/baseline.txt" ;;
  rm) : ;;
  inspect) printf 'sha256:%064d\n' "${MOCK_CONTAINER_IMAGE:-2}" ;;
  compose)
    if [[ " $* " == *" ps -aq ros "* ]]; then
      printf '123456789abc\n'
    else
      printf '%s %s\n' "${ASTRO_ROBOT_MODEL:-unset}" "$*" >> "$MOCK_COMPOSE"
    fi
    ;;
  *) echo "Unexpected Docker invocation: $*" >&2; exit 1 ;;
esac
EOF
chmod +x "$scratch/bin/ssh" "$scratch/bin/docker"
mkdir -p "$scratch/payload"
printf 'overlay\n' > "$scratch/payload/local_setup.bash"
export MOCK_REMOTE="$scratch/remote" MOCK_PAYLOAD="$scratch/payload"
export MOCK_BUILDS="$scratch/builds" MOCK_COMPOSE="$scratch/compose"
export PATH="$scratch/bin:$PATH"

cd "$scratch/work"
make overlay-init PI=pi.local PI_USER=pi ROBOT_MODEL=burger
if make overlay-deploy PI=pi.local PI_USER=pi PACKAGE=demo; then
  echo "Mismatched robot model was not rejected." >&2
  exit 1
fi
make overlay-deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger PACKAGE=demo
make overlay-deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger PACKAGE=another
[[ -z "$(grep -v '^burger ' "$MOCK_COMPOSE")" ]]
[[ "$(tail -n 1 "$MOCK_BUILDS")" == "another demo" ]]
[[ -f "$MOCK_REMOTE/astro-robot/docker-compose.overlay.yml" ]]
"$real_docker" compose -f "$MOCK_REMOTE/astro-robot/docker-compose.yml" \
  -f "$MOCK_REMOTE/astro-robot/docker-compose.overlay.yml" config -q
baseline="$(<.overlay/current)"
[[ "$(<"$baseline/packages")" == $'another\ndemo' ]]
export MOCK_IMAGE_CONTENT='different image content'
if make overlay-deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger PACKAGE=demo; then
  echo "Mismatched Pi image was not rejected." >&2
  exit 1
fi
unset MOCK_IMAGE_CONTENT
export MOCK_CONTAINER_IMAGE=3
if make overlay-deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger PACKAGE=demo; then
  echo "Mismatched running container was not rejected." >&2
  exit 1
fi
unset MOCK_CONTAINER_IMAGE
printf '# changed\n' >> "$MOCK_REMOTE/astro-robot/docker-compose.yml"
if make overlay-deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger PACKAGE=demo; then
  echo "Changed Pi Compose file was not rejected." >&2
  exit 1
fi
cp "$baseline/context/docker-compose.yml" "$MOCK_REMOTE/astro-robot/docker-compose.yml"
printf '<changed/>\n' >> src/demo/package.xml
if make overlay-deploy PI=pi.local PI_USER=pi ROBOT_MODEL=burger PACKAGE=demo; then
  echo "Manifest change was not rejected." >&2
  exit 1
fi
make overlay-reset PI=pi.local PI_USER=pi ROBOT_MODEL=burger
[[ "$(tail -n 1 "$MOCK_COMPOSE")" == burger* ]]
[[ ! -s "$baseline/packages" ]]
echo "Overlay workflow mock test passed."
