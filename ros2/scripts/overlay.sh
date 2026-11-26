#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
root="$PWD/.overlay"
mkdir -p "$root"
exec 9>"$root/lock"
flock -n 9 || { echo "Another overlay operation is running." >&2; exit 1; }

target="${PI_USER:+${PI_USER}@}${PI}"
case "$REMOTE_DIR" in
  ""|/*|*..*|*[!a-zA-Z0-9_./-]*)
    echo "REMOTE_DIR must be a simple path relative to the SSH home directory." >&2
    exit 1
    ;;
esac
for value in "$IMAGE" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"; do
  [[ "$value" =~ ^[a-zA-Z0-9_./:@-]+$ ]] || {
    echo "Invalid image or ROS environment value." >&2
    exit 1
  }
done
case "$ROBOT_MODEL" in
  burger|waffle) ;;
  *) echo "ROBOT_MODEL must be burger or waffle." >&2; exit 1 ;;
esac
compose="IMAGE='$IMAGE' ROS_DOMAIN_ID='$ROS_DOMAIN_ID' RMW_IMPLEMENTATION='$RMW_IMPLEMENTATION' ASTRO_ROBOT_MODEL='$ROBOT_MODEL' docker compose --project-name astro --file '$REMOTE_DIR/docker-compose.yml'"
image_format='{{.Os}}/{{.Architecture}} {{json .RootFS}} {{json .Config}}'

current_baseline() {
  test -f "$root/current" || { echo "Run make overlay-init first." >&2; exit 1; }
  baseline="$(<"$root/current")"
  test -d "$baseline" && test -f "$baseline/image-id" && test -f "$baseline/image-fingerprint" &&
    test -f "$baseline/robot-model" || {
    echo "Overlay baseline is incomplete; run make overlay-init again." >&2; exit 1;
  }
  [[ "$(<"$baseline/target")" == "$target" && "$(<"$baseline/remote-dir")" == "$REMOTE_DIR" &&
     "$(<"$baseline/image")" == "$IMAGE" && "$(<"$baseline/domain")" == "$ROS_DOMAIN_ID" &&
     "$(<"$baseline/rmw")" == "$RMW_IMPLEMENTATION" && "$(<"$baseline/robot-model")" == "$ROBOT_MODEL" ]] || {
    echo "Target, image, remote directory, ROS environment, or robot model differs from the baseline." >&2; exit 1;
  }
}

check_image() {
  local expected remote_id remote_details remote_fingerprint container_id container_image local_compose remote_compose
  expected="$(<"$baseline/image-fingerprint")"
  remote_id="$(ssh "$target" "docker image inspect '$IMAGE' --format '{{.Id}}'")"
  [[ "$remote_id" =~ ^sha256:[a-f0-9]{64}$ ]] || {
    echo "Unexpected Pi image ID: $remote_id" >&2; exit 1;
  }
  remote_details="$(ssh "$target" "docker image inspect '$remote_id' --format '$image_format'")"
  remote_fingerprint="$(printf '%s\n' "$remote_details" | sha256sum | cut -d ' ' -f1)"
  [[ "$remote_fingerprint" == "$expected" ]] || {
    echo "Pi image differs from the overlay baseline; run overlay-init again." >&2; exit 1;
  }
  local_compose="$(sha256sum "$baseline/context/docker-compose.yml" | cut -d ' ' -f1)"
  remote_compose="$(ssh "$target" "sha256sum '$REMOTE_DIR/docker-compose.yml' | cut -d ' ' -f1")"
  [[ "$remote_compose" == "$local_compose" ]] || {
    echo "Pi Compose file differs from the overlay baseline." >&2; exit 1;
  }
  container_id="$(ssh "$target" "$compose ps -aq ros")"
  if [[ -z "$container_id" ]]; then
    [[ "${1:-}" == allow-missing ]] || { echo "Pi service does not exist." >&2; exit 1; }
  else
    [[ "$container_id" =~ ^[a-f0-9]{12,64}$ ]] || {
      echo "Unexpected container ID: $container_id" >&2; exit 1;
    }
    container_image="$(ssh "$target" "docker inspect '$container_id' --format '{{.Image}}'")"
    [[ "$container_image" == "$remote_id" ]] || {
      echo "Service image differs from the overlay baseline." >&2; exit 1;
    }
  fi
}

manifest_hashes() {
  ( cd "$1"; find src -path src/external -prune -o -name package.xml -type f -print0 |
      sort -z | xargs -0 -r sha256sum )
}

case "${1:-}" in
  overlay-init)
    baseline="$(mktemp -d "$root/baseline.XXXXXXXX")"
    mkdir -p "$baseline/context" "$baseline/expected/install" "$baseline/expected/runtime-lib"
    tar --exclude=src/external -cf - src DockerFile .dockerignore astro.repos \
      docker-entrypoint.sh docker-compose.yml Makefile | tar -xf - -C "$baseline/context"
    make -C "$baseline/context" deploy PI="$PI" PI_USER="$PI_USER" IMAGE="$IMAGE" \
      REMOTE_DIR="$REMOTE_DIR" ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
      RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" ROBOT_MODEL="$ROBOT_MODEL" BUILD_JOBS="$BUILD_JOBS" \
      PARALLEL_WORKERS="$PARALLEL_WORKERS"
    docker image inspect "$IMAGE" --format '{{.Id}}' > "$baseline/image-id"
    image_id="$(<"$baseline/image-id")"
    docker image inspect "$image_id" --format "$image_format" | sha256sum | cut -d ' ' -f1 > "$baseline/image-fingerprint"
    printf '%s\n' "$target" > "$baseline/target"
    printf '%s\n' "$REMOTE_DIR" > "$baseline/remote-dir"
    printf '%s\n' "$IMAGE" > "$baseline/image"
    printf '%s\n' "$ROS_DOMAIN_ID" > "$baseline/domain"
    printf '%s\n' "$RMW_IMPLEMENTATION" > "$baseline/rmw"
    printf '%s\n' "$ROBOT_MODEL" > "$baseline/robot-model"
    check_image
    container="$(docker create --platform linux/arm64 "$image_id")"
    trap 'docker rm "$container" >/dev/null' EXIT
    docker cp "$container:/ws/install/." "$baseline/expected/install/"
    docker cp "$container:/usr/local/lib/." "$baseline/expected/runtime-lib/"
    docker rm "$container" >/dev/null
    trap - EXIT
    docker buildx build --platform linux/arm64 --target overlay-fingerprint \
      --build-context "overlay-baseline=$baseline/expected" \
      --build-arg "BUILD_JOBS=$BUILD_JOBS" --build-arg "PARALLEL_WORKERS=$PARALLEL_WORKERS" \
      --output "type=local,dest=$baseline/metadata" \
      -f "$baseline/context/DockerFile" "$baseline/context"
    cp "$baseline/metadata/packages.txt" "$baseline/expected/packages.txt"
    manifest_hashes "$baseline/context" > "$baseline/manifests"
    : > "$baseline/packages"
    printf '%s\n' "$baseline" > "$root/current"
    echo "Overlay baseline initialized. Existing baseline snapshots were retained."
    ;;

  overlay-deploy)
    current_baseline
    [[ "${PACKAGE:-}" =~ ^[a-zA-Z][a-zA-Z0-9_]*$ ]] || {
      echo "Set PACKAGE to one local ROS package name." >&2; exit 1;
    }
    for file in DockerFile .dockerignore astro.repos docker-entrypoint.sh docker-compose.yml; do
      cmp -s "$file" "$baseline/context/$file" || {
        echo "$file changed; create a new baseline with make overlay-init." >&2; exit 1;
      }
    done
    if ! diff -u "$baseline/manifests" <(manifest_hashes "$PWD"); then
      echo "Package manifests changed; create a new baseline." >&2
      exit 1
    fi
    check_image
    packages="$( { cat "$baseline/packages"; printf '%s\n' "$PACKAGE"; } | sort -u | paste -sd ' ' -)"
    archive="$baseline/overlay.tar"
    docker buildx build --platform linux/arm64 --target overlay-export \
      --build-context "overlay-source=$PWD" \
      --build-context "overlay-baseline=$baseline/expected" \
      --build-arg "OVERLAY_PACKAGES=$packages" --build-arg "BUILD_JOBS=$BUILD_JOBS" \
      --build-arg "PARALLEL_WORKERS=$PARALLEL_WORKERS" \
      --output "type=tar,dest=$archive" -f "$baseline/context/DockerFile" "$baseline/context"
    printf '%s\n' "$packages" | tr ' ' '\n' > "$baseline/packages"
    check_image
    release="$(ssh "$target" "mkdir -p '$REMOTE_DIR/.overlays' && mktemp -d '$REMOTE_DIR/.overlays/release.XXXXXXXX'")"
    [[ "$release" == "$REMOTE_DIR"/.overlays/release.* && "$release" != *[!a-zA-Z0-9_./-]* ]] || {
      echo "Unexpected remote overlay directory: $release" >&2; exit 1;
    }
    gzip -c "$archive" | ssh "$target" "tar -xzf - -C '$release'"
    release_name="${release##*/}"
    ssh "$target" "cat > '$REMOTE_DIR/docker-compose.overlay.next.yml'" <<EOF
services:
  ros:
    volumes:
      - ./.overlays/$release_name:/opt/astro-overlay:ro
    entrypoint:
      - /bin/bash
      - -c
      - 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && source /opt/astro-overlay/local_setup.bash && exec "\$\$@"'
      - --
EOF
    ssh "$target" "$compose --file '$REMOTE_DIR/docker-compose.overlay.next.yml' up -d && mv '$REMOTE_DIR/docker-compose.overlay.next.yml' '$REMOTE_DIR/docker-compose.overlay.yml'"
    echo "Overlay deployed for: $packages"
    ;;

  overlay-reset)
    current_baseline
    check_image allow-missing
    ssh "$target" "$compose up -d"
    : > "$baseline/packages"
    echo "Baseline service restored. Overlay files were retained on the Pi."
    ;;

  *)
    echo "Usage: overlay.sh overlay-init|overlay-deploy|overlay-reset" >&2
    exit 2
    ;;
esac
