SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

PI ?=
PI_USER ?= astro
IMAGE ?= astro:humble-arm64
REMOTE_DIR ?= astro-robot
ROS_DOMAIN_ID ?= 10
RMW_IMPLEMENTATION ?= rmw_fastrtps_cpp
BUILD_JOBS ?= 2
PARALLEL_WORKERS ?= 2
SSH_TARGET = $(if $(PI_USER),$(PI_USER)@)$(PI)
REMOTE_COMPOSE = IMAGE='$(IMAGE)' ROS_DOMAIN_ID='$(ROS_DOMAIN_ID)' RMW_IMPLEMENTATION='$(RMW_IMPLEMENTATION)' docker compose --project-name astro --file '$(REMOTE_DIR)/docker-compose.yml'

.PHONY: setup-buildx build ship deploy logs shell check-pi

setup-buildx:
	docker run --privileged --rm tonistiigi/binfmt:qemu-v9.2.2 --install arm64
	docker buildx inspect --bootstrap

build:
	docker buildx build --platform linux/arm64 --load -f DockerFile \
		--build-arg BUILD_JOBS=$(BUILD_JOBS) \
		--build-arg PARALLEL_WORKERS=$(PARALLEL_WORKERS) \
		-t "$(IMAGE)" .

check-pi:
	@test -n "$(PI)" || { echo "Set PI to the Pi host (and optionally PI_USER)." >&2; exit 1; }

ship: check-pi
	docker save "$(IMAGE)" | gzip | ssh "$(SSH_TARGET)" "bash -o pipefail -c 'gzip -dc | docker load'"

deploy: check-pi
	$(MAKE) build IMAGE="$(IMAGE)" BUILD_JOBS="$(BUILD_JOBS)" PARALLEL_WORKERS="$(PARALLEL_WORKERS)"
	$(MAKE) ship PI="$(PI)" PI_USER="$(PI_USER)" IMAGE="$(IMAGE)"
	ssh "$(SSH_TARGET)" "mkdir -p '$(REMOTE_DIR)'"
	ssh "$(SSH_TARGET)" "cat > '$(REMOTE_DIR)/docker-compose.yml'" < docker-compose.yml
	ssh "$(SSH_TARGET)" "$(REMOTE_COMPOSE) up -d"

logs: check-pi
	ssh -t "$(SSH_TARGET)" "$(REMOTE_COMPOSE) logs -f"

shell: check-pi
	ssh -t "$(SSH_TARGET)" "$(REMOTE_COMPOSE) exec ros bash"
