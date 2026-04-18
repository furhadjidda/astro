# ==============================================================================
# Astro — justfile
# ==============================================================================
#
# Install just: https://github.com/casey/just#installation
#   cargo install just    OR    brew install just    OR    sudo apt install just
#
# Usage:
#   just                             Show all recipes
#
# Pico firmware:
#   just pico pico_w                 Build for Pico W  (RP2040)
#   just pico pico2_w                Build for Pico 2W (RP2350)
#   just pico-all                    Build both Pico boards
#   just pico-test                   Build & run host unit tests
#
# Zephyr firmware:
#   just zephyr app_sensor_node rak        Build for RAK3112
#   just zephyr app_sensor_node feather    Build for Adafruit Feather
#   just zephyr-all                        Build all apps for all boards
#   just zephyr-init                       Initialize west workspace
#
# ROS 2:
#   just ros2                        Build ROS 2 workspace (native)
#   just ros2-deps                   Import repos + install rosdep deps
#   just ros2-pi4                    Cross-compile for Pi4 (Docker)
#   just ros2-pi4 ubuntu@10.0.0.1   Cross-compile + deploy to Pi4
#
# Clean:
#   just clean                       Clean ALL build artifacts
#   just clean-pico                  Clean Pico builds
#   just clean-zephyr                Clean Zephyr builds
#   just clean-zephyr-microros       Clean Zephyr micro-ROS library
#   just clean-zephyr-all            Clean Zephyr builds + micro-ROS
#   just clean-ros2                  Clean ROS 2 builds
#   just clean-ros2-pi4              Clean Pi4 cross-compile builds
#
# ==============================================================================

set shell := ["bash", "-euo", "pipefail", "-c"]

# Repo root (resolved once)
root         := justfile_directory()

# Directory shortcuts
pico_dir     := "firmware/pico"
zephyr_dir   := "firmware/zephyr"
ros2_dir     := "ros2"
pico_sdk     := root / pico_dir / "external/pico-sdk"
microros_dir := zephyr_dir / "astro_custom_modules/libmicroros"

# Zephyr apps (excluding app_examples which needs snippet args)
zephyr_apps  := "app_sensor_node app_microros_node app_bluetooth app_sd_card_fs"

# ==============================================================================
# Default: show help
# ==============================================================================

@_default:
    just --list --unsorted

# ==============================================================================
# Pico firmware
# ==============================================================================

# Build Pico firmware for a board (pico_w or pico2_w)
[group('pico')]
pico board:
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{ board }}" in
        pico_w)   pico_board=pico_w  platform=""     mcu=cortex-m0  build_dir=build_pico_w  ;;
        pico2_w)  pico_board=pico2_w platform=rp2350 mcu=cortex-m33 build_dir=build_pico2_w ;;
        *)        pico_board={{ board }} platform=""  mcu=""         build_dir=build_{{ board }} ;;
    esac

    cmake_args="-DPICO_BOARD=${pico_board} -DPICO_SDK_PATH={{ pico_sdk }}"
    [[ -n "$mcu" ]]      && cmake_args+=" -DMCU_TYPE=${mcu}"
    [[ -n "$platform" ]] && cmake_args+=" -DPICO_PLATFORM=${platform}"

    cmake -S {{ pico_dir }} -B {{ pico_dir }}/${build_dir} ${cmake_args}
    cmake --build {{ pico_dir }}/${build_dir}

# Build Pico firmware for all boards
[group('pico')]
pico-all:
    just pico pico_w
    just pico pico2_w

# Build and run Pico host unit tests
[group('pico')]
pico-test:
    cmake -S {{ pico_dir }} -B {{ pico_dir }}/host_build -DBUILD_FOR_HOST=ON
    cmake --build {{ pico_dir }}/host_build
    cd {{ pico_dir }}/host_build && ctest --output-on-failure

# ==============================================================================
# Zephyr firmware
# ==============================================================================

# Private: validate board name
[private]
[no-exit-message]
@_zephyr-board-check board:
    case "{{ board }}" in \
        rak|feather) ;; \
        *) echo "Error: BOARD must be 'rak' or 'feather', got '{{ board }}'"; exit 1 ;; \
    esac

# Private: emit shell variable assignments for a board name
[private]
_zephyr-board-vars board:
    @case "{{ board }}" in \
        rak) \
            echo 'zephyr_board=rak3112/esp32s3/procpu'; \
            echo 'overlay=boards/rak_wireless_rak3312.overlay'; \
            echo 'conf=boards/rak_wireless_rak3312.conf'; \
            echo 'flash_snippet=flash-16M'; \
            echo 'psram_snippet=psram-8M'; \
            ;; \
        feather) \
            echo 'zephyr_board=adafruit_feather_esp32s3/esp32s3/procpu'; \
            echo 'overlay=boards/adafruit_feather_s3.overlay'; \
            echo 'conf=boards/adafruit_feather_s3.conf'; \
            echo 'flash_snippet=flash-4M'; \
            echo 'psram_snippet=psram-2M'; \
            ;; \
    esac

# Initialize Zephyr west workspace
[group('zephyr')]
zephyr-init:
    #!/usr/bin/env bash
    set -euo pipefail
    cd {{ zephyr_dir }}
    if [[ ! -d ".west" ]]; then
        west init -l west
        west update --narrow
    fi
    west packages pip --install
    west blobs fetch hal_espressif

# Build a Zephyr app (e.g. just zephyr app_sensor_node rak)
[group('zephyr')]
zephyr app board: (_zephyr-board-check board)
    #!/usr/bin/env bash
    set -euo pipefail
    eval "$(just _zephyr-board-vars {{ board }})"
    cd {{ zephyr_dir }}
    west build -p always \
        -b "${zephyr_board}" \
        -d "build/{{ app }}" \
        --sysbuild \
        {{ app }} \
        -- -DDTC_OVERLAY_FILE="${overlay}" -DEXTRA_CONF_FILE="${conf}"

# Build app_examples with flash/psram snippets
[group('zephyr')]
zephyr-examples board: (_zephyr-board-check board)
    #!/usr/bin/env bash
    set -euo pipefail
    eval "$(just _zephyr-board-vars {{ board }})"
    cd {{ zephyr_dir }}
    west build -p always \
        -b "${zephyr_board}" \
        -d build/app_examples \
        -S "${flash_snippet}" -S "${psram_snippet}" \
        --sysbuild \
        app_examples \
        -- -DDTC_OVERLAY_FILE="${overlay}" -DEXTRA_CONF_FILE="${conf}"

# Build all Zephyr apps for all boards
[group('zephyr')]
zephyr-all:
    #!/usr/bin/env bash
    set -euo pipefail
    for board in rak feather; do
        for app in {{ zephyr_apps }}; do
            echo "=== Building ${app} for ${board} ==="
            just zephyr "${app}" "${board}"
        done
        echo "=== Building app_examples for ${board} ==="
        just zephyr-examples "${board}"
    done

# ==============================================================================
# ROS 2
# ==============================================================================

# Import repos + install rosdep dependencies
[group('ros2')]
ros2-deps:
    cd {{ ros2_dir }} && vcs import . < astro.repos
    bash -c 'source /opt/ros/humble/setup.bash && cd {{ ros2_dir }} && rosdep install --from-paths src --ignore-src -r -y'

# Build ROS 2 workspace (native)
[group('ros2')]
ros2:
    bash -c 'cd {{ ros2_dir }} && colcon build --event-handlers console_direct+'

# Cross-compile ROS 2 for Pi4 (optionally deploy: just ros2-pi4 ubuntu@<ip>)
[group('ros2')]
ros2-pi4 pi="":
    #!/usr/bin/env bash
    set -euo pipefail
    cd {{ ros2_dir }}
    if [[ -n "{{ pi }}" ]]; then
        python3 deploy/build_and_deploy_pi4.py --pi "{{ pi }}" --first-run
    else
        python3 deploy/build_and_deploy_pi4.py --skip-deploy --first-run
    fi

# ==============================================================================
# Clean targets
# ==============================================================================

# Clean all Pico build directories
[group('clean'), confirm("Remove all Pico build dirs?")]
clean-pico:
    rm -rf {{ pico_dir }}/build_pico_w {{ pico_dir }}/build_pico2_w {{ pico_dir }}/host_build

# Clean Zephyr build outputs
[group('clean'), confirm("Remove Zephyr build dir?")]
clean-zephyr:
    rm -rf {{ zephyr_dir }}/build

# Clean Zephyr micro-ROS library artifacts
[group('clean'), confirm("Remove micro-ROS library artifacts?")]
clean-zephyr-microros:
    rm -rf {{ microros_dir }}/libmicroros.a \
           {{ microros_dir }}/include \
           {{ microros_dir }}/micro_ros_dev \
           {{ microros_dir }}/micro_ros_src \
           {{ microros_dir }}/configured_colcon.meta \
           {{ microros_dir }}/zephyr_toolchain.cmake

# Clean Zephyr builds + micro-ROS library
[group('clean'), confirm("Remove ALL Zephyr artifacts (builds + micro-ROS)?")]
clean-zephyr-all: clean-zephyr clean-zephyr-microros

# Clean ROS 2 native build outputs
[group('clean'), confirm("Remove ROS 2 build/install/log dirs?")]
clean-ros2:
    rm -rf {{ ros2_dir }}/build {{ ros2_dir }}/install {{ ros2_dir }}/log

# Clean Pi4 cross-compile outputs
[group('clean'), confirm("Remove Pi4 cross-compile dirs?")]
clean-ros2-pi4:
    rm -rf {{ ros2_dir }}/build_pi4 {{ ros2_dir }}/install_pi4 {{ ros2_dir }}/log_pi4

# Clean ALL build artifacts
[group('clean'), confirm("Remove ALL build artifacts across pico, zephyr, and ros2?")]
clean: clean-pico clean-zephyr clean-zephyr-microros clean-ros2 clean-ros2-pi4
    @echo "All build artifacts cleaned."
