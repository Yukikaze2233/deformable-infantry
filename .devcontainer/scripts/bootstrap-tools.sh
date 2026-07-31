#!/usr/bin/env bash

set -eu

installed_tools=""
skipped_tools=""
manifest_path="/workspaces/RMCS/.devcontainer/.generated/host-tools.manifest"

ensure_mvsusb3core_cmake_package() {
    if [ -f /opt/mvs-usb3-core/lib/cmake/MVSUSB3Core/MVSUSB3CoreConfig.cmake ]; then
        return 0
    fi

    case "$(uname -m)" in
    x86_64)
        arch_dir=64
        ;;
    aarch64)
        arch_dir=aarch64
        ;;
    *)
        printf 'Warning: unsupported architecture for MVSUSB3Core bootstrap: %s\n' "$(uname -m)" >&2
        return 0
        ;;
    esac

    sudo mkdir -p /opt/mvs-usb3-core/lib/cmake/MVSUSB3Core
    sudo tee /opt/mvs-usb3-core/lib/cmake/MVSUSB3Core/MVSUSB3CoreConfig.cmake >/dev/null <<EOF
get_filename_component(MVSUSB3Core_ROOT "\${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

set(MVSUSB3Core_INCLUDE_DIR "\${MVSUSB3Core_ROOT}/include")
set(MVSUSB3Core_INCLUDE_DIRS "\${MVSUSB3Core_INCLUDE_DIR}")
set(MVSUSB3Core_LIBRARY_DIR "\${MVSUSB3Core_ROOT}/lib/${arch_dir}")
set(MVSUSB3Core_LIBRARY "\${MVSUSB3Core_LIBRARY_DIR}/libMvCameraControl.so")
set(MVSUSB3Core_LIBRARIES "\${MVSUSB3Core_LIBRARY}")

if(NOT EXISTS "\${MVSUSB3Core_INCLUDE_DIR}/MvCameraControl.h")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core headers not found under \${MVSUSB3Core_INCLUDE_DIR}")
endif()

if(NOT EXISTS "\${MVSUSB3Core_LIBRARY}")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core library libMvCameraControl.so not found under \${MVSUSB3Core_LIBRARY_DIR}")
endif()

if(NOT TARGET MVSUSB3Core::MVSUSB3Core)
  add_library(MVSUSB3Core::MVSUSB3Core SHARED IMPORTED GLOBAL)
  set_target_properties(MVSUSB3Core::MVSUSB3Core PROPERTIES
    IMPORTED_LOCATION "\${MVSUSB3Core_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "\${MVSUSB3Core_INCLUDE_DIR}"
  )
endif()

set(MVSUSB3Core_FOUND TRUE)
EOF
}

ensure_ros_mavlink_headers() {
    if [ -f /opt/ros/jazzy/include/mavlink/v2.0/common/mavlink.h ]; then
        return 0
    fi

    printf 'Installing ros-jazzy-mavlink for rmcs_core...\n'
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends ros-jazzy-mavlink
    sudo apt-get clean
    sudo rm -rf /var/lib/apt/lists/*
}

ensure_mvsusb3core_cmake_package
ensure_ros_mavlink_headers

if [ ! -f "$manifest_path" ]; then
    printf 'Warning: host tools manifest not found at %s\n' "$manifest_path" >&2
fi

if ! command -v npm >/dev/null 2>&1; then
    printf 'Warning: npm is not available in the container, skipping host tool bootstrap.\n' >&2
    exit 0
fi

add_item() {
    local current_list="$1"
    local item="$2"

    if [ -z "$current_list" ]; then
        printf '%s' "$item"
    else
        printf '%s\n%s' "$current_list" "$item"
    fi
}

install_tool() {
    local tool_name="$1"
    local package_name="$2"

    if [ ! -f "$manifest_path" ]; then
        skipped_tools=$(add_item "$skipped_tools" "$tool_name (host manifest missing)")
        return 0
    fi

    local manifest_line=""
    local version=""

    manifest_line=$(grep -m 1 -E "^${tool_name}=" "$manifest_path" 2>/dev/null || true)
    if [ -z "$manifest_line" ]; then
        skipped_tools=$(add_item "$skipped_tools" "$tool_name (not installed on host)")
    else
        version=${manifest_line#*=}
        if printf '%s' "$version" | grep -Eq '^(v)?[0-9]+([.][0-9]+)*([-.][0-9A-Za-z]+)*$'; then
            version=${version#v}
            if sudo npm install -g "$package_name@$version"; then
                installed_tools=$(add_item "$installed_tools" "$tool_name@$version")
            else
                skipped_tools=$(add_item "$skipped_tools" "$tool_name (install failed for $version)")
            fi
        else
            printf 'Warning: skipping %s because version could not be parsed on host\n' "$tool_name" >&2
            skipped_tools=$(add_item "$skipped_tools" "$tool_name (version could not be parsed on host)")
        fi
    fi
}

install_tool "codex" "@openai/codex"
install_tool "claude" "@anthropic-ai/claude-code"
install_tool "opencode" "opencode-ai"
install_tool "lark-cli" "@larksuite/cli"

printf 'Bootstrap summary:\n'
if [ -n "$installed_tools" ]; then
    printf 'Installed:\n%s\n' "$installed_tools"
else
    printf 'Installed: none\n'
fi

if [ -n "$skipped_tools" ]; then
    printf 'Skipped:\n%s\n' "$skipped_tools"
else
    printf 'Skipped: none\n'
fi

exit 0
