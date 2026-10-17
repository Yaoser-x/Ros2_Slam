#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v vcs >/dev/null 2>&1; then
    echo "vcs command not found. Install python3-vcstool on the Raspberry Pi." >&2
    exit 1
fi

mkdir -p vendor src/vendor
vcs import . < deps/ydlidar-sdk.repos
vcs import . < deps/ydlidar-ros2-driver.repos

apply_patch_once() {
    local repo_dir="$1"
    local patch_file="$2"

    # First try to apply with --ignore-space-change and --ignore-whitespace
    if git -C "$repo_dir" apply --ignore-space-change --ignore-whitespace --check "$patch_file" >/dev/null 2>&1; then
        git -C "$repo_dir" apply --ignore-space-change --ignore-whitespace "$patch_file"
        # After patch succeeds, normalise file modes (only if no changes would be created)
        find "$repo_dir" -type f -not -name '*.sh' -not -name '*.py' -exec chmod 644 {} + 2>/dev/null || true
        return
    fi

    # Check if already applied
    if git -C "$repo_dir" apply --ignore-space-change --ignore-whitespace --reverse --check "$patch_file" >/dev/null 2>&1; then
        echo "Patch already applied: $patch_file"
        # Still normalise file modes to be safe
        find "$repo_dir" -type f -not -name '*.sh' -not -name '*.py' -exec chmod 644 {} + 2>/dev/null || true
        return
    fi

    if patch_present_by_markers "$repo_dir" "$patch_file"; then
        echo "Patch already applied: $patch_file"
        find "$repo_dir" -type f -not -name '*.sh' -not -name '*.py' -exec chmod 644 {} + 2>/dev/null || true
        return
    fi

    echo "Patch cannot be applied cleanly: $patch_file" >&2
    exit 1
}

patch_present_by_markers() {
    local repo_dir="$1"
    local patch_file="$2"

    case "$patch_file" in
        *ydlidar-ros2-driver-local-safety.patch)
            grep -Fq "time_increment <= 0.0f" "$repo_dir/src/ydlidar_ros2_driver_client.cpp" &&
                grep -Fq "static_cast<int>(scan->ranges.size())" "$repo_dir/src/ydlidar_ros2_driver_client.cpp" &&
                grep -Fq "std::abs(scan.config.angle_increment) < 1e-9f" "$repo_dir/src/ydlidar_ros2_driver_node.cpp" &&
                grep -Fq "computed size %d is out of valid range" "$repo_dir/src/ydlidar_ros2_driver_node.cpp"
            ;;
        *)
            return 1
            ;;
    esac
}

if [ -d vendor/ydlidar-sdk ]; then
    apply_patch_once vendor/ydlidar-sdk ../../deps/patches/ydlidar-sdk-modern-cmake.patch
    apply_patch_once vendor/ydlidar-sdk ../../deps/patches/ydlidar-sdk-fix-install-export.patch
    apply_patch_once vendor/ydlidar-sdk ../../deps/patches/ydlidar-sdk-scan-framing.patch
    touch vendor/ydlidar-sdk/COLCON_IGNORE
fi

if [ -d src/vendor/ydlidar_ros2_driver ]; then
    apply_patch_once src/vendor/ydlidar_ros2_driver ../../../deps/patches/ydlidar-ros2-driver-local-safety.patch
fi
