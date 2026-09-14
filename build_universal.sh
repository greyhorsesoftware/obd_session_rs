#!/bin/bash

# build_universal.sh - Build libraries for obd_session_rs
#
# This script builds libraries for iOS and macOS (Apple Silicon only) platforms.
# macOS builds target deployment target 15.2.
# All build commands run tests first and abort if tests fail.
#
# Usage:
#   ./build_universal.sh debug      # Build debug library (runs tests first)
#   ./build_universal.sh release    # Build release library (runs tests first)
#   ./build_universal.sh ios        # Build iOS universal library
#   ./build_universal.sh macos      # Build macOS universal library
#   ./build_universal.sh all        # Build both iOS and macOS
#   ./build_universal.sh clean      # Clean build artifacts
#   ./build_universal.sh            # Interactive mode

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project configuration
PROJECT_NAME="obd_session_rs"
LIBRARY_NAME="lib${PROJECT_NAME}.a"
HEADER_NAME="${PROJECT_NAME}.h"
OUTPUT_LIB_DIR="${OBD_LIB_DIR:-../lib}"  # where the .a + header land (override with OBD_LIB_DIR)

# Function to print colored output
print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# Function to generate C header file using cbindgen
generate_header() {
    local output_dir=$1
    
    print_info "Generating C header file..."
    
    # Check if cbindgen is installed
    if ! command -v cbindgen &> /dev/null; then
        print_warning "cbindgen not found. Installing..."
        cargo install cbindgen
    fi
    
    mkdir -p "${output_dir}"
    
    if cbindgen --config cbindgen.toml --crate "${PROJECT_NAME}" --output "${output_dir}/${HEADER_NAME}"; then
        # Stamp the crate version (source of truth: Cargo.toml) into the header;
        # the same version is embedded in the .a via obd_session_version().
        local crate_version
        crate_version=$(grep -m1 '^version' Cargo.toml | cut -d'"' -f2)
        printf '/* %s version %s (from Cargo.toml) */\n' "${PROJECT_NAME}" "${crate_version}" \
            | cat - "${output_dir}/${HEADER_NAME}" > "${output_dir}/${HEADER_NAME}.tmp" \
            && mv "${output_dir}/${HEADER_NAME}.tmp" "${output_dir}/${HEADER_NAME}"
        print_status "Generated header: ${output_dir}/${HEADER_NAME} (v${crate_version})"
    else
        print_warning "Header generation failed (FFI functions may not be exported)"
    fi
}

# Function to run all tests
run_tests() {
    print_info "Running tests..."
    
    if cargo test --lib 2>&1; then
        print_status "All library tests passed"
    else
        print_error "Tests failed! Aborting build."
        exit 1
    fi
    
    if cargo test --test integration_test 2>&1; then
        print_status "Integration tests passed"
    else
        print_error "Integration tests failed! Aborting build."
        exit 1
    fi

    if cargo test --test ffi_test --features ffi-test 2>&1; then
        print_status "FFI tests passed"
    else
        print_error "FFI tests failed! Aborting build."
        exit 1
    fi
    
    print_status "All tests passed"
}

# Resolve where cargo actually writes artifacts for a given profile.
# Respects CARGO_BUILD_TARGET env var and the [build] target setting in
# .cargo/config.toml — without this, copying from the bare target/<profile>/
# path picks up stale artifacts when a default target triple is configured.
resolve_target_dir() {
    local profile=$1
    local triple=""

    if [[ -n "${CARGO_BUILD_TARGET:-}" ]]; then
        triple="${CARGO_BUILD_TARGET}"
    elif [[ -f .cargo/config.toml ]]; then
        triple=$(awk '
            /^\[build\]/ { in_build=1; next }
            /^\[/        { in_build=0 }
            in_build && /^[[:space:]]*target[[:space:]]*=/ {
                sub(/^[^=]*=[[:space:]]*/, "")
                gsub(/^"|"$/, "")
                print
                exit
            }
        ' .cargo/config.toml)
    fi

    if [[ -n "${triple}" ]]; then
        echo "target/${triple}/${profile}"
    else
        echo "target/${profile}"
    fi
}

# Function to check if target is installed
check_target() {
    local target=$1
    if ! rustup target list --installed | grep -q "^${target}$"; then
        print_error "Target '${target}' is not installed."
        print_info "Install with: rustup target add ${target}"
        exit 1
    fi
}

# Function to build for a specific target
build_target() {
    local target=$1
    local description=$2

    print_info "Building for ${description} (${target})..."

    if ! cargo build --release --target "${target}"; then
        print_error "Failed to build for ${target}"
        exit 1
    fi

    print_status "Built for ${target}"
}

# Copy one iOS target's .a + header into its per-SDK lib dir
stage_ios_lib() {
    local target=$1
    local subdir=$2
    local output_dir="${OUTPUT_LIB_DIR}/${subdir}"

    mkdir -p "${output_dir}"
    cp "target/${target}/release/${LIBRARY_NAME}" "${output_dir}/"
    print_status "Copied: ${output_dir}/${LIBRARY_NAME}"

    # Generate header file
    generate_header "${output_dir}"
}

# Function to build iOS libraries (arm64 device + arm64 simulator).
# Device and simulator are the SAME arch on different platforms, so they
# can't be lipo'd into one fat lib — each SDK gets its own output dir
# (lib/ios-device, lib/ios-sim), selected via per-SDK
# LIBRARY_SEARCH_PATHS in Xcode.
# Pass "skip_tests" as argument to skip test run
build_ios() {
    if [[ "$1" != "skip_tests" ]]; then
        run_tests
    fi

    export IPHONEOS_DEPLOYMENT_TARGET="18.2"
    print_info "Building iOS libraries (deployment target: iOS ${IPHONEOS_DEPLOYMENT_TARGET})..."

    # Check targets
    check_target "aarch64-apple-ios"
    check_target "aarch64-apple-ios-sim"

    # Build targets
    build_target "aarch64-apple-ios" "iOS device"
    build_target "aarch64-apple-ios-sim" "iOS simulator"

    # Stage per-SDK outputs (plain .a per SDK, same pattern as lib/release)
    stage_ios_lib "aarch64-apple-ios" "ios-device"
    stage_ios_lib "aarch64-apple-ios-sim" "ios-sim"

    print_status "iOS libraries ready:"
    print_info "  - ${OUTPUT_LIB_DIR}/ios-device/${LIBRARY_NAME}"
    print_info "  - ${OUTPUT_LIB_DIR}/ios-sim/${LIBRARY_NAME}"
}

# Function to build macOS Apple Silicon library
# Pass "skip_tests" as argument to skip test run
build_macos() {
    if [[ "$1" != "skip_tests" ]]; then
        run_tests
    fi

    print_info "Building macOS Apple Silicon library (aarch64, deployment target 15.2)..."

    # Check target
    check_target "aarch64-apple-darwin"

    # Build for Apple Silicon with macOS 15.2 deployment target
    export MACOSX_DEPLOYMENT_TARGET="15.2"
    build_target "aarch64-apple-darwin" "macOS Apple Silicon"

    # Copy library to output directory
    local output_dir="${OUTPUT_LIB_DIR}/macos"
    mkdir -p "${output_dir}"
    cp "target/aarch64-apple-darwin/release/${LIBRARY_NAME}" "${output_dir}/"
    print_status "Copied: ${output_dir}/${LIBRARY_NAME}"

    # Generate header file
    generate_header "${output_dir}"

    print_status "macOS Apple Silicon library ready:"
    print_info "  - ${output_dir}/${LIBRARY_NAME}"
    print_info "  - ${output_dir}/${HEADER_NAME}"
}

# Function to clean build artifacts
clean() {
    print_info "Cleaning build artifacts..."

    # Remove target directories for cross-compilation targets
    rm -rf target/aarch64-apple-ios
    rm -rf target/aarch64-apple-ios-sim
    rm -rf target/aarch64-apple-darwin

    # NOTE: ${OUTPUT_LIB_DIR} is deliberately left alone — it holds the latest
    # built libraries/headers (shared with obd_equation_rs) that the Xcode
    # build links against.

    # Also clean regular debug/release builds
    cargo clean

    print_status "Cleaned intermediate build artifacts (kept ${OUTPUT_LIB_DIR})"
}

# Function to build debug library for current platform
# Pass "skip_tests" as argument to skip test run
build_debug() {
    if [[ "$1" != "skip_tests" ]]; then
        run_tests
    fi

    export MACOSX_DEPLOYMENT_TARGET="15.2"
    print_info "Building debug library (deployment target: macOS ${MACOSX_DEPLOYMENT_TARGET})..."
    cargo build

    mkdir -p "${OUTPUT_LIB_DIR}/debug"

    local debug_target_dir
    debug_target_dir=$(resolve_target_dir debug)

    # Copy static library
    if [[ -f "${debug_target_dir}/${LIBRARY_NAME}" ]]; then
        cp "${debug_target_dir}/${LIBRARY_NAME}" "${OUTPUT_LIB_DIR}/debug/"
        print_status "Copied: ${OUTPUT_LIB_DIR}/debug/${LIBRARY_NAME} (from ${debug_target_dir})"
    else
        print_warning "Static library not found in ${debug_target_dir}/"
    fi
    
    # Generate header file
    generate_header "${OUTPUT_LIB_DIR}/debug"
    
    print_status "Debug library ready: ${OUTPUT_LIB_DIR}/debug/"
}

# Function to build release library for current platform
# Pass "skip_tests" as argument to skip test run
build_release() {
    if [[ "$1" != "skip_tests" ]]; then
        run_tests
    fi

    export MACOSX_DEPLOYMENT_TARGET="15.2"
    print_info "Building release library (deployment target: macOS ${MACOSX_DEPLOYMENT_TARGET})..."
    cargo build --release

    mkdir -p "${OUTPUT_LIB_DIR}/release"

    local release_target_dir
    release_target_dir=$(resolve_target_dir release)

    # Copy static library
    if [[ -f "${release_target_dir}/${LIBRARY_NAME}" ]]; then
        cp "${release_target_dir}/${LIBRARY_NAME}" "${OUTPUT_LIB_DIR}/release/"
        print_status "Copied: ${OUTPUT_LIB_DIR}/release/${LIBRARY_NAME} (from ${release_target_dir})"
    else
        print_warning "Static library not found in ${release_target_dir}/"
    fi
    
    # Generate header file
    generate_header "${OUTPUT_LIB_DIR}/release"
    
    print_status "Release library ready: ${OUTPUT_LIB_DIR}/release/"
}

# Function to show usage
usage() {
    echo "Usage: $0 [debug|release|ios|macos|all|clean]"
    echo ""
    echo "Commands:"
    echo "  debug   Build debug library for current platform"
    echo "  release Build release library for current platform"
    echo "  ios     Build iOS libraries (arm64 device + arm64 simulator, per-SDK dirs)"
    echo "  macos   Build macOS Apple Silicon library (aarch64, deployment target 15.2)"
    echo "  all     Build both iOS and macOS libraries"
    echo "  clean   Remove intermediate build artifacts (keeps lib/ outputs)"
    echo ""
    echo "Output directory: ${OUTPUT_LIB_DIR}"
    echo ""
    echo "Note: All build commands run tests first. Build aborts if tests fail."
    echo ""
    echo "If no command is specified, interactive mode will be used."
    echo ""
    echo "Prerequisites:"
    echo "  - Install iOS targets: rustup target add aarch64-apple-ios aarch64-apple-ios-sim"
    echo "  - Install macOS target: rustup target add aarch64-apple-darwin"
    echo "  - Xcode command line tools must be installed"
}

# Interactive mode
interactive() {
    echo "Library Builder for ${PROJECT_NAME}"
    echo "========================================"
    echo "Output directory: ${OUTPUT_LIB_DIR}"
    echo ""
    echo "Available options:"
    echo "1) Build debug library (current platform)"
    echo "2) Build release library (current platform)"
    echo "3) Build iOS libraries (per-SDK dirs)"
    echo "4) Build macOS Apple Silicon library"
    echo "5) Build all libraries"
    echo "6) Clean build artifacts"
    echo "7) Exit"
    echo ""

    while true; do
        read -p "Choose an option (1-7): " choice
        case $choice in
            1) build_debug; break ;;
            2) build_release; break ;;
            3) build_ios; break ;;
            4) build_macos; break ;;
            5) build_ios && build_macos; break ;;
            6) clean; break ;;
            7) exit 0 ;;
            *) print_warning "Invalid option. Please choose 1-7." ;;
        esac
    done
}

# Main script logic
main() {
    local command=$1

    # Check if we're in the right directory
    if [[ ! -f "Cargo.toml" ]]; then
        print_error "Please run this script from the project root directory (where Cargo.toml is located)"
        exit 1
    fi

    # Check if lipo is available (macOS development tool)
    if ! command -v lipo &> /dev/null; then
        print_error "lipo command not found. Please install Xcode command line tools."
        print_info "Run: xcode-select --install"
        exit 1
    fi

    case $command in
        "debug")
            build_debug
            ;;
        "release")
            build_release
            ;;
        "ios")
            build_ios
            ;;
        "macos")
            build_macos
            ;;
        "all")
            # Run tests once at the start
            run_tests
            # Build both with skip_tests flag
            build_ios skip_tests
            echo ""
            build_macos skip_tests
            ;;
        "clean")
            clean
            ;;
        "")
            interactive
            ;;
        *)
            usage
            exit 1
            ;;
    esac

    echo ""
    print_status "Build script completed successfully!"
}

# Run main function with all arguments
main "$@"
