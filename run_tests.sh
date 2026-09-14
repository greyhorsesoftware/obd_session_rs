#!/bin/bash

# OBD Session RS Test Runner
# Usage: ./run_tests.sh [all|ffi|core]
#   all  - Run both FFI and integration tests (default)
#   ffi  - Run only FFI tests
#   core - Run only core integration tests

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default to 'all' if no argument provided
TEST_TYPE=${1:-all}

echo -e "${BLUE}🚗 OBD Session RS Test Runner${NC}"
echo -e "${BLUE}===============================${NC}"
echo -e "${YELLOW}Test type: ${TEST_TYPE}${NC}"
echo

# Function to run a test and check result
run_test() {
    local test_name="$1"
    local test_cmd="$2"

    echo -e "${BLUE}Running ${test_name}...${NC}"
    echo -e "${YELLOW}Command: ${test_cmd}${NC}"

    if eval "$test_cmd"; then
        echo -e "${GREEN}✅ ${test_name} passed${NC}"
        echo
        return 0
    else
        echo -e "${RED}❌ ${test_name} failed${NC}"
        echo
        return 1
    fi
}

# Counter for failed tests
FAILED_TESTS=0

case "$TEST_TYPE" in
    "all")
        echo -e "${BLUE}Running all tests...${NC}"
        echo

        # Run FFI tests
        if ! run_test "FFI Tests" "cargo test --test ffi_test --features ffi-test -- --nocapture"; then
            ((FAILED_TESTS++))
        fi

        # Run integration/core tests
        if ! run_test "Core Integration Tests" "cargo test --test integration_test -- --nocapture"; then
            ((FAILED_TESTS++))
        fi
        ;;

    "ffi")
        echo -e "${BLUE}Running FFI tests only...${NC}"
        echo

        if ! run_test "FFI Tests" "cargo test --test ffi_test --features ffi-test -- --nocapture"; then
            ((FAILED_TESTS++))
        fi
        ;;

    "core")
        echo -e "${BLUE}Running core integration tests only...${NC}"
        echo

        if ! run_test "Core Integration Tests" "cargo test --test integration_test -- --nocapture"; then
            ((FAILED_TESTS++))
        fi
        ;;

    *)
        echo -e "${RED}❌ Invalid test type: ${TEST_TYPE}${NC}"
        echo -e "${YELLOW}Usage: $0 [all|ffi|core]${NC}"
        echo -e "${YELLOW}  all  - Run both FFI and integration tests (default)${NC}"
        echo -e "${YELLOW}  ffi  - Run only FFI tests${NC}"
        echo -e "${YELLOW}  core - Run only core integration tests${NC}"
        exit 1
        ;;
esac

echo -e "${BLUE}===============================${NC}"

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}🎉 All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}💥 ${FAILED_TESTS} test(s) failed${NC}"
    exit 1
fi
