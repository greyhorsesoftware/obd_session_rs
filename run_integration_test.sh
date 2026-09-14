#!/bin/bash
#
# OBD Session RS Integration Test Runner
#
# This script runs the integration test that validates the OBD session functionality
# with the mock platform, testing 27 PIDs including background thread logging.
#
# Usage: ./run_integration_test.sh
#
# The test will:
# - Subscribe to 27 PIDs (25 Mode 01 + 1 Mode 06 + 1 Mode 09)
# - Show COMMAND_SENT and DATA_RECEIVED logging from background threads
# - Rate limit commands to 20/second with burst capacity
# - Verify all responses match expected data from default.json
#

set -e  # Exit on any error

echo "=========================================="
echo "🚗 OBD Session RS Integration Test"
echo "=========================================="
echo

# Clean previous build artifacts
echo "🧹 Cleaning previous build..."
cargo clean --quiet

# Build the project
echo "🔨 Building project..."
cargo build --quiet

# Run the integration test with verbose output to show logging
echo "🧪 Running integration test..."
echo "   (This will test 27 PIDs with background thread logging)"
echo "   - Mode 01: 25 Powertrain Diagnostic Data PIDs"
echo "   - Mode 06: 1 On-board Test Results PID"
echo "   - Mode 09: 1 Vehicle Information PID"
echo "   - Rate limited to 20 commands/second"
echo

# Run test and capture exit code
set +e
cargo test --test integration_test -- --nocapture
test_exit_code=$?
set -e

# Return the test exit code
exit $test_exit_code

echo
echo "=========================================="
echo "✅ Integration test completed successfully!"
echo "=========================================="
