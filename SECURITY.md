# Security

This crate talks to OBD-II adapters over serial, TCP, BLE and Bluetooth
Classic and parses whatever the adapter and the vehicle send back. The main
risk surfaces are a malformed or hostile byte stream causing a panic, unbounded
memory, or a hang in a host application, and — because the library can put
arbitrary commands on a vehicle bus — any path in a host that lets untrusted
input reach `send_command`.

If you find such a case, or anything else you believe is a security issue,
email **info@greyhorsesoftware.com** rather than opening a public issue.
Include the adapter, the transport, and the wire log (`adapter_<ts>.log`) with
the VIN redacted. We aim to acknowledge within a week.
