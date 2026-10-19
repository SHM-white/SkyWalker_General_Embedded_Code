# Execution status contracts

Current production code publishes `RunStatus` snapshots and transforms them to wire feedback. Tests assert multi-reader non-consuming snapshots, immutable producer ages, failure without destination mutation, all wait-reason mappings, endpoint fail-closed invalid configuration and an actual endpoint/transport/link round trip. Obsolete pre-refactor `RecoveryGate` scenarios have been removed because that class no longer exists.
