# Serial Protocol

The optional Arduino receiver uses newline-terminated ASCII commands:

```text
J,<base_deg>,<shoulder_deg>,<elbow_deg>\n
```

Example:

```text
J,90.0,80.0,110.0
```

All three values must be within 0–180° in the reference firmware. Host-side calibration should enforce tighter mechanical limits before transmission.

For a more advanced arm, add acknowledgements, sequence numbers, velocity limits, and a watchdog rather than relying on open-loop position writes.
