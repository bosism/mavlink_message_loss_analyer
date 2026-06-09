# Message Loss Debug Console

Standalone Python console for debugging MAVLink telemetry loss in a way that matches the QGroundControl Telemetry settings screen.

## What It Measures

The QGC-style loss counters are sequence-gap based:

- Every successfully decoded MAVLink message increments received count.
- Loss is inferred from missing MAVLink sequence numbers per `(sysid, compid)`.
- Computed sent count is `received + loss`.
- The QGC-style loss rate uses the same running average behavior as QGC.

This is not the same as TCP/UDP packet loss. TCP retransmits can hide transport loss, and bad MAVLink frames are counted separately as parser errors.

## Bandwidth

The console displays two bandwidth numbers:

- `Transport bytes received`: total bytes seen by pymavlink, including malformed input and parser overhead.
- `Decoded MAVLink bytes`: serialized frame bytes for successfully decoded MAVLink messages.

The `Top Message Bandwidth` table shows each message type's decoded bandwidth over the rolling window:

- `B/s`: bytes per second for that message type.
- `share`: percentage of decoded MAVLink bandwidth used by that message type.
- `msg/s`: message rate.
- `total`: cumulative decoded message count.
- `bytes`: cumulative decoded wire bytes for that message type.

## Configure

Edit the constants at the top of `mavlink_status_console.py`.

Common UDP listener:

```python
LINK_MODE = "udp_listen"
UDP_BIND_HOST = "0.0.0.0"
UDP_PORT = 14552
```

Common TCP client:

```python
LINK_MODE = "tcp_client"
TCP_HOST = "127.0.0.1"
TCP_PORT = 5760
```

Leave stream-rate requests disabled for passive observation:

```python
REQUEST_DATA_STREAM = False
SET_MESSAGE_INTERVALS_HZ = {}
```

Enable them only when you want this tool to actively change telemetry rates.

## Run

Install dependency if needed:

```bash
python3 -m pip install pymavlink
```

Run:

```bash
cd /home/nuc/work/message_loss_debug
python3 mavlink_status_console.py
```

or:

```bash
./mavlink_status_console.py
```

Stop with `Ctrl-C`.

## Logs

The app writes:

- `mavlink_status_events.log`: sequence gaps and send errors.
- `mavlink_status_samples.csv`: one status sample per display refresh.

Both files are ignored by git.

## Debug Hints

Large gaps, especially `>= 128`, often indicate a component reboot, sequence reset, out-of-order stream, or multiple producers using the same `(sysid, compid)`.

Parser receive errors indicate malformed MAVLink frames. QGC's Telemetry settings loss counter only reflects gaps between successfully decoded frames.

If QGC shows high loss but this tool does not, check whether both are attached to the same endpoint and whether one connection changes stream rates or routing behavior.
