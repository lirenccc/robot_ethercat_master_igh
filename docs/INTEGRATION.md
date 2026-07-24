# IgH hard-RT integration

IgH is a **production-candidate hard-RT motion backend** with the same cycle ownership and fault semantics as the EC-Master path. PDO exchange is owned by Timing + Job after `Master::start()`; `Master::cycle()` only pushes setpoints and reads cached state.

## Cycle ownership

```text
Timing (SCHED_FIFO 99) ──tick──► Job (SCHED_FIFO 98)
                                    │
                                    ├─ receive / domain process
                                    ├─ external command freshness (CST/optional CSV)
                                    ├─ DC sync (domain)
                                    ├─ safe-output OR sendData (merge pending)
                                    ├─ DC monitor → anomaly latch
                                    └─ WKC / deadline / DC anomaly → latch fault

HardwareBus::exchange → Master::cycle()
  └─ push setpoints only (pending_commands_) + read PDO cache
```

After `Master::start()` succeeds, PDO exchange **must not** be driven by `cycle()`.

`IghJobCycleDiag` (when `IGH_DEBUG_LOG=1`) exposes period / lateness / execution / `deadline_met` / `skipped_slots` / WKC / DC fields.

## Safe-output

On `comm_fault` / `safe_output_required`:

- latch actual position, velocity/torque = 0, control word = Shutdown `0x06`
- `cycle()` stops injecting commands and reports fault
- no implicit Fault Reset (`0x0080`) / re-enable

## Cycle skip / deadline metrics

Timing uses `nextCycleDeadline`: overdue slots are **skipped** (no catch-up burst).
Job records lateness / execution / `deadline_met` / `skipped_slots`.

**Trajectory impact:** application / pending merges run once per **executed** Job cycle; wall-clock may advance without intermediate beats.

## Realtime hard constraints

| Variable | Default | Meaning |
| --- | --- | --- |
| `IGH_LOCK_MEMORY` | `1` | `mlockall` at runtime start |
| `IGH_REQUIRE_REALTIME` | `1` | Fail-closed if mlock / `SCHED_FIFO` / affinity fails |
| `IGH_CPU_AFFINITY` | `5` | Timing/Job affinity; `-1` to skip |
| `IGH_BUS_CYCLE_US` | `1000` | Bus cycle µs (align DC SYNC0) |
| `IGH_DEBUG_LOG` | `0` | JobDiag stderr (~every 250 cycles) |

Production systemd: `LimitMEMLOCK=infinity`, `LimitRTPRIO=99`.
Dev without privileges: explicit `IGH_REQUIRE_REALTIME=0` (never silent).

## Anomaly policy + safety reset

| Source | Default policy | Action |
| --- | --- | --- |
| WKC / RX incomplete | consec 5 or window 16/≥8 | `raiseCommFault` → safe-output |
| Deadline miss | consec 10 or window 32/≥16 | same |
| DC out-of-sync | consec 5 or window 16/≥8 | same (after warmup) |

Recovery:

1. Product `/request_safety_reset` → `Master::request_safety_reset()` — clears latch, **does not** auto-enable
2. Healthy dwell (`IGH_HEALTHY_DWELL_CYCLES`, default 50)
3. `setEnable(true)` only when `motion_reenable_allowed()`

| Variable | Default |
| --- | --- |
| `IGH_ANOMALY_WKC_CONSEC` | `5` |
| `IGH_ANOMALY_WKC_WINDOW` | `16` |
| `IGH_ANOMALY_WKC_WINDOW_STOP` | `8` |
| `IGH_ANOMALY_DEADLINE_CONSEC` | `10` |
| `IGH_ANOMALY_DEADLINE_WINDOW` | `32` |
| `IGH_ANOMALY_DEADLINE_WINDOW_STOP` | `16` |
| `IGH_ANOMALY_DC_CONSEC` | `5` |
| `IGH_ANOMALY_DC_WINDOW` | `16` |
| `IGH_ANOMALY_DC_WINDOW_STOP` | `8` |
| `IGH_DC_MONITOR_WARMUP` | `100` |
| `IGH_DC_SYNC_THRESHOLD_NS` | `500000` (0.5 ms) |
| `IGH_HEALTHY_DWELL_CYCLES` | `50` |

## Startup evidence gate

After `activate()` reaches OP:

1. **PDO evidence** — joint modules must expose 0x6041 / 0x6064 / 0x607A (and 0x6040); gateway skipped
2. **SDO gate 0x60C2** — read-only match to `IGH_BUS_CYCLE_US`; profiles with `require_interpolation_period_gate=false` (e.g. SJD17) skip

Failure → observation-only: state readable, `Master::start()` returns false, `setEnable(true)` refused.

## Command freshness (CST / optional CSV)

| Variable | Default | Meaning |
| --- | --- | --- |
| `IGH_CMD_WATCHDOG_MS` | `250` | External CST refresh timeout; `0` disables |
| `IGH_CSV_CMD_WATCHDOG` | `0` | Also arm CSV velocity watchdog |

- External `Master::cycle()` velocity/torque uses `SetpointSource::External` → arms watchdog
- Job file trajectory: `SetpointSource::JobInternal` → **not** armed
- Stale external command → zero torque/velocity + `raiseCommFault`
- Adapter `apply_command_contention_fallback()` → zero armed CST/CSV for one beat without refreshing deadline

## DC monitor

Job samples IgH DC PLL via `EtherCATSync::lastDcDiffNs()` / `isDcPllActive()`.

| JobDiag field | Meaning |
| --- | --- |
| `dc_status_valid` | Reference clock readable and PLL active |
| `dc_in_sync` | `|deviation_ns| ≤ IGH_DC_SYNC_THRESHOLD_NS` |
| `dc_deviation_ns` | Current deviation |
| `max_dc_deviation_ns` | Peak since start |
| `dc_out_of_sync_*` | Anomaly tracker counters |

Healthy dwell requires `dc_ok = !dc_status_valid || dc_in_sync`.

## Capability matrix (EC-Master vs IgH)

| Capability | EC-Master | IgH |
| --- | --- | --- |
| Safe-output | ✅ | ✅ |
| Skip-slot / deadline | ✅ | ✅ |
| mlock / RT fail-closed | ✅ | ✅ |
| PDO / `0x60C2` evidence gate | ✅ | ✅ |
| Anomaly + dwell | ✅ | ✅ |
| CST command freshness | ✅ | ✅ |
| DC out-of-sync tracker | ✅ | ✅ |
| Job cycle ownership | ✅ (Job) | ✅ (`IghMasterRuntime`) |

## Environment

Full list: `config/env.example` (`IGH_*`).

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Cannot start without RT privs | `IGH_REQUIRE_REALTIME=0` explicitly |
| `start()` fails evidence gate | Check PDO map / `0x60C2` vs `IGH_BUS_CYCLE_US`; SJD17 skips SDO gate |
| Stuck after cable pull | `/request_safety_reset` → wait dwell → enable |
| CST drag stops unexpectedly | Check `IGH_CMD_WATCHDOG_MS`; use JobInternal for file trajectories |
| Trajectory time compresses under load | Skip-slot; advance on executed beats |

## HIL checklist (pending)

Bench validation still required for: link loss, deadline overrun, DC drift, evidence gate negative cases, CST timeout, reset dwell before re-enable.
