# BYD Seal — openpilot Longitudinal Control via Case A Harness Re-wire

> **Status: ON HOLD** — saved for later use. Current state: MADS (lateral-only)
> is fully working; stock ACC handles longitudinal.
>
> Car: BYD Seal Performance 2025, comma 3X, 26-pin harness
> Repo: `github.com/Quintine/sunnypilot-byd`, branch `staging`

---

## 1. Goal

Enable openpilot longitudinal control (replacing stock ACC) by re-routing the
CAN pair that carries `ACC_HUD_ADAS` / `ACC_CMD` / `ACC_AEB` (813 / 814 / 815)
through the panda, so it can **block stock ACC_CMD and inject openpilot's**,
while keeping the ADAS ECU alive (AEB, FCW, HUD) and restoring full stock
behavior when the device is off.

## 2. Why it's needed (measured bus topology)

From a live dump on the Seal (`tools/byd_can_dump.py`, route
`00000022--3a1ad65541--1`):

| Message | Han topology | Seal topology (measured) |
|---|---|---|
| ACC_HUD_ADAS (813, cruise state) | bus 2 (camera) | **bus 0 (vehicle CAN)** |
| ACC_CMD (814, stock ACC echo) | bus 2 | **bus 0** |
| ACC_AEB (815) | bus 2 | **bus 0** |
| Steering feedback | 792 STEERING_TORQUE | **508 STEERING_TORQUE_ANGLE** |
| PCM_BUTTONS (944, LKAS btn) | ? | **bus 0 @ 20 Hz** |
| MPC_LKAS_CMD (790, torque cmd) | bus 2 | bus 2 |
| MPC_LKAS_CMD_ANGLE (482, angle cmd) | — | bus 2 @ 50 Hz |
| panda CAN1 | — | **silent / unused** |

Because stock `ACC_CMD` (814) is **born on bus 0** — the same bus the ESP
listens to — the panda cannot intercept it with the current wiring. The only
fix is to physically move that pair through the panda.

## 3. Target end state

```
                ┌──────────────────────────────────────────┐
 LKAS pair  ════╡ CAN2 (camera)   panda (in comma 3X)      │
 (790/482)      │                    CAN0 (vehicle CAN)    ╞════ vehicle CAN
 ADAS pair ════╡ CAN1 (ADAS ECU)                           │
 (813/14/15)    └──────────────────────────────────────────┘
   relay restores BOTH pairs when device is off/unpowered
```

- **CAN1 → CAN0**: forward 813, 815 (AEB/FCW + HUD stay alive); **block 814**;
  panda substitutes its own 814 on CAN0 (code already exists in `bydcan.py`)
- **CAN0 → CAN1**: forward vehicle traffic so the ADAS ECU does not fault
- **CAN2 ↔ CAN0**: unchanged (LKAS interception, working today)

---

## 4. Phase 0 — Research & feasibility (no car modification)

Four unknowns decide everything:

- [ ] **0.1 Harness origin** — where did the 26-pin harness come from
  (comma BYD kit? HotIce0's design? self-made?) and is there a wiring diagram?
  Determines available relay contacts / free channels.
- [ ] **0.2 Is 813/814/815 on a dedicated ADAS pair or the shared vehicle CAN?**
  *Non-destructive experiment:* T-tap panda CAN1 onto the candidate pair
  (do NOT cut it), run `tools/byd_can_dump.py --live 15` (panda noOutput =
  listen-only). If 813/814/815 appear on **bus 1**, the pair is found.
- [ ] **0.3 Does the ADAS ECU need inbound traffic on that pair?**
  (does traffic flow vehicle→ADAS on it today?) If yes, CAN0→CAN1 forwarding
  is mandatory (panda firmware work in Phase 2).
- [ ] **0.4 Stock restore on device-off** — the stock comma harness box switches
  **one** pair through a relay. A second intercepted pair needs a multi-pole
  relay on the same panda-driven relay line (standard community dual-tap
  approach), otherwise unplugging the device leaves ADAS permanently dead.
  Verify relay contacts with a multimeter (continuity per pair, device
  powered vs unpowered).

**GATE:** dedicated pair exists (or ADAS separable at a connector) AND relay
solution exists → proceed. Otherwise stay MADS-only (already working).

## 5. Phase 1 — Wiring (hardware)

1. Break the ADAS pair at the harness: **ADAS-ECU side → panda CAN1**,
   **vehicle side → CAN0** (joins the vehicle CAN net).
2. Multi-pole relay (or second coil paralleled on the existing relay drive
   line) so **both** the LKAS pair and ADAS pair restore on device-off.
3. **Termination check** (multimeter, car off): each broken segment should
   read ~60 Ω across CAN-H/L (two 120 Ω terminators). If a segment reads
   ~120 Ω or open, add a 120 Ω resistor at the harness on that side.
4. Bench-verify before driving: device on, `byd_can_dump.py --live` →
   813/815 forwarded to bus 0, 814 absent on bus 0 (until openpilot sends),
   no ADAS DTCs on dash.

## 6. Phase 2 — Panda firmware

1. **Forwarding architecture**: panda's `get_fwd_bus` only supports the fixed
   `{0→2, 2→0}` route — bus 1 forwarding does not exist. Add a small
   **per-mode forwarding route table** (default `{0→2, 2→0}` = zero change for
   all other cars; BYD long gets `{0→2, 2→0, 1→0, 0→1}`), with the BYD fwd
   hook blocking 814 on the 1→0 route.
2. BYD long safety config: rx checks for 813 on bus 1; keep TX `{814, 0}`;
   extend fwd-hook test infra for multi-route tables; full libsafety suite +
   MISRA; rebuild `panda_h7.bin.signed`.
3. **Contingency (likely needed)**: if the ADAS ECU notices its 814 vanished
   and faults (AEB disabled = unacceptable), add a **keep-alive echo** — panda
   sends the ADAS ECU's own 814 back to it on bus 1 so it thinks the bus is
   alive. Implement behind a flag; enable if ADAS faults appear.
   (Recommendation: build it from the start — safer first try.)

## 7. Phase 3 — opendbc

1. **Automatic wiring detection, no config**: `_get_params` receives the live
   fingerprint. If 813 appears on **bus 1** → car is long-wired →
   `alphaLongitudinalAvailable = True` + ADAS parser moves to bus 1.
   If on bus 0 (today) → stays MADS-only. One codebase serves both.
2. carstate: third parser on bus 1 (ADAS messages); `cp_adas` accessor points
   there when long-wired; ACC_CMD echo for `bydcan` read from bus 1.
3. Full test suite (interface sim with 3 parsers, lateral limits, docs).

## 8. Phase 4 — On-car validation ladder

1. Stationary, ignition on: dump shows correct routing, zero DTCs, stock AEB
   light off
2. MADS regression (lateral must be untouched)
3. Enable *Experimental Longitudinal* toggle → **closed lot / empty road**:
   ACC engage, gentle accel, gentle brake, standstill resume
4. Normal roads → highway. First ~100 km = test pilot mode, hands/feet ready.

## 9. Fallbacks

- Device-off relay restores both pairs → full stock, always.
- If ADAS faults despite keep-alive: revert wiring to passthrough → back to
  MADS-only. No software rollback needed (fingerprint detection handles it).

## 10. Open questions (to answer when resuming)

1. Harness origin + wiring diagram availability?
2. Second CAN interface for probing (PCAN / spare panda / SavvyCAN), or probe
   via comma CAN1 T-tap?
3. Crimp/relay work DIY or commission a modified harness from a builder?
4. Risk acceptance: ADAS-might-fault window vs building keep-alive echo from
   the start?

---

### Reference: current software state (already merged, working)

- BYD car port in `opendbc_repo/opendbc/car/byd/` (HAN EV 2023 + SEAL
  PERFORMANCE 2025, Seal fingerprint = 91 messages)
- Platform-correct buses: 508 steering (Seal), 792 (Han); ADAS on bus 0 (Seal)
  / bus 2 (Han); PCM_BUTTONS bus-flexible
- MADS: `allow_always` for byd, LKAS button via PCM_BUTTONS.LKAS_ON_BTN
- panda `SAFETY_BYD` (35) with real TX checks, MISRA clean, 120 safety tests
- `alphaLongitudinalAvailable = False` for SEAL (bus 0 ACC_CMD can't be
  intercepted) — HAN keeps openpilot-long option
- panda firmware `panda/board/obj/panda_h7.bin.signed` (TIZI) built and in repo
- Diagnostic tool: `tools/byd_can_dump.py` (route analyzer + `--live` sniff)
