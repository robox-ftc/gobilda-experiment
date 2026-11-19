# NewAuto: Translating in Circles — Debug & Fix

**Code:** `src/main/java/org/firstinspires/ftc/teamcode/NewAuto.java`  
**Typical run:** SHOOT → `TRANSLATE(-10, 7)` (field pose relative to origin after `start()`)

Chinese version: `FIX-TRANSLATE-SPIN.zh.md`

---

## 1. Can we test with a one-line change?

**Yes.**

In `case TRANSLATE`, at the drivetrain assignment (around lines 273–276), **only** set:

```java
controls.rotationPower = 0;
```

**Comment out** any `rotationPower` assignment that uses `headingErr`, `H_DIR`, or `HEADING_P`.

- Leave `readX/Y`, `Y_DIR`, `errorX = task.target - x`, and field→robot transform unchanged for this test  
- Build → deploy → run SHOOT → note whether it still spins  
- Use the decision tree below after the test  

Passing hand-push checks in init only validates **open-loop odometry**. Auto translate adds **rotation + cos/sin**, so do this A/B test first.

---

## 2. Symptoms

- Hand-push checks (axes & scale) OK  
- Arcing or spinning during SHOOT `TRANSLATE`  
- Sometimes `ARRIVED` but path is not a straight line to the goal  

---

## 3. What the program does (why spin happens)

Each `loop()` → `TRANSLATE`:

1. **Field error** — `errorX = task.target - x`, `errorY = task.target2 - y`  
2. **Field → robot** — `robotRight` / `robotForward` via `cos/sin` (uses `heading` or `headingDrift`)  
3. **Power** — `X_DIR` / `Y_DIR` × `drivePower` (includes `MIN_POWER_*`)  
4. **Optional heading hold** — `rotationPower` from heading  

Mecanum **diagonal + min power** yaws the chassis. Wrong **heading-hold sign** or a bad transform with step 4 often produces **circles**.

---

## 4. Decision tree (in order)

### Step A — One-line test

```java
controls.rotationPower = 0;  // TRANSLATE only
```

| Observation | Conclusion |
|-------------|------------|
| **No obvious spinning**, approaches goal | Main cause: **heading control** during translate |
| **Still spins** | Suspect **transform**, **X axis**, or **dual-axis min power** |

---

### Step B — Still spins with `rotationPower = 0`

Temporarily **skip rotating the error** (test only, inside `TRANSLATE` `else`):

```java
double robotRight = errorX;
double robotForward = errorY;
// comment out the cos/sin branch
```

| Observation | Next step |
|-------------|-----------|
| Straighter path | Fix `headingDrift` / `cos/sin` or increase `FIELD_DRIVE_HEADING_DEG` |
| Still chaotic | Check `X_DIR`, `PINPOINT_UPSIDE_DOWN`, `setEncoderDirections` |

---

### Step C — No spin when `rotationPower = 0`

Re-enable **heading hold** (relative to task start, negative feedback):

```java
// At task start: translateHeadingHold = heading (with translateCaptureTask logic)
double headingErr = translateHeadingHold - heading;
while (headingErr > 180) headingErr -= 360;
while (headingErr < -180) headingErr += 360;

controls.rotationPower = H_DIR * Math.min(
        Math.max(headingErr * HEADING_P, -MAX_ROTATION_POWER),
        MAX_ROTATION_POWER);
```

Do **not** use `rotationPower ∝ +heading` toward absolute 0° — that tends to **positive feedback**.

If it spins hard one way after restore: flip **`H_DIR`** only (`-1.0` or `1.0`); do not change the `headingErr` formula.

---

## 5. Tunables

| Constant | Notes |
|----------|--------|
| `X_DIR` / `Y_DIR` | Match motor wiring; forward often needs `Y_DIR = -1` |
| `PINPOINT_UPSIDE_DOWN` | `true` if CCW rotation reads negative on raw heading |
| `MIN_POWER_X` / `MIN_POWER_Y` | Too low = stall; too high = skew and yaw |
| `TOLERANCE` | ~1″; too tight → `TIMEOUT` |
| `FIELD_DRIVE_HEADING_DEG` | Use field `errorX/Y` directly when \(|\Delta\text{heading}|\) is small |
| `H_DIR` | Flip heading-hold motor sign only |

---

## 6. Driver Hub telemetry

| Key | What to expect |
|-----|----------------|
| `Error X` / `Error Y` | Trend toward 0 |
| `Current Heading` | Large drift with `rotationPower=0` → transform/strafe issue |
| `Heading hold` / `Heading err` | After hold enabled, err should shrink |
| `Last Translate` | `ARRIVED` vs `TIMEOUT` |
| `Translation-Strafe` / `Forward` | Power signs vs errors |

---

## 7. Why PARK can work but SHOOT spins

- PARK: mostly `(0, ±10)` — mostly Y only  
- SHOOT: e.g. `(-10, 7)` — **both axes** + min power + (optional) rotation  

---

## 8. Team workflow

1. **One line:** `rotationPower = 0` → run SHOOT → record behavior  
2. Follow **§4**; change **one thing at a time**  
3. Commit / PR: note which step was validated  

---

## 9. Restore production heading hold (after Step C)

In `TRANSLATE` `else`, **replace** the test line:

```java
controls.rotationPower = 0;
```

with the Step C block; remove `// TEST: no heading hold ...`.

---

*Doc version: 2026-11-19 — NewAuto field coordinates + Pinpoint upside-down setup.*