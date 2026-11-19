# NewAuto 平移打转排查与修复

**对应代码：** `src/main/java/org/firstinspires/ftc/teamcode/NewAuto.java`  
**典型任务：** SHOOT → `TRANSLATE(-10, 7)`（场坐标，相对 `start()` 后原点）

英文版见：`FIX-TRANSLATE-SPIN.en.md`

---

## 1. 能不能只改一行先测？

**可以。**

在 `case TRANSLATE` 里，给底盘赋值的位置（约 273–276 行）**只保留**：

```java
controls.rotationPower = 0;
```

并**注释掉**任何带 `headingErr`、`H_DIR`、`HEADING_P` 的 `rotationPower` 赋值。

- 其它代码（`readX/Y`、`Y_DIR`、`errorX = task.target - x`、场向→车身）先不动  
- Build 装车 → 跑 SHOOT → 看还打不打转  
- 测完再按下面决策树改  

手推 init 时 X/Y/heading 正确，只说明**开环里程计**对；自动平移还会加 **转头 + cos/sin**，所以必须先做这一刀 A/B 测试。

---

## 2. 症状

- 手推 1–3（方向/比例）正确  
- SHOOT 平移时画弧、打转、路径很乱  
- 有时 `ARRIVED` 但不像直线到目标  

---

## 3. 程序在做什么（为何会转）

每圈 `loop()` → `TRANSLATE` 时大致顺序：

1. **场坐标误差** `errorX = task.target - x`，`errorY = task.target2 - y`  
2. **场向 → 车身** `robotRight` / `robotForward`（`cos/sin`，与 `heading` 或 `headingDrift` 有关）  
3. **功率** `X_DIR` / `Y_DIR` × `drivePower`（含 `MIN_POWER_*`）  
4. **（可选）车头保持** `rotationPower` 与 heading 有关  

麦轮**斜移 + 最小功率**会让车头自然偏转；若第 4 步符号错或与第 2 步叠加，容易**绕圈**。

---

## 4. 决策树（按顺序做）

### Step A — 只改一行

```java
controls.rotationPower = 0;  // 仅 TRANSLATE
```

| 现象 | 结论 |
|------|------|
| **明显不打转**，能靠近目标 | 主因是 **heading 控制**（环节 1/2） |
| **照样打转** | 主因更可能是 **场向变换 / X 轴 / 双轴最小功率** |

---

### Step B — `rotationPower = 0` 后仍打转

临时把场向变换改成**不旋转误差**（仅测试，在 `TRANSLATE` 的 `else` 里）：

```java
double robotRight = errorX;
double robotForward = errorY;
// 注释掉 cos/sin 分支
```

| 现象 | 下一步 |
|------|--------|
| 变直 | 调整 `headingDrift` / `cos/sin` 或增大 `FIELD_DRIVE_HEADING_DEG` |
| 仍乱 | 查 `X_DIR`、`PINPOINT_UPSIDE_DOWN`、`setEncoderDirections` |

---

### Step C — `rotationPower = 0` 后不打转

恢复 **车头保持**（相对任务开始角，负反馈）：

```java
// 任务开始时: translateHeadingHold = heading（已有 translateCaptureTask 逻辑时）
double headingErr = translateHeadingHold - heading;
while (headingErr > 180) headingErr -= 360;
while (headingErr < -180) headingErr += 360;

controls.rotationPower = H_DIR * Math.min(
        Math.max(headingErr * HEADING_P, -MAX_ROTATION_POWER),
        MAX_ROTATION_POWER);
```

**不要**用 `rotationPower ∝ +heading`（易正反馈）。

若恢复后**单边猛转**：只改 `H_DIR` 为 `-1.0`（或 `1.0`），不要改 `headingErr` 公式。

---

## 5. 常用常量（可调）

| 常量 | 说明 |
|------|------|
| `X_DIR` / `Y_DIR` | 电机与误差符号对齐；前进常需 `Y_DIR = -1` |
| `PINPOINT_UPSIDE_DOWN` | 从上往下看逆时针转时 raw heading 为负 → `true` |
| `MIN_POWER_X` / `MIN_POWER_Y` | 太小不到位，太大易斜移扭头 |
| `TOLERANCE` | 建议约 1″，过小易 `TIMEOUT` |
| `FIELD_DRIVE_HEADING_DEG` | 相对 hold 小于此角时用 `errorX/Y` 直驱 |
| `H_DIR` | 仅翻转车头修正电机向 |

---

## 6. Telemetry 看什么

| 键 | 含义 |
|----|------|
| `Error X` / `Error Y` | 应朝 0 收敛 |
| `Current Heading` | `rotationPower=0` 时若仍大增 → 变换/横移问题 |
| `Heading hold` / `Heading err` | 恢复转头后 err 应变小 |
| `Last Translate` | `ARRIVED` vs `TIMEOUT` |
| `Translation-Strafe` / `Forward` | 与误差符号是否合理 |

---

## 7. 与 PARK 的差异

- PARK 多为 `(0, ±10)`，几乎只动 Y  
- SHOOT 如 `(-10, 7)`：**双轴同时** + 最小功率 +（可选）转头  

---

## 8. 推荐工作流（给队友）

1. **只改一行** `rotationPower = 0` → 跑 SHOOT → 记录现象  
2. 根据 **§4 决策树** 改下一步（**勿一次改很多行**）  
3. 确认后 commit，PR 里写清测了哪一步  

---

## 9. 恢复「正式版」转头（Step C 通过后）

在 `TRANSLATE` 的 `else` 末尾，用 Step C 代码块**替换**：

```java
controls.rotationPower = 0;
```

删除测试注释 `// TEST: no heading hold ...`。

---

*文档版本：2026-11-19 — 对应 NewAuto 场坐标 + Pinpoint upside-down 配置。*