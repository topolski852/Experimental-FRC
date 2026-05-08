# Team 1507 Warlocks — FRC Robot Code Base

A beginner-friendly FRC robot code base built for high school students.
Designed to be readable, reusable each season, and easy to extend for any game.

---

## Quick Start

| Task | Where to look |
|------|---------------|
| Change a CAN device ID | `Constants.java` → `RobotMap` |
| Tune a motor's PID | `Constants.java` → the subsystem's `MotorConfig` |
| Add a button binding | `Robot.java` → `configureBindings()` |
| Add a new auto routine | Copy `DriveForwardAuto.java`, follow the comments |
| Add a new subsystem | See [Adding a Subsystem](#adding-a-subsystem) below |

---

## Project Structure

```
src/main/java/org/team1507/
│
├── lib/core/                    ← Reusable library (don't edit unless you know why)
│   ├── framework/
│   │   ├── LoggedRobot.java     ← Base robot class with telemetry enabled
│   │   └── Subsystem1507.java  ← Base class for all subsystems
│   ├── impl/ctre/
│   │   ├── Motor1507.java       ← All motor hardware lives here (TalonFX / TalonFXS)
│   │   ├── MotorConfig.java     ← Declarative motor configuration (PID, limits, etc.)
│   │   ├── CtreMotorConfigurator.java  ← Applies MotorConfig to real hardware
│   │   └── CtreMotorSignals.java       ← CAN signal wrappers (position, velocity, current)
│   ├── logging/
│   │   ├── Telemetry.java       ← Central NetworkTables publisher
│   │   ├── InputField.java      ← Rate-limited sensor value logging
│   │   └── TelemetryRate.java   ← FAST / NORMAL / SLOW update rates
│   ├── swerve/
│   │   └── SwerveModule1507.java  ← One swerve module (drive + steer + encoder)
│   ├── util/
│   │   ├── CommandBuilder.java  ← Fluent command factory (used inside subsystems)
│   │   └── MotorConfig.java     ← Motor configuration record + builder
│   └── vision/
│       └── QuestNavSubsystem.java  ← Meta Quest vision integration
│
└── robot/                       ← Your robot-specific code (edit this every season)
    ├── Robot.java               ← Entry point, subsystems, button bindings
    ├── RobotBehaviors.java      ← Multi-subsystem actions (score, intake, stow)
    ├── Constants.java           ← All tuning numbers and hardware IDs
    ├── subsystems/
    │   ├── Swerve.java          ← Drivetrain (swerve drive with odometry)
    │   ├── ArmSystem.java       ← Example arm subsystem
    │   └── BasicMotor.java      ← Example single-motor subsystem
    └── auto/
        ├── AutoBuilder.java     ← Holds references to all subsystems for auto
        ├── AutoSequence.java    ← Fluent auto routine builder
        └── routines/
            └── DriveForwardAuto.java  ← Example auto routine
```

---

## Adding a Subsystem

1. **Create the subsystem file** in `robot/subsystems/`. Copy `BasicMotor.java` as a starting point.

2. **Configure your motors** in `Constants.java`. Add a new nested class:
   ```java
   public static final class kMySubsystem {
       public static final MotorConfig MOTOR_CONFIG =
           MotorConfig.builder(ControlMode.VELOCITY)
               .inverted(false)
               .withPID(1.0, 0.0, 0.0)
               .withFeedforward(0.1, 2.5, 0.0)
               .build();
   }
   ```

3. **Add the CAN ID** to `Constants.RobotMap`:
   ```java
   public static final int MY_MOTOR = 20;
   ```

4. **Create the motor** in your subsystem's constructor:
   ```java
   private final Motor1507 motor;

   public MySubsystem() {
       super("MySubsystem");
       motor = new Motor1507("MyMotor", Motor1507.Type.FX, RobotMap.MY_MOTOR,
           Constants.kMySubsystem.MOTOR_CONFIG);
   }
   ```

5. **Add commands** to the subsystem class (see `BasicMotor.java` for examples).

6. **Register it** in `Robot.java`:
   ```java
   public final MySubsystem mySubsystem = new MySubsystem();
   ```
   And in the `AutoBuilder.init()` call if you want it available in auto.

---

## Configuring a Motor (`MotorConfig`)

`MotorConfig` is how you tell a motor what it should do and how. You configure it
in `Constants.java` and the motor applies those settings automatically on startup.

```java
MotorConfig.builder(ControlMode.VELOCITY)   // VELOCITY, POSITION, DUTY_CYCLE, MOTION_MAGIC
    .inverted(false)                        // true = flip the positive direction
    .withPID(kP, kI, kD)                   // proportional, integral, derivative gains
    .withFeedforward(kS, kV, kA)           // static friction, velocity, acceleration gains
    .withVoltageLimits(12.0, -12.0)        // max forward/reverse voltage (volts)
    .withStatorCurrentLimit(Amps.of(60))   // prevents brownouts (amps)
    .withBrake()                           // hold position when stopped (vs. coast)
    .withGravity(kG, GravityType.COSINE)   // counteract gravity on arms
    .build();
```

**Which control mode to use:**
| Mode | Use it when… | Example |
|------|-------------|---------|
| `DUTY_CYCLE` | Simple open-loop spin (no feedback) | Intake rollers |
| `VELOCITY` | Spin at a specific RPM | Shooter flywheel |
| `POSITION` | Move to a specific angle/position | Arm, turret |
| `MOTION_MAGIC` | Smooth S-curve moves to a position | Elevator, precise arm |

---

## CAN IDs (`Constants.RobotMap`)

**All** CAN device IDs are in one place: `Constants.RobotMap`. If you replace a motor
or re-wire the CAN bus, you only update this one class.

```java
public static final class RobotMap {
    public static final int FL_DRIVE = 7;   // Front Left drive motor
    public static final int FL_STEER = 8;   // Front Left steer motor
    public static final int FL_ENCODER = 9; // Front Left CANcoder
    // ... etc.
    public static final int PIGEON2 = 30;   // Gyro
}
```

**Encoder offsets** (also in `RobotMap`) are the rotation values that align each
swerve wheel to point straight forward. Re-calibrate these whenever a swerve module
is disassembled or the wheel position changes.

---

## Adding a Button Binding

Open `Robot.java`, find `configureBindings()`, and add a line:

```java
// Hold A to run the intake forward
driver.a().whileTrue(mySubsystem.runForwardCommand());

// Press X once to toggle the arm
driver.x().onTrue(arm.deployCommand());

// Tap Y once to run a coordinated multi-subsystem action
driver.y().onTrue(RobotBehaviors.scoreHigh());
```

**Which trigger to use:**
| Trigger | Behavior |
|---------|---------|
| `.onTrue(cmd)` | Start command when button is first pressed |
| `.whileTrue(cmd)` | Run command while button is held, stop when released |
| `.onFalse(cmd)` | Start command when button is released |

---

## Writing an Auto Routine

Copy `robot/auto/routines/DriveForwardAuto.java` and rename the class. Then
use `AutoSequence` to chain actions together:

```java
public static Command build() {
    return new AutoSequence()
        .resetPose(new Pose2d(0, 0, Rotation2d.kZero))  // always start with this
        .driveForwardMeters(2.0)                          // drive forward 2 meters
        .armHigh()                                        // deploy arm
        .motorForward()                                   // run motor
        .stop()
        .build();
}
```

Register your new routine in `Robot.java`:
```java
autoChooser.addOption("My Auto", MyAuto.build());
```

---

## QuestNav Vision Setup

**Step 1 — Install the vendor library**
1. In VS Code, open the WPILib Command Palette: `Ctrl+Shift+P`
2. Select **WPILib: Manage Vendor Libraries → Install new libraries (online)**
3. Paste this URL: `https://maven.questnav.gg/releases/gg/questnav/questnavlib-java/latest/questnavlib.json`

**Step 2 — Run the Quest app**
- Install and launch the QuestNav app on the Meta Quest headset
- It will automatically start publishing pose data over NetworkTables

**Step 3 — Wire it in `Robot.java`**
```java
// Add after creating the swerve subsystem.
// The :: syntax passes swerve's methods as callbacks — no complex setup needed.
public final QuestNavSubsystem questNav = new QuestNavSubsystem(
    swerve::addVisionMeasurement,  // called each loop with fresh pose estimates
    swerve::resetPose              // called to snap odometry to Quest's pose
);
```

**Step 4 — Bind the reset button in `configureBindings()`**
```java
// Press Start on the controller before auto or when placing the robot on the field.
driver.start().onTrue(questNav.resetPoseFromQuestCommand());
```

After reset, `QuestNavSubsystem` feeds vision updates automatically every loop.
Check `questNav.isConnected()` and `questNav.isTracking()` on the dashboard.

> **Note on mounting offset:** If the Quest is not mounted at the robot's center,
> you'll need to measure the offset and pass it to the constructor:
> ```java
> questNav = new QuestNavSubsystem(
>     swerve::addVisionMeasurement, swerve::resetPose,
>     new Transform3d(0.2, 0.0, 0.5, new Rotation3d())  // example: 20cm forward, 50cm up
> );
> ```

---

## Telemetry & Debugging

All robot data is published to NetworkTables and viewable in:
- **Shuffleboard** (Driver Station)
- **AdvantageScope** (recommended — shows field visualizations, graphs, and more)

Key NetworkTables paths:
| Path | What it shows |
|------|--------------|
| `Swerve/` | Robot pose, module states, stall flags |
| `QuestNav/` | Quest tracking status, pose |
| `{MotorName}/Input/` | Position, velocity, current, voltage, temp |
| `Command/{name}/` | Active commands and their timing |

---

## Frequently Asked Questions

**Q: The robot drives in the wrong direction on the field.**
> Check that the gyro (Pigeon2) is zeroed facing the correct alliance wall. You can reset it in the Driver Station or by pressing the "reset" binding.

**Q: A swerve module is spinning but not driving, or driving at the wrong angle.**
> The encoder offset for that module is probably wrong. Open `Constants.RobotMap`, find the `_ENCODER_OFFSET` for that module, and re-run the CTRE Phoenix Tuner calibration to get the correct value.

**Q: The arm or mechanism isn't moving to the right position.**
> The `kP` gain in `Constants` for that motor config may need tuning. Start low (0.1) and increase slowly until the motor moves to the target without overshooting.

**Q: I'm getting brownouts during auto.**
> Reduce the stator current limit (`withStatorCurrentLimit`) on the drive motors, or add a supply current limit. Also consider reducing max speed in your auto routines.

**Q: I want to add a second controller for an operator.**
> Add `private final CommandXboxController operator;` in `Robot.java` and initialize it with port `Constants.RobotMap.OPERATOR_CONTROLLER`. Then bind commands in `configureBindings()` using `operator.b()...` etc.
