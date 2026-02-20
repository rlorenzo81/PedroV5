package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Front V2", group = "Examples")
public class RedFrontV2 extends OpMode {

    // ---------------- HARDWARE (INTAKE + SHOOTER + SERVOS) ----------------
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ---------------- PEDRO ----------------
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // ---------------- TURRET TRACKING (AUTO) ----------------
    private final TurretTracker turretTracker = new TurretTracker();
    private double lastLoopTime = 0.0;

    // Tune these if needed (Pedro velocity magnitude units are typically inches/sec)
    private static final double SPEED_FULL_BLEND = 40.0;      // speed where we consider "fully moving"
    private static final double OMEGA_FULL_BLEND_DPS = 140.0; // match your TURNING_DPS_FULL

    // ---------------- START PAUSE (SPINUP) ----------------
    private static final double PRELOAD_SPINUP_SEC = 1.0;

    // -------- Reusable pause helper --------
    private boolean pauseActive = false;
    private double pauseEndTime = 0.0;

    // ---------------- SHOOTER VOLTAGE COMP ----------------
    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double MIN_VOLTAGE_FOR_COMP = 10.5;
    private static final double MAX_COMP_MULT = 1.35;
    private static final double MIN_COMP_MULT = 0.85;

    private static final double SHOOTER_VEL_KP = 24.0;
    private static final double SHOOTER_VEL_KI = 0.0;
    private static final double SHOOTER_VEL_KD = 0.001;
    private static final double SHOOTER_VEL_KF = 15.5;

    // ---------------- POSES ----------------
    private final Pose startPose = new Pose(84, 18, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 25, Math.toRadians(65));
    private final Pose toArtifactLine1 = new Pose(98, 42.5, Math.toRadians(0));
    private final Pose driveThroughLine1 = new Pose(132, 42.5, Math.toRadians(0));
    private final Pose driveToShoot2 = new Pose(83, 25, Math.toRadians(62.5));
    private final Pose toArtifactLine2 = new Pose(100, 66.5, Math.toRadians(0));
    private final Pose driveThroughLine2 = new Pose(136, 66.5, Math.toRadians(0));
    private final Pose driveToShoot3 = new Pose(83, 25, Math.toRadians(64));
    private final Pose leavePose = new Pose(100, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain driveToLine1, pickUpLine1, goShoot2, driveToLine2, pickUpLine2, goShoot3, leaveOutChain;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        driveToLine1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toArtifactLine1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toArtifactLine1.getHeading())
                .build();

        pickUpLine1 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine1, driveThroughLine1))
                .setLinearHeadingInterpolation(toArtifactLine1.getHeading(), driveThroughLine1.getHeading())
                .build();

        goShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughLine1, driveToShoot2))
                .setLinearHeadingInterpolation(driveThroughLine1.getHeading(), driveToShoot2.getHeading())
                .build();

        driveToLine2 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot2, toArtifactLine2))
                .setLinearHeadingInterpolation(driveToShoot2.getHeading(), toArtifactLine2.getHeading())
                .build();

        pickUpLine2 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine2, driveThroughLine2))
                .setLinearHeadingInterpolation(toArtifactLine2.getHeading(), driveThroughLine2.getHeading())
                .build();

        goShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughLine2, driveToShoot3))
                .setLinearHeadingInterpolation(driveThroughLine2.getHeading(), driveToShoot3.getHeading())
                .build();

        leaveOutChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot3, leavePose))
                .setLinearHeadingInterpolation(driveToShoot3.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // 0) Pause at start to let shooter reach speed (shooter is commanded in loop())
            case 0:
                intakeStop();
                if (pathTimer.getElapsedTimeSeconds() >= PRELOAD_SPINUP_SEC) {
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;

            // 1) After reaching score pose, feed to score preload, then go to line1
            case 1:
                if (!follower.isBusy()) {

                    // Turn on shooting intake
                    intakeShootFeed();

                    // Hold robot still 1.5 seconds to shoot
                    if (pauseTime(1.5)) {
                        follower.followPath(driveToLine1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    // Start collecting for line 1
                    intakeSlow();
                    follower.followPath(pickUpLine1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    // Stop intake before going back to shoot 2 (as you had)
                    intakeStop();
                    follower.followPath(goShoot2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(driveToLine2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickUpLine2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(goShoot3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    follower.getVelocity(); // left as your original line
                }
                break;

            default:
                // no-op
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ---------------- MAIN LOOP ----------------
    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        // Shooter ON at all times (as requested)
        shooterOnCompensated(3200);

        follower.update();
        autonomousPathUpdate();

        // ---------------- TURRET TRACKING UPDATE (RUNS ALWAYS) ----------------
        // Pedro-safe velocity magnitude
        double speed = follower.getVelocity().getMagnitude();
        double moveMag = clip(speed / SPEED_FULL_BLEND, 0.0, 1.0);

        // Per Pedro docs, getAngularVelocity() is radians/sec
        double omegaRad = follower.getAngularVelocity();
        double omegaDps = omegaRad * (180.0 / Math.PI);

        // ✅ FIX #1: signed rx so tracker knows left vs right turning (like a joystick)
        double rx = clip(omegaDps / OMEGA_FULL_BLEND_DPS, -1.0, 1.0);

        turretTracker.update(now, dt, moveMag, rx, true);

        // ---- Telemetry ----
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("speed", "%.2f", speed);
        telemetry.addData("omega dps", "%.2f", omegaDps);
        telemetry.addData("spinup time left", "%.2f", Math.max(0.0, PRELOAD_SPINUP_SEC - pathTimer.getElapsedTimeSeconds()));
        telemetry.update();
    }

    // ---------------- INIT / START ----------------
    @Override
    public void init() {
        // Timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Pedro
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Intake motors
        fi = hardwareMap.dcMotor.get("fi");
        bi = hardwareMap.dcMotor.get("bi");
        intakeStop();

        // Shooter motor (velocity control)
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_VEL_KP, SHOOTER_VEL_KI, SHOOTER_VEL_KD, SHOOTER_VEL_KF);
        } catch (Exception ignored) {}

        // Servos (same names + init positions as your RCRED teleop)
        lt = hardwareMap.get(Servo.class, "lt"); //2
        rt = hardwareMap.get(Servo.class, "rt"); //1
        ki = hardwareMap.get(Servo.class, "ki"); //0

        lt.setPosition(0.35);
        rt.setPosition(0.35);
        ki.setPosition(0.2);

        // Turret tracker init (uses turretSpin + limelight + imu internally)
        turretTracker.init(hardwareMap, "turretSpin", "limelight", 9, "imu", true);

        lastLoopTime = getRuntime();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);          // ✅ start in spinup pause
        lastLoopTime = getRuntime();
    }

    @Override
    public void stop() {
        // no-op
    }

    // =========================
    // INTAKE HELPERS
    // =========================

    // Slow intake / “collect” mode
    private void intakeSlow() {
        fi.setPower(1.0);
        bi.setPower(-0.65);
    }

    // Full / “shooting feed” mode
    private void intakeShootFeed() {
        fi.setPower(1.0);
        bi.setPower(1.0);
    }

    // Stop intake
    private void intakeStop() {
        fi.setPower(0.0);
        bi.setPower(0.0);
    }

    // =========================
    // SHOOTER HELPERS (VOLTAGE COMPENSATED)
    // =========================

    /** Call this with your desired tick/sec setpoint (example: 1350, 1600, etc.) */
    private void shooterOnCompensated(double targetVelTicksPerSec) {
        double mult = voltageCompMultiplier();
        double cmd = targetVelTicksPerSec * mult;
        shooter.setVelocity(cmd);
    }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor v : hardwareMap.voltageSensor) {
            double val = v.getVoltage();
            if (val > 0) min = Math.min(min, val);
        }
        if (!Double.isFinite(min) || min <= 0) return NOMINAL_VOLTAGE;
        return min;
    }

    private double voltageCompMultiplier() {
        double v = getBatteryVoltage();
        v = Math.max(v, MIN_VOLTAGE_FOR_COMP);

        double mult = NOMINAL_VOLTAGE / v;
        return clamp(mult, MIN_COMP_MULT, MAX_COMP_MULT);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ---------------- SMALL HELPERS ----------------
    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ========================= TURTET TRACKER (PASTED IN) =========================
    public static class TurretTracker {

        // ----------------- Hardware -----------------
        private DcMotorEx turretSpin;
        private Limelight3A limelight;
        private IMU imu;

        // =========================================================
        // TUNING (ported from your RCRED)
        // =========================================================

        private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        private static final double TX_DEADBAND_DEG = 1.5;

        private static final double TURRET_KP_STRAIGHT = 0.050;
        private static final double TURRET_KP_TURNING  = 0.025;

        private static final double TURRET_KD_STRAIGHT = 0.003;
        private static final double TURRET_KD_TURNING  = 0.0030;

        private static final double TURNING_DPS_START = 25.0;
        private static final double TURNING_DPS_FULL  = 140.0;

        private static final double MAX_TURRET_POWER_SLOW = 0.40;
        private static final double TURRET_FILTER_ALPHA_SLOW = 0.50;
        private static final double TURRET_SLEW_PER_LOOP_SLOW = 0.12;

        private static final double MAX_TURRET_POWER_FAST = 0.85;
        private static final double TURRET_FILTER_ALPHA_FAST = 0.30;
        private static final double TURRET_SLEW_PER_LOOP_FAST = 0.45;

        private static final double MIN_TURRET_POWER = 0.10;
        private static final double MIN_POWER_ENABLE_ERROR_DEG = 4.5;

        private static final double TX_FILTER_ALPHA_STRAIGHT = 0.74;
        private static final double TX_FILTER_ALPHA_TURNING  = 0.15;

        private static final double TURN_RATE_FF_SLOW = 0.0065;
        private static final double TURN_RATE_FF_FAST = 0.0120;

        private static final double YAW_RATE_DEADBAND_DPS = 3.0;
        private static final double YAW_RATE_CLAMP_DPS = 250.0;
        private static final double YAW_RATE_FILTER_ALPHA = 0.70;

        private static final double FF_ENABLE_TURN_DPS = 25.0;

        private static final double IMU_FF_ENABLE_ERROR_DEG = 3.0;
        private static final double IMU_FF_FULL_ERROR_DEG   = 10.0;

        private static final double LOST_TARGET_GRACE_SEC = 0.10;
        private static final double LOST_TARGET_SEARCH_POWER = 0.00;
        private static final double LOST_SEARCH_DISABLE_TURN_DPS = 90.0;
        private static final double LOST_IMU_FOLLOW_ENABLE_DPS = 35.0;

        private static final boolean ENABLE_SOFT_LIMITS = false;
        private static final int TURRET_MIN_TICKS = -2000;
        private static final int TURRET_MAX_TICKS =  2000;

        private static final double RX_TURN_BLEND_START = 0.10;
        private static final double RX_TURN_BLEND_FULL  = 0.60;

        private static final double MOVE_EPS = 0.12;
        private static final double SETTLE_HOLD_SEC = 0.45;

        private static final double SETTLE_YAW_DPS = 12.0;
        private static final double SETTLE_RX_EPS = 0.10;

        private static final double SETTLE_MAX_TURRET_POWER = 0.35;
        private static final double SETTLE_KP_MULT = 0.65;
        private static final double SETTLE_KD_MULT = 1.15;

        private static final double SETTLE_TX_DEADBAND = 1.7;
        private static final double SETTLE_MINPOWER_ENABLE_ERR = 999.0;

        private static final double STATIONARY_YAW_DPS = 8.0;
        private static final double STATIONARY_RX_EPS  = 0.08;

        private static final double STATIONARY_MAX_TURRET_POWER = 0.22;
        private static final double STATIONARY_KP_MULT = 0.55;
        private static final double STATIONARY_KD_MULT = 1.20;

        private static final double STATIONARY_TX_DEADBAND = 2.0;
        private static final double STATIONARY_MINPOWER_ENABLE_ERR = 999.0;

        private static final double TURN_STOP_HOLD_SEC = 0.18;
        private static final double TURNING_RX_EPS = 0.10;
        private static final double TURNING_YAW_DPS = 18.0;

        private static final double TURNSTOP_MAX_TURRET_POWER = 0.28;
        private static final double TURNSTOP_KP_MULT = 0.60;
        private static final double TURNSTOP_KD_MULT = 1.25;

        // =========================================================
        // State
        // =========================================================
        private double turretFilteredPower = 0.0;
        private double lastTxError = 0.0;
        private double txFiltered = 0.0;

        private double yawRateFiltered = 0.0;

        private double lastSeenTime = -999.0;
        private double lastSeenTx = 0.0;

        private boolean wasMoving = false;
        private double settleUntilTime = -999.0;

        private boolean wasTurning = false;
        private double turnStopUntilTime = -999.0;

        public void init(HardwareMap hardwareMap,
                         String turretMotorName,
                         String limelightName,
                         int limelightPipeline,
                         String imuName,
                         boolean turretReverse) {

            turretSpin = hardwareMap.get(DcMotorEx.class, turretMotorName);
            turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretSpin.setDirection(turretReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            turretSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            limelight = hardwareMap.get(Limelight3A.class, limelightName);
            limelight.pipelineSwitch(limelightPipeline);
            limelight.start();

            imu = hardwareMap.get(IMU.class, imuName);
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB);
            imu.initialize(new IMU.Parameters(orientation));
            imu.resetYaw();

            turretFilteredPower = 0.0;
            lastTxError = 0.0;
            txFiltered = 0.0;
            yawRateFiltered = 0.0;
            lastSeenTime = -999.0;
            lastSeenTx = 0.0;

            wasMoving = false;
            settleUntilTime = -999.0;

            wasTurning = false;
            turnStopUntilTime = -999.0;
        }

        public void update(double nowSec, double dtSec, double moveMag, double rx, boolean enabled) {
            if (dtSec < 0.001) dtSec = 0.001;

            // ----- settle detect (stop event) -----
            boolean movingNow = moveMag > MOVE_EPS;
            if (wasMoving && !movingNow) {
                settleUntilTime = nowSec + SETTLE_HOLD_SEC;
            }
            wasMoving = movingNow;

            // ----- yaw rate filter -----
            double yawRateRaw = getYawRateDegPerSec();
            yawRateFiltered = (YAW_RATE_FILTER_ALPHA * yawRateFiltered) + ((1.0 - YAW_RATE_FILTER_ALPHA) * yawRateRaw);

            double yawRateUse = applyDeadband(yawRateFiltered, YAW_RATE_DEADBAND_DPS);
            yawRateUse = clip(yawRateUse, -YAW_RATE_CLAMP_DPS, YAW_RATE_CLAMP_DPS);

            // ----- turn-stop detect -----
            boolean turningNow = (Math.abs(rx) > TURNING_RX_EPS) || (Math.abs(yawRateFiltered) > TURNING_YAW_DPS);
            if (wasTurning && !turningNow) {
                turnStopUntilTime = nowSec + TURN_STOP_HOLD_SEC;
            }
            wasTurning = turningNow;

            // ----- turn blend -----
            double rxAbs = Math.abs(rx);
            double turnBlendRx = (rxAbs - RX_TURN_BLEND_START) / (RX_TURN_BLEND_FULL - RX_TURN_BLEND_START);
            turnBlendRx = clip(turnBlendRx, 0.0, 1.0);

            double turnBlendYaw = (Math.abs(yawRateFiltered) - TURNING_DPS_START) / (TURNING_DPS_FULL - TURNING_DPS_START);
            turnBlendYaw = clip(turnBlendYaw, 0.0, 1.0);

            double turnBlend = Math.max(turnBlendRx, turnBlendYaw);

            // Adaptive parameters
            double maxTurretPower = lerp(MAX_TURRET_POWER_SLOW, MAX_TURRET_POWER_FAST, turnBlend);
            double filterAlpha    = lerp(TURRET_FILTER_ALPHA_SLOW, TURRET_FILTER_ALPHA_FAST, turnBlend);
            double slewPerLoop    = lerp(TURRET_SLEW_PER_LOOP_SLOW, TURRET_SLEW_PER_LOOP_FAST, turnBlend);

            double txAlpha = lerp(TX_FILTER_ALPHA_STRAIGHT, TX_FILTER_ALPHA_TURNING, turnBlend);
            double kpUse   = lerp(TURRET_KP_STRAIGHT, TURRET_KP_TURNING, turnBlend);
            double kdUse   = lerp(TURRET_KD_STRAIGHT, TURRET_KD_TURNING, turnBlend);

            double ffGain = lerp(TURN_RATE_FF_SLOW, TURN_RATE_FF_FAST, turnBlend);

            // ----- settle / stationary / turnstop overrides -----
            boolean settleWindow = (nowSec <= settleUntilTime);

            boolean settleEligible =
                    settleWindow &&
                            (Math.abs(yawRateFiltered) <= SETTLE_YAW_DPS) &&
                            (Math.abs(rx) <= SETTLE_RX_EPS);

            boolean stationaryEligible =
                    (Math.abs(yawRateFiltered) <= STATIONARY_YAW_DPS) &&
                            (Math.abs(rx) <= STATIONARY_RX_EPS);

            boolean turnStopEligible = (nowSec <= turnStopUntilTime);

            double txDeadbandUse = TX_DEADBAND_DEG;
            double minPowerEnableErrUse = MIN_POWER_ENABLE_ERROR_DEG;
            boolean killFFThisLoop = false;

            if (turnStopEligible) {
                maxTurretPower = Math.min(maxTurretPower, TURNSTOP_MAX_TURRET_POWER);
                kpUse *= TURNSTOP_KP_MULT;
                kdUse *= TURNSTOP_KD_MULT;
                killFFThisLoop = true;
            } else if (settleEligible) {
                maxTurretPower = Math.min(maxTurretPower, SETTLE_MAX_TURRET_POWER);
                kpUse *= SETTLE_KP_MULT;
                kdUse *= SETTLE_KD_MULT;
                txDeadbandUse = SETTLE_TX_DEADBAND;
                minPowerEnableErrUse = SETTLE_MINPOWER_ENABLE_ERR;
            } else if (stationaryEligible) {
                maxTurretPower = Math.min(maxTurretPower, STATIONARY_MAX_TURRET_POWER);
                kpUse *= STATIONARY_KP_MULT;
                kdUse *= STATIONARY_KD_MULT;
                txDeadbandUse = STATIONARY_TX_DEADBAND;
                minPowerEnableErrUse = STATIONARY_MINPOWER_ENABLE_ERR;
            }

            double desiredTurretPower = 0.0;

            if (enabled) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    double tx = result.getTx();

                    lastSeenTime = nowSec;
                    lastSeenTx = tx;

                    txFiltered = (txAlpha * txFiltered) + ((1.0 - txAlpha) * tx);

                    double txError = applyDeadband(txFiltered, txDeadbandUse);

                    double dError = (txError - lastTxError) / dtSec;
                    lastTxError = txError;

                    double visionPower = (kpUse * txError) + (kdUse * dError);

                    double absErr = Math.abs(txError);

                    double ffFromErr = 0.0;
                    if (absErr >= IMU_FF_ENABLE_ERROR_DEG) {
                        ffFromErr = (absErr - IMU_FF_ENABLE_ERROR_DEG) /
                                (IMU_FF_FULL_ERROR_DEG - IMU_FF_ENABLE_ERROR_DEG);
                        ffFromErr = clip(ffFromErr, 0.0, 1.0);
                    }

                    double ffFromTurn = 0.55 * turnBlend;
                    double ffScale = Math.max(ffFromErr, ffFromTurn);

                    double imuFF = 0.0;
                    if (!killFFThisLoop && Math.abs(yawRateUse) >= FF_ENABLE_TURN_DPS) {
                        imuFF = (+ffGain * yawRateUse) * ffScale;
                    }

                    double autoPower = visionPower + imuFF;

                    if (Math.abs(txError) >= minPowerEnableErrUse) {
                        autoPower = addMinPower(autoPower, MIN_TURRET_POWER);
                    }

                    desiredTurretPower = clip(autoPower, -maxTurretPower, maxTurretPower);

                } else {
                    double timeSinceSeen = nowSec - lastSeenTime;

                    if (timeSinceSeen <= LOST_TARGET_GRACE_SEC) {
                        if (Math.abs(yawRateUse) >= LOST_IMU_FOLLOW_ENABLE_DPS) {
                            desiredTurretPower = clip((+ffGain * yawRateUse), -maxTurretPower, maxTurretPower);
                        } else if (Math.abs(yawRateUse) <= LOST_SEARCH_DISABLE_TURN_DPS) {
                            double sign = (lastSeenTx >= 0) ? 1.0 : -1.0;
                            desiredTurretPower = sign * LOST_TARGET_SEARCH_POWER;
                        } else {
                            desiredTurretPower = 0.0;
                        }
                    } else {
                        desiredTurretPower = 0.0;
                    }

                    lastTxError = 0.0;
                }
            } else {
                lastTxError = 0.0;
                desiredTurretPower = 0.0;
            }

            // ----- smooth -----
            double filteredTarget = (filterAlpha * turretFilteredPower) + ((1.0 - filterAlpha) * desiredTurretPower);
            turretFilteredPower = slewTo(turretFilteredPower, filteredTarget, slewPerLoop);

            // ----- soft limits -----
            if (ENABLE_SOFT_LIMITS) {
                int ticks = turretSpin.getCurrentPosition();
                if (ticks >= TURRET_MAX_TICKS && turretFilteredPower > 0) turretFilteredPower = 0;
                if (ticks <= TURRET_MIN_TICKS && turretFilteredPower < 0) turretFilteredPower = 0;
            }

            turretSpin.setPower(turretFilteredPower);
        }

        private double getYawRateDegPerSec() {
            AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            return av.zRotationRate;
        }

        private static double clip(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        private static double applyDeadband(double value, double deadband) {
            return (Math.abs(value) <= deadband) ? 0.0 : value;
        }

        private static double addMinPower(double power, double minPower) {
            if (power == 0.0) return 0.0;
            double sign = (power > 0) ? 1.0 : -1.0;
            return sign * Math.max(Math.abs(power), minPower);
        }

        private static double slewTo(double current, double target, double maxDelta) {
            double delta = target - current;
            if (delta > maxDelta) return current + maxDelta;
            if (delta < -maxDelta) return current - maxDelta;
            return target;
        }

        private static double lerp(double a, double b, double t) {
            return a + (b - a) * t;
        }
    }
    // ======================= END TURRET TRACKER =======================
    // =========================
// NON-BLOCKING PAUSE
// =========================
    private boolean pauseTime(double seconds) {
        double now = getRuntime();

        if (!pauseActive) {
            pauseActive = true;
            pauseEndTime = now + seconds;
        }

        if (now >= pauseEndTime) {
            pauseActive = false;
            return true; // pause finished
        }

        return false; // still pausing
    }
}