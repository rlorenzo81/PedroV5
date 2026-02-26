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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Front V9", group = "Examples")
@Disabled
public class RedFrontV9 extends OpMode {

    // ================= HARDWARE =================
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private double stateStartTime = 0.0;

    private static final double SHOOTER_TARGET_TPS = 1450;

    // ================= TURRET =================
    private final TurretTracker turret = new TurretTracker();
    private double lastLoopTime = 0;

    // ================= SHOOTER COMP (kept as in originals) =================
    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double MIN_VOLTAGE_FOR_COMP = 10.5;
    private static final double MAX_COMP_MULT = 1.35;
    private static final double MIN_COMP_MULT = 0.85;

    private static final double SHOOTER_KP = 24.0;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.001;
    private static final double SHOOTER_KF = 15.5;

    // ================= MOVEMENT BLEND =================
    private static final double SPEED_FULL_BLEND = 40.0;

    // ================= SCORE WINDOW (AIM TIMEOUT) =================
    private static final double AIM_TIMEOUT_SEC = 1.0; // was.35

    // ================= LATCHED SHOOTING SEQUENCE =================
    private boolean shootingSequenceActive = false;
    private double shootingSequenceStart = 0.0;
    private static final double SHOOT_FEED_SEC = 3.0; // from RedBackV9

    // ================= SHOOTER SPIN-UP DELAY =================
    // ONLY used for FIRST shot window now
    private static final double SHOOTER_SPINUP_SEC = 6.0; // from RedBackV9

    // ================= QUICK SETTLE FOR SUBSEQUENT SHOTS =================
    private static final double SHOOT_SETTLE_SEC = 1.0;

    // ================= FIRST-SPINUP DONE FLAG =================
    private boolean firstSpinupDone = false;

    // ================= SHOOT FEED PULSE STATE (added) =================
    private boolean shootPulseActive = false;
    private double shootPulseStart = 0;

    private static final double SHOOT_PULSE_ON_SEC = 0.25;
    private static final double SHOOT_PULSE_OFF_SEC = 0.25;

    // ================= PAUSE (kept for reuse elsewhere) =================
    private boolean pauseActive = false;
    private double pauseEnd = 0;

    /** Non-blocking pause: returns true when finished. */
    private boolean pauseTime(double seconds) {
        double now = getRuntime();
        if (!pauseActive) {
            pauseActive = true;
            pauseEnd = now + seconds;
        }
        if (now >= pauseEnd) {
            pauseActive = false;
            return true;
        }
        return false;
    }

    // ================= POSES (KEEP EXACTLY AS RedFrontV8) =================
    private final Pose startPose = new Pose(84, 18, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 22, Math.toRadians(65)); //y was 25
    private final Pose toArtifactLine1 = new Pose(98, 42.5, Math.toRadians(0));
    private final Pose driveThroughLine1 = new Pose(131, 42.5, Math.toRadians(0));
    private final Pose driveToShoot2 = new Pose(83, 22, Math.toRadians(62.5)); //y was 25
    private final Pose toArtifactLine2 = new Pose(100, 66.5, Math.toRadians(0));
    private final Pose driveThroughLine2 = new Pose(131, 66.5, Math.toRadians(0));
    private final Pose driveToShoot3 = new Pose(83, 22, Math.toRadians(64)); //y was 25
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

    // ================= STATE HELPERS =================
    private void setPathState(int newState) {
        pathState = newState;
        stateStartTime = getRuntime();
        if (pathTimer != null) pathTimer.resetTimer();

        shootingSequenceActive = false;
        shootingSequenceStart = 0.0;

        // reset pulse so first call to intakeShootFeed restarts properly in each shoot window
        shootPulseActive = false;
    }

    /** FIRST shot window (includes flywheel spinup delay). */
    private boolean runShootWindowFirst() {
        double now = getRuntime();
        double t = now - stateStartTime;

        if (firstSpinupDone) {
            return runShootWindowQuick();
        }

        if (t < SHOOTER_SPINUP_SEC) {
            intakeStop();
            return false;
        }

        firstSpinupDone = true;

        boolean aimOk = turret.isAimedStable(now);
        boolean timedOut = t >= (SHOOTER_SPINUP_SEC + AIM_TIMEOUT_SEC);

        if (!shootingSequenceActive) {
            if (aimOk || timedOut) {
                shootingSequenceActive = true;
                shootingSequenceStart = now;
            } else {
                intakeStop();
                return false;
            }
        }

        // PULSE feed during window
        intakeShootFeed();
        return (now - shootingSequenceStart) >= SHOOT_FEED_SEC;
    }

    /** Subsequent shot windows (NO flywheel spinup; adds settle time). */
    private boolean runShootWindowQuick() {
        double now = getRuntime();
        double t = now - stateStartTime;

        if (t < SHOOT_SETTLE_SEC) {
            intakeStop();
            return false;
        }

        boolean aimOk = turret.isAimedStable(now);
        boolean timedOut = t >= (SHOOT_SETTLE_SEC + AIM_TIMEOUT_SEC);

        if (!shootingSequenceActive) {
            if (aimOk || timedOut) {
                shootingSequenceActive = true;
                shootingSequenceStart = now;
            } else {
                intakeStop();
                return false;
            }
        }

        // PULSE feed during window
        intakeShootFeed();
        return (now - shootingSequenceStart) >= SHOOT_FEED_SEC;
    }

    // ================= AUTO STATE MACHINE (KEEP EXACTLY AS RedFrontV8) =================
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                intakeStop();
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // FIRST shooting position: use spinup window once
                if (!follower.isBusy()) {
                    if (runShootWindowFirst()) {
                        intakeSlow();
                        follower.followPath(driveToLine1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intakeHolds();
                    follower.followPath(goShoot2, 0.5,true);
                    setPathState(4);
                }
                break;

            case 4:
                // Subsequent shoot: NO spinup, settle + aim gate/timeout + feed
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeSlow();
                        follower.followPath(driveToLine2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intakeHolds();
                    follower.followPath(goShoot3,0.5, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Subsequent shoot: NO spinup, settle + aim gate/timeout + feed
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeStop();
                        follower.followPath(leaveOutChain, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    intakeStop();
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    // ================= SHOOTER: KICK -> PIDF (from RedBackV9) =================
    private SpinUpFastKickThenPIDF spinup;
    private boolean shooterSpinupStarted = false;

    // ================= LOOP =================
    @Override
    public void loop() {

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        // Shooter always on (kick -> PIDF) from RedBackV9
        if (!shooterSpinupStarted) {
            spinup.kickMs = 1800;
            spinup.kickPower = 1.0;
            spinup.earlyTakeoverPct = 0.98;
            spinup.start(1450); // same as RedBackV9
            shooterSpinupStarted = true;
        }
        spinup.update();

        // IMPORTANT: choose paths FIRST, then update follower
        autonomousPathUpdate();
        follower.update();

        // Movement blend
        double speed = follower.getVelocity().getMagnitude();
        double moveBlend = clip(speed / SPEED_FULL_BLEND, 0, 1);

        // Pedro getAngularVelocity() is radians/sec -> convert to deg/sec
        double omegaRad = follower.getAngularVelocity();
        double omegaDps = omegaRad * (180.0 / Math.PI);

        // Turret update always
        turret.update(now, dt, moveBlend, omegaDps);

        telemetry.addData("pathState", pathState);
        telemetry.addData("firstSpinupDone", firstSpinupDone);
        telemetry.addData("timeInState", "%.2f", (now - stateStartTime));
        telemetry.addData("aimStable", turret.isAimedStable(now));
        telemetry.addData("tx", "%.2f", turret.getLastTx());
        telemetry.addData("turretPwr", "%.3f", turret.getPower());
        telemetry.addData("Shooter Target / Actual", "%.0f / %.0f",
                SHOOTER_TARGET_TPS, shooter.getVelocity());
        telemetry.update();
    }

    // ================= INIT =================
    @Override
    public void init() {

        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Intake motors
        fi = hardwareMap.dcMotor.get("fi");
        bi = hardwareMap.dcMotor.get("bi");
        fi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeStop();

        // Shooter motor (velocity control)
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        } catch (Exception ignored) {}

        // Spinup helper (from RedBackV9)
        spinup = new SpinUpFastKickThenPIDF(shooter);
        shooterSpinupStarted = false;

        // Servos
        lt = hardwareMap.get(Servo.class, "lt");
        rt = hardwareMap.get(Servo.class, "rt");
        ki = hardwareMap.get(Servo.class, "ki");

        lt.setPosition(0.2);
        rt.setPosition(0.2);
        ki.setPosition(0.15);

        // Turret tracker init
        turret.init(hardwareMap);

        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        // Only spin up once per autonomous
        firstSpinupDone = false;

        setPathState(-1);
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        // Reset the “spinup once” flag at match start
        firstSpinupDone = false;

        // Reset shooter kick-start
        shooterSpinupStarted = false;

        setPathState(0);
    }

    @Override
    public void stop() {
        // no-op
    }

    // ================= INTAKE (pulse added, otherwise unchanged) =================
    private void intakeSlow() {
        shootPulseActive = false;
        fi.setPower(1.0);
        bi.setPower(-0.65);
    }

    /** Pulse continuously: ON for 0.25s, OFF for 0.30s, repeating as long as shooting window calls this. */
    private void intakeShootFeed() {

        double now = getRuntime();

        if (!shootPulseActive) {
            shootPulseActive = true;
            shootPulseStart = now;
        }

        double t = now - shootPulseStart;
        double cycle = SHOOT_PULSE_ON_SEC + SHOOT_PULSE_OFF_SEC;
        double phase = t % cycle;

        if (phase < SHOOT_PULSE_ON_SEC) {
            fi.setPower(1.0);
            bi.setPower(1.0);
        } else {
            fi.setPower(0.0);
            bi.setPower(0.0);
        }
    }

    private void intakeStop() {
        shootPulseActive = false;
        fi.setPower(0.0);
        bi.setPower(0.0);
    }

    private void intakeHolds() {
        shootPulseActive = false;
        fi.setPower(0.1);
        bi.setPower(0.1);
    }

    // ================= SHOOTER (REPLACEMENT HELPERS) =================
    public static class SpinUpFastKickThenPIDF {

        public long kickMs = 1800;
        public double kickPower = 1.0;
        public double earlyTakeoverPct = 0.95;
        public long maxTotalMs = 3000;

        private boolean active = false;
        private long startMs = 0;
        private double targetTicksPerSec = 0;

        private final DcMotorEx shooter;

        public SpinUpFastKickThenPIDF(DcMotorEx shooterMotor) {
            this.shooter = shooterMotor;
        }

        public void start(double targetTicksPerSec) {
            this.targetTicksPerSec = targetTicksPerSec;
            this.startMs = System.currentTimeMillis();
            this.active = true;

            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(kickPower);
        }

        public void update() {
            if (!active) return;

            long elapsed = System.currentTimeMillis() - startMs;
            double vel = shooter.getVelocity();

            boolean timeUp = elapsed >= kickMs;
            boolean closeEnough = vel >= targetTicksPerSec * earlyTakeoverPct;
            boolean safetyUp = elapsed >= maxTotalMs;

            if (timeUp || closeEnough || safetyUp) {
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setVelocity(targetTicksPerSec);
                active = false;
            } else {
                shooter.setPower(kickPower);
            }
        }
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ================= TURRET TRACKER (KEEP OFFSET) =================
    public static class TurretTracker {

        private DcMotorEx turret;
        private Limelight3A limelight;
        private IMU imu;

        private double lastTx = 0;
        private double lastPower = 0;

        private double txFiltered = 0;

        private boolean aimHold = false;
        private double aimStart = 0;

        private double lastErr = 0;

        private static final int TX_SIGN = 1;
        private static final int LEAD_SIGN = 1;

        private static final double KP = 0.055; //was 0.045
        private static final double KD = 0.003;

        private static final double FF = 0.012;
        private static final double LEAD_TIME = 0.03;

        private static final double AIM_TOL = 2.5;
        private static final double AIM_PWR_TOL = 0.12;
        private static final double AIM_STABLE = 0.20;

        // ================= AIM OFFSET (from RedFrontV8) =================
        private static final double TURRET_AIM_OFFSET_DEG = 3.0; // <-- KEEP

        public void init(HardwareMap hw) {
            turret = hw.get(DcMotorEx.class, "turretSpin");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setDirection(DcMotorSimple.Direction.REVERSE);

            limelight = hw.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(9);
            limelight.start();

            imu = hw.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )));
            imu.resetYaw();

            lastTx = 0;
            lastPower = 0;
            txFiltered = 0;
            aimHold = false;
            aimStart = 0;
            lastErr = 0;
        }

        public void update(double now, double dt, double moveBlend, double omegaDps) {

            if (dt < 0.001) dt = 0.001;

            LLResult r = limelight.getLatestResult();
            if (r == null || !r.isValid()) {
                turret.setPower(0);
                lastPower = 0;
                aimHold = false;
                return;
            }

            double txRaw = r.getTx();
            lastTx = txRaw;

            double txPred =
                    (TX_SIGN * txRaw)
                            + (LEAD_SIGN * omegaDps * LEAD_TIME)
                            + TURRET_AIM_OFFSET_DEG;

            double alpha = (moveBlend > 0.2) ? 0.28 : 0.55;
            txFiltered = alpha * txFiltered + (1.0 - alpha) * txPred;

            double error = txFiltered;
            double errAbs = Math.abs(error);

            double dErr = (error - lastErr) / dt;
            lastErr = error;

            double p = KP * error;
            double d = KD * dErr;

            double ffScale = clip((errAbs - 1.5) / (8.0 - 1.5), 0.0, 1.0);
            double ff = FF * omegaDps * (1.0 + moveBlend) * ffScale;

            double power = p + d + ff;

            double brake = clip(errAbs / 6.0, 0.25, 1.0);
            power *= brake;

            power = clip(power, -0.75, 0.75);

            turret.setPower(power);
            lastPower = power;

            if (errAbs < AIM_TOL && Math.abs(power) < AIM_PWR_TOL) {
                if (!aimHold) {
                    aimHold = true;
                    aimStart = now;
                }
            } else {
                aimHold = false;
            }
        }

        public boolean isAimedStable(double now) {
            return aimHold && (now - aimStart) > AIM_STABLE;
        }

        public double getLastTx() { return lastTx; }
        public double getPower() { return lastPower; }
    }
}