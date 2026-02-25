package org.firstinspires.ftc.teamcode.OldCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Back V8 Line1", group = "Examples")
@Disabled
public class RedBackV8WithLine1 extends OpMode {

    // ================= HARDWARE =================
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private double stateStartTime = 0.0;
    double target = 1000.0; // change if needed

    // ================= TURRET =================
    private final TurretTracker turret = new TurretTracker();
    private double lastLoopTime = 0;

    // ================= SHOOTER COMP =================
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
    private static final double AIM_TIMEOUT_SEC = 0.35;

    // ================= LATCHED SHOOTING SEQUENCE =================
    private boolean shootingSequenceActive = false;
    private double shootingSequenceStart = 0.0;
    private static final double SHOOT_FEED_SEC = 1.5;

    // ================= SHOOTER SPIN-UP DELAY =================
    // ONLY used for FIRST shot window now
    private static final double SHOOTER_SPINUP_SEC = 2.5; //was 2.0

    // ================= QUICK SETTLE FOR SUBSEQUENT SHOTS =================
    private static final double SHOOT_SETTLE_SEC = 0.20;

    // ================= FIRST-SPINUP DONE FLAG =================
    private boolean firstSpinupDone = false;

    // ================= SHOOT FEED PULSE STATE =================
    private boolean shootPulseActive = false;
    private double shootPulseStart = 0;

    private static final double SHOOT_PULSE_ON_SEC = 0.20;
    private static final double SHOOT_PULSE_OFF_SEC = 0.10;

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

    // ================= POSES =================
    private final Pose startPose = new Pose(120, 126, Math.toRadians(45));
    private final Pose scorePose = new Pose(93, 77, Math.toRadians(53));//89, 87

    private final Pose toArtifactLine2 = new Pose(100, 59, Math.toRadians(0));
    private final Pose driveThroughLine2 = new Pose(142, 59, Math.toRadians(0));
    private final Pose driveToShoot2 = new Pose(93, 77, Math.toRadians(0)); // was 53

    private final Pose driveTowardsGate1= new Pose(100, 68, Math.toRadians(53)); // was 53
    private final Pose driveToGate1= new Pose(141, 60, Math.toRadians(53)); // was 120,71 [move y back so it doesnt hit the line (10:09)]

    // intake post
    private final Pose intakeFromGate1 = new Pose(143, 56, Math.toRadians(55));
    private final Pose intakeFromGate2 = new Pose(141, 53, Math.toRadians(53));

    private final Pose driveToShoot3= new Pose(93, 77, Math.toRadians(53)); // was 53

    private final Pose driveTowardsGate2= new Pose(100, 71, Math.toRadians(53)); // was 53
    private final Pose driveToGate2= new Pose(139, 55, Math.toRadians(53)); // was 127,71 [go a little more forward (10:05am)]
    private final Pose driveToShoot4= new Pose(93, 77, Math.toRadians(53)); // was 53

    // ====== NEW (requested) ======
    private final Pose toArtifactLine1 = new Pose(100, 81, Math.toRadians(0));
    private final Pose driveThroughLine1 = new Pose(130, 81, Math.toRadians(0));
    private final Pose driveToShoot5 = new Pose(93, 77, Math.toRadians(53));
    // ============================

    private final Pose leavePose = new Pose(115, 66, Math.toRadians(0));

    private Path scorePreload;
    private PathChain driveToLine1, pickUpLine1, goShoot2, driveToLine2, pickUpLine2,
            onWaytoGate1, toGate1, goShoot3, onWaytoGate2, toGate2, goShoot4, leaveOutChain;

    // combined chains
    private PathChain goGate1Combined;
    private PathChain goGate2Combined;

    // gate -> intake post, then intake post -> shoot
    private PathChain gate1ToIntakePost, intakePostToShoot3;
    private PathChain gate2ToIntakePost, intakePostToShoot4;

    // ====== NEW paths (requested) ======
    private PathChain shoot4ToLine1, line1DriveThrough, line1ToShoot5;
    // ==================================

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        driveToLine2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toArtifactLine2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toArtifactLine2.getHeading())
                .build();

        pickUpLine2 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine2, driveThroughLine2))
                .setLinearHeadingInterpolation(toArtifactLine2.getHeading(), driveThroughLine2.getHeading())
                .build();

        goShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughLine2, driveToShoot2))
                .setLinearHeadingInterpolation(driveThroughLine2.getHeading(), driveToShoot2.getHeading())
                .build();

        onWaytoGate1 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot2, driveTowardsGate1))
                .setLinearHeadingInterpolation(driveToShoot2.getHeading(), driveTowardsGate1.getHeading())
                .build();

        toGate1 = follower.pathBuilder()
                .addPath(new BezierLine(driveTowardsGate1, driveToGate1))
                .setLinearHeadingInterpolation(driveTowardsGate1.getHeading(), driveToGate1.getHeading())
                .build();

        // gate1 smooth curve through driveTowardsGate1
        goGate1Combined = follower.pathBuilder()
                .addPath(new BezierCurve(driveToShoot2, driveTowardsGate1, driveToGate1))
                .setLinearHeadingInterpolation(driveToShoot2.getHeading(), driveToGate1.getHeading())
                .build();

        // kept (unused in new flow)
        goShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(driveToGate1, driveToShoot3))
                .setLinearHeadingInterpolation(driveToGate1.getHeading(), driveToShoot3.getHeading())
                .build();

        onWaytoGate2 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot3, driveTowardsGate2))
                .setLinearHeadingInterpolation(driveToShoot3.getHeading(), driveTowardsGate2.getHeading())
                .build();

        toGate2 = follower.pathBuilder()
                .addPath(new BezierLine(driveTowardsGate2, driveToGate2))
                .setLinearHeadingInterpolation(driveTowardsGate2.getHeading(), driveToGate2.getHeading())
                .build();

        // gate2 combined (unchanged)
        goGate2Combined = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot3, driveTowardsGate2))
                .setLinearHeadingInterpolation(driveToShoot3.getHeading(), driveTowardsGate2.getHeading())
                .addPath(new BezierLine(driveTowardsGate2, driveToGate2))
                .setLinearHeadingInterpolation(driveTowardsGate2.getHeading(), driveToGate2.getHeading())
                .build();

        // kept (unused in new flow)
        goShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(driveToGate2, driveToShoot4))
                .setLinearHeadingInterpolation(driveToGate2.getHeading(), driveToShoot4.getHeading())
                .build();

        // leaveOutChain now happens AFTER Shoot5
        leaveOutChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot5, leavePose))
                .setLinearHeadingInterpolation(driveToShoot5.getHeading(), leavePose.getHeading())
                .build();

        // gate -> intake post (gate 1 uses intakeFromGate1)
        gate1ToIntakePost = follower.pathBuilder()
                .addPath(new BezierLine(driveToGate1, intakeFromGate1))
                .setLinearHeadingInterpolation(driveToGate1.getHeading(), intakeFromGate1.getHeading())
                .build();

        intakePostToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeFromGate1, driveToShoot3))
                .setLinearHeadingInterpolation(intakeFromGate1.getHeading(), driveToShoot3.getHeading())
                .build();

        // gate 2 uses intakeFromGate2
        gate2ToIntakePost = follower.pathBuilder()
                .addPath(new BezierLine(driveToGate2, intakeFromGate2))
                .setLinearHeadingInterpolation(driveToGate2.getHeading(), intakeFromGate2.getHeading())
                .build();

        intakePostToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(intakeFromGate2, driveToShoot4))
                .setLinearHeadingInterpolation(intakeFromGate2.getHeading(), driveToShoot4.getHeading())
                .build();

        // ====== NEW (requested) ======
        // After Shoot4, go to Line1 entry
        shoot4ToLine1 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot4, toArtifactLine1))
                .setLinearHeadingInterpolation(driveToShoot4.getHeading(), toArtifactLine1.getHeading())
                .build();

        // Drive through Line1 (intake slow during this path)
        line1DriveThrough = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine1, driveThroughLine1))
                .setLinearHeadingInterpolation(toArtifactLine1.getHeading(), driveThroughLine1.getHeading())
                .build();

        // From driveThroughLine1 to Shoot5
        line1ToShoot5 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughLine1, driveToShoot5))
                .setLinearHeadingInterpolation(driveThroughLine1.getHeading(), driveToShoot5.getHeading())
                .build();
        // ============================
    }

    // ================= STATE HELPERS =================
    private void setPathState(int newState) {
        pathState = newState;
        stateStartTime = getRuntime();
        if (pathTimer != null) pathTimer.resetTimer();

        shootingSequenceActive = false;
        shootingSequenceStart = 0.0;

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

        intakeShootFeed();
        return (now - shootingSequenceStart) >= SHOOT_FEED_SEC;
    }

    /** Subsequent shot windows (NO flywheel spinup; adds 200ms settle time). */
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

        intakeShootFeed();
        return (now - shootingSequenceStart) >= SHOOT_FEED_SEC;
    }

    // ================= AUTO STATE MACHINE =================
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                intakeStop();
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (runShootWindowFirst()) {
                        intakeSlow();
                        follower.followPath(driveToLine2, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine2, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(goShoot2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeStop();
                        follower.followPath(goGate1Combined, true);
                        setPathState(6);
                    }
                }
                break;

            // ---- Gate 1 behavior ----
            case 6:
                if (!follower.isBusy()) {
                    intakeStop();
                    if (pauseTime(0.5)) {
                        follower.followPath(gate1ToIntakePost, true);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intakeSlow();
                    if (pauseTime(1.5)) {
                        intakeStop();
                        follower.followPath(intakePostToShoot3, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeStop();
                        follower.followPath(goGate2Combined, true);
                        setPathState(9);
                    }
                }
                break;

            // ---- Gate 2 behavior ----
            case 9:
                if (!follower.isBusy()) {
                    intakeStop();
                    if (pauseTime(0.5)) {
                        follower.followPath(gate2ToIntakePost, true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    intakeSlow();
                    if (pauseTime(1.5)) {
                        intakeStop();
                        follower.followPath(intakePostToShoot4, true);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        // ====== ONLY CHANGE STARTS HERE (requested addition) ======
                        intakeSlow();
                        follower.followPath(shoot4ToLine1, true);
                        setPathState(12);
                        // =========================================================
                    }
                }
                break;

            // ====== NEW (requested) ======
            case 12:
                if (!follower.isBusy()) {
                    // Drive through Line1 with slow intake (same style as Line2)
                    intakeSlow();
                    follower.followPath(line1DriveThrough, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    // After pickup, stop intake and go to Shoot5
                    intakeStop();
                    follower.followPath(line1ToShoot5, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    // Shoot5
                    if (runShootWindowQuick()) {
                        intakeStop();
                        follower.followPath(leaveOutChain, true);
                        setPathState(15);
                    }
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    intakeStop();
                    setPathState(-1);
                }
                break;
            // ============================

            default:
                break;
        }
    }

    // ================= SHOOTER: KICK -> PIDF =================
    private SpinUpFastKickThenPIDF spinup;
    private boolean shooterSpinupStarted = false;

    // ================= LOOP =================
    @Override
    public void loop() {

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        // Shooter always on (kick -> PIDF)
        if (!shooterSpinupStarted) {
            spinup.kickMs = 1800;
            spinup.kickPower = 1.0;
            spinup.earlyTakeoverPct = 0.98;
            spinup.start(1230);
            shooterSpinupStarted = true;
        }
        spinup.update();

        autonomousPathUpdate();
        follower.update();

        double speed = follower.getVelocity().getMagnitude();
        double moveBlend = clip(speed / SPEED_FULL_BLEND, 0, 1);

        double omegaRad = follower.getAngularVelocity();
        double omegaDps = omegaRad * (180.0 / Math.PI);

        turret.update(now, dt, moveBlend, omegaDps);

        telemetry.addData("pathState", pathState);
        telemetry.addData("firstSpinupDone", firstSpinupDone);
        telemetry.addData("timeInState", "%.2f", (now - stateStartTime));
        telemetry.addData("aimStable", turret.isAimedStable(now));
        telemetry.addData("tx", "%.2f", turret.getLastTx());
        telemetry.addData("turretPwr", "%.3f", turret.getPower());
        telemetry.addData("Shooter Target / Actual", "%.0f / %.0f", target, shooter.getVelocity());
        telemetry.update();
    }

    // ================= INIT =================
    @Override
    public void init() {

        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        fi = hardwareMap.dcMotor.get("fi");
        bi = hardwareMap.dcMotor.get("bi");
        fi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        intakeStop();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        } catch (Exception ignored) {}

        spinup = new SpinUpFastKickThenPIDF(shooter);
        shooterSpinupStarted = false;

        lt = hardwareMap.get(Servo.class, "lt");
        rt = hardwareMap.get(Servo.class, "rt");
        ki = hardwareMap.get(Servo.class, "ki");

        lt.setPosition(0.3);
        rt.setPosition(0.3);
        ki.setPosition(0.2);

        turret.init(hardwareMap);

        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        firstSpinupDone = false;

        setPathState(-1);
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        firstSpinupDone = false;
        shooterSpinupStarted = false;

        setPathState(0);
    }

    @Override
    public void stop() {
        // no-op
    }

    // ================= INTAKE =================
    private void intakeSlow() {
        shootPulseActive = false;
        fi.setPower(1.0);
        bi.setPower(-0.65);
    }

    /** Pulse ON/OFF continuously the entire time this is called. */
    private void intakeShootFeed() {

        double now = getRuntime();

        if (!shootPulseActive) {
            shootPulseActive = true;
            shootPulseStart = now;
        }

        double on = SHOOT_PULSE_ON_SEC;
        double off = SHOOT_PULSE_OFF_SEC;
        double period = on + off;
        if (period <= 0.0001) period = 0.0001;

        double t = now - shootPulseStart;
        double phase = t % period;

        if (phase < on) {
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

    // ================= TURRET TRACKER =================
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

        private static final double KP = 0.045;
        private static final double KD = 0.003;

        private static final double FF = 0.012;
        private static final double LEAD_TIME = 0.03;

        private static final double AIM_TOL = 2.5;
        private static final double AIM_PWR_TOL = 0.12;
        private static final double AIM_STABLE = 0.12;

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

            double txPred = (TX_SIGN * txRaw) + (LEAD_SIGN * omegaDps * LEAD_TIME);

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