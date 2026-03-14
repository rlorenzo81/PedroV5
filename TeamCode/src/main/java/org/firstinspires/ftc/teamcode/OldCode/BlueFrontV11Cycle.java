package org.firstinspires.ftc.teamcode.OldCode;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Front Cycle2", group = "Examples")
@Disabled
public class BlueFrontV11Cycle extends OpMode {

    // ================= HARDWARE =================
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private double stateStartTime = 0.0;

    private static final double SHOOTER_TARGET_TPS = 1530; //shooter speed (CHANGE THIS) was 1445

    // ================= FLYWHEEL SPEED GATE =================
    private static final double FLYWHEEL_TOLERANCE_TPS = 35;  // start 25–40
    private static final double FLYWHEEL_STABLE_TIME = 0.12;  // seconds at speed
    private double flywheelStableStart = 0.0;

    // ================= TURRET =================
    private final TurretTracker turret = new TurretTracker();
    private double lastLoopTime = 0;
    private boolean turretLockedAfterPreload = false;

    // ================= SHOOTER COMP (kept as in originals) =================
    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double MIN_VOLTAGE_FOR_COMP = 10.5;
    private static final double MAX_COMP_MULT = 1.35;
    private static final double MIN_COMP_MULT = 0.85;

    private static final double SHOOTER_KP = 300;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.006;//0.006
    private static final double SHOOTER_KF = 15.5;

    // ================= MOVEMENT BLEND =================
    private static final double SPEED_FULL_BLEND = 40.0;

    // ================= SCORE WINDOW (AIM TIMEOUT) =================
    private static final double AIM_TIMEOUT_SEC = .8; // was.35

    // ================= LATCHED SHOOTING SEQUENCE =================
    private boolean shootingSequenceActive = false;
    private double shootingSequenceStart = 0.0;
    private static final double SHOOT_FEED_SEC = 1.8; // from RedBackV9

    // ================= SHOOTER SPIN-UP DELAY =================
    // ONLY used for FIRST shot window now
    private static final double SHOOTER_SPINUP_SEC = 3.5; // from RedBackV9 was 6

    // ================= QUICK SETTLE FOR SUBSEQUENT SHOTS =================
    private static final double SHOOT_SETTLE_SEC = 2.0;

    // ================= FIRST-SPINUP DONE FLAG =================
    private boolean firstSpinupDone = false;

    // ================= SHOOT FEED PULSE (FROM FIRST CLASS) =================
    private static final double FEED_ON_SEC  = 0.20;
    private static final double FEED_OFF_SEC = 0.5;

    private boolean feedPulseOn = false;
    private double feedNextToggleTime = 0.0;

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
    private final Pose startPose = new Pose(60, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 14, Math.toRadians(109));//12
    private final Pose toArtifactLine1 = new Pose(44, 34, Math.toRadians(180));//x=38
    private final Pose driveThroughLine1 = new Pose(11, 34, Math.toRadians(180));
    private final Pose driveToShoot2 = new Pose(60, 14, Math.toRadians(109));

    // (kept declared but NO LONGER USED in paths/state machine)
    private final Pose toArtifactLine2 = new Pose(46, 56, Math.toRadians(180));//x was 44
    private final Pose driveThroughLine2 = new Pose(11, 56, Math.toRadians(180));

    private final Pose driveToShoot3 = new Pose(60, 14, Math.toRadians(109)); //116

    // ===== ADDED POSES (per your request) =====
    private final Pose cornerPickUp1 = new Pose(10, 10, Math.toRadians(180));
    private final Pose driveToShoot4 = new Pose(60, 14, Math.toRadians(109));
    private final Pose cornerPickUp2 = new Pose(10, 10, Math.toRadians(180));
    private final Pose driveToShoot5 = new Pose(60, 14, Math.toRadians(109));

    private final Pose leavePose = new Pose(33, 26, Math.toRadians(180));

    private Path scorePreload;

    // removed driveToLine2 / pickUpLine2 from the used chains
    private PathChain driveToLine1, pickUpLine1, goShoot2, goShoot3;

    // ===== ADDED CHAINS =====
    private PathChain goCorner1, goShoot4, goCorner2, goShoot5, leaveOutChain;

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

        // NEW: from goShoot2 we go straight to cornerPickUp1 (no line2 paths)
        goCorner1 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot2, cornerPickUp1))
                .setLinearHeadingInterpolation(driveToShoot2.getHeading(), cornerPickUp1.getHeading())
                .build();

        // From cornerPickUp1 to shoot4
        goShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(cornerPickUp1, driveToShoot4))
                .setLinearHeadingInterpolation(cornerPickUp1.getHeading(), driveToShoot4.getHeading())
                .build();

        // From shoot4 back to cornerPickUp2
        goCorner2 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot4, cornerPickUp2))
                .setLinearHeadingInterpolation(driveToShoot4.getHeading(), cornerPickUp2.getHeading())
                .build();

        // From cornerPickUp2 to shoot5
        goShoot5 = follower.pathBuilder()
                .addPath(new BezierLine(cornerPickUp2, driveToShoot5))
                .setLinearHeadingInterpolation(cornerPickUp2.getHeading(), driveToShoot5.getHeading())
                .build();

        // Keep goShoot3 path definition, but now it starts from driveToShoot5
        goShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot5, driveToShoot3))
                .setLinearHeadingInterpolation(driveToShoot5.getHeading(), driveToShoot3.getHeading())
                .build();

        // Finally leave pose (still from driveToShoot5 in your current cycle)
        leaveOutChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot5, leavePose))
                .setLinearHeadingInterpolation(driveToShoot5.getHeading(), leavePose.getHeading())
                .build();
    }

    // ================= STATE HELPERS =================
    private void setPathState(int newState) {
        pathState = newState;
        stateStartTime = getRuntime();
        if (pathTimer != null) pathTimer.resetTimer();

        shootingSequenceActive = false;
        shootingSequenceStart = 0.0;

        // reset pulse state whenever we change states (keeps shooting pulse clean)
        feedPulseOn = false;
        feedNextToggleTime = 0.0;
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
            boolean speedOk = flywheelAtSpeed(now);
            if (aimOk || timedOut) {
                shootingSequenceActive = true;
                shootingSequenceStart = now;

                // start pulse ON immediately
                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;

            } else {
                intakeStop();
                return false;
            }
        }

        // PULSED FEED during shooting window
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
            boolean speedOk = flywheelAtSpeed(now);
            if (aimOk || timedOut) {
                shootingSequenceActive = true;
                shootingSequenceStart = now;

                // start pulse ON immediately
                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;

            } else {
                intakeStop();
                return false;
            }
        }

        // PULSED FEED during shooting window
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
                    if (!turretLockedAfterPreload) {
                        turret.lockCurrentPosition();
                        turretLockedAfterPreload = true;
                    }
                    if (runShootWindowFirst()) {
                        intakeSlow();
                        follower.followPath(driveToLine1, 1.0,true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine1,1.0, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intakeHolds();
                    follower.followPath(goShoot2, 1.0, true);
                    setPathState(4);
                }
                break;

            case 4:
                // After goShoot2 we go to cornerPickUp1 (removed line2)
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeSlow();
                        follower.followPath(goCorner1, 1.0,true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                // Arrived cornerPickUp1, now go to shoot 4
                if (!follower.isBusy()) {
                    intakeHolds();
                    follower.followPath(goShoot4, 1.0, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Shoot 4, then go back to cornerPickUp2 with intakeSlow the whole drive
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeSlow();
                        follower.followPath(goCorner2, 1.0,true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                // Arrived cornerPickUp2, now go to shoot 5
                if (!follower.isBusy()) {
                    intakeHolds();
                    follower.followPath(goShoot5, 1.0, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Shoot 5, then leave
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeStop();
                        follower.followPath(leaveOutChain,1.0, true);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intakeStop();
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    // ================= SHOOTER: RAMP-VELOCITY (FROM FIRST CLASS) =================
    private SpinUpRampVelocity spinup;
    private boolean shooterSpinupStarted = false;

    // ================= LOOP =================
    @Override
    public void loop() {

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        // Shooter always on (RAMP VELOCITY ONLY)
        if (!shooterSpinupStarted) {
            spinup.start(SHOOTER_TARGET_TPS, now);
            shooterSpinupStarted = true;
        }
        spinup.update(now);

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

        // NEW: ramp-velocity spinup (matches first class logic)
        spinup = new SpinUpRampVelocity(shooter);
        shooterSpinupStarted = false;

        // Servos
        lt = hardwareMap.get(Servo.class, "lt");
        rt = hardwareMap.get(Servo.class, "rt");
        ki = hardwareMap.get(Servo.class, "ki");

        lt.setPosition(0.2);
        rt.setPosition(1.0); //was .
        ki.setPosition(0.15);

        // Turret tracker init
        turret.init(hardwareMap);

        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        // Only spin up once per autonomous
        firstSpinupDone = false;
        turretLockedAfterPreload = false;

        setPathState(-1);
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        // Reset the “spinup once” flag at match start
        firstSpinupDone = false;

        // Reset shooter ramp-start
        shooterSpinupStarted = false;
        turretLockedAfterPreload = false;

        setPathState(0);
    }

    @Override
    public void stop() {
        // no-op
    }

    // ================= INTAKE (PULSE FROM FIRST CLASS) =================
    private void intakeSlow() {
        feedPulseOn = false;
        feedNextToggleTime = 0.0;
        fi.setPower(1.0);
        bi.setPower(-0.65);
    }

    /** PULSED feed: ON for FEED_ON_SEC, OFF for FEED_OFF_SEC while called. */
    private void intakeShootFeed() {
        double now = getRuntime();

        if (feedNextToggleTime <= 0.0) {
            feedPulseOn = true;
            feedNextToggleTime = now + FEED_ON_SEC;
        } else if (now >= feedNextToggleTime) {
            feedPulseOn = !feedPulseOn;
            feedNextToggleTime = now + (feedPulseOn ? FEED_ON_SEC : FEED_OFF_SEC);
        }

        if (feedPulseOn) {
            fi.setPower(1.0);
            bi.setPower(1.0);
        } else {
            fi.setPower(0.0);
            bi.setPower(0.0);
        }
    }

    private void intakeStop() {
        feedPulseOn = false;
        feedNextToggleTime = 0.0;
        fi.setPower(0.0);
        bi.setPower(0.0);
    }

    private void intakeHolds() {
        feedPulseOn = false;
        feedNextToggleTime = 0.0;
        fi.setPower(0.1);
        bi.setPower(0.1);
    }

    // ================= SHOOTER (RAMP-VELOCITY HELPER) =================
    // Matches the first class "SpinUpRampVelocity" approach: velocity ramp only, no raw kick.
    public static class SpinUpRampVelocity {

        public double rampSec = 0.50;     // tune if needed
        public double minRampSec = 0.12;  // safety
        public double maxRampSec = 1.50;  // safety

        private boolean active = false;
        private double startTime = 0.0;   // OpMode runtime seconds
        private double targetVel = 0.0;

        private double startVel = 0.0;
        private double cmdVel = 0.0;

        private final DcMotorEx shooter;

        public SpinUpRampVelocity(DcMotorEx shooterMotor) {
            this.shooter = shooterMotor;
        }

        public void start(double targetTicksPerSec, double nowSec) {
            this.targetVel = targetTicksPerSec;
            this.startTime = nowSec;
            this.active = true;

            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            startVel = Math.max(0.0, shooter.getVelocity());
            cmdVel = startVel;
            shooter.setVelocity(cmdVel);
        }

        public void update(double nowSec) {
            if (!active) return;

            double r = clip(rampSec, minRampSec, maxRampSec);
            double elapsed = nowSec - startTime;

            double t = clip(elapsed / r, 0.0, 1.0);
            cmdVel = lerp(startVel, targetVel, t);

            shooter.setVelocity(cmdVel);

            if (t >= 1.0) {
                cmdVel = targetVel;
                shooter.setVelocity(cmdVel);
                active = false;
            }
        }

        public void stop() {
            active = false;
            cmdVel = 0.0;
        }
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
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

        private boolean locked = false;
        private int lockedTicks = 0;

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
        private static final double TURRET_AIM_OFFSET_DEG = -0.4; // was 0.5

        private static final double LOCK_KP = 0.003;
        private static final double LOCK_MAX_PWR = 0.35;

        public void init(HardwareMap hw) {
            turret = hw.get(DcMotorEx.class, "turretSpin");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setDirection(DcMotorSimple.Direction.REVERSE);

            limelight = hw.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
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
            locked = false;
            lockedTicks = 0;
        }

        public void lockCurrentPosition() {
            lockedTicks = turret.getCurrentPosition();
            locked = true;
            aimHold = true;
        }

        public void update(double now, double dt, double moveBlend, double omegaDps) {

            if (dt < 0.001) dt = 0.001;

            if (locked) {
                double errorTicks = lockedTicks - turret.getCurrentPosition();
                double power = clip(errorTicks * LOCK_KP, -LOCK_MAX_PWR, LOCK_MAX_PWR);
                turret.setPower(power);
                lastPower = power;
                return;
            }

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
            return locked || (aimHold && (now - aimStart) > AIM_STABLE);
        }

        public double getLastTx() { return lastTx; }
        public double getPower() { return lastPower; }
    }

    private boolean flywheelAtSpeed(double now) {
        double actual = shooter.getVelocity();
        double error = Math.abs(actual - SHOOTER_TARGET_TPS);

        if (error < FLYWHEEL_TOLERANCE_TPS) {
            if (flywheelStableStart == 0.0) {
                flywheelStableStart = now;
            }
            return (now - flywheelStableStart) >= FLYWHEEL_STABLE_TIME;
        } else {
            flywheelStableStart = 0.0;
            return false;
        }
    }
}