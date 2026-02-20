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

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Front V7", group = "Examples")
public class RedFrontV7 extends OpMode {

    // ================= HARDWARE =================
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private double stateStartTime = 0.0;

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
    // Pedro velocity magnitude units are typically inches/sec (depends on config), this is just a blend scaler
    private static final double SPEED_FULL_BLEND = 40.0;

    // ================= SCORE WINDOW (AIM TIMEOUT) =================
    // shoot anyway after this additional time (after spinup window)
    private static final double AIM_TIMEOUT_SEC = 0.35;

    // ================= LATCHED SHOOTING SEQUENCE =================
    private boolean shootingSequenceActive = false;
    private double shootingSequenceStart = 0.0;
    private static final double SHOOT_FEED_SEC = 1.5;  // how long to feed balls through

    // ================= SHOOTER SPIN-UP DELAY =================
    private static final double SHOOTER_SPINUP_SEC = 1.8; // give flywheel time before feeding

    // ================= SHOOT FEED PULSE STATE =================
    private boolean shootPulseActive = false;
    private double shootPulseStart = 0;

    // FIXED per your request:
    private static final double SHOOT_PULSE_ON_SEC = 0.25;
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

    // ================= STATE HELPERS =================
    private void setPathState(int newState) {
        pathState = newState;
        stateStartTime = getRuntime();
        if (pathTimer != null) pathTimer.resetTimer();

        // reset per-state shoot latch so it never "sticks" between shoot states
        shootingSequenceActive = false;
        shootingSequenceStart = 0.0;

        // reset pulse so first call to intakeShootFeed restarts properly in each shoot window
        shootPulseActive = false;
    }

    /** Common “shoot window” used at scorePose / shoot2 / shoot3 */
    private boolean runShootWindow() {
        double now = getRuntime();
        double t = now - stateStartTime;

        // 1) Spin-up window: do NOT feed
        if (t < SHOOTER_SPINUP_SEC) {
            intakeStop();
            return false;
        }

        // 2) Wait for aim stable OR timeout (after spinup)
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

        // 3) Feed for SHOOT_FEED_SEC (latched)
        intakeShootFeed();

        if ((now - shootingSequenceStart) >= SHOOT_FEED_SEC) {
            return true;
        }
        return false;
    }

    // ================= AUTO STATE MACHINE =================
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Start: go to score pose
                intakeStop();
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // At score pose: shoot preload, then go to line 1
                if (!follower.isBusy()) {
                    if (runShootWindow()) {
                        intakeSlow(); // start collecting while heading to line 1
                        follower.followPath(driveToLine1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                // Drive to line 1 endpoint, then sweep through line 1
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine1, true);
                    setPathState(3);
                }
                break;

            case 3:
                // After sweeping line 1, go back to shoot 2
                if (!follower.isBusy()) {
                    intakeStop(); // stop while traveling back to shoot pose (optional)
                    follower.followPath(goShoot2, true);
                    setPathState(4);
                }
                break;

            case 4:
                // At shoot 2 pose: shoot, then go to line 2
                if (!follower.isBusy()) {
                    if (runShootWindow()) {
                        intakeSlow();
                        follower.followPath(driveToLine2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                // Drive to line 2 endpoint, then sweep through line 2
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine2, true);
                    setPathState(6);
                }
                break;

            case 6:
                // After sweeping line 2, go back to shoot 3
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.followPath(goShoot3, true);
                    setPathState(7);
                }
                break;

            case 7:
                // At shoot 3 pose: shoot, then leave
                if (!follower.isBusy()) {
                    if (runShootWindow()) {
                        intakeStop();
                        follower.followPath(leaveOutChain, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                // Done when leave path completes
                if (!follower.isBusy()) {
                    intakeStop();
                    setPathState(-1);
                }
                break;

            default:
                // idle
                break;
        }
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        // Shooter always on (no spool delay needed from driver)
        shooterOnComp(2350);

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
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading(rad)", follower.getPose().getHeading());
        telemetry.addData("speed", "%.2f", speed);
        telemetry.addData("omega dps", "%.2f", omegaDps);

        telemetry.addData("tx", "%.2f", turret.getLastTx());
        telemetry.addData("turret pwr", "%.3f", turret.getPower());
        telemetry.addData("aim stable", turret.isAimedStable(now));

        telemetry.addData("time in state", "%.2f", (now - stateStartTime));
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

        // Servos
        lt = hardwareMap.get(Servo.class, "lt"); //2
        rt = hardwareMap.get(Servo.class, "rt"); //1
        ki = hardwareMap.get(Servo.class, "ki"); //0

        lt.setPosition(0.35);
        rt.setPosition(0.35);
        ki.setPosition(0.2);

        // Turret tracker init
        turret.init(hardwareMap);

        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        setPathState(-1); // idle until start()
    }

    @Override
    public void start() {
        lastLoopTime = getRuntime();

        // Reset pause
        pauseActive = false;
        pauseEnd = 0;

        // Start routine
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

    /** Pulse ON for 0.25s, OFF for 0.10s, then ON forever until another intake mode is called. */
    private void intakeShootFeed() {

        double now = getRuntime();

        // Start pulse the first time this function is called
        if (!shootPulseActive) {
            shootPulseActive = true;
            shootPulseStart = now;
        }

        double t = now - shootPulseStart;

        // Phase 1: ON for 0.25s
        if (t < SHOOT_PULSE_ON_SEC) {
            fi.setPower(1.0);
            bi.setPower(1.0);
            return;
        }

        // Phase 2: OFF for 0.10s
        if (t < SHOOT_PULSE_ON_SEC + SHOOT_PULSE_OFF_SEC) {
            fi.setPower(0.0);
            bi.setPower(0.0);
            return;
        }

        // Phase 3: ON forever
        fi.setPower(1.0);
        bi.setPower(1.0);
    }

    private void intakeStop() {
        shootPulseActive = false;
        fi.setPower(0.0);
        bi.setPower(0.0);
    }

    // ================= SHOOTER =================
    private void shooterOnComp(double targetTicksPerSec) {
        double mult = voltageComp();
        shooter.setVelocity(targetTicksPerSec * mult);
    }

    private double voltageComp() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor v : hardwareMap.voltageSensor) {
            double val = v.getVoltage();
            if (val > 0) min = Math.min(min, val);
        }
        if (!Double.isFinite(min) || min <= 0) min = NOMINAL_VOLTAGE;

        min = Math.max(min, MIN_VOLTAGE_FOR_COMP);
        double m = NOMINAL_VOLTAGE / min;
        return clip(m, MIN_COMP_MULT, MAX_COMP_MULT);
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

        // derivative state
        private double lastErr = 0;

        // =========================================================
        // SIGN KNOBS
        // =========================================================
        // If turret turns WRONG way when tx > 0, flip TX_SIGN to -1.
        private static final int TX_SIGN = 1;

        // If turret behaves fine when NOT turning, but "leads away" while turning, flip LEAD_SIGN to -1.
        private static final int LEAD_SIGN = 1;

        // =========================================================
        // Tunables
        // =========================================================
        private static final double KP = 0.045;
        private static final double KD = 0.003;

        // feedforward base (scaled down near center)
        private static final double FF = 0.012;

        // predictive lead reduced to prevent overshoot
        private static final double LEAD_TIME = 0.03;

        // ===== Loosened aiming gate =====
        private static final double AIM_TOL = 2.5;
        private static final double AIM_PWR_TOL = 0.12;
        private static final double AIM_STABLE = 0.12;

        public void init(HardwareMap hw) {
            turret = hw.get(DcMotorEx.class, "turretSpin");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // IMPORTANT: match your TeleOp (RCRED) direction
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

            // reset state
            lastTx = 0;
            lastPower = 0;
            txFiltered = 0;
            aimHold = false;
            aimStart = 0;
            lastErr = 0;
        }

        /** omegaDps should be deg/sec (you already convert from Pedro rad/sec) */
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

            // 1) Predictive lead + SIGN KNOBS
            double txPred = (TX_SIGN * txRaw) + (LEAD_SIGN * omegaDps * LEAD_TIME);

            // 2) Smoothing (less lag while moving)
            double alpha = (moveBlend > 0.2) ? 0.28 : 0.55;
            txFiltered = alpha * txFiltered + (1.0 - alpha) * txPred;

            double error = txFiltered;
            double errAbs = Math.abs(error);

            // 3) Derivative
            double dErr = (error - lastErr) / dt;
            lastErr = error;

            double p = KP * error;
            double d = KD * dErr;

            // 4) Feedforward fades out near center
            double ffScale = clip((errAbs - 1.5) / (8.0 - 1.5), 0.0, 1.0);
            double ff = FF * omegaDps * (1.0 + moveBlend) * ffScale;

            double power = p + d + ff;

            // 5) Braking near center
            double brake = clip(errAbs / 6.0, 0.25, 1.0);
            power *= brake;

            // cap
            power = clip(power, -0.75, 0.75);

            turret.setPower(power);
            lastPower = power;

            // Aim-stable gate
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

        private static double clip(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }
    }
}