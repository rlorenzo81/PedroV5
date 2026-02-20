package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Front V6", group = "Examples")
public class RedFrontV6 extends OpMode {

    // ================= HARDWARE =================
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

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

    // ================= SCORE WINDOW (AIM TIMEOUT) =================
    private boolean scoreWindowStarted = false;
    private double scoreWindowStart = 0.0;
    private static final double AIM_TIMEOUT_SEC = 0.35; // shoot anyway after this

    // ================= SHOOT FEED PULSE STATE =================
    private boolean shootPulseActive = false;
    private double shootPulseStart = 0;

    private static final double SHOOT_PULSE_ON_SEC = 0.30;
    private static final double SHOOT_PULSE_OFF_SEC = 0.5;

    // ================= LATCHED SHOOTING SEQUENCE =================
    private boolean shootingSequenceActive = false;
    private double shootingSequenceStart = 0.0;
    private static final double SHOOT_FEED_SEC = 1.5; // how long to feed balls through

    // ================= NEW: SHOOTER SPIN-UP DELAY =================
    private static final double SHOOTER_SPINUP_SEC = 1.8; // give flywheel time before feeding

    // ================= POSES =================
    private final Pose startPose = new Pose(84, 18, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 25, Math.toRadians(65));

    private Path scorePreload;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    // ================= AUTO STATE =================
    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start movement immediately
                intakeStop();
                follower.followPath(scorePreload);

                // reset gates for this scoring action
                scoreWindowStarted = false;
                scoreWindowStart = 0.0;
                shootingSequenceActive = false;
                shootingSequenceStart = 0.0;

                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {

                    // start a scoring window timer the first time we arrive
                    if (!scoreWindowStarted) {
                        scoreWindowStarted = true;
                        scoreWindowStart = getRuntime();
                    }

                    double timeAtPose = getRuntime() - scoreWindowStart;

                    // ===============================
                    // V6: Flywheel spin-up window
                    // ===============================
                    if (timeAtPose < SHOOTER_SPINUP_SEC) {
                        intakeStop(); // wait for flywheel RPM
                        break;
                    }

                    boolean aimOk = turret.isAimedStable(getRuntime());
                    boolean timedOut = timeAtPose >= (AIM_TIMEOUT_SEC + SHOOTER_SPINUP_SEC);

                    // If we haven't started shooting yet, decide when to start
                    if (!shootingSequenceActive) {
                        if (aimOk || timedOut) {
                            shootingSequenceActive = true;
                            shootingSequenceStart = getRuntime();
                        } else {
                            intakeStop();
                            break;
                        }
                    }

                    // Once shooting starts, latch intake ON for full duration
                    intakeShootFeed();

                    if ((getRuntime() - shootingSequenceStart) >= SHOOT_FEED_SEC) {
                        intakeSlow();
                        pathState = -1; // stop after preload in this simplified routine
                    }
                }
                break;

            default:
                // do nothing
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

        // Shooter always on (no spool delay)
        shooterOnComp(2350);

        // IMPORTANT: state machine FIRST so the follower has a path THIS cycle
        autonomousPathUpdate();
        follower.update();

        // Movement blend
        double speed = follower.getVelocity().getMagnitude();
        double moveBlend = clip(speed / SPEED_FULL_BLEND, 0, 1);

        // Pedro getAngularVelocity() is radians/sec
        double omegaRad = follower.getAngularVelocity();
        double omegaDps = omegaRad * (180.0 / Math.PI);

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

        telemetry.addData("spinup left (s)", scoreWindowStarted
                ? String.format("%.2f", Math.max(0.0, SHOOTER_SPINUP_SEC - (now - scoreWindowStart)))
                : "n/a");

        telemetry.addData("scoreWindowStarted", scoreWindowStarted);
        if (scoreWindowStarted) {
            telemetry.addData("aim timeout left", "%.2f",
                    Math.max(0.0, (AIM_TIMEOUT_SEC + SHOOTER_SPINUP_SEC) - (now - scoreWindowStart)));
        }
        telemetry.addData("shootingActive", shootingSequenceActive);
        if (shootingSequenceActive) {
            telemetry.addData("shoot time left", "%.2f",
                    Math.max(0.0, SHOOT_FEED_SEC - (now - shootingSequenceStart)));
        }

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

        // Servos (same names + init positions as your RCRED teleop)
        lt = hardwareMap.get(Servo.class, "lt"); //2
        rt = hardwareMap.get(Servo.class, "rt"); //1
        ki = hardwareMap.get(Servo.class, "ki"); //0

        lt.setPosition(0.35);
        rt.setPosition(0.35);
        ki.setPosition(0.2);

        // Turret tracker init
        turret.init(hardwareMap);

        // Reset timing / gates
        lastLoopTime = getRuntime();

        pauseActive = false;
        pauseEnd = 0;

        scoreWindowStarted = false;
        scoreWindowStart = 0.0;

        shootingSequenceActive = false;
        shootingSequenceStart = 0.0;
    }

    @Override
    public void start() {
        pathState = 0;

        // Reset pause + score window + shooting latch so nothing deadlocks
        pauseActive = false;
        pauseEnd = 0;

        scoreWindowStarted = false;
        scoreWindowStart = 0.0;

        shootingSequenceActive = false;
        shootingSequenceStart = 0.0;

        lastLoopTime = getRuntime();
    }

    @Override
    public void stop() {
        // no-op
    }

    // ================= INTAKE =================
    private void intakeSlow() {
        shootPulseActive=false;
        fi.setPower(1.0);
        bi.setPower(-0.65);
    }

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

        // Phase 2: OFF for 0.1s
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

    // ================= TURRET TRACKER (WITH LOOSER STABLE GATE + ALPHA TWEAK) =================
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