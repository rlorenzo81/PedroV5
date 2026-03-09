package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Back V17", group = "Examples")
public class RedBackV17 extends OpMode {

    // ================= HARDWARE =================
    private DcMotor fi, bi;
    private DcMotorEx shooter;
    private Servo lt, rt, ki;

    // ================= PEDRO =================
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private double stateStartTime = 0.0;

    // RCRed-style target variable (ticks/sec). Change this value to change flywheel velocity.
    double target = 1124.0; // change if needed was 1150 working good but some bounce outr

    // ================= TURRET =================
    private final TurretTracker turret = new TurretTracker();
    private double lastLoopTime = 0;

    // ================= SHOOTER PIDF (MATCH RCRed) =================
    private static final double SHOOTER_KP = 300;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.001;
    private static final double SHOOTER_KF = 12.5;

    // ================= MOVEMENT BLEND =================
    private static final double SPEED_FULL_BLEND = 40.0;

    // ================= SCORE WINDOW (AIM TIMEOUT) =================
    private static final double AIM_TIMEOUT_SEC = 0.5; //was 0.5

    private static final int TURRET_OFFSET_TICKS = 0; // adjust this number

    // ================= LATCHED SHOOTING SEQUENCE =================
    private boolean shootingSequenceActive = false;
    private double shootingSequenceStart = 0.0;
    private static final double SHOOT_FEED_SEC = 1.3; //was 1.6 changed

    // ================= SHOOTER SPIN-UP DELAY =================
    // ONLY used for FIRST shot window now
    private static final double SHOOTER_SPINUP_SEC = 1.25; //was 2.0 (how long the robot has to speed up to shoot) changed

    // ================= QUICK SETTLE FOR SUBSEQUENT SHOTS =================
    private static final double SHOOT_SETTLE_SEC = 0.5; //was 1.0 changed

    // ================= FIRST-SPINUP DONE FLAG =================
    private boolean firstSpinupDone = false;

    // ================= SHOOT FEED PULSE (REQUESTED) =================
    private static final double FEED_ON_SEC  = 0.20;
    private static final double FEED_OFF_SEC = 0.30;

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

    // ================= POSES =================

    private final Pose startPose = new Pose(120, 126, Math.toRadians(45)); //done was 62, angle was 45
    private final Pose scorePose = new Pose(98, 97, Math.toRadians(41));// was 45, 84 done angle was 42

    // ====== Line1 pickup + exit curve ======
    private final Pose toArtifactLine1 = new Pose(100, 81, Math.toRadians(0)); //y was 84 done
    private final Pose driveThroughLine1 = new Pose(130, 81, Math.toRadians(0)); //y was 84 done

    private final Pose line1TowardsGate  = new Pose(127, 81, Math.toRadians(0)); //done
    private final Pose line1PressGate = new Pose(134, 75, Math.toRadians(0)); //done
    private final Pose driveToShoot2 = new Pose(107, 97, Math.toRadians(41)); // was 93,93 (97) done.
    private final Pose toArtifactLine2 = new Pose(100, 58, Math.toRadians(0));//y was 60 done
    private final Pose driveThroughLine2 = new Pose(142, 58, Math.toRadians(0));//x was 5 done

    private final Pose line2ToShoot3Mid = new Pose(130, 40, Math.toRadians(53)); //done

    private final Pose driveToShoot3= new Pose(107, 97, Math.toRadians(40.5)); // was 93,97 done
    private final Pose driveTowardsGate1= new Pose(117, 55, Math.toRadians(25)); // was 95 done

    private final Pose driveToGate1= new Pose(138, 58, Math.toRadians(25)); // y was 62

    private final Pose intakeFromGate1 = new Pose(143, 51, Math.toRadians(75)); //y was 54, angle was 105 done

    private final Pose driveToShoot4= new Pose(105, 97, Math.toRadians(41)); // was 93,95 done

    private final Pose driveTowardsGate2= new Pose(110, 55, Math.toRadians(25)); // was 95 done

    private final Pose driveToGate2= new Pose(138, 58, Math.toRadians(25)); // was 2,71 done

    private final Pose intakeFromGate2 = new Pose(143, 54, Math.toRadians(75)); //angle was 105 done

    // (kept but unused in this flow)
    private final Pose driveToShoot5 = new Pose(105, 97, Math.toRadians(41)); // was 93,93

    private final Pose leavePose = new Pose(115, 66, Math.toRadians(0));

    private Path scorePreload;
    private PathChain driveToLine1, pickUpLine1,  line1CurveToGate, goShoot2, driveToLine2, pickUpLine2, gate1ToIntake,
            gate1IntakeToShoot4, gate2ToIntake , goShoot3,gate2IntakeToShoot5 , leaveOutChain, goGate1Combined, goGate2Combined;



    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score -> Line1 entry
        driveToLine1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toArtifactLine1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toArtifactLine1.getHeading())
                .build();

        // Line1 drive-through (pick up)
        pickUpLine1 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine1, driveThroughLine1))
                .setLinearHeadingInterpolation(toArtifactLine1.getHeading(), driveThroughLine1.getHeading())
                .build();

        // After driveThroughLine1, curve: (133,81)->(130,70)->(133,77)
        line1CurveToGate = follower.pathBuilder()
                .addPath(new BezierCurve(driveThroughLine1, line1TowardsGate, line1PressGate))
                .setLinearHeadingInterpolation(driveThroughLine1.getHeading(),line1TowardsGate.getHeading(), line1PressGate.getHeading())
                .build();

        // After Line1 exit -> Shoot2
        goShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(line1PressGate ,driveToShoot2))
                .setLinearHeadingInterpolation(line1PressGate.getHeading(), driveToShoot2.getHeading())
                .build();

        // After Shoot2 -> Line2 entry
        driveToLine2 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot2, toArtifactLine2))
                .setLinearHeadingInterpolation(driveToShoot2.getHeading(), toArtifactLine2.getHeading())
                .build();

        // Line2 drive-through
        pickUpLine2 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine2, driveThroughLine2))
                .setLinearHeadingInterpolation(toArtifactLine2.getHeading(), driveThroughLine2.getHeading())
                .build();

        // UPDATED: pickUpLine2 end -> goShoot3 is now a BezierCurve with midpoint (135,58,0)
        goShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(driveThroughLine2, line2ToShoot3Mid, driveToShoot3))
                .setLinearHeadingInterpolation(driveThroughLine2.getHeading(), line2ToShoot3Mid.getHeading(), driveToShoot3.getHeading())
                .build();

        // gate2 combined (unchanged)

        goGate1Combined = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot3, driveTowardsGate1))
                .setLinearHeadingInterpolation(driveToShoot3.getHeading(), driveTowardsGate1.getHeading())
                .addPath(new BezierLine(driveTowardsGate1, driveToGate1))
                .setLinearHeadingInterpolation(driveTowardsGate1.getHeading(), driveToGate1.getHeading())
                .build();

        // gate 2 uses intakeFromGate2
        gate1ToIntake = follower.pathBuilder()
                .addPath(new BezierLine(driveToGate1, intakeFromGate1))
                .setLinearHeadingInterpolation(driveToGate1.getHeading(), intakeFromGate1.getHeading())
                .build();

        gate1IntakeToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(intakeFromGate1, driveToShoot4))
                .setLinearHeadingInterpolation(intakeFromGate1.getHeading(), driveToShoot4.getHeading())
                .build();

        goGate2Combined = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot4, driveTowardsGate2))
                .setLinearHeadingInterpolation(driveToShoot4.getHeading(), driveTowardsGate2.getHeading())
                .addPath(new BezierLine(driveTowardsGate2, driveToGate2))
                .setLinearHeadingInterpolation(driveTowardsGate2.getHeading(), driveToGate2.getHeading())
                .build();

        // gate 2 uses intakeFromGate2
        gate2ToIntake = follower.pathBuilder()
                .addPath(new BezierLine(driveToGate2, intakeFromGate2))
                .setLinearHeadingInterpolation(driveToGate2.getHeading(), intakeFromGate2.getHeading())
                .build();

        gate2IntakeToShoot5 = follower.pathBuilder()
                .addPath(new BezierLine(intakeFromGate2, driveToShoot5))
                .setLinearHeadingInterpolation(intakeFromGate2.getHeading(), driveToShoot5.getHeading())
                .build();


        // leave after Shoot4
        leaveOutChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot5, leavePose))
                .setLinearHeadingInterpolation(driveToShoot5.getHeading(), leavePose.getHeading())
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
            if (aimOk || timedOut) {
                shootingSequenceActive = true;
                shootingSequenceStart = now;

                // kept (no longer used for pulsing)
                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;

            } else {
                intakeStop();
                return false;
            }
        }

        // CONTINUOUS FEED during shooting window (NO PULSE)
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

                // kept (no longer used for pulsing)
                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;

            } else {
                intakeStop();
                return false;
            }
        }

        // CONTINUOUS FEED during shooting window (NO PULSE)
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
                        turret.saveFirstShotPosition();
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

            // After driveThroughLine1, run the exit curve
            case 3:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(line1CurveToGate, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    // requirement: when it gets to the last point, shut intake off, then goShoot2
                    intakeStop();
                    follower.followPath(goShoot2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeSlow();
                        follower.followPath(driveToLine2, true);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intakeSlow();
                    follower.followPath(pickUpLine2, true);
                    setPathState(7);
                }
                break;

            // pickUpLine2 end -> goShoot3 (now BezierCurve)
            case 7:
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.followPath(goShoot3, true);
                    setPathState(8);
                }
                break;

            // Shoot3 -> Gate2
            case 8:
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeStop();
                        follower.followPath(goGate1Combined, true);
                        setPathState(9);
                    }
                }
                break;

            // ---- Gate 2 behavior ----
            case 9:
                if (!follower.isBusy()) {
                    intakeSlow();
                    if (pauseTime(0.4)) { //was 0.5
                        follower.followPath(gate1ToIntake, true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    intakeSlow();
                    if (pauseTime(0.3)) {
                        intakeSlow();
                        follower.followPath(gate1IntakeToShoot4, true);
                        setPathState(11);
                    }
                }
                break;

            // Shoot4 -> Gate2 again
            case 11:
                if (!follower.isBusy()) {
                    if (runShootWindowQuick()) {
                        intakeSlow();
                        follower.followPath(goGate2Combined, true);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intakeSlow();
                    if (pauseTime(0.4)) {
                        follower.followPath(gate2ToIntake, true);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    intakeSlow();
                    if (pauseTime(0.30)) {
                        intakeSlow();
                        follower.followPath(gate2IntakeToShoot5, true);
                        setPathState(14);
                    }
                }
                break;

            // Shoot5 -> Leave
            case 14:
                if (!follower.isBusy()) {
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

            default:
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

        // Shooter always on (MATCH RCRed: direct setVelocity in ticks/sec; no target*comp)
        shooter.setVelocity(target);

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
        telemetry.addData("omegaDps", "%.2f", omegaDps);
        telemetry.addData("moveBlend", "%.2f", moveBlend);
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

        lt = hardwareMap.get(Servo.class, "lt");
        rt = hardwareMap.get(Servo.class, "rt");
        ki = hardwareMap.get(Servo.class, "ki");

        rt.setPosition(1.0); //was 0.9
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

        setPathState(0);
    }

    @Override
    public void stop() {
        // no-op
    }

    // ================= INTAKE =================
    private void intakeSlow() {
        fi.setPower(1.0);
        bi.setPower(-0.65);
    }

    /** CONTINUOUS feed (NO PULSE): when shooting, both motors at 1.0 power. */
    private void intakeShootFeed() {
        fi.setPower(1.0);
        bi.setPower(1.0);
    }

    private void intakeStop() {
        fi.setPower(0.0);
        bi.setPower(0.0);
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

        // Last seen turret position for simple reacquire behavior
        private int lastSeenTurretTicks = 0;
        private boolean hasLastSeenTarget = false;
        private double lastSeenTime = 0.0;

        // Saved first-shot turret position
        private int firstShotTurretTicks = 0;
        private boolean hasFirstShotTurretTicks = false;

        private static final int TX_SIGN = 1;
        private static final int LEAD_SIGN = 1;

        private static final double KP_MOVING = 0.055;
        private static final double KP_STOPPED = 0.032;

        private static final double KD_MOVING = 0.004;
        private static final double KD_STOPPED = 0.0015;

        private static final double FF_MOVING = 0.012;
        private static final double FF_STOPPED = 0.0;

        private static final double LEAD_TIME = 0.06; // rl was 0.03

        private static final double STOPPED_MOVE_BLEND = 0.08;
        private static final double STOPPED_OMEGA_DPS = 8.0;

        private static final double CENTER_DEADBAND_TX = 0.75;
        private static final double D_ENABLE_TX = 2.0;

        private static final double MAX_PWR_MOVING = 0.75;
        private static final double MAX_PWR_STOPPED = 0.35;

        private static final double AIM_TOL = 2.5;
        private static final double AIM_PWR_TOL = 0.12;
        private static final double AIM_STABLE = 0.12;

        // Simple encoder return when tag is lost
        private static final double LOST_TICKS_KP = 0.003;
        private static final double LOST_MAX_POWER = 0.35;

        public void init(HardwareMap hw) {
            turret = hw.get(DcMotorEx.class, "turretSpin");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setDirection(DcMotorSimple.Direction.REVERSE);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            lastSeenTurretTicks = turret.getCurrentPosition();
            hasLastSeenTarget = false;
            lastSeenTime = 0.0;

            firstShotTurretTicks = 0;
            hasFirstShotTurretTicks = false;
        }

        public void saveFirstShotPosition() {
            firstShotTurretTicks = turret.getCurrentPosition() + TURRET_OFFSET_TICKS;
            hasFirstShotTurretTicks = true;

            turret.setTargetPosition(firstShotTurretTicks);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(1.0);
            lastPower = 1.0;
        }

        public void update(double now, double dt, double moveBlend, double omegaDps) {

            if (dt < 0.001) dt = 0.001;

            // Once first shot is saved, stop all tracking and hard-hold this encoder position
            if (hasFirstShotTurretTicks) {
                turret.setTargetPosition(firstShotTurretTicks);
                if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                turret.setPower(1.0);
                lastPower = 1.0;
                return;
            }

            LLResult r = limelight.getLatestResult();
            if (r == null || !r.isValid()) {
                aimHold = false;

                int targetTicks;
                boolean haveFallback = false;

                if (hasLastSeenTarget && (now - lastSeenTime) < 0.25) {
                    targetTicks = lastSeenTurretTicks;
                    haveFallback = true;
                } else if (hasFirstShotTurretTicks) {
                    targetTicks = firstShotTurretTicks;
                    haveFallback = true;
                } else if (hasLastSeenTarget) {
                    targetTicks = lastSeenTurretTicks;
                    haveFallback = true;
                } else {
                    targetTicks = 0;
                }

                if (haveFallback) {
                    int tickError = targetTicks - turret.getCurrentPosition();
                    double power = clip(tickError * LOST_TICKS_KP, -LOST_MAX_POWER, LOST_MAX_POWER);

                    turret.setPower(power);
                    lastPower = power;
                } else {
                    turret.setPower(0);
                    lastPower = 0;
                }
                return;
            }

            // Save where the turret was when the target was last seen
            lastSeenTurretTicks = turret.getCurrentPosition();
            hasLastSeenTarget = true;
            lastSeenTime = now;

            double txRaw = r.getTx();
            lastTx = txRaw;

            boolean robotStopped = (moveBlend < STOPPED_MOVE_BLEND) && (Math.abs(omegaDps) < STOPPED_OMEGA_DPS);

            double txPred = (TX_SIGN * txRaw) + (LEAD_SIGN * omegaDps * LEAD_TIME);

            // More smoothing when stopped, less when moving
            double alpha = robotStopped ? 0.72 : 0.28;
            txFiltered = alpha * txFiltered + (1.0 - alpha) * txPred;

            double error = txFiltered;

            // Small deadband near center to stop chasing tiny Limelight noise
            if (Math.abs(error) < CENTER_DEADBAND_TX) {
                error = 0.0;
            }

            double errAbs = Math.abs(error);

            double dErr = (error - lastErr) / dt;
            lastErr = error;

            double kp = robotStopped ? KP_STOPPED : KP_MOVING;
            double kd = robotStopped ? KD_STOPPED : KD_MOVING;
            double ffBase = robotStopped ? FF_STOPPED : FF_MOVING;
            double maxPower = robotStopped ? MAX_PWR_STOPPED : MAX_PWR_MOVING;

            double p = kp * error;

            // Disable D near center so noise doesn't cause wagging
            double d = 0.0;
            if (errAbs > D_ENABLE_TX) {
                d = kd * dErr;
            }

            double ff = 0.0;
            if (!robotStopped) {
                double ffScale = clip((errAbs - 1.5) / (8.0 - 1.5), 0.0, 1.0);
                ff = ffBase * omegaDps * (1.0 + moveBlend) * ffScale;
            }

            double power = p + d + ff;

            // Stronger softening near center when stopped
            double brake;
            if (robotStopped) {
                brake = clip(errAbs / 4.0, 0.15, 1.0);
            } else {
                brake = clip(errAbs / 5.5, 0.25, 1.0);
            }
            power *= brake;

            power = clip(power, -maxPower, maxPower);

            turret.setPower(power);
            lastPower = power;

            if (errAbs < AIM_TOL && Math.abs(power) < AIM_PWR_TOL && Math.abs(omegaDps) < STOPPED_OMEGA_DPS) {
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