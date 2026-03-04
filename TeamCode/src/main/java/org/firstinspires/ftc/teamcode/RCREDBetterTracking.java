package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="RC RED Better Tracking (Pinpoint)", group="Robot")
public class RCREDBetterTracking extends OpMode {

    // ----------------- DRIVE -----------------
    private DcMotor lf, lr, rf, rr, fi, bi;
    private Servo lt, rt, ki;

    // ----------------- TURRET + SHOOTER -----------------
    private DcMotorEx turretSpin, shooter;

    // ----------------- PEDRO / PINPOINT -----------------
    private Follower follower;

    // =========================================================
    // FIELD GOAL (CLOSE AIM POINT)  <<< SET THESE >>>
    // =========================================================
    private static final double GOAL_X_IN = 144; // TODO: set
    private static final double GOAL_Y_IN = 144; // TODO: set

    // =========================================================
    // NEW: AIM DEBUG / MATH FIX TUNES
    // =========================================================
    // Set so that: when you manually rotate turret RIGHT, TurretDeg(enc) INCREASES.
    // If it DECREASES, flip this between +1 and -1.
    private static final int TURRET_ENC_SIGN = -1; // TODO: +1 or -1 after test

    // Turret "zero" offset so that when turret is physically forward, turretDeg ~= 0.
    // Set TURRET_ZERO_OFFSET_DEG = -telemetryTurretDegWhenForward
    private static final double TURRET_ZERO_OFFSET_DEG = 0.0; // TODO: tune

    // If your Pedro heading sign is opposite (CCW rotation makes heading go DOWN),
    // set this true to use desiredGlobal + heading instead of desiredGlobal - heading.
    private static final boolean HEADING_SIGN_FLIPPED = false; // TODO: set after test

    // =========================================================
    // TUNING (PULLED FROM YOUR KNOWN-GOOD RCRedV7)
    // =========================================================

    private static final double TX_DEADBAND_DEG = 1.5;

    private static final double TURRET_KP_STRAIGHT = 0.050;
    private static final double TURRET_KP_TURNING  = 0.025;

    private static final double TURRET_KD_STRAIGHT = 0.003;
    private static final double TURRET_KD_TURNING  = 0.0030;

    private static final double TURNING_DPS_START = 25.0;
    private static final double TURNING_DPS_FULL  = 140.0;

    private static final double MAX_TURRET_POWER_SLOW = 0.4;
    private static final double TURRET_FILTER_ALPHA_SLOW = 0.50;
    private static final double TURRET_SLEW_PER_LOOP_SLOW = 0.12;

    private static final double MAX_TURRET_POWER_FAST = 0.85;
    private static final double TURRET_FILTER_ALPHA_FAST = 0.30;
    private static final double TURRET_SLEW_PER_LOOP_FAST = 0.45;

    private static final double MIN_TURRET_POWER = 0.1;
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

    private static final double MANUAL_TURRET_POWER = 0.80;

    private static final boolean ENABLE_SOFT_LIMITS = false;
    private static final int TURRET_MIN_TICKS = -2000;
    private static final int TURRET_MAX_TICKS =  2000;

    private static final double RX_TURN_BLEND_START = 0.10;
    private static final double RX_TURN_BLEND_FULL  = 0.60;

    private static final double SHOOTER_VEL_KP = 24.0;
    private static final double SHOOTER_VEL_KI = 0.0;
    private static final double SHOOTER_VEL_KD = 0.001;
    private static final double SHOOTER_VEL_KF = 12.5;

    private static final double SHOOT_READY_TOL = 40.0;
    private static final double VEL_FILTER_ALPHA = 0.75;

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

    private static final double FEED_ON_SEC  = 0.20;
    private static final double FEED_OFF_SEC = 0.20;

    private static final double RB_HOLD_PULSE_ON_SEC  = 0.20;
    private static final double RB_HOLD_PULSE_OFF_SEC = 0.30;
    private static final double RB_HOLD_PULSE_POWER   = 1.00;

    private static final double REVERSE_POWER = 0.70;
    private static final double TRIGGER_DEADBAND = 0.10;

    private static final double FAR_TURRET_TX_OFFSET_DEG = 3.0;

    private static final double RAMP_SEC_CLOSE = 0.40;
    private static final double RAMP_SEC_FAR   = 0.55;

    private static final double CLOSE_AIM_OK_DEG = 2.0;
    private static final double CLOSE_STABLE_HOLD_SEC = 0.08;

    // =========================================================
    // TURRET ENCODER SCALE (your saved reference)
    // =========================================================
    private static final double TURRET_COUNTS_PER_DEG = 8.713;

    // =========================================================
    // STATE
    // =========================================================
    private boolean trackingEnabled = true;
    private boolean xWasPressed = false;

    private double turretFilteredPower = 0.0;

    private double lastErrDeg = 0.0;
    private double lastLoopTime = 0.0;

    private double errFiltered = 0.0;

    private double yawRateFiltered = 0.0;

    private double shooterSetpoint = 0.0;
    private boolean a2WasPressed = false;
    private boolean y2WasPressed = false;
    private boolean dpad_downWasPressed = false;

    private double shooterVelFilt = 0.0;

    private boolean wasMoving = false;
    private double settleUntilTime = -999.0;

    private boolean wasTurning = false;
    private double turnStopUntilTime = -999.0;

    private enum IntakeMode {
        OFF,
        INTAKE_SLOW,
        SHOOTER_FULL,
        SHOOTER_PULSE_HOLD
    }

    private IntakeMode intakeMode = IntakeMode.OFF;

    private boolean lb2WasPressed = false;
    private boolean rb2WasPressed = false;

    private IntakeMode preRbHoldMode = IntakeMode.OFF;

    private boolean feedPulseOn = false;
    private double feedNextToggleTime = 0.0;
    private IntakeMode lastIntakeMode = IntakeMode.OFF;

    private boolean rbHoldPulseOn = false;
    private double rbHoldNextToggleTime = 0.0;

    private enum ShotRangeMode { CLOSE, FAR }
    private ShotRangeMode shotRangeMode = ShotRangeMode.CLOSE;
    private ShotRangeMode lastShotRangeMode = ShotRangeMode.CLOSE;

    private SpinUpRampVelocity spinup;
    private boolean shooterWasOnDiag = false;
    private double lastSpinupSetpointDiag = 0.0;

    private double closeStableUntil = -999.0;

    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");
        fi = hardwareMap.dcMotor.get("fi");
        bi = hardwareMap.dcMotor.get("bi");

        lt = hardwareMap.get(Servo.class, "lt");
        rt = hardwareMap.get(Servo.class, "rt");
        ki = hardwareMap.get(Servo.class, "ki");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        turretSpin.setDirection(DcMotorSimple.Direction.REVERSE);
        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_VEL_KP, SHOOTER_VEL_KI, SHOOTER_VEL_KD, SHOOTER_VEL_KF);
        } catch (Exception ignored) {}

        spinup = new SpinUpRampVelocity(shooter);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        lastLoopTime = getRuntime();

        lt.setPosition(0.1);
        rt.setPosition(1);
        ki.setPosition(0.2);

        fi.setPower(0);
        bi.setPower(0);

        shooterVelFilt = 0.0;

        wasMoving = false;
        settleUntilTime = -999.0;

        wasTurning = false;
        turnStopUntilTime = -999.0;

        intakeMode = IntakeMode.OFF;
        lastIntakeMode = IntakeMode.OFF;

        feedPulseOn = false;
        feedNextToggleTime = 0.0;

        rbHoldPulseOn = false;
        rbHoldNextToggleTime = 0.0;
        preRbHoldMode = IntakeMode.OFF;

        shotRangeMode = ShotRangeMode.CLOSE;
        lastShotRangeMode = shotRangeMode;

        closeStableUntil = -999.0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {

        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        follower.update();
        Pose pose = follower.getPose();

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double moveMag = Math.abs(y) + Math.abs(x) + Math.abs(rx);
        boolean movingNow = moveMag > MOVE_EPS;

        if (wasMoving && !movingNow) {
            settleUntilTime = now + SETTLE_HOLD_SEC;
        }
        wasMoving = movingNow;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denom;
        double bl = (y - x + rx) / denom;
        double fr = (y - x - rx) / denom;
        double br = (y + x - rx) / denom;

        lf.setPower(fl);
        lr.setPower(bl);
        rf.setPower(fr);
        rr.setPower(br);

        boolean lb2Now = gamepad2.left_bumper;
        boolean rb2Now = gamepad2.right_bumper;
        boolean b2Now  = gamepad2.b;

        boolean lb2Pressed = lb2Now && !lb2WasPressed;
        lb2WasPressed = lb2Now;

        boolean rb2JustPressed  = rb2Now && !rb2WasPressed;
        boolean rb2JustReleased = !rb2Now && rb2WasPressed;
        rb2WasPressed = rb2Now;

        if (b2Now) {
            intakeMode = IntakeMode.OFF;
        } else if (rb2Now) {
            if (rb2JustPressed) {
                preRbHoldMode = intakeMode;
                rbHoldPulseOn = true;
                rbHoldNextToggleTime = now + RB_HOLD_PULSE_ON_SEC;
            }
            intakeMode = IntakeMode.SHOOTER_PULSE_HOLD;

        } else {
            if (rb2JustReleased) {
                intakeMode = preRbHoldMode;
                rbHoldPulseOn = false;
                rbHoldNextToggleTime = 0.0;
            }

            if (lb2Pressed) {
                intakeMode = (intakeMode == IntakeMode.INTAKE_SLOW) ? IntakeMode.OFF : IntakeMode.INTAKE_SLOW;
            }
        }

        boolean a2Now = gamepad2.a;
        boolean y2Now = gamepad2.y;
        boolean dDownNow = gamepad1.x;

        boolean a2Pressed = a2Now && !a2WasPressed;
        boolean y2Pressed = y2Now && !y2WasPressed;
        boolean ddownPressed = dDownNow && !dpad_downWasPressed;

        if (a2Pressed) {
            shotRangeMode = ShotRangeMode.CLOSE;
            rt.setPosition(1.0);
            shooterSetpoint = 1260;
            intakeMode = IntakeMode.SHOOTER_FULL;
        }
        if (y2Pressed) {
            shotRangeMode = ShotRangeMode.CLOSE;
            shooterSetpoint = 1150;
            rt.setPosition(0.7);
            intakeMode = IntakeMode.SHOOTER_FULL;
        }
        if (ddownPressed) {
            shotRangeMode = ShotRangeMode.FAR;
            shooterSetpoint = 1630;
            rt.setPosition(1.0);
            intakeMode = IntakeMode.SHOOTER_FULL;
        }

        a2WasPressed = a2Now;
        y2WasPressed = y2Now;
        dpad_downWasPressed = dDownNow;

        boolean shooterOn = (shooterSetpoint > 0.0);

        if (shooterOn) {

            spinup.rampSec = (shotRangeMode == ShotRangeMode.FAR) ? RAMP_SEC_FAR : RAMP_SEC_CLOSE;

            if (!shooterWasOnDiag || shooterSetpoint != lastSpinupSetpointDiag) {
                spinup.start(shooterSetpoint, now);
                lastSpinupSetpointDiag = shooterSetpoint;
            }

            spinup.update(now);

            shooterWasOnDiag = true;

        } else {
            spinup.stop();
            shooter.setVelocity(0.0);

            shooterWasOnDiag = false;
            lastSpinupSetpointDiag = 0.0;
        }

        double velRaw = shooter.getVelocity();
        shooterVelFilt = (VEL_FILTER_ALPHA * shooterVelFilt) + ((1.0 - VEL_FILTER_ALPHA) * velRaw);

        double rt2 = gamepad2.right_trigger;
        boolean reverseHeld = rt2 > TRIGGER_DEADBAND;

        boolean xNow = gamepad2.x;
        if (xNow && !xWasPressed) {
            trackingEnabled = !trackingEnabled;

            lastErrDeg = 0.0;
            turretFilteredPower = 0.0;
            errFiltered = 0.0;
            yawRateFiltered = 0.0;

            settleUntilTime = -999.0;
            wasMoving = false;

            turnStopUntilTime = -999.0;
            wasTurning = false;

            closeStableUntil = -999.0;
        }
        xWasPressed = xNow;

        boolean manualLeft  = gamepad2.dpad_left;
        boolean manualRight = gamepad2.dpad_right;

        double desiredTurretPower = 0.0;

        if (manualLeft)  desiredTurretPower = -MANUAL_TURRET_POWER;
        if (manualRight) desiredTurretPower =  MANUAL_TURRET_POWER;

        double yawRateRaw = estimateYawRateDegPerSec(dt, pose.getHeading());
        yawRateFiltered =
                (YAW_RATE_FILTER_ALPHA * yawRateFiltered) +
                        ((1.0 - YAW_RATE_FILTER_ALPHA) * yawRateRaw);

        double yawRateUse = applyDeadband(yawRateFiltered, YAW_RATE_DEADBAND_DPS);
        yawRateUse = clip(yawRateUse, -YAW_RATE_CLAMP_DPS, YAW_RATE_CLAMP_DPS);

        boolean turningNow = (Math.abs(rx) > TURNING_RX_EPS) || (Math.abs(yawRateFiltered) > TURNING_YAW_DPS);
        if (wasTurning && !turningNow) {
            turnStopUntilTime = now + TURN_STOP_HOLD_SEC;
        }
        wasTurning = turningNow;

        double rxAbs = Math.abs(rx);
        double turnBlendRx = (rxAbs - RX_TURN_BLEND_START) / (RX_TURN_BLEND_FULL - RX_TURN_BLEND_START);
        turnBlendRx = clip(turnBlendRx, 0.0, 1.0);

        double turnBlendYaw = (Math.abs(yawRateFiltered) - TURNING_DPS_START) / (TURNING_DPS_FULL - TURNING_DPS_START);
        turnBlendYaw = clip(turnBlendYaw, 0.0, 1.0);

        double turnBlend = Math.max(turnBlendRx, turnBlendYaw);

        double maxTurretPower = lerp(MAX_TURRET_POWER_SLOW, MAX_TURRET_POWER_FAST, turnBlend);
        double filterAlpha    = lerp(TURRET_FILTER_ALPHA_SLOW, TURRET_FILTER_ALPHA_FAST, turnBlend);
        double slewPerLoop    = lerp(TURRET_SLEW_PER_LOOP_SLOW, TURRET_SLEW_PER_LOOP_FAST, turnBlend);

        double errAlpha = lerp(TX_FILTER_ALPHA_STRAIGHT, TX_FILTER_ALPHA_TURNING, turnBlend);
        double kpUse   = lerp(TURRET_KP_STRAIGHT, TURRET_KP_TURNING, turnBlend);
        double kdUse   = lerp(TURRET_KD_STRAIGHT, TURRET_KD_TURNING, turnBlend);

        double ffGain = lerp(TURN_RATE_FF_SLOW, TURN_RATE_FF_FAST, turnBlend);

        boolean settleWindow = (now <= settleUntilTime);

        boolean settleEligible =
                settleWindow &&
                        (Math.abs(yawRateFiltered) <= SETTLE_YAW_DPS) &&
                        (Math.abs(rx) <= SETTLE_RX_EPS);

        boolean stationaryEligible =
                (Math.abs(yawRateFiltered) <= STATIONARY_YAW_DPS) &&
                        (Math.abs(rx) <= STATIONARY_RX_EPS);

        boolean turnStopEligible = (now <= turnStopUntilTime);

        double errDeadbandUse = TX_DEADBAND_DEG;
        double minPowerEnableErrUse = MIN_POWER_ENABLE_ERROR_DEG;
        boolean killFFThisLoop = false;

        if (turnStopEligible) {
            maxTurretPower = Math.min(maxTurretPower, TURNSTOP_MAX_TURRET_POWER);
            kpUse *= TURNSTOP_KP_MULT;
            kdUse *= TURNSTOP_KD_MULT;
            killFFThisLoop = true;
        }
        else if (settleEligible) {
            maxTurretPower = Math.min(maxTurretPower, SETTLE_MAX_TURRET_POWER);
            kpUse *= SETTLE_KP_MULT;
            kdUse *= SETTLE_KD_MULT;

            errDeadbandUse = SETTLE_TX_DEADBAND;
            minPowerEnableErrUse = SETTLE_MINPOWER_ENABLE_ERR;
        }
        else if (stationaryEligible) {
            maxTurretPower = Math.min(maxTurretPower, STATIONARY_MAX_TURRET_POWER);
            kpUse *= STATIONARY_KP_MULT;
            kdUse *= STATIONARY_KD_MULT;

            errDeadbandUse = STATIONARY_TX_DEADBAND;
            minPowerEnableErrUse = STATIONARY_MINPOWER_ENABLE_ERR;
        }

        double errDeg = 0.0;
        double dErr = 0.0;
        double autoPower = 0.0;
        double ffScale = 0.0;
        double yawFF = 0.0;

        // NEW debug vars
        double dxDbg = 0.0, dyDbg = 0.0;
        double desiredGlobalDegDbg = 0.0;
        double robotHeadingDegDbg = Math.toDegrees(pose.getHeading());
        double desiredTurretDegDbg = 0.0;
        double turretDegDbg = 0.0;

        if (!manualLeft && !manualRight && trackingEnabled) {

            double dx = GOAL_X_IN - pose.getX();
            double dy = GOAL_Y_IN - pose.getY();
            double desiredGlobal = Math.atan2(dy, dx);

            // HEADING SIGN FIX OPTION
            double desiredTurretRad = HEADING_SIGN_FLIPPED
                    ? wrapRad(desiredGlobal + pose.getHeading())
                    : wrapRad(desiredGlobal - pose.getHeading());

            // Turret angle from encoder with SIGN + ZERO OFFSET
            double turretDeg = (TURRET_ENC_SIGN * ticksToDeg(turretSpin.getCurrentPosition())) + TURRET_ZERO_OFFSET_DEG;
            double desiredTurretDeg = Math.toDegrees(desiredTurretRad);


            double offsetDeg = (shotRangeMode == ShotRangeMode.FAR) ? FAR_TURRET_TX_OFFSET_DEG : 0.0;

            errDeg = wrapDeg((desiredTurretDeg + offsetDeg) - turretDeg);

            errFiltered = (errAlpha * errFiltered) + ((1.0 - errAlpha) * errDeg);

            double errDb = applyDeadband(errFiltered, errDeadbandUse);

            dErr = (errDb - lastErrDeg) / dt;
            lastErrDeg = errDb;

            double pd = (kpUse * errDb) + (kdUse * dErr);

            double absErr = Math.abs(errDb);
            double ffFromErr = 0.0;
            if (absErr >= IMU_FF_ENABLE_ERROR_DEG) {
                ffFromErr = (absErr - IMU_FF_ENABLE_ERROR_DEG) /
                        (IMU_FF_FULL_ERROR_DEG - IMU_FF_ENABLE_ERROR_DEG);
                ffFromErr = clip(ffFromErr, 0.0, 1.0);
            }
            double ffFromTurn = 0.55 * turnBlend;
            ffScale = Math.max(ffFromErr, ffFromTurn);

            if (!killFFThisLoop && Math.abs(yawRateUse) >= FF_ENABLE_TURN_DPS) {
                yawFF = (+ffGain * yawRateUse) * ffScale;
            } else {
                yawFF = 0.0;
            }

            autoPower = pd + yawFF;

            if (Math.abs(errDb) >= minPowerEnableErrUse) {
                autoPower = addMinPower(autoPower, MIN_TURRET_POWER);
            }

            desiredTurretPower = clip(autoPower, -maxTurretPower, maxTurretPower);

            // NEW: debug capture
            dxDbg = dx;
            dyDbg = dy;
            desiredGlobalDegDbg = Math.toDegrees(desiredGlobal);
            desiredTurretDegDbg = desiredTurretDeg;
            turretDegDbg = turretDeg;

        } else {
            lastErrDeg = 0.0;
        }

        double filteredTarget =
                filterAlpha * turretFilteredPower +
                        (1.0 - filterAlpha) * desiredTurretPower;

        turretFilteredPower = slewTo(turretFilteredPower, filteredTarget, slewPerLoop);

        int turretTicks = turretSpin.getCurrentPosition();

        if (ENABLE_SOFT_LIMITS) {
            if (turretTicks >= TURRET_MAX_TICKS && turretFilteredPower > 0) turretFilteredPower = 0;
            if (turretTicks <= TURRET_MIN_TICKS && turretFilteredPower < 0) turretFilteredPower = 0;
        }

        turretSpin.setPower(turretFilteredPower);

        boolean flywheelReady = shooterOn && (Math.abs(shooterVelFilt - shooterSetpoint) <= SHOOT_READY_TOL);
        boolean aimReadyClose = (Math.abs(errFiltered) <= CLOSE_AIM_OK_DEG);

        if (shotRangeMode == ShotRangeMode.CLOSE && shooterOn && intakeMode == IntakeMode.SHOOTER_FULL) {
            if (flywheelReady && aimReadyClose) {
                closeStableUntil = now + CLOSE_STABLE_HOLD_SEC;
            }
        } else {
            closeStableUntil = -999.0;
        }

        boolean closeGateOpen = (now <= closeStableUntil);

        if (lastIntakeMode != intakeMode) {
            if (intakeMode != IntakeMode.SHOOTER_FULL) {
                feedPulseOn = false;
                feedNextToggleTime = 0.0;
            } else {
                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;
            }
            lastIntakeMode = intakeMode;
        }

        if (lastShotRangeMode != shotRangeMode) {
            if (shotRangeMode == ShotRangeMode.FAR) {
                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;
            }
            lastShotRangeMode = shotRangeMode;
        }

        if (reverseHeld) {
            fi.setPower(-REVERSE_POWER);
            bi.setPower(-REVERSE_POWER);

        } else if (intakeMode == IntakeMode.SHOOTER_PULSE_HOLD) {

            if (rbHoldNextToggleTime <= 0.0) {
                rbHoldPulseOn = true;
                rbHoldNextToggleTime = now + RB_HOLD_PULSE_ON_SEC;
            } else if (now >= rbHoldNextToggleTime) {
                rbHoldPulseOn = !rbHoldPulseOn;
                rbHoldNextToggleTime = now + (rbHoldPulseOn ? RB_HOLD_PULSE_ON_SEC : RB_HOLD_PULSE_OFF_SEC);
            }

            if (rbHoldPulseOn) {
                fi.setPower(RB_HOLD_PULSE_POWER);
                bi.setPower(RB_HOLD_PULSE_POWER);
            } else {
                fi.setPower(0.0);
                bi.setPower(0.0);
            }

        } else if (intakeMode == IntakeMode.INTAKE_SLOW) {

            fi.setPower(1.0);
            bi.setPower(-0.65);

        } else if (intakeMode == IntakeMode.SHOOTER_FULL) {

            if (!shooterOn) {
                fi.setPower(1.0);
                bi.setPower(1.0);

                feedPulseOn = true;
                feedNextToggleTime = now + FEED_ON_SEC;

            } else {

                if (shotRangeMode == ShotRangeMode.FAR) {
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

                } else {
                    if (closeGateOpen) {
                        fi.setPower(1.0);
                        bi.setPower(1.0);
                    } else {
                        fi.setPower(0.0);
                        bi.setPower(0.0);
                    }

                    feedPulseOn = true;
                    feedNextToggleTime = now + FEED_ON_SEC;
                }
            }

        } else {
            fi.setPower(0.0);
            bi.setPower(0.0);
        }

        // =========================
        // TELEMETRY
        // =========================
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1fdeg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Goal", "x=%.1f y=%.1f", GOAL_X_IN, GOAL_Y_IN);

        // NEW: truth telemetry for field->turret math
        telemetry.addData("dx,dy", "%.1f, %.1f", dxDbg, dyDbg);
        telemetry.addData("GlobalToGoal(deg)", "%.1f", desiredGlobalDegDbg);
        telemetry.addData("RobotHeading(deg)", "%.1f", robotHeadingDegDbg);
        telemetry.addData("DesiredTurret(deg)", "%.1f", desiredTurretDegDbg);
        telemetry.addData("TurretDeg(enc)", "%.1f", turretDegDbg);
        telemetry.addData("AimCfg", "EncSign=%d ZeroOff=%.1f HeadingFlip=%b",
                TURRET_ENC_SIGN, TURRET_ZERO_OFFSET_DEG, HEADING_SIGN_FLIPPED);

        telemetry.addData("RangeMode", shotRangeMode.toString());
        telemetry.addData("TrackingEnabled", trackingEnabled);

        telemetry.addData("ErrDeg raw/filt", "%.2f / %.2f", errDeg, errFiltered);
        telemetry.addData("YawRate raw/filt/use (dps)", "%.2f / %.2f / %.2f", yawRateRaw, yawRateFiltered, yawRateUse);

        telemetry.addData("TurretTicks", turretSpin.getCurrentPosition());
        telemetry.addData("TurretPower", "%.3f", turretFilteredPower);

        telemetry.addData("Shooter set/act", "%.0f / %.0f", shooterSetpoint, velRaw);
        telemetry.addData("FlywheelReady", flywheelReady);
        telemetry.addData("AimReadyClose", aimReadyClose);
        telemetry.addData("CloseGateOpen", closeGateOpen);

        telemetry.addData("IntakeMode", intakeMode.toString());
        telemetry.addData("ReverseHeld(RT2)", reverseHeld);

        telemetry.update();
    }

    // =========================================================
    // Helpers
    // =========================================================

    private double lastHeadingRad = 0.0;
    private boolean headingInit = false;

    private double estimateYawRateDegPerSec(double dt, double headingRadNow) {
        if (!headingInit) {
            headingInit = true;
            lastHeadingRad = headingRadNow;
            return 0.0;
        }
        double d = wrapRad(headingRadNow - lastHeadingRad);
        lastHeadingRad = headingRadNow;
        return Math.toDegrees(d / dt);
    }

    private static double ticksToDeg(int ticks) {
        return ticks / TURRET_COUNTS_PER_DEG;
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

    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrapDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    // =========================================================
    // INNER CLASS: Velocity Ramp Spinup (same as your V7)
    // =========================================================
    public static class SpinUpRampVelocity {

        public double rampSec = 0.50;
        public double minRampSec = 0.12;
        public double maxRampSec = 1.50;

        private boolean active = false;
        private double startTime = 0.0;
        private double targetVel = 0.0;

        private double startVel = 0.0;
        private double cmdVel = 0.0;

        private String phase = "OFF";

        private final DcMotorEx shooter;

        public SpinUpRampVelocity(DcMotorEx shooterMotor) {
            this.shooter = shooterMotor;
        }

        public void start(double targetTicksPerSec, double nowSec) {
            this.targetVel = targetTicksPerSec;
            this.startTime = nowSec;
            this.active = true;
            this.phase = "RAMP";

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
                phase = "HOLD";
                active = false;
            }
        }

        public void stop() {
            active = false;
            phase = "OFF";
            cmdVel = 0.0;
        }

        public String getPhase() {
            return phase;
        }

        public double getCommandedVel() {
            return cmdVel;
        }
    }
}