package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

@TeleOp(name="RED v3", group="Robot")
public class RCREDv3 extends OpMode {

    // ----------------- DRIVE -----------------
    private DcMotor lf, lr, rf, rr, fi, bi;
    private Servo lt, rt, ki;

    // ----------------- TURRET + SHOOTER -----------------
    private DcMotorEx turretSpin, shooter;

    // ----------------- LIMELIGHT -----------------
    private Limelight3A limelight;

    // ----------------- IMU -----------------
    private IMU imu;

    // =========================================================
    // TUNING (CHANGE THESE ONLY)
    // =========================================================

    // DO NOT CHANGE (your config is correct)
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // -------- Vision tracking --------
    private static final double TX_DEADBAND_DEG = 1.5;

    // P term
    private static final double TURRET_KP_STRAIGHT = 0.050;
    private static final double TURRET_KP_TURNING  = 0.025;  // keep

    // D term
    private static final double TURRET_KD_STRAIGHT = 0.003;
    private static final double TURRET_KD_TURNING  = 0.0030; // keep

    // -------- Adaptive based on yaw rate --------
    private static final double TURNING_DPS_START = 25.0;
    private static final double TURNING_DPS_FULL  = 140.0;

    // Slow / straight
    private static final double MAX_TURRET_POWER_SLOW = 0.4;
    private static final double TURRET_FILTER_ALPHA_SLOW = 0.50;
    private static final double TURRET_SLEW_PER_LOOP_SLOW = 0.12;

    // Fast turn
    private static final double MAX_TURRET_POWER_FAST = 0.85; // keep from your current v3
    private static final double TURRET_FILTER_ALPHA_FAST = 0.30;
    private static final double TURRET_SLEW_PER_LOOP_FAST = 0.45;

    // -------- Minimum turret power (stiction) --------
    private static final double MIN_TURRET_POWER = 0.1;
    private static final double MIN_POWER_ENABLE_ERROR_DEG = 4.5;

    // -------- tx filtering --------
    private static final double TX_FILTER_ALPHA_STRAIGHT = 0.74;
    private static final double TX_FILTER_ALPHA_TURNING  = 0.15; // keep

    // -------- IMU feedforward (SPLIT SLOW/FAST) --------
    private static final double TURN_RATE_FF_SLOW = 0.0065; // keep
    private static final double TURN_RATE_FF_FAST = 0.0120; // keep

    private static final double YAW_RATE_DEADBAND_DPS = 3.0;
    private static final double YAW_RATE_CLAMP_DPS = 250.0;
    private static final double YAW_RATE_FILTER_ALPHA = 0.70;

    private static final double FF_ENABLE_TURN_DPS = 25.0; // keep

    private static final double IMU_FF_ENABLE_ERROR_DEG = 3.0;
    private static final double IMU_FF_FULL_ERROR_DEG   = 10.0;

    // -------- Manual turret jog power (D-pad) --------
    private static final double MANUAL_TURRET_POWER = 0.80;

    // -------- Lost target behavior --------
    private static final double LOST_TARGET_GRACE_SEC = 0.10;
    private static final double LOST_TARGET_SEARCH_POWER = 0.00;
    private static final double LOST_SEARCH_DISABLE_TURN_DPS = 90.0;

    // if target is lost but robot is still turning, keep following IMU yaw
    private static final double LOST_IMU_FOLLOW_ENABLE_DPS = 35.0;

    // -------- Soft limits (optional) --------
    private static final boolean ENABLE_SOFT_LIMITS = false;
    private static final int TURRET_MIN_TICKS = -2000;
    private static final int TURRET_MAX_TICKS =  2000;

    // -------- Instant turn-detect from driver rx (zero-lag) --------
    private static final double RX_TURN_BLEND_START = 0.10;
    private static final double RX_TURN_BLEND_FULL  = 0.60;

    // ===================== SHOOTER: VOLTAGE-COMP VELOCITY =====================
    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double MIN_VOLTAGE_FOR_COMP = 10.5;
    private static final double MAX_COMP_MULT = 1.35;
    private static final double MIN_COMP_MULT = 0.85;

    private static final double SHOOTER_VEL_KP = 24.0;
    private static final double SHOOTER_VEL_KI = 0.0;
    private static final double SHOOTER_VEL_KD = 0.001;
    private static final double SHOOTER_VEL_KF = 15.5;

    // ===================== SHOOTER RAPID-FIRE TELEMETRY (DIP + RECOVERY) =====================
    private static final double SHOOT_READY_TOL = 40.0;
    private static final double DIP_DETECT_DROP = 120.0;
    private static final double DIP_END_TOL = 60.0;
    private static final double VEL_FILTER_ALPHA = 0.75;

    // ===================== SETTLE MODE (ARRIVAL ANTI-HUNT) =====================
    private static final double MOVE_EPS = 0.12;
    private static final double SETTLE_HOLD_SEC = 0.45;

    private static final double SETTLE_YAW_DPS = 12.0;
    private static final double SETTLE_RX_EPS = 0.10;

    private static final double SETTLE_MAX_TURRET_POWER = 0.35;
    private static final double SETTLE_KP_MULT = 0.65;
    private static final double SETTLE_KD_MULT = 1.15;

    private static final double SETTLE_TX_DEADBAND = 1.7;
    private static final double SETTLE_MINPOWER_ENABLE_ERR = 999.0;

    // ===================== STATIONARY ANTI-HUNT (ALWAYS WHEN STILL) =====================
    private static final double STATIONARY_YAW_DPS = 8.0;
    private static final double STATIONARY_RX_EPS  = 0.08;

    private static final double STATIONARY_MAX_TURRET_POWER = 0.22;
    private static final double STATIONARY_KP_MULT = 0.55;
    private static final double STATIONARY_KD_MULT = 1.20;

    private static final double STATIONARY_TX_DEADBAND = 2.0;
    private static final double STATIONARY_MINPOWER_ENABLE_ERR = 999.0; // disable min-power at rest

    // ===================== TURN-STOP SETTLE (ANTI-OVERSHOOT WHEN STOP TURNING) =====================
    private static final double TURN_STOP_HOLD_SEC = 0.18;
    private static final double TURNING_RX_EPS = 0.10;
    private static final double TURNING_YAW_DPS = 18.0;

    private static final double TURNSTOP_MAX_TURRET_POWER = 0.28;
    private static final double TURNSTOP_KP_MULT = 0.60;
    private static final double TURNSTOP_KD_MULT = 1.25;

    // =========================================================

    // Tracking toggle
    private boolean trackingEnabled = false;
    private boolean xWasPressed = false;

    // Turret smoothing state
    private double turretFilteredPower = 0.0;

    // PD dt tracking
    private double lastTxError = 0.0;
    private double lastLoopTime = 0.0;

    // tx filter state
    private double txFiltered = 0.0;

    // yaw-rate filter state
    private double yawRateFiltered = 0.0;

    // Target loss handling
    private double lastSeenTime = -999.0;
    private double lastSeenTx = 0.0;

    // Shooter telemetry
    private double shooterTargetVel = 0.0;
    private double shooterTargetVelCmd = 0.0;

    // Shooter latch state (press once = stays on)
    private double shooterSetpoint = 0.0; // 0 = off, otherwise target velocity
    private boolean a2WasPressed = false;
    private boolean y2WasPressed = false;

    // ===== Shooter tuning telemetry state =====
    private double shooterVelFilt = 0.0;
    private double shooterVelPrev = 0.0;

    private double shotDipStartVel = 0.0;
    private double shotMinVel = 999999.0;
    private double shotRecoverStartTime = 0.0;
    private double lastShotTime = -999.0;

    private int shotCount = 0;
    private boolean dipActive = false;

    // ===== Settle-mode state =====
    private boolean wasMoving = false;
    private double settleUntilTime = -999.0;

    // ===== Turn-stop settle state =====
    private boolean wasTurning = false;
    private double turnStopUntilTime = -999.0;

    @Override
    public void init() {

        // ----------------- DRIVE -----------------
        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");
        fi = hardwareMap.dcMotor.get("fi");
        bi = hardwareMap.dcMotor.get("bi");
        lt = hardwareMap.get(Servo.class, "lt"); //2
        rt = hardwareMap.get(Servo.class, "rt"); //1
        ki = hardwareMap.get(Servo.class, "ki"); //0

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ----------------- TURRET + SHOOTER -----------------
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        turretSpin.setDirection(DcMotorSimple.Direction.REVERSE);
        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter: velocity control
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_VEL_KP, SHOOTER_VEL_KI, SHOOTER_VEL_KD, SHOOTER_VEL_KF);
        } catch (Exception ignored) {}

        // ----------------- LIMELIGHT -----------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.start();

        // ----------------- IMU -----------------
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        lastLoopTime = getRuntime();

        lt.setPosition(0.35);
        rt.setPosition(0.35);
        ki.setPosition(0.2);

        fi.setPower(0);
        bi.setPower(0);

        // Shooter telemetry init
        shooterVelFilt = 0.0;
        shooterVelPrev = 0.0;
        dipActive = false;
        shotCount = 0;
        lastShotTime = -999.0;

        // Settle init
        wasMoving = false;
        settleUntilTime = -999.0;

        // Turn-stop init
        wasTurning = false;
        turnStopUntilTime = -999.0;
    }

    @Override
    public void loop() {

        // =========================
        // Loop timing for dt
        // =========================
        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        if (dt < 0.001) dt = 0.001;

        // =========================
        // 1) DRIVE
        // =========================
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        // ===================== SETTLE DETECT (STOP EVENT) =====================
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

        // =========================
        // INTAKE (UNCHANGED MANUAL CONTROL)
        // =========================
        if (gamepad2.left_bumper) {
            fi.setPower(1.0);
            bi.setPower(-0.65);
        }
        else if (gamepad2.right_bumper) {
            fi.setPower(1);
            bi.setPower(1);
        }
        else if (gamepad2.right_trigger==1) {
            fi.setPower(-0.4);
            bi.setPower(-0.4);
        }
        else if (gamepad2.b) {
            fi.setPower(0);
            bi.setPower(0);
        }

        // =========================
        // 1.5) SHOOTER (LATCHED ON PRESS)
        // =========================
        boolean a2Now = gamepad2.a;
        boolean y2Now = gamepad2.y;

        boolean a2Pressed = a2Now && !a2WasPressed;
        boolean y2Pressed = y2Now && !y2WasPressed;

        if (a2Pressed) {
            // lt.setPosition(.8);//not use
            rt.setPosition(.35);
            shooterSetpoint = 1350;
        }
        if (y2Pressed) {
            shooterSetpoint = 1600;
            // lt.setPosition(.6);//not used
            rt.setPosition(0.0);
        }

        a2WasPressed = a2Now;
        y2WasPressed = y2Now;

        if (shooterSetpoint > 0.0) {
            shooterOn3Compensated(shooterSetpoint);
        } else {
            shooterTargetVel = 0.0;
            shooterTargetVelCmd = 0.0;
            shooter.setVelocity(0.0);
        }

        // =========================
        // 1.6) SHOOTER RAPID-FIRE TELEMETRY (DIP + RECOVERY)
        // =========================
        double velRaw = shooter.getVelocity(); // ticks/sec
        shooterVelFilt = (VEL_FILTER_ALPHA * shooterVelFilt) + ((1.0 - VEL_FILTER_ALPHA) * velRaw);

        double dvVel = shooterVelFilt - shooterVelPrev;
        shooterVelPrev = shooterVelFilt;

        boolean shooterOn = (shooterSetpoint > 0.0);

        if (shooterOn) {
            if (!dipActive) {
                boolean suddenDrop = (dvVel < -DIP_DETECT_DROP);
                boolean farBelowTarget = (shooterVelFilt < (shooterTargetVelCmd - 2.0 * DIP_DETECT_DROP));

                if (suddenDrop || farBelowTarget) {
                    dipActive = true;
                    shotDipStartVel = shooterVelFilt;
                    shotMinVel = shooterVelFilt;
                    shotRecoverStartTime = now;
                    lastShotTime = now;
                    shotCount++;
                }
            } else {
                if (shooterVelFilt < shotMinVel) shotMinVel = shooterVelFilt;

                if (shooterVelFilt >= (shotDipStartVel - DIP_END_TOL)) {
                    dipActive = false;
                }
            }
        } else {
            dipActive = false;
            shotCount = 0;
            lastShotTime = -999.0;
        }

        // =========================
        // 2) TRACKING TOGGLE (press X)
        // =========================
        boolean xNow = gamepad2.x;
        if (xNow && !xWasPressed) {
            trackingEnabled = !trackingEnabled;

            lastTxError = 0.0;
            turretFilteredPower = 0.0;
            txFiltered = 0.0;
            yawRateFiltered = 0.0;

            lastSeenTime = -999.0;
            lastSeenTx = 0.0;

            settleUntilTime = -999.0;
            wasMoving = false;

            // turn-stop reset
            turnStopUntilTime = -999.0;
            wasTurning = false;
        }
        xWasPressed = xNow;

        // =========================
        // 3) Manual turret override
        // =========================
        boolean manualLeft  = gamepad2.dpad_left;
        boolean manualRight = gamepad2.dpad_right;

        double desiredTurretPower = 0.0;

        if (manualLeft)  desiredTurretPower = -MANUAL_TURRET_POWER;
        if (manualRight) desiredTurretPower =  MANUAL_TURRET_POWER;

        // =========================
        // 4) IMU yaw rate (filtered + deadband + clamp)
        // =========================
        double yawRateRaw = getYawRateDegPerSec();

        yawRateFiltered =
                (YAW_RATE_FILTER_ALPHA * yawRateFiltered) +
                        ((1.0 - YAW_RATE_FILTER_ALPHA) * yawRateRaw);

        double yawRateUse = applyDeadband(yawRateFiltered, YAW_RATE_DEADBAND_DPS);
        yawRateUse = clip(yawRateUse, -YAW_RATE_CLAMP_DPS, YAW_RATE_CLAMP_DPS);

        // ===================== TURN-STOP DETECT (STOP TURN EVENT) =====================
        boolean turningNow = (Math.abs(rx) > TURNING_RX_EPS) || (Math.abs(yawRateFiltered) > TURNING_YAW_DPS);
        if (wasTurning && !turningNow) {
            turnStopUntilTime = now + TURN_STOP_HOLD_SEC;
        }
        wasTurning = turningNow;

        // =========================
        // 5) Turn amount (0..1) for blending
        // =========================
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

        // IMU FF gain blend
        double ffGain = lerp(TURN_RATE_FF_SLOW, TURN_RATE_FF_FAST, turnBlend);

        // ===================== SETTLE + STATIONARY + TURNSTOP OVERRIDES =====================
        boolean settleWindow = (now <= settleUntilTime);

        boolean settleEligible =
                settleWindow &&
                        (Math.abs(yawRateFiltered) <= SETTLE_YAW_DPS) &&
                        (Math.abs(rx) <= SETTLE_RX_EPS);

        boolean stationaryEligible =
                (Math.abs(yawRateFiltered) <= STATIONARY_YAW_DPS) &&
                        (Math.abs(rx) <= STATIONARY_RX_EPS);

        boolean turnStopEligible = (now <= turnStopUntilTime);

        double txDeadbandUse = TX_DEADBAND_DEG;
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

            txDeadbandUse = SETTLE_TX_DEADBAND;
            minPowerEnableErrUse = SETTLE_MINPOWER_ENABLE_ERR;
        }
        else if (stationaryEligible) {
            maxTurretPower = Math.min(maxTurretPower, STATIONARY_MAX_TURRET_POWER);
            kpUse *= STATIONARY_KP_MULT;
            kdUse *= STATIONARY_KD_MULT;

            txDeadbandUse = STATIONARY_TX_DEADBAND;
            minPowerEnableErrUse = STATIONARY_MINPOWER_ENABLE_ERR;
        }

        // Auto tracking variables (telemetry)
        double tx = 0.0;
        boolean hasTarget = false;

        double txError = 0.0;
        double dError = 0.0;
        double visionPower = 0.0;
        double imuFF = 0.0;
        double autoPower = 0.0;
        double ffScale = 0.0;

        if (!manualLeft && !manualRight && trackingEnabled) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                hasTarget = true;
                tx = result.getTx();

                lastSeenTime = now;
                lastSeenTx = tx;

                // Filter tx (adaptive)
                txFiltered = (txAlpha * txFiltered) + ((1.0 - txAlpha) * tx);

                // Error (deadband)
                txError = applyDeadband(txFiltered, txDeadbandUse);

                // Derivative (deg/sec)
                dError = (txError - lastTxError) / dt;
                lastTxError = txError;

                // Vision PD
                visionPower = (kpUse * txError) + (kdUse * dError);

                // IMU FF scale
                double absErr = Math.abs(txError);

                double ffFromErr = 0.0;
                if (absErr >= IMU_FF_ENABLE_ERROR_DEG) {
                    ffFromErr = (absErr - IMU_FF_ENABLE_ERROR_DEG) /
                            (IMU_FF_FULL_ERROR_DEG - IMU_FF_ENABLE_ERROR_DEG);
                    ffFromErr = clip(ffFromErr, 0.0, 1.0);
                }

                double ffFromTurn = 0.55 * turnBlend;

                ffScale = Math.max(ffFromErr, ffFromTurn);

                // Only use FF if turning enough (and not in turn-stop window)
                if (!killFFThisLoop && Math.abs(yawRateUse) >= FF_ENABLE_TURN_DPS) {
                    imuFF = (+ffGain * yawRateUse) * ffScale;
                } else {
                    imuFF = 0.0;
                }

                autoPower = visionPower + imuFF;

                // Min power only when meaningfully off-center
                if (Math.abs(txError) >= minPowerEnableErrUse) {
                    autoPower = addMinPower(autoPower, MIN_TURRET_POWER);
                }

                desiredTurretPower = clip(autoPower, -maxTurretPower, maxTurretPower);

            } else {
                // Lost target: IMU follow during grace
                double timeSinceSeen = now - lastSeenTime;

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
        }

        // =========================
        // 6) Smooth turret power
        // =========================
        double filteredTarget =
                filterAlpha * turretFilteredPower +
                        (1.0 - filterAlpha) * desiredTurretPower;

        turretFilteredPower = slewTo(turretFilteredPower, filteredTarget, slewPerLoop);

        // =========================
        // 7) Soft limits (optional)
        // =========================
        int turretTicks = turretSpin.getCurrentPosition();

        if (ENABLE_SOFT_LIMITS) {
            if (turretTicks >= TURRET_MAX_TICKS && turretFilteredPower > 0) turretFilteredPower = 0;
            if (turretTicks <= TURRET_MIN_TICKS && turretFilteredPower < 0) turretFilteredPower = 0;
        }

        turretSpin.setPower(turretFilteredPower);

        // =========================
        // TELEMETRY
        // =========================
        telemetry.addLine("RCREDv3: distance sensors removed (intake manual only)");
        telemetry.addData("Tracking Enabled", trackingEnabled);
        telemetry.addData("Has Target", hasTarget);

        telemetry.addData("YawRate raw/filt/use (dps)", "%.2f / %.2f / %.2f", yawRateRaw, yawRateFiltered, yawRateUse);
        telemetry.addData("turnBlend rx/yaw/final", "%.2f / %.2f / %.2f", turnBlendRx, turnBlendYaw, turnBlend);

        telemetry.addData("Settle eligible", settleEligible);
        telemetry.addData("Stationary eligible", stationaryEligible);
        telemetry.addData("TurnStop eligible", turnStopEligible);
        telemetry.addData("TurnStop time left", "%.2f", Math.max(0.0, turnStopUntilTime - now));
        telemetry.addData("Settle time left", "%.2f", Math.max(0.0, settleUntilTime - now));

        telemetry.addData("tx raw/filt/err", "%.2f / %.2f / %.2f", tx, txFiltered, txError);
        telemetry.addData("kp/kd", "%.3f / %.3f", kpUse, kdUse);
        telemetry.addData("txAlpha", "%.2f", txAlpha);

        telemetry.addData("ffGain/ffScale", "%.4f / %.2f", ffGain, ffScale);
        telemetry.addData("vision/imuFF/auto", "%.3f / %.3f / %.3f", visionPower, imuFF, autoPower);
        telemetry.addData("desired/smoothed", "%.3f / %.3f", desiredTurretPower, turretFilteredPower);

        telemetry.addData("Shooter set/target/cmd/act", "%.0f / %.0f / %.0f / %.0f",
                shooterSetpoint, shooterTargetVel, shooterTargetVelCmd, velRaw);
        telemetry.addData("Battery V", "%.2f", getBatteryVoltage());

        double errVel = shooterTargetVelCmd - shooterVelFilt;
        boolean ready = Math.abs(errVel) <= SHOOT_READY_TOL;

        telemetry.addLine("===== SHOOTER TUNING =====");
        telemetry.addData("Vel raw/filt", "%.0f / %.0f", velRaw, shooterVelFilt);
        telemetry.addData("Vel error (cmd-filt)", "%.0f", errVel);
        telemetry.addData("Ready", ready ? "YES" : "NO");

        telemetry.addData("Dip active", dipActive ? "YES" : "NO");
        telemetry.addData("Shot count (dip events)", shotCount);

        if (lastShotTime > 0) telemetry.addData("Last shot age (s)", "%.2f", (now - lastShotTime));
        if (shotCount > 0) {
            telemetry.addData("Last dip start vel", "%.0f", shotDipStartVel);
            telemetry.addData("Last dip min vel", "%.0f", shotMinVel);
            telemetry.addData("Last dip drop", "%.0f", (shotDipStartVel - shotMinVel));
            telemetry.addData("Recover time so far (s)", "%.2f", (now - shotRecoverStartTime));
        }

        telemetry.update();
    }

    // ----------------- IMU HELPER -----------------
    private double getYawRateDegPerSec() {
        AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return av.zRotationRate;
    }

    // ===================== SHOOTER HELPERS =====================

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
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

    private void shooterOn3Compensated(double targetVelTicksPerSec) {
        shooterTargetVel = targetVelTicksPerSec;

        double mult = voltageCompMultiplier();
        shooterTargetVelCmd = targetVelTicksPerSec * mult;

        shooter.setVelocity(shooterTargetVelCmd);
    }

    // ----------------- OTHER HELPERS -----------------
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
