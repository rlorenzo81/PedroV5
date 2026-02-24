package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="ShooterRampTuneV1", group="Test")
@Disabled
public class ShooterRampTuneV1 extends OpMode {

    // ===== Hardware =====
    private DcMotorEx shooter;

    // ===== Voltage-comp + PIDF (use what you had working) =====
    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double MIN_VOLTAGE_FOR_COMP = 10.5;
    private static final double MAX_COMP_MULT = 1.35;
    private static final double MIN_COMP_MULT = 0.85;

    // Your existing velocity PIDF (keep these as your baseline)
    private static final double SHOOTER_KP = 24.0;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.001;
    private static final double SHOOTER_KF = 15.5;

    // ===== Ramp settings =====
    // Duration to ramp command from current -> target (seconds)
    private static final double RAMP_SEC = 0.30;

    // Optional “boost”: allow command above target briefly to accelerate harder.
    // Keep this conservative to avoid overshoot/hunting.
    private static final double BOOST_MULT = 1.18;     // 12% over target during ramp
    private static final double BOOST_MAX_ADD = 220.0; // or max +220 ticks/sec over target
    private static final double BOOST_END_ERR = 90.0;  // stop boosting when within this error band

    // ===== State =====
    private double targetVel = 0.0;     // ticks/sec (your desired)
    private double cmdVel = 0.0;        // ticks/sec command we actually send (before voltage comp)
    private double rampStartCmd = 0.0;
    private double rampStartTime = 0.0;
    private boolean rampActive = false;

    private boolean aWas=false, yWas=false, xWas=false, bWas=false;
    private boolean dUpWas=false, dDownWas=false;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        } catch (Exception ignored) {}

        targetVel = 0.0;
        cmdVel = 0.0;
        rampActive = false;
        shooter.setVelocity(0.0);
    }

    @Override
    public void loop() {
        double now = getRuntime();

        // ===== Inputs =====
        boolean aNow = gamepad1.a;
        boolean yNow = gamepad1.y;
        boolean xNow = gamepad1.x;
        boolean bNow = gamepad1.b;

        boolean dUpNow = gamepad1.dpad_up;
        boolean dDownNow = gamepad1.dpad_down;

        boolean aPressed = aNow && !aWas;
        boolean yPressed = yNow && !yWas;
        boolean xPressed = xNow && !xWas;
        boolean bPressed = bNow && !bWas;

        boolean dUpPressed = dUpNow && !dUpWas;
        boolean dDownPressed = dDownNow && !dDownWas;

        aWas=aNow; yWas=yNow; xWas=xNow; bWas=bNow;
        dUpWas=dUpNow; dDownWas=dDownNow;

        if (aPressed) setTargetWithRamp(1350, now);
        if (yPressed) setTargetWithRamp(1600, now);
        if (xPressed) setTargetWithRamp(2000, now);

        if (dUpPressed) setTargetWithRamp(targetVel + 25.0, now);
        if (dDownPressed) setTargetWithRamp(Math.max(0.0, targetVel - 25.0), now);

        if (bPressed) {
            targetVel = 0.0;
            rampActive = false;
            cmdVel = 0.0;
            shooter.setVelocity(0.0);
        }

        // ===== Control update =====
        double actualVel = shooter.getVelocity(); // ticks/sec

        if (targetVel > 0.0) {
            // Build a ramped velocity command (no mode switching)
            cmdVel = computeRampedCommand(now, actualVel);

            // Voltage compensate the command velocity
            double mult = voltageCompMultiplier();
            double cmdVelComp = cmdVel * mult;

            shooter.setVelocity(cmdVelComp);
        }

        // ===== Telemetry =====
        telemetry.addLine("ShooterRampTuneV1 (A=1350, Y=1600, X=2000, B=stop, dpad +/-25)");
        telemetry.addData("Battery V", "%.2f", getBatteryVoltage());
        telemetry.addData("TargetVel", "%.0f", targetVel);
        telemetry.addData("CmdVel (pre-comp)", "%.0f", cmdVel);
        telemetry.addData("ActualVel", "%.0f", actualVel);
        telemetry.addData("Error", "%.0f", (targetVel - actualVel));
        telemetry.addData("RampActive", rampActive);
        telemetry.update();
    }

    /** Call whenever you change target so the ramp restarts cleanly. */
    private void setTargetWithRamp(double newTarget, double now) {
        targetVel = Math.max(0.0, newTarget);

        // Start ramp from the CURRENT command (not actual) to avoid jumps
        rampStartCmd = cmdVel;
        rampStartTime = now;
        rampActive = (targetVel > 0.0);

        if (targetVel == 0.0) {
            cmdVel = 0.0;
            shooter.setVelocity(0.0);
        }
    }

    /**
     * Ramps command from rampStartCmd -> targetVel over RAMP_SEC.
     * Adds a small, safe "boost" early in ramp to reach speed faster, then removes boost near target.
     */
    private double computeRampedCommand(double now, double actualVel) {
        if (!rampActive) return targetVel;

        double t = (now - rampStartTime) / Math.max(0.001, RAMP_SEC);
        if (t >= 1.0) {
            rampActive = false;
            return targetVel;
        }

        // Base ramp (linear). You can swap to smootherstep if you want.
        double base = lerp(rampStartCmd, targetVel, clamp(t, 0.0, 1.0));

        // Optional boost: only while far from target
        double err = targetVel - actualVel;
        if (err > BOOST_END_ERR) {
            double boostCap = Math.min(targetVel * BOOST_MULT, targetVel + BOOST_MAX_ADD);
            return Math.min(base + 0.0, boostCap); // base already increasing; cap it
        }

        // Near target: no boost, let PID settle
        return base;
    }

    // ===== Voltage comp helpers =====
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

    // ===== Small helpers =====
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}