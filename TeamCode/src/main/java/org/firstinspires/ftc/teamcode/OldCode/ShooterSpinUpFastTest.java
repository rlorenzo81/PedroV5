package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "ShooterSpinUpFastTest", group = "Test")
@Disabled
public class ShooterSpinUpFastTest extends OpMode {

    // ================= HARDWARE =================
    private DcMotorEx shooter;
    private IMU imu; // not required, but keeps your hub orientation pattern available if needed

    // ================= VOLTAGE COMP =================
    private static final double NOMINAL_VOLTAGE = 12.6;
    private static final double MIN_VOLTAGE_FOR_COMP = 10.5;
    private static final double MAX_COMP_MULT = 1.35;
    private static final double MIN_COMP_MULT = 0.85;

    // ===== Use your current tuned PIDF (same as RedFront) =====
    private static final double SHOOTER_KP = 24.0;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.001;
    private static final double SHOOTER_KF = 15.5;

    // ================= SPINUP-FAST SETTINGS =================
    // Burst open-loop power briefly, then hand off to RUN_USING_ENCODER velocity
    private static final long   KICK_MS     = 120;  // try 80–160ms
    private static final double KICK_POWER  = 1.00; // try 0.90 if too aggressive
    private static final double KICK_MIN_PCT = 0.70; // only kick if below 70% of target

    private static final double HANDOFF_PCT = 0.85; // declare ramp "done" at 85%

    // ================= TARGET CONTROL =================
    private double targetTicksPerSec = 1600; // start here (you said 1200–2000 typical)
    private boolean shooterEnabled = false;

    // ramp state
    private long rampStartMs = 0;
    private boolean ramping = false;

    // button edge tracking
    private boolean aWas = false;
    private boolean bWas = false;
    private boolean dpadUpWas = false;
    private boolean dpadDownWas = false;
    private boolean xWas = false;
    private boolean yWas = false;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            shooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        } catch (Exception ignored) {}

        // optional IMU init (not required for this test)
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )));
        } catch (Exception ignored) {}

        shooter.setVelocity(0);
        shooterEnabled = false;
        ramping = false;
    }

    @Override
    public void loop() {
        // ============ INPUTS ============
        boolean aNow = gamepad1.a;           // toggle ON/OFF (spinUpFast when ON)
        boolean bNow = gamepad1.b;           // hard OFF
        boolean upNow = gamepad1.dpad_up;    // +100
        boolean downNow = gamepad1.dpad_down;// -100
        boolean xNow = gamepad1.x;           // set 1350
        boolean yNow = gamepad1.y;           // set 1750

        boolean aPress = aNow && !aWas;
        boolean bPress = bNow && !bWas;
        boolean upPress = upNow && !dpadUpWas;
        boolean downPress = downNow && !dpadDownWas;
        boolean xPress = xNow && !xWas;
        boolean yPress = yNow && !yWas;

        if (aPress) {
            shooterEnabled = !shooterEnabled;
            // when turning on, force ramp to start fresh
            if (shooterEnabled) {
                ramping = false;
            } else {
                stopShooter();
            }
        }

        if (bPress) {
            shooterEnabled = false;
            stopShooter();
        }

        if (upPress) targetTicksPerSec += 100;
        if (downPress) targetTicksPerSec -= 100;
        targetTicksPerSec = clamp(targetTicksPerSec, 0, 3000);

        if (xPress) targetTicksPerSec = 1350;
        if (yPress) targetTicksPerSec = 2000;

        // ============ RUN ============
        if (shooterEnabled && targetTicksPerSec > 0) {
            spinUpFast(targetTicksPerSec);
        } else {
            // keep it off
            stopShooter();
        }

        // ============ TELEMETRY ============
        double vel = shooter.getVelocity();
        double vbat = getBatteryVoltage();
        double mult = voltageComp();
        long elapsed = ramping ? (System.currentTimeMillis() - rampStartMs) : 0;

        telemetry.addLine("ShooterSpinUpFastTest");
        telemetry.addData("Enabled", shooterEnabled);
        telemetry.addData("Target (ticks/s)", "%.0f", targetTicksPerSec);
        telemetry.addData("Cmd (ticks/s)", "%.0f", targetTicksPerSec * mult);
        telemetry.addData("Vel (ticks/s)", "%.0f", vel);
        telemetry.addData("Battery (V)", "%.2f", vbat);
        telemetry.addData("Comp mult", "%.3f", mult);
        telemetry.addData("Ramping", ramping);
        telemetry.addData("Ramp elapsed (ms)", elapsed);
        telemetry.addData("Mode", shooter.getMode());

        telemetry.addLine("Controls:");
        telemetry.addLine("  A = toggle shooter ON/OFF");
        telemetry.addLine("  B = OFF");
        telemetry.addLine("  Dpad Up/Down = +/- 100 ticks/s");
        telemetry.addLine("  X = 1350, Y = 1750 presets");

        telemetry.update();

        aWas = aNow;
        bWas = bNow;
        dpadUpWas = upNow;
        dpadDownWas = downNow;
        xWas = xNow;
        yWas = yNow;
    }

    // ===================== SPIN-UP FAST (KICK + HANDOFF) =====================
    private void spinUpFast(double targetTicksPerSec) {
        long now = System.currentTimeMillis();

        if (!ramping) {
            ramping = true;
            rampStartMs = now;
        }

        double vel = shooter.getVelocity();
        double mult = voltageComp();

        // 1) short open-loop kick (full power) if we're still low
        if ((now - rampStartMs) < KICK_MS && vel < (targetTicksPerSec * KICK_MIN_PCT)) {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(KICK_POWER); // keep this strong
            return;
        }

        // 2) handoff to velocity control (voltage compensated)
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(targetTicksPerSec * mult);

        // 3) stop "ramping" once close enough (so next loop is steady)
        if (vel >= targetTicksPerSec * HANDOFF_PCT) {
            ramping = false;
        }
    }

    private void stopShooter() {
        ramping = false;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(0);
    }

    // ===================== VOLTAGE COMP HELPERS =====================
    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor v : hardwareMap.voltageSensor) {
            double val = v.getVoltage();
            if (val > 0) min = Math.min(min, val);
        }
        if (!Double.isFinite(min) || min <= 0) return NOMINAL_VOLTAGE;
        return min;
    }

    private double voltageComp() {
        double v = getBatteryVoltage();
        v = Math.max(v, MIN_VOLTAGE_FOR_COMP);
        double mult = NOMINAL_VOLTAGE / v;
        return clamp(mult, MIN_COMP_MULT, MAX_COMP_MULT);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}