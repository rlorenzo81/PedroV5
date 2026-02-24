package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "SpinUp Diagnostics", group = "Test")

public class SpinUpDiagnostics extends OpMode {

    private DcMotorEx shooter;
    private SpinUpFastKickThenPIDF spinup;
    private VoltageSensor voltageSensor;

    private double lastVelocity = 0;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        spinup = new SpinUpFastKickThenPIDF(shooter);

        telemetry.addLine("READY");
        telemetry.addLine("A = spin up");
        telemetry.addLine("B = stop");
    }

    @Override
    public void loop() {

        // Start spinup
        if (gamepad1.a) {
            spinup.kickMs = 1800;        // longer kick for testing
            spinup.kickPower = 1.0;
            spinup.earlyTakeoverPct = 0.98;
            spinup.start(1700);          // change target here
        }

        // Stop
        if (gamepad1.b) {
            spinup.stop();
        }

        spinup.update();

        double vel = shooter.getVelocity();
        double accel = vel - lastVelocity;
        lastVelocity = vel;

        double volts = voltageSensor.getVoltage();

        // ================= TELEMETRY =================
        telemetry.addLine("==== SHOOTER DIAGNOSTICS ====");
        telemetry.addData("Velocity", "%.1f", vel);
        telemetry.addData("Velocity Delta", "%.1f", accel);
        telemetry.addData("Motor Power", "%.2f", shooter.getPower());
        telemetry.addData("Holding PIDF", spinup.isHoldingPIDF());
        telemetry.addData("Elapsed ms", spinup.getElapsedMs());

        telemetry.addLine();
        telemetry.addLine("==== ELECTRICAL ====");
        telemetry.addData("Battery Voltage", "%.2f", volts);

        telemetry.update();
    }

    // =====================================================
    // INNER SPINUP CLASS
    // =====================================================
    public static class SpinUpFastKickThenPIDF {

        public long kickMs = 1800;
        public double kickPower = 1.0;
        public double earlyTakeoverPct = 0.98;
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

        public void stop() {
            active = false;
            shooter.setPower(0);
        }

        public boolean isHoldingPIDF() {
            return !active;
        }

        public long getElapsedMs() {
            return System.currentTimeMillis() - startMs;
        }
    }
}