package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "SpinUp Fast Kick Test", group = "Test")
@Disabled
public class SpinUpFastKickTest extends OpMode {

    private DcMotorEx shooter;
    private SpinUpFastKickThenPIDF spinup;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // IMPORTANT: make sure motor direction matches your robot
        // shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        spinup = new SpinUpFastKickThenPIDF(shooter);

        telemetry.addLine("Ready - Press A to spin up");
    }

    @Override
    public void loop() {

        // PRESS A = start spinup
        if (gamepad1.a) {
            spinup.kickMs = 1900;        // longer kick
            spinup.kickPower = 1.0;      // full send
            spinup.earlyTakeoverPct = 0.98;

            spinup.start(1800);          // CHANGE target ticks/sec here
        }

        // PRESS B = stop shooter
        if (gamepad1.b) {
            spinup.stop();
        }

        spinup.update();

        telemetry.addData("Velocity", spinup.getVelocity());
        telemetry.addData("Elapsed ms", spinup.getElapsedMs());
        telemetry.addData("Holding PIDF", spinup.isHoldingPIDF());
    }

    // =========================================================
    // INNER CLASS (keeps everything in one file for testing)
    // =========================================================
    public static class SpinUpFastKickThenPIDF {

        public long kickMs = 1900;
        public double kickPower = 1.0;
        public double earlyTakeoverPct = 0.98;
        public long maxTotalMs = 2500;

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

        public double getVelocity() {
            return shooter.getVelocity();
        }
    }
}