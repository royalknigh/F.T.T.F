package org.firstinspires.ftc.teamcode.noncomp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Test", group = "TEST")
public class TurretTest extends OpMode {

    private Configuration config;
    private DcMotorEx turret;
    private Follower follower;

    // --- Turret constants ---
    private final double TICKS_PER_DEGREE = (145.1 * (190.0 / 45.0)) / 360.0;

    private boolean fieldMode = false;

    @Override
    public void init() {
        config = new Configuration(hardwareMap);
        turret = config.turretMotor;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        // ===============================
        // DRIVER CONTROLS
        // ===============================

        // Left stick X = manual turret control
        double manualPower = -gamepad1.left_stick_x;

        // Toggle field tracking
        if (gamepad1.a) fieldMode = true;
        if (gamepad1.b) fieldMode = false;

        // Reset encoder
        if (gamepad1.x) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (!fieldMode) {
            turret.setPower(manualPower * 0.4);
        } else {
            // ===============================
            // FIELD HOLD TEST (keep turret at 0° field)
            // ===============================

            double robotHeadingRad = pose.getHeading();

            // Convert to degrees
            double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

            // Target: keep turret pointing at field 0°
            double targetDeg = -robotHeadingDeg;

            // Limit to ±90°
            targetDeg = Range.clip(targetDeg, -90, 90);

            double currentDeg = getCurrentDeg();
            double error = targetDeg - currentDeg;

            // Simple P control
            double power = error * 0.02;

            // static friction compensation
            if (Math.abs(error) > 1) {
                power += Math.signum(power) * 0.07;
            }

            turret.setPower(Range.clip(power, -0.5, 0.5));
        }

        // ===============================
        // TELEMETRY
        // ===============================

        telemetry.addLine("=== MODE ===");
        telemetry.addData("Field Mode (A/B)", fieldMode);

        telemetry.addLine("=== HEADING ===");
        telemetry.addData("Heading RAW", pose.getHeading());
        telemetry.addData("Heading Deg", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Encoder", turret.getCurrentPosition());
        telemetry.addData("Turret Deg", getCurrentDeg());

        telemetry.addLine("Controls:");
        telemetry.addLine("Left stick X = manual");
        telemetry.addLine("A = Field hold");
        telemetry.addLine("B = Manual");
        telemetry.addLine("X = Reset encoder");

        telemetry.update();
    }

    private double getCurrentDeg() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }
}
