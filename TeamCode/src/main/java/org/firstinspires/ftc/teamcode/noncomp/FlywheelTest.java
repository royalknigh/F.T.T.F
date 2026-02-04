package org.firstinspires.ftc.teamcode.noncomp;

import androidx.core.text.util.LocalePreferences;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Configurable
@TeleOp(name = "Flywheel PID Tuner", group = "Tuning")
public class FlywheelTest extends OpMode {

    private DcMotorEx motor1, motor2;

    // Target Velocities
    public static double highVelocity = 1500.0;
    public static double lowVelocity = 900.0;
    public static double curTargetVelocity;

    // Single PIDF Shared Constants
    public static double P = 0.0;
    public static double F = 0.0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "lm1");
        motor2 = hardwareMap.get(DcMotorEx.class, "lm2");

        // Both use encoders
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- DIRECTION CHECK ---
        // If motors are on opposite sides of a wheel, one must be REVERSE
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        curTargetVelocity = highVelocity;
        telemetry.addLine("Ready to Tune");
    }

    @Override
    public void loop() {
        // --- 1. Inputs ---
        if (gamepad1.yWasPressed()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust shared P and F
        if (gamepad1.dpadLeftWasPressed())  F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];
        if (gamepad1.dpadUpWasPressed())    P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed())  P -= stepSizes[stepIndex];

        P = Math.max(0, P);
        F = Math.max(0, F);

        // --- 2. Apply One PID to Both Motors ---
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        motor1.setVelocity(curTargetVelocity);
        motor2.setVelocity(curTargetVelocity);

        // --- 3. Telemetry ---
        telemetry.addData("Target", curTargetVelocity);
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Step", stepSizes[stepIndex]);
        telemetry.addLine("--- Performance ---");
        telemetry.addData("Motor 1 Vel", "%.1f", motor1.getVelocity());
        telemetry.addData("Motor 2 Vel", "%.1f", motor2.getVelocity());
        telemetry.update();
    }
}