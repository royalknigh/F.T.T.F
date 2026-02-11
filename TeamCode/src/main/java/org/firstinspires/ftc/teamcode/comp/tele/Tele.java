package org.firstinspires.ftc.teamcode.comp.tele;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Tele extends OpMode {
    private Follower follower;
    private LaunchSystem launchSystem;
    private Configuration config;
    public static Pose startPose;
    public enum State { PIKCUP, LAUNCH }
    public State state = State.PIKCUP;
    public double angle = 0.5, speed = 1500;
    public boolean idle = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 7, Math.toRadians(180)));
        follower.update();
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config);
    }

    @Override
    public void start(){
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);

        // Update Turret and Shooting Logic
        stateMachine();
        launchSystem.updateTurret(follower.getPose());

        // --- Nudge Controls (D-Pad Left/Right) ---
        if (gamepad1.dpad_right) launchSystem.turretOffset += 5;
        if (gamepad1.dpad_left)  launchSystem.turretOffset -= 5;

        // --- Hood & Velocity Controls ---
        angleCalculator();
        speedCalculator();

        displayData();
    }

    public void stateMachine() {
        if (gamepad1.aWasPressed()) launchSystem.toggleTracking();
        if (gamepad1.xWasPressed()) launchSystem.startReset();

        switch (state) {
            case PIKCUP:
                if (gamepad1.leftBumperWasPressed()) idle = !idle;
                if (idle) launchSystem.idle(); else launchSystem.fullStop();
                config.intakeMotor.setPower(gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger : 0);

                if (gamepad1.yWasPressed()) {
                    state = State.LAUNCH;
                    launchSystem.start(speed);
                }
                break;
            case LAUNCH:
                if (launchSystem.update()) {
                    state = State.PIKCUP;
                }
                break;
        }
    }

    public void displayData() {
        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Mode", launchSystem.isTracking() ? "AUTO-AIM" : "MANUAL/RESET");
        telemetry.addData("Target Deg", "%.2f", launchSystem.getTargetDeg(follower.getPose()));
        telemetry.addData("Current Deg", "%.2f", launchSystem.getCurrentDeg());
        telemetry.addData("Offset (Ticks)", launchSystem.turretOffset);

        telemetry.addData("--- LOCALIZATION ---", "");
        telemetry.addData("Heading", "%.2f Deg", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("--- FLYWHEEL ---", "");
        telemetry.addData("Velocity", "%.0f / %.0f", launchSystem.getVelocity(), speed);
        telemetry.update();
    }

    public void angleCalculator(){
        if(gamepad1.dpadUpWasPressed()) angle += 0.05;
        if(gamepad1.dpadDownWasPressed()) angle -= 0.05;
        angle = Range.clip(angle, 0.15, 0.85);
        config.marco.setPosition(angle);
    }

    public void speedCalculator(){


    }
}