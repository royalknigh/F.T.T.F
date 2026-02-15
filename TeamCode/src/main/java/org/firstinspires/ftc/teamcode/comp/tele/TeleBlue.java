package org.firstinspires.ftc.teamcode.comp.tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleBlue")
public class TeleBlue extends OpMode {
    public Follower follower;
    public LaunchSystem launchSystem;
    public Configuration config;

    public static Pose startPose;

    public enum State { PICKUP, LAUNCH }
    public State state = State.PICKUP;

    // These are the values actually used by the motors/servos
    public double angle = 0.5;
    public double speed = 1500;
    public boolean idle = true;
    public Servo marco;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        if (startPose == null) {
            startPose = new Pose(8, 7, Math.toRadians(180));
        }

        follower.setStartingPose(startPose);
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config, LaunchSystem.blueGoalPose);
        this.marco = config.marco;
    }

    @Override
    public void start(){
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);

        double distance = launchSystem.returnDistance(follower.getPose());

        // --- THE FIX: Assign the calculated values to the class variables ---
        this.speed = speedCalculator(distance);
        this.angle = angleCalculator(distance);

        // Apply the angle to the servo immediately
        marco.setPosition(this.angle);

        stateMachine();
        launchSystem.updateTurret(follower.getPose());

        if(gamepad1.rightBumperWasPressed()) {
            follower.setPose(new Pose(25, 124, Math.toRadians(143)));
            launchSystem.manualZeroTurret();
        }

        displayData();
    }

    public void stateMachine() {
        if (gamepad1.aWasPressed()) launchSystem.toggleTracking();
        if (gamepad1.xWasPressed()) launchSystem.startReset();

        switch (state) {
            case PICKUP:
                if (gamepad1.leftBumperWasPressed()) idle = !idle;

                // Keep flywheel at idle speed or stop it
                if (idle) launchSystem.idle(); else launchSystem.fullStop();

                config.intakeMotor.setPower(gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger : 0);

                if (gamepad1.yWasPressed()) {
                    state = State.LAUNCH;
                    // Start launch with the current calculated speed
                    launchSystem.start(this.speed);
                }
                break;
            case LAUNCH:
                if (launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                    state = State.PICKUP;
                }
                break;
        }
    }

    public double angleCalculator(double x) {
//        if (gamepad1.dpad_up) angle += 0.001;
//        if (gamepad1.dpad_down) angle -= 0.001;

        // Base formula:
         double baseAngle = -0.0000347794 * x * x + 0.00953371 * x - 0.209821 +0.05;
         return Range.clip(baseAngle, 0.15, 0.85);

//        return Range.clip(angle, 0.15, 0.85);
    }

    public double speedCalculator(double x) {
//        if (gamepad1.dpad_right) speed += 10;
//        if (gamepad1.dpad_left) speed -= 10;

         double speedValue = 7.97132 * x + 1066.07612+100;
         return Range.clip(speedValue, 1000, 2500);

//        return Range.clip(speed, 1000, 2500);
    }

    public void displayData() {
        telemetry.addData("Status", state);
        telemetry.addData("Target Speed", "%.0f", speed);
        telemetry.addData("Target Angle", "%.3f", angle);
        telemetry.addData("Actual Velo", "%.0f", launchSystem.getVelocity());
        telemetry.addData("Dist", "%.2f", launchSystem.returnDistance(follower.getPose()));
        telemetry.update();
    }
}