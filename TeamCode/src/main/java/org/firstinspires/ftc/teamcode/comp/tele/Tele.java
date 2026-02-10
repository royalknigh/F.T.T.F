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
    private Follower follower; private LaunchSystem launchSystem;
    public static Pose startPose; private Configuration config;
    public enum State{PIKCUP, LAUNCH} public State state = State.PIKCUP;
    public boolean idle = true;
    public double angle = 0.5, speed = 1500;
    public boolean autoHood = false;

    // blue reset pose (30, 131, Math.toRadians(143))

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
//        if (startPose != null) {
//            follower.setStartingPose(startPose);
//            startPose = null;
//        } else {
            follower.setStartingPose(new Pose(8, 7, Math.toRadians(180)));
//        }
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config);
        follower.update();
    }

    @Override
    public void init_loop(){
        if(gamepad1.xWasPressed())  launchSystem.setGoal(launchSystem.blueGoalPose);
        if(gamepad1.bWasPressed())  launchSystem.setGoal(launchSystem.redGoalPose);

//        telemetry.addData("goal pose x: ", launchSystem.getGoal().getX());
//        telemetry.addData("goal pose y: ", launchSystem.getGoal().getY());
        telemetry.update();
    }

    @Override
    public void start() {
        // Prepare motors for TeleOp (sets brake modes, etc.)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.75, true);
//        handleMovement();
        stateMachine();

        launchSystem.updateTurret(follower.getPose());
        launchSystem.addOffset(gamepad1);
//        angleCalculator(launchSystem.getDistanceToGoal(follower.getPose()));
        angleCalculator();
        speedCalculator();

        if(gamepad1.rightBumperWasPressed())
            follower.setPose(new Pose(30 ,131, Math.toRadians(144)));

        displayData();
    }



    public void stateMachine(){
        if (gamepad1.aWasPressed()) {
            launchSystem.toggleTracking();
        }
        switch (state){
            case PIKCUP: {
                if(gamepad1.leftBumperWasPressed())
                    idle=!idle;
                if(idle) launchSystem.idle();
                else launchSystem.fullStop();
                double intakePower = (gamepad1.left_trigger>0.1)? gamepad1.left_trigger : 0;
                config.intakeMotor.setPower(intakePower);

                if(gamepad1.yWasPressed()) {
                    state = State.LAUNCH;
                    launchSystem.start(speed);
                }
                break;
            }
            case LAUNCH:{
                if(launchSystem.update()) {
                    state = State.PIKCUP;
                }
                break;
            }
        }
    }

    //TODO: FIND THE FORMULA FOR HOOD POS AND LAUNCH SPEED

    /*public double speedCalculator(double dist){
        speed = dist*2;          //havent found formula yet
        return speed;
    }*/

    public void speedCalculator(){
        if(gamepad1.bWasPressed()) speed +=100;
        if(gamepad1.xWasPressed()) speed -=100;

    }

    /*public void angleCalculator(double dist){
        if(autoHood) {
            dist = launchSystem.getDistanceToGoal(follower.getPose());
            angle = dist * 2;
        }
        else {
            if (gamepad1.dpadUpWasPressed()) angle += 0.05;
            if (gamepad1.dpadDownWasPressed()) angle -= 0.05;
        }
        //havent found formula yet
        config.marco.setPosition(Range.clip(angle, 0.16, 0.85));
    }*/

    public void angleCalculator(){
        if (gamepad1.dpadRightWasPressed()) angle += 0.05;
        if (gamepad1.dpadLeftWasPressed()) angle -= 0.05;

        angle = Range.clip(angle, 0.16, 0.85);
        config.marco.setPosition(angle);
    }

    public void displayData(){
        telemetry.addData("turret ticks: ", launchSystem.getTurretTicks());
        telemetry.addData("turret offset: ", launchSystem.turretOffset);
        telemetry.addData("turret speed: ", speed);
        telemetry.addData("turret angle: ", angle);

        telemetry.addData("getx: ", follower.getPose().getX());
        telemetry.addData("gety: ", follower.getPose().getY());
        telemetry.addData("heading: ", Math.toRadians(follower.getPose().getHeading()));

        telemetry.addData("state: ", state);
        telemetry.addData("flywheel velocity", launchSystem.getVelocity());


        telemetry.update();
    }

    private void handleMovement() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x / 2.0;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        config.setMotorPowers(
                (y + x + rx) / denominator, (y - x + rx) / denominator,
                (y - x - rx) / denominator, (y + x - rx) / denominator
        );
    }

}