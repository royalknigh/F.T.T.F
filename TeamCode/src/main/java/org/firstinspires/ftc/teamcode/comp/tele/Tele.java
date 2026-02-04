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


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if (startPose != null) {
            follower.setStartingPose(startPose);
            startPose = null;
        } else {
            follower.setStartingPose(new Pose(8, 7, Math.toRadians(180)));
        }
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config);
    }

    @Override
    public void init_loop(){
        if(gamepad1.xWasPressed())  launchSystem.setGoal(launchSystem.blueGoalPose);
        if(gamepad1.bWasPressed())  launchSystem.setGoal(launchSystem.redGoalPose);

        telemetry.addData("goal pose x: ", launchSystem.getGoal().getX());
        telemetry.addData("goal pose y: ", launchSystem.getGoal().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        stateMachine();

        launchSystem.updateTurret(follower.getPose());
        launchSystem.addOffset();
        angleCalculator();

        telemetry.addData("state: ", state);
        telemetry.addData("flywheel velocity", launchSystem.getVelocity());
        telemetry.update();
        displayData();
    }

    public void stateMachine(){
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
                    launchSystem.start(speedCalculator());
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

    public double speedCalculator(){
        double dist = launchSystem.getDistanceToGoal(follower.getPose());
        double speed = dist*2;          //havent found formula yet
        return speed;
    }

    public void angleCalculator(){
        double dist = launchSystem.getDistanceToGoal(follower.getPose());
        double angle = dist*2;          //havent found formula yet
        config.marco.setPosition(angle);
    }

    public void displayData(){
        telemetry.addData("turret ticks: ", launchSystem.getTurretTicks());
        telemetry.addData("turret offset: ", launchSystem.turretOffset);

        telemetry.update();
    }

}