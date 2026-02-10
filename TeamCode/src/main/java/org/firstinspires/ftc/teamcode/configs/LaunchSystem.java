package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Configurable
@Disabled
public class LaunchSystem {
    private final DcMotorEx lm1, lm2, im, turret;
    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean isLaunching = false;
    public static double currentTargetVelocity, idleVelocity = 900;

    private Servo stopper;
    public double holdBall = 0.71, passBall = 0.95;

    public static double P =20, F = 13.07;

    private final double TICKS_PER_DEGREE = (145.1 * 4.2) / 360.0;      //1922.5 / 360.0
    public int turretOffset =0;

    public final Pose blueGoalPose = new Pose(12, 136);
    public final Pose redGoalPose = new Pose(130, 136);
    private Pose goalPose = blueGoalPose;

    public LaunchSystem(Configuration config) {
        this.lm1 = config.launchMotor1;
        this.lm2 = config.launchMotor2;
        this.im = config.intakeMotor;
        this.turret = config.turretMotor;
        this.stopper = config.stopper;

        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean trackingEnabled = false;

    public void toggleTracking() {
        trackingEnabled = !trackingEnabled;
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    public void updateTurret(Pose robotPose) {
        if (!trackingEnabled) {
            return;
        }

        double dx = Math.abs(goalPose.getX() - robotPose.getX());
        double dy = goalPose.getY() - robotPose.getY();

        double angleToGoal = Math.atan2(dy, dx);
        double relativeAngle = Math.toDegrees(angleToGoal - robotPose.getHeading());

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        int targetTicks = (int) (relativeAngle * TICKS_PER_DEGREE);
        turret.setTargetPosition(targetTicks + turretOffset);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1.0);
    }

//   TODO:  launch speed based in distance

    private void setDualVelocity(double velocity) {
        lm1.setVelocity(velocity);
        lm2.setVelocity(velocity);
    }

    public void updatePIDF() {
        lm1.setVelocityPIDFCoefficients(P, 0, 0, F);
        lm2.setVelocityPIDFCoefficients(P, 0, 0, F);
    }

    public void start(double target) {
        this.currentTargetVelocity = target;
        launchTimer.reset();
        isLaunching = true;
        resetTimer = true;
        setDualVelocity(currentTargetVelocity);
    }

    public void idle() {
        isLaunching = false;
        stopper.setPosition(holdBall);
        setDualVelocity(idleVelocity);
    }

    public boolean resetTimer= true;

    public boolean update() {
        if (!isLaunching) return true;

        // Check if flywheels are at target speed
        if (getVelocity() >= (currentTargetVelocity - 50)) {
            if(resetTimer){
                launchTimer.reset();
                resetTimer = false;
            }
            stopper.setPosition(passBall);
            im.setPower(1);

            if (launchTimer.milliseconds() > 1000) {
                return true;
            }
        }
        return false;
    }

    public double getVelocity() {
        return (lm1.getVelocity() + lm2.getVelocity()) / 2.0;
    }

    public void fullStop() {
        isLaunching = false;
        setDualVelocity(0);
    }

    public double getDistanceToGoal(Pose robotPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }
    public double getTurretTicks(){
        return turret.getCurrentPosition();
    }

    public void addOffset(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        if(gamepad.dpadDownWasPressed()) turretOffset -= 10;
        if(gamepad.dpadUpWasPressed()) turretOffset += 10;
    }

    public void setGoal(Pose pose){
        goalPose= pose;
    }
    public Pose getGoal(){
        return goalPose;
    }
}