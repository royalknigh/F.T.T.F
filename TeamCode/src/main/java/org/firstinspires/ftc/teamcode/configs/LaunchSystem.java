package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchSystem {
    private final DcMotorEx lm1, lm2, im, turret;
    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean isLaunching = false;
    public double currentTargetVelocity, highVelocity = 1600, lowVelocity = 1250;

    private Servo stpp;
    public double holdBall, passBall;

    private final double TICKS_PER_DEGREE = (145.1 * 5.0) / 360.0;
    private final Pose goalPose = new Pose(12, 132);

    public LaunchSystem(Configuration config) {
        this.lm1 = config.launchMotor1;
        this.lm2 = config.launchMotor2;
        this.im = config.intakeMotor;
        this.turret = config.turretMotor;
        this.stpp = config.stpp;

        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.8);
    }

    public void updateTurret(Pose robotPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double angleToGoal = Math.atan2(dy, dx);
        double relativeAngle = Math.toDegrees(angleToGoal - robotPose.getHeading());

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        int targetTicks = (int) (relativeAngle * TICKS_PER_DEGREE);
        turret.setTargetPosition(targetTicks);
    }

//   TODO:  launch speed based in distance

    private void setDualVelocity(double velocity) {
        lm1.setVelocity(velocity);
        lm2.setVelocity(velocity);
    }

    public void start(double target) {
        this.currentTargetVelocity = target;
        launchTimer.reset();
        isLaunching = true;
        setDualVelocity(currentTargetVelocity);
    }

    public void idle() {
        isLaunching = false;
        stpp.setPosition(holdBall);
        setDualVelocity(900);
    }

    public boolean update() {
        if (!isLaunching) return true;
        if(getVelocity()>=currentTargetVelocity)
            if (launchTimer.milliseconds() < 1000) {
                stpp.setPosition(passBall);
                im.setPower(1);
                return true;
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
}