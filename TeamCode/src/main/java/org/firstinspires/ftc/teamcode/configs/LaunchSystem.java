package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchSystem {
    private DcMotorEx lm1, lm2, im, turret;
    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean isLaunching = false;
    private double currentTargetVelocity = 1800.0;

    // 35:175 (1:5) Gear Ratio Math
    // 435 RPM Motor = 384.5 ticks/rev. 384.5 * 5 = 1922.5 total ticks per turret rev.
    private final double TICKS_PER_DEGREE = 1922.5 / 360.0;
    private final Pose GOAL_POSE = new Pose(12, 132);

    public LaunchSystem(Configuration config) {
        this.lm1 = config.launchMotor1;
        this.lm2 = config.launchMotor2;
        this.im = config.intakeMotor;
        this.turret = config.turretMotor; // Assumes turret is in your config

        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.8);
    }

    public void updateTurret(Pose robotPose) {
        // 1. Calculate Global Angle to Goal
        double dx = GOAL_POSE.getX() - robotPose.getX();
        double dy = GOAL_POSE.getY() - robotPose.getY();
        double angleToGoal = Math.atan2(dy, dx);

        // 2. Relative Angle (Global - Robot Heading)
        double relativeAngle = Math.toDegrees(angleToGoal - robotPose.getHeading());

        // 3. Shortest Path Normalization (-180 to 180)
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        // 4. Set Position using 1:5 Gear Ratio
        int targetTicks = (int) (relativeAngle * TICKS_PER_DEGREE);
        turret.setTargetPosition(targetTicks);
    }

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
        setDualVelocity(900);
    }

    public boolean update() {
        if (!isLaunching) return true;

        if(launchTimer.milliseconds()<1500){
            im.setPower(1);
            setDualVelocity(currentTargetVelocity);
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
}