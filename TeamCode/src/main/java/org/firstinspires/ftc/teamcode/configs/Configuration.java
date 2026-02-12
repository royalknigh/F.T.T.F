package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Configuration {

    public static DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor,
            backRightMotor, launchMotor1, launchMotor2, intakeMotor, turretMotor;

    public static Servo stopper, marco;

    public Configuration(HardwareMap hardwareMap) {
        frontLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "fr");
        backRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "br");

        launchMotor1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lm1");
        launchMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lm2");
        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "im");
        turretMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "turret");

        stopper = hardwareMap.get(Servo.class, "stopper");
        marco = hardwareMap.get(Servo.class, "marco");

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setCurrentAlert(0.9, CurrentUnit.AMPS);


        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //wont reset turret to see if i can keep pos between auto and teleop

        launchMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        launchMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launchMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launchMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        MotorConfigurationType turret = turretMotor.getMotorType().clone();
        turret.setAchieveableMaxRPMFraction(1.0);
        turretMotor.setMotorType(turret);

        MotorConfigurationType configlaunchMotor1 = launchMotor1.getMotorType().clone();
        configlaunchMotor1.setAchieveableMaxRPMFraction(1.0);
        launchMotor1.setMotorType(configlaunchMotor1);

        MotorConfigurationType configlaunchMotor2 = launchMotor2.getMotorType().clone();
        configlaunchMotor2.setAchieveableMaxRPMFraction(1.0);
        launchMotor2.setMotorType(configlaunchMotor2);

        MotorConfigurationType configFrontLeftMotor = frontLeftMotor.getMotorType().clone();
        configFrontLeftMotor.setAchieveableMaxRPMFraction(1.0);
        frontLeftMotor.setMotorType(configFrontLeftMotor);

        MotorConfigurationType configBackLeftMotor = backLeftMotor.getMotorType().clone();
        configBackLeftMotor.setAchieveableMaxRPMFraction(1.0);
        backLeftMotor.setMotorType(configBackLeftMotor);

        MotorConfigurationType configFrontRightMotor = frontRightMotor.getMotorType().clone();
        configFrontRightMotor.setAchieveableMaxRPMFraction(1.0);
        frontRightMotor.setMotorType(configFrontRightMotor);

        MotorConfigurationType configBackRightMotor = backRightMotor.getMotorType().clone();
        configBackRightMotor.setAchieveableMaxRPMFraction(1.0);
        backRightMotor.setMotorType(configBackRightMotor);

        MotorConfigurationType configIntakeMotor = intakeMotor.getMotorType().clone();
        configIntakeMotor.setAchieveableMaxRPMFraction(1.0);
        intakeMotor.setMotorType(configIntakeMotor);
    }

    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }


}