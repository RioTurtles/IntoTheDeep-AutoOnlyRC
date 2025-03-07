package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class NoYaw_Project1Hardware {

    DcMotor frontLeft, frontRight, backLeft, backRight, arm;
    Servo gripper;
    IMU imu;
    HardwareMap hwmap;
    public NoYaw_Project1Hardware(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
    }

    public void init(HardwareMap hardwareMap) {

        hwmap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotor.class, "motorFL");
        frontRight = hardwareMap.get(DcMotor.class, "motorFR");
        backLeft = hardwareMap.get(DcMotor.class, "motorBL");
        backRight = hardwareMap.get(DcMotor.class, "motorBR");

        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: check direction
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gripper.setDirection(Servo.Direction.REVERSE);

        arm.setPower(0);
        gripper.setPosition(0);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    public void setGripperPos(int gripperPos) {
        switch (gripperPos) {
            case 0: //Open claw
                gripper.setPosition(0);
                break;
            case 1: //Close claw
                gripper.setPosition(0.6);
                break;
        }
    }

    public void setArmPos(int armPos, double armPower) { //Positions:
        switch (armPos) {
            case 0: //Reset position
                arm.setTargetPosition(0);
                break;
            case 1: //Above submersible
                arm.setTargetPosition(6820); //TODO: check position
                break;
            case 2: //Lowered down
                arm.setTargetPosition(7335); //TODO: check position
                break;
            case 3: //High chamber
                arm.setTargetPosition(3915); //TODO: check position
                break;
            case 4: //Scoring high chamber
                arm.setTargetPosition(4555); //TODO: check position
                break;
            case 5: //Near reset position
                arm.setTargetPosition(680); //TODO: check position
                break;
            case 6: //After release scoring
                arm.setTargetPosition(4750); //TODO: check position
                break;
        }
        arm.setPower(armPower);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
