package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Project1Hardware {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, arm, sliderL, sliderR;
    Servo claw, clawYaw;
    IMU imu;
    HardwareMap hwmap;

    boolean clawClosed;

    public Project1Hardware(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
    }

    public void init(HardwareMap hardwareMap) {
        hwmap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotorEx.class, "motorFL");
        frontRight = hardwareMap.get(DcMotorEx.class, "motorFR");
        backLeft = hardwareMap.get(DcMotorEx.class, "motorBL");
        backRight = hardwareMap.get(DcMotorEx.class, "motorBR");

        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "gripper");
        clawYaw = hardwareMap.get(Servo.class, "gripperYaw");

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        sliderL.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: Check direction
        sliderR.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: Check direction

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE



        ); //TODO: Change to brake
        sliderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Change to brake

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: check direction
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        claw.setDirection(Servo.Direction.REVERSE);
        clawYaw.setDirection(Servo.Direction.FORWARD);

        sliderL.setPower(0);
        sliderR.setPower(0);

        arm.setPower(0);
        setClawPos(0);
        setClawYaw(0);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
    }

    public void initAuto(HardwareMap hardwareMap) {
        hwmap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotorEx.class, "motorFL");
        frontRight = hardwareMap.get(DcMotorEx.class, "motorFR");
        backLeft = hardwareMap.get(DcMotorEx.class, "motorBL");
        backRight = hardwareMap.get(DcMotorEx.class, "motorBR");

        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "gripper");
        clawYaw = hardwareMap.get(Servo.class, "gripperYaw");

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        sliderL.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: Check direction
        sliderR.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: Check direction

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Change to brake
        sliderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //TODO: Change to brake

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: check direction
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        claw.setDirection(Servo.Direction.REVERSE);
        clawYaw.setDirection(Servo.Direction.FORWARD);

        sliderL.setPower(0);
        sliderR.setPower(0);

        arm.setPower(0);
        setClawPos(1);
        setClawYaw(0);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
    }

    public double getIMUYaw() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    /**
     * Sets claw position.
     * @param clawPos 0 - open; 1 - closed
     */
    public void setClawPos(int clawPos) {
        switch (clawPos) {
            case 0: //Open claw
                claw.setPosition(0);
                clawClosed = false;
                break;
            case 1: //Close claw
                claw.setPosition(0.4);
                clawClosed = true;
                break;
        }
    }

    /**
     * Sets the yaw of the claw.
     * @param clawYawPos 0 - sample intake; 1 - specimen intake
     */
    public void setClawYaw(int clawYawPos) {
        switch (clawYawPos) {
            case 0: //Sample Intake
                clawYaw.setPosition(0.026);
                break;
            case 1: //Specimen Intake
                clawYaw.setPosition(0.686);
                break;
        }
    }

    /**
     * Sets arm position.
     * @param armPos Arm position.<br/>
     *               0 - reset/intake<br/>
     *               1 - above submersible<br/>
     *               2 - submersible lowered down<br/>
     *               3 - prepared to score specimen / vertically above<br/>
     *               4 - scoring, high chamber<br/>
     *               5 - release, after scoring
     *               6 - above intake specimen
     *               7 - RESETTING CAR
     * @param armPower Arm power.
     */
    public void setArmPos(int armPos, double armPower) { //Positions:
        switch (armPos) {
            case 0: //Reset or Intake Specimen position
                arm.setTargetPosition(30);
                break;
            case 1: //Above submersible
                arm.setTargetPosition(1075);
                break;
            case 2: //submersible lowered down
                arm.setTargetPosition(1187); //TODO: adjustments
                break;
            case 3: //Prepare score specimen or Vertical above
                arm.setTargetPosition(655);
                break;
            case 4: //Scoring high chamber
                arm.setTargetPosition(683);
                break;
            case 5: //After release scoring
                arm.setTargetPosition(705);
                break;
            case 6: //After intake specimen
                arm.setTargetPosition(180);
                break;
            case 7: //RESETTING CAR
                arm.setTargetPosition(0);
                break;
            case 8: //Rigging position
                arm.setTargetPosition(530);
        }
        arm.setPower(armPower);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean armInPosition() {
        return Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) <= 5;
    }
    public void lowerRidging() {
        sliderL.setPower(-0.7);
        sliderR.setPower(-0.7);
    }
    public void raiseRidging() {
        sliderL.setPower(0.7);
        sliderR.setPower(0.7);
    }
    public void noRigging() {
        sliderL.setPower(0);
        sliderR.setPower(0);
    }

//    public void setArmPos(int armPos, double armPower) { //Positions:
//        switch (armPos) {
//            case 0: //Reset position
//                arm.setTargetPosition(0);
//                break;
//            case 1: //Above submersible
//                arm.setTargetPosition(6640); //TODO: check position
//                break;
//            case 2: //Lowered down
//                arm.setTargetPosition(7210); //TODO: check position
//                break;
//            case 3: //High chamber
//                arm.setTargetPosition(3880);
//                break;
//            case 4: //Scoring high chamber
//                arm.setTargetPosition(4480);
//                break;
//            case 5: //Near reset position
//                arm.setTargetPosition(370); //TODO: check position
//                break;
//            case 6: //After release scoring
//                arm.setTargetPosition(4720);
//                break;
//            case 7: //Intake Specimen
//                arm.setTargetPosition(1130); //TODO: check position
//                break;
//            case 8: //Above Intake Specimen
//                arm.setTargetPosition(1740); //TODO: check position
//                break;
//            case 9: //Put down sample
//                arm.setTargetPosition(370); //TODO: check position
//                break;
//        }
//        arm.setPower(armPower);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

}
