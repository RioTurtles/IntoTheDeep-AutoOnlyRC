package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp_v1 extends LinearOpMode {
    public enum Stage {
        INIT,
        INTAKE_READY,
        INTAKE_GRABBED,
        SCORING_READY,
        HIGH_CHAMBER
    }

    Stage stage = Stage.INIT;
    int armUpDownPos = 0;
    double armPos, gripperPos;
    double direction_y, direction_x, pivot, heading;
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        robot.init(hardwareMap);

        waitForStart();

        robot.imu.resetYaw();
        drivetrain.remote(0, 0, 0, 0);

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            if (stage == Stage.INTAKE_READY || stage == Stage.INTAKE_GRABBED || stage == Stage.SCORING_READY) {
                direction_y = gamepad.left_stick_y * 0.7;
                direction_x = -gamepad.left_stick_x * 0.7;
                pivot = gamepad.right_stick_x * 0.8;
                heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            } else {
                direction_y = gamepad.left_stick_y;
                direction_x = -gamepad.left_stick_x;
                pivot = gamepad.right_stick_x * 0.8;
                heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            armPos = robot.arm.getCurrentPosition();
            gripperPos = robot.gripper.getPosition();

            if (gamepad.touchpad) {
                robot.imu.resetYaw();
            }

            switch (stage) {
                case INIT:
                    robot.setArmPos(5, 1.0);
                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                        robot.setArmPos(1, 1.0);
                        stage = Stage.INTAKE_READY;
                    }
                    break;
                case INTAKE_READY:
                    robot.setGripperPos(0);
                    if (gamepad.right_bumper && !(lastGamepad.right_bumper)) {
                        switch (armUpDownPos) {
                            case 0: //Above sub
                                robot.setArmPos(2, 1.0);
                                armUpDownPos = 1;
                                break;
                            case 1: //Lowered down
                                robot.setArmPos(1, 1.0);
                                armUpDownPos = 0;
                                break;
                        }
                    }
                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                        stage = Stage.INTAKE_GRABBED;
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        stage = Stage.INIT;
                    }
                    break;
                case INTAKE_GRABBED:
                    if (robot.gripper.getPosition() < 0.437) {timer1.reset();}
                    robot.setGripperPos(1);
                    if (timer1.milliseconds() > 300) { //TODO: CHECK MILLISECONDS
                        robot.setArmPos(1, 1.0);
                        armUpDownPos = 0;
                        if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                            stage = Stage.SCORING_READY;
                        }
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        robot.setArmPos(2, 1.0);

                        if (timer1.milliseconds() > 200) {
                            stage = Stage.INTAKE_READY;
                        }
                    }
                    break;
                case SCORING_READY:
                    robot.setArmPos(3, 1.0);
                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                        stage = Stage.HIGH_CHAMBER;
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        robot.setArmPos(2, 1.0);
                        armUpDownPos = 1;
                        stage = Stage.INTAKE_GRABBED;
                    }
                    break;
                case HIGH_CHAMBER:
                    if (robot.arm.getCurrentPosition() < 4500){timer1.reset();}
                    robot.setArmPos(4, 1.0);
                    if (timer1.milliseconds() > 300) {
                        robot.setGripperPos(0);
                    }
                    if (timer1.milliseconds() > 600) {
                        stage = Stage.INIT;
                    }
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("armUpDownPos", armUpDownPos);
            telemetry.addLine();
            telemetry.addData("armPos", armPos);
            telemetry.addData("gripperPos", gripperPos);

            telemetry.update();
            drivetrain.remote(direction_y, direction_x, -pivot, heading);

        }
    }
}
