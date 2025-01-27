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
        SAMPLE_INTAKE_READY,
        SAMPLE_GRABBED,
        SAMPLE_INIT_POS,
        SAMPLE_PUTDOWN,
        INTAKE_SPECIMEN,
        SPECIMEN_GRABBED,
        SPECIMEN_RAISED,
        SCORING_READY,
        HIGH_CHAMBER,
        DOING_SCORING
    }
    public enum IntakeStage {
        SAMPLE,
        SPECIMEN
    }

    Stage stage = Stage.INIT;
    IntakeStage intakeStage = IntakeStage.SAMPLE;
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

            if (gamepad.left_bumper && !(lastGamepad.left_bumper)) {
                switch (intakeStage) {
                    case SAMPLE:
                        intakeStage = IntakeStage.SPECIMEN;
                        break;
                    case SPECIMEN:
                        intakeStage = IntakeStage.SAMPLE;
                        break;
                }
            }

            if (stage == Stage.SAMPLE_INTAKE_READY) {
                direction_y = gamepad.left_stick_y * 0.55;
                direction_x = -gamepad.left_stick_x * 0.55;
                pivot = gamepad.right_stick_x * 0.6;
                heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            } else if (stage == Stage.SAMPLE_GRABBED || stage == Stage.SCORING_READY) {
                direction_y = gamepad.left_stick_y * 0.65;
                direction_x = -gamepad.left_stick_x * 0.65;
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
                    robot.setGripperPos(0);
                    robot.setGripperYaw(0);
                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                        if (intakeStage == IntakeStage.SAMPLE) {
                            robot.setArmPos(1, 1.0);
                            stage = Stage.SAMPLE_INTAKE_READY;
                        }
                        if (intakeStage == IntakeStage.SPECIMEN) {
                            stage = Stage.INTAKE_SPECIMEN;
                        }
                    }
                    break;
                case SAMPLE_INTAKE_READY:
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
                        stage = Stage.SAMPLE_GRABBED;
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        stage = Stage.INIT;
                    }
                    break;
                case SAMPLE_GRABBED:
                    if (robot.gripper.getPosition() < 0.6) {timer1.reset();} //TODO SET POSITION
                    robot.setGripperPos(1);
                    if (timer1.milliseconds() > 200) {
                        robot.setArmPos(1, 1.0);
                        armUpDownPos = 0;
                        if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                            stage = Stage.SAMPLE_INIT_POS;
                        }
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        robot.setArmPos(2, 1.0);

                        if (timer1.milliseconds() > 300) {
                            stage = Stage.SAMPLE_INTAKE_READY;
                        }
                    }
                    break;
                case SAMPLE_INIT_POS:
                    if (robot.arm.getCurrentPosition() > 370) {timer1.reset();} //TODO SET POSITION
                    robot.setArmPos(9, 1.0);
                    if (timer1.milliseconds() > 100) {
                        stage = Stage.SAMPLE_PUTDOWN;
                    }
                    break;
                case SAMPLE_PUTDOWN:
                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                        robot.setGripperPos(0);
                        if (intakeStage == IntakeStage.SAMPLE) {
                            intakeStage = IntakeStage.SPECIMEN;
                        }
                        stage = Stage.INIT;
                    }
                    break;
                case INTAKE_SPECIMEN:
                    robot.setArmPos(7, 1.0);
                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                        stage = Stage.SPECIMEN_GRABBED;
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        robot.setGripperYaw(0);
                        stage = Stage.INIT;
                    }
                    break;
                case SPECIMEN_GRABBED:
                    if (robot.gripper.getPosition() < 0.6){timer1.reset();} //TODO SET POSITION
                    robot.setGripperPos(1);
                    if (timer1.milliseconds() > 200) { //TODO: check milliseconds
                        stage = Stage.SPECIMEN_RAISED;
                    }
                    break;
                case SPECIMEN_RAISED:
                    if (robot.arm.getCurrentPosition() < 1740){timer1.reset();} //TODO SET POSITION
                    robot.setArmPos(8, 1.0);
                    if (timer1.milliseconds() > 100) {
                        robot.setGripperYaw(1);
                        if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                            stage = Stage.SCORING_READY;
                        }
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        robot.setGripperYaw(0);
                        robot.setGripperPos(0);
                        stage = Stage.INTAKE_SPECIMEN;
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
                        stage = Stage.SAMPLE_GRABBED;
                    }
                    break;
                case HIGH_CHAMBER:
                    if (robot.arm.getCurrentPosition() < 4480){timer1.reset();} //TODO SET POSITION
                    robot.setArmPos(4, 1.0);
                    if (timer1.milliseconds() > 300) {
                        stage = Stage.DOING_SCORING;
                    }
                    break;
                case DOING_SCORING:
                    if (robot.arm.getCurrentPosition() < 4480){timer1.reset();} //TODO SET POSITION
                    if (timer1.milliseconds() > 200) {
                        robot.setGripperPos(0);
                    }
                    if (timer1.milliseconds() > 300) {
                        robot.setArmPos(6, 1.0);
                    }
                    if (timer1.milliseconds() > 700) {
                        if (intakeStage == IntakeStage.SPECIMEN) {
                            intakeStage = IntakeStage.SAMPLE;
                        }
                        stage = Stage.INIT;
                    }
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        stage = Stage.SCORING_READY;
                    }
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("intakeStage", intakeStage);
            telemetry.addData("armUpDownPos", armUpDownPos);
            telemetry.addLine();
            telemetry.addData("armPos", armPos);
            telemetry.addData("gripperPos", gripperPos);

            telemetry.update();
            drivetrain.remote(direction_y, direction_x, -pivot, heading);

        }
    }
}
