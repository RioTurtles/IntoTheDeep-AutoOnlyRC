package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp_v2 extends LinearOpMode {
    public enum CurrentArmPos {
        INIT_POSITION,
        ABOVE_SUBMERSIBLE,
        LOWERED_DOWN,
        PREPARE_SCORING,
        CURRENTLY_SCORING,
        FINISH_SCORING
    }
    double direction_y, direction_x, pivot, heading;
    CurrentArmPos currentArmPos = CurrentArmPos.INIT_POSITION;
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

            direction_y = gamepad.left_stick_y;
            direction_x = -gamepad.left_stick_x;
            pivot = gamepad.right_stick_x * 0.8;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad.touchpad) {
                robot.imu.resetYaw();
            }

            // CONTROLS

            if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                if (currentArmPos == CurrentArmPos.ABOVE_SUBMERSIBLE){
                    robot.setArmPos(2, 1.0);
                    currentArmPos = CurrentArmPos.LOWERED_DOWN;
                } else {
                    robot.setArmPos(1, 1.0);
                    currentArmPos = CurrentArmPos.ABOVE_SUBMERSIBLE;
                }
            }
            if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                robot.setArmPos(3, 1.0);
                currentArmPos = CurrentArmPos.PREPARE_SCORING;
            }
            if (gamepad.left_trigger > 0 && gamepad.right_trigger > 0 && currentArmPos == CurrentArmPos.PREPARE_SCORING) {
            }

            telemetry.update();
            drivetrain.remote(direction_y, direction_x, -pivot, heading);
        }
    }

}
