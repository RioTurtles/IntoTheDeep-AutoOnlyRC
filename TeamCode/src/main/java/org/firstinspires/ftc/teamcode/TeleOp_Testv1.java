package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp_Testv1 extends LinearOpMode {
    double direction_y, direction_x, pivot, heading;
    Project1Hardware robot = new Project1Hardware(hardwareMap);
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drivetrain = new MecanumDrive(robot);

        Gamepad gamepad = new Gamepad();
        robot.init(hardwareMap);

        waitForStart();

        robot.imu.resetYaw();
        drivetrain.remote(0, 0, 0, 0);

        while (opModeIsActive()) {
            gamepad.copy(gamepad1);

            direction_y = gamepad.left_stick_y;
            direction_x = -gamepad.left_stick_x;
            pivot = gamepad.right_stick_x * 0.8;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad.touchpad) {
                robot.imu.resetYaw();
            }
            if (gamepad.triangle) {
                robot.setArmPos(0, 1.0);
            }
            if (gamepad.square) {
                robot.setArmPos(1, 1.0);
            }
            if (gamepad.cross) {
                robot.setArmPos(2, 1.0);
            }
            if (gamepad.circle) {
                robot.setArmPos(3, 1.0);
            }
            if (gamepad.dpad_up) {
                robot.setArmPos(4, 1.0);
            }
            if (gamepad.dpad_left) {
                robot.setArmPos(5, 1.0);
            }
            if (gamepad.left_bumper) {
                robot.setGripperPos(0);
            }
            if (gamepad.right_bumper) {
                robot.setGripperPos(1);
            }
            if (gamepad.left_trigger > 0) {
                robot.setArmPos(4, 1.0);

                if (robot.arm.getCurrentPosition() < 3900) {timer1.reset();}

                if (timer1.milliseconds() > 300) {
                    robot.setArmPos(5,1.0);
                    robot.setGripperPos(0);
                }
            }

            drivetrain.remote(direction_y, direction_x, -pivot, heading);
            telemetry.addData("arm", robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}