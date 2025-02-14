package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class RidgingTest extends LinearOpMode {
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        Gamepad gamepad = new Gamepad();
        robot.init(hardwareMap);

        waitForStart();

        robot.imu.resetYaw();
        drivetrain.remote(0, 0, 0, 0);

        while (opModeIsActive()) {
            gamepad.copy(gamepad1);

            if (gamepad.touchpad) {
                robot.imu.resetYaw();
            }
            if (gamepad.left_bumper) {
                robot.lowerRidging(1.0);
            }
            if (gamepad.right_bumper) {
                robot.raiseRidging(1.0);
            }

            telemetry.addData("arm", robot.arm.getCurrentPosition());
            telemetry.addData("sliderL", robot.sliderL.getCurrentPosition());
            telemetry.addData("sliderR", robot.sliderR.getCurrentPosition());
            telemetry.update();
        }
    }
}