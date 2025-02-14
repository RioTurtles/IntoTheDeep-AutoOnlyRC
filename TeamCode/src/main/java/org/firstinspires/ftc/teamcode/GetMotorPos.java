package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GetMotorPos extends LinearOpMode {
    double armPos;

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.setClawPos(0);
            }
            if (gamepad1.right_bumper) {
                robot.setClawPos(1);
            }
            if (gamepad1.left_trigger > 0) {
                robot.setClawYaw(0);
            }
            if (gamepad1.right_trigger > 0) {
                robot.setClawYaw(1);
            }

            telemetry.addData("armPos", robot.arm.getCurrentPosition());
            telemetry.addData("sliderLPos", robot.sliderL.getCurrentPosition());
            telemetry.addData("sliderRPos", robot.sliderR.getCurrentPosition());
            telemetry.update();
        }

    }

}