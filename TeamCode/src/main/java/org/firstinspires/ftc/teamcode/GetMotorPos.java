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
            armPos = robot.arm.getCurrentPosition();

            if (gamepad1.left_bumper) {
                robot.setGripperPos(0);
            }
            if (gamepad1.right_bumper) {
                robot.setGripperPos(1);
            }
            if (gamepad1.left_trigger > 0) {
                robot.setGripperYaw(0);
            }
            if (gamepad1.right_trigger > 0) {
                robot.setGripperYaw(1);
            }

            telemetry.addData("armPos", armPos);
            telemetry.update();
        }

    }

}