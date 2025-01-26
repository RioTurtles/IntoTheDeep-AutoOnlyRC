package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GetMotorPos extends LinearOpMode {
    double armPos;

    @Override
    public void runOpMode() throws InterruptedException {

        NoYaw_Project1Hardware robot = new NoYaw_Project1Hardware(hardwareMap);
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

            telemetry.addData("armPos", armPos);
            telemetry.update();
        }

    }

}