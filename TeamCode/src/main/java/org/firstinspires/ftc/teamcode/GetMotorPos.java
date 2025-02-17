package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class GetMotorPos extends LinearOpMode {
    double armPos;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap);
        robot.setClawYaw(1);
        robot.setClawPos(1);

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