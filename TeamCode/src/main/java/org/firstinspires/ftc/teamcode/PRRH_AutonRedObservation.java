package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class PRRH_AutonRedObservation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, drivetrain::remote);

        robot.init(hardwareMap);

        roadrunner.setPIDCoefficients(Axis.X, 0.1, 0, 0.0001);
        roadrunner.setPIDCoefficients(Axis.Y, 0.3, 0, 0.0001);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.8, 0.0001, 1);

        roadrunner.setPoseEstimate(-33.14, -62.99, 270.00);

        waitForStart();

        while (opModeIsActive()) {

            roadrunner.update();
            telemetry.update();
        }
    }
}
