package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RRAutoRedBasket extends LinearOpMode {

    public enum Movestep {
        INITIAL_MOVEMENT
    }

    Movestep movestep = Movestep.INITIAL_MOVEMENT;
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        robot.setClawPos(1);

        Pose2d startPose = new Pose2d(33.00, -63.00, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        Trajectory initialMovement = drive.trajectoryBuilder(startPose)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            switch (movestep) {
                case INITIAL_MOVEMENT:
                    break;
            }
        }

    }
}
