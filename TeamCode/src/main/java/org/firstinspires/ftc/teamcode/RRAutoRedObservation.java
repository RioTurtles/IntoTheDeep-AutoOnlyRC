package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RRAutoRedObservation extends LinearOpMode {
    public enum Movestep {
        INITIAL_MOVEMENT,
        SCORING_SPECIMEN,
        FIRST_SAMPLE
    }
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        robot.setGripperPos(1);

        Movestep movestep = Movestep.INITIAL_MOVEMENT;

        Pose2d startPose = new Pose2d(33, -63, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory initialMovement = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(6.00, -29.00), Math.toRadians(90.00))
                .build();

        Trajectory firstSample = drive.trajectoryBuilder(initialMovement.end())
                .lineToLinearHeading(new Pose2d(25.00, -40.00, Math.toRadians(-10.00)))
                .splineToSplineHeading(new Pose2d(49.00, -36.00, Math.toRadians(270.00)), Math.toRadians(0.00))
                .build();

        waitForStart();
//        if (isStopRequested()) return;

        switch (movestep) {
            case INITIAL_MOVEMENT:
                if (robot.arm.getCurrentPosition() < 3880) {timer1.reset();} //TODO: SET POSITION
                robot.setArmPos(3, 1.0);
                if (timer1.milliseconds() > 100) {
                    robot.setGripperYaw(1);
                }
                drive.followTrajectory(initialMovement);
                if (timer1.milliseconds() > 2000) { //CHECK MILLISECONDS
                    movestep = Movestep.SCORING_SPECIMEN;
                }
                break;
            case SCORING_SPECIMEN:
                if (robot.arm.getCurrentPosition() < 4480){timer1.reset();} //TODO: SET POSITION
                robot.setArmPos(4, 1.0);
                if (timer1.milliseconds() > 200) {
                    robot.setGripperPos(0);
                }
                if (timer1.milliseconds() > 300) {
                    robot.setArmPos(6, 1.0);
                }
                if (timer1.milliseconds() > 700) {
                    robot.setArmPos(3, 1.0);
                    robot.setGripperPos(0);
                    robot.setGripperYaw(0);

                    movestep = Movestep.FIRST_SAMPLE;
                }
                break;
            case FIRST_SAMPLE:
                timer1.reset();
                if (timer1.nanoseconds() > 500) {
                    robot.setArmPos(2, 1.0);
                }
                drive.followTrajectory(firstSample);
                break;
        }
    }
}