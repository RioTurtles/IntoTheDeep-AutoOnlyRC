package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRAutoRedObservation extends LinearOpMode {
    public enum Movestep {
        SCORING_SPECIMEN,
        GRAB_FIRST_SAMPLE,
        RELEASE_FIRST_SAMPLE,
        GRAB_SECOND_SAMPLE,
        RELEASE_SECOND_SAMPLE,
        GRAB_THIRD_SAMPLE,
        RELEASE_THIRD_SAMPLE,
        PARKING
    }
    Movestep movestep = Movestep.SCORING_SPECIMEN;
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        robot.setGripperPos(1);

        Pose2d startPose = new Pose2d(33.00, -63.00, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        Trajectory initialMovement = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(6.00, -29.00), Math.toRadians(90.00))
                .addTemporalMarker(0.00, 0.00, () -> {
                    robot.setArmPos(3, 1.0);
                })
                .addTemporalMarker(0.20, 0.00, () -> {
                    robot.setGripperYaw(1);
                })
                .addTemporalMarker(0.99, 0.00, () -> {
                    robot.setArmPos(4, 1.0);
                    if (timer1.milliseconds() > 200) {
                        robot.setGripperPos(0);
                    }
                    if (timer1.milliseconds() > 300) {
                        robot.setArmPos(5, 1.0);
                    }
                })
                .build();

        TrajectorySequence grabFirstSample = drive.trajectorySequenceBuilder(initialMovement.end())
                .splineToConstantHeading(new Vector2d(25.00, -40.00), Math.toRadians(35.00))
                .addTemporalMarker(0.35, 0.00, () -> {
                    robot.setArmPos(2, 1.0);
                })
                .splineToConstantHeading(new Vector2d(49.00, -33.00), Math.toRadians(45.00))
                .build();

        Trajectory releaseFirstSample = drive.trajectoryBuilder(grabFirstSample.end())
                .splineToSplineHeading(new Pose2d(57.00, -56.00, Math.toRadians(260.00)), Math.toRadians(0.00))
                .build();

        Trajectory grabSecondSample = drive.trajectoryBuilder(releaseFirstSample.end())
                .splineToSplineHeading(new Pose2d(59.00, -33.00, Math.toRadians(90.00)), Math.toRadians(0.00))
                .build();

        Trajectory releaseSecondSample = drive.trajectoryBuilder(releaseFirstSample.end())
                .splineToSplineHeading(new Pose2d(64.00, -56.00, Math.toRadians(260.00)), Math.toRadians(30.00))
                .build();

        Trajectory grabThirdSample = drive.trajectoryBuilder(releaseFirstSample.end())
                .splineToSplineHeading(new Pose2d(69.00, -33.00, Math.toRadians(90.00)), Math.toRadians(0.00))
                .build();

        Trajectory releaseThirdSample = drive.trajectoryBuilder(releaseFirstSample.end())
                .splineToSplineHeading(new Pose2d(57.00, -56.00, Math.toRadians(260.00)), Math.toRadians(0.00))
                .build();

        Trajectory parking = drive.trajectoryBuilder(releaseFirstSample.end())
                .lineToSplineHeading(new Pose2d(57.00, -67.00, Math.toRadians(90.00)))
                .addTemporalMarker(0.01, 0.00, () -> {
                    robot.setArmPos(0, 1.0);
                })
                .build();

        waitForStart();
//        if (isStopRequested()) return;

        while (opModeIsActive()) {
            switch (movestep) {
                case SCORING_SPECIMEN:
                    if (robot.arm.getCurrentPosition() < 0) {timer1.reset();} //TODO: SET POSITION
                    drive.followTrajectory(initialMovement);
                    if (timer1.milliseconds() > 700) { //TODO: check milliseconds
                        robot.setArmPos(3, 1.0);
                        robot.setGripperYaw(0);
                        movestep = Movestep.GRAB_FIRST_SAMPLE;
                    }
                    break;
                case GRAB_FIRST_SAMPLE:
                    drive.followTrajectorySequence(grabFirstSample);
                    timer1.reset();
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.RELEASE_FIRST_SAMPLE;
                    }
                    break;
                case RELEASE_FIRST_SAMPLE:
                    drive.followTrajectory(releaseFirstSample);
                    timer1.reset();
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.GRAB_SECOND_SAMPLE;
                    }
                    break;
                case GRAB_SECOND_SAMPLE:
                    drive.followTrajectory(grabSecondSample);
                    timer1.reset();
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.RELEASE_SECOND_SAMPLE;
                    }
                    break;
                case RELEASE_SECOND_SAMPLE:
                    drive.followTrajectory(releaseSecondSample);
                    timer1.reset();
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.GRAB_THIRD_SAMPLE;
                    }
                    break;
                case GRAB_THIRD_SAMPLE:
                    drive.followTrajectory(grabThirdSample);
                    timer1.reset();
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.RELEASE_THIRD_SAMPLE;
                    }
                    break;
                case RELEASE_THIRD_SAMPLE:
                    drive.followTrajectory(releaseThirdSample);
                    timer1.reset();
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PARKING;
                    }
                    break;
                case PARKING:
                    drive.followTrajectory(parking);
                    movestep = Movestep.PARKING;
                    break;
            }

        }

    }

}