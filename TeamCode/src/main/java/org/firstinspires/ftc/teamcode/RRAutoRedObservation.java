package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRAutoRedObservation extends LinearOpMode {
    public enum Movestep {
        SCORING_SPECIMEN,
        PREPARE_PUSH,
        PUSH_SAMPLES,
        PARKING
    }
    Movestep movestep = Movestep.SCORING_SPECIMEN;
    ElapsedTime timer1 = new ElapsedTime();
    double maxVel = 30;
    double maxAccel = 17;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.initAuto(hardwareMap);

        Pose2d startPose = new Pose2d(9.00, -63.00, Math.toRadians(90.00));

        Trajectory scoringSpecimen = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(2.00, -28.00)
                        ,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .addTemporalMarker(0.00, 0.00, () -> {
                    robot.setArmPos(3, 1.0);
                })
                .addTemporalMarker(0.20, 0.00, () -> {
                    robot.setClawYaw(1);
                })
                .addTemporalMarker(1.00, 0.00, () -> {
                    robot.setArmPos(4, 1.0);
                    if (timer1.milliseconds() > 800) {
                        robot.setClawPos(0);
                    }
                    if (timer1.milliseconds() > 1000) {
                        robot.setArmPos(5, 1.0);
                    }
                })
                .build();

        Trajectory preparePush = drive.trajectoryBuilder(scoringSpecimen.end())
                .lineToConstantHeading(new Vector2d(18.00, -45.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .splineToConstantHeading(new Vector2d(34.00, -27.00), Math.toRadians(100.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .splineToConstantHeading(new Vector2d(45.00, -10.00), Math.toRadians(40.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .addTemporalMarker(1.00, 0.00, () -> {
                    timer1.reset();
                })
                .build();

        TrajectorySequence pushSamples = drive.trajectorySequenceBuilder(preparePush.end())
                .lineToConstantHeading(new Vector2d(49.00, -55.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .splineToConstantHeading(new Vector2d(60.00, -10.00), Math.toRadians(40.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .lineToConstantHeading(new Vector2d(60.00, -55.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .splineToConstantHeading(new Vector2d(68.00, -10.00), Math.toRadians(40.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .lineToConstantHeading(new Vector2d(68.00, -55.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .addTemporalMarker(1.00, 0.00, () -> {
                    timer1.reset();
                })
                .build();

        TrajectorySequence parking;
        try {
            parking = drive.trajectorySequenceBuilder(pushSamples.end())
                    .lineToConstantHeading(new Vector2d(68.00, -55.00))
                    .build();
        } catch (EmptyPathSegmentException e) {
            parking = pushSamples;
        }

        waitForStart();
        drive.setPoseEstimate(startPose);
//        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (movestep == Movestep.SCORING_SPECIMEN) {
                if (robot.arm.getCurrentPosition() < 3440) {timer1.reset();} //TODO: SET POSITION
            }

            switch (movestep) {
                case SCORING_SPECIMEN:
                    drive.followTrajectory(scoringSpecimen);
                    if (robot.arm.getCurrentPosition() >= 3670) {
                        robot.setArmPos(0, 1.0);
                        robot.setClawYaw(0);
                        movestep = Movestep.PREPARE_PUSH;
                    }
                    break;
                case PREPARE_PUSH:
                    drive.followTrajectory(preparePush);
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES;
                    }
                    break;
                case PUSH_SAMPLES:
                    drive.followTrajectorySequence(pushSamples);
                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PARKING;
                    }
                    break;
                case PARKING:
                    drive.followTrajectorySequence(parking);
                    movestep = Movestep.PARKING;
                    break;
            }

            telemetry.addData("movestep", movestep);
            telemetry.update();
        }

    }

}