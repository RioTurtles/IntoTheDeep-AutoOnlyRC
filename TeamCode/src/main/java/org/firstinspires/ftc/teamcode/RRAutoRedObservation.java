package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        INITIAL_MOVEMENT,
        SCORING_SPECIMEN,
        PREPARE_PUSH,
        PUSH_SAMPLES_1,
        PUSH_SAMPLES_2,
        PUSH_SAMPLES_3,
        PARKING
    }
    Movestep movestep = Movestep.INITIAL_MOVEMENT;
    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.initAuto(hardwareMap);

        Pose2d currentPos = drive.getPoseEstimate();
        Pose2d startPose = new Pose2d(9.00, -63.00, Math.toRadians(90.00));

        Trajectory initialMovement = drive.trajectoryBuilder(startPose)
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
//                .addTemporalMarker(1.00, 0.10, () -> {
//                    robot.setArmPos(4, 1.0);
//                    robot.setClawPos(0);
//                    timer1.reset();
//                    sleep(100);
//                    robot.setArmPos(5, 1.0);
//                })
                .build();

        Trajectory preparePush = drive.trajectoryBuilder(initialMovement.end())
                .lineToConstantHeading(new Vector2d(15.00, -45.00)
                        ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(17)
                )
                .addTemporalMarker(0.00, 0.00, () -> {
                    robot.setArmPos(0, 1.0);
                    robot.setClawYaw(0);
                })
                .splineToConstantHeading(new Vector2d(35.00, -27.00), Math.toRadians(100.00)
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

        TrajectorySequence pushSamples1 = null;
        TrajectorySequence pushSamples2 = null;
        TrajectorySequence pushSamples3 = null;

        TrajectorySequence parking = null;

        waitForStart();
        drive.setPoseEstimate(startPose);
        //if (isStopRequested()) return;

        while (opModeIsActive()) {

            switch (movestep) {
                case INITIAL_MOVEMENT:
                    drive.followTrajectory(initialMovement);
                    movestep = Movestep.SCORING_SPECIMEN;
                    break;
                case SCORING_SPECIMEN:
                    pushSamples1 = drive.trajectorySequenceBuilder(preparePush.end())
                            .lineToConstantHeading(new Vector2d(49.00, -52.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(17)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();
                    robot.setArmPos(4, 1.0);
                    if (robot.arm.getCurrentPosition() >= 3545) {
                        robot.setClawPos(0);
                        robot.setArmPos(5, 1.0);
                        if (robot.arm.getCurrentPosition() >= 3670) {
                            movestep = Movestep.PREPARE_PUSH;
                        }
                    }
                    break;
                case PREPARE_PUSH:
                    drive.followTrajectory(preparePush);

                    pushSamples2 = drive.trajectorySequenceBuilder(pushSamples1.end())
                            .lineToConstantHeading(new Vector2d(56.00, -10.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(17)
                            )
                            .lineToConstantHeading(new Vector2d(56.00, -52.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(17)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES_1;
                    }
                    break;
                case PUSH_SAMPLES_1:
                    drive.followTrajectorySequence(pushSamples1);

                    parking = drive.trajectorySequenceBuilder(pushSamples3.end())
                            .lineToConstantHeading(new Vector2d(65.00, -53.00))
                            .build();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES_2;
                    }
                    break;
                case PUSH_SAMPLES_2:
                    drive.followTrajectorySequence(pushSamples2);

                    pushSamples3 = drive.trajectorySequenceBuilder(pushSamples2.end())
                            .splineToConstantHeading(new Vector2d(64.00, -10.00), Math.toRadians(60.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(17)
                            )
                            .lineToConstantHeading(new Vector2d(65.00, -52.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(17)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES_3;
                    }
                    break;
                case PUSH_SAMPLES_3:
                    drive.followTrajectorySequence(pushSamples3);

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PARKING;
                    }
                    break;
                case PARKING:
                    drive.followTrajectorySequence(parking);
                    movestep = Movestep.PARKING;
                    break;
            }

            currentPos = drive.getPoseEstimate();

            telemetry.addData("Movestep", movestep);
            telemetry.addData("ArmPos", robot.arm.getCurrentPosition());
            telemetry.addData("currentPos", currentPos);
            telemetry.update();
        }

    }

}