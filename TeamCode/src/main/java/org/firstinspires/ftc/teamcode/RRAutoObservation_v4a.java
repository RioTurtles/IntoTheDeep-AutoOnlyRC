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
public class RRAutoObservation_v4a extends LinearOpMode {
    public enum Movestep {
        INITIAL_MOVEMENT,
        SCORING_SPECIMEN_1,
        PREPARE_PUSH,
        PUSH_SAMPLES_1,
        PUSH_SAMPLES_2,
        PUSH_SAMPLES_3,
        GRAB_SPECIMEN,
        GRABBING_SPECIMEN,
        GO_SCORE_SPECIMEN,
        SCORING_SPECIMEN_2,
        PARKING,
        END;
    }
    Movestep movestep = Movestep.INITIAL_MOVEMENT;
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    Trajectory preparePush, initialMovement, pushSamples1, pushSamples2, pushSamples3, grabSpecimen, goScoreSpecimen;
    TrajectorySequence parking;

    double maxVel, maxAccel;

    @Override
    public void runOpMode() throws InterruptedException {

        maxVel = 55;
        maxAccel = 35;

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.initAuto(hardwareMap);

        Pose2d currentPos = drive.getPoseEstimate();
        Pose2d startPose = new Pose2d(9.00, -63.00, Math.toRadians(90.00));

        initialMovement = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(2.00, -28.00)
                        ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                )
                .addTemporalMarker(0.00, 0.00, () -> {
                    robot.setArmPos(6, 1.0);
                })
                .addTemporalMarker(0.20, 0.00, () -> {
                    robot.setClawYaw(1);
                })
                .addTemporalMarker(1.00, 0.00, () -> {
                    timer1.reset();
                })
                .build();

        waitForStart();
        drive.setPoseEstimate(startPose);
        autoTimer.reset();

        while (opModeIsActive()) {
            currentPos = drive.getPoseEstimate();

            switch (movestep) {
                case INITIAL_MOVEMENT:
                    drive.followTrajectory(initialMovement);
                    currentPos = drive.getPoseEstimate();

                    preparePush = drive.trajectoryBuilder(currentPos)
                            .lineToConstantHeading(new Vector2d(15.00, -45.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(0.00, 0.00, () -> {
                                robot.setArmPos(0, 1.0);
                                robot.setClawYaw(0);
                            })
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    movestep = Movestep.SCORING_SPECIMEN_1;
                    break;
                case SCORING_SPECIMEN_1:
                    robot.setArmPos(4, 1.0);
                    if (robot.arm.getCurrentPosition() >= 678) {
                        robot.setClawPos(0);
                        robot.setArmPos(5, 1.0);
                        if (robot.arm.getCurrentPosition() >= 700) {
                            movestep = Movestep.PREPARE_PUSH;
                        }
                    }
                    break;
                case PREPARE_PUSH:
                    drive.followTrajectory(preparePush);
                    currentPos = drive.getPoseEstimate();

                    pushSamples1 = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(35.00, -27.00), Math.toRadians(110.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .splineToConstantHeading(new Vector2d(36.00, -10.00), Math.toRadians(40.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .splineToConstantHeading(new Vector2d(46.00, -56.00), Math.toRadians(-85.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
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
                    drive.followTrajectory(pushSamples1);
                    currentPos = drive.getPoseEstimate();

                    pushSamples2 = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(46.00, -10.00), Math.toRadians(40.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .splineToConstantHeading(new Vector2d(54.00, -56.00), Math.toRadians(-90.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES_2;
                    }
                    break;
                case PUSH_SAMPLES_2:
                    drive.followTrajectory(pushSamples2);
                    currentPos = drive.getPoseEstimate();

                    pushSamples3 = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(51.00, -10.00), Math.toRadians(40.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .splineToConstantHeading(new Vector2d(60.00, -56.00), Math.toRadians(-90.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
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
                    drive.followTrajectory(pushSamples3);
                    currentPos = drive.getPoseEstimate();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        if (autoTimer.seconds() < 25) {
                                grabSpecimen = drive.trajectoryBuilder(currentPos)
                                        .lineToConstantHeading(new Vector2d(56.00, -64.00)
                                                ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                                        )
                                        .addTemporalMarker(1.00, 0.00, () -> {
                                            timer1.reset();
                                        })
                                    .build();

                            movestep = Movestep.GRAB_SPECIMEN;
                        } else {
                            parking = drive.trajectorySequenceBuilder(currentPos)
                                    .lineToConstantHeading(new Vector2d(47.00, -63.00)
                                            ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                                    )
                                    .addTemporalMarker(1.00, 0.00, () -> {
                                        timer1.reset();
                                    })
                                    .build();

                            movestep = Movestep.PARKING;
                        }
                    }
                    break;
                case GRAB_SPECIMEN:
                    drive.followTrajectory(grabSpecimen);
                    currentPos = drive.getPoseEstimate();

                    goScoreSpecimen = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(0.00, -28.00), Math.toRadians(90.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(0.00, 0.00, () -> {
                                robot.setArmPos(6, 1.0);
                            })
                            .addTemporalMarker(0.20, 0.00, () -> {
                                robot.setClawYaw(1);
                            })
                        .build();

                    if (timer1.milliseconds() > 100) {
                        movestep = Movestep.GRABBING_SPECIMEN;
                    }
                    break;
                case GRABBING_SPECIMEN:
                    robot.setClawPos(1);
                    if (robot.claw.getPosition() >= 0.35) {
                        timer1.reset();
                        movestep = Movestep.GO_SCORE_SPECIMEN;
                    }
                    break;
                case GO_SCORE_SPECIMEN:
                    if (timer1.milliseconds() > 300) {
                        drive.followTrajectory(goScoreSpecimen);
                        currentPos = drive.getPoseEstimate();

                        parking = drive.trajectorySequenceBuilder(currentPos)
                                .lineToConstantHeading(new Vector2d(47.00, -63.00)
                                        , SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                                )
                                .addTemporalMarker(1.00, 0.00, () -> {
                                    timer1.reset();
                                })
                                .build();

                        if (timer1.milliseconds() > 100) {
                            movestep = Movestep.SCORING_SPECIMEN_2;
                        }
                    }
                    break;
                case SCORING_SPECIMEN_2:
                    robot.setArmPos(4, 1.0);
                    if (robot.arm.getCurrentPosition() >= 678) {
                        robot.setClawPos(0);
                        robot.setArmPos(5, 1.0);
                        if (robot.arm.getCurrentPosition() >= 700) {
                            movestep = Movestep.PARKING;
                        }
                    }
                    break;
                case PARKING:
                    drive.followTrajectorySequence(parking);
                    currentPos = drive.getPoseEstimate();

                    if (timer1.milliseconds() > 100) {
                        movestep = Movestep.END;
                    }
                    break;
                case END:
                    break;
            }

            telemetry.addData("Movestep", movestep);
            telemetry.addData("ArmPos", robot.arm.getCurrentPosition());
            telemetry.addData("currentPos", currentPos);
            telemetry.update();
        }

    }

}