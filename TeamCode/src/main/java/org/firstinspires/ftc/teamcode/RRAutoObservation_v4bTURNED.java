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
public class RRAutoObservation_v4bTURNED extends LinearOpMode {
    public enum Movestep {
        INITIAL_MOVEMENT,
        SCORING_SPECIMEN_INITIAL,
        PUSH_SAMPLES_1,
        PUSH_SAMPLES_2,
        GRAB_SPECIMEN_1,
        GRABBING_SPECIMEN_1,
        GO_SCORE_SPECIMEN_1,
        SCORING_SPECIMEN_1,
        GRAB_SPECIMEN_2,
        GRABBING_SPECIMEN_2,
        GO_SCORE_SPECIMEN_2,
        SCORING_SPECIMEN_2,
        PARKING,
        END;
    }
    Movestep movestep = Movestep.INITIAL_MOVEMENT;
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    Trajectory initialMovement, pushSamples1, pushSamples2, grabSpecimen, goScoreSpecimen, grabSpecimen2, goScoreSpecimen2;
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
        Pose2d startPose = new Pose2d(9.00, -63.00, Math.toRadians(180.00));

        initialMovement = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2.00, -28.00, Math.toRadians(90.00))
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

                    pushSamples1 = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(15.00, -45.00), Math.toRadians(-50.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(0.00, 0.00, () -> {
                                robot.setArmPos(0, 1.0);
                                robot.setClawYaw(0);
                            })
                            .addTemporalMarker(0.70, 0.00, () -> {
                                robot.setClawPos(0);
                            })
                            .splineToConstantHeading(new Vector2d(37.00, -27.00), Math.toRadians(110.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .splineToConstantHeading(new Vector2d(35.00, -10.00), Math.toRadians(40.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .splineToConstantHeading(new Vector2d(45.00, -56.00), Math.toRadians(-85.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    movestep = Movestep.SCORING_SPECIMEN_INITIAL;
                    break;
                case SCORING_SPECIMEN_INITIAL:
                    robot.setArmPos(4, 1.0);
                    if (robot.arm.getCurrentPosition() >= 678) {
                        robot.setClawPos(0);
                        robot.setArmPos(5, 1.0);
                        if (robot.arm.getCurrentPosition() >= 700) {
                            movestep = Movestep.PUSH_SAMPLES_1;
                        } else
                        if (autoTimer.seconds() >= 6) {
                            robot.setClawPos(0);
                            movestep = Movestep.PUSH_SAMPLES_1;
                        }
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

                    grabSpecimen = drive.trajectoryBuilder(currentPos)
                            .lineToConstantHeading(new Vector2d(36.00, -64.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.GRAB_SPECIMEN_1;
                    }
                    break;
                case GRAB_SPECIMEN_1:
                    drive.followTrajectory(grabSpecimen);
                    currentPos = drive.getPoseEstimate();

                    goScoreSpecimen = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(0.50, -28.00), Math.toRadians(90.00)
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
                        movestep = Movestep.GRABBING_SPECIMEN_1;
                    }
                    break;
                case GRABBING_SPECIMEN_1:
                    robot.setClawPos(1);
                    if (robot.claw.getPosition() >= 0.35) {
                        timer1.reset();
                        movestep = Movestep.GO_SCORE_SPECIMEN_1;
                    }
                    break;
                case GO_SCORE_SPECIMEN_1:
                    if (timer1.milliseconds() > 300) {
                        drive.followTrajectory(goScoreSpecimen);
                        currentPos = drive.getPoseEstimate();

                        grabSpecimen2 = drive.trajectoryBuilder(currentPos)
                                .splineToConstantHeading(new Vector2d(11.00, -49.00), Math.toRadians(-25.00)
                                        ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                                )
                                .splineToConstantHeading(new Vector2d(38.00, -64.00), Math.toRadians(-100.00)
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

                        if (timer1.milliseconds() > 100) {
                            movestep = Movestep.SCORING_SPECIMEN_1;
                        }
                    }
                    break;
                case SCORING_SPECIMEN_1:
                    robot.setArmPos(4, 1.0);
                    if (robot.arm.getCurrentPosition() >= 678) {
                        robot.setClawPos(0);
                        robot.setArmPos(5, 1.0);
                        if (robot.arm.getCurrentPosition() >= 700) {
                            if (timer1.milliseconds() > 500) {
                                movestep = Movestep.GRAB_SPECIMEN_2;
                            } else {
                                movestep = Movestep.PARKING;
                            }
                        }
                    }
                    break;
                case GRAB_SPECIMEN_2:
                    drive.followTrajectory(grabSpecimen2);
                    currentPos = drive.getPoseEstimate();

                    goScoreSpecimen2 = drive.trajectoryBuilder(currentPos)
                            .splineToConstantHeading(new Vector2d(-1.00, -28.00), Math.toRadians(40.00)
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
                        movestep = Movestep.GRABBING_SPECIMEN_2;
                    }
                    break;
                case GRABBING_SPECIMEN_2:
                    robot.setClawPos(1);
                    if (robot.claw.getPosition() >= 0.35) {
                        timer1.reset();
                        movestep = Movestep.GO_SCORE_SPECIMEN_2;
                    }
                    break;
                case GO_SCORE_SPECIMEN_2:
                    if (timer1.milliseconds() > 300) {
                        drive.followTrajectory(goScoreSpecimen2);
                        currentPos = drive.getPoseEstimate();

                        parking = drive.trajectorySequenceBuilder(currentPos)
                                .lineToConstantHeading(new Vector2d(46.00, -63.00)
                                        , SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                                )
                                .addTemporalMarker(0.00, 0.00, () -> {
                                    robot.setClawYaw(0);
                                })
                                .addTemporalMarker(0.30, 0.00, () -> {
                                    robot.setArmPos(7, 1.0);
                                })
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