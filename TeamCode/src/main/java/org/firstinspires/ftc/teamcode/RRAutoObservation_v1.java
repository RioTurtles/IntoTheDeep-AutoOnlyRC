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
public class RRAutoObservation_v1 extends LinearOpMode {
    public enum Movestep {
        INITIAL_MOVEMENT,
        SCORING_SPECIMEN,
        PREPARE_PUSH,
        PUSH_SAMPLES_1,
        PUSH_SAMPLES_2,
        PUSH_SAMPLES_3,
        PARKING,
        END;
    }
    Movestep movestep = Movestep.INITIAL_MOVEMENT;
    ElapsedTime timer1 = new ElapsedTime();
    TrajectorySequence pushSamples1, pushSamples2, pushSamples3, parking;

    double maxVel, maxAccel;

    @Override
    public void runOpMode() throws InterruptedException {

        maxVel = 45;
        maxAccel = 30;

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.initAuto(hardwareMap);

        Pose2d currentPos = drive.getPoseEstimate();
        Pose2d startPose = new Pose2d(9.00, -63.00, Math.toRadians(90.00));

        Trajectory initialMovement = drive.trajectoryBuilder(startPose)
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
                        ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                )
                .addTemporalMarker(0.00, 0.00, () -> {
                    robot.setArmPos(0, 1.0);
                    robot.setClawYaw(0);
                })
                .splineToConstantHeading(new Vector2d(35.00, -27.00), Math.toRadians(100.00)
                        ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                )
                .splineToConstantHeading(new Vector2d(45.00, -10.00), Math.toRadians(40.00)
                        ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                )
                .addTemporalMarker(1.00, 0.00, () -> {
                    timer1.reset();
                })
                .build();

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
                            .lineToConstantHeading(new Vector2d(49.00, -56.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    robot.setArmPos(4, 1.0);
                    if (robot.armInPosition()) {
                        robot.setClawPos(0);
                        if (robot.arm.getCurrentPosition() >= 675) {
                            robot.setArmPos(5, 1.0);
                            movestep = Movestep.PREPARE_PUSH;
                        }
                    }
                    break;
                case PREPARE_PUSH:

                    pushSamples2 = drive.trajectorySequenceBuilder(pushSamples1.end())
                            .splineToConstantHeading(new Vector2d(56.00, -10.00), Math.toRadians(40.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .lineToConstantHeading(new Vector2d(56.00, -56.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .addTemporalMarker(1.00, 0.00, () -> {
                                timer1.reset();
                            })
                            .build();

                    drive.followTrajectory(preparePush);

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES_1;
                    }
                    break;
                case PUSH_SAMPLES_1:
                    drive.followTrajectorySequence(pushSamples1);

                    pushSamples3 = drive.trajectorySequenceBuilder(pushSamples2.end())
                            .splineToConstantHeading(new Vector2d(63.30, -10.00), Math.toRadians(60.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .lineToConstantHeading(new Vector2d(63.30, -56.00)
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
                    drive.followTrajectorySequence(pushSamples2);

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PUSH_SAMPLES_3;
                    }
                    break;
                case PUSH_SAMPLES_3:
                    drive.followTrajectorySequence(pushSamples3);

                    parking = drive.trajectorySequenceBuilder(pushSamples3.end())
                            .lineToConstantHeading(new Vector2d(57, -58.00)
                                    ,SampleMecanumDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                            )
                            .build();

                    if (timer1.milliseconds() > 100) { //TODO: check milliseconds
                        movestep = Movestep.PARKING;
                    }
                    break;
                case PARKING:
                    drive.followTrajectorySequence(parking);
                    movestep = Movestep.END;
                    break;
                case END:
                    movestep = Movestep.END;
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