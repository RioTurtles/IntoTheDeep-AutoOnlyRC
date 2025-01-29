package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class PRRHAutonRedObservation extends LinearOpMode {

    public enum Movestep {
        INITIAL_MOVEMENT,
        SCORE_SPECIMEN_READY,
        SCORING_SPECIMEN,
        PARKING_MOVE1,
        PARKING_MOVE2
    }

    ElapsedTime timer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, drivetrain::remote);
        Movestep movestep = Movestep.INITIAL_MOVEMENT;

        robot.init2(hardwareMap);
        robot.setGripperPos(1);

        roadrunner.setPIDCoefficients(Axis.X, 0.5, 0, 0);
        roadrunner.setPIDCoefficients(Axis.Y, 0.5, 0, 0);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.2, 0, 0);

        waitForStart();

        roadrunner.setPoseEstimate(33.00, -63.00, 90.00); //TODO: change starting position

        while (opModeIsActive()) {

            switch (movestep) {
                case INITIAL_MOVEMENT:
                    if ((!roadrunner.isInPosition()) && robot.arm.getCurrentPosition() < 3880) {timer1.reset();}
                    roadrunner.setTarget(6.00, -29.00, 90.00);
                    robot.setArmPos(3, 1.0);
                    if (timer1.milliseconds() > 200) {
                        robot.setGripperYaw(1);
                        movestep = Movestep.SCORE_SPECIMEN_READY;
                    }
                    break;
                case SCORE_SPECIMEN_READY:
                    if (robot.arm.getCurrentPosition() < 4480){timer1.reset();}
                    robot.setArmPos(4, 1.0);
                    if (timer1.milliseconds() > 300) {
                        movestep = Movestep.SCORING_SPECIMEN;
                    }
                    break;
                case SCORING_SPECIMEN:
                    if (robot.arm.getCurrentPosition() < 4480){timer1.reset();}
                    if (timer1.milliseconds() > 200) {
                        robot.setGripperPos(0);
                    }
                    if (timer1.milliseconds() > 300) {
                        robot.setArmPos(6, 1.0);
                    }
                    if (timer1.milliseconds() > 700) {
                        movestep = Movestep.PARKING_MOVE1; //TODO
                    }
                    break;
                case PARKING_MOVE1:
                    if (!roadrunner.isInPosition()) {timer1.reset();}
                    roadrunner.setTarget(30.00, -48.00, 90.00);
                    if (timer1.milliseconds() > 200) {
                        movestep = Movestep.PARKING_MOVE2;
                    }
                    break;
                case PARKING_MOVE2:
                        roadrunner.setTarget(60.00, -63.00, 90.00);
                    movestep = Movestep.PARKING_MOVE2;
                    break;
            }

            telemetry.addData("Movestep", movestep);

            roadrunner.update();
            telemetry.update();
        }
    }
}
