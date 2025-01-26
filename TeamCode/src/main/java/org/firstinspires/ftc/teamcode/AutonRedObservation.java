package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutonRedObservation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware(hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);

        // Variables TODO: TUNE PID
        double xTarget = 0;
        double yTarget = 0;
        double headingTarget = 0;
        double botHeading;
        double left_x;
        double left_y;
        double rot_x;
        double lx;
        double ly;
        double denominator;
        double error1;
        double lastError1=0;
        double integral1=0;
        double kp1 = 0.16;
        double ki1 =0.015;
        ki1=0;

        double kd1 =0.3;
        double error2;
        double lastError2=0;
        double integral2=0;
        double kp2 = 0.05;
        double ki2 =0;
        double kd2 =0;
        double error3;
        double lastError3 = 0;
        double integral3=0;
        double kp3 = 1;
        double ki3=0.001;
        double kd3 = 1;



        int moveStep = 1;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime auton30 = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialised");
        telemetry.update();
        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(270))); // TODO
        robot.setGripperPos(1);

        waitForStart();

        kp2=kp1;
        ki2=ki1;
        kd2=kd1;

        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            // heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            // Initial movement
            if (moveStep == 1) {
                robot.setArmPos(2, 1.0);
                robot.setGripperPos(1);
                xTarget = -10; // TODO
                yTarget = 30; // TODO
                headingTarget = 270; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 300) { // TODO
                    moveStep = 2;

                    timer1.reset();
                }
            }

            // Score initial specimen
            if (moveStep == 2) {
                robot.setArmPos(4, 1.0);

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 300) { // TODO
                    robot.setArmPos(5, 1.0);

                    timer1.reset();
                    if (timer1.milliseconds() > 100) { // TODO
                        robot.setGripperPos(0);

                        moveStep = 3;
                        timer1.reset();
                    }
                }
            }

            // Move to 1st sample position
            if (moveStep == 3) {
                robot.setArmPos(2, 1.0);
                xTarget = -49; // TODO
                yTarget = 36; // TODO
                headingTarget = 270; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 300) { // TODO
                    moveStep = 4;

                    timer1.reset();
                }
            }

            // Grab 1st sample
            if (moveStep == 4) {
                robot.setGripperPos(1);

                if (timer1.milliseconds() > 400) { // TODO
                        moveStep = 5;

                        timer1.reset();
                }
            }

            // Move to observation zone + Release sample (1)
            if (moveStep == 5) {

                xTarget = -54; // TODO
                yTarget = 54; // TODO
                headingTarget = 115; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if(timer1.milliseconds() > 300){ // TODO
                    robot.setGripperPos(0);
                    moveStep = 6;

                    timer1.reset();
                }
            }

            // Move + Grab 2nd sample
            if (moveStep == 6) {

                xTarget = -60; // TODO
                yTarget = 36; // TODO
                headingTarget = 270; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if(timer1.milliseconds() > 100) { // TODO
                    robot.setGripperPos(1);

                    moveStep = 7;
                    timer1.reset();
                }
            }

            // Move to observation zone + Release sample (2)
            if (moveStep == 7) {

                xTarget = -66; // TODO
                yTarget = 54; // TODO
                headingTarget = 115; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if(timer1.milliseconds() > 300){ // TODO
                    robot.setGripperPos(0);

                    moveStep = 8;
                    timer1.reset();
                }
            }

            // Move + Grab 3rd sample
            if (moveStep == 8) {

                xTarget = -70; // TODO
                yTarget = 36; // TODO
                headingTarget = 270; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if(timer1.milliseconds() > 100) { // TODO
                    robot.setGripperPos(1);

                    moveStep = 9;
                    timer1.reset();
                }
            }

            // Move to observation zone + Release sample (3)
            if (moveStep == 9) {

                xTarget = -60; // TODO
                yTarget = 54; // TODO
                headingTarget = 115; // TODO

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if(timer1.milliseconds() > 300){ // TODO
                    robot.setGripperPos(0);
                    moveStep = 10;

                    timer1.reset();
                }

            }

            // Park in observation zone
            if(moveStep == 10){
                robot.setArmPos(0, 1.0);

                if (timer1.milliseconds() > 100) { // TODO
                    yTarget = 66;
                }

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 400){
                    moveStep = 10;
                    timer1.reset();
                }
            }

            botHeading = poseEstimate.getHeading();
            error1 = (xTarget - poseEstimate.getX());
            left_y = ((error1) * kp1 + integral1*ki1 +((error1 - lastError1) * kd1));
            error2 = (poseEstimate.getY() - yTarget);
            left_x = -((error2) * kp2 + integral1*ki2 +((error2 - lastError2) * kd2));
            error3 = (Math.toRadians(headingTarget) - botHeading);
            if (error3 > Math.PI) {
                error3 -= 2 * Math.PI;
            }
            if (error3 < -Math.PI) {
                error3 += 2 * Math.PI;
            }


            rot_x = -((error3) * kp3 + integral3*ki3 +((error3 - lastError3) * kd3));
            if (Math.abs(error3) < Math.toRadians(1)) {
                rot_x = 0;
            }
            integral1 += error1;
            integral2 += error2;
            integral3 +=error3;
            lastError1 = error1;
            lastError2 = error2;
            lastError3 = error3;
            //rot_x = (headingTarget - poseEstimate.getHeading()) * -kp3;
            if((Math.abs(left_x)>0.5||Math.abs(left_y)>0.5)&&(left_y!=0)&&left_x!=0) {


                if (Math.abs(left_y) > Math.abs(left_x)) {
                    left_x = left_x * Math.abs((0.5 / left_y));
                    if(left_y>0){
                        left_y=0.5;
                    }else {
                        left_y=-0.5;
                    }
                }else{
                    left_y = left_y * Math.abs((0.5 / left_x));
                    if(left_x>0){
                        left_x=0.5;
                    }else {
                        left_x=-0.5;
                    }
                }
            }

            drivetrain.remote(-left_y,left_x,-rot_x,poseEstimate.getHeading());

            drive.update();

            telemetry.addData("ly", left_y);
            telemetry.addData("lx", left_x);
            telemetry.addData("rot", rot_x);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Move",moveStep);

            telemetry.addData("stage", moveStep);

            telemetry.update();
        }
    }

}