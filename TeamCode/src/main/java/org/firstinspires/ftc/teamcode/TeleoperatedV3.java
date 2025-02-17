package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Project1HardwareCustom.ArmPosition;


@TeleOp
public class TeleoperatedV3 extends LinearOpMode {
    @Override
    public void runOpMode() {

        Project1HardwareCustom robot = new Project1HardwareCustom(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        ElapsedTime timer1 = new ElapsedTime();
        boolean isSpecimen = false;

        robot.clawClosed = false;

        waitForStart();

        while (opModeIsActive()) {

            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);

            boolean lb = gamepad.left_bumper && !lastGamepad.left_bumper;
            boolean rb = gamepad.right_bumper && !lastGamepad.right_bumper;
            boolean lt = (gamepad.left_trigger > 0) && !(lastGamepad.left_trigger > 0);
            boolean rt = (gamepad.right_trigger > 0) && !(lastGamepad.right_trigger > 0);

            boolean dpadU = gamepad.dpad_up && !lastGamepad.dpad_up;
            boolean dpadD = gamepad.dpad_down && !lastGamepad.dpad_down;

            /*if (gamepad.options && lastGamepad.options) {
                robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.arm.setPower(1.0);
            } else {
                robot.arm.setPower(0.0);
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setDirection(DcMotorSimple.Direction.FORWARD);
                state = State.FAILSAFE;
            }*/

            if (state == State.INIT) {
                if (rb) state = State.READY_SAMPLE;
            } else

            if (state == State.READY_SAMPLE) {
                robot.setArmPosition(ArmPosition.READY);
                robot.setYawSample();

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;
                if (rb) {robot.clawOpen(); state = State.INTAKE_SAMPLE;}
                if (isSpecimen) {robot.clawOpen(); state = State.READY_SPECIMEN;}
            } else

            if (state == State.INTAKE_SAMPLE) {
                robot.setArmPosition(ArmPosition.INTAKE);

                // TODO: fix this thing for some reason claw controls don't work.
                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}
                if (lb) {state = State.READY_SAMPLE;}
                if (rb) {robot.clawClose(); state = State.READY_SAMPLE;}
            } else

            if (state == State.READY_SPECIMEN) {
                robot.setArmPosition(ArmPosition.INIT);
                robot.setYawSample();

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                if (rb) {
                    robot.clawClose();
                    state = State.GRABBED_SPECIMEN;
                    continue;
                }

                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}

                if (!isSpecimen) state = State.READY_SAMPLE;
            } else

            if (state == State.GRABBED_SPECIMEN) {
                robot.setArmPosition(ArmPosition.INIT);
                if (rb) {
                    state = State.INTAKE_SPECIMEN;
                }
                if (lb) {
                    robot.clawOpen();
                    state = State.READY_SPECIMEN;
                }
            } else

            if (state == State.INTAKE_SPECIMEN) {
                robot.setYawSample();
                robot.setArmPosition(ArmPosition.SPECIMEN_LIFT);

                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}
                if (lb) state = State.GRABBED_SPECIMEN;
                if (rb) state = State.SET;
            } else

            if (state == State.SET) {
                robot.clawClose();
                robot.setYawSpecimen();
                robot.setArmPosition(ArmPosition.SET);
                if (rb) state = State.SCORE_1;
                if (lb) state = State.INTAKE_SPECIMEN;
            } else

            if (state == State.SCORE_1) {
                robot.setArmPosition(ArmPosition.SCORE);

                if (robot.armInPosition() && !gamepad.left_bumper) {
                    robot.clawOpen();
                    state = State.SCORE_2;
                }

                if (lt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}

                if (lb) state = State.SET;
                if (rb) {timer1.reset(); state = State.SCORE_2;}
            } else

            if (state == State.SCORE_2) {
                if (timer1.milliseconds() > 300) robot.setArmPosition(ArmPosition.CONFIRM);

                if (lb) state = State.SCORE_1;
                if (rb) {
                    if (isSpecimen) state = State.READY_SPECIMEN; else state = State.READY_SAMPLE;
                }
            }

            if (state == State.FAILSAFE) {
                robot.setClawPos(0);
                robot.setArmPos(0, 1.0);
                robot.setClawYaw(0);
                state = State.INIT;
            }

            if (dpadU && robot.arm.getCurrentPosition() >= 3000) { robot.raiseRidging(1.0); }
            if (dpadD && robot.arm.getCurrentPosition() >= 3000) { robot.lowerRidging(1.0); }

            if (gamepad.touchpad) robot.resetYaw();
            robot.remote(gamepad);
            telemetry.addData("State", state.value + " | " + state);
            telemetry.addData("Gyro",robot.getYaw());
            telemetry.update();
        }
    }

    enum State {
        INIT(0),
        READY_SAMPLE(1),
        READY_SPECIMEN(2),
        GRABBED_SPECIMEN(3),
        INTAKE_SAMPLE(4),
        INTAKE_SPECIMEN(5),
        SET(6),
        SCORE_1(7),
        SCORE_2(8),
        FAILSAFE(9);

        public final int value;
        State(int value) {this.value = value;}
        State() {this.value = 0;}
    }
}
