package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        waitForStart();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);

            boolean lb = gamepad.left_bumper && !lastGamepad.left_bumper;
            boolean rb = gamepad.right_bumper && !lastGamepad.right_bumper;
            boolean lt = (gamepad.left_trigger > 0) && !(lastGamepad.left_trigger > 0);
            boolean rt = (gamepad.right_trigger > 0) && !(lastGamepad.right_trigger > 0);

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
                robot.setYawSpecimen();

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                if (rb) {
                    robot.clawClose();
                    robot.setArmPosition(ArmPosition.SPECIMEN_LIFT);
                    state = State.INTAKE_SPECIMEN;
                    continue;
                }

                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}

                if (!isSpecimen) state = State.READY_SAMPLE;
            } else

            if (state == State.INTAKE_SPECIMEN) {
                robot.setArmPosition(ArmPosition.SPECIMEN_LIFT);

                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}
                if (lb) state = State.READY_SPECIMEN;
                if (rb) state = State.SET;
            } else

            if (state == State.SET) {
                robot.clawClose();
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

            if (gamepad.touchpad) robot.resetYaw();
            robot.remote(gamepad);
            telemetry.addData("State", state.value + " | " + state);
            telemetry.update();
        }
    }

    enum State {
        INIT(0),
        READY_SAMPLE(1),
        READY_SPECIMEN(2),
        INTAKE_SAMPLE(3),
        INTAKE_SPECIMEN(4),
        SET(5),
        SCORE_1(6),
        SCORE_2(7);

        public final int value;
        State(int value) {this.value = value;}
        State() {this.value = 0;}
    }
}
