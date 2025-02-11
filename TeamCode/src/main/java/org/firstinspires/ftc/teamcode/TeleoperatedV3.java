package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp v3.0")
public class TeleoperatedV3 extends LinearOpMode {
    State state;

    @Override
    public void runOpMode() {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        state = State.INIT;
        robot.init(hardwareMap);
        ElapsedTime timer1 = new ElapsedTime();
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        boolean suppressGenericStateControls;
        boolean nextState, lastState, lTrigger, rTrigger;
        boolean isSpecimen = false;
        int scoreStep = 0;

        waitForStart();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);
            nextState = gamepad.right_bumper && !lastGamepad.right_bumper;
            lastState = gamepad.left_bumper && !lastGamepad.left_bumper;
            lTrigger = gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0);
            rTrigger = gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0);
            suppressGenericStateControls = false;

            switch (state) {
                case INIT:
                    scoreStep = 0;
                    robot.setArmPos(0, 1);
                    suppressGenericStateControls = true;
                    if (nextState) {
                        if (isSpecimen) state = State.INTAKE_SPECIMEN; else state = State.INTAKE;
                    }
                    break;
                case READY:
                    robot.setArmPos(1, 1);
                    break;
                case INTAKE:
                    robot.setClawYaw(0);
                    robot.setArmPos(2, 1);
                    if (rTrigger) {
                        if (robot.clawClosed) robot.setClawPos(0); else robot.setClawPos(1);
                    }

                    if (nextState) {
                        suppressGenericStateControls = true;
                        robot.setClawPos(1);
                        scoreStep = 0;
                        state = State.SET;
                    }
                    break;
                case INTAKE_SPECIMEN:
                    robot.setClawYaw(1);
                    robot.setArmPos(2, 1);
                    suppressGenericStateControls = true;
                    if (lastState) state = State.INIT;
                    if (nextState) nextState();
                    break;
                case SET:
                    robot.setArmPos(3, 1);
                    if (rTrigger) scoreStep++; else if (lTrigger) scoreStep--;
                    scoreStep = Range.clip(scoreStep, 0, 2);

                    switch (scoreStep) {
                        case 0: robot.setArmPos(3, 1); break;
                        case 1:
                            robot.setArmPos(4, 1);
                            if (robot.armInPosition()) robot.setClawPos(0);
                            break;
                        case 2:
                            robot.setArmPos(5, 1);
                    }

                    suppressGenericStateControls = true;
                    if (nextState) state = State.INIT;
                    if (lastState) lastState();
                    break;
            }

            if (gamepad.square || operator.square) isSpecimen = false;
            if (gamepad.circle || operator.circle) isSpecimen = true;

            if (!suppressGenericStateControls) {
                if (nextState) nextState(); else if (lastState) lastState();
            }
        }
    }

    public enum State {
        INIT,
        READY,
        INTAKE,
        INTAKE_SPECIMEN,
        SET
    }

    public void changeStateBy(int dX) {
        int r = state.ordinal() + dX;
        if (0 <= r && r < State.values().length) state = State.values()[r];
    }

    public void nextState() {changeStateBy(1);}
    public void lastState() {changeStateBy(-1);}
}
