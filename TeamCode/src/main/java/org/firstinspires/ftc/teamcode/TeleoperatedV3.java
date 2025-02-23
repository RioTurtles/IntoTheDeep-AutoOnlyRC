package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        boolean isSpecimen = true;

        robot.clawClosed = false;

        waitForStart();

        while (opModeIsActive()) {

            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);

            boolean lb = gamepad.left_bumper && !lastGamepad.left_bumper;
            boolean rb = gamepad.right_bumper && !lastGamepad.right_bumper;
            boolean lt = (gamepad.left_trigger > 0) && !(lastGamepad.left_trigger > 0);
            boolean rt = (gamepad.right_trigger > 0) && !(lastGamepad.right_trigger > 0);

            if (gamepad.options) {
                robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.arm.setPower(1.0);
                state = State.FAILSAFE;
            }

            if (state == State.INIT) {
                if (rb) state = State.READY_SAMPLE;
            } else

            if (state == State.READY_SAMPLE) {
                robot.setArmPosition(ArmPosition.READY);
                robot.setYawSample();

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                if (rb) {robot.clawOpen(); state = State.INTAKE_SAMPLE;}
                if (lb) {robot.setArmPosition(ArmPosition.INTAKE); state = State.GRABBED_SAMPLE;}
                if (isSpecimen) {robot.clawOpen(); state = State.READY_SPECIMEN;}
            } else

            if (state == State.GRABBED_SAMPLE) {
                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                if (rb) {state = State.RAISE_BACK_SAMPLE;}
                if (lb) {robot.clawOpen(); state = State.INTAKE_SAMPLE;}
                if (isSpecimen) {robot.clawOpen(); state = State.READY_SPECIMEN;}
            } else

            if (state == State.INTAKE_SAMPLE) {
                robot.setArmPosition(ArmPosition.INTAKE);

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                // TODO: fix this thing for some reason claw controls don't work.
                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}
                if (lb) {state = State.READY_SAMPLE;}
                if (rb) {robot.clawClose(); state = State.GRABBED_SAMPLE;}
                if (isSpecimen) {robot.clawOpen(); state = State.READY_SPECIMEN;}
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

            if (state == State.RAISE_BACK_SAMPLE) {
                robot.setArmPosition(ArmPosition.READY);

                if (rb) {state = State.PLACED_SAMPLE_1;}
                if (lb) {robot.setArmPosition(ArmPosition.INTAKE); state = State.GRABBED_SAMPLE;}

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                if (rt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}
            } else

            if (state == State.PLACED_SAMPLE_1) {
                robot.setArmPosition(ArmPosition.INTAKE);
                robot.clawOpen();
                if (robot.arm.getCurrentPosition() >= 1170 && robot.claw.getPosition() <= 0.05) {
                    state = State.PLACED_SAMPLE_2;
                }

            } else

            if (state == State.PLACED_SAMPLE_2) {
                robot.setArmPosition(ArmPosition.READY);

                if (rb) {state = State.INTAKE_SAMPLE;}

                if (gamepad.square || operator.square) isSpecimen = false;
                if (gamepad.circle || operator.circle) isSpecimen = true;

                if (isSpecimen) {robot.clawOpen(); state = State.READY_SPECIMEN;}

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
                robot.setArmPosition(ArmPosition.SPECIMEN_LIFT);
                if (rb) state = State.SCORE_1;
                if (lb) state = State.INTAKE_SPECIMEN;
            } else

            if (state == State.SCORE_1) {
                robot.setArmPosition(ArmPosition.SCORE);

                if (lt) {if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();}

                if (lb) state = State.SET;
                if (rb) {robot.clawOpen(); state = State.SCORE_2;}
            } else

            if (state == State.SCORE_2) {
                robot.setArmPosition(ArmPosition.CONFIRM);

                if (lb) state = State.SCORE_1;
                if (rb) {
                    if (isSpecimen) state = State.READY_SPECIMEN; else state = State.READY_SAMPLE;
                }
            } else

            if (state == State.FAILSAFE) {
                if (operator.dpad_up) {
                    robot.arm.setPower(1); //TODO
                } else
                if (operator.dpad_down) {
                    robot.arm.setPower(-1); //TODO
                } else {
                    robot.arm.setPower(0);
                }
            } else

            if (state == State.RIGGING) {
                if (operator.dpad_up) {
                    robot.raiseRidging();
                } else
                if (operator.dpad_down) {
                    robot.lowerRidging();
                } else {
                    robot.noRigging();
                }
                if (operator.triangle && !lastOperator.triangle) {state = State.INIT;}
            }

            if (operator.touchpad && !lastOperator.touchpad) {
                if (state == state.FAILSAFE) {
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    state = State.INIT;
                } else {
                    robot.setClawPos(0);
                    robot.setClawYaw(0);

                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    state = State.FAILSAFE;
                }
            }
            if (operator.triangle && !lastOperator.triangle) {robot.setArmPosition(ArmPosition.RIGGING); state = State.RIGGING;}

            if (gamepad.touchpad) robot.resetYaw();
            robot.remote(gamepad);
            telemetry.addData("State", state.value + " | " + state);
            telemetry.addData("Gyro",robot.getYaw());
            telemetry.addData("armPos", robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }

    enum State {
        INIT(0),
        READY_SAMPLE(1),
        GRABBED_SAMPLE(2),
        RAISE_BACK_SAMPLE(3),
        PLACED_SAMPLE_1(4),
        PLACED_SAMPLE_2(5),
        READY_SPECIMEN(6),
        GRABBED_SPECIMEN(7),
        INTAKE_SAMPLE(8),
        INTAKE_SPECIMEN(9),
        SET(10),
        SCORE_1(11),
        SCORE_2(12),
        FAILSAFE(13),
        RIGGING(14);
        public final int value;
        State(int value) {this.value = value;}
        State() {this.value = 0;}
    }
}