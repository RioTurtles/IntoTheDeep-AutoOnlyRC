package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp v3.0")
public class TeleoperatedV3 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        robot.init(hardwareMap);
        State state = State.INIT;
        ElapsedTime timer1 = new ElapsedTime();
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        waitForStart();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            switch (state) {
                case INIT:
//                    robot.setArmPos();
                    break;
            }
        }
    }

    enum State {
        INIT,
        READY
    }
}
