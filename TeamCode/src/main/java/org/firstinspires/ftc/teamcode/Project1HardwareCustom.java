package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Project1HardwareCustom extends Project1Hardware {
    RemoteMethod remote;

    public Project1HardwareCustom(HardwareMap hardwareMap) {
        super(hardwareMap);
        super.init(hardwareMap);
        remote = (new MecanumDrive(this))::remote;
        clawClosed = false;
    }

    public void setArmEncoder(int value) {
        arm.setTargetPosition(value);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void clawOpen() {setClawPos(0); clawClosed = false;}
    public void clawClose() {setClawPos(1); clawClosed = true;}

    public enum ArmPosition {
        INIT,
        SPECIMEN_LIFT,
        READY,
        INTAKE,
        SET,
        SCORE,
        CONFIRM,
        RESET,
        RIGGING
    }

    public void setArmPosition(ArmPosition position) {
        switch (position) {
            case INIT: setArmPos(0, 1); break;
            case SPECIMEN_LIFT: setArmPos(6, 1); break;
            case READY: setArmPos(1, 1); break;
            case INTAKE: setArmPos(2, 1); break;
            case SET: setArmPos(3, 1); break;
            case SCORE: setArmPos(4, 1); break;
            case CONFIRM: setArmPos(5, 1); break;
            case RESET: setArmPos(7, 1); break;
            case RIGGING: setArmPos(8, 1); break;
        }
    }

    public void remote(Gamepad gamepad) {
        remote.call(gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, Math.toRadians(getYaw()));
    }

    public void setYawSample() {setClawYaw(0);}
    public void setYawSpecimen() {setClawYaw(1);}

    public double getYaw() {return imu.getRobotYawPitchRollAngles().getYaw();}
    public void resetYaw() {imu.resetYaw();}

    public interface RemoteMethod {void call(double y, double x, double r, double h);}
}
