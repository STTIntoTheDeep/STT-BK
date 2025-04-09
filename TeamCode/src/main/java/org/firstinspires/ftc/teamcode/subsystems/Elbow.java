package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import org.firstinspires.ftc.teamcode.hardware;

public class Elbow extends Subsystem {
    // BOILERPLATE
    public static final Elbow INSTANCE = new Elbow();

    // USER CODE
    public Servo left, right;
    double lastLeftPos, lastRightPos, elbowAngle;

    /**
     *
     * @return
     */
    public Command cameraDown() {return setElbow(hardware.servoPositions.cameraDown.getDifferential());}
    public Command cameraWide() {return setElbow(hardware.servoPositions.cameraWide.getDifferential());}
    public Command toTransfer() {return setElbow(hardware.servoPositions.elbowTransfer.getDifferential());}
    public Command toDown() {return setElbow(hardware.servoPositions.elbowDrop.getDifferential());}
    public Command toggle(){
        return new BlockingConditionalCommand(
                () -> left.getPosition() == hardware.servoPositions.elbowDrop.getDifferential()[0] - hardware.servoPositions.elbowDrop.getDifferential()[1],
                this::toTransfer,
                this::toDown
        );
    }

    /**
     * Centimeters perpendicular to the slides, negative is to the left
     * TODO: documentation
     * @param cm
     */
    public Command yDistance(double cm) {
        elbowAngle = Math.acos(cm/hardware.armLength);
        return setElbow(hardware.servoPositions.elbowRight.getDifferential()[0] + (hardware.servoPositions.elbowLeft.getDifferential()[0] - hardware.servoPositions.elbowRight.getDifferential()[0]) * elbowAngle / Math.PI,
                hardware.servoPositions.elbowRight.getDifferential()[1] + (hardware.servoPositions.elbowLeft.getDifferential()[1] - hardware.servoPositions.elbowRight.getDifferential()[1]) * elbowAngle / Math.PI);
    }


    public Command setElbow(double[] positions) {return setElbow(positions[0], positions[1]);}

    public Command setElbow(double yaw, double pitch) {
        lastLeftPos = hardware.servos.elbowLeft.getLastPosition();
        lastRightPos = hardware.servos.elbowRight.getLastPosition();
        return new SequentialGroup(
                new ParallelGroup(
                        new ServoToPosition(left, yaw - pitch, this),
                        new ServoToPosition(right, yaw + pitch, this)
                ),
                new Delay(0.5*Math.max(Math.abs(hardware.servos.elbowLeft.getLastPosition() - lastLeftPos), Math.abs(hardware.servos.elbowLeft.getLastPosition() - lastRightPos)))
        );
    }

    @Override
    public void initialize() {
        hardware.servos.elbowLeft.initServo(OpModeData.INSTANCE.getHardwareMap());
        hardware.servos.elbowRight.initServo(OpModeData.INSTANCE.getHardwareMap());
        left = hardware.servos.elbowLeft.servo;
        right = hardware.servos.elbowRight.servo;
    }
}
