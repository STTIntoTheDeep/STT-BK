package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import org.firstinspires.ftc.teamcode.hardware;

public class Wrist extends Subsystem {
    public static final Wrist INSTANCE = new Wrist();
    private Wrist() { }

    public Servo servo;
    double halfRotationInPos = hardware.servoPositions.wristSpecimenCamera.getPosition() - hardware.servoPositions.wristSampleCamera.getPosition(), lastWristPos;

    public Command cameraToFront() {
        return new ServoToPosition(servo, hardware.servoPositions.wristSampleCamera.getPosition(),this);
    }

    public Command toTransfer() {
        return new ServoToPosition(servo, hardware.servoPositions.wristTransfer.getPosition(), this);
    }

    public Command cameraToBack() {
        return new ServoToPosition(servo, hardware.servoPositions.wristSpecimenCamera.getPosition(), this);
    }

    public Command toAngle(double theta) {
        lastWristPos = hardware.servos.wrist.getLastPosition();
        theta = (theta * halfRotationInPos) / Math.PI;
        theta = theta + 0.5*halfRotationInPos + hardware.servoPositions.wristSampleCamera.getPosition();
        while (theta < 0.0) theta += halfRotationInPos;
        if (theta < 1.0 - halfRotationInPos) {
            if (Math.abs(theta - lastWristPos) > Math.abs(theta + halfRotationInPos - lastWristPos)) {
                theta += halfRotationInPos;
            }
        }

        while (theta > 1.0) theta -= halfRotationInPos;
        if (theta > 0.0 + halfRotationInPos) {
            if (Math.abs(theta - lastWristPos) > Math.abs(theta - halfRotationInPos - lastWristPos)) {
                theta -= halfRotationInPos;
            }
        }
        return new ServoToPosition(servo, theta, this).thenWait(1.2 * Math.abs(theta - lastWristPos));
    }

    @Override
    public void initialize() {
        hardware.servos.wrist.initServo(OpModeData.INSTANCE.getHardwareMap());
        servo = hardware.servos.wrist.servo;
    }
}
