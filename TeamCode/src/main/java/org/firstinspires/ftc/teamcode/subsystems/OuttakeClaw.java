package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import org.firstinspires.ftc.teamcode.hardware;

public class OuttakeClaw extends Subsystem {
    // BOILERPLATE
    public static final OuttakeClaw INSTANCE = new OuttakeClaw();
    private OuttakeClaw() { }

    // USER CODE
    public Servo servo;

    public Command open() {
        double timer = Math.abs(hardware.servos.outtakeClaw.getLastPosition() - hardware.servoPositions.outtakeRelease.getPosition());
        return new SequentialGroup(
                new ServoToPosition(hardware.servos.outtakeClaw.servo, // SERVO TO MOVE
                        hardware.servoPositions.outtakeRelease.getPosition(), // POSITION TO MOVE TO
                        this),
                new Delay(timer)// IMPLEMENTED SUBSYSTEM
        );
    }

    public Command clear() {
        double timer = 2.0 * Math.abs(hardware.servos.outtakeClaw.getLastPosition() - hardware.servoPositions.outtakeClear.getPosition()) + 8.0;
        return new SequentialGroup(
                new ServoToPosition(hardware.servos.outtakeClaw.servo, // SERVO TO MOVE
                        hardware.servoPositions.outtakeClear.getPosition(), // POSITION TO MOVE TO
                        this),
                new Delay(timer)// IMPLEMENTED SUBSYSTEM
        );
    }

    public Command close() {
        double timer = Math.abs(hardware.servos.outtakeClaw.getLastPosition() - hardware.servoPositions.outtakeGrip.getPosition());
        return new SequentialGroup(
                new ServoToPosition(hardware.servos.outtakeClaw.servo, // SERVO TO MOVE
                        hardware.servoPositions.outtakeGrip.getPosition(), // POSITION TO MOVE TO
                        this), // IMPLEMENTED SUBSYSTEM
                new Delay(timer)
        );
    }

    @Override
    public void initialize() {
        hardware.servos.outtakeClaw.initServo(OpModeData.INSTANCE.getHardwareMap());
        servo = hardware.servos.outtakeClaw.servo;
    }
}
