package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import org.firstinspires.ftc.teamcode.hardware;

public class IntakeClaw extends Subsystem {
    // BOILERPLATE
    public static final IntakeClaw INSTANCE = new IntakeClaw();
    private IntakeClaw() { }

    // USER CODE
    public Servo servo;

    public Command open() {
        return new SequentialGroup(
                new ServoToPosition(hardware.servos.intake.servo, // SERVO TO MOVE
                        hardware.servoPositions.intakeRelease.getPosition(), // POSITION TO MOVE TO
                        this),// IMPLEMENTED SUBSYSTEM
                new Delay(0.25)
        );
    }

    public Command close() {
        return new SequentialGroup(
                new ServoToPosition(hardware.servos.intake.servo, // SERVO TO MOVE
                        hardware.servoPositions.intakeGrip.getPosition(), // POSITION TO MOVE TO
                        this), // IMPLEMENTED SUBSYSTEM
                new Delay(0.25));
    }

    @Override
    public void initialize() {
        hardware.servos.intake.initServo(OpModeData.INSTANCE.getHardwareMap());
        servo = hardware.servos.intake.servo;
    }
}
