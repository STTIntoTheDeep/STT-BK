package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.firstinspires.ftc.teamcode.hardware;

public class Slides extends Subsystem {
    public static final Slides INSTANCE = new Slides();
    private Slides() { }
    public MotorEx motor;

    public PIDFController controller = new PIDFController(0.02, 0.0, 0.0, new StaticFeedforward(0.0));

    public Command setPower(double power) {
        return new SetPower(motor, power, this);
    }

    public Command setPowerWithinLimits(double power) {
        if (hardware.tooLong()) {
            return setPower(Math.min(power, -0.3));
        } else return setPower(power);
    }

    public Command toPosition(int position) {
        return new RunToPosition(motor, position, controller, this);
    }

    public Command toCM(double cm) {
        return new RunToPosition(motor, (int) cm * hardware.ticksPerCM, controller, this);
    }

    @Override
    public void initialize() {
        controller.setSetPointTolerance(10);
        hardware.motors.intake.initMotor(OpModeData.INSTANCE.getHardwareMap());
        motor = new MotorEx(hardware.motors.intake.dcMotorEx);
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor, controller, this);
    }
}
