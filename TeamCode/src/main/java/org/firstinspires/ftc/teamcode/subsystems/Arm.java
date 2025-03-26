package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.firstinspires.ftc.teamcode.hardware;

public class Arm extends Subsystem {
    public static final Arm INSTANCE = new Arm();
    private Arm() {}
    // USER CODE
    public MotorEx motor;

    public PIDFController controller = new PIDFController(0.004, 0.0, 0.0, new StaticFeedforward(0), 20);
//new ArmFeedforward(0.3, ticksToAngle -> 750/Math.PI)
    public Command toDown() {
        return new ParallelGroup(
                new WaitUntil(() -> true),
                toPosition(0)
        );
    }

    public Command toHigh() {
        return toPosition(420);
    }

    public Command toPosition(double position) {
        return new RunToPosition(motor,
                position,
                controller,
                this);
    }

    public Command holdPosition() {return new HoldPosition(motor, controller, this);}

    public Command setPower(double power) {
        return new SetPower(motor, power, this);
    }

    @Override
    public void initialize() {
        hardware.motors.outtake.initMotor(OpModeData.INSTANCE.getHardwareMap());
        motor = new MotorEx(hardware.motors.outtake.dcMotorEx);
    }

    @Override
    public void periodic() {
        OpModeData.telemetry.addData("pos", motor.getCurrentPosition());
        OpModeData.telemetry.update();
    }
}
