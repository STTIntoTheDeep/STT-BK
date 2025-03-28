package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**TODO: fix link
 * This is the SQUIDFController class. This class is a variant of the
 * <a href="#PIDFController(CustomPIDFCoefficients)">PIDFController class</a>,
 * with the only difference being the proportional error gets square rooted, so it doesn't increase as quickly.
 * This is most useful in PIDF-controllers where the starting error can be unlimited, for example drive PID, to prevent slippage.
 *
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 0.5, 27/03/2025
 */
public class SQUIDFController extends PIDFController{

    /**
     * This creates a new PIDFController from a CustomPIDFCoefficients.
     *
     * @param set the coefficients to use.
     */
    public SQUIDFController(CustomPIDFCoefficients set) {
        super(set);
    }

    @Override
    public double runPIDF() {
        return Math.signum(error) * Math.sqrt(Math.abs(error)) * P() + errorDerivative * D() + errorIntegral * I() + F();
    }
}