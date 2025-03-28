package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**
 * This is the FilteredPIDFController class. This class handles the running of filtered filtered PIDFs. This
 * behaves very similarly to a regular filtered PIDF controller, but the derivative portion is filtered with
 * a low pass filter to reduce high frequency noise that could affect results.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/15/2024
 */
public class FilteredSQUIDFController extends PIDFController{
    private CustomFilteredPIDFCoefficients coefficients;
    private double previousDerivative;
    private double filteredDerivative;
    private double feedForwardInput;

    /**
     * This creates a new filtered PIDFController from a CustomFilteredPIDFCoefficients.
     *
     * @param set the coefficients to use.
     */
    public FilteredSQUIDFController(CustomFilteredPIDFCoefficients set) {
        setCoefficients(set);
        reset();
    }

    /**
     * This takes the current error and runs the filtered PIDF on it.
     *
     * @return this returns the value of the filtered PIDF from the current error.
     */
    @Override
    public double runPIDF() {
        return Math.signum(error) * Math.sqrt(Math.abs(error)) * P() * tuningVoltage / currentVoltage + filteredDerivative * D() * tuningVoltage / currentVoltage + errorIntegral * I() * tuningVoltage / currentVoltage + F() * tuningVoltage / currentVoltage;
    }

    @Override
    public void updatePosition(double update) {
        position = update;
        previousError = error;
        error = targetPosition - position;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        previousDerivative = filteredDerivative;
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
        filteredDerivative = T() * previousDerivative + (1 - T()) * errorDerivative;
    }

    @Override
    public void updateError(double error) {
        previousError = this.error;
        this.error = error;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        previousDerivative = errorDerivative;
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
        filteredDerivative = T() * previousDerivative + (1 - T()) * errorDerivative;
    }

    @Override
    public void updateFeedForwardInput(double input) {
        feedForwardInput = input;
    }

    @Override
    public void reset() {
        previousError = 0;
        error = 0;
        position = 0;
        targetPosition = 0;
        errorIntegral = 0;
        errorDerivative = 0;
        previousDerivative = 0;
        filteredDerivative = 0;
        previousUpdateTimeNano = System.nanoTime();
    }

    /**
     * This is used to set the coefficients of the filtered PIDF.
     *
     * @param set the coefficients that the filtered PIDF will use.
     */
    public void setCoefficients(CustomFilteredPIDFCoefficients set) {
        this.coefficients = set;
    }

    /**
     * This returns the filtered PIDF's current coefficients.
     *
     * @return this returns the current coefficients.
     */
    public CustomFilteredPIDFCoefficients getFilteredCoefficients() {
        return this.coefficients;
    }

    @Override
    public void setP(double set) {
        this.coefficients.P = set;
    }

    @Override
    public double P() {
        return this.coefficients.P;
    }

    @Override
    public void setI(double set) {
        this.coefficients.I = set;
    }

    @Override
    public double I() {
        return this.coefficients.I;
    }

    @Override
    public void setD(double set) {
        this.coefficients.D = set;
    }

    @Override
    public double D() {
        return this.coefficients.D;
    }

    /**
     * This sets the time constant (T) of the filtered PIDF only.
     *
     * @param set this sets the time constant.
     */
    public void setT(double set) {
        this.coefficients.T = set;
    }

    /**
     * This returns the time constant (T) of the filtered PIDF.
     *
     * @return this returns the time constant.
     */
    public double T() {
        return this.coefficients.T;
    }

    @Override
    public void setF(double set) {
        this.coefficients.F = set;
    }

    @Override
    public double F() {
        return this.coefficients.getCoefficient(feedForwardInput);
    }
}
