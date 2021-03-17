package frc.robot.commands.autonomous.routines;

public class GetSOTMTestPowers {

    private final double maxPower;
    private final double h;
    private final double aValue;

    // Uses a parabola to accelerate left and right motor powers
    // Max powers are height of vertex, time is width
    public GetSOTMTestPowers(double maxPower, double timeToFinish) {
        this.maxPower = maxPower;
        this.h = timeToFinish / 2;
        this.aValue = - maxPower / Math.pow(h, 2);
    }

    public double getVariablePower(double timePassed) {
        return this.aValue * Math.pow(timePassed - h, 2) + this.maxPower;
    }


}