package frc.robot.util;

public class DistanceControl {
    double lastTime = -1.;
    double setPoint = 0.;
    double velocity = 0.;
    double distance = 0.;

    double lowerBound = 0.;
    double upperBound = 0.;

    public DistanceControl(double lowerBound, double upperBound) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    // return time in seconds
    double getTime() {
        return ((double)System.currentTimeMillis()) / 1000.;
    }

    public double mainloop(double velocity) {
        if (lastTime < 0) {
            lastTime = getTime();
            return distance;
        }

        double newLastTime = getTime();
        double deltaTime = newLastTime - this.lastTime;
        this.lastTime = newLastTime;

        distance += velocity * deltaTime;
        distance = Util.clamp(distance, lowerBound, upperBound);

        return distance;
    }

    public void resetWithValue(double value) {
        this.distance = value;
        this.lastTime = -1.;
    }

    public void reset() {
        this.resetWithValue(0);
    }
}
