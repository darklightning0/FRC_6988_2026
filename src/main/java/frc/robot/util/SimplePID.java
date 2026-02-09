package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

public class SimplePID {
    double kP = 0.08, kF = 0, kFF = 0, setPoint = 0, maxAbs = 0;
    String id = "";

    public SimplePID(String id, double maxAbs) {
        this.maxAbs = maxAbs;
        this.id = id;
        refreshValues();
        setF(this.kF);
        setFF(this.kFF);
        setP(this.kP);
    }

    public void refreshValues() {
        this.kP = getPrefDouble("kP", 0.1);
        this.kF = getPrefDouble("kF", 0.0);
        this.kFF = getPrefDouble("kFF", 0.0);
    }

    double getPrefDouble(String key, double backup) {
        return Preferences.getDouble("pid_" + id + "_" + key, backup);
    }
    void setPrefDouble(String key, double value) {
        Preferences.setDouble("pid_" + id + "_" + key, value);
    }

    public void setP(double value) { kP = value; setPrefDouble("kP", value); }
    public void setF(double value) { kF = value; setPrefDouble("kF", value); }
    public void setFF(double value) { kFF = value; setPrefDouble("kFF", value); }
    public void setSetPoint(double value) { setPoint = value; }
    public double getP() { return kP; }
    public double getF() { return kF; }
    public double getFF() { return kFF; }
    public double getSetPoint() { return setPoint; }

    public double getError(double current) { return setPoint - current; }

    public double getSignal(double current) {
        double error = getError(current);

        /*double signal = kP * error;
        if (Math.abs(setPoint) < 0.02) signal += Math.signum(setPoint) * kF;
        if (Math.abs(signal) > maxAbs) signal = Math.signum(signal) * maxAbs;*/
        
        // ignore kF
        double p = kP * error;
        double ff = setPoint * kFF;
        double signal = p + ff;
        return signal;
    }
}