package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PID {

    double P;
    double I;
    double D;
    double I_limit = 0;
    double error;
    double sum_error;
    double prev_error;
    double Output;

    public double PID_Control(double Target, double kp, double ki, double kd, double Actual) {

        error = Target - Actual;
        sum_error = sum_error + error;
        P = kp * error;
        I = ki * sum_error;
        D = kd * (error - prev_error);

        // Limit the integral term if desired
        // If the integral term is clipped, we should reset the PID to free up sum_error immediately
        // Otherwise, 'I' will continue to get clipped every time which means the PID control is no longer working
        if (I_limit != 0) {
            I = Range.clip(I, -I_limit, I_limit);

            if (Math.abs(I) == Math.abs(I_limit)) {
                Reset_PID();
            }
        }
        Output = P + I + D;
        prev_error = error;

        return Output;
    }

    // Set a limit for I to prevent integral runaway
    // A value of zero means no limit
    // If the Actual overshoots the Target badly or the control no longer has control,
    // it might be caused by the integral term running away
    // To prevent this from occurring, it would be a good idea to limit the integral term
    public void Limit_Integral(double limit) {
        I_limit = limit;
    }

    // Reset the PID to forget whatever it has calculated in the past
    // Useful when a control calibration is performed
    // Also useful to prevent integral term runaway
    public void Reset_PID() {
        sum_error = 0;
        prev_error = 0;
    }

}
