package org.firstinspires.ftc.teamcode.common;

import android.os.Build;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Optional;

import androidx.annotation.RequiresApi;

class PIDVAController {

    private PIDCoeffs pid;
    private double kV = 0;
    private double kA = 0;
    private double kStatic = 0;


    private ElapsedTime clock = new ElapsedTime();
    private double errorSum = 0.0;
    private double lastUpdateTimestamp = Double.NaN;

    private boolean inputBounded = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;

    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetAcceleration = 0.0;




    private double lastError = 0.0;

    @RequiresApi(api = Build.VERSION_CODES.N)//umm
    double update(double measuredPosition, Optional<Double> measuredVelocity) {
        double currentTimestamp = clock.seconds();
        double error = getPositionError(measuredPosition);

        if(Double.isNaN(lastUpdateTimestamp)) {
            lastError = error;
            lastUpdateTimestamp = currentTimestamp;
            return 0.0;
        } else {
            double dt = currentTimestamp - lastUpdateTimestamp;
            errorSum += 0.5 * (error + lastError)*dt;
            double errorDeriv = (error - lastError)/dt;

            lastError = error;
            lastUpdateTimestamp = currentTimestamp;

            double baseOutput;
            if(measuredVelocity.isPresent()) {
                baseOutput = pid.kP * error + pid.kI * errorSum +
                        pid.kD * (targetVelocity - measuredVelocity.get()) +
                        kV * targetVelocity + kA * targetAcceleration + kF(measuredPosition, measuredVelocity);
            } else {
                baseOutput = pid.kP * error + pid.kI * errorSum +
                        pid.kD * (errorDeriv) +
                        kV * targetVelocity + kA * targetAcceleration + kF(measuredPosition, measuredVelocity);
            }
            double output = Math.abs(baseOutput) < 0.000001 ? 0 : baseOutput;

            return outputBounded ? Math.max(minOutput, Math.min(output, maxOutput)) : output;
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)//umm
    public double kF(double measuredPosition, Optional<Double> measuredVelocity) {
        return 0;
    }

    private double getPositionError(double measuredPosition) {
        return 0;//TODO
    }


}
