//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//public class SensorMethods {
//
//    BNO055IMU IMU;
//    Orientation orientation;
//    double globalangle;
//    double current_value;
//    double prev_value = 0;
//    double final_value;
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//    public SensorMethods(BNO055IMU imu, Orientation Orientation) {
//
//        IMU = imu;
//        orientation = Orientation;
//        //Configure IMU for GyroTurning
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        IMU.initialize(parameters);
//        globalangle = 0;
//    }
//
//    public double GyroContinuity() {
//
//        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        current_value = orientation.firstAngle;
//
//        final_value = current_value - prev_value;
//
//        if (final_value < -180)
//            final_value += 360;
//        else if (final_value > 180)
//            final_value -= 360;
//
//        globalangle += final_value;
//
//        prev_value = current_value;
//
//        return -globalangle;
//    }
//
//
//}
