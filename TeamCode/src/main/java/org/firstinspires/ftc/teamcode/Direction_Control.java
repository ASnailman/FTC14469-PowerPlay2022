package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Direction_Control {

    DcMotor frontRight, frontLeft, backRight, backLeft;
    BNO055IMU IMU;
    Orientation orientation;
    double globalangle = 0;
    double current_value;
    double prev_value = 0;
    double final_value;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    boolean turnright = false;
    boolean turnleft = false;
    double pwr;
    double Angle;
    double AngleTolerance;
    Task_State run_state;

    public Direction_Control(BNO055IMU imu, DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright) {
        IMU = imu;
        frontRight = frontright;
        frontLeft = frontleft;
        backRight = backright;
        backLeft = backleft;
        run_state = Task_State.INIT;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
    }

    public void SetTargetDirection(double angle, double power) {
        pwr = power;
        Angle = angle;
        run_state = Task_State.RUN;
    }

    public void Override() {
        run_state = Task_State.OVERRIDE;
    }

    public void GyroTask() {

        //Continuously turn to the direction chosen with the specified amount of power
        if (run_state == Task_State.RUN) {

            if (((Angle + 2) > GyroContinuity()) || ((Angle - 2) > GyroContinuity()) || turnright) {

                turnright = true;

                if (GyroContinuity() < Angle) {
                    MotorTurn(-pwr, pwr, -pwr, pwr);
                } else {
                    SetMotorPower(0);
                    turnright = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

            } else if (((Angle + 2) < GyroContinuity()) || ((Angle - 2) < GyroContinuity()) || turnleft) {

                turnleft = true;

                if (GyroContinuity() > Angle) {
                    MotorTurn(pwr, -pwr, pwr, -pwr);
                } else {
                    SetMotorPower(0);
                    turnleft = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
        //Force Stop
        else if (run_state == Task_State.OVERRIDE) {
            SetMotorPower(0);
            pwr = 0;
            run_state = Task_State.DONE;
        }
    }

    public double GyroTask_TeleOp() {

        double command = 0;

        //Continuously turn to the direction chosen with the specified amount of power
        if (run_state == Task_State.RUN) {

            if (((Angle + 2) > GyroContinuity()) || ((Angle - 2) > GyroContinuity()) || turnright) {

                turnright = true;

                if (GyroContinuity() < Angle) {
                    command = pwr;
                } else {
                    command = 0;
                    turnright = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

            } else if (((Angle + 2) < GyroContinuity()) || ((Angle - 2) < GyroContinuity()) || turnleft) {

                turnleft = true;

                if (GyroContinuity() > Angle) {
                    command = -pwr;
                } else {
                    command = 0;
                    turnleft = false;

//                while (frontRight.getCurrentPosition() != 0) {
//                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
        //Force Stop
        else if (run_state == Task_State.OVERRIDE) {
            command = 0;
            pwr = 0;
            run_state = Task_State.DONE;
        }

        return command;
    }

    public Task_State GetTaskState() {
        return run_state;
    }

    public double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;
    }

    public void MotorTurn(double FR, double FL, double BR, double BL) {
        frontRight.setPower(FR);
        frontLeft.setPower(FL);
        backRight.setPower(BR);
        backLeft.setPower(BL);
    }

    public void SetMotorPower(double x) {
        frontLeft.setPower(x);
        frontRight.setPower(x);
        backLeft.setPower(x);
        backRight.setPower(x);
    }

}
