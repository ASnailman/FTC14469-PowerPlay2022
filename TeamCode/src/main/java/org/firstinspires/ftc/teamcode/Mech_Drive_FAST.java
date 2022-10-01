package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mech_Drive_FAST {

    DcMotor FrontLeft, FrontRight, BackLeft, BackRight;     // DC motors for each of the Mecanum wheels
    double flpower, frpower, blpower, brpower;              // Power command for each of the DC motors

    ElapsedTime ET = new ElapsedTime();
    Telemetry telemetry;
    PID pid;
    double targetdistance;
    double strafingangle;
    double headingangle = 0;
    double targetpower;
    double steeringoutput;
    Task_State state;
    int encoderselect; //0 is frontright, 1 is backright

    double finalpower;

    // CONSTRUCTOR
    public Mech_Drive_FAST(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, MoveDirection Direction, Telemetry Telemetry) {

        // Assign the motor connected to the bucket and initialize it
        FrontRight = fr;
        FrontLeft = fl;
        BackRight = br;
        BackLeft = bl;

        SetDirection(Direction);
        pid = new PID();
        telemetry = Telemetry;

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        state = Task_State.INIT;

    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTargets(double StrafingAngle, double TargetDistance, double TargetPower, int EncoderSelect) {


        encoderselect = EncoderSelect;
        strafingangle = StrafingAngle;

        targetdistance = TargetDistance;

        targetpower = TargetPower;
        state = Task_State.RUN;

    }

    public void Override() {
        state = Task_State.OVERRIDE;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void Task (double gyro_Z_reading) {

        double power_x_old, power_x_new;
        double power_y_old, power_y_new;
        double denominator;
        double encoder;
        double radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule
        double power;

        if (encoderselect == 0) {
            encoder = FrontRight.getCurrentPosition();
        }
        else {
            encoder = BackRight.getCurrentPosition();
        }

        // Always run this PID control when in RUN
        if (state == Task_State.RUN) {

            if (encoderselect == 0) {
                power = ((targetdistance - FrontRight.getCurrentPosition()) / targetdistance) * targetpower;
            } else {
                power = ((targetdistance - BackRight.getCurrentPosition()) / targetdistance) * targetpower;
            }
            power = 1.2*power;
            finalpower = Range.clip(power, 0.35, targetpower);

            power_x_old = 0;                // make x_old 0 to make the degrees start at the front of the robot
            power_y_old = finalpower;

            power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
            power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
            steeringoutput = pid.PID_Control(headingangle, 0.03, 0.0001, 0, gyro_Z_reading);

            if (encoder < 0) {
                encoder = -encoder;
            }

            if (encoder < targetdistance) {

                //if ((radians <= Math.toRadians(90) && radians >= Math.toRadians(0)) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
                //encoder = FrontLeft.getCurrentPosition();
                //} else {
                //encoder = BackLeft.getCurrentPosition();
                //}
                //encoder = FrontRight.getCurrentPosition();
                ///if (encoder < 0) {
                   // encoder = -encoder;
                //}

                denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                flpower = (power_y_new + 1.1 * power_x_new + steeringoutput) / denominator;
                blpower = (power_y_new - 1.1 * power_x_new + steeringoutput) / denominator;
                frpower = (power_y_new - 1.1 * power_x_new - steeringoutput) / denominator;
                brpower = (power_y_new + 1.1 * power_x_new - steeringoutput) / denominator;

                FrontLeft.setPower(flpower);
                FrontRight.setPower(frpower);
                BackLeft.setPower(blpower);
                BackRight.setPower(brpower);
            }
            else {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);

                FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                state = Task_State.DONE;
            }
        }
        else if (state == Task_State.DONE){
            state = Task_State.READY;
        }
        else if (state == Task_State.OVERRIDE) {

        }

        //telemetry.addData("ActualDistance", encoder);
        //telemetry.addData("steering output", steeringoutput);
        //telemetry.addData("Steering", steeringoutput);
        //telemetry.addData("DirectionZ", gyro_Z_reading);
        //telemetry.addData("Position", FrontRight.getCurrentPosition());
        telemetry.update();

    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return state;
    }

    private void SetDirection (MoveDirection direction) {

        if (direction == MoveDirection.FORWARD) {
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == MoveDirection.REVERSE) {
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }
}
