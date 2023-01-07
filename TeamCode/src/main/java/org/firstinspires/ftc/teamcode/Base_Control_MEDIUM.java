package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Base_Control_MEDIUM {

    DcMotor motor_obj;               // the motor connected to the extending rail
    PID pid_obj;
    double target_position;             // Target position for the extending rail (in encoder ticks: +ve or -ve)
    double max;
    double min;
    double cmd;                         // Power command for the DC motor (Left)
    double p_gain;
    double i_gain;
    double d_gain;
    ElapsedTime et;                     // ElapsedTime object (only used during calibration)
    final double tolerance = 50;        // How close should we be within the target rail position before saying we're done
    Task_State run_state;               // This is used by the opmode to determine when this task has completed and proceed to the next task

    // CONSTRUCTOR
    public Base_Control_MEDIUM(DcMotor motor) {

        // Assign the motor connected to the bucket and initialize it
        motor_obj = motor;
        motor_obj.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create ElapsedTime object (only used during calibration)
        et = new ElapsedTime();

        // Create a new PID object to control the bucket
        pid_obj = new PID();
        pid_obj = new PID();
        run_state = Task_State.INIT;
    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTargetPosition(double target, double min_pwr, double max_pwr) {

        target_position = target;
        max = max_pwr;
        min = min_pwr;

        run_state = Task_State.RUN;
        pid_obj.Reset_PID();
    }

    // METHOD TO CALIBRATE THE BUCKET POSITION.
    // OVERRIDES THE PID BUCKET CONTROL'S OUTPUT TO ZERO TO ALLOW MANUAL ADJUSTMENT OF BUCKET POSITION
    // ONCE CALIBRATION IS DONE, THE BUCKET WILL RETURN TO ITS TARGET POSITION
    public void Calibrate() {
        et.reset();
        run_state = Task_State.CALIBRATE;
    }

    // METHOD TO OVERRIDE THE ARM MOTOR COMMAND TO ZERO FOR ONE SECOND
    // USEFUL FOR ALLOWING THE ARM TO HANG LOOSELY TO BE TUCKED INTO THE BASE
    public void Override() {
        et.reset();
        run_state = Task_State.OVERRIDE;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void RotatingBaseTask() {

        double clipped_cmd;

        // Always run this PID control when in RUN, DONE or READY mode
        if (run_state == Task_State.RUN || run_state == Task_State.DONE || run_state == Task_State.READY) {

            //correct numbers
//            cmd = pid_obj.PID_Control(target_position, 0.0022, 0.00002, 0.0037, motor_obj.getCurrentPosition() );

            //testing
            cmd = pid_obj.PID_Control(target_position, 0.0021, 0.00003, 0.0039, motor_obj.getCurrentPosition() );


            // Don't let the motor run too fast. Otherwise, it will overshoot
            clipped_cmd = Range.clip(cmd, min, max);
            motor_obj.setPower(clipped_cmd);

            if (run_state == Task_State.DONE) {
                run_state = Task_State.READY;
            }

            // If the rail is within range of the target position, treat the task as done so that the opmode can move on to
            // the next task in its list
            else if (run_state == Task_State.RUN) {

                if (target_position > motor_obj.getCurrentPosition()) {

                    if (motor_obj.getCurrentPosition() > (target_position - tolerance)) {
                        run_state = Task_State.DONE;
                    }
                }
                else if (target_position < motor_obj.getCurrentPosition()) {

                    if (motor_obj.getCurrentPosition() < (target_position + tolerance)) {
                        run_state = Task_State.DONE;
                    }
                }
                else {
                    if (run_state != Task_State.READY && motor_obj.getCurrentPosition() > (target_position - tolerance) &&
                            motor_obj.getCurrentPosition() < (target_position + tolerance)) {
                        run_state = Task_State.DONE;
                    }
                }
            }
        }
        else if (run_state == Task_State.CALIBRATE) {
            motor_obj.setPower(0);

            if (et.milliseconds() >= 1000) {

                // Reset the rail's DC motor encoder
                motor_obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Reset the PID object (otherwise, the PID will still have leftover
                // memory of what it previously did which may cause bad commands from carrying forward)
                pid_obj.Reset_PID();
                run_state = Task_State.DONE;
                motor_obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        else if (run_state == Task_State.OVERRIDE) {
            motor_obj.setPower(0);

            if (et.milliseconds() >= 1000) {

                // Reset the PID object (otherwise, the PID will still have leftover
                // memory of what it previously did which may cause bad commands from carrying forward)
                pid_obj.Reset_PID();
                run_state = Task_State.DONE;
            }
        }
    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return run_state;
    }

    public void GetDoneState() {
//        return Task_State.DONE;
        run_state = Task_State.DONE;
    }
}
