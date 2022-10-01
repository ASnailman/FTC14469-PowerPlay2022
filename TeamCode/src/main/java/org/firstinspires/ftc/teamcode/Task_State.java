package org.firstinspires.ftc.teamcode;

public enum Task_State {

    // INIT = Task object is created
    // RUN = Task is running
    // DONE = Task is done but still running in the background
    // OVERRIDE = Ignore the results of the task and manually provide the result
    INIT (0), RUN (1), DONE (2), READY (3), CALIBRATE (4), OVERRIDE (5);

    int state;

    Task_State(int n) {
        state = n;
    }
}
