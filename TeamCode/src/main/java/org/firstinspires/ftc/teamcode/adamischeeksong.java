package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "adamischeeksong", group = "MecanumDrive")
public class adamischeeksong extends LinearOpMode {

    DcMotor rotatingBase;

    public void runOpMode() {

        rotatingBase = hardwareMap.get(DcMotor.class, "rotatingBase");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                rotatingBase.setPower(1);
            }

            if (gamepad1.x) {
                rotatingBase.setPower(1);
            }
        }

    }

}
