package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "SprintTeleop2",  group = "MecanumDrive")
public class SprintTeleop2 extends LinearOpMode {

    //Control Hub Orientation
    byte AXIS_MAP_CONFIG_BYTE = 0x06; //rotates control hub 90 degrees around y axis by swapping x and z axis
    byte AXIS_MAP_SIGN_BYTE = 0x01; //Negates the remapped z-axis

    //Motors
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor RailLeft;
    static DcMotor RailRight;
    static DcMotor ExtendingRail;
    static DcMotor RotatingBase;
    //    static CRServo LeftClaw;
//    static CRServo RightClaw;
    static CRServo Claw;

    static VoltageSensor voltageSensor;

    //Sensors
    BNO055IMU IMU;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;
    //    Rail_Control RailControl;
    Rail_ControlV2 RailControlV2;
    ExtendingRail_Control ExtendingRailControl;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    ElapsedTime ET = new ElapsedTime();
    int leftHighPickup = 0;
    int rightHighPickup = 0;
    int resetSequence = 0;
    int targetJunction = 0;
    int baseManualReset = 0;
    double movement;
    boolean lowJunctionResetMode;
    boolean PowerSetting = true;
    boolean ClawSetting = false;
    boolean coneStackMode = false;
    boolean groundJunctionMode = false;
    int basePosition;
    int readVoltOnce = 0;

    double l;
//    double assist_gain = 0.03;
//    double assist_offset = 0.1;    // compensation - robot drifting to the right when base is 0 deg

    //Gyrocontinuity Variables
    double current_value;
    double prev_value = 0;
    double final_value;

    //Button Pressing Variables
    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;
    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_right_already_pressed2 = false;
    boolean button_dpad_left_already_pressed2 = false;
    boolean button_dpad_up_already_pressed2 = false;
    boolean button_dpad_down_already_pressed2 = false;
    boolean button_left_trigger_already_pressed = false;
    boolean button_right_trigger_already_pressed = false;
    boolean button_left_trigger_already_pressed2 = false;
    boolean button_right_trigger_already_pressed2 = false;
    boolean double_trigger_already_pressed = false;

    public void runOpMode() {

        //Initialize the motors and sensors
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
//        LeftClaw = hardwareMap.get(CRServo.class, "leftClaw");
//        RightClaw = hardwareMap.get(CRServo.class, "rightClaw");
        Claw = hardwareMap.get(CRServo.class, "Claw");
        RailLeft = hardwareMap.get(DcMotor.class, "RailLeft");
        RailRight = hardwareMap.get(DcMotor.class, "RailRight");
        ExtendingRail = hardwareMap.get(DcMotor.class, "ExtendingRail");
        RotatingBase = hardwareMap.get(DcMotor.class, "RotatingBase");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

//        voltageSensor = hardwareMap.get(VoltageSensor.class, "Motor Controller 1");

        //Configure the control hub orientation
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        //Attachment Presets
        AttachmentMotorPresets();

        //Claw Presets
        Claw.setDirection(DcMotorSimple.Direction.FORWARD);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
//        RailControl = new Rail_Control(RailLeft, RailRight);
        RailControlV2 = new Rail_ControlV2(RailLeft, RailRight);
        ExtendingRailControl = new ExtendingRail_Control(ExtendingRail);
        //        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RotatingBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        ET.reset();

        while (opModeIsActive()) {

            /*****************************************************************
             * Up Dpad (G1) - Set Low Power Mode/High Power Mode for driving
             *****************************************************************/

            if (PowerSetting) {
                movement = 0.8;
            } else {
                movement = 0.5;
            }

//            if (!button_bumper_right_already_pressed) {
//                if (gamepad1.right_bumper) {
//                    if (!PowerSetting) {
//                        PowerSetting = true;
//                    } else {
//                        PowerSetting = false;
//                    }
//                    button_bumper_right_already_pressed = true;
//                }
//            } else {
//                if (!gamepad1.right_bumper) {
//                    button_bumper_right_already_pressed = false;
//                }
//            }

            if (gamepad1.right_bumper) {
                PowerSetting = true;
            } else {
                PowerSetting = false;
            }

            /*****************************************************************
             * Left/Right Analog Sticks (G1) - Movement
             *****************************************************************/

            double y = -gamepad1.left_stick_y * movement;
            double x = gamepad1.left_stick_x * movement;
            double rx = gamepad1.right_stick_x * movement;

//            if (y < 0.1 && y > -0.1 || x < 0.1 && x > -0.1) {
//                l = 0;
//            }
//            else if ((y > 0.45 && y < 0.55) && (x > 0.45 && x < 0.55)) {
//                l = 0;
//            }
//            else if ((y > 0.45 && y < 0.55) && (x < -0.45 && x > -0.55)) {
//                l = 0;
//            }
//            else if ((y < -0.45 && y > -0.55) && (x < -0.45 && x > -0.55)) {
//                l = 0;
//            }
//            else if ((y < -0.45 && y > -0.55) && (x > 0.45 && x < 0.55)) {
//                l = 0;
//            }
//            else {
//                l = assist_gain * (RotatingBase.getCurrentPosition() / 1020f);
//                        //- assist_offset;
//            }
//
//            if (y < 0) {
//                l = l * -1;
//            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx + l) / denominator;
            double BLPower = (y - x + rx + l) / denominator;
            double FRPower = (y - x - rx - l) / denominator;
            double BRPower = (y + x - rx - l) / denominator;

            FrontLeft.setPower(FLPower);
            BackLeft.setPower(BLPower);
            FrontRight.setPower(FRPower);
            BackRight.setPower(BRPower);

            /*****************************************************************
             * Dpad Up (G2) - Open/Close Claw
             *****************************************************************/

            if (!button_dpad_up_already_pressed2) {
                if (gamepad2.dpad_up) {
                    if (!ClawSetting) {
                        ClawSetting = true;
                    } else {
                        ClawSetting = false;
                    }
                    button_dpad_up_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_up) {
                    button_dpad_up_already_pressed2 = false;
                }
            }

            if (!ClawSetting) {
                Claw.setPower(-1);
            } else {
                Claw.setPower(1);
            }

            switch (targetJunction) {
                case 1:
                    ET.reset();
                    targetJunction++;
                    break;
                case 2:
                    if (ET.milliseconds() > 100) {
//                        SetExtendingPosition(0);
                        RailControlV2.SetTargetPosition(1280, -1, 1);
                        ClawSetting = false;
                        Claw.setPower(-1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;
                case 3:
                    if (ET.milliseconds() > 500) {
                        SetBasePosition(0);
                        targetJunction++;
                    }
                    break;

                case 4:
                    break;

                case 5:
                    ET.reset();
                    targetJunction++;
                    break;

                case 6:
                    if (ET.milliseconds() > 800) {
//                        SetExtendingPosition(100);
                        RailControlV2.SetTargetPosition(2150, -1, 1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;
                case 7:
                    if (ET.milliseconds() > 500) {
//                            SetBasePosition(0);
                        targetJunction++;
                    }
                    break;

                case 8:
                    break;

                case 9:
                    ET.reset();
                    targetJunction++;
                    break;

                case 10:
                    if (ET.milliseconds() > 800) {
//                        SetExtendingPosition(100);
                        RailControlV2.SetTargetPosition(2950, -1, 1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;

                case 11:
                    if (ET.milliseconds() > 500) {
//                            SetBasePosition(0);
                        targetJunction++;
                    }
                    break;

                case 12:
                    break;

                case 13:
                    ET.reset();
                    targetJunction++;
                    break;

                case 14:
                    if (ET.milliseconds() > 800) {
                        RailControlV2.SetTargetPosition(215, -1, 1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;

                case 15:
                    if (ET.milliseconds() > 500) {
                        SetBasePosition(0);
                        targetJunction++;
                    }
                    break;

                case 16:
                    break;

                case 17:
                    ET.reset();
                    ClawSetting = true;
                    Claw.setPower(1);
                    targetJunction++;
                    break;
                case 18:
                    if (ET.milliseconds() > 800) {
//                        SetExtendingPosition(100);
                        RailControlV2.SetTargetPosition(1280, -1, 1);

                        ET.reset();
                        targetJunction++;
                    }
                    break;
                case 19:
                    if (ET.milliseconds() > 500) {
                        SetBasePosition(0);
                        targetJunction++;
                    }
                    break;

                case 20:
                    break;

                default:
                    break;

            }

            if (!button_dpad_left_already_pressed2) {
                if (gamepad2.dpad_left) {
                    if (!coneStackMode) {
                        coneStackMode = true;
                    } else {
                        coneStackMode = false;
                    }
                    button_dpad_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_left) {
                    button_dpad_left_already_pressed2 = false;
                }
            }

            //If cone stack mode is false, then buttons a, b, and y will correspond to the height of the junctions
            //If ground junction mode is true, then button a will not close claw when moving to low junction position
            if (!coneStackMode) {

                /*****************************************************************
                 * Button A (G2) : Set Rail height to place on the low junction
                 *****************************************************************/

                if (groundJunctionMode) {
                    if (!button_a_already_pressed2) {
                        if (gamepad2.a) {
                            //code for low junction when pressed
                            ClawSetting = true;
                            groundJunctionMode = false;
//                            RightClaw.setPosition(0);
//                            LeftClaw.setPosition(0);
                            Claw.setPower(1);
                            targetJunction = 1;
                            button_a_already_pressed2 = true;
                        }
                    } else {
                        if (!gamepad2.a) {
                            button_a_already_pressed2 = false;
                        }
                    }
                } else if (lowJunctionResetMode) {
                    if (!button_a_already_pressed2) {
                        if (gamepad2.a) {
                            //code for low junction when pressed
//                            ClawSetting = false;
                            lowJunctionResetMode = false;
//                            RightClaw.setPosition(1);
//                            LeftClaw.setPosition(1);
//                            RightClaw.setPower(-1);
//                            LeftClaw.setPower(-1);
                            targetJunction = 1;
                            button_a_already_pressed2 = true;
                        }
                    } else {
                        if (!gamepad2.a) {
                            button_a_already_pressed2 = false;
                        }
                    }
                } else {
                    if (!button_a_already_pressed2) {
                        if (gamepad2.a) {
                            //code for low junction when pressed
                            ClawSetting = true;
//                            RightClaw.setPosition(1);
//                            LeftClaw.setPosition(1);
                            Claw.setPower(1);
                            targetJunction = 17;
                            button_a_already_pressed2 = true;
                        }
                    } else {
                        if (!gamepad2.a) {
                            button_a_already_pressed2 = false;
                        }
                    }
                }

                /*****************************************************************
                 * Button B (G2) : Set Rail height to place on the medium junction
                 *****************************************************************/

                if (!button_b_already_pressed2) {
                    if (gamepad2.b) {
                        //code for medium junction when pressed
                        ClawSetting = true;
                        lowJunctionResetMode = true;
//                        RightClaw.setPosition(1);
//                        LeftClaw.setPosition(1);
                        Claw.setPower(1);
                        targetJunction = 5;
                        button_b_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.b) {
                        button_b_already_pressed2 = false;
                    }
                }

                /*****************************************************************
                 * Button Y (G2) : Set Rail height to place on the high junction
                 *****************************************************************/

                if (!button_y_already_pressed2) {
                    if (gamepad2.y) {
                        //code for low junction when pressed
                        ClawSetting = true;
                        lowJunctionResetMode = true;
//                        RightClaw.setPosition(1);
//                        LeftClaw.setPosition(1);
                        Claw.setPower(1);
                        targetJunction = 9;
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }

                /*****************************************************************
                 * Button X (G2) : Reset Rail and Base from any height to original position
                 *****************************************************************/

                if (!button_x_already_pressed2) {
                    if (gamepad2.x) {
                        //code for releasing cone and resetting base
                        ClawSetting = false;
                        coneStackMode = false;
                        groundJunctionMode = false;
                        lowJunctionResetMode = false;
                        Claw.setPower(-1);
                        RailControlV2.SetTargetPosition(0, -1, 1);
//                        SetExtendingPosition(0);
                        button_x_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.x) {
                        button_x_already_pressed2 = false;
                    }
                }

                //if cone stack mode is true, buttons a, b, and y will correspond to the height of the cone stack
            } else {

                if (!button_x_already_pressed2) {
                    if (gamepad2.x) {
                        //code for 4th cone on stack
                        ClawSetting = false;
                        coneStackMode = false;
                        Claw.setPower(-1);
                        RailControlV2.SetTargetPosition(139, -1, 1);
                        SetBasePosition(0);
                        button_x_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.x) {
                        button_x_already_pressed2 = false;
                    }
                }

                if (!button_a_already_pressed2) {
                    if (gamepad2.a) {
                        //code for 3rd highest cone on stack
                        ClawSetting = false;
                        coneStackMode = false;
                        Claw.setPower(-1);
                        RailControlV2.SetTargetPosition(246, -1, 1);
                        SetBasePosition(0);
                        button_a_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.a) {
                        button_a_already_pressed2 = false;
                    }
                }

                if (!button_b_already_pressed2) {
                    if (gamepad2.b) {
                        //code for 2nd highest cone on stack
                        ClawSetting = false;
                        coneStackMode = false;
                        Claw.setPower(-1);
                        RailControlV2.SetTargetPosition(371, -1, 1);
                        SetBasePosition(0);
                        button_b_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.b) {
                        button_b_already_pressed2 = false;
                    }
                }

                if (!button_y_already_pressed2) {
                    if (gamepad2.y) {
                        //code for highest cone on stack
                        ClawSetting = false;
                        coneStackMode = false;
                        Claw.setPower(-1);
                        RailControlV2.SetTargetPosition(479, -1, 1);
                        SetBasePosition(0);
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }

            }

            ///////////////////////////////////////////////////////////////////////////

            //Lower Rail In the Direction it is facing
            if (!button_dpad_right_already_pressed2) {
                if (gamepad2.dpad_right) {
                    //code for releasing cone and resetting base
                    ClawSetting = true;
                    coneStackMode = false;
                    groundJunctionMode = false;
//                    RightClaw.setPosition(0);
//                    LeftClaw.setPosition(0);
                    Claw.setPower(1);
                    RailControlV2.SetTargetPosition(0, -1, 1);
                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Dpad Down (G2) : Set Rail For Ground Junction
             *****************************************************************/

            if (!button_dpad_down_already_pressed2) {
                if (gamepad2.dpad_down) {
                    //code for releasing cone and resetting base
                    ClawSetting = true;
                    coneStackMode = false;
                    groundJunctionMode = true;
//                    RightClaw.setPosition(1);
//                    LeftClaw.setPosition(1);
                    Claw.setPower(1);
                    targetJunction = 13;
                    button_dpad_down_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_down) {
                    button_dpad_down_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Bumper Right (G2) : Rotate 360 base 90 degrees
             *****************************************************************/

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {
                    RotatingBase.setTargetPosition(-1035);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Bumper Left (G2) : Rotate 360 base to -90
             *****************************************************************/

            if (!button_bumper_left_already_pressed2) {
                if (gamepad2.left_bumper) {
                    RotatingBase.setTargetPosition(1035);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button A (G1) : Extend Rail
             *****************************************************************/

            if (!button_a_already_pressed) {
                if (gamepad1.a) {
//                    ExtendingRailControl.SetTargetPosition(400, -0.6, 0.6);
                    ExtendingRail.setTargetPosition(610);
                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ExtendingRail.setPower(0.6);
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed = false;
                }
            }

            if (!button_x_already_pressed) {
                if (gamepad1.x) {
//                    ExtendingRailControl.SetTargetPosition(0, -0.01, 0.01);
                    ExtendingRail.setTargetPosition(0);
                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ExtendingRail.setPower(0.6);
                    button_x_already_pressed = true;
                }
            } else {
                if (!gamepad1.x) {
                    button_x_already_pressed = false;
                }
            }

            /******************************************
             * Trigger (G2) Manual Calibration For Rotating Base
             ******************************************/

            if (button_left_trigger_already_pressed2 == false) {
                if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {

//                    basePosition = basePosition + 25;
//                    baseManualReset = 2;
                    RotatingBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RotatingBase.setPower(0.2);

                    button_left_trigger_already_pressed2 = true;
                }
            } else {
                if (gamepad2.left_trigger == 0) {
//                    baseManualReset = 0;
                    RotatingBase.setPower(0);
                    RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RotatingBase.setTargetPosition(0);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    button_left_trigger_already_pressed2 = false;
                }
            }

            if (button_right_trigger_already_pressed2 == false) {
                if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {

//                    basePosition = basePosition - 25;
//                    baseManualReset = 2;
                    RotatingBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RotatingBase.setPower(-0.2);

                    button_right_trigger_already_pressed2 = true;
                }
            } else {
                if (gamepad2.right_trigger == 0) {
//                    baseManualReset = 0;
                    RotatingBase.setPower(0);
                    RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RotatingBase.setTargetPosition(0);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    button_right_trigger_already_pressed2 = false;
                }
            }

            /******************************************
             * Trigger (G1) Manual Calibration For Viper Slides
             ******************************************/

            if (button_left_trigger_already_pressed == false) {
                if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger <= 0.2) {

                    RailRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailRight.setPower(-0.4);
                    RailLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailLeft.setPower(0.4);

                    button_left_trigger_already_pressed = true;
                }
            } else {
                if (gamepad1.left_trigger <= 0.2) {
                    RailRight.setPower(0);
                    RailLeft.setPower(0);
//                    RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    RailRight.setTargetPosition(0);
//                    RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    RailLeft.setTargetPosition(0);
//                    RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    button_left_trigger_already_pressed = false;
                }
            }

            if (button_right_trigger_already_pressed == false) {
                if (gamepad1.right_trigger > 0.4 && gamepad1.left_trigger <= 0.2) {

                    RailRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailRight.setPower(0.4);
                    RailLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailLeft.setPower(-0.4);

                    button_right_trigger_already_pressed = true;
                }
            } else {
                if (gamepad1.right_trigger <= 0.2) {
                    RailRight.setPower(0);
                    RailLeft.setPower(0);
//                    RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    RailRight.setTargetPosition(0);
//                    RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    RailLeft.setTargetPosition(0);
//                    RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    button_right_trigger_already_pressed = false;
                }
            }

            RailControlV2.RailTask();
//            ExtendingRailControl.ExtendingRailTask();
//            telemetry.addData("Voltage", voltageSensor.getVoltage());
            if (readVoltOnce == 0) {
                telemetry.addData("voltage", "%.1f volts", new Func<Double>() { @Override public Double value() { return getBatteryVoltage(); } });
                readVoltOnce++;
            }
            telemetry.addData("LeftRail Power", RailLeft.getPower());
            telemetry.addData("LeftRail Encoder", RailLeft.getCurrentPosition());
            telemetry.addData("RightRail Power", RailRight.getPower());
            telemetry.addData("RightRail Encoder", RailRight.getCurrentPosition());
            telemetry.addData("Extending Encoder", ExtendingRail.getCurrentPosition());
            telemetry.update();
        }
    }


    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private double GyroContinuity() {

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

    public void AttachmentMotorPresets() {
        RailLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RailLeft.setTargetPosition(0);
//        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RailRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RailRight.setTargetPosition(0);
//        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RotatingBase.setDirection(DcMotorSimple.Direction.FORWARD);
        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotatingBase.setTargetPosition(0);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtendingRail.setDirection(DcMotorSimple.Direction.FORWARD);
        ExtendingRail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendingRail.setTargetPosition(0);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        ExtendingRail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetRailPosition(int position) {
        RailRight.setTargetPosition(position);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailRight.setPower(1);
        RailLeft.setTargetPosition(-position);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailLeft.setPower(1);
    }

    public void SetBasePosition(int position) {
        RotatingBase.setTargetPosition(position);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotatingBase.setPower(0.8);
    }

    public void SetExtendingPosition(int extendingPos) {
        ExtendingRail.setTargetPosition(extendingPos);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendingRail.setPower(0.6);
    }
}
