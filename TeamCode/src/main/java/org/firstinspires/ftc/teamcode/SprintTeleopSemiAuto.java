package org.firstinspires.ftc.teamcode;

import android.graphics.ColorSpace;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "SprintTeleopSemiAuto",  group = "MecanumDrive")
public class SprintTeleopSemiAuto extends LinearOpMode {

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
    static ColorSensor rightColorSensor;
    static ColorSensor leftColorSensor;

    static RevBlinkinLedDriver LightStrip;

    static VoltageSensor voltageSensor;

    //Sensors
    BNO055IMU IMU;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;
    //    Rail_Control RailControl;
    Rail_ControlV2 RailControlV2;
    ExtendingRail_Control ExtendingRailControl;
    Direction_Control DirectionControl;
    Base_Control BaseControl;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    ElapsedTime ET = new ElapsedTime();
    ElapsedTime HJBET = new ElapsedTime(); //High Junction Blitz Elapsed Time
    int leftHighPickup = 0;
    int rightHighPickup = 0;
    int resetSequence = 0;
    int targetJunction = 0;
    int baseManualReset = 0;
    int extendoSeq = 0;
    double movement;
    boolean lowJunctionResetMode;
    boolean PowerSetting = false;
    boolean ClawSetting = false;
    boolean originalMode = false;
    boolean coneStackMode = false;
    boolean groundJunctionMode = false;
    boolean semiAutoMode = false;
    int basePosition;
    int readVoltOnce = 0;
    boolean directionControl = false;
    boolean manualRotateBase;
    double powerCompensation;
    double movementPowerCompensation;
    double strafingPowerCompensationL;
    double strafingPowerCompensationR;
    boolean changeBaseDeg60;

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
    boolean button_dpad_right_already_pressed = false;
    boolean button_dpad_left_already_pressed = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_down_already_pressed = false;
    boolean button_dpad_right_already_pressed2 = false;
    boolean button_dpad_left_already_pressed2 = false;
    boolean button_dpad_up_already_pressed2 = false;
    boolean button_dpad_down_already_pressed2 = false;
    boolean button_left_trigger_already_pressed = false;
    boolean button_right_trigger_already_pressed = false;
    boolean button_left_trigger_already_pressed2 = false;
    boolean button_right_trigger_already_pressed2 = false;
    boolean button_back_already_pressed = false;
    boolean button_back_already_pressed2 = false;
    boolean button_start_already_pressed = false;
    boolean button_start_already_pressed2 = false;
    boolean double_trigger_already_pressed = false;


    //Semi-Auto Blitz for Cone Stack Vars
    int semi_auto_sc_blitz_step = 0;
    int blitzAdjustSequence = 0;
    ElapsedTime SC_Blitz_Timer = new ElapsedTime();
    ElapsedTime SCBlitzET = new ElapsedTime();
    int SC_ConeLevel = 0;
    int SC_AngleAdjustment;
//    int SC_LeftAngleAdjustment;
    boolean OpenClaw = false;
    boolean DontContinueExtend = true;
    boolean runBlitz = false;

    int R_semi_auto_sc_blitz_step = 0;
    int R_blitzAdjustSequence = 0;
    int R_SC_ConeLevel = 0;
//    int SC_RightAngleAdjustment;
    boolean R_runBlitz = false;

    //Semi-Auto Blitz for Substation High Junction Vars
    boolean HJBlitz = false;
    boolean dropCone;
    boolean HJAutoBlitzReady = true;
    int semi_auto_HJ_blitz_order = 0;
    int firstTimeBlitz = 0;

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
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        LightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "LightStrip");

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
        DirectionControl = new Direction_Control(IMU, FrontLeft, FrontRight, BackLeft, BackRight);
        //        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);
        BaseControl = new Base_Control(RotatingBase);

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RotatingBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        ET.reset();
        SC_Blitz_Timer.reset();

        while (opModeIsActive()) {

            /*****************************************************************
             * Bumper Right (G1) - Set Low Power Mode/High Power Mode for driving
             *****************************************************************/



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

            if (PowerSetting) {
                if ((y > 0.45) || (y < -0.45) || (x > 0.45) || (x < -0.45)) {
                    movement = movement + 0.01;
                }
                else {
                    movement = 0.5;
                }

                if (movement > 1) {
                    movement = 1;
                }
            } else {
                movement = 0.5;
            }

            if (!PowerSetting) {
                movementPowerCompensation = 0;
            } else {
                movementPowerCompensation = 0.01;
            }

            if (y > 0.45) {
                powerCompensation = 0.0015 + movementPowerCompensation;
            } else if (y < -0.45) {
                powerCompensation = -0.0015 - movementPowerCompensation;
            } else {
                powerCompensation = 0;
            }

            if (x > 0.45) {
//                strafingPowerCompensationR = strafingPowerCompensationR + 0.001;
                strafingPowerCompensationR = 0.048;
                strafingPowerCompensationL = 0;
//                if (strafingPowerCompensationR > 0.048) {
//                    strafingPowerCompensationR = 0;
//                }
            } else if (x < -0.45) {
//                strafingPowerCompensationL = strafingPowerCompensationL + 0.001;

                strafingPowerCompensationL = 0.02;
                strafingPowerCompensationR = 0;
//                if (strafingPowerCompensationL > 0.02) {
//                    strafingPowerCompensationL = 0.02;
//                }
            } else {
                strafingPowerCompensationR = 0;
                strafingPowerCompensationL = 0;
            }

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

            if (!button_bumper_left_already_pressed) {
                if (gamepad1.left_bumper) {
                    if (!directionControl) {
                        directionControl = true;
                    } else {
                        directionControl = false;
                    }
                    button_bumper_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.left_bumper) {
                    button_bumper_left_already_pressed = false;
                }
            }

            if (!directionControl) {
                l = 0;
            } else {
                DirectionControl.SetTargetDirection(0, 0.17);
                l = DirectionControl.GyroTask_TeleOp();
            }

            //if turning, set the gains to 0
            if (rx != 0) {
//                l = DirectionControl.GyroTask_TeleOp() * 0;
                directionControl = false;
            }

//            LightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
            LightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
//            LightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx + l) / denominator;
            double BLPower = (y - x + rx + l) / denominator;
            double FRPower = (y - x - rx - l) / denominator;
            double BRPower = (y + x - rx - l) / denominator;

            FrontLeft.setPower(FLPower);
            BackLeft.setPower(BLPower + strafingPowerCompensationL);
            FrontRight.setPower(FRPower + powerCompensation);
            BackRight.setPower(BRPower + powerCompensation + strafingPowerCompensationR);

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
                Claw.setPower(-0.4);
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
                        Claw.setPower(-0.4);
                        ET.reset();
                        targetJunction++;
                    }
                    break;
                case 3:
//                    if (ET.milliseconds() > 500) {
//                        SetBasePosition(0);
//                        targetJunction++;
//                    }
                    break;

                case 4:
                    break;

                case 5:
                    ET.reset();
                    changeBaseDeg60 = true;
                    targetJunction++;
                    break;

                case 6:
                    if (ET.milliseconds() > 800) {
                        ExtendingRail.setTargetPosition(0);
                        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ExtendingRail.setPower(0.7);
                        RailControlV2.SetTargetPosition(2150, -1, 1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;
                case 7:
//                    if (ET.milliseconds() > 500) {
//                        SetBasePositionRTP(0);
                        targetJunction++;
//                    }
                    break;

                case 8:
                    break;

                case 9:
                    ET.reset();
                    changeBaseDeg60 = true;
                    targetJunction++;
                    break;

                case 10:
                    if (ET.milliseconds() > 800) {
//                        SetExtendingPosition(100);
                        ExtendingRail.setTargetPosition(0);
                        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ExtendingRail.setPower(0.7);
                        RailControlV2.SetTargetPosition(2950, -1, 1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;

                case 11:
//                    if (ET.milliseconds() > 500) {
//                        SetBasePositionRTP(0);
                        targetJunction++;
//                    }
                    break;

                case 12:
                    break;

                case 13:
                    ET.reset();
                    targetJunction++;
                    break;

                case 14:
                    if (ET.milliseconds() > 800) {
                        ExtendingRail.setTargetPosition(0);
                        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ExtendingRail.setPower(0.7);
                        RailControlV2.SetTargetPosition(130, -0.7, 0.7);
                        ET.reset();
                        targetJunction++;
                    }
                    break;

                case 15:
//                    if (ET.milliseconds() > 500) {
//                        SetBasePositionRTP(0);
//                        targetJunction++;
//                    }
                    break;

                case 16:
                    break;

                case 17:
                    ET.reset();
                    ClawSetting = true;
                    changeBaseDeg60 = true;
                    Claw.setPower(1);
//                    LightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    targetJunction++;
                    break;
                case 18:
                    if (ET.milliseconds() > 800) {
//                        SetExtendingPosition(100);
                        ExtendingRail.setTargetPosition(0);
                        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ExtendingRail.setPower(0.7);
                        RailControlV2.SetTargetPosition(1280, -1, 1);
                        ET.reset();
                        targetJunction++;
                    }
                    break;
                case 19:
                    if (ET.milliseconds() > 500) {
                        SetBasePositionRTP(0);
                        targetJunction++;
                    }
                    break;

                case 20:
                    break;

                case 21:
                    changeBaseDeg60 = false;
                    RailControlV2.SetTargetPosition(0, -1, 1);
                    ExtendingRail.setTargetPosition(0);
                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ExtendingRail.setPower(0.7);
                    ET.reset();
                    targetJunction++;
                    break;

                case 22:
                    if (ET.milliseconds() > 150) {
                        ClawSetting = false;
                        Claw.setPower(-0.4);
                        ET.reset();
                        targetJunction++;
                    }
                    break;

                case 23:
                    if (ET.milliseconds() > 400) {
                        SetBasePositionRTP(0);
                        targetJunction++;
                    }
                    break;

                case 24:
                    break;

                default:
                    break;

            }

            //Set mode to cone stack mode and turn off all other modes
            if (!button_dpad_left_already_pressed2) {
                if (gamepad2.dpad_left) {
                    if (!coneStackMode) {
                        coneStackMode = true;
                        semiAutoMode = false;

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

            //set mode to semi auto mode and turn off all other modes
            if (!button_dpad_left_already_pressed) {
                if (gamepad1.dpad_left) {
                    if (!semiAutoMode) {
                        coneStackMode = false;
                        semiAutoMode = true;

                    } else {
                        semiAutoMode = false;
                    }
                    button_dpad_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.dpad_left) {
                    button_dpad_left_already_pressed = false;
                }
            }

            //If cone stack mode is false, then buttons a, b, and y will correspond to the height of the junctions
            //If ground junction mode is true, then button a will not close claw when moving to low junction position
            if (!coneStackMode && !semiAutoMode) {

                /*****************************************************************
                 * Button A (G2) : Set Rail height to place on the low junction
                 *****************************************************************/

                if (groundJunctionMode) {
                    if (!button_a_already_pressed2) {
                        if (gamepad2.a) {
                            //code for low junction when pressed
                            groundJunctionMode = false;
//                            RightClaw.setPosition(0);
//                            LeftClaw.setPosition(0);
                            RailControlV2.SetTargetPosition(0, -0.7, 0.7);
                            ClawSetting = false;
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
                 * Button X (G2) : Reset Rail and Base from any height
                 * to original position
                 *****************************************************************/

                if (!button_x_already_pressed2) {
                    if (gamepad2.x) {
                        //code for releasing cone and resetting base
                        coneStackMode = false;
                        groundJunctionMode = false;
                        lowJunctionResetMode = false;
                        targetJunction = 21;
//                        ClawSetting = false;
//                        Claw.setPower(-1);
//                        LightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                        RailControlV2.SetTargetPosition(0, -1, 1);
                        button_x_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.x) {
                        button_x_already_pressed2 = false;
                    }
                }

            //if cone stack mode is true, buttons a, b, and y will correspond to the height of the cone stack
            } else if (coneStackMode && !semiAutoMode) {

                /*****************************************************************
                 * Button A,B,X,Y (G2) : In Cone Stack Mode, buttons will
                 * set heights for cones on stack
                 *****************************************************************/

                if (!button_x_already_pressed2) {
                    if (gamepad2.x) {
                        //code for 4th cone on stack
                        ClawSetting = false;
                        coneStackMode = false;
                        Claw.setPower(-0.4);
                        RailControlV2.SetTargetPosition(119, -1, 1);
//                        SetBasePositionRTP(0);
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
                        Claw.setPower(-0.4);
                        RailControlV2.SetTargetPosition(246, -1, 1);
//                        SetBasePositionRTP(0);
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
                        Claw.setPower(-0.4);
                        RailControlV2.SetTargetPosition(371, -1, 1);
//                        SetBasePositionRTP(0);
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
                        Claw.setPower(-0.4);
                        RailControlV2.SetTargetPosition(479, -1, 1);
//                        SetBasePositionRTP(0);
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }
            }

            //if semiAutoMode is on, pressing dpad up on G1 will enable the mid rank sequence to start
            else if (semiAutoMode && !coneStackMode) {

                //Dpad right during semiAutoMode continues the sequence of the SemiAutoSCBlitz_Left() method
                //Opens claw while rails are above the junction to continue
                if (!button_dpad_right_already_pressed) {
                    if (gamepad1.dpad_right) {
                        DontContinueExtend = false;
                        OpenClaw = true;
                        button_dpad_right_already_pressed = true;
                    }
                } else {
                    if (!gamepad1.dpad_right) {
                        DontContinueExtend = true;
                        OpenClaw = false;
                        button_dpad_right_already_pressed = false;
                    }
                }

                //Dpad up starts the SemiAutoSCBlitz_Left() method
                if (!button_dpad_up_already_pressed) {
                    if (gamepad1.dpad_up) {
                        runBlitz = true;
                        button_dpad_up_already_pressed = true;
                    }
                } else {
                    if (!gamepad1.dpad_up) {
                        button_dpad_up_already_pressed = false;
                    }
                }

                //Dpad down starts the SemiAutoSCBlitz_Right() method
                if (!button_dpad_down_already_pressed) {
                    if (gamepad1.dpad_down) {
                        R_runBlitz = true;
                        button_dpad_down_already_pressed = true;
                    }
                } else {
                    if (!gamepad1.dpad_down) {
                        button_dpad_down_already_pressed = false;
                    }
                }

                //Run the blitz for left or right side
                if (runBlitz) {
                    R_runBlitz = false;
                    SemiAutoSCBlitz_Left();
                }

                if (R_runBlitz) {
                    runBlitz = false;
                    SemiAutoSCBlitz_Right();
                }

                //Manually turn base during semi auto mode
                if (button_right_trigger_already_pressed2 == false) {
                    if (gamepad2.right_trigger > 0) {

                        blitzAdjustSequence = 1;

                        button_right_trigger_already_pressed2 = true;
                    }
                } else {
                    if (gamepad2.right_trigger == 0) {
                        button_right_trigger_already_pressed2 = false;
                    }
                }

                if (button_left_trigger_already_pressed2 == false) {
                    if (gamepad2.left_trigger > 0) {

                        blitzAdjustSequence = 4;

                        button_left_trigger_already_pressed2 = true;
                    }
                } else {
                    if (gamepad2.left_trigger == 0) {
                        button_left_trigger_already_pressed2 = false;
                    }
                }

                switch (blitzAdjustSequence) {
                    case 1:
                        SCBlitzET.reset();
                        blitzAdjustSequence++;
                        break;
                    case 2:
                        if (SCBlitzET.milliseconds() > 100) {
                            SC_AngleAdjustment = SC_AngleAdjustment - 40;
                            blitzAdjustSequence++;
                        }
                        break;

                    case 3:
                        break;
                    case 4:
                        SCBlitzET.reset();
                        blitzAdjustSequence++;
                        break;
                    case 5:
                        if (SCBlitzET.milliseconds() > 100) {
                            SC_AngleAdjustment = SC_AngleAdjustment + 40;
                            blitzAdjustSequence++;
                        }
                        break;


                    default:
                        break;
                }
            }

            ///////////////////////////////////////////////////////////////////////////

            /*****************************************************************
             * Button A & B (G1) : Auto High Junction Blitz Algorithm
             *****************************************************************/

            if (!button_a_already_pressed) {
                if (gamepad1.a) {
                    if (!HJBlitz) {
                        HJBlitz = true;
                    } else {
                        firstTimeBlitz = 0;
                        HJBlitz = false;
                    }
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }

            if (!button_b_already_pressed) {
                if (gamepad1.b) {
                    dropCone = true;
                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad1.b) {
                    dropCone = false;
                    button_b_already_pressed = false;
                }
            }

            if (HJBlitz) {
                SemiAutoHJBlitz();
            }

            if (!button_x_already_pressed) {
                if (gamepad1.x) {
                    if (HJAutoBlitzReady) {
                        HJAutoBlitzReady = false;
                    } else {
                        HJAutoBlitzReady = true;
                    }
                    button_x_already_pressed = true;
                }
            } else {
                if (!gamepad1.x) {
                    button_x_already_pressed = false;
                }
            }

            /*****************************************************************
             * Button Dpad Right (G2) : Bring rail down without opening claw
             *****************************************************************/

            //Lower Rail In the Direction it is facing
            if (!button_dpad_right_already_pressed2) {
                if (gamepad2.dpad_right) {
                    //code for releasing cone and resetting base
                    ClawSetting = true;
                    coneStackMode = false;
                    groundJunctionMode = false;
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
                    //code for picking up cone for ground junction
                    ClawSetting = true;
                    coneStackMode = false;
                    groundJunctionMode = true;
                    Claw.setPower(1);
                    targetJunction = 13;
                    button_dpad_down_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_down) {
                    button_dpad_down_already_pressed2 = false;
                }
            }

//            /*****************************************************************
//             * Bumper Right (G1) : Reset Rail to 0
//             *****************************************************************/
//
//            if (!button_bumper_right_already_pressed) {
//                if (gamepad1.right_bumper) {
////                    ExtendingRail.setTargetPosition(0);
////                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                    ExtendingRail.setPower(1);
//                    ExtendingRail.setPower(0);
////                    ExtendingRail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    ExtendingRail.setTargetPosition(0);
////                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    button_bumper_right_already_pressed = true;
//                }
//            } else {
//                if (!gamepad1.right_bumper) {
//                    button_bumper_right_already_pressed = false;
//                }
//            }

            /*****************************************************************
             * Bumper Right (G2) : Rotate 360 base 90 degrees
             *****************************************************************/

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {
//                    RotatingBase.setTargetPosition(-1035);
                    if (changeBaseDeg60) {
                        RotatingBase.setTargetPosition(-610);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RotatingBase.setPower(1);
                    } else {
                        RotatingBase.setTargetPosition(-1035);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RotatingBase.setPower(1);
                    }
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
//                    RotatingBase.setTargetPosition(1035);
                    if (changeBaseDeg60) {
                        RotatingBase.setTargetPosition(610);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RotatingBase.setPower(1);
                    } else {
                        RotatingBase.setTargetPosition(1035);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RotatingBase.setPower(1);
                    }

                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Bumper Left & Bumper Right (G2) : Extend arm out and grab cone from substation
             *****************************************************************/

            if (!button_back_already_pressed2) {
                if (gamepad2.back) {
                    extendoSeq = 1;
                    button_back_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.back) {
                    button_back_already_pressed2 = false;
                }
            }

            if (!button_start_already_pressed2) {
                if (gamepad2.start) {
                    extendoSeq = 5;
                    button_start_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.start) {
                    button_start_already_pressed2 = false;
                }
            }

            switch (extendoSeq) {

                case 1:
                    RailControlV2.SetTargetPosition(120, -0.75, 0.75);
                    extendoSeq++;
                    break;

                case 2:
                    ExtendingRail.setTargetPosition(580);
                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ExtendingRail.setPower(0.65);
                    extendoSeq++;
                    break;

                case 3:

                    break;

                case 4:
                    if (ExtendingRail.getCurrentPosition() > 400) {
                        ClawSetting = true;
                        ET.reset();
                        extendoSeq++;
                    }
                    break;

                case 5:
//                    if (ET.milliseconds() > 450) {
                        ExtendingRail.setTargetPosition(0);
                        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ExtendingRail.setPower(0.5);
                        extendoSeq++;
//                    }
                    break;

                default:
                    break;
            }

            /*****************************************************************
             * Button Y (G1) : Rotate 360 base to 0
             *****************************************************************/

            if (!button_y_already_pressed) {
                if (gamepad1.y) {
                    RotatingBase.setTargetPosition(0);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    button_y_already_pressed = true;
                }
            } else {
                if (!gamepad1.y) {
                    button_y_already_pressed = false;
                }
            }

//            /*****************************************************************
//             * Button X (G1) : Reset Rail
//             *****************************************************************/

//            if (!button_a_already_pressed) {
//                if (gamepad1.a) {
////                    ExtendingRailControl.SetTargetPosition(400, -0.6, 0.6);
//                    ExtendingRail.setTargetPosition(610);
//                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ExtendingRail.setPower(0.6);
//                    button_a_already_pressed = true;
//                }
//            } else {
//                if (!gamepad2.a) {
//                    button_a_already_pressed = false;
//                }
//            }

//            if (!button_x_already_pressed) {
//                if (gamepad1.x) {
////                    ExtendingRailControl.SetTargetPosition(0, -0.01, 0.01);
//                    ExtendingRail.setTargetPosition(0);
//                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ExtendingRail.setPower(0);
//                    button_x_already_pressed = true;
//                }
//            } else {
//                if (!gamepad1.x) {
//                    button_x_already_pressed = false;
//                }
//            }

            /******************************************
             * Trigger (G2) Manual Calibration For Rotating Base
             ******************************************/

//            if (manualRotateBase) {
                if (button_left_trigger_already_pressed2 == false) {
                    if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
                        RotatingBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        RotatingBase.setPower(0.2);

                        button_left_trigger_already_pressed2 = true;
                    }
                } else {
                    if (gamepad2.left_trigger == 0) {
                        RotatingBase.setPower(0);
                        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RotatingBase.setTargetPosition(0);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        button_left_trigger_already_pressed2 = false;
                    }
                }

                if (button_right_trigger_already_pressed2 == false) {
                    if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {
                        RotatingBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        RotatingBase.setPower(-0.2);

                        button_right_trigger_already_pressed2 = true;
                    }
                } else {
                    if (gamepad2.right_trigger == 0) {
                        RotatingBase.setPower(0);
                        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        RotatingBase.setTargetPosition(0);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        button_right_trigger_already_pressed2 = false;
                    }
                }
//            }

            /******************************************
             * Trigger (G1) Manual Calibration For Viper Slides
             ******************************************/

            if (button_left_trigger_already_pressed == false) {

                RailRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RailLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger == 0) {

                    RailRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailRight.setPower(-0.4);
                    RailLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailLeft.setPower(0.4);

                    button_left_trigger_already_pressed = true;
                }
            } else {
                if (gamepad1.left_trigger == 0) {
                    ET.reset();
                    RailRight.setPower(0);
                    RailLeft.setPower(0);
                    RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    button_left_trigger_already_pressed = false;
                }
            }

            if (button_right_trigger_already_pressed == false) {

                RailRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RailLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (gamepad1.right_trigger > 0.4 && gamepad1.left_trigger == 0) {

                    RailRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailRight.setPower(0.4);
                    RailLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RailLeft.setPower(-0.4);

                    button_right_trigger_already_pressed = true;
                }
            } else {
                if (gamepad1.right_trigger == 0) {
                    ET.reset();
                    RailRight.setPower(0);
                    RailLeft.setPower(0);
                    RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    button_right_trigger_already_pressed = false;
                }
            }

//            if (runBlitz || R_runBlitz || HJBlitz) {
//                RotatingBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                BaseControl.RotatingBaseTask();
//            } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
//               manualRotateBase = true;
//            } else {
//                manualRotateBase = false;
//                RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

//            MechDrive.Task(GyroContinuity());
            RailControlV2.RailTask();
            BaseControl.RotatingBaseTask();
//            ExtendingRailControl.ExtendingRailTask();
//            telemetry.addData("Voltage", voltageSensor.getVoltage());
            if (readVoltOnce == 0) {
                telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
                    @Override
                    public Double value() {
                        return getBatteryVoltage();
                    }
                });
                readVoltOnce++;
            }
            //DirectionControl.GyroTask();
            telemetry.addData("LeftRail Power", RailLeft.getPower());
            telemetry.addData("LeftRail Encoder", RailLeft.getCurrentPosition());
            telemetry.addData("RightRail Power", RailRight.getPower());
            telemetry.addData("RightRail Encoder", RailRight.getCurrentPosition());
            telemetry.addData("Extending Encoder", ExtendingRail.getCurrentPosition());
            telemetry.addData("SemiAutoMode", semiAutoMode);
            telemetry.addData("ConeStackMode", coneStackMode);
            telemetry.addData("PowerSetting", PowerSetting);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
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

        RotatingBase.setDirection(DcMotorSimple.Direction.REVERSE);
//        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RotatingBase.setTargetPosition(0);
//        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void SetBasePositionRTP(int position) {
        RotatingBase.setTargetPosition(position);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotatingBase.setPower(0.7);
//        BaseControl.SetTargetPosition(position, -0.8, 0.8);
    }

    public void SetBasePosition(int position) {
        RotatingBase.setTargetPosition(position);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotatingBase.setPower(0.5);
//        BaseControl.SetTargetPosition(position, -0.8, 0.8);
    }

    public void SetExtendingPosition(int extendingPos) {
        ExtendingRail.setTargetPosition(extendingPos);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendingRail.setPower(1);
    }

    public void SetExtendingPositionLowPower(int extendingPos) {
        ExtendingRail.setTargetPosition(extendingPos);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendingRail.setPower(0.6);
    }

    public void SemiAutoSCBlitz_Left() {

        switch (semi_auto_sc_blitz_step) {

            case 0:
                if (RailControlV2.GetTaskState() == Task_State.INIT || RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
                    if (SC_Blitz_Timer.milliseconds() > 400) {
                        RailControlV2.SetTargetPosition(570, -1, 1);
//                        SetBasePosition(1020);
                        RotatingBase.setTargetPosition(1020);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RotatingBase.setPower(1);
                        SC_Blitz_Timer.reset();
                        semi_auto_sc_blitz_step++;
                    }
                }
                break;
            case 1:
                if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
//                    SetExtendingPosition(620);
                    SetExtendingPosition(650);
                    SC_Blitz_Timer.reset();
                    semi_auto_sc_blitz_step++;
                }
                break;
            case 2:
//                if (SC_Blitz_Timer.milliseconds() > 480) {
                    if (!DontContinueExtend) {
                        ClawSetting = true;
                        Claw.setPower(1);
                        SC_Blitz_Timer.reset();
                        semi_auto_sc_blitz_step++;
                    }
//                }
                break;
            case 3:
                if (SC_Blitz_Timer.milliseconds() > 480) {
                    RailControlV2.SetTargetPosition(1200, -1, 1);
                    semi_auto_sc_blitz_step++;
                }
                break;
            case 4:
                if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
//                    SetExtendingPosition(75);
                    SetExtendingPositionLowPower(0);
                    RailControlV2.SetTargetPosition(2925, -1, 1);
                    SetBasePosition(-505 + SC_AngleAdjustment);
                    SC_Blitz_Timer.reset();
                    semi_auto_sc_blitz_step++;
                }
                break;
            case 5:
                if ((RotatingBase.getCurrentPosition() <= -250) &&
                        (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY)) {
                    if (OpenClaw) {
                        SC_Blitz_Timer.reset();
                        semi_auto_sc_blitz_step++;
                    } else {
                        SetBasePosition(-505 + SC_AngleAdjustment);
                    }
                }
                break;
            case 6:
                RailControlV2.SetTargetPosition(2690, -1, 1);
                SC_Blitz_Timer.reset();
                semi_auto_sc_blitz_step++;
                break;
            case 7:
                if (SC_Blitz_Timer.milliseconds() > 500) {
                    ClawSetting = false;
                    Claw.setPower(-0.4);
                    if (Claw.getPower() < -0.3) {
                        semi_auto_sc_blitz_step++;
                        SC_Blitz_Timer.reset();
                    }
                }
                break;
            case 8:
                SC_ConeLevel++;
                if (SC_ConeLevel >= 4) {
                    RailControlV2.SetTargetPosition(0, -1, 1);
                    SetBasePosition(0);
                    SetExtendingPositionLowPower(0);
                    semi_auto_sc_blitz_step = 11;
                } else {
                    semi_auto_sc_blitz_step++;
                    SC_Blitz_Timer.reset();
                }
                break;
            case 9:
                if (SC_Blitz_Timer.milliseconds() > 200) {
                    RailControlV2.SetTargetPosition(2690, -1, 1);
//                    SetBasePosition(1020);
                    RotatingBase.setTargetPosition(1020);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    SetExtendingPositionLowPower(0);
                    semi_auto_sc_blitz_step++;
                }
                break;
            case 10:
                if (RotatingBase.getCurrentPosition() > 0) {

                    if (SC_ConeLevel == 1) {
                        RailControlV2.SetTargetPosition(490, -1, 1);
                        semi_auto_sc_blitz_step = 1;
                    } else if (SC_ConeLevel == 2) {
                        RailControlV2.SetTargetPosition(380, -1, 1);
                        semi_auto_sc_blitz_step = 1;
                    } else if (SC_ConeLevel == 3) {
                        RailControlV2.SetTargetPosition(295, -1, 1);
                        semi_auto_sc_blitz_step = 1;
                    } else {
                        semi_auto_sc_blitz_step++;
                    }
                }
                break;
            case 11:
                runBlitz = false;
                semi_auto_sc_blitz_step++;
                break;
            default:
                break;
        }
    }

    public void SemiAutoSCBlitz_Right() {

        switch (R_semi_auto_sc_blitz_step) {

            case 0:
                if (RailControlV2.GetTaskState() == Task_State.INIT || RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
                    if (SC_Blitz_Timer.milliseconds() > 400) {
                        RailControlV2.SetTargetPosition(570, -1, 1);
//                        SetBasePosition(-1020);
                        RotatingBase.setTargetPosition(-1020);
                        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RotatingBase.setPower(1);
                        SC_Blitz_Timer.reset();
                        R_semi_auto_sc_blitz_step++;
                    }
                }
                break;
            case 1:
                if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
//                    SetExtendingPosition(620);
                    SetExtendingPosition(650);
                    SC_Blitz_Timer.reset();
                    R_semi_auto_sc_blitz_step++;
                }
                break;
            case 2:
//                if (SC_Blitz_Timer.milliseconds() > 480) {
                    if (!DontContinueExtend) {
                        ClawSetting = true;
                        Claw.setPower(1);
                        SC_Blitz_Timer.reset();
                        R_semi_auto_sc_blitz_step++;
                    }
//                }
                break;
            case 3:
                if (SC_Blitz_Timer.milliseconds() > 480) {
                    RailControlV2.SetTargetPosition(1200, -1, 1);
                    R_semi_auto_sc_blitz_step++;
                }
                break;
            case 4:
                if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
//                    SetExtendingPosition(75);
                    SetExtendingPositionLowPower(0);
                    RailControlV2.SetTargetPosition(2925, -1, 1);
                    SetBasePosition(505 + SC_AngleAdjustment);
                    SC_Blitz_Timer.reset();
                    R_semi_auto_sc_blitz_step++;
                }
                break;
            case 5:
                if ((RotatingBase.getCurrentPosition() >= 250) &&
                        (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY)) {
                    if (OpenClaw) {
                        SC_Blitz_Timer.reset();
                        R_semi_auto_sc_blitz_step++;
                    } else {
                        SetBasePosition(505 + SC_AngleAdjustment);
                    }
                }
                break;
            case 6:
                RailControlV2.SetTargetPosition(2690, -1, 1);
                SC_Blitz_Timer.reset();
                R_semi_auto_sc_blitz_step++;
                break;
            case 7:
                if (SC_Blitz_Timer.milliseconds() > 500) {
                    ClawSetting = false;
                    Claw.setPower(-0.4);
                    if (Claw.getPower() < -0.3) {
                        R_semi_auto_sc_blitz_step++;
                        SC_Blitz_Timer.reset();
                    }
                }
                break;
            case 8:
                R_SC_ConeLevel++;
                if (R_SC_ConeLevel >= 4) {
                    RailControlV2.SetTargetPosition(0, -1, 1);
                    SetBasePosition(0);
                    SetExtendingPositionLowPower(0);
                    R_semi_auto_sc_blitz_step = 11;
                } else {
                    R_semi_auto_sc_blitz_step++;
                    SC_Blitz_Timer.reset();
                }
                break;
            case 9:
                if (SC_Blitz_Timer.milliseconds() > 200) {
                    RailControlV2.SetTargetPosition(2690, -1, 1);
//                    SetBasePosition(-1020);
                    RotatingBase.setTargetPosition(-1020);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    SetExtendingPositionLowPower(0);
                    R_semi_auto_sc_blitz_step++;
                }
                break;
            case 10:
                if (RotatingBase.getCurrentPosition() < 0) {

                    if (R_SC_ConeLevel == 1) {
                        RailControlV2.SetTargetPosition(490, -1, 1);
                        R_semi_auto_sc_blitz_step = 1;
                    } else if (R_SC_ConeLevel == 2) {
                        RailControlV2.SetTargetPosition(380, -1, 1);
                        R_semi_auto_sc_blitz_step = 1;
                    } else if (R_SC_ConeLevel == 3) {
                        RailControlV2.SetTargetPosition(295, -1, 1);
                        R_semi_auto_sc_blitz_step = 1;
                    } else {
                        R_semi_auto_sc_blitz_step++;
                    }
                }
                break;
            case 11:
                R_runBlitz = false;
                R_semi_auto_sc_blitz_step++;
                break;
            default:
                break;
        }
    }

    public void SemiAutoHJBlitz() {

        switch (semi_auto_HJ_blitz_order) {
            case 0:
                if (HJAutoBlitzReady) {
                    if (firstTimeBlitz == 0) {
                        RailControlV2.SetTargetPosition(380, -1, 1);
                        firstTimeBlitz++;
                        ET.reset();
                        semi_auto_HJ_blitz_order++;
                    } else {
                        ExtendingRail.setTargetPosition(370);
                        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ExtendingRail.setPower(1);
                        semi_auto_HJ_blitz_order = 3;
                    }
                }
                break;

            case 1:
                if (ET.milliseconds() > 700) {
                    ExtendingRail.setTargetPosition(370);
                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ExtendingRail.setPower(1);
                    semi_auto_HJ_blitz_order++;
                }
                break;

            case 2:
                if (ExtendingRail.getCurrentPosition() > 270) {
                    if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
                        RailControlV2.SetTargetPosition(0, -1, 1);
                        semi_auto_HJ_blitz_order++;
                    }
                }
                break;

            case 3:
                if (ExtendingRail.getCurrentPosition() > 360) {
                    ClawSetting = true;
                    HJBET.reset();
                    semi_auto_HJ_blitz_order++;
                }
                break;

            case 4:
                if (HJBET.milliseconds() > 460) {
                    RailControlV2.SetTargetPosition(2925, -1, 1);
                    RotatingBase.setTargetPosition(1020);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(0.6);
//                    BaseControl.SetTargetPosition(1020, -0.57, 0.57);
                    SetExtendingPosition(0);
                    semi_auto_HJ_blitz_order++;
                }
                break;

            case 5:
                if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
                    if (dropCone) {
                        RailControlV2.SetTargetPosition(2725, -1, 1);
                        HJBET.reset();
                        semi_auto_HJ_blitz_order++;
                    }
                }
                break;

            case 6:
                if (HJBET.milliseconds() > 250) {
                    ClawSetting = false;
                    HJBET.reset();
                    semi_auto_HJ_blitz_order++;
                }
                break;

            case 7:
                if (HJBET.milliseconds() > 180) {
                    SetExtendingPosition(0);
                    RotatingBase.setTargetPosition(-1020);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
//                    BaseControl.SetTargetPosition(1020, -1, 1);
                    semi_auto_HJ_blitz_order++;
                }
                break;

            case 8:
                if (RotatingBase.getCurrentPosition() < -300) {
                    RailControlV2.SetTargetPosition(0, -1, 1);
                    semi_auto_HJ_blitz_order++;
                }
                break;

            case 9:
                if (RotatingBase.getCurrentPosition() < -1000) {
                    semi_auto_HJ_blitz_order = 0;
                }
                break;

            default:
                break;
        }
    }
}
