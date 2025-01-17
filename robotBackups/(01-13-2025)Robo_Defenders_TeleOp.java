package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import java.util.Set;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="RoboDefenders TeleOp")
public class Robo_Defenders_TeleOp extends OpMode {
    
    private static final int LIFT_HEIGHT_LOW = 1000;
    private static final int LIFT_HEIGHT_MED = 2000;
    private static final int LIFT_HEIGHT_HIGH = 3000;
    private static final int LIFT_HEIGHT_TOP = 4000;
    
    private static final double INTAKE_PRESET_0 = -0.125;
    private static final double INTAKE_PRESET_1 = .125;
    private static final double INTAKE_PRESET_2 = .25;
    private static final double INTAKE_PRESET_3 = .375;
    private static final double INTAKE_PRESET_4 = .5;
    
    private static final double INTAKE_POWER = 1;
    
    private static final double LIFT_POWER = 1;
    private static final int LIFT_RESET_POSITION = 0;
    
    private static final double LIFT_GRABBER_OPEN = .75;
    private static final double LIFT_GRABBER_CLOSE = 1; 
    
    private static final double INTAKE_TILT_UPRIGHT = 0;
    private static final double INTAKE_TILT_LEAN = 1;
    
    private static final double FOUR_BAR_TILT_UPRIGHT = 0;
    private static final double FOUR_BAR_TILT_DOWNRIGHT = 1;
    
    private ElapsedTime runtime = new ElapsedTime();
    private Blinker control_Hub;
    private Servo intakeLeft, intakeRight;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intakeMotor;
    private NormalizedColorSensor intakeColorSensor;
    private Servo claw;
    private Servo _clawTilt;
    private DcMotor _liftMotor;
    private DcMotor _liftHanger;
    private DcMotor _backHanger;
    private Servo _intakeTilt;
    private Servo _fourBarTiltLeft;
    private Servo _fourBarTiltRight;

    private DcMotor frontleft;
    private DcMotor frontright;
    private Gyroscope imu;
    
    double y, x, rx;
    double denominator, frontLeftPower, backRightPower, frontRightPower, backLeftPower;
    int _currentLiftHeight = 0;
    double _currentLiftMotorUp = 0;
    double _currentLiftMotorDown = 0;
    boolean _isLiftGrabbing = true;
    double _frontHangerMotorPower = 0;
    double _backHangerMotorPower = 0;
    boolean _isTransferProcessStarted = false;
    double _currentIntakePosition = 0;
    final double INTAKE_GAIN = .01; 
    double _currentIntakePower = 0;
    boolean _isIntakeLeaning = false;
    boolean _isIntakeIntaking = true;
    boolean _isIntakeOn = false;
    
    boolean _isTeamRed = false; 
    boolean _isTeamBlue = !_isTeamRed; 
    
    float[] hsvValues = new float[3]; 
    double intakeDistance; 
    final double INTAKE_RANGE = 5; //degrees 
    final double BLUE = 215; 
    final double RED = 2; 
    
    //
    // Toggle variables
    //
    boolean _wasRightBumperPressed = false;
    boolean _wasLeftBumperPressed = false;
    boolean _wasDpadUpPressed = false;
    boolean _wasDpadDownPressed = false;
    boolean _wasDpadLeftPressed = false;
    boolean _wasDpadRightPressed = false;
    boolean _wasAPressed = false;
    boolean _wasXPressed = false;
    boolean _wasYPressed = false;
    boolean _wasRightBumper2Pressed = false;
    boolean _wasLeftBumper2Pressed = false;
    boolean _wasDpadUp2Pressed = false;
    boolean _wasDpadRight2Pressed = false;
    boolean _wasDpadLeft2Pressed = false;
    boolean _wasDpadDown2Pressed = false;
    boolean _wasA2Pressed = false;
    boolean _wasY2Pressed = false;
    boolean _wasB2Pressed = false;
    boolean _wasX2Pressed = false;
    
    //
    // Temporary counts
    //
    int _liftStartCount = 0;
    int _liftResetCount = 0;
    int _transitionCount = 0;
    int _intakeCount = 0;
    int _startHangCount = 0;

    @Override
    public void init() {
        frontleft  = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        frontleft .setDirection(DcMotor.Direction.REVERSE);
        backleft  .setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright .setDirection(DcMotor.Direction.REVERSE);
        
        intakeLeft = hardwareMap.get(Servo.class, "leftIntakeExtension");
        intakeRight = hardwareMap.get(Servo.class, "rightIntakeExtension");
        
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        _liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        _liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Prevent lift from rising on its own
        _liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        claw = hardwareMap.get(Servo.class, "claw");
        _clawTilt = hardwareMap.get(Servo.class, "clawTilt");
        claw.setPosition(LIFT_GRABBER_OPEN);
        _isLiftGrabbing = false;
        
        _liftHanger = hardwareMap.get(DcMotor.class, "liftHanger");
        _backHanger = hardwareMap.get(DcMotor.class, "backHanger");
        _liftHanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _liftHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backHanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        _intakeTilt.setPosition(INTAKE_TILT_UPRIGHT);
        _fourBarTiltLeft = hardwareMap.get(Servo.class, "fourBarTiltLeft");
        _fourBarTiltLeft.setPosition(FOUR_BAR_TILT_UPRIGHT);
        _fourBarTiltRight = hardwareMap.get(Servo.class, "fourBarTiltRight");
        _fourBarTiltRight.setPosition(1-FOUR_BAR_TILT_UPRIGHT);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        x =  getAdjustedX();
        y = -getAdjustedY();
        rx = gamepad1.right_stick_x;
        
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower  = (y + x + rx) / denominator;
        backLeftPower   = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower  = (y + x - rx) / denominator;
        
        frontleft .setPower(frontLeftPower);
        backleft  .setPower(backLeftPower);
        frontright.setPower(frontRightPower);
        backright .setPower(backRightPower);
       
       //
       // Start lift
       //
       if (gamepad1.right_bumper && _wasRightBumperPressed == false) {
            _wasRightBumperPressed = true;
        }
        
        if (!gamepad1.right_bumper && _wasRightBumperPressed == true) {
            _wasRightBumperPressed = false;
            startLift();
        }
        
        //
        // Reset lift
        //
        if (gamepad1.left_bumper && _wasLeftBumperPressed == false) {
            _wasLeftBumperPressed = true;
        }
        
        if (!gamepad1.left_bumper && _wasLeftBumperPressed == true) {
            _wasLeftBumperPressed = false;
            resetLift();
        }
         
        //
        // Setting lift height
        //
        
        if (gamepad1.dpad_up && _wasDpadUpPressed == false) {
            _wasDpadUpPressed = true;
        }
        
        if (!gamepad1.dpad_up && _wasDpadUpPressed == true) {
            _wasDpadUpPressed = false;
            setLiftHeight(LIFT_HEIGHT_TOP);
        }
            
        if (gamepad1.dpad_down && _wasDpadDownPressed == false) {
            _wasDpadDownPressed = true;
        }
        
        if (!gamepad1.dpad_down && _wasDpadDownPressed == true) {
            _wasDpadDownPressed = false;
            setLiftHeight(LIFT_HEIGHT_LOW);
        }

        if (gamepad1.dpad_right && _wasDpadRightPressed == false) {
            _wasDpadRightPressed = true;
            
        }
        
        if (!gamepad1.dpad_right && _wasDpadRightPressed == true) {
            _wasDpadRightPressed = false;
            setLiftHeight(LIFT_HEIGHT_MED);
        }
            
        if (gamepad1.dpad_left && _wasDpadLeftPressed == false) {
            _wasDpadLeftPressed = true;
        }
        
        if (!gamepad1.dpad_left && _wasDpadLeftPressed == true) {
            _wasDpadLeftPressed = false;
            setLiftHeight(LIFT_HEIGHT_HIGH);
        }
            
        //
        // Manual lift height up
        //
       
        setManualLiftHeightUp(gamepad1.right_trigger);
        setManualLiftHeightDown(gamepad1.left_trigger);
        
        //
        // toggle lift grab/release
        //
        
        if (gamepad1.a && _wasAPressed == false) {
            _wasAPressed = true;
        }
        if (!gamepad1.a && _wasAPressed == true) {
            _wasAPressed = false;
            toggleLiftGrabRelease();
        }
        
        //
        // begin low to high procedure
        //
        
        if (gamepad1.x && _wasXPressed == false) {
            _wasXPressed = true;
        }
        if (!gamepad1.x && _wasXPressed == true) {
            _wasXPressed = false;
            beginLowToHighProcedure();
        }
        //
        // lift hanger extension
        //
        handleLiftHangerExtension(gamepad2.left_stick_x);
        handleBackHangerExtension(gamepad2.right_stick_x);
        
        //
        // start intake process
        //
        
        if (gamepad2.right_bumper && _wasRightBumper2Pressed == false) {
            _wasRightBumper2Pressed = true;
        }
        if (!gamepad2.right_bumper && _wasRightBumper2Pressed == true) {
            _wasRightBumper2Pressed = false;
            beginIntakeProcess();
        }
        
        if (gamepad2.left_bumper && _wasLeftBumper2Pressed == false){
             _wasLeftBumper2Pressed = true;
        }
        if (!gamepad2.left_bumper && _wasLeftBumper2Pressed == true) {
            _wasLeftBumper2Pressed = false;
            endIntakeProcess();
        }    
        
        //
        // Intake presets
        //
        
        if (gamepad2.dpad_up && _wasDpadUp2Pressed == false) {
            _wasDpadUp2Pressed = true;
        }
        
        if (!gamepad2.dpad_up && _wasDpadUp2Pressed == true) {
            _wasDpadUp2Pressed = false;
            _currentIntakePosition = INTAKE_PRESET_1;
        }
        
        if (gamepad2.dpad_down && _wasDpadDown2Pressed == false) {
            _wasDpadDown2Pressed = true;
        }
        
        if (!gamepad2.dpad_down && _wasDpadDown2Pressed == true) {
            _wasDpadDown2Pressed = false;
            _currentIntakePosition = INTAKE_PRESET_3;
        }

        if (gamepad2.dpad_right && _wasDpadRight2Pressed == false) {
            _wasDpadRight2Pressed = true;
            
        }
        
        if (!gamepad2.dpad_right && _wasDpadRight2Pressed == true) {
            _wasDpadRight2Pressed = false;
            _currentIntakePosition = INTAKE_PRESET_2;
        }
            
        if (gamepad2.dpad_left && _wasDpadLeft2Pressed == false) {
            _wasDpadLeft2Pressed = true;
        }
        
        if (!gamepad2.dpad_left && _wasDpadLeft2Pressed == true) {
            _wasDpadLeft2Pressed = false;
            _currentIntakePosition = INTAKE_PRESET_4;
        }
        
                if (gamepad2.x && _wasX2Pressed == false) {
            _wasX2Pressed = true;
        }
        
        if (!gamepad2.x && _wasX2Pressed == true) {
            _wasX2Pressed = false;
            _currentIntakePosition = INTAKE_PRESET_0;
        }

        handleIntakeManualExtend(gamepad2.right_trigger);
        handleIntakeManualRetract(gamepad2.left_trigger);
        
        if (gamepad2.a && _wasA2Pressed == false) {
            _wasA2Pressed = true;
        }
        
        if (!gamepad2.a && _wasA2Pressed == true) {
            _wasA2Pressed = false;
            toggleIntakePower ();
        }
        
        if (gamepad1.y && _wasYPressed == false) {
            _wasYPressed = true;
        }
        
        if (!gamepad1.y && _wasYPressed == true) {
            _wasYPressed = false;
            startHangProcedure ();
        }
    
        if (gamepad2.y && _wasY2Pressed == false) {
            _wasY2Pressed = true;
        }
        
        if (!gamepad2.y && _wasY2Pressed == true) {
            _wasY2Pressed = false;
            toggleIntakeLean ();
        }
        
        if(gamepad2.b && _wasB2Pressed == false) {
            _wasB2Pressed =true;
        }
        
        if (!gamepad2.b && _wasB2Pressed == true) {
            _wasB2Pressed =false;
            toggleIntakeDirection();
        }
        
        _currentIntakePosition += _currentIntakePower * INTAKE_GAIN; 
        _currentIntakePosition = Math.min(Math.max(-0.125, _currentIntakePosition), 0.5); 
        
        intakeLeft.setPosition(_currentIntakePosition);
        intakeRight.setPosition(_currentIntakePosition); 

        if (!_isIntakeOn) {
            intakeMotor.setPower(0);
        } else {
            handleColorSensor();
            if (_isIntakeIntaking){
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(-INTAKE_POWER);
            }
        }

        handleIntakeTilt();
        
        updateTelemetry();
    }

    @Override 
    public void stop() {
        
    }
    
    private double getAdjustedX(){
        return Math.pow(gamepad1.left_stick_x * 1.1, 3);
    }
    private double getAdjustedY() {
        return Math.pow(gamepad1.left_stick_y, 3);
    }
    
    void updateTelemetry() {
        telemetry.addData("Team", _isTeamBlue ? "blue" : "red"); 
        telemetry.addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", x, y, rx);
        telemetry.addData("Powers", "FL (%.2f), BL (%.2f), FR (%.2f), BR (%.2f)", 
            frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        telemetry.addData("Lift", "unknown");
        telemetry.addData("Current Lift Height", _currentLiftHeight);
        telemetry.addData("Current Lift Motor Up", _currentLiftMotorUp);
        telemetry.addData("Current Lift Motor Down", _currentLiftMotorDown);
        telemetry.addData("Lift State", _isLiftGrabbing ? "Grabbing" : "Releasing");
        telemetry.addData("Count of Transitions", _transitionCount);
        telemetry.addData("front hanger motor power", _frontHangerMotorPower);
        telemetry.addData("back hanger motor power", _backHangerMotorPower);
        telemetry.addData("Transfer process", _isTransferProcessStarted ? "Running" : "Stopped");
        telemetry.addData("intakePosition", _currentIntakePosition);
        telemetry.addData("Intake Power", _currentIntakePower);
        telemetry.addData("Is Intake On?", _isIntakeOn);
        telemetry.addData("Start Hang Count", _startHangCount);
        telemetry.addData("Intake Lean State", _isIntakeLeaning ? "Leaning" : "Upright");
        telemetry.addData("Intake Direction", _isIntakeIntaking ? "Intake" : "Expelling");
        telemetry.addData("Color Sensor", hsvValues[0]); 
        telemetry.addData("Intake distance", intakeDistance); 
        telemetry.update();
    }
    
    //
    // hardware control methods
    //
    
    void handleColorSensor() {
        intakeDistance = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.INCH);

        if(intakeDistance < 3) {
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            
            if(_isTeamRed) {
                //check for blue 
                if(hsvValues[0] > BLUE - INTAKE_RANGE && hsvValues[0] < BLUE + INTAKE_RANGE) {
                    _isIntakeIntaking = false; 
                }
            } else {
                //check for team red 
                if(hsvValues[0] > RED - INTAKE_RANGE && hsvValues[0] < RED + INTAKE_RANGE) {
                    _isIntakeIntaking = false; 
                }
            }
        }
    }
    
    void startLift() {
        if(_liftMotor.isBusy()) {
            return;
        }
        _liftMotor.setTargetPosition(_currentLiftHeight);
        _liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _liftMotor.setPower(LIFT_POWER);
    }
    
    void resetLift() {
        setLiftHeight(LIFT_RESET_POSITION);
        
        startLift();
    }
    
    void setLiftHeight(int newLiftHeight){
        _currentLiftHeight = newLiftHeight;
    }
    
    void setManualLiftHeightUp(double liftMotorPower){
        if (liftMotorPower > 0) {
            _currentLiftMotorUp = liftMotorPower;
        }
        if (liftMotorPower == 0 && _currentLiftMotorUp > 0) {
            _currentLiftMotorUp = 0;
        }
        
        _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _liftMotor.setPower(_currentLiftMotorUp);
    }
    
    void setManualLiftHeightDown(double liftMotorPower) {
        if (liftMotorPower > 0) {
            _currentLiftMotorDown = liftMotorPower;
        }
        if (liftMotorPower == 0 && _currentLiftMotorDown > 0) {
            _currentLiftMotorDown = 0;
        }
        
        _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _liftMotor.setPower(_currentLiftMotorDown);
    }
    
    void toggleLiftGrabRelease() {
        _isLiftGrabbing = ! _isLiftGrabbing;
        if (_isLiftGrabbing){
            claw.setPosition(LIFT_GRABBER_CLOSE);
        } else {
            claw.setPosition(LIFT_GRABBER_OPEN);
        }
    }
    
    void beginLowToHighProcedure() {
        _transitionCount++;
        
        // TODO
    }
    
    void handleLiftHangerExtension(double stickPosition) {
        _frontHangerMotorPower = stickPosition;
        _liftHanger.setPower(_frontHangerMotorPower);
    }
    
    void handleBackHangerExtension(double stickPosition) {
        _backHangerMotorPower = stickPosition;
        _backHanger.setPower(_backHangerMotorPower);
    }
    
    void beginIntakeProcess() {
        _intakeTilt.setPosition(INTAKE_TILT_LEAN);
        intakeMotor.setPower(INTAKE_POWER);
    }
    
    void endIntakeProcess() {
        intakeMotor.setPower(0);
        _intakeTilt.setPosition(INTAKE_TILT_UPRIGHT);
    }
    
    void setIntakePreset(int intakePreset) {
        _currentIntakePosition = intakePreset;
    }
    
    void handleIntakeManualExtend(double power) {
        if (power > 0) {
            _currentIntakePower = power;
        }
        if (power == 0 && _currentIntakePower > 0) {
            _currentIntakePower = 0;
        }
        
        intakeMotor.setPower(_currentIntakePower);
    }
    
    void handleIntakeManualRetract(double power) {
        if (power > 0) {
            _currentIntakePower = -power;
        }
        if (power == 0 && _currentIntakePower < 0) {
            _currentIntakePower = 0;
        }
        
        intakeMotor.setPower(_currentIntakePower);
    }
    
    void toggleIntakePower () {
        _isIntakeOn = ! _isIntakeOn;
        if (_isIntakeOn){
            //always reset the direction whilst turning on the intake
            _isIntakeIntaking = true;
        }
    }
    
    void startHangProcedure () {
        _startHangCount ++ ;
        
        // TODO
    }
    
    void toggleIntakeLean () {
        _isIntakeLeaning = ! _isIntakeLeaning;
    }
    
    void handleIntakeTilt () {
        if (_isIntakeLeaning) {
            _intakeTilt.setPosition(INTAKE_TILT_LEAN);
        } else {
            _intakeTilt.setPosition(INTAKE_TILT_UPRIGHT);
        }
    }
    
    void toggleIntakeDirection () {
        _isIntakeIntaking = ! _isIntakeIntaking;
    }
}
