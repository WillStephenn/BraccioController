/*
This code is designed to control a robotic arm (Braccio) equipped with an ultrasonic sensor and a colour sensor.
The main functionality of the code is to scan the surrounding area for objects, detect their colour, and then
sort them into respective bins based on their colour (red, green, or blue).

The code utilises the Braccio library for controlling the robotic arm's servos, the InverseK library for
inverse kinematics calculations, and includes functions for ultrasonic distance measurement, colour detection,
and various arm movements (such as grabbing, dropping, and sorting objects).

The arm can scan a 180-degree area in front of it, detect the closest object within a specified distance range,
move towards the object, grab it, detect its colour, and then drop it into the appropriate colour-coded bin.
The process is repeated in a continuous loop.

Servo descriptions:

M1 = Arm Base.
M2 = Shoulder.
M3 = Elbow.
M4 = Wrist Vertical.
M5 = Wrist Rotation.
M6 = Gripper Control.
*/

#include <Braccio.h>
#include <Servo.h>
#include <InverseK.h>
#include <math.h>

//---BRACCIO ARM CONFIGURATION---
// Servo Class Declarations
    Servo base;
    Servo shoulder;
    Servo elbow;
    Servo wrist_rot;
    Servo wrist_ver;
    Servo gripper;
    Servo Radar;

// Gripper States
const int GRIPPER_OPEN = 10;
const int GRIPPER_CLOSED = 73;

// Servo Calibration Offsets
const int SERVO_M1_OFFSET = 0;
const int SERVO_M2_OFFSET = 3;
const int SERVO_M3_OFFSET = -3;
const int SERVO_M4_OFFSET = -5;
const int SERVO_M5_OFFSET = -10;

// Resting Arm Angles
#define M1_REST_ANGLE (90 + SERVO_M1_OFFSET)
#define M2_REST_ANGLE (90 + SERVO_M2_OFFSET)
#define M3_REST_ANGLE (90 + SERVO_M3_OFFSET)
#define M4_REST_ANGLE (90 + SERVO_M4_OFFSET)
#define M5_REST_ANGLE (90 + SERVO_M5_OFFSET)

// Object Handling
const int LIFT_HEIGHT = 200; // Height to lift object after grabbing
const int OBJECT_PICKUP_Z = 0; // Height to pick up object from
const int SCAN_RANGE_UPPER = 250; 
const int SCAN_RANGE_LOWER = 10;
const int LIDAR_CONE_OFFSET = 20;

// Bin Drop Coordinates
const int RED_BIN_X = -240;
const int RED_BIN_Y = -140;
const int RED_BIN_Z = 0;
const int GREEN_BIN_X = -254;
const int GREEN_BIN_Y = 0;
const int GREEN_BIN_Z = 0;
const int BLUE_BIN_X = -254;
const int BLUE_BIN_Y = 100;
const int BLUE_BIN_Z = 0;

// Colour Sensor Position
const int COLOUR_SENSOR_X = -160;
const int COLOUR_SENSOR_Y = 0;
const int COLOUR_SENSOR_Z = 40;

//---COLOUR SENSOR CONFIGURATION---
// Colour Sensor Calibration Parameters
const int RED_LOWER_BOUND = 20;
const int RED_UPPER_BOUND = 70;
const int GREEN_LOWER_BOUND = 70;
const int GREEN_UPPER_BOUND = 100;
const int BLUE_LOWER_BOUND = 60;
const int BLUE_UPPER_BOUND = 100;

// Colour Codes
const int COLOUR_RED = 1;
const int COLOUR_GREEN = 2;
const int COLOUR_BLUE = 3;

//---I/O Configuration---
const int ColourPinS0 = 4;
const int ColourPinS1 = 7;
const int ColourPinS2 = 8;
const int ColourPinS3 = 12;
const int ColourSensorOut = 13;

// Ultrasonic Sensor Pins
const int TrigPin = A1;  
const int EchoPin = A2; 
    
//--Global Variables---
// Struct definition for OffsetValues
struct OffsetValues {
    float XOffset, YOffset;
};

/**
 * @brief Initialises the Braccio arm, sensors, and serial communication.
 */
void setup() {

    // Braccio Startup function
    Braccio.begin();

    // InverseK library Setup
    Link base, upperarm, forearm, hand;

    // Set up the links for the InverseK library passing length and joint limits
    // Base: 71.5 mm, Upper Arm: 125 mm, Forearm: 125 mm, Hand: 192 mm
    base.init(71.5, DegreesToRadians(0.0), DegreesToRadians(180.0));
    upperarm.init(125, DegreesToRadians(15.0), DegreesToRadians(165.0));
    forearm.init(125, DegreesToRadians(0.0), DegreesToRadians(180.0));
    hand.init(192, DegreesToRadians(0.0), DegreesToRadians(180.0));

    InverseK.attach(base, upperarm, forearm, hand);

    // HC-SR04 IO Setup
    pinMode(TrigPin, OUTPUT);  
    pinMode(EchoPin, INPUT);  
    
    // Colour Sensor IO Setup
    pinMode(ColourPinS0, OUTPUT);
    pinMode(ColourPinS1, OUTPUT);
    pinMode(ColourPinS2, OUTPUT);
    pinMode(ColourPinS3, OUTPUT);
    pinMode(ColourSensorOut, INPUT);
    digitalWrite(ColourPinS0, HIGH);
    digitalWrite(ColourPinS1, LOW);

    // Open Serial Comms
    Serial.begin(9600);

    // Set arm to upright stance
    ResetArm();

    //---Setup Complete---
    }

/**
 * @brief Resets the arm to a calibrated upright position.
 */
void ResetArm() {
    Braccio.ServoMovement(30, M1_REST_ANGLE, M2_REST_ANGLE, M3_REST_ANGLE, M4_REST_ANGLE, M5_REST_ANGLE, GRIPPER_OPEN);
    }
    
/**
 * @brief Converts an angle from degrees to radians.
 *
 * @param Degrees The angle in degrees.
 * @return float The angle in radians.
 */
float DegreesToRadians(float Degrees){
    return Degrees / 180.0 * PI - HALF_PI;
    }

/**
 * @brief Converts an angle from radians to degrees.
 *
 * @param Radians The angle in radians.
 * @return float The angle in degrees.
 */
float RadiansToDegrees(float Radians) {
    return (Radians + HALF_PI) * 180 / PI;
    }

/**
 * @brief Reads a single colour channel from the sensor.
 *
 * @param s2 The state for the S2 pin.
 * @param s3 The state for the S3 pin.
 * @param lowerBound The lower bound for mapping the frequency.
 * @param upperBound The upper bound for mapping the frequency.
 * @return int The mapped colour value.
 */
int ReadColourChannel(int s2, int s3, int lowerBound, int upperBound) {
    digitalWrite(ColourPinS2, s2);
    digitalWrite(ColourPinS3, s3);
    int frequency = pulseIn(ColourSensorOut, LOW);
    return map(frequency, upperBound, lowerBound, 255, 0);
}

/**
 * @brief Scans an object's colour and returns a corresponding integer code.
 *
 * @return int 1 for red, 2 for green, 3 for blue.
 */
int ColourScan(){

    // Move to 40 mm above colour sensor located at (-160, 0) with gripper closed
    GoTo(COLOUR_SENSOR_X, COLOUR_SENSOR_Y, COLOUR_SENSOR_Z, GRIPPER_CLOSED);

    int redValue = ReadColourChannel(LOW, LOW, RED_LOWER_BOUND, RED_UPPER_BOUND);
    int greenValue = ReadColourChannel(HIGH, HIGH, GREEN_LOWER_BOUND, GREEN_UPPER_BOUND);
    int blueValue = ReadColourChannel(LOW, HIGH, BLUE_LOWER_BOUND, BLUE_UPPER_BOUND);
    
    // ---Scanning Complete---
    // Select Lowest Colour Value & return 1, 2 or 3 for Red, Green or Blue 

    if(redValue < blueValue && redValue < greenValue){
        Serial.println("RED");
        return COLOUR_RED;
    }

    if(greenValue < blueValue && greenValue < redValue){
        Serial.println("GREEN");
        return COLOUR_GREEN;
    }

    if(blueValue < redValue && blueValue < greenValue){
     Serial.println("BLUE");
     return COLOUR_BLUE;
    }
    return 0; // Return 0 if no colour is dominant
    }

/**
 * @brief Measures the distance to an object using the ultrasonic sensor.
 *
 * @return float The measured distance in millimetres.
 */
float UltrasonicScan(){

    // Read echo time
    digitalWrite(TrigPin, LOW);  
    delayMicroseconds(2);  
    digitalWrite(TrigPin, HIGH);  
    delayMicroseconds(10);  
    digitalWrite(TrigPin, LOW); 
    float duration = pulseIn(EchoPin, HIGH);  

    // Convert time reading to distance, prints & returns distance. 
    float distance = 10*(duration*.0343)/2;  
    Serial.println(distance);
    return distance;
    }

/**
 * @brief Scans a given angular range for an object.
 *
 * @param startAngle The starting angle of the scan.
 * @param endAngle The ending angle of the scan.
 * @param offsets A reference to the struct to store the object's coordinates.
 * @return bool True if an object was found, false otherwise.
 */
bool ScanRange(int startAngle, int endAngle, OffsetValues &offsets) {
    const int radius = 40;
    int ObjectAngle = 0;
    for (int M1 = startAngle; M1 <= endAngle; M1 += 2) {
        //Rotate Base
        Braccio.ServoMovement(25, M1, M2_REST_ANGLE, M3_REST_ANGLE, M4_REST_ANGLE, M5_REST_ANGLE, GRIPPER_OPEN);
        delay(200);

        //Scan Distance, returns distance in MM
        float distance = UltrasonicScan();

        //Range Check, if < SCAN_RANGE_UPPER mm update angle and distance variables
        if (distance <= SCAN_RANGE_UPPER && distance >= SCAN_RANGE_LOWER) {
            ObjectAngle = M1 + LIDAR_CONE_OFFSET; // Adjust for LIDAR offset

            //Calculates and returns x/y co-ordinates
            offsets.XOffset = - (distance + radius) * cos(DegreesToRadians(ObjectAngle));
            offsets.YOffset = - (distance + radius) * sin(DegreesToRadians(ObjectAngle));
            return true;
        }
    }
    return false;
}

/**
 * @brief Scans the environment for the closest object and returns its coordinates.
 *
 * @return OffsetValues A struct containing the x and y coordinates of the detected object.
 */
OffsetValues FindObject() {
    OffsetValues offsets = {0.0f, 0.0f};

    if (ScanRange(0, 85, offsets)) {
        return offsets;
    }

    if (ScanRange(110, 180, offsets)) {
        return offsets;
    }

    return offsets; // Return {0,0} if no object is found
 }

/**
 * @brief Moves the robotic arm to a specified set of coordinates.
 *
 * @param x The target x-coordinate.
 * @param y The target y-coordinate.
 * @param z The target z-coordinate.
 * @param claw The desired state of the gripper (e.g., open or closed).
 */
void GoTo(float x, float y, float z, int claw){
    // ---Local Variables---
    float a0, a1, a2, a3;

    // Solve for given x, y, z co-ordinates, trying to approach from 45 degrees first
    bool solved = InverseK.solve(x, y, z, a0, a1, a2, a3, DegreesToRadians(45));
    if (!solved) {
        // If 45 degree approach is not possible, approach from any angle
        InverseK.solve(x, y, z, a0, a1, a2, a3);
    }

    // Move Servos to solved angles
    float M1 = RadiansToDegrees(a0);
    float M2 = RadiansToDegrees(a1);
    float M3 = RadiansToDegrees(a2);
    float M4 = RadiansToDegrees(a3);

    // Calibration:
    Braccio.ServoMovement(30, M1 + SERVO_M1_OFFSET, M2 + SERVO_M2_OFFSET, M3 + SERVO_M3_OFFSET, M4 + SERVO_M4_OFFSET, 90 + SERVO_M5_OFFSET, claw);
    }

/**
 * @brief Executes a sequence to grab an object at the specified coordinates.
 *
 * @param x The x-coordinate of the object.
 * @param y The y-coordinate of the object.
 * @param z The z-coordinate of the object.
 */
void GrabObject(float x, float y, float z){
    GoTo(x, y, z, GRIPPER_OPEN);
    GoTo(x, y, z, GRIPPER_CLOSED);
    GoTo(x, y, z + LIFT_HEIGHT, GRIPPER_CLOSED);
    }

/**
 * @brief Executes a sequence to drop an object at the specified coordinates.
 *
 * @param x The x-coordinate for the drop location.
 * @param y The y-coordinate for the drop location.
 * @param z The z-coordinate for the drop location.
 */
void DropObject(float x, float y, float z){
    GoTo(x, y, z + LIFT_HEIGHT, GRIPPER_CLOSED);
    GoTo(x, y, z, GRIPPER_CLOSED);
    GoTo(x, y, z, GRIPPER_OPEN);
    }

/**
 * @brief Sorts an object into a bin based on its detected colour.
 *
 * @param colour An integer representing the object's colour (1=red, 2=green, 3=blue).
 */
void SortObject(int colour){

    // Red bin (or if colour is unknown)
    if (colour == COLOUR_RED || colour == 0){
        DropObject(RED_BIN_X, RED_BIN_Y, RED_BIN_Z);
    }
    // Green Bin
    else if (colour == COLOUR_GREEN){
        DropObject(GREEN_BIN_X, GREEN_BIN_Y, GREEN_BIN_Z);
    }
    // Blue Bin
    else if (colour == COLOUR_BLUE){
        DropObject(BLUE_BIN_X, BLUE_BIN_Y, BLUE_BIN_Z);
    }
}

/**
 * @brief The main control loop that continuously finds, grabs, and sorts objects.
 */
void loop() {
    ResetArm();  

    //Find object, returns x/y co-ordinates
    OffsetValues offsets = FindObject(); 

    // Grab found object 
    GrabObject(offsets.XOffset, offsets.YOffset, OBJECT_PICKUP_Z);

    // Sort into colour bins based on ColourScan
    SortObject(ColourScan());

    }
