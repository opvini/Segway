// Wire library for the I2C communication with the MPU6050 (acc and gyro)
#include <Wire.h>


/* for debug popouses - monitor data via serial monitor */
#define DEBUG_SHOW_RAW    false
#define DEBUG_SHOW_ANG    false
#define DEBUG_SHOW_SPEED  false
#define DEBUG_SHOW_POS    false

/* define the current moviment */
#define SEGWAY_GOING_WRONG   0u
#define SEGWAY_GOING_CORRECT 1u


/* defines for moviments of motor */
#define MOVE_MOTOR_BACK  0u
#define MOVE_MOTOR_FRONT 1u
#define ROTATE_RIGHT     0u
#define ROTATE_LEFT      1u


/* Offsets for the angles */
#define OFFSET_ANG_X   86.6f
#define OFFSET_ANG_Y   -1.2f
#define OFFSET_ANG_Z  -87.1f


// I2C address of MPU6050
#define MPU_ADDRESS 0x68

// acelerometer sensibility according with the datasheet
// convert the raw data to g forces
#define ACC_SENSIBILITY  16384u

// gyroscope sensibility according with the datasheet
// convert the raw data to DPS (degrees per second)
#define GY_SENSIBILITY  131u

// alpha parameter for the Complementary Filter
// used to calculate the angle based on acc and gyro together
#define COMP_FILTER_ALPHA  0.5f

// constant to conver rad to degrees, used by atan2()
#define RAD_TO_DEGREES (float)(180/M_PI)

// define the g acceleration in [m/s^2]
#define G_ACCELERATION_M_S2   9.80665f

// define the g acceleration in [cm/ms^2]
#define G_ACCELERATION_CM_MS2 (G_ACCELERATION_M_S2*0.000001f)

// Calibrated data: 
// with the sensor in stand still 
// there is a deviation because the integration
// this is normaly constant and shall be removed
// from the integration clculation
#define GYRO_BIAS_X   -0.018f
#define GYRO_BIAS_Y   -0.0075f
#define GYRO_BIAS_Z   -0.04f

// define the pin of the H bridge for the motors
#define PIN_MOTOR_LEFT_POS 4u
#define PIN_MOTOR_LEFT_NEG 5u
#define PIN_MOTOR_LEFT_VEL 9u

#define PIN_MOTOR_RIGHT_POS  7u
#define PIN_MOTOR_RIGHT_NEG  6u
#define PIN_MOTOR_RIGHT_VEL  10u


// Variables to store the read data from the sensor
float AcX=0.f, AcY=0.f, AcZ=0.f, Tmp=0.f, GyX=0.f, GyY=0.f, GyZ=0.f;

// Variables to store the filtered calculated angles
float AngX=0.f, AngY=0.f, AngZ=0.f;

// Variables to store the position of the segway
float PosX=0.f, PosY=0.f, PosZ=0.f;

// Variables to store the position of the segway
float SpeedX=0.f, SpeedY=0.f, SpeedZ=0.f;
 
// auxiliar variables
int velocidade          = 0u;
int lastDirectionRotate = -1u;
int lastDirectionMotor  = -1u;
int statusAngleMoviment = SEGWAY_GOING_WRONG;




float dt=0.f, dt_ant=0.f;


/* ultimo sentido do motor */
int lastRota = -1u;

/* OffSets: média dos valores amostrados com o robô em cima da base fixa */
struct StructOffSet
{
  float AcX = -1.0f;
  float AcY =  0.03f;
  float AcZ = -0.04f;
  float Tmp =  22.46f;
  float GyX = -1.09f;
  float GyY =  0.34f;
  float GyZ = -0.67f;
} OffSet;

/* Definition of the Segway struct to store the states */
struct StructSegway{
  float pos_x; 
  float pos_y;   
  float pos_z; 
  float ang_x;  
  float ang_y;  
  float ang_z;    
  float speed_pos_x;
  float speed_pos_y;
  float speed_pos_z;  
  float ang_speed_x;
  float ang_speed_y;
  float ang_speed_z;
} Segway;



/* Params of PID */
float kp=2.4f, ki=0.008f, kd=0.f;
int   dt_cont   = 60u;
int   maxSatura = 150u;
int   minSatura = 63u;

float acceptableError       = 1.2f;
float acceptableInertialErr = 3.f;

bool showSensor=false, showControl=false;
bool led = 0;


/* OffSets: média dos valores amostrados com o robô em cima da base fixa */
struct StructTime
{
  unsigned long int ETsR        = 0L;
  unsigned long int lastTime    = 0L;
  unsigned long int elapsedTime = 0L;
  
  unsigned long int counter_2ms    = 0L;
  unsigned long int counter_5ms    = 0L;
  unsigned long int counter_10ms   = 0L;
  unsigned long int counter_100ms  = 0L;
  unsigned long int counter_1000ms = 0L;
  
} Timer;


/************************* COPY THE VALUES TO USE IN A TERMINAL AS INITIAL VALUES */
//
// 2.4 0.008 0 1000 1.5 63 150 0 0
//
/**********************************************************************************/


void setup()
{
  /* configure the pins of the motor */
  pinMode(PIN_MOTOR_LEFT_POS,OUTPUT);
  pinMode(PIN_MOTOR_LEFT_NEG,OUTPUT);
  pinMode(PIN_MOTOR_LEFT_VEL,OUTPUT);
    
  pinMode(PIN_MOTOR_RIGHT_POS,OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_NEG,OUTPUT);
  pinMode(PIN_MOTOR_LEFT_VEL,OUTPUT);

  digitalWrite(PIN_MOTOR_LEFT_POS,LOW);
  digitalWrite(PIN_MOTOR_LEFT_NEG,LOW);
  digitalWrite(PIN_MOTOR_RIGHT_POS,LOW);
  digitalWrite(PIN_MOTOR_RIGHT_NEG,LOW);

  /* LED */
  pinMode(13, OUTPUT);

  Serial.begin(9600);

  /* I2C communication with the MPU sensor */
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B); 
   
  /* init MPU-6050 */
  Wire.write(0); 
  Wire.endTransmission(true);

}


/*********************************************************** MOTOR HANDLING */


// MoveMotor()
// Check if it is needed to change the direction
// Directions: MOVE_MOTOR_FRONT or MOVE_MOTOR_BACK
// Intensity: PWM intensity [0..255]

void MoveMotor(int direction, int intensity)
{  

  static float lastAngle = AngZ;

  analogWrite(PIN_MOTOR_LEFT_VEL,  intensity);
  analogWrite(PIN_MOTOR_RIGHT_VEL, intensity);

  if( lastDirectionMotor != direction )
  {
    lastDirectionMotor = direction;
    
    if( MOVE_MOTOR_FRONT == direction )
    {
      digitalWrite(PIN_MOTOR_LEFT_NEG,  LOW);
      digitalWrite(PIN_MOTOR_RIGHT_NEG, LOW);
      delay(1);
      digitalWrite(PIN_MOTOR_LEFT_POS,  HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_POS, HIGH);
    }
    
    else if( MOVE_MOTOR_BACK == direction )
    {
      digitalWrite(PIN_MOTOR_LEFT_POS,  LOW);
      digitalWrite(PIN_MOTOR_RIGHT_POS, LOW);
      delay(1);
      digitalWrite(PIN_MOTOR_LEFT_NEG,  HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_NEG, HIGH);  
    }   

    /* Check if angle is in correction status or not */
    if( abs(lastAngle) < abs(AngZ) )
    {
      statusAngleMoviment = SEGWAY_GOING_CORRECT;
    }
    else
    {
      statusAngleMoviment = SEGWAY_GOING_WRONG;
    }

    lastAngle = AngZ;
    
  } // end   
} // end MoveMotor()



// ROTATE SEGWAY
// Possible values:
// ROTATE_RIGHT, ROTATE_LEFT

void RotateSegway(int direction, int intensity)
{ 

  analogWrite(PIN_MOTOR_LEFT_VEL,  intensity);
  analogWrite(PIN_MOTOR_RIGHT_VEL, intensity);

  if( lastDirectionRotate != direction )
  {
    lastDirectionRotate = direction;
    
    if( ROTATE_RIGHT == direction )
    {
      digitalWrite(PIN_MOTOR_LEFT_NEG,  LOW);
      digitalWrite(PIN_MOTOR_RIGHT_POS, LOW);
      delay(1);
      digitalWrite(PIN_MOTOR_LEFT_POS,  HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_NEG, HIGH);
    }
    
    else if( ROTATE_LEFT == direction )
    {
      digitalWrite(PIN_MOTOR_LEFT_POS,  LOW);
      digitalWrite(PIN_MOTOR_RIGHT_NEG, LOW);
      delay(1);
      digitalWrite(PIN_MOTOR_LEFT_NEG,  HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_POS, HIGH);  
    }   
    
  } // end   
} // end RotateSegway()



/* Turn off the motors */
void TurnOffMotor()
{
    /* turn off the FETs */
    digitalWrite(PIN_MOTOR_LEFT_POS,  LOW);
    digitalWrite(PIN_MOTOR_LEFT_NEG,  LOW);
    digitalWrite(PIN_MOTOR_RIGHT_POS, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_NEG, LOW);

    /* reinitialize the auziliar variables */
    lastDirectionRotate = -1u;
    lastDirectionMotor  = -1u;
}


/* On-Off controller */
void OnOffControl(int intensity)
{
  /* Turn off the motor if the moviment is in correction and the error is small */
  /* this compensate the inertia of the system */
  if( (SEGWAY_GOING_CORRECT == statusAngleMoviment) && (abs(AngZ) < acceptableInertialErr) )
  {
    statusAngleMoviment = SEGWAY_GOING_WRONG;
    TurnOffMotor();
  }
  else if( AngZ > acceptableError )
  {
    MoveMotor(MOVE_MOTOR_FRONT, intensity);
  }
  else if( AngZ < (-acceptableError) )
  {
    MoveMotor(MOVE_MOTOR_BACK, intensity);
  }

  /* Turn off the motor if the angle is less than the acceptable */
  if(abs(AngZ) < acceptableError)
  {
    statusAngleMoviment = SEGWAY_GOING_WRONG;
    TurnOffMotor();
  }

}

/* Basic control of Segway */
void ControlSegway()
{
  float OutProportional = abs(AngZ)* 5.28f + 38.66f;
  OnOffControl(OutProportional);
}


void PID(float kp, float ki, float kd, int td)
{
  unsigned long int saida      = 0;
  static float      integrador = 0;
  static float      lastError  = 0;
  static float      erro       = 0;
  float             derivativo = 0;

  lastError = erro;
  erro      = abs(AngZ - 90);

  /* calcula integrador e derivativo */
  if( erro < acceptableError )
  {
    integrador = 0;
  }
  else
  {
    integrador += (erro+lastError)*dt_cont/2;  /* integral usando regra do trapézio */
    derivativo = abs(erro-lastError)/dt_cont;
  }

  /* saída do PID */
  saida = (int)( erro*kp + integrador*ki + derivativo*kd);

  /* sturador */
  if( saida > maxSatura )
  {
    saida = maxSatura;
    integrador = maxSatura;
  }
  else if( saida < minSatura )
  {
    saida = minSatura;
  }
}


void receiveParams()
{
  String data;
  char resp[9][10];
  
  int j = 0;
  int k = 0;
  int i=0;
  
  if (Serial.available() > 0) 
  {
     data = Serial.readString(); 

    /* parse to floats */
    for(i=0; i<data.length(); i++)
    {
      if( data[i] == ' ' )
      { 
        resp[k][j] = '\0'; j=0; k++; 
      }
      else
      {
        resp[k][j++] = data[i];
      }
    }
    resp[k][j] = '\0';

    /* set the params */
    kp      = atof(resp[0]);
    ki      = atof(resp[1]);
    kd      = atof(resp[2]);  
    dt_cont = atoi(resp[3]);

    acceptableError = atof(resp[4]); 
    minSatura    = atoi(resp[5]);
    maxSatura    = atoi(resp[6]);
    showSensor   = atof(resp[7]);
    showControl  = atof(resp[8]);
     
    Serial.println();
    Serial.println("------------------------------------------");
    Serial.println("PARAMS SETTED: ");
    Serial.println("------------------------------------------");
    
    Serial.print("Kp: "); Serial.println(kp,5);
    Serial.print("Ki: "); Serial.println(ki,5);
    Serial.print("Kd: "); Serial.println(kd,5);
    Serial.print("Td: "); Serial.println(dt_cont);

    Serial.print("Erro Aceitavel: "); Serial.println(acceptableError, 5);
    Serial.print("minSatura: "); Serial.println(minSatura);
    Serial.print("maxSatura: "); Serial.println(maxSatura );   
    
    Serial.print("showSensor: ");  Serial.println(showSensor ); 
    Serial.print("showControl: "); Serial.println(showControl );  

    Serial.println("------------------------------------------");

    Serial.println("COPY:");
    
    Serial.print(kp,5);      Serial.print(" ");
    Serial.print(ki,5);      Serial.print(" ");
    Serial.print(kd,5);      Serial.print(" ");
    Serial.print(dt_cont);   Serial.print(" ");

    Serial.print(acceptableError, 5); Serial.print(" ");
    Serial.print(minSatura);       Serial.print(" ");
    Serial.print(maxSatura );      Serial.print(" ");   
    
    Serial.print(showSensor );  Serial.print(" ");
    Serial.println(showControl );  
    Serial.println("------------------------------------------"); 

    delay(5000);
  }

}



/*********************************************************** SERVICES */


/* MPU5060 - I2C communication 
 * Read the data from acelerometer, Gyro and calculate the angles 
 * Provide the data on AcX, AcY,GyX, GyY
*/
void ReadRawDataFromMPU()
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);               /* starting with register 0x3B (ACCEL_XOUT_H) */
  Wire.endTransmission(false);
  
  /* Read the data from the sensor */
  Wire.requestFrom(MPU_ADDRESS,14,true);  
  
  /********* Read the RAW data */
  AcX = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  
  // for some reason the GyX was changed with GyY
  GyY = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  GyX = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)


  /* Convert the acc data to G force
   * Convert the raw data of gyro to DPS (degrees per second)
   * Provide the read data on the global variables 
  */
  AcX = AcX/ACC_SENSIBILITY;    GyX = GyX/GY_SENSIBILITY;
  AcY = AcY/ACC_SENSIBILITY;    GyY = GyY/GY_SENSIBILITY;
  AcZ = AcZ/ACC_SENSIBILITY;    GyZ = GyZ/GY_SENSIBILITY; 
  
}


/* Calculate the mean for <samples> samples
 * of the raw values of the sensor
*/
void ReadMeanRawDataFromMPU(int samples)
{
  // Variables to store the read data from the sensor
  float LocalAcX=0.f, LocalAcY=0.f, LocalAcZ=0.f, LocalGyX=0.f, LocalGyY=0.f, LocalGyZ=0.f;
  int i=0u;

  /* calculate the mean */
  for(i=0u; i<samples; i++)
  {
    ReadRawDataFromMPU();
    LocalAcX += AcX;    LocalGyX += GyX;
    LocalAcY += AcY;    LocalGyY += GyY;
    LocalAcZ += AcZ;    LocalGyZ += GyZ; 
  }

  /* store the mean */
  AcX = LocalAcX/samples;   GyX = LocalGyX/samples;
  AcY = LocalAcY/samples;   GyY = LocalGyY/samples;
  AcZ = LocalAcZ/samples;   GyZ = LocalGyZ/samples;

  #if DEBUG_SHOW_RAW == true
    Serial.print(AcX,6); Serial.print("\t");
    Serial.print(AcY,6); Serial.print("\t");
    Serial.print(AcZ,6); Serial.println("\t");
  #endif
  
}


/*  Calculate the angles X, Y, Z
 *  First with a simple trigonometry and after
 *  use the acelerometer angles and gyroscope together
 *  with a Complementary Filter to calculate the final angles with precision
 *  More info: 
 *  http://www.starlino.com/imu_guide.html
 *  https://naylampmechatronics.com/blog/45_Tutorial-MPU6050-Aceler%C3%B3metro-y-Giroscopio.html
 */
void CalculateAnglesFromRawData()
{

  // flag used to initialize the angles in the Complementary Filter
  // with the accelerometer data
  static bool isAnglesInitialized = 0;

  // Variables to store the calculated angles from the accelerometer
  float AngAccX, AngAccY, AngAccZ;

  // delta time "dt" for the calculation of the angle using the gyroscope
  static float dt_ant_gy=0;
         float dt_gy=0;

  // variables to store the deviation of the gyroscope
  float DevGyX=0, DevGyY=0, DevGyZ=0;

  // variables to store the angle calculated from gyro
  float AngGyX=0, AngGyY=0, AngGyZ=0;


  // Calculate the angles using the acelerometer using 3D trigonometry
  // The resultant vector is R
  // using "3D pitagoras": R^2 = X^2 + Y^2 + Z^2
  // the angles are so calculated
  AngAccX = (atan2( AcX, sqrt( AcY*AcY + AcZ*AcZ ) ) * RAD_TO_DEGREES) + OFFSET_ANG_X;
  AngAccY = (atan2( AcY, sqrt( AcX*AcX + AcZ*AcZ ) ) * RAD_TO_DEGREES) + OFFSET_ANG_Y;
  AngAccZ = (atan2( sqrt( AcX*AcX + AcY*AcY ), AcZ ) * RAD_TO_DEGREES) + OFFSET_ANG_Z;

  // Calculate the angle using the gyroscope
  // use a delta time "dt", in microseconds
  // and convert to seconds, since we will use degrees per second
  dt_gy     = (micros() - dt_ant_gy)/1000000;
  dt_ant_gy = micros();

  // integrate the gyro data to calculate the angle 
  // exclude the calibrated bias observable in stand still 
  DevGyX = (GyX - GYRO_BIAS_X)*dt_gy;   AngGyX += DevGyX;
  DevGyY = (GyY - GYRO_BIAS_Y)*dt_gy;   AngGyY += DevGyY;
  DevGyZ = (GyZ - GYRO_BIAS_Z)*dt_gy;   AngGyZ += DevGyZ;

  //// Complementary Filter
  // uses 93% (alpha) of the calculated angle using the gyro
  // calculated from the last filtered calculated value
  // and uses 7% of the accelerometer angle
  // because the instant value from the gyro is more precise

  // initialize the angles with the accelerometer data
  if(false == isAnglesInitialized)
  {
    AngX = AngAccX;
    AngY = AngAccY;
    AngZ = AngAccZ;
    isAnglesInitialized = true;
  }

  // Apply the Complementary Filter
  // Calculate and makes available the calculated angles
  AngX = COMP_FILTER_ALPHA*( AngX+DevGyX ) + (1-COMP_FILTER_ALPHA)*(AngAccX);
  AngY = COMP_FILTER_ALPHA*( AngY+DevGyY ) + (1-COMP_FILTER_ALPHA)*(AngAccY);
  AngZ = COMP_FILTER_ALPHA*( AngZ+DevGyZ ) + (1-COMP_FILTER_ALPHA)*(AngAccZ);

  #if DEBUG_SHOW_ANG == true
    Serial.print(AngX,6); Serial.print("\t");
    Serial.print(AngY,6); Serial.print("\t");
    Serial.print(AngZ,6); Serial.println("\t");
  #endif

}


/* Calculate the speed and position X,Y,Z based on the accelerometers 
 * the speed is a simple integrate of the accelerate
 * the position is a double integration of the accelerate
*/
void CalculatePositionFromRawData()
{
  // delta time "dt" for the calculation of the angle using the gyroscope
  static float dt_ant_calc_pos=0.f;
         float dt_calc_pos=0.f;
  
  // Calculate the angle using the gyroscope
  // use a delta time "dt", in miliseconds
  dt_calc_pos     = (micros() - dt_ant_calc_pos)/1000;
  dt_ant_calc_pos = micros();

  // calculate the speedy in X,Y,Z in cm/ms
  // which is a simple integration of the accelaration
  SpeedX += (G_ACCELERATION_CM_MS2 * (AcX-AngX))*dt_calc_pos;
  SpeedY += (G_ACCELERATION_CM_MS2 * AcY)*dt_calc_pos;
  SpeedZ += (G_ACCELERATION_CM_MS2 * AcZ)*dt_calc_pos;

  // calculate the position which is a simple integration of the speed
  // in cm
  PosX += SpeedX*dt_calc_pos;
  PosY += SpeedY*dt_calc_pos;
  PosZ += SpeedZ*dt_calc_pos;


  #if DEBUG_SHOW_SPEED == true
    Serial.print(SpeedX,6); Serial.print("\t");
    Serial.print(SpeedY,6); Serial.print("\t");
    Serial.print(SpeedZ,6); Serial.println("\t");
  #endif

  #if DEBUG_SHOW_POS == true
    Serial.print(PosX,6); Serial.print("\t");
    Serial.print(PosY,6); Serial.print("\t");
    Serial.print(PosZ,6); Serial.println("\t");
  #endif
}


/* Update the Segway states such as position, velocity */
void UpdateSegwayStates()
{
  // update the angles states with the filtered calculated angles
  Segway.pos_x = PosX;
  Segway.pos_y = PosY;
  Segway.pos_z = PosZ;
  
  // update the angles states with the filtered calculated angles
  Segway.ang_x = AngX;
  Segway.ang_y = AngY;
  Segway.ang_z = AngZ;

  // update the angle speed
  Segway.ang_speed_x = GyX;
  Segway.ang_speed_y = GyY;
  Segway.ang_speed_z = GyZ;
}


void PrintSegwayStates()
{
  //Serial.println(AcX*G_ACCELERATION_M_S2, 10);   
}


/****************************************************** SCHEDULER */

void Task_2ms()
{
  /* each package takes 2ms. 10 measurements takes 20ms */
  ReadMeanRawDataFromMPU(10);
  CalculateAnglesFromRawData();
  //CalculatePositionFromRawData();
  //UpdateSegwayStates();
  //ControlSegway();

  led = !led;
  if(led == 0) digitalWrite(13,LOW);
  else         digitalWrite(13,HIGH);
}

void Task_5ms(){}
void Task_10ms(){}

void Task_100ms()
{
  PrintSegwayStates();
}

void Task_1000ms(){}





/****************************************************** TIMER HANDLER */

/* calculate and provide the elapsed time */
void UpdateTimer()
{
  Timer.lastTime    = Timer.ETsR;
  Timer.ETsR        = millis();
  Timer.elapsedTime = (Timer.ETsR - Timer.lastTime);
}

/* run the tasks: 2ms, 5ms, 10ms, 100ms and 1000ms */
void RunScheduler()
{

  // increment counters with the elapsed time in [ms]
  Timer.counter_2ms    += Timer.elapsedTime;
  Timer.counter_5ms    += Timer.elapsedTime;
  Timer.counter_10ms   += Timer.elapsedTime;
  Timer.counter_100ms  += Timer.elapsedTime;
  Timer.counter_1000ms += Timer.elapsedTime;
  
  // execute tasks and reset counters
  if( Timer.counter_2ms >= 2)
  {
    Task_2ms();
    Timer.counter_2ms = 0;
  }
  
  if( Timer.counter_5ms >= 5){
    Task_5ms();
    Timer.counter_5ms  = 0;
  }
  
  if( Timer.counter_10ms >= 10){
    Task_10ms();
    Timer.counter_10ms = 0;
  }

  if( Timer.counter_100ms >= 100){
    Task_100ms();
    Timer.counter_100ms = 0;
  }

  if( Timer.counter_1000ms >= 1000){
    Task_1000ms();
    Timer.counter_1000ms = 0;
  }
  
} // end RunScheduler()





/****************************************************** LOOP */

void loop()
{   
  
  UpdateTimer();
  RunScheduler();

//  MoveMotor(MOVE_MOTOR_BACK, 120);
//  delay(5000);
//  MoveMotor(MOVE_MOTOR_FRONT, 120);
//  delay(5000);

//  receiveParams();
//  
//  readMPU();
//  if(showSensor) printMPU();

//  printAngles();
//  MoveMotor(1, 120);
//  delay(5000);
//  MoveMotor(0, 120);
//  delay(5000);
//  rotateSegway(0,120);
//  delay(5000);
//  rotateSegway(1,120);
//  delay(5000);
  
  //PID(kp,ki,kd,dt_cont);

}
