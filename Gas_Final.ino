#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>

#define         MQ_PIN                       (0)     //FOR MQ8 SENSOR(A0)
const int gas_sensor = A1; //FOR MQ9 SENSOR
#define pin_a A2    //For MQ4 SENSOR


#define         RL_VALUE                     (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.21)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation                                                     //which is derived from the chart in datasheet
#define         GAS_H2                      (0)


float           H2Curve[3]  =  {2.3, 0.93, -1.44};   //two points are taken from the curve in datasheet.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, lg8.5), point2: (lg10000, lg0.03)

float           Ro           =  10;                  //Ro is initialized to 10 kilo ohms


const float m = -1.749; //Slope
const float b = 2.827; //Y-Intercept
const float R0 = 0.91; //Sensor Resistance in fresh air from previous code


ros::NodeHandle nh;

geometry_msgs::Point c_monoxide;
std_msgs::Float32 h2;
std_msgs::Int16 methane;

ros::Publisher chatter("GAS_SENSORS", &c_monoxide); //MQ9
ros::Publisher chatter1("GAS_SENSORS1", &h2); //MQ8
ros::Publisher chatter2("GAS_SENSORS2",&methane); //MQ4

void setup()
{ Ro = MQCalibration(MQ_PIN);
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(chatter1);
   nh.advertise(chatter2);
}

void loop() {
  float sensor_volt; //Define variable for sensor voltage
  float RS_gas; //Define variable for sensor resistance
  float ratio; //Define variable for ratio
  float sensorValue = analogRead(gas_sensor); //Read analog values of sensor
  sensor_volt = (sensorValue * (5.0 / 1023.0)); //Convert analog values to voltage
  RS_gas = ((5.0 * 10.0) / sensor_volt) - 10.0; //Get value of RS in a gas
  ratio = RS_gas / R0;   // Get ratio RS_gas/RS_air
  float ppm_log = (log10(ratio) - b) / m; //Get ppm value in linear scale according to the the ratio value
  float ppm = pow(10, ppm_log); //Convert ppm value to log scale
  float percentage = ppm / 10000; //Convert to percentage
   int value;
  int limit;
  
  value= analogRead(pin_a);//reads the analaog value from the methane sensor's AOUT pin

  
  c_monoxide.x = ppm_log;
  c_monoxide.y = ppm;
  c_monoxide.z = percentage;
  

  h2.data = (MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_H2));

  methane.data=value;

  chatter2.publish( &methane);

  chatter1.publish( &h2);

  chatter.publish( &c_monoxide);
  nh.spinOnce();
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}


float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}


float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}


int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_H2) {
    return MQGetPercentage(rs_ro_ratio, H2Curve);
  }
  return 0;
}


int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
