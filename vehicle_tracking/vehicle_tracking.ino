#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define INVALID_LATITUDE 200.0
#define INVALID_LONGITUDE 200.0
#define INVALID_ALTITUDE -1.0
#define ZERO_SATELLITE_CONNECTION 0
#define DEFAULT_BAUD_RATE 9600

const String EMERGENCY_CONTACT_NUM = "+4915566033783";  // Emergency contact number
const String OWNERS_CONTACT_NUM = "+4915566033783";     // Owner's contact number
const double INITIAL_LATITUDE = 50.137931;              // Center Latitude
const double INITIAL_LONGITUDE = 8.697117;              // Center Longitude
const double GEOFENCE_RADIUS = 10;                      // Radius in meters
const unsigned long SMS_INTERVAL = 10000;               // Send sms after every 10 seconds
const unsigned long MAX_CALL_DURATION = 30000;          // Max call duration in milliseconds (30 seconds)

TinyGPSPlus gps;
SoftwareSerial gpsSerial(8, 9);
SoftwareSerial gsmSerial(12, 13);

double latitude, longitude, altitude;
unsigned int noOfSatellites = ZERO_SATELLITE_CONNECTION;
int cnt = 0;
unsigned long previousAlertTime = 0;

int smsButtonPressed = 0;
int callButtonPressed = 0;

void setup() {
  latitude = INVALID_LATITUDE;
  longitude = INVALID_LONGITUDE;
  altitude = INVALID_ALTITUDE;
  previousAlertTime = millis();  // Intialize with current time in millisecond

  pinMode(4, INPUT);  // Button 1 for emergency SMS
  pinMode(5, INPUT);  // Button 2 for emergency call

  Serial.begin(DEFAULT_BAUD_RATE);
  gpsSerial.begin(DEFAULT_BAUD_RATE);
  gsmSerial.begin(DEFAULT_BAUD_RATE);
  gsmSerial.println("AT+CNMI=1,1,0,0,0");  // Decides how newly arrived SMS should be handled
  delay(5000);
  Serial.println("Initialization in setup() method...");
}

void loop() {
  getLocationData();

  // Serial.println(digitalRead(4));
  // Serial.println(digitalRead(5));
  // Handling emergency SMS (Button 1 press)
  if (digitalRead(4) == smsButtonPressed && isValidLocation()) {
    if (smsButtonPressed == 1) smsButtonPressed = 0;
    else smsButtonPressed = 1;
    Serial.println("SMS button is pressed!");
    sendAlert(latitude, longitude, "Alert! I am in danger and need help.\r", EMERGENCY_CONTACT_NUM);
    displayGPSInfo(latitude, longitude, noOfSatellites, altitude);
  }

  // Handling emergency Call (Button 2 press)
  if (digitalRead(5) == callButtonPressed) {
    if (callButtonPressed == 1) callButtonPressed = 0;
    else callButtonPressed = 1;
    Serial.println("call button is pressed!");
    initiateCall(EMERGENCY_CONTACT_NUM);
  }

  unsigned long currentTime = millis();

  // Checking if the bike is outside the geofence
  if (isValidLocation()
      && calculateDistance(latitude, longitude, INITIAL_LATITUDE, INITIAL_LONGITUDE) > GEOFENCE_RADIUS
      && (currentTime - previousAlertTime) > SMS_INTERVAL) {

    sendAlert(latitude, longitude, "Alert! The Bike is outside the geofence area.\r", OWNERS_CONTACT_NUM);
    displayGPSInfo(latitude, longitude, noOfSatellites, altitude);
    Serial.print("Distance: ");
    Serial.println(calculateDistance(latitude, longitude, INITIAL_LATITUDE, INITIAL_LONGITUDE), 6);
    previousAlertTime = currentTime;
  }
}

// Function to check valid location or not
bool isValidLocation() {
  return (latitude >= -90.0 && latitude <= 90.0) &&      // Latitude within range
         (longitude >= -180.0 && longitude <= 180.0) &&  // Longitude within range
         (altitude >= -500 && altitude <= 9000) &&       // Altitude within plausible range
         (noOfSatellites >= 3);                          // At least 3 satellites for a valid fix
}

// Function to calculate distance between two GPS coordinates (Haversine formula)
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;  // Earth's radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// Get location data (latitude, longitude)
void getLocationData() {
  // Initialize variables to track new GPS data
  bool newData = false;

  // Wait for up to 2000ms (2 seconds) for GPS data
  unsigned long timeout = 2000;
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    // Check if data is available from the GPS module
    while (gpsSerial.available()) {
      // Process the incoming GPS data
      if (gps.encode(gpsSerial.read())) {
        newData = true;
        break;
      }
    }

    // If new data is found, no need to wait further
    if (newData) {
      break;
    }
  }

  // If new data is available, retrieve latitude, longitude, no of connected satellites and altitude
  if (newData) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    noOfSatellites = gps.satellites.value();
    altitude = gps.altitude.meters();
  } else {
    // No GPS data is available, set defaults
    latitude = INVALID_LATITUDE;
    longitude = INVALID_LONGITUDE;
    noOfSatellites = ZERO_SATELLITE_CONNECTION;
    altitude = INVALID_ALTITUDE;
  }
}

// Function to display GPS information
void displayGPSInfo(double latitude, double longitude, double noOfSatellites, double altitude) {

  Serial.print("Latitude: ");
  Serial.println(latitude, 6);  // 6 decimal precision
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
  Serial.print("Satellites: ");
  Serial.println(noOfSatellites);
  Serial.print("Altitude (meters): ");
  Serial.println(altitude);
  Serial.println("----------------------------------");
}

// Function to send sms with google map link
void sendAlert(double latitude, double longitude, String alertMsg, String contactNum) {
  String sms_data = "";
  sms_data += alertMsg;
  sms_data += "http://maps.google.com/maps?q=loc:";
  sms_data += String(latitude, 6) + "," + String(longitude, 6);

  gsmSerial.print("AT+CMGF=1\r");
  delay(1000);
  gsmSerial.print("AT+CMGS=\"" + contactNum + "\"\r");
  delay(1000);
  gsmSerial.print(sms_data);
  delay(100);
  gsmSerial.write(0x1A);  //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
  gsmSerial.println("SMS Sent Successfully.");
}

void initiateCall(String contactNum) {
  // Send the command to initiate the call
  gsmSerial.print("ATD");
  gsmSerial.print(contactNum);
  gsmSerial.println(";");
  delay(100);
  Serial.println("Call initiated.");

  unsigned long startCallTime = millis();  // Record the time call started

  // Check for timeout
  while (true) {
    if (millis() - startCallTime > MAX_CALL_DURATION) {
      Serial.println("Call timeout: Ending call after 30 seconds.");
      gsmSerial.println("ATH");  // Send hang-up command
      delay(100);
      break;
    }
  }
  Serial.println("Call ended.");
}
