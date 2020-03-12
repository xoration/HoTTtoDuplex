// Copyright (C) 2020 Stefan Seifert
// 
// This file is part of HoTTJeti.
// 
// HoTTJeti is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HoTTJeti is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with HoTTJeti.  If not, see <http://www.gnu.org/licenses/>.



#include "HoTTTelemetryProtocol.h"
#include <SoftwareSerial.h>
#include <arduino.h>
#include "ElapsedMillis.h"
#define RELEASE
#ifdef RELEASE
#include "JetiExProtocol.h"
#endif 


#define HOTTV4_RXTX 6

//  Left = 0x70, down = 0xb0, up= 0xd0, right = 0xe0
#define JETI_LEFT 0x70
#define JETI_RIGHT 0xe0
#define JETI_LEFT_RIGHT 0x60
#define JETI_UP 0xd0
#define JETI_DOWN 0xb0

bool allowEdit = false;
bool textmode = false;

int firstRun = 1;

elapsedMillis graupnerBinaryTimer;
elapsedMillis graupnerTextmodeTimer;

char display[8][22];

void HandleTextmode();
void CreateJetiLines(int linePosition, int displayPosition, bool setmode, char *line1, char *line2, int lineLength);
void PollGraupner(HoTTCommand command, bool immediately);

HoTTTelemetryProtocol hott;
HoTTSensor sensor = HoTTSensor::None;

#ifdef RELEASE
JetiExProtocol jetiEx;
#endif

// EAM
enum
{
	ID_EAM_CAPACITY = 1,
	ID_EAM_CURRENT,
	ID_EAM_VOLTAGE_1,
	ID_EAM_VOLTAGE_2,
	ID_EAM_MAIN_VOLTAGE,
	ID_EAM_CELL_VOLTAGE_1,
	ID_EAM_CELL_VOLTAGE_2,
	ID_EAM_CELL_VOLTAGE_3,
	ID_EAM_CELL_VOLTAGE_4,
	ID_EAM_CELL_VOLTAGE_5,
	ID_EAM_CELL_VOLTAGE_6,
	ID_EAM_CELL_VOLTAGE_7,
	ID_EAM_CELL_VOLTAGE_8,
	ID_EAM_CELL_VOLTAGE_9,
	ID_EAM_CELL_VOLTAGE_10,
	ID_EAM_CELL_VOLTAGE_11,
	ID_EAM_CELL_VOLTAGE_12,
	ID_EAM_CELL_VOLTAGE_13,
	ID_EAM_CELL_VOLTAGE_14,
	ID_EAM_ALTITUDE,
	ID_EAM_CLIMBRATE,
	ID_EAM_TEMPERATURE_1,
	ID_EAM_TEMPERATURE_2,
	ID_EAM_RPM,
};

// GAM
enum
{
	ID_GAM_CAPACITY = 1,
	ID_GAM_CURRENT,
	ID_GAM_VOLTAGE_1,
	ID_GAM_VOLTAGE_2,
	ID_GAM_MAIN_VOLTAGE,
	ID_GAM_CELL_VOLTAGE_1,
	ID_GAM_CELL_VOLTAGE_2,
	ID_GAM_CELL_VOLTAGE_3,
	ID_GAM_CELL_VOLTAGE_4,
	ID_GAM_CELL_VOLTAGE_5,
	ID_GAM_CELL_VOLTAGE_6,
	ID_GAM_ALTITUDE,
	ID_GAM_CLIMBRATE,
	ID_GAM_FUEL_PERCENTAGE,
	ID_GAM_FUEL,
	ID_GAM_TEMPERATURE_1,
	ID_GAM_TEMPERATURE_2,
	ID_GAM_RPM,
};

// GPS
enum
{
	ID_GPS_ALTITUDE = 1,
	ID_GPS_CLIMBRATE,
	ID_GPS_SPEED,
	ID_GPS_DISTANCE,
	ID_GPS_DIRECTION,
	ID_GPS_LOGITUDE_CURRENT_POSITION,
	ID_GPS_LATITUDE_CURRENT_POSITION,
};

// VARIO
enum
{
	ID_VARIO_ALTITUDE = 1,
	ID_VARIO_MAXALTITUDE,
	ID_VARIO_MINALTITUDE,
	ID_VARIO_CLIMBRATE,
};

// AIR-ESC
enum
{
	ID_AIRESC_CAPACITY = 1,
	ID_AIRESC_CURRENT,
	ID_AIRESC_MAIN_VOLTAGE,
	ID_AIRESC_ESC_TEMPERATURE,
	ID_AIRESC_RPM,
};

#ifdef RELEASE
// EAM
JETISENSOR_CONST sensors_eam[] PROGMEM = {
	//    id                      name               unit        data type             precision 0->0, 1->0.0, 2->0.00
	{ ID_EAM_CAPACITY,        "Capacity",        "mAh",      JetiSensor::TYPE_22b, 0 },
	{ ID_EAM_CURRENT,         "Current",         "A",        JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_VOLTAGE_1,       "Voltage 1",       "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_VOLTAGE_2,       "Voltage 2",       "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_MAIN_VOLTAGE,    "Main voltage",    "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_CELL_VOLTAGE_1,  "Cell 1 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_2,  "Cell 2 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_3,  "Cell 3 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_4,  "Cell 4 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_5,  "Cell 5 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_6,  "Cell 6 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_7,  "Cell 7 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_8,  "Cell 8 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_9,  "Cell 9 voltage",  "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_10, "Cell 10 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_11, "Cell 11 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_12, "Cell 12 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_13, "Cell 13 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_CELL_VOLTAGE_14, "Cell 14 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_EAM_ALTITUDE,        "Rel. altitude",   "m",        JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_CLIMBRATE,       "Vario",           "m/s",      JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_TEMPERATURE_1,   "Temperature 1",   "\xB0\x43", JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_TEMPERATURE_2,   "Temperature 2",   "\xB0\x43", JetiSensor::TYPE_14b, 1 },
	{ ID_EAM_RPM,             "RPM",             "/min",     JetiSensor::TYPE_22b, 1 },
	0 // end of array
};

// GAM
JETISENSOR_CONST sensors_gam[] PROGMEM = {
	//    id                      name              unit        data type             precision 0->0, 1->0.0, 2->0.00
	{ ID_GAM_CAPACITY,        "Capacity",       "mAh",      JetiSensor::TYPE_22b, 0 },
	{ ID_GAM_CURRENT,         "Current", 	    "A",        JetiSensor::TYPE_14b, 1 },
	{ ID_GAM_VOLTAGE_1,       "Voltage 1",      "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_GAM_VOLTAGE_2,       "Voltage 2", 	    "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_GAM_MAIN_VOLTAGE,    "Main voltage",   "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_GAM_CELL_VOLTAGE_1,  "Cell 1 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_GAM_CELL_VOLTAGE_2,  "Cell 2 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_GAM_CELL_VOLTAGE_3,  "Cell 3 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_GAM_CELL_VOLTAGE_4,  "Cell 4 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_GAM_CELL_VOLTAGE_5,  "Cell 5 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_GAM_CELL_VOLTAGE_6,  "Cell 6 voltage", "V",        JetiSensor::TYPE_14b, 2 },
	{ ID_GAM_ALTITUDE,        "Rel. altitude",  "m",        JetiSensor::TYPE_14b, 1 },
	{ ID_GAM_CLIMBRATE,       "Vario",          "m/s",      JetiSensor::TYPE_14b, 1 },
	{ ID_GAM_FUEL_PERCENTAGE, "Fuel percent",   "%",        JetiSensor::TYPE_14b, 0 },
	{ ID_GAM_FUEL,            "Fuel",           "ml",       JetiSensor::TYPE_22b, 1 },
	{ ID_GAM_TEMPERATURE_1,   "Temperature 1",  "\xB0\x43", JetiSensor::TYPE_14b, 0 },
	{ ID_GAM_TEMPERATURE_2,   "Temperature 2",  "\xB0\x43", JetiSensor::TYPE_14b, 0 },
	{ ID_GAM_RPM,             "RPM",            "/min",     JetiSensor::TYPE_22b, 0 },
	0 // end of array
};

// GPS
JETISENSOR_CONST sensors_gps[] PROGMEM = {
	//    id                                name               unit    data type             precision 0->0, 1->0.0, 2->0.00
	{ ID_GPS_ALTITUDE,                  "Rel. altitude",   "m",    JetiSensor::TYPE_14b, 1 },
	{ ID_GPS_CLIMBRATE,                 "Vario",           "m/s",  JetiSensor::TYPE_14b, 1 },
	{ ID_GPS_SPEED,                     "Speed",           "km/h", JetiSensor::TYPE_14b, 0 },
	{ ID_GPS_DISTANCE,                  "Distance",        "m",    JetiSensor::TYPE_22b, 1 },
	{ ID_GPS_DIRECTION,                 "Direction",       "\xB0", JetiSensor::TYPE_14b, 0 },
	{ ID_GPS_LOGITUDE_CURRENT_POSITION, "Log. curr. pos.", "-",    JetiSensor::TYPE_GPS, 0 },
	{ ID_GPS_LATITUDE_CURRENT_POSITION, "Lat. curr. pos.", "-",    JetiSensor::TYPE_GPS, 0 },
	0 // end of array
};

// VARIO
JETISENSOR_CONST sensors_vario[] PROGMEM = {
	//    id                  name             unit   data type             precision 0->0, 1->0.0, 2->0.00
	{ ID_VARIO_ALTITUDE,  "Rel. altitude", "m",   JetiSensor::TYPE_14b, 1 },
	{ ID_VARIO_MAXALTITUDE, "Max altitude",         "m", JetiSensor::TYPE_14b, 1 },
	{ ID_VARIO_MINALTITUDE, "Min altitude",         "m", JetiSensor::TYPE_14b, 1 },
	{ ID_VARIO_CLIMBRATE, "Vario",         "m/s", JetiSensor::TYPE_14b, 1 },
	0 // end of array
};

// AIR-ESC
JETISENSOR_CONST sensors_airesc[] PROGMEM = {
	//    id                         name            unit        data type             precision 0->0, 1->0.0, 2->0.00
	{ ID_AIRESC_CAPACITY,        "Capacity",     "mAh",      JetiSensor::TYPE_22b, 0 },
	{ ID_AIRESC_CURRENT,         "Current",      "A",        JetiSensor::TYPE_14b, 1 },
	{ ID_AIRESC_MAIN_VOLTAGE,    "Main voltage", "V",        JetiSensor::TYPE_14b, 1 },
	{ ID_AIRESC_ESC_TEMPERATURE, "ESC Temp.",    "\xB0\x43", JetiSensor::TYPE_14b, 0 },
	{ ID_AIRESC_RPM,             "RPM",          "/min",     JetiSensor::TYPE_22b, 0 },
	0 // end of array
};
#endif

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);

	delay(3000);

	hott.Init(HOTTV4_RXTX);

#ifdef RELEASE
	while (sensor == HoTTSensor::None) {
		digitalWrite(LED_BUILTIN, HIGH);
		/*
		if (hott.PollSensorBinaryMode(HoTTSensor::EAM)) {
			sensor = HoTTSensor::EAM;
			jetiEx.Start("HoTT EAM", sensors_eam);
		}
		else if (hott.PollSensorBinaryMode(HoTTSensor::GAM)) {
			sensor = HoTTSensor::GAM;
			jetiEx.Start("HoTT GAM", sensors_gam);
		}
		else if (hott.PollSensorBinaryMode(HoTTSensor::GPS)) {
			sensor = HoTTSensor::GPS;
			jetiEx.Start("HoTT GPS", sensors_gps);
		}
		else*/
    if (hott.PollSensorBinaryMode(HoTTSensor::VARIO)) {
			sensor = HoTTSensor::VARIO;
			jetiEx.Start("HoTT Vario", sensors_vario);
		}
		else if (hott.PollSensorBinaryMode(HoTTSensor::AIRESC)) {
			sensor = HoTTSensor::AIRESC;
			jetiEx.Start("HoTT ESC", sensors_airesc);
		}
		digitalWrite(LED_BUILTIN, LOW);
		delay(500);
	}
#else
	Serial.begin(9600);
	Serial.println("Start Debug.");

	while (sensor == HoTTSensor::None) {
		digitalWrite(LED_BUILTIN, HIGH);

		Serial.println("Checking EAM.");
		if (hott.PollSensorBinaryMode(HoTTSensor::EAM)) {
			sensor = HoTTSensor::EAM; 
			Serial.println("Found EAM");
		}

		Serial.println("Checking GAM.");
		if (hott.PollSensorBinaryMode(HoTTSensor::GAM)) {
			sensor = HoTTSensor::GAM;
			Serial.println("Found GAM");
		}

		Serial.println("Checking GPS.");
		if (hott.PollSensorBinaryMode(HoTTSensor::GPS)) {
			sensor = HoTTSensor::GPS;
			Serial.println("Found GPS");
		}

		Serial.println("Checking VARIO.");
		if (hott.PollSensorBinaryMode(HoTTSensor::VARIO)) {
			sensor = HoTTSensor::VARIO;
			Serial.println("Found VARIO");
		}

		Serial.println("Checking AIRESC.");
		if (hott.PollSensorBinaryMode(HoTTSensor::AIRESC)) {
			sensor = HoTTSensor::AIRESC;
			Serial.println("Found AIRESC");
		}
		digitalWrite(LED_BUILTIN, LOW);
		delay(500);
	}

#endif // RELEASE





	//jetiEx.SetDeviceId(0x76, 0x32);
#ifdef RELEASE
	jetiEx.SetJetiboxText(JetiExProtocol::LINE1, "HoTT Converter 1.0");
#endif // RELEASE

	graupnerBinaryTimer = 0;
	graupnerTextmodeTimer = 0;

	digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
#ifdef RELEASE
	if (graupnerBinaryTimer > HOTT_BINARY_INTERVAL && !textmode) {
		graupnerBinaryTimer = 0;
		if (hott.PollSensorBinaryMode(sensor)) {
			switch (sensor) {
			case HoTTSensor::VARIO:
				jetiEx.SetSensorValue(1, (static_cast<int32_t>(hott.HoTTPacketBinary.Vario.altitude) - 500) * 10);
				jetiEx.SetSensorValue(2, static_cast<int32_t>(hott.HoTTPacketBinary.Vario.maxAltitude));
				jetiEx.SetSensorValue(3, static_cast<int32_t>(hott.HoTTPacketBinary.Vario.minAltitude));

				if (hott.HoTTPacketBinary.Vario.m1s > 30000) {
					jetiEx.SetSensorValue(4, static_cast<int32_t>((hott.HoTTPacketBinary.Vario.m1s - 30000) * 0.01));
				}
				else {
					jetiEx.SetSensorValue(4, static_cast<int32_t>(hott.HoTTPacketBinary.Vario.m1s * 0.01));
				}
				break;

			case HoTTSensor::AIRESC:
				jetiEx.SetSensorValue(ID_AIRESC_CAPACITY, hott.HoTTPacketBinary.AirEsc.Capacity);
				jetiEx.SetSensorValue(ID_AIRESC_CURRENT, hott.HoTTPacketBinary.AirEsc.Current);
				jetiEx.SetSensorValue(ID_AIRESC_MAIN_VOLTAGE, hott.HoTTPacketBinary.AirEsc.InputVolt);
				jetiEx.SetSensorValue(ID_AIRESC_ESC_TEMPERATURE, hott.HoTTPacketBinary.AirEsc.EscTemperature);
				jetiEx.SetSensorValue(ID_AIRESC_RPM, hott.HoTTPacketBinary.AirEsc.RPM);
				break;
			}
		}
	}

	HandleTextmode();

	jetiEx.DoJetiSend();
#endif // RELEASE
}




void HandleTextmode()
{
	static bool editMode = false;
	static bool active = false;


	static int linePosition = 0;
	static int displayPosition = 0;
	static char line1[16] = { 0 };
	static char line2[16] = { 0 };


	uint8_t c = 0;

	//c = jetiEx.GetJetiboxKey();


	if (c == JETI_DOWN) // Down
	{
		if (!active) {
			digitalWrite(13, HIGH);
			active = true;
			textmode = true;
			PollGraupner(HoTTCommand::NOTHING, true);
		}
		else if (editMode)
		{
			PollGraupner(HoTTCommand::DOWN, true);
		}
		else if (linePosition == 7)
		{
			PollGraupner(HoTTCommand::NEXT_PAGE, true);
			linePosition = 0;
			displayPosition++;
		}
		else if (linePosition < 7) {
			linePosition++;
		}
	}

	if (c == JETI_UP) // Up
	{
		if (editMode)
		{
			PollGraupner(HoTTCommand::UP, true);
		}
		else if (linePosition == 0 && displayPosition > 0)
		{
			PollGraupner(HoTTCommand::PREVIOUS_PAGE, true);
			linePosition = 7;
			displayPosition--;
		}
		else if (linePosition > 0) {
			linePosition--;
		}
		else if (linePosition == 0 && displayPosition == 0)
		{
			active = false;
			textmode = false;
			digitalWrite(13, LOW);
		}
	}

	if (c == JETI_LEFT) // Down
	{
		editMode = !editMode;
	}

	if (c == JETI_RIGHT) // Up
	{
		if (editMode)
		{
			PollGraupner(HoTTCommand::SET, true);
		}
	}
	else if (active)
	{
		PollGraupner(HoTTCommand::NOTHING, false);
	}
	else
	{
		return;
	}


	CreateJetiLines(linePosition, displayPosition, editMode, line1, line2, sizeof(line1));

#ifdef RELEASE
	jetiEx.SetJetiboxText(JetiExProtocol::LINE1, line1);
	jetiEx.SetJetiboxText(JetiExProtocol::LINE2, line2);
#endif // RELEASE
}

void PollGraupner(HoTTCommand command, bool immediately)
{
	if (graupnerTextmodeTimer > HOTT_TEXT_INTERVAL || immediately)
	{
		graupnerTextmodeTimer = 0;
		hott.PollSensorTextMode(HoTTSensor::VARIO, command);
	}
}

static void CreateJetiLines(int linePosition, int displayPosition, bool editMode, char *line1, char *line2, int lineLength)
{
	static char localBuffer[32] = { 0 };

	memset(localBuffer, 0x00, sizeof(localBuffer));
	memcpy(localBuffer, (const void*)hott.HoTTPacketText.TextMode.display[linePosition], 21);

	bool first = true;

	for (int i = 0; i < 32; i++)
	{

		if (localBuffer[i] & 0x80)
		{
			if (!(localBuffer[i - 1] & 0x80) && first)
			{
				localBuffer[i - 1] = 0x7e;
				first = false;
			}

			if (!(localBuffer[i + 1] & 0x80))
			{
				localBuffer[i + 1] = 0x7e;
			}
		}

		localBuffer[i] = localBuffer[i] & ~0x80;
	}

	if (localBuffer[0] == '>')
	{
		allowEdit = true;
	}
	else
	{
		allowEdit = false;
	}

	// Draw Jeti Lines
	String lineComplete = "L" + String(linePosition);
	lineComplete += "D" + String(displayPosition);
	lineComplete += editMode;
	lineComplete += ":";
	lineComplete += String(localBuffer);


	lineComplete.substring(0, 16).toCharArray(line1, lineLength);
	lineComplete.substring(15).toCharArray(line2, lineLength);
}




