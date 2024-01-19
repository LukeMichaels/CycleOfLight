#include <Arduino.h>
#include "constants.h"
#include "configuration.h"
#ifdef DEBUG
	#include <debug.h>
#endif
// #ifdef MENU
// 	#include "menu.h"
// #endif
#include "state.h"
#include "actions.h"

#define FRAMETIME 20;

//uint16_t touchdelay=9999;
unsigned long nextUpdate;

#include "patterns.h"


void setup() { 
    // Set the same bitrate in platformio.ini monitor_speed = xxx
    // or your serial console
	delay(50); // Serial Port delay
    Serial.begin(9600);
	delay(50); // Serial Port delay
    #ifdef DEBUG
	debugn('G',"Res");
	Serial.print("Res..");
	debugI('i');
	debugI(13.1f);
	showDebugOn();
	#endif
	
	setupPatterns();
	resetPattern();
	setupConfiguration();
	setupState();
	// #ifdef MENU
	// 	setupMenu();
	// #endif
	nextUpdate=millis()+FRAMETIME;
}

void loop() { 
	// #ifdef MENU
	// 	updateMenu();
	// #endif
	// 1 cycle equals approximately to 50ms. 
	void executeActions();
	displayPattern();
	//delay(20);
	// TODO: Stabilize Frametimes with Nextupdate
}
