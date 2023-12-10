/*******************************************************************************
 *
 *		Pluto Setup Configuration
 *
 *              
 *
*******************************************************************************/

#ifndef PlutoStorage_H
#define PlutoStorage_H

#include "Arduino.h"
#include "EEPROM.h"

// Class Prototype
class PlutoStorage
{
public:
    // Public Constructors
    PlutoStorage();

    // Public Methods

    bool saveTempSetpoint(float* f_setpoint);
    float getTempSetpoint();

private:
    // Private Variables
    bool m_isDebug = true;
    String m_plutotoken = "";
    const uint8_t EEPROMSIZE = 4;
    const int TOKENADDRESS = 0;
    
};

#endif /* PlutoWiFi */
