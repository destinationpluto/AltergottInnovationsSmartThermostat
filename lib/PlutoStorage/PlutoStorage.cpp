/*******************************************************************************
 *
 *		Pluto Setup
 *
 *         
 *
*******************************************************************************/

#include "PlutoStorage.h"

/**************************************************************************************************************/
/*                                                CONSTRUCTORS                                                */
/**************************************************************************************************************/
PlutoStorage::PlutoStorage()
{
    EEPROM.begin(sizeof(float));
}

/**************************************************************************************************************/
/*                                              PUBLIC METHODS                                                */
/**************************************************************************************************************/

bool PlutoStorage::saveTempSetpoint(float *f_setpoint)
{

    byte *data;
    data = (byte *)f_setpoint;
    for (size_t i = 0; i < sizeof(float); i++)
    {
        EEPROM.write(TOKENADDRESS + i, *(data + i));
    }

    bool result = EEPROM.commit();
    if (m_isDebug && result)
    {
        Serial.println("PlutoStorage: successfully saved setpoint");
    }
    return result;
}

float PlutoStorage::getTempSetpoint()
{

    float f;
    byte *data = (byte *)&f;

        for (size_t i = 0; i < sizeof(float); i++)
    {
        *(data + i) = EEPROM.read(TOKENADDRESS + i);
    }

    return f;
}

/**************************************************************************************************************/
/*                                              PRIVATE METHODS                                                */
/**************************************************************************************************************/
