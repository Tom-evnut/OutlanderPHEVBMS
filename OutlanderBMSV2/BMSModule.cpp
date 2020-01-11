#include "config.h"
#include "BMSModule.h"
#include "BMSUtil.h"
#include "Logger.h"


BMSModule::BMSModule()
{
  for (int i = 0; i < 8; i++)
  {
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  exists = false;
  reset = false;
  moduleAddress = 0;
  timeout = 30000; //milliseconds before comms timeout;
}

void BMSModule::clearmodule()
{
  for (int i = 0; i < 8; i++)
  {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

void BMSModule::decodecan(int Id, CAN_message_t &msg)
{
  switch (Id)
  {
    case 0x1:
      balstat = msg.buf[0];
      temperatures[0] = (msg.buf[2] * 256 + msg.buf[3]) * tempconv + tempoff;
      temperatures[1] = (msg.buf[4] * 256 + msg.buf[5]) * tempconv + tempoff;
      temperatures[2] = (msg.buf[6] * 256 + msg.buf[7]) * tempconv + tempoff;
      break;

    case 0x3:
      if (float((msg.buf[0] * 256 + msg.buf[1]) * 0.001) > IgnoreCell && float((msg.buf[0] * 256 + msg.buf[1]) * 0.001) < 60.0)
      {
        cellVolt[4] = float((msg.buf[0] * 256 + msg.buf[1]) * 0.001);
        cmuerror = 0;
      }
      else
      {

      }

      if (float((msg.buf[2] * 256 + msg.buf[3]) * 0.001) > IgnoreCell && float((msg.buf[2] * 256 + msg.buf[3]) * 0.001) < 60.0)
      {
        cellVolt[5] = float((msg.buf[2] * 256 + msg.buf[3]) * 0.001);
        cmuerror = 0;
      }
      else
      {

      }

      if (float((msg.buf[4] * 256 + msg.buf[5]) * 0.001) > IgnoreCell && float((msg.buf[4] * 256 + msg.buf[5]) * 0.001) < 60.0)
      {
        cellVolt[6] = float((msg.buf[4] * 256 + msg.buf[5]) * 0.001);
        cmuerror = 0;
      }
      else
      {

      }

      if (float((msg.buf[6] * 256 + msg.buf[7]) * 0.001) > IgnoreCell && float((msg.buf[6] * 256 + msg.buf[7]) * 0.001) < 60.0)
      {
        cellVolt[7] = float((msg.buf[6] * 256 + msg.buf[7]) * 0.001);
        cmuerror = 0;
      }
      else
      {

      }

      break;

    case 0x2:
      if (float((msg.buf[0] * 256 + msg.buf[1]) * 0.001) > IgnoreCell && float((msg.buf[0] * 256 + msg.buf[1]) * 0.001) < 60.0)
      {
        cellVolt[0] = float((msg.buf[0] * 256 + msg.buf[1]) * 0.001);
        cmuerror = 0;
      }
      else
      {
        cmuerror = 1;
      }

      if (float((msg.buf[2] * 256 + msg.buf[3]) * 0.001) > IgnoreCell && float((msg.buf[2] * 256 + msg.buf[3]) * 0.001) < 60.0)
      {
        cellVolt[1] = float((msg.buf[2] * 256 + msg.buf[3]) * 0.001);
        cmuerror = 0;
      }
      else
      {
        cmuerror = 1;
      }

      if (float((msg.buf[4] * 256 + msg.buf[5]) * 0.001) > IgnoreCell && float((msg.buf[4] * 256 + msg.buf[5]) * 0.001) < 60.0)
      {
        cellVolt[2] = float((msg.buf[4] * 256 + msg.buf[5]) * 0.001);
        cmuerror = 0;
      }
      else
      {
        cmuerror = 1;
      }

      if (float((msg.buf[6] * 256 + msg.buf[7]) * 0.001) > IgnoreCell && float((msg.buf[6] * 256 + msg.buf[7]) * 0.001) < 60.0)
      {
        cellVolt[3] = float((msg.buf[6] * 256 + msg.buf[7]) * 0.001);
        cmuerror = 0;
      }
      else
      {
        cmuerror = 1;
      }

      break;

    default:

      break;
  }
  if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
  if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();

  for (int i = 0; i < 8; i++)
  {
    if (lowestCellVolt[i] > cellVolt[i] && cellVolt[i] >= IgnoreCell)
    {
      lowestCellVolt[i] = cellVolt[i];
    }
    if (highestCellVolt[i] < cellVolt[i])
    {
      highestCellVolt[i] = cellVolt[i];
    }
  }

  if (cmuerror == 0)
  {
    lasterror = millis();
  }
  else
  {
    if (millis() - lasterror < timeout)
    {
      if (lasterror + timeout - millis() < 5000)
      {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("Module");
        SERIALCONSOLE.print(moduleAddress);
        SERIALCONSOLE.print("Counter Till Can Error : ");
        SERIALCONSOLE.println(lasterror + timeout - millis() );
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        cellVolt[i] = 0.0f;
      }
      moduleVolt = 0.0f;
      temperatures[0] = 0.0f;
      temperatures[1] = 0.0f;
      temperatures[2] = 0.0f;
    }
  }
}


/*
  Reading the status of the board to identify any flags, will be more useful when implementing a sleep cycle
*/
void BMSModule::readStatus()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = moduleAddress << 1; //adresss
  payload[1] = REG_ALERT_STATUS;//Alert Status start
  payload[2] = 0x04;
  BMSUtil::sendDataWithReply(payload, 3, false, buff, 7);
  alerts = buff[3];
  faults = buff[4];
  COVFaults = buff[5];
  CUVFaults = buff[6];
}

uint8_t BMSModule::getFaults()
{
  return faults;
}

uint8_t BMSModule::getAlerts()
{
  return alerts;
}

uint8_t BMSModule::getCOVCells()
{
  return COVFaults;
}

uint8_t BMSModule::getCUVCells()
{
  return CUVFaults;
}

float BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 8) return 0.0f;
  return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 8; i++) if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell) lowVal = cellVolt[i];
  return lowVal;
}

float BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 8; i++)
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      if (cellVolt[i] > hiVal) hiVal = cellVolt[i];
    }
  return hiVal;
}

float BMSModule::getAverageV()
{
  int x = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 8; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      x++;
      avgVal += cellVolt[i];
    }
  }

  if (x != 0)
  {
    scells = x;
  }
  avgVal /= x;
  return avgVal;
}

int BMSModule::getscells()
{
  return scells;
}

float BMSModule::getHighestModuleVolt()
{
  return highestModuleVolt;
}

float BMSModule::getLowestModuleVolt()
{
  return lowestModuleVolt;
}

float BMSModule::getHighestCellVolt(int cell)
{
  if (cell < 0 || cell > 8) return 0.0f;
  return highestCellVolt[cell];
}

float BMSModule::getLowestCellVolt(int cell)
{
  if (cell < 0 || cell > 8) return 0.0f;
  return lowestCellVolt[cell];
}

float BMSModule::getHighestTemp()
{
  return highestTemperature;
}

float BMSModule::getLowestTemp()
{
  return lowestTemperature;
}

float BMSModule::getLowTemp()
{
  if (sensor == 0)
  {
    if (temperatures[0] < temperatures[1])
    {
      if (temperatures[0] < temperatures[2])
      {
        return (temperatures[0]);
      }
      else
      {
        return (temperatures[2]);
      }
    }
    else
    {
      if (temperatures[1] < temperatures[2])
      {
        return (temperatures[1]);
      }
      else
      {
        return (temperatures[2]);
      }
    }
  }
  if (sensor == 1)
  {
    if (temperatures[2] < temperatures[1])
    {
      return (temperatures[2]);
    }
    else
    {
      return (temperatures[1]);
    }
  }
  if (sensor == 2)
  {
    if (temperatures[0] < temperatures[2])
    {
      return (temperatures[0]);
    }
    else
    {
      return (temperatures[2]);
    }
  }
  if (sensor == 3)
  {
    if (temperatures[0] < temperatures[1])
    {
      return (temperatures[0]);
    }
    else
    {
      return (temperatures[1]);
    }
  }
  if (sensor == 12)
  {
    return (temperatures[2]);
  }
  if (sensor == 23)
  {
    return (temperatures[0]);
  }
  if (sensor == 13)
  {
    return (temperatures[1]);
  }
}

float BMSModule::getHighTemp()
{
  if (sensor == 0)
  {
    if (temperatures[0] > temperatures[1])
    {
      if (temperatures[0] > temperatures[2])
      {
        return (temperatures[0]);
      }
      else
      {
        return (temperatures[2]);
      }
    }
    else
    {
      if (temperatures[1] > temperatures[2])
      {
        return (temperatures[1]);
      }
      else
      {
        return (temperatures[2]);
      }
    }
  }
  if (sensor == 1)
  {
    if (temperatures[2] > temperatures[1])
    {
      return (temperatures[2]);
    }
    else
    {
      return (temperatures[1]);
    }
  }
  if (sensor == 2)
  {
    if (temperatures[0] > temperatures[2])
    {
      return (temperatures[0]);
    }
    else
    {
      return (temperatures[2]);
    }
  }
  if (sensor == 3)
  {
    if (temperatures[0] > temperatures[1])
    {
      return (temperatures[0]);
    }
    else
    {
      return (temperatures[1]);
    }
  }
  if (sensor == 12)
  {
    return (temperatures[2]);
  }
  if (sensor == 23)
  {
    return (temperatures[0]);
  }
  if (sensor == 13)
  {
    return (temperatures[1]);
  }
}

float BMSModule::getAvgTemp()
{
  if (sensor == 0)
  {
    return ((temperatures[0] + temperatures[1] + temperatures[2]) / 3);
  }
  if (sensor == 1)
  {
    return ((temperatures[1] + temperatures[2]) * 0.5);
  }
  if (sensor == 2)
  {
    return ((temperatures[0] + temperatures[2]) * 0.5);
  }
  if (sensor == 3)
  {
    return ((temperatures[0] + temperatures[1]) * 0.5);
  }
  if (sensor == 12)
  {
    return (temperatures[2]);
  }
  if (sensor == 23)
  {
    return (temperatures[0]);
  }
  if (sensor == 13)
  {
    return (temperatures[1]);
  }

}

float BMSModule::getModuleVoltage()
{
  moduleVolt = 0;
  for (int I; I < 8; I++)
  {
    if (cellVolt[I] > IgnoreCell && cellVolt[I] < 60.0)
    {
      moduleVolt = moduleVolt + cellVolt[I];
    }
  }
  return moduleVolt;
}

float BMSModule::getTemperature(int temp)
{
  if (temp < 0 || temp > 2) return 0.0f;
  return temperatures[temp];
}

void BMSModule::setAddress(int newAddr)
{
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR) return;
  moduleAddress = newAddr;
}

int BMSModule::getAddress()
{
  return moduleAddress;
}

uint8_t BMSModule::getBalStat()
{
  return balstat;
}

bool BMSModule::isExisting()
{
  return exists;
}

bool BMSModule::isReset()
{
  return reset;
}

void BMSModule::settempsensor(int tempsensor)
{
  sensor = tempsensor;
}

void BMSModule::setExists(bool ex)
{
  exists = ex;
}

void BMSModule::setReset(bool ex)
{
  reset = ex;
}

void BMSModule::setIgnoreCell(float Ignore)
{
  IgnoreCell = Ignore;
  Serial.print(Ignore);
  Serial.println();
}

void BMSModule::setTempconv(float tempconvin, int tempoffin)
{
  tempconv = tempconvin;
  tempoff = tempoffin;
}
