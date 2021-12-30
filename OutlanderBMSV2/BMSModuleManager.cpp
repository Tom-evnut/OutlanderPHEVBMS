#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

extern EEPROMSettings settings;

BMSModuleManager::BMSModuleManager()
{
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;

  //////////////smoothing of readings/////////////////////

  LowCellVoltsmooth = 0;
  HighCellVoltsmooth = 0;
  MeasurementStep = 0.001;


  avgindex = 0;
  avgtotal = 0;
  lowindex = 0;
  lowtotal = 0;
  hightotal = 0;
  highindex = 0;
  for (int y = 0; y < 8; y++)
  {
    avgcell[y] = 0;
    lowcell[y] = 0;
    highcell [y] = 0;
  }

}

void BMSModuleManager::clearmodules()
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      //modules[y].clearmodule();
      modules[y].setExists(false);
      modules[y].setAddress(y);
    }
  }
}

bool BMSModuleManager::checkcomms()
{
  int g = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      g = 1;
      if (modules[y].isReset())
      {
        //Do nothing as the counter has been reset
      }
      else
      {
        return false;
      }
    }
    modules[y].setReset(false);
  }
  if ( g == 0)
  {
    return false;
  }
  return true;
}

int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::decodecan(CAN_message_t &msg)
{
  int Id, CMU = 0;
  if ((msg.id & 0x80000000) == 0x80000000)
  {
    CMU = (((msg.id & 0x00000FF0) - 0x600) >> 4);
    Id = msg.id & 0x00F;
    Serial.println();
    Serial.print("Long ID recieved :");
    Serial.print(msg.id & 0x80000000);
    Serial.println();
  }
  else
  {
    Id = msg.id & 0x00F;
    CMU = (((msg.id & 0xFF0) - 0x600) >> 4);
  }
  modules[CMU].setExists(true);
  modules[CMU].setReset(true);
  modules[CMU].decodecan(Id, msg);
}

void BMSModuleManager::getAllVoltTemp()
{
  /////smoothing Low////////////////////

  /*lowtotal = lowtotal - lowcell[lowindex];

    lowcell[lowindex] = LowCellVolt;

    lowtotal = lowtotal + LowCellVolt;

    lowindex = lowindex + 1;
    Serial.println();
    Serial.print("Low | ");
    Serial.print(lowindex);
    Serial.print(" | ");
    Serial.print(LowCellVolt);
    Serial.print(" | ");
    Serial.print(lowtotal);
    Serial.print(" | ");
    if (lowindex > 7)
    {
    lowindex = 0;
    }

    LowCellVoltsmooth = lowtotal / 8;
  */

  if (abs(LowCellVoltsmooth - LowCellVolt) > MeasurementStep)
  {
    if (LowCellVoltsmooth > LowCellVolt)
    {
      LowCellVoltsmooth = LowCellVoltsmooth - MeasurementStep;
    }
    else
    {
      LowCellVoltsmooth = LowCellVoltsmooth + MeasurementStep;
    }
  }
  else
  {
    LowCellVoltsmooth = LowCellVolt;
  }


  // Serial.print(LowCellVoltsmooth);

  /////smoothing High////////////////////
  /*
    hightotal = hightotal - highcell[highindex];

    if ( LowCellVolt > HighCellVolt)
    {
      HighCellVolt = LowCellVolt;
    }

    highcell[highindex] = HighCellVolt;

    hightotal = hightotal + HighCellVolt;

    highindex = highindex + 1;
    /*
      Serial.println();
      Serial.print("High | ");
      Serial.print(highindex);
      Serial.print(" | ");
      Serial.print(HighCellVolt);
      Serial.print(" | ");
      Serial.print(hightotal);
      Serial.print(" | ");

    if (highindex > 7)
    {
      highindex = 0;
    }

    HighCellVoltsmooth = hightotal / 8;
  */
  // Serial.print(HighCellVoltsmooth);

  if (abs(HighCellVoltsmooth - HighCellVolt) > MeasurementStep)
  {
    if (HighCellVoltsmooth > HighCellVolt)
    {
      HighCellVoltsmooth = HighCellVoltsmooth - MeasurementStep;
    }
    else
    {
      HighCellVoltsmooth = HighCellVoltsmooth + MeasurementStep;
    }
  }
  else
  {
    HighCellVoltsmooth = HighCellVolt;
  }

  /////smoothing////////////////////

  if (avg > 0 && avg < 10)
  {
    avgtotal = avgtotal - avgcell[avgindex];
    avgcell[avgindex] = avg;
    avgtotal = avgtotal + avg;
    avgindex = avgindex + 1;

    if (avgindex > 7)
    {
      avgindex = 0;
    }
    avgsmooth = avgtotal / 8;
  }
  /////smoothing////////////////////

  packVolt = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      Logger::debug("");
      Logger::debug("Module %i exists. Reading voltage and temperature values", x);
      Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
      Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
      Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
      packVolt += modules[x].getModuleVoltage();
      if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
    }
  }

  packVolt = packVolt / Pstring;
  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

  if (digitalRead(11) == LOW) {
    if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
    isFaulted = true;
  }
  else
  {
    if (isFaulted) Logger::info("All modules have exited a faulted state");
    isFaulted = false;
  }
}

float BMSModuleManager::getLowCellVolt()
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      LowCellVolt = 5.0;
    }
  }
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[x].getLowCellV();
    }
  }

  //////smoothing////////////////////
  /*
    lowtotal = lowtotal - lowcell[lowindex];

    lowcell[lowindex] = LowCellVolt;

    lowtotal = lowtotal + LowCellVolt;

    lowindex = lowindex + 1;

    if (lowindex > 7)
    {
      lowindex = 0;
    }

    LowCellVolt = lowtotal / 8;
  */
  /////smoothing////////////////////

  return LowCellVoltsmooth;
}

float BMSModuleManager::getHighCellVolt()
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      HighCellVolt = 0.0;
    }
  }
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[x].getHighCellV();
    }
  }


  //////smoothing////////////////////
  /*
    hightotal = hightotal - highcell[highindex];

    if ( LowCellVolt > HighCellVolt)
    {
      HighCellVolt = LowCellVolt;
    }

    highcell[highindex] = HighCellVolt;

    hightotal = hightotal + HighCellVolt;

    highindex = highindex + 1;

    if (highindex > 7)
    {
      highindex = 0;
    }

    HighCellVolt = hightotal / 8;
  */
  /////smoothing////////////////////

  return HighCellVoltsmooth;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  Pstring = Pstrings;
}

void BMSModuleManager::setSensors(int sensor, float Ignore, float tempconvin, int tempoffin)
{
  tempsens = sensor;
  ignorevolt = Ignore;
  tempconv = tempconvin;
  tempoff = tempoffin;

  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].settempsensor(sensor);
      modules[x].setIgnoreCell(Ignore);
      modules[x].setTempconv(tempconvin, tempoffin);
    }
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avgtemp = 0.0f;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  int y = 0; //counter for modules below -70 (no sensors connected)
  numFoundModules = 0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      numFoundModules++;
      if (modules[x].getAvgTemp() > -70)
      {
        avgtemp += modules[x].getAvgTemp();
        if (modules[x].getHighTemp() > highTemp)
        {
          highTemp = modules[x].getHighTemp();
        }
        if (modules[x].getLowTemp() < lowTemp)
        {
          lowTemp = modules[x].getLowTemp();
        }
      }
      else
      {
        y++;
      }
    }
  }
  avgtemp = avgtemp / (float)(numFoundModules - y);

  if (numFoundModules != numFoundModulesOLD)
  {
    numFoundModulesOLD = numFoundModules;
    setSensors(tempsens, ignorevolt, tempconv, tempoff);
  }
  return avgtemp;
}

float BMSModuleManager::getHighTemperature()
{
  return highTemp;
}

int BMSModuleManager::getNumModules()
{
  return numFoundModules;
}



float BMSModuleManager::getLowTemperature()
{
  return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
  avg = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting()) avg += modules[x].getAverageV();
  }
  avg = avg / (float)numFoundModules;


  //////smoothing////////////////////
  /*
    if (avg > 0 && avg < 10)
    {
      avgtotal = avgtotal - avgcell[avgindex];
      avgcell[avgindex] = avg;
      avgtotal = avgtotal + avg;
      avgindex = avgindex + 1;

      if (avgindex > 7)
      {
        avgindex = 0;
      }
      avg = avgtotal / 8;
  */
  /////smoothing////////////////////
  return avgsmooth;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i  Cells: %i  Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules, seriescells(),
                  getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Logger::console("                               Module #%i", y);

      Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
      if (faults > 0)
      {
        Logger::console("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          SERIALCONSOLE.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 8; i++)
          {
            if (COV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 2)
        {
          SERIALCONSOLE.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 8; i++)
          {
            if (CUV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 4)
        {
          Logger::console("    CRC error in received packet");
        }
        if (faults & 8)
        {
          Logger::console("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          Logger::console("    Test fault active");
        }
        if (faults & 0x20)
        {
          Logger::console("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        Logger::console("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          Logger::console("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          Logger::console("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          Logger::console("    Sleep mode active");
        }
        if (alerts & 8)
        {
          Logger::console("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          Logger::console("    Test Alert");
        }
        if (alerts & 0x20)
        {
          Logger::console("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          Logger::console("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          Logger::console("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) SERIALCONSOLE.println();
    }
  }
}

void BMSModuleManager::printPackDetails(int digits, bool showbal)
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;
  uint8_t bal;
  int cellNum = 0;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV Delta Voltage: %zmV   Avg Temp: %fC ", numFoundModules, seriescells(),
                  Pstring, getPackVoltage(), getAvgCellVolt(), LowCellVoltsmooth, HighCellVoltsmooth, (HighCellVolt - LowCellVolt) * 1000, getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();
      bal = modules[y].getBalStat();

      SERIALCONSOLE.print("Module #");
      SERIALCONSOLE.print(y);
      if (y < 10) SERIALCONSOLE.print(" ");
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(modules[y].getModuleVoltage(), digits);
      SERIALCONSOLE.print("V");
      for (int i = 0; i < 8; i++)
      {
        if (cellNum < 10) SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print("  Cell");
        SERIALCONSOLE.print(cellNum++);
        SERIALCONSOLE.print(": ");
        SERIALCONSOLE.print(modules[y].getCellVoltage(i), digits);
        SERIALCONSOLE.print("V");
        if (showbal == 1)
        {
          if ((bal & (0x1 << i)) > 0)
          {
            SERIALCONSOLE.print(" X");
          }
          else
          {
            SERIALCONSOLE.print(" -");
          }
        }
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.print(" Temp 1: ");
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print("C Temp 2: ");
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      SERIALCONSOLE.print("C Temp 3: ");
      SERIALCONSOLE.print(modules[y].getTemperature(2));

      if (showbal == 1)
      {
        SERIALCONSOLE.print("C  Bal Stat: ");
        SERIALCONSOLE.println( bal, BIN);
      }
      else
      {
        SERIALCONSOLE.println("C");
      }

    }
  }
}

void BMSModuleManager::printAllCSV(unsigned long timestamp, float current, int SOC)
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      SERIALCONSOLE.print(timestamp);
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(current, 0);
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(SOC);
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(y);
      SERIALCONSOLE.print(",");
      for (int i = 0; i < 8; i++)
      {
        SERIALCONSOLE.print(modules[y].getCellVoltage(i));
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(modules[y].getTemperature(2));
      SERIALCONSOLE.println();
    }
  }
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      Serial2.print(timestamp);
      Serial2.print(",");
      Serial2.print(current, 0);
      Serial2.print(",");
      Serial2.print(SOC);
      Serial2.print(",");
      Serial2.print(y);
      Serial2.print(",");
      for (int i = 0; i < 8; i++)
      {
        Serial2.print(modules[y].getCellVoltage(i));
        Serial2.print(",");
      }
      Serial2.print(modules[y].getTemperature(0));
      Serial2.print(",");
      Serial2.print(modules[y].getTemperature(1));
      Serial2.print(",");
      Serial2.print(modules[y].getTemperature(2));
      Serial2.println();
    }
  }
}
