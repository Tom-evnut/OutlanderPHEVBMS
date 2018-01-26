// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
const int nummod = 8;
short boardpres = 0;
int Id, CMU;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
char mes[8] = {0, 0, 0, 4, 3, 0, 0, 0};

long voltage[nummod][8];
long lowcell = 5000;
long temp[nummod][6];
int balstat[nummod];
unsigned long looptime = 0;
unsigned long canupdate = 0;
long balvol = 5000;
char  incomingByte;
int balance = 0;

int Debug = 0;

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(9);                               // Set CS to pin 9 de[ending on shield used


void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  Serial.println("Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8");
}

void loop()
{
  if (Serial.available())
  {
    menu();
  }


  if (CAN0.checkReceive() == 3)                        // If CAN0_INT pin is low, read receive buffer
  {
    candecode();
  }

  if (millis() > canupdate + 400)
  {
    cansend();
  }
  if (millis() > looptime + 500)
  {
    looptime = millis();
    packinfo();
    
  }
}

void cansend()
{
  if (balance == 1)
  {
    mes[0] = highByte(lowcell);
    mes[1] = lowByte(lowcell);
    mes[2] = balance;
  }
  else
  {
    mes[0] = 0;
    mes[1] = 0;
    mes[2] = 0;
  }
  CAN0.sendMsgBuf(0x3C3, 0, 8, mes);

}

void candecode()
{
      CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    Id = rxId & 0x00F;
    CMU = ((rxId & 0x0F0) >> 4) - 1;

    boardpres = boardpres | (1 << CMU);

    switch (Id)
    {
      case 0x1:
        balstat[CMU] = rxBuf[0];
        temp[CMU][0] = rxBuf[2] * 256 + rxBuf[3];
        temp[CMU][1] = rxBuf[4] * 256 + rxBuf[5];
        temp[CMU][2] = rxBuf[6] * 256 + rxBuf[7];
        break;

      case 0x3:
        voltage[CMU][4] = rxBuf[0] * 256 + rxBuf[1];
        voltage[CMU][5] = rxBuf[2] * 256 + rxBuf[3];
        voltage[CMU][6] = rxBuf[4] * 256 + rxBuf[5];
        voltage[CMU][7] = rxBuf[6] * 256 + rxBuf[7];
        break;

      case 0x2:
        voltage[CMU][0] = rxBuf[0] * 256 + rxBuf[1];
        voltage[CMU][1] = rxBuf[2] * 256 + rxBuf[3];
        voltage[CMU][2] = rxBuf[4] * 256 + rxBuf[5];
        voltage[CMU][3] = rxBuf[6] * 256 + rxBuf[7];
        break;

      default:

        break;
    }

    if (Debug == 1)
    {
      Serial.print(millis());
      if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, ",0x%.3lX,false,%1d", rxId, len);

      Serial.print(msgString);

      if ((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for (byte i = 0; i < len; i++) {
          sprintf(msgString, ", 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
      Serial.println();
    }
}

void packinfo()
{
  Serial.println();
  Serial.println();
  for (int x = 0; x <= nummod; x++)
  {
    if ( boardpres == (1 << x) )
    {
      Serial.print("Module ");
      Serial.print(x + 1);
      Serial.print(" || ");
      Serial.print(balstat[x], BIN);
      Serial.print(" || ");
      for (int i = 0; i < 8; i++)
      {
        Serial.print("Cell ");
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.print(voltage[x][i]);
        Serial.print(" mV ");
        if (voltage[x][i] < lowcell)
        {
          lowcell = voltage[x][i];
        }
      }
      for (int i = 0; i < 3; i++)
      {
        Serial.print("Temp ");
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.print(float(temp[x][i]) * 0.001);
        Serial.print(" C ");
      }
    }
  }
}

void menu ()
{
  incomingByte = Serial.read(); // read the incoming byte:
  switch (incomingByte)
  {
    /*
      case 'v'://v Balance setpoint
      balvol = Serial.parseInt();
      if (balvol < 3500)
      {
        balvol = 3500;
      }
      Serial.println();
      Serial.print(balvol);
      Serial.print(" mV balance setpoint");
      Serial.println();
      break;
    */
    case 'b' ://toggle balance
      Serial.println();
      if (balance == 0)
      {
        balance = 1;
        Serial.print(" Balancing ON");
      }
      else
      {
        balance = 0;
        Serial.print(" Balancing OFF");
      }
      Serial.println();
      break;

    default:
      Serial.println(" Incorrect input");
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
