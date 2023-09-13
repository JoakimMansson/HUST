#include "Arduino.h"
#include "RAK811.h"

extern "C" {
  #include "string.h"
  #include "stdlib.h"
  }

/*
  @param serial Needs to be an already opened Stream ({Software/Hardware}Serial) to write to and read from.
*/


const char *hexdigits = "0123456789ABCDEF";

char* convertBytesToString (uint8_t* inputBuffer, int inputSize) {
    int i, j;
    char* compositionBuffer = (char*) malloc(inputSize*2 + 1);
    for (i = j = 0; i < inputSize; i++) {
        unsigned char c;
        c = (inputBuffer[i] >> 4) & 0xf;
        compositionBuffer[j++] = hexdigits[c];
        c = inputBuffer[i] & 0xf;
        compositionBuffer[j++] = hexdigits[c];
    }
    return compositionBuffer;
}


RAK811::RAK811(Stream& serial,Stream& serial1):
_serial(serial),_serial1(serial1)
{
  _serial.setTimeout(2000);
  _serial1.setTimeout(1000);
}

String RAK811::rk_getVersion()
{
  String ret = sendRawCommand(F("at+version"));
  ret.trim();
  return ret;
}

String RAK811::rk_getBand()
{
  String ret = sendRawCommand(F("at+band"));
  ret.trim();
  return ret;
}

String RAK811::rk_getSignal()
{
  String ret = sendRawCommand(F("at+signal"));
  ret.trim();
  return ret;
}

bool RAK811::rk_setRate(int rate)
{
  String ret = sendRawCommand("at+dr=" + (String)rate);
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RAK811::rk_sleep()
{
  sendRawCommand(F("at+sleep"));
}

void RAK811::rk_reload()
{
  sendRawCommand(F("at+reload"));
}

void RAK811::rk_reset(int mode)
{
  if (mode == 1)
  {
    sendRawCommand(F("at+reset=1"));
  }
  else if(mode == 0)
  {
    sendRawCommand(F("at+reset=0"));
  }
  else
  {
    Serial.println("The mode set Error,the mode is '0'or '1'.");
  }
}

String RAK811::rk_exitBootMode(void)
{
  String command = "";
  command = "at+set_config=device:restart";
  String ret = sendRawCommand(command);
  return ret;
}

String RAK811::rk_getCurrentMode(void)
{
  String command = "";
  command = "at+mode";

  String ret = sendRawCommand(command);
  if(ret.startsWith("OK"))
  {
    return ret;
  }
  else
  {
    return "SNOAJMSOJNJINWOMSKOÖA";
  }
  //return ret;
}

String RAK811::rk_setWorkingMode(int mode)
{
  String ver;
  switch (mode)
  {
    case 0:
      ver = sendRawCommand("at+mode=0"); //Set LoRaWAN Mode.
      break;
    case 1:
      ver = sendRawCommand("at+mode=1"); //Set LoRaP2P Mode.
      break;
  }
  return ver;
  //Serial.println(ver);
  /*
  if(ver.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
  */
}

String RAK811::rk_currentVersion(void)
{
  String command = "";
  command = "at+version";
  
  String ret = sendRawCommand(command);
  return ret;
}

bool RAK811::rk_sendData(int type, int port, char* datahex)
{
  String command = "";
  command = "at+send=" + (String)type + "," + port + "," + datahex;
//  Serial.println(command);
  String ret = sendRawCommand(command);
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RAK811::rk_sendBytes(int type, int port, uint8_t* buffer, int bufSize)
{
  //Send Bytes Command

  String command = "";
  char* p = convertBytesToString(buffer, bufSize);
  command = "at+send=" + (String)type + "," + port + "," + p;
  free(p);
  //Serial.println(command);
  String ret = sendRawCommand(command);
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

String RAK811::rk_recvData(void)
{
  _serial.setTimeout(2000);
  String ret = _serial.readStringUntil('\n');
#if defined DEBUG_MODE
  _serial1.println("-> " + ret);
#endif
  ret.trim();
  return ret;
}

bool RAK811::rk_setConfig(String Key,String Value)
{
  String command = "";
  command = "at+set_config=" + Key + ":" + Value;
//  Serial.println(command);
  String ret = sendRawCommand(command);
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

String RAK811::rk_getConfig(String Key)
{
  String ret = sendRawCommand("at+get_config=" + Key);
  ret.trim();
  return ret;
}

String RAK811::rk_getP2PConfig(void)
{
  String ret = sendRawCommand("at+rf_config");
  ret.trim();
  return ret;
}

String RAK811::rk_initP2P(String FREQ, String SF, String BW, String CR, String PRlen, String PWR)
{
  String command = "";
  command = "at+rf_config=" + FREQ + "," + SF + "," + BW + "," + CR + "," + PRlen + "," + PWR;
//  Serial.println(command);
  String ret = sendRawCommand(command);
  return ret;
  /*
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
  */
}

bool RAK811::rk_initTransferMode(int mode)
{
  String command = "";
  command = "at+set_config=lorap2p:transfer_mode:" + mode;

  String ret = sendRawCommand(command);
  if(ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

String RAK811::rk_recvP2PData(int report_en)
{
  String ver;
  switch (report_en)
  {
    case 0:
      ver = sendRawCommand(F("at+rxc=0")); //
      break;
    case 1:
      ver = sendRawCommand(F("at+rxc=1")); //
      break;
  }
  return ver;
  /*
  if (ver.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
  */
}

String RAK811::rk_sendP2PData(int nrPackets, String timeInterval, char* DATAHex)
{
  String command = "";
  command = "at+txc=" + (String)nrPackets + "," + timeInterval + "," + DATAHex;
//  Serial.println(command);
  
  String ret = sendRawCommand(command);
  return ret;
  /*
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
  */
}

bool RAK811::rk_stopSendP2PData(void)
{
  String ret = sendRawCommand(F("at+tx_stop"));
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RAK811::rk_stopRecvP2PData(void)
{
  String ret = sendRawCommand(F("at+rx_stop"));
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

String RAK811::rk_checkStatusStatistics(void)
{
  String ret = sendRawCommand(F("at+status"));
  ret.trim();
  return ret;
}

bool RAK811::rk_cleanStatusStatistics(void)
{
  String ret = sendRawCommand(F("at+status=0"));
  if (ret.startsWith("OK"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

String RAK811::rk_setUARTConfig(int Baud, int Data_bits, int Parity, int Stop_bits, int Flow_ctrl)
{
  String command = "";
  command = "at+uart=" + (String)Baud + "," + Data_bits + "," + Parity + "," + Stop_bits + "," + Flow_ctrl;
//  Serial.println(command);
  String ret = sendRawCommand(command);
  return ret;
}

String RAK811::sendRawCommand(String command)
{
  delay(100);
  while(_serial.available()){
#if defined DEBUG_MODE
    _serial1.println("-> " + _serial.readStringUntil('\0'));
#else
    _serial.read();
#endif
  }
  delay(100);
  _serial.println(command);
  delay(200);
  String ret = _serial.readStringUntil('\n');
  ret.trim();
  delay(500);
#if defined DEBUG_MODE
  _serial1.println("<- " + command);
  _serial1.println("-> " + ret);
#endif
  return ret;
}


/*------------------------- NOT NECESSARY ----------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/


 

