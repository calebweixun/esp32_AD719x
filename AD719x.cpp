/*******************************************************************************
 *   @lib     AD719x.cpp
 *   @brief   AD7190/AD7195 Drivers.
 *   @author  CalebZhang（amee.caleb@gmail.com) 
 *   
 *   此函式庫基於 ESP32-HSPI 開發，可直接使用於 ESP32 上
 *   撰寫時依照 Analog Device AD7190 NoOS 的範例，部分參考了 annem/AD7193 函式庫的撰寫方法
 *   
 *   Last edit @ 2022057
*******************************************************************************/

#include <Arduino.h>
#include "AD719x.h"
#include <SPI.h>

SPIClass * hspi = NULL;
bool STA_SER = true;
unsigned char ADCModel = 0x00;
unsigned char DataRegSize = 3;
unsigned char range_setup[2] = {0,AD719x_CONF_GAIN_1};

/******************************************************************************
 * @brief 讀取寄存器資料
 * @param 地址、寄存器參數、字節數
*******************************************************************************/
unsigned long AD719x::GetRegisterValue(unsigned char registerAddress,
                                       unsigned char bytesNumber){
    unsigned char receiveBuffer = 0;
    unsigned char writeByte = 0;
    unsigned char byteIndex = 0;
    unsigned long buffer = 0;
    
    writeByte = AD719x_COMM_READ | AD719x_COMM_ADDR(registerAddress);
    digitalWrite(AD719x_CS_PIN, LOW);
    
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    hspi->transfer(writeByte);
    hspi->endTransaction();
    
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
    for(byteIndex = 0;byteIndex < bytesNumber;byteIndex++){
        receiveBuffer = hspi->transfer(0x00);
        buffer = (buffer << 8) + receiveBuffer;
    }
    digitalWrite(AD719x_CS_PIN, HIGH);
    hspi->endTransaction();
    
    return(buffer);
}

/******************************************************************************
 * @brief 寫入寄存器資料
 * @param 地址、寄存器參數、字節數
*******************************************************************************/
void AD719x::SetRegisterValue(unsigned char registerAddress,  
                              unsigned int  registerValue,  
                              unsigned char bytesNumber){ 

    unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;

    writeCommand[0] = AD719x_COMM_WRITE | AD719x_COMM_ADDR(registerAddress);
    
    while(bytesNr > 0){
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }
    
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(AD719x_CS_PIN, LOW);
    for(bytesNr = 0;bytesNr<bytesNumber+2;bytesNr++){
        hspi->transfer(writeCommand[bytesNr]);
    }
    digitalWrite(AD719x_CS_PIN, HIGH);
    hspi->endTransaction();
    
    /*輸出寄存器檢視、除錯用*/
    if(STA_SER){
      Serial.print("Write-Reg-Add: ");
      Serial.print(registerAddress, HEX);
      Serial.print(", ");
      Serial.print(writeCommand[0], HEX);
      Serial.print(", ");
      Serial.println(registerValue, HEX);
    }
}

/*******************************************************************************
 * @brief 啟動SPI運作，並檢查AD719x狀態
 * @param ADC 狀態是否輸出
*******************************************************************************/
void AD719x::begin(bool staser) {  

    STA_SER = staser;
    
    hspi = new SPIClass(HSPI);
    hspi->setClockDivider(SPI_CLOCK_DIV16);
    hspi->begin();
    
    pinMode(AD719x_CS_PIN, OUTPUT);  
    digitalWrite(AD719x_CS_PIN, HIGH);
   
    Reset();
    delay(10);
    uint32_t regVal = 0;
    regVal = GetRegisterValue(AD719x_REG_ID, 1);
    
    if((regVal & AD719x_ID_MASK) == ID_AD7190){
      if(STA_SER){
        Serial.println("AD7190 exist");
        ADCModel = ID_AD7190;
      }
    }else if((regVal & AD719x_ID_MASK) == ID_AD7195){ 
      if(STA_SER){
        Serial.println("AD7195 exist");
        ADCModel = ID_AD7195;
      }
    }else{
      while(1){
        /*異常則不得啟用，提示ADC異常*/
        Serial.println("Adc Not Found.");
        delay(10000);
      }
    }
}

/*******************************************************************************
 * @brief 重置 ADC 狀態
 * @input 
*******************************************************************************/
void AD719x::Reset(void)  { 
    /* on page.34 in AD7190-Datasheet*/
    if(STA_SER){
      Serial.print("ADC:reseting...");
    }
    unsigned char registerWord[7] = {0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    digitalWrite(AD719x_CS_PIN, LOW);
    delayMicroseconds(100);
    
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    for(int i = 0; i < 7; i++){
      hspi->transfer(registerWord[i]);
    }
    hspi->endTransaction();
    digitalWrite(AD719x_CS_PIN, HIGH);
    /*500us (0.5ms) 後才能繼續訪問寄存器*/
    delayMicroseconds(500);
}

/*******************************************************************************
 * @brief 等 RDY 狀態回復
 * @param 
*******************************************************************************/
void AD719x::WaitRdyGoLow(void)  {
    while(1){
      if (digitalRead(AD719x_RDY_STATE) == 0){      // Break if RDY goes low
        break;
      }
    }
}

/***************************************************************************//**
 * @brief 設定ADC電源狀態，可以參閱資料表中的 page 33 
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
*******************************************************************************/
void AD719x::SetPower(unsigned char pwrMode) {
    if(STA_SER){
      Serial.println("ADC: Power-Down..");
    }
    unsigned int oldPwrMode = 0x0;
    unsigned int newPwrMode = 0x0; 
  
    oldPwrMode = GetRegisterValue(AD719x_REG_MODE, 3);
    oldPwrMode &= ~(AD719x_MODE_SEL(0x7));
    newPwrMode = oldPwrMode | AD719x_MODE_SEL((pwrMode * (AD719x_MODE_IDLE)) | (!pwrMode * (AD719x_MODE_PWRDN)));
    SetRegisterValue(AD719x_REG_MODE, newPwrMode, 3);
}


/*******************************************************************************
 * @brief 配置ADC量程（會自動開啟BUF），可參考 AD7190 資料表 page.23
 * @param polarity  - 極性（0=單極性、1=雙極性）
 * @param range     - 增益值（可參考標頭檔的AD719x_CONF_GAIN_）
*******************************************************************************/
void AD719x::RangeSetup(unsigned char polarity, unsigned char range)  {
    if(STA_SER){
      Serial.print("ADC: polarity ");
      Serial.print(polarity);
      Serial.print("ADC: range ");
      Serial.println(range);
    }
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;
    /*讀取寄存器目前的值*/
    oldRegValue = GetRegisterValue(AD719x_REG_CONF,3);
  
    oldRegValue &= ~(AD719x_CONF_UNIPOLAR | AD719x_CONF_GAIN(0x7));
    newRegValue = oldRegValue | (polarity * AD719x_CONF_UNIPOLAR) | AD719x_CONF_GAIN(range) | AD719x_CONF_BUF; 

    range_setup[0] = polarity;
    range_setup[1] = range;
    
    SetRegisterValue(AD719x_REG_CONF, newRegValue, 3);
}

/*******************************************************************************
 * @brief 配置ADC運作通道（可直接配置多通道），可參考 AD7190 資料表 page.24
 * @param 通道（可參考標頭檔的AD719x_CONF_CHAN(x)）
*******************************************************************************/
void AD719x::ChannelSelect(unsigned short channel) {
    if(STA_SER){
      Serial.print("ADC: Channel enable on h");
      Serial.println(channel,HEX);
    }
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;   
     
    oldRegValue  = GetRegisterValue(AD719x_REG_CONF, 3);
    oldRegValue &= ~(AD719x_CONF_CHAN(0xFF));
    newRegValue  = oldRegValue | AD719x_CONF_CHAN(channel);
      
    // write channel selected to Configuration register
    SetRegisterValue(AD719x_REG_CONF, newRegValue, 3);
    delay(10);
}

/*******************************************************************************
 * @brief 配置ADC斬波，可參考 AD7190 資料表 page.23
 * @param chop - chop setting
 *               Example: 0 - Disable
 *                        1 - enable
*******************************************************************************/
void AD719x::ChopSetting(unsigned char chop){
    if(STA_SER){
      Serial.print("ADC: Chop set ");
      Serial.println(chop);
    }
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;   
     
    oldRegValue = GetRegisterValue(AD719x_REG_CONF, 3);
    if(chop==1)
    {
      newRegValue = oldRegValue | AD719x_CONF_CHOP;
    }
    else
    {
      newRegValue = oldRegValue & (~AD719x_CONF_CHOP); 
    }
    
    SetRegisterValue(AD719x_REG_CONF, newRegValue, 3);
}
/*******************************************************************************
 * @brief 配置ADC ACX，可參考 AD7195 資料表 page.22 ///AD7195 only\\\
 * @param chop - chop setting
 *               Example: 0 - Disable
 *                        1 - enable
*******************************************************************************/
void AD719x::ACXSetting(unsigned char acx){
    if(ADCModel == ID_AD7195){
      if(STA_SER){
        Serial.print("ADC: ACX set ");
        Serial.println(acx);
      }
      unsigned int oldRegValue = 0x0;
      unsigned int newRegValue = 0x0;   
       
      oldRegValue = GetRegisterValue(AD719x_REG_CONF, 3);
      if(acx==1)
      {
        newRegValue = oldRegValue | AD719x_CONF_ACX;
      }
      else
      {
        newRegValue = oldRegValue & (~AD719x_CONF_ACX); 
      }
      
      SetRegisterValue(AD719x_REG_CONF, newRegValue, 3);
    }else{
      if(STA_SER){
        Serial.println("ADC: ACX not support");
        Serial.println(acx);
      }
    } 
}
/*******************************************************************************
 * @brief 執行校正操作 
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
*******************************************************************************/
void AD719x::Calibrate(unsigned char mode, unsigned char channel) {
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;
  
  ChannelSelect(channel);
  oldRegValue = GetRegisterValue(AD719x_REG_MODE, 3);
  oldRegValue &= ~AD719x_MODE_SEL(0x7);
  newRegValue = oldRegValue | AD719x_MODE_SEL(mode);
  
  if(STA_SER){
      Serial.print("ADC: Calibrate Mode-code");
      Serial.println(mode,HEX);
  }
  
  SetRegisterValue(AD719x_REG_MODE, newRegValue, 3);
  WaitRdyGoLow();
}

/*******************************************************************************
 * @brief 設定模式寄存器中可以開關的選項，請參考 AD7190資料表中 page.21
 * @param datasta - DATA_STA  設定狀態寄存器是否會與數據一同回來
 *               Example: 0 - Disable
 *                        1 - enable
 * @param sinc - sinc3/sinc4  Sinc4有更好的抑制率，Sinc3則是建立時間更短
 *               Example: 0 - sinc4
 *                        1 - sinc3
 * @param enpqr - ENPAR       開啟奇偶校驗，在開啟DATA_STA時，校驗位於狀態寄存器中的 SR4 顯示
 *               Example: 0 - Disable
 *                        1 - enable
 * @param scycle - Single     開啟使單週期轉換，在單通道連續轉換模式啟用時，降低ADC延遲
 *               Example: 0 - Disable
 *                        1 - enable
 * @param rej60 - REJ60       開啟 50/60 Hz 同步抑制的功能
 *               Example: 0 - Disable
 *                        1 - enable
 *******************************************************************************/
void AD719x::ModeRegMISCSetting(unsigned char datasta,
                                unsigned char sinc,
                                unsigned char enpqr,
                                unsigned char scycle,
                                unsigned char rej60){
    if(STA_SER){
        Serial.print("ADC: Status will ");
      if(datasta==1){
        Serial.println("append to DATA");
      }else{
        Serial.println("not append to DATA");
      }
        Serial.print("ADC: Sinc-Filter set ");
      if(sinc==1){  
        Serial.println("Sinc4");
      }else{        
        Serial.println("Sinc3");
      }
      Serial.print("ADC: ENPAR set ");
      Serial.println(enpqr);
      Serial.print("ADC: Single set ");
      Serial.println(scycle);
      Serial.print("ADC: REJ60 set ");
      Serial.println(rej60);
    }
    
    unsigned int oldRegValue = 0x0;
    unsigned int newRegValue = 0x0;
    
    DataRegSize += datasta;
    oldRegValue = GetRegisterValue(AD719x_REG_MODE, 3);
    
    if(datasta==1){
      newRegValue = oldRegValue | AD719x_MODE_DAT_STA;
    }else{
      newRegValue = oldRegValue & (~AD719x_MODE_DAT_STA); 
    }
    oldRegValue = newRegValue;
    
    if(sinc==1){
      newRegValue = oldRegValue | AD719x_MODE_SINC3;
    }else{
      newRegValue = oldRegValue & (~AD719x_MODE_SINC3); 
    }
    oldRegValue = newRegValue;
    
    if(enpqr==1){
      newRegValue = oldRegValue | AD719x_MODE_ENPAR;
    }else{
      newRegValue = oldRegValue & (~AD719x_MODE_ENPAR); 
    }
    oldRegValue = newRegValue;

    if(scycle==1){
      newRegValue = oldRegValue | AD719x_MODE_SCYCLE;
    }else{
      newRegValue = oldRegValue & (~AD719x_MODE_SCYCLE); 
    }
    oldRegValue = newRegValue;

    if(rej60==1){
      newRegValue = oldRegValue | AD719x_MODE_REJ60;
    }else{
      newRegValue = oldRegValue & (~AD719x_MODE_REJ60); 
    }
    
    SetRegisterValue(AD719x_REG_MODE, newRegValue, 3);
}

/*******************************************************************************
 * @brief 設定數據輸出速率，可參考 AD7190資料表中 MR9到MR0 說明
 *        Chop disable -> DataRate = (fmod/64)/FS 
 *        Chop enable  -> DataRate = (fmod/64)/(N x FS)
 *        (fmod = MCLK(4.92MHz)/16, FS = 1~1023, N= Sinc濾波器階數
 *        
 * @param FS - Filter Setting（十進制）
*******************************************************************************/
void AD719x::DataRate(int FS)  {
    if(FS>1023){
      FS = 1023;
    }else if(FS<1){
      FS = 1;
    }
    
    if(STA_SER){
      Serial.print("ADC: Set SampleAveraging: ");
      Serial.println(FS);
    }
    unsigned int RegValue = 0x0;

    RegValue  = GetRegisterValue(AD719x_REG_MODE, 3);
    RegValue &= 0xFFFC00;
    RegValue |= AD719x_MODE_RATE(FS);
    
    SetRegisterValue(AD719x_REG_MODE, RegValue, 3);
}

/*******************************************************************************
 * @brief 設定擷取時脈來源，可參考AD7190資料表中的 MR19 MR18 項目
 * @param clksrc - 可參考標頭檔 AD719x_MODE_CLKSRC(x) 項目
 *                 AD719x_CLK_EXT_MCLK1_2
 *                 AD719x_CLK_EXT_MCLK2
 *                 AD719x_CLK_INT
 *                 AD719x_CLK_INT_CO
*******************************************************************************/
void AD719x::CLKSetting(unsigned char clksrc)  {
    if(STA_SER){
      Serial.print("ADC: Set CLK to Code ");
      Serial.println(clksrc);
    }
    
    unsigned int RegValue = 0x0;

    RegValue  = GetRegisterValue(AD719x_REG_MODE, 3);
    RegValue &= 0xF3FFFF;
    RegValue |= AD719x_MODE_CLKSRC(clksrc);
    
    SetRegisterValue(AD719x_REG_MODE, RegValue, 3);
}

/*******************************************************************************
 * @brief 將ADC轉換模式設定為單次轉換 可以參閱AD7190資料表中的 page.22 & page.30
 * @param mode - 可參考標頭檔中的 AD719x_MODE_SEL(x) 進行設定
*******************************************************************************/
void AD719x::SetWorkingMode(unsigned char Mode) {
  if(STA_SER){
    Serial.print("ADC: Conversion-Mode set to code ");
    Serial.println(Mode);
  }

  digitalWrite(AD719x_CS_PIN, LOW);
  
  unsigned long RegValue = 0x0;

  RegValue  = GetRegisterValue(AD719x_REG_MODE, 3);
  RegValue &= 0x1FFFFF; /*先去除原始的模式*/
  RegValue |= AD719x_MODE_SEL(Mode);

  SetRegisterValue(AD719x_REG_MODE, RegValue, 3);
}

/*******************************************************************************
 * @brief   配置連續讀取模式
 * @param   cread - 是否開啟連續讀取
 *               Example: 0 - Disable
 *                        1 - enable
 * @return  regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
void AD719x::Set_Continuous_Read(unsigned char cread) {

  unsigned char registerWord=0;
  
  if(cread==1){ 
    /*開啟連續*/
    SetWorkingMode(AD719x_MODE_CONT);
    digitalWrite(AD719x_CS_PIN, LOW);
    registerWord=0x5C;
  }
  else{
    /*禁用連續*/
    WaitRdyGoLow();  
    registerWord=0x58;
    digitalWrite(AD719x_CS_PIN, HIGH);
    SetWorkingMode(AD719x_MODE_IDLE);
  }
  
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  hspi->transfer(registerWord);
  hspi->endTransaction();
}
/*******************************************************************************
 * @brief   連續讀取資料，使用前須先配置通道，並執行 Set_Continuous_Read()
 * @return  buffer - Result of a Continuous analog-to-digital conversion.
*******************************************************************************/
unsigned long AD719x::Read_Continuous_Data(void)  {
  
    unsigned char byteIndex = 0;
    unsigned long buffer = 0;
    unsigned char receiveBuffer = 0;

    WaitRdyGoLow();
    
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
    while(byteIndex < DataRegSize){
      receiveBuffer = hspi->transfer(0x0);
      buffer = (buffer << 8) + receiveBuffer;
      byteIndex++;
    }
    hspi->endTransaction();
    
    return(buffer);
}

/*******************************************************************************
 * @brief   執行單次轉換取得結果，一個通道，執行完後 ADC 會自動回到斷關模式
 * @param   channel - 配置擷取通道
 * @return  regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long AD719x::Read_Single_Conversion(unsigned short channel){
  
  unsigned int RegValue = 0x0;
  unsigned int RegConf = 0x0;
  unsigned long regData = 0x0;
  
  ChannelSelect(channel);
  
  RegValue  = GetRegisterValue(AD719x_REG_MODE, 3);
  RegValue &= 0x1FFFFF; /*去除MR23~MR21*/
  RegValue |= AD719x_MODE_SEL(AD719x_MODE_SINGLE);
  
  unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
  unsigned char* dataPointer    = (unsigned char*)&RegValue;
  unsigned char bytesNr         = 3;

  writeCommand[0] = AD719x_COMM_WRITE | AD719x_COMM_ADDR(AD719x_REG_MODE);
    
  while(bytesNr > 0){
      writeCommand[bytesNr] = *dataPointer;
      dataPointer ++;
      bytesNr --;
  }

  digitalWrite(AD719x_CS_PIN, LOW);
  
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  for(bytesNr = 0;bytesNr<5;bytesNr++){
      hspi->transfer(writeCommand[bytesNr]);
  }
  hspi->endTransaction();

  WaitRdyGoLow();

  regData = GetRegisterValue(AD719x_REG_DATA, DataRegSize);

  digitalWrite(AD719x_CS_PIN, HIGH);
  return regData;
}


/*******************************************************************************
 * @brief   讀取溫度資料，並轉換輸出
 * @param   rawData - 是否開啟連續讀取
 * @return  degC - 攝氏溫度輸出
*******************************************************************************/
float AD719x::TempSensorDataToDegC(unsigned long rawData)  {
  
  float degC = (float(rawData - 0x800000) / 2815) - 273;
  float degF = (degC * 9 / 5) + 32;

  if(STA_SER){
    Serial.print(degC);
    Serial.print(" degC, ");
    Serial.print(degF);
    Serial.print(" degF\t");
  }
  return(degC);
}

/*******************************************************************************
 * @brief   轉換電壓資料
 * @param   rawData   - ADC原始電壓資料
 *          Volt_Ref  - 設計參考電壓(float)
 * @return  voltage   - 電壓輸出
*******************************************************************************/
float AD719x::DataToVoltage(unsigned long rawData,float Volt_Ref)  {

  float voltage = 0;
  
  /* range_setup[2] = {Polarity,gain} */
  int PGAGain = pow(2,range_setup[1]);  

  /* DAT_STA is ON, shift out STA-REG from rawData */
  if(DataRegSize == 4){
    rawData = rawData >> 8;
  }
  
  if(range_setup[0] == 1){ /*單極性模式*/
    voltage = ((double)rawData / 16777216 / (1 << PGAGain)) * Volt_Ref; 
    /* rawdata = (2^24x voltage x PGAGain) / Volt_Ref  */
  }
  if(range_setup[0] == 0){ /*雙極性模式*/
    voltage = (((float)rawData / (float)8388608) - (float)1) * (Volt_Ref / (float)PGAGain);
    /* rawdata = 2^23x[(voltage x PGAGain x Volt_Ref)]+1 */
  }

  return(voltage);
}


/*******************************************************************************
 * @brief 輸出 ADC 目前所有寄存器的狀態
 * @input 
*******************************************************************************/
void AD719x::ReadRegisterMap(void)  {
  Serial.println("");
  Serial.println("All adc register:");
  Serial.print("STAT");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_STAT, 1), HEX);
  Serial.print("MODE");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_MODE, 3), HEX);
  Serial.print("CONF");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_CONF, 3), HEX);
  Serial.print("DATA");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_DATA, 3), HEX);
  Serial.print("ID  ");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_ID, 1), HEX);
  Serial.print("GPOCON");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_GPOCON, 1), HEX);
  Serial.print("OFFSET");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_OFFSET, 3), HEX);
  Serial.print("FUSCALE");Serial.print('\t');
  Serial.println(GetRegisterValue(AD719x_REG_FULLSCALE, 3), HEX);
  Serial.println("");

}
