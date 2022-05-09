/*******************************************************************************
 *   @lib     AD719x.h
 *   @brief   AD7190/AD7195 Drivers.
 *   @author  CalebZhang（amee.caleb@gmail.com) 
 *   
 *   此函式庫基於 ESP32-HSPI 開發，可直接使用於 ESP32 上
 *   撰寫時依照 Analog Device AD7190 NoOS 的範例，部分參考了 annem/AD7193 函式庫的撰寫方法
 *   
 *   Last edit @ 2022057
*******************************************************************************/

#ifndef AD719x_h
#define AD719x_h

#include "Arduino.h"
#include "SPI.h"

/*  SPI IO         |  ESP-Pin
 *  CS:               15：選擇ADC，輸出LOW，使ADC可以在3線模式下工作，
 *  DIN:              13：配置晶片訊號
 *  SCLK:             14：通訊時脈
 *  DOUT/RDY:         12：當轉換完成後會變成LOW，再由此腳SPI傳輸數據
 *  SYNC:             27：拉低會使濾波器、控制節點重置
*/

#define MISO 12 //DOUT/RDY
#define MOSI 13 //DIN
#define SYNC 27 //SYNC

#define AD719x_CS_PIN     15 
#define AD719x_RDY_STATE  12   // pin to watch for data ready state

/* AD7195 Register Map */
#define AD719x_REG_COMM         0 // Communications Register (WO, 8-bit)      通信
#define AD719x_REG_STAT         0 // Status Register         (RO, 8-bit)      狀態
#define AD719x_REG_MODE         1 // Mode Register           (RW, 24-bit      模式
#define AD719x_REG_CONF         2 // Configuration Register  (RW, 24-bit)     配置
#define AD719x_REG_DATA         3 // Data Register           (RO, 24/32-bit)  數據
#define AD719x_REG_ID           4 // ID Register             (RO, 8-bit)      ID
#define AD719x_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit)      GPOCON
#define AD719x_REG_OFFSET       6 // Offset Register         (RW, 24-bit)     失調
#define AD719x_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)     滿量程

/* Communications Register Bit Designations (AD719x_REG_COMM) */
#define AD719x_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD719x_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD719x_COMM_READ        (1 << 6)           // Read Operation. 
#define AD719x_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD719x_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

/* Status Register Bit Designations (AD719x_REG_STAT) */
#define AD719x_STAT_RDY         (1 << 7) // Ready.
#define AD719x_STAT_ERR         (1 << 6) // ADC error bit.
#define AD719x_STAT_NOREF       (1 << 5) // Error no external reference. 
#define AD719x_STAT_PARITY      (1 << 4) // Parity check of the data register. 
#define AD719x_STAT_CH2         (1 << 2) // Channel 2. 
#define AD719x_STAT_CH1         (1 << 1) // Channel 1. 
#define AD719x_STAT_CH0         (1 << 0) // Channel 0. 

/* Mode Register Bit Designations (AD719x_REG_MODE) */
#define AD719x_MODE_SEL(x)      (((x) & 0x7) << 21) // Operation Mode Select.
#define AD719x_MODE_DAT_STA     (1 << 20)           // Status Register transmission.
#define AD719x_MODE_CLKSRC(x)   (((x) & 0x3) << 18)  // Clock Source Select.
#define AD719x_MODE_SINC3       (1 << 15)           // SINC3 Filter Select.
#define AD719x_MODE_ENPAR       (1 << 13)           // Parity Enable.
#define AD719x_MODE_SCYCLE      (1 << 11)           // Single cycle conversion.
#define AD719x_MODE_REJ60       (1 << 10)           // 50/60Hz notch filter.
#define AD719x_MODE_RATE(x)     ((x) & 0x3FF)       // Filter Update Rate Select.

/* Mode Register: AD719x_MODE_SEL(x) options */
#define AD719x_MODE_CONT                0 // Continuous Conversion Mode.
#define AD719x_MODE_SINGLE              1 // Single Conversion Mode.
#define AD719x_MODE_IDLE                2 // Idle Mode.
#define AD719x_MODE_PWRDN               3 // Power-Down Mode.
#define AD719x_MODE_CAL_INT_ZERO        4 // Internal Zero-Scale Calibration.
#define AD719x_MODE_CAL_INT_FULL        5 // Internal Full-Scale Calibration.
#define AD719x_MODE_CAL_SYS_ZERO        6 // System Zero-Scale Calibration.
#define AD719x_MODE_CAL_SYS_FULL        7 // System Full-Scale Calibration.

/* Mode Register: AD719x_MODE_CLKSRC(x) options */
#define AD719x_CLK_EXT_MCLK1_2          0 // External crystal. The external crystal
                                          // is connected from MCLK1 to MCLK2.
#define AD719x_CLK_EXT_MCLK2            1 // External Clock applied to MCLK2 
#define AD719x_CLK_INT                  2 // Internal 4.92 MHz clock. 
                                          // Pin MCLK2 is tristated.
#define AD719x_CLK_INT_CO               3 // Internal 4.92 MHz clock. The internal
                                          // clock is available on MCLK2.

/* Configuration Register Bit Designations (AD719x_REG_CONF) */
#define AD719x_CONF_CHOP        (1 << 23)            // CHOP enable.
#define AD719x_CONF_ACX         (1 << 22)            // ACX enable.
#define AD719x_CONF_REFSEL      (1 << 20)            // REFIN1/REFIN2 Reference Select.
#define AD719x_CONF_CHAN(x)     (((x) & 0xFF) << 8)  // Channel select.
#define AD719x_CONF_BURN        (1 << 7)             // Burnout current enable.
#define AD719x_CONF_REFDET      (1 << 6)             // Reference detect enable.
#define AD719x_CONF_BUF         (1 << 4)             // Buffered Mode Enable.
#define AD719x_CONF_UNIPOLAR    (1 << 3)             // Unipolar/Bipolar Enable.
#define AD719x_CONF_GAIN(x)     ((x) & 0x7)          // Gain Select.

/* Configuration Register: AD719x_CONF_CHAN(x) options */
#define AD719x_CH_AIN1P_AIN2M      0x1    // AIN1(+) - AIN2(-)       
#define AD719x_CH_AIN3P_AIN4M      0x2    // AIN3(+) - AIN4(-)       
#define AD719x_CH_TEMP_SENSOR      0x4    // Temperature sensor       
#define AD719x_CH_AIN2P_AIN2M      0x8    // AIN2(+) - AIN2(-)       
#define AD719x_CH_AIN1P_AINCOM     0x16   // AIN1(+) - AINCOM       
#define AD719x_CH_AIN2P_AINCOM     0x32   // AIN2(+) - AINCOM       
#define AD719x_CH_AIN3P_AINCOM     0x64   // AIN3(+) - AINCOM       
#define AD719x_CH_AIN4P_AINCOM     0x128  // AIN4(+) - AINCOM

/* Configuration Register: AD719x_CONF_GAIN(x) options */
//                                             ADC Input Range (3 V Reference)
#define AD719x_CONF_GAIN_1    0 // Gain 1    +-5 V
#define AD719x_CONF_GAIN_8    3 // Gain 8    +-375 mV
#define AD719x_CONF_GAIN_16   4 // Gain 16   +-187.5 mV
#define AD719x_CONF_GAIN_32   5 // Gain 32   +-93.75 mV
#define AD719x_CONF_GAIN_64   6 // Gain 64   +-46.875 mV
#define AD719x_CONF_GAIN_128  7 // Gain 128  +-23.4375 mV

/* ID Register Bit Designations (AD719x_REG_ID) */
#define ID_AD7190               0x4
#define ID_AD7195               0x6
#define AD719x_ID_MASK          0x0F

/* GPOCON Register Bit Designations (AD719x_REG_GPOCON) */
#define AD719x_GPOCON_BPDSW     (1 << 6) // Bridge power-down switch enable
#define AD719x_GPOCON_GP32EN    (1 << 5) // Digital Output P3 and P2 enable
#define AD719x_GPOCON_GP10EN    (1 << 4) // Digital Output P1 and P0 enable
#define AD719x_GPOCON_P3DAT     (1 << 3) // P3 state
#define AD719x_GPOCON_P2DAT     (1 << 2) // P2 state
#define AD719x_GPOCON_P1DAT     (1 << 1) // P1 state
#define AD719x_GPOCON_P0DAT     (1 << 0) // P0 state

class AD719x
{
  private:
  long spiClk = 4920000;

  public:
    /*讀寫*/
    unsigned long GetRegisterValue(unsigned char registerAddress,
                                   unsigned char bytesNumber);
    void SetRegisterValue(unsigned char registerAddress,
                          unsigned int registerValue,
                          unsigned char bytesNumber);
    /*運作狀態*/
  	void begin(bool staser);
  	void Reset(void);
    void SetPower(unsigned char pwrMode);
    void WaitRdyGoLow(void);
    
    /*配置寄存器*/
  	void RangeSetup(unsigned char polarity, unsigned char range);
    void ChannelSelect(unsigned short channel); 
    void ChopSetting(unsigned char chop);
    void ACXSetting(unsigned char acx); // only for AD7195
    
    /*模式寄存區*/
    void Calibrate(unsigned char mode, unsigned char channel);
    void ModeRegMISCSetting(unsigned char datasta,
                            unsigned char sinc,
                            unsigned char enpqr,
                            unsigned char scycle,
                            unsigned char rej60);
  	void DataRate(int filterRate);
    void CLKSetting(unsigned char clksrc);
    void SetWorkingMode(unsigned char Mode);

    /*轉換擷取*/
    void Set_Continuous_Read(unsigned char cread);
    unsigned long Read_Continuous_Data(void);
    unsigned long Read_Single_Conversion(unsigned short channel);

    /*數據轉換*/
    float TempSensorDataToDegC(unsigned long rawData);
    float DataToVoltage(unsigned long rawData,float Volt_Ref);

    /*除錯*/
  	void ReadRegisterMap(void);
};

#endif
