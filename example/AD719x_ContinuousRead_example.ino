#include "AD719x.h"
#include <SPI.h>

#define FS          96 /* 1=傳輸最快 (page.20 on DS)*/ /*58（10p)*/ /*12 (100p)*/
#define Polarity    1 /* 極性（0=單極性、1=雙極性） */
#define gain        AD719x_CONF_GAIN_64 /* 倍率 1, 8, 16, 32, 64, 128  (page.22 on DS)*/
#define datasta     1 /* DAT_STA */
#define sinc        0 /* 1= sinc3 , 0= sinc4 (sinc4在輸出速率較高時有更好的訊號品質)*/
#define enpqr       1 /* 奇偶較驗 */
#define scycle      1 /* 單週期轉換 */
#define rej60       1 /* 50/60Hz濾波 */
#define Vref        2.5 /*參考電壓*/

AD719x Adc;

void setup(){
  Serial.begin(115200);
  
  Adc.begin(0);
  Adc.ChopSetting(1);
  Adc.CLKSetting(AD719x_CLK_INT);
  
  Adc.Calibrate(AD719x_MODE_CAL_INT_ZERO, AD719x_CH_AIN1P_AIN2M);
  Adc.Calibrate(AD719x_MODE_CAL_INT_ZERO, AD719x_CH_AIN3P_AIN4M);

  Adc.RangeSetup(Polarity, gain);  

  Adc.Calibrate(AD719x_MODE_CAL_INT_FULL, AD719x_CH_AIN1P_AIN2M);
  Adc.Calibrate(AD719x_MODE_CAL_INT_FULL, AD719x_CH_AIN3P_AIN4M);

  Adc.ModeRegMISCSetting(datasta,sinc,enpqr,scycle,rej60);
  
  Adc.DataRate(FS);
  Adc.ChannelSelect(0x3);
  Adc.ReadRegisterMap();

  Adc.Set_Continuous_Read(1);
}

void loop(){

  unsigned long _aRead[2] = {0x0,0x0};
  
  _aRead[0] = Adc.Read_Continuous_Data();
  Serial.print(Adc.DataToVoltage(_aRead[0],Vref));


  _aRead[1] = Adc.Read_Continuous_Data();
  Serial.print(",");
  Serial.println(Adc.DataToVoltage(_aRead[1],Vref));

}
