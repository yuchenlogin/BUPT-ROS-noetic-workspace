|Address|Size(Byte)|Data Name|Description|---|Initial Value|是否有使用|
|---|---|---|---|---|---|---|
|0|2|Model_Number|Model_Number||0x73|NO|
|2|1|Firmware_Version |Firmware_Version ||0x13|YES|
|3|1|ID|ID ||0xc8|YES|
|4|1|Baud_Rate |Baud_Rate ||0x01|YES|
|5|1|Return_Delay_Time |Return_Delay_Time ||0x00|YES|
|6|2|CW_Angle_Limit	|CW_Angle_Limit	||0x00|YES|
|8|2|CCW_Angle_Limit |CCW_Angle_Limit ||0x0FFF|YES|
|11|1|Temperature_Limit|Temperature_Limit||0x50|YES|
|12|1|Min_Voltage_Limit |Min_Voltage_Limit ||0x3C|YES|
|13|1|Max_Voltage_Limit |Max_Voltage_Limit||0xA0|YES|
|14|2|Max_Torque|Max_Torque ||0x03FF|YES|
|16|1|Status_Return_Level|Select Types of Status Return||0x01|YES|
|17|1|Alarm_LED |Alarm_LED ||0x24|NO|
|18|1|P_ALARM_SHUTDOWN |开机时默认为0，设为1后停止响||0x0|YES|
|20|2|DOWN_CALIBRATION|DOWN_CALIBRATION||0x0000|NO|
|22|1|UP_CALIBRATION|UP_CALIBRATION||0x00|NO|
|24|1|P_DYNAMIXEL_POWER|P_DYNAMIXEL_POWER||0x00|YES|
|25|1|P_LED_PANNEL  |P_LED_PANNEL||0x00|YES|
|26|2|P_LED_HEAD|P_LED_HEAD||0x00|YES|
|28|1|P_LED_EYE|P_LED_EYE||0x20|YES|
|30|1|P_BUTTON|P_BUTTON||0x07D0|YES|
|38|2|P_GYRO_Z|P_GYRO_Z||0x0000|YES|
|40|2|P_GYRO_Y|P_GYRO_Y||0x0000|YES|
|42|2|P_GYRO_X|P_GYRO_X||0x0000|YES|
|44|2|P_ACC_X|P_ACC_X||0x00|YES|
|46|2|P_ACC_Y|P_ACC_Y||0x00|YES|
|48|2|P_ACC_Z|P_ACC_Z||0x0000|YES|
|50|1|P_PRESENT_VOLTAGE|P_PRESENT_VOLTAGE||0x00|YES|
|51|2|P_LEFT_MIC|P_LEFT_MIC||0x0000|YES|
|53|2|P_ADC_CH2|P_ADC_CH2||0x0000|YES|
|55|2|P_ADC_CH3|P_ADC_CH3||0x0000|YES|
|57|2|P_ADC_CH4|P_ADC_CH4||0x0000|NO|
|59|2|P_ADC_CH5|P_ADC_CH5||0x0000|YES|
|61|2|P_ADC_CH6|P_ADC_CH6||0x0000|NO|
|63|2|P_ADC_CH7|P_ADC_CH7||0x0000|NO|
|65|2|P_ADC_CH8|P_ADC_CH8||0x0000|NO|
|67|2|P_ADC_CH9|P_ADC_CH9||0x0000|NO|
|69|2|P_ADC_CH10|P_ADC_CH10||0x0000|NO|
|71|2|P_ADC_CH11|P_ADC_CH11||0x0000|NO|
|73|2|P_ADC_CH12|P_ADC_CH12||0x0000|NO|
|75|2|P_ADC_CH13|P_ADC_CH13||0x0000|NO|
|77|2|P_ADC_CH14|P_ADC_CH14||0x0000|NO|
|79|2|P_ADC_CH15|P_ADC_CH15||0x0000|NO|
|81|1|Firmware_Version0|Firmware_Version0||0x00|YES|
|82|1|Firmware_Version1|Firmware_Version1||0x00|YES|
|83|1|Firmware_Version2|Firmware_Version2||0x00|YES|



























