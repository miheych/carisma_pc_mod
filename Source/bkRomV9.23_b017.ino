#include <OneWire.h>
#include <EEPROM.h>
//#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
//Pins OBD/MUT
#define K_IN  9
#define K_OUT 8
//Hardware Slave SPI PINs
#define MOSI_PIN 11
#define SCK_PIN  13
#define SS_PIN 10
//temperature sensor pin
#define TEMP_PIN 3 
//Sofware Master SPI PINs
#define SOFT_MOSI_PIN 7
#define SOFT_SCK_PIN 6
#define SS_PIN_OUT 5
//Pins for export data
#define SERIAL_OUT A3
#define K_CONTROL A4 
#define DOOR_UNLOCK_PIN A5
//#define SYSSETUP // Меню настройки с консоли. В разработке.


const byte 	set_menu=2,
			set_var=1,
			show_value=3;
static PROGMEM const char menu1_file_0[] = "Press key:\n\r";      
static PROGMEM const char menu1_file_1[] = "1-Alternative method of reading\n\r";
static PROGMEM const char menu1_file_2[] = "2-Serial output via pin A3\n\r";
static PROGMEM const char menu1_file_3[] = "3-Fuel consumption correction\n\r";
static PROGMEM const char menu1_file_4[] = "4-Run correction\n\r";
static PROGMEM const char menu1_file_5[] = "5-VBAT correction\n\r";
static PROGMEM const char menu2_file_0[] = "Current value= \n\r";
static PROGMEM const char menu2_file_1[] = "Default value= \n\r";
static PROGMEM const char menu2_file_3[] = "1-Set value:\r\n";
static PROGMEM const char menu2_file_4[] = "0-Back\r\n";

struct menu
{
	byte item_id;
	const char *item_name;
	const byte cmd;
	byte attr;
	const byte param;  
};

menu menu_items[]=
{
	{10, menu1_file_0, NULL, NULL, NULL},
  {11, menu1_file_1, set_menu, 2, 1},
	{12, menu1_file_2, set_menu, 2, 2},
  {13, menu1_file_3, set_menu, 2, 3},
  {14, menu1_file_4, set_menu, 2, 4},
  {15, menu1_file_5, set_menu, 2, 5},
    {21, menu2_file_0, show_value, 100, NULL},
    {22, menu2_file_1, show_value, 100, NULL},
  	{23, menu2_file_3, set_var, 2, NULL},
    {20, menu2_file_4, set_menu, 1, NULL},
};


const uint8_t blink_mode=20, init_mode=21, READ_ATTEMPTS=15, REQUEST_ATT=5;
const boolean Dot=true, noDot=false, plus=true, Round=true, NoRound=false;
//константы отвечающие за работу отдельных модулей
const boolean serial_out_flag=true;

volatile uint8_t rx[4][11];
volatile byte pos, com;
volatile boolean process_it;
String 	str_dtc;
boolean flag=false, 
		start_on_flag=false,
		write_flag=false, 
		save_flag=true, 
//		change_mode=false, 
		Sign=true, 
		visible_flag=true,
		MUT_init_flag=false,
		DTC_read_flag=false,
		chModeFlag=false,
		mode_change_permit_flag=true,
		iter_10000_flag=false,
		Speed20LimitFlag=false,
		DoorBlockStatus=false,
    render_bit=true,
		save_data_flag=true;
int 	j=0, 
 init_cod,
		tp,
		volt_avg, 
		volt_, 
		volt,
		ISO_init_flag=-1,
		fuel_level_avg,
		fuel_level_avg_,
		dtc_number=0,
		t, 
		volt_lcd, 
		volt_result, 
		min_voltage=1300, 
		MUTvoltage, 
		MUTvoltage_, 
		coolantTemp,
		LTFT,
		Speed0,
		Speed;
unsigned int RPM, RPM_, 
		showParam, 
		showParam2, 
		readParam, 
		FuelConsumptionInHour0,
		FuelConsumptionInHour, 
		FuelConsumption,		
		iter_fuel_level=0,		
		InjPulseWidth;
unsigned long time_cam, 
		checkTemp_time, 
		displayU_time, 
		MUT_Read_time,
		Fuel_Cons_Read_time,
		save_time, 
		blink_time, 
		btn_press_time,
		btn_long_press_time,
		iso_init_time,
		micro_time_cam,
		loop_time,
		Scroll_time,
		stop_time,
		run,
		run_summ,
		RunInTrip,
		l,
		FuelConsSummary,
		FuelConsSummary2,
		FuelConsInTrip,
		prev,
		FuelConsumptionAvg,
		fuel_level,
		DoorLockImpulsTime,
		volt_result_long;
byte tx0[4][11], 
		tx[4][11], 
		rx0[4][11], 
		mode, 
		mode_save,
		mode_in_mem,
		x,
		item_count,
   menu_page=1,
		s0=5,
		s1=5,
		s2=5,
		s3=5,
		bl=0, 
		read_error=0,
		addr[8], 
		data[12];

OneWire  ds(TEMP_PIN); 

//AltSoftSerial OBDSerial;
SoftwareSerial OBDSerial(K_IN, K_OUT);
SoftwareSerial SerialOUT (NULL,SERIAL_OUT);

void setup() {
	if ( !ds.search(addr)) {
		//no more sensors on chain, reset search
		ds.reset_search();
		// return -1000;
	}
//настройки soft spi
	pinMode (SS_PIN_OUT, OUTPUT);

//обнуляем регистр управления SPI
	SPCR = B00000000;
//инициализируем последовательное соединение SPI
	SPCR =	((1<<SPIE) //прерывания
			|(1<<SPE)//spi enable
			|(1<<DORD)//младший бит первым
			|(1<<CPHA));

#ifdef SYSSETUP
	Serial.begin(115200);
	item_count = sizeof(menu_items) / sizeof(menu);
#endif
//инициализация вывода данных по software serial
	if(serial_out_flag){
		SerialOUT.begin(115200);
	}  
//определяем пины для работы с Hard SPI
	pinMode(MOSI_PIN, INPUT);
    pinMode(2, INPUT);
    digitalWrite(2, HIGH);
	digitalWrite(MOSI_PIN, HIGH);
	pinMode(SCK_PIN, INPUT);
	pinMode(SS_PIN, INPUT);
	digitalWrite(SS_PIN, LOW);
	pinMode(SOFT_MOSI_PIN, OUTPUT);
	pinMode(SOFT_SCK_PIN, OUTPUT);	
	pos = 0;
	com = 0;
	process_it = false;
	digitalWrite(4, HIGH);//подтягиваем контакт кнопки к +5В
	pinMode(DOOR_UNLOCK_PIN, OUTPUT);
	digitalWrite(DOOR_UNLOCK_PIN, LOW);
	pinMode(K_CONTROL, OUTPUT);
	digitalWrite(K_CONTROL, HIGH);// По умолчанию K-Line разорвана НЗ реле
	DDRC |= _BV(DDC1);//Конфигурируем A1 как выход.
	PORTC &= ~_BV(PORTC1);//Устанавливаем A1 в 0
	attachInterrupt (0, ss_falling, RISING); 
	EEPROM.get(0, mode_in_mem);
	if (mode_in_mem!=0xFF) mode=mode_in_mem;
	if(EEPROM.read(9)==1){
//		byte val_1[4];
		EEPROM.get(1, FuelConsSummary2);
		EEPROM.get(5, run_summ);
		EEPROM.get(10, FuelConsInTrip);
		EEPROM.get(14, RunInTrip);		
	}
	FuelConsumptionAvg = ((1000*FuelConsSummary2)/run_summ);
//очищаем дисплей
	for (int i=0; i<4; i++)
		for(int k=0; k<11; k++){
			tx0[i][k]=0x00;
			if((i==0)&&(k==10)) tx0[i][k]=0x08;
			if((i==1)&&(k==10)) tx0[i][k]=0x10;
			if((i==2)&&(k==10)) tx0[i][k]=0x20;
			if((i==3)&&(k==10)) tx0[i][k]=0x40;
		}
	SPI_WriteArray(&tx0[0][0]);
}
//обработка прерывания 0(падение SS в 0)
void ss_falling ()
{ 
	SPCR |= (1<<SPIE)|(1<<SPE); 
//	pos=0;
}  
//обработка прерывания SPI
ISR (SPI_STC_vect){
	uint8_t c = SPDR;  // grab byte from SPI Data Register
	rx[com][pos] = c;
	pos++;      
	if(pos==11){ 
		com++; 
		pos=0;
	}  
	if (com==4){
		detachInterrupt(0);
		process_it = true;
		SPCR &= (~(1<<SPIE))&(~(1<<SPE));//отключаем прерывание SPI пока данные не будут обработаны	
	}  
}

void loop() {
	time_cam = millis();
//	обработка данных полученных с контроллера дисплея по SPI	
    if (process_it){
	//	if (rx[0][10]==0) write_flag=true;
		if ((rx[0][10]==0x08)&&(rx[1][10]==0x10)&&(rx[2][10]==0x20)&&(rx[3][10]==0x40)){
			for(int i=0; i<4; i++){
				for(int p=0; p<11; p++){
					rx0[i][p] = rx[i][p]|0x00;
				}
			}
		}
 //---------------------------------------------for test in terminal		
/* 		for(int p=0; p<4; p++){
			Serial.println();
			for (int i=0; i<11; i++){
				Serial.print(rx[p][i],HEX);
				Serial.print(":");
			}  
		}
		Serial.println();
		Serial.println("--------------------------------------------");
 *///----------------------------------------------end test
		pos = 0;
		com = 0;
		process_it = false;
		attachInterrupt (0, ss_falling, FALLING); 
//		SPCR |= (1<<SPIE)|(1<<SPE);		
	} // end process_it
	write_flag=true;	
	for(int i=0; i<4; i++)
		for(int p=0; p<11; p++)
			tx[i][p] = rx0[i][p];

//Чтение уровня бортового напряжения
	volt = analogRead (0);
	volt_=volt_+volt;
	j++;
	if (j==50){
		volt_avg=volt_/50;
		volt_=0;
		j=0;
		volt_result_long=long(volt_avg)*396;
		volt_result =(int(volt_result_long/100));
	}   	
	volt=volt*4;
	//Расчет остатка топлива. Пока никуда не выводится. Надо тестировать.
	//uint16_t Vcc=VCCLevel();
	uint16_t fuel_level0=FuelLevel(4950);
	if(fuel_level0>100){ ///условия инициализации - коннект только если есть напряжение с датчика топлива(включено зажигание)
		fuel_level+=fuel_level0;
		iter_fuel_level++;
		if (iter_fuel_level==10000){
			iter_10000_flag=true;
			fuel_level_avg=int(fuel_level/10000);
			fuel_level=0;
			iter_fuel_level=0;
		}
		if (!iter_10000_flag){
			if ((iter_fuel_level%100)==0){
				fuel_level_avg=int(fuel_level/iter_fuel_level);
			}
		}
		fuel_level_avg_=(3450-fuel_level_avg)*10/3;
	//конец расчета остатка топлива
	//Запуск и условия процедуры MUT_init
		if((!MUT_init_flag)&&(mode>2)&&(mode!=9)) { //INIT если нет установленной сессии и номер режима 3-8
//		if((volt[j] > 1370)||((volt[j] < 1180)&&(volt[j] > 1100))){
			digitalWrite(K_CONTROL, LOW); //Замыкаем НЗ реле для соединение линии K-Line

			ISO_init_flag=-1;//Возвращаем надпись DTC MODE при следующем входе в режим 9
			if (!start_on_flag) {
				start_on_flag=true;
				stop_time=time_cam;
				tp=6000;//пауза перед MUT_init после включения зажигания
			}
			//x=mode;
			mode=init_mode;
			if (tp==1) {
				MUT_init_flag=true;
				mode=mode_in_mem;
				tp=0;
			}
			if (!MUT_init_flag){
				if(time_cam-stop_time>2000+tp){			
					if((init_cod=MUT_init())==1) {
						tp=1;
					}
					else {
						OBDSerial.end();
						tp=8000;
						stop_time=time_cam;
					}	
				}	
				if (time_cam-stop_time>tp){
					if (tp==1) writeSTR("INIT OK",0);
					else writeSTR("MUT INIT",0);	
				}
				else{//2 сек на показ ошибки, далее показываем WAITING
					if (time_cam-stop_time>4000) writeSTR("WAITING",0);
					else{
						if (tp==8000){ 
							writeSTR("FAIL",4);
							writeSym(plus, noDot, NoRound, init_cod*10);
						}
					}
				}	
			}
		}	
	}
	else {
		MUT_init_flag=false;
		tp=0;
		stop_time=time_cam;
		RPM_=0;
		start_on_flag=false;
		if(DoorBlockStatus==true) {
			if(DoorLockImpulsTime<300000){
				DoorLockImpulsTime+=loop_time;
				digitalWrite(DOOR_UNLOCK_PIN, HIGH); // Подаем импульс разблокировки на замок
			}
			else {
				digitalWrite(DOOR_UNLOCK_PIN, LOW); // Убираем импульс с замка
				DoorBlockStatus=false;
				DoorLockImpulsTime=0;
			}	
		}
	}

	if(MUT_init_flag) {
		//Затираем символы в верхней строке расхода
		tx[2][3]&=0xFE; //1 символ
		tx[0][2]=0x00; tx[1][2]=0x00; tx[2][2]=0x00; tx[3][2]=0x00;//2,3,4 символы
	//	tx[1][2]=0x00; // стереть - - -
	//	tx[3][2]=0x00;//стереть 0x80 - L/100
		tx[0][7]=0x00;//стереть 0x40 - km	
		tx[0][10]=0x08;
		tx[1][10]=0x10;
		tx[2][10]=0x20;
		tx[3][10]=0x40;

		LoadBar(get_Load());
		RPM_= get_RPM();
		InjPulseWidth = get_InjPulseWidth();
		FuelConsumptionInHour = int((51 * long(InjPulseWidth) * ((long(RPM_))/10))/100000L);//до 52 было 45 -->51 
		Speed = get_Speed();
		micro_time_cam=micros();
		loop_time=micro_time_cam-l;
		l = micro_time_cam;
		if ((loop_time>0)&&(Speed>=0)) {
			run = run + ((long(Speed0/10+Speed/10) * 139)*loop_time/10)/10;// 1.0E-7 m
//			run = run + (((Speed/10) * 28)*loop_time)/10; // 1.0E-7 m
			Speed0=Speed;
			FuelConsSummary = FuelConsSummary + (((FuelConsumptionInHour0/10+FuelConsumptionInHour/10)*139)*loop_time/10)/10;
			FuelConsumptionInHour0=FuelConsumptionInHour;
		}	
		
		if ((Speed < 0)||(InjPulseWidth<0)){//если больше 5 ошибок чтения подряд, сбрасываем сессию
			if ((read_error++)>5){
				MUT_init_flag = false;
				stop_time=time_cam;
                read_error=0;
				Speed=0;
				Speed0=0;
				RPM_=0;
				FuelConsumptionInHour0=0;
				FuelConsumptionInHour=0;
			}
        }
		else read_error=0;
		if (run>=1000000000L){// if run==100м 
			run_summ++;
			RunInTrip++;
			run=0;
		}
		if (FuelConsSummary>=1000000000L){// if fuel cons ==10mL
			FuelConsSummary2++;
			FuelConsInTrip++;
			FuelConsSummary=0;
			if ((FuelConsSummary2-prev>=1)&&(run_summ!=0)) {//если израсходованы очередные 10 мл топлива и пробег не равен нулю
				prev=FuelConsSummary2;
				FuelConsumptionAvg = ((1000*FuelConsSummary2)/run_summ);
			}
		}
		if (!DoorBlockStatus){ // Блокировка дверей при Speed > 20 km/h. Продолжительность импульса 300 мс
			if ((Speed/10) > 20){
				Speed20LimitFlag=true;
			}
			if (Speed20LimitFlag){
				if(DoorLockImpulsTime<300000){
					DoorLockImpulsTime+=loop_time;
					PORTC |= _BV(PORTC1);//Устанавливаем A1 в 1
				}
				else {
					PORTC &= ~_BV(PORTC1);//Устанавливаем A1 в 0
					DoorBlockStatus=true;
					DoorLockImpulsTime=0;
					Speed20LimitFlag=false;
				}	
			}
		}	
		if (time_cam - Fuel_Cons_Read_time>=300){
			Fuel_Cons_Read_time = time_cam;
			showParam2=int(FuelConsumptionAvg);//отображаем средний расход в верхней строке
		}
		tx[3][2]=0x80;//L/100
		tx[0][7]=0x40;//km		
		writeSym2(Dot, showParam2);	
		coolantTemp=get_CoolantTemp();
		if ((coolantTemp < 90)&&(coolantTemp != -410)&&(chModeFlag==false)&&(mode!=blink_mode)&&(mode!=init_mode)){//отображение температуры пока она < 9 C
			mode=6;
			chModeFlag=true;			
		} 
		else {
			if ((chModeFlag==true)&&(coolantTemp >=90)){
				mode=mode_in_mem;
				chModeFlag=false;
			}  
		}  
	}
/*	if ((volt_result < min_voltage)&&(bl==0)&&(time_cam > 1000)){ //показ напряжения батареи при его опускании ниже порога. Пока не требуется
		bl = 8;
		blink_time = time_cam;
		visible_flag = false;
		mode_save = mode;
		mode = blink_mode;
	} */
//обработка кнопки смены режимов
	if((!(PIND&(1<<4)))&&(flag==false)&&((time_cam - btn_press_time) >100)){
		btn_long_press_time=time_cam;
		flag=true;
		btn_press_time = time_cam;
	}
	if(mode_change_permit_flag&&(PIND&(1<<4))&&(flag==true)&&((time_cam - btn_press_time) >300)){ 
		mode++;
		if(mode>9) mode=0;
		flag=false;
		save_flag=false;
		save_time=time_cam;
		
	}  
//	if(mode!=init_mode) {
		ModeNumb(mode);
//	}
//	else ModeNumb(x);	
switch (mode) {
  case 0: {    
    #ifdef SYSSETUP
//		if (time_cam - MUT_Read_time>=300){
//			MUT_Read_time = time_cam;
      SystemSetup();
//}
    #endif 
	if ((flag==true)&&(time_cam-btn_long_press_time>3000)){
		btn_press_time = time_cam;
		mode_change_permit_flag=false;
		if(PIND&(1<<4))	{
			flag=false;
			MUT_init_flag=false;//принудительное отключение обмена по MUT
			digitalWrite(K_CONTROL, HIGH); //разрываем K-Line НЗ реле
			mode_change_permit_flag=true;
		}	
	}				
  }
  break;
  case 1: { //режим темпереатуры
    if ((time_cam-checkTemp_time) >=1000) {
		t=TempRead();     
		checkTemp_time = time_cam;
		if (t & 0x8000){ // test most sig bit
			Sign=false;
		} 
		else Sign=true;               
		if (!Sign){ // negative
			t = (t ^ 0xffff) + 1; // 2's comp
		}
		t = (6 * t) + t / 4;
    }
    writeSym(Sign, true, Round, t);
	writeSTR("TEMP",4);
  }
  break;
  case 2: { //режим вольтметра
    if (time_cam - MUT_Read_time >=300){
		if(MUT_init_flag) {
			showParam=get_MUT_voltage();
		}
		else showParam=volt_result;
		MUT_Read_time = time_cam;
    }
    writeSym(plus, Dot, Round, showParam);
	writeSTR("VBAT",4); 
  }
  break;
	case 3: { //пробег за поездку(от сроса до сброса)
  		if ((flag==true)&&(time_cam-btn_long_press_time>3000)){
			btn_press_time = time_cam;
			mode_change_permit_flag=false;
			if(PIND&(1<<4))	{
				flag=false;
				mode_change_permit_flag=true;
			}
			RunInTrip=0;	
		}						
		if (time_cam - MUT_Read_time>=300){
			MUT_Read_time = time_cam;
			showParam=int(RunInTrip)*10;
//			showParam=int(loop_time/10);
		}
		writeSym(plus, Dot, NoRound, showParam);
		writeSTR("RUN",5);
	}
	break;
	case 4: { //cуммарный расход топлива(от сроса до сброса)
		if ((flag==true)&&(time_cam-btn_long_press_time>3000)){
			btn_press_time = time_cam;
			mode_change_permit_flag=false;
			if(PIND&(1<<4))	{
				flag=false;
				mode_change_permit_flag=true;
			}
			FuelConsInTrip=0;	
		}						
		if (time_cam - MUT_Read_time>=300){
			MUT_Read_time = time_cam;
				showParam = int(FuelConsInTrip);
			}
		writeSym(plus, Dot, NoRound, showParam);
		writeSTR("SUMMARNIY RASHOD TOPLIVA",4);
	}
	break;
	case 5: { //мгоновенный расход, сброс показаний среднего расхода	
		if ((flag==true)&&(time_cam-btn_long_press_time>3000)){ //сброс в 0 показаний среднего расхода
			btn_press_time = time_cam;
			mode_change_permit_flag=false;
			if(PIND&(1<<4))	{
				flag=false;
				mode_change_permit_flag=true;
			}	
			FuelConsSummary2=0;
			run_summ=0;
			FuelConsumptionAvg=0;	
		}	
		
		if ((Speed/10) > 5){
			FuelConsumption = int(((long(FuelConsumptionInHour))*100)/long(Speed/10));
			writeSTR("L100",4);
		}
		else {
			FuelConsumption = FuelConsumptionInHour;
			writeSTR("L1H",4);
		}
		if (time_cam - MUT_Read_time>=300){
			MUT_Read_time = time_cam;
			showParam=FuelConsumption;
		}
		writeSym(plus, Dot, Round, showParam);
//		writeSTR("SREDNIY RASHOD",4);
	}
	break;
	case 6: { //чтение температуры охлаждающей жидкости по MUT
		if(MUT_init_flag){
      		//coolantTemp=get_CoolantTemp();ситывается ранее
			if (time_cam - MUT_Read_time>=300){
				if (coolantTemp < 0){
					coolantTemp = abs(coolantTemp);
					Sign=false;
				}
				else Sign=true;
				MUT_Read_time = time_cam;
				showParam=coolantTemp;
			}
			writeSym(Sign, noDot, Round, showParam);
		}
		writeSTR("COOLANT TEMP",3);
	}
	break;
	case 7: { //чтение оборотов двигателя по MUT
		if(MUT_init_flag){
      			//readParam = get_RPM();
				if (time_cam - MUT_Read_time>=300){
					MUT_Read_time = time_cam;
					showParam=RPM_;
			}
			writeSym(plus, noDot, Round, showParam);
		}
		writeSTR("RPM",5);
	}
	break;
	case 8: { //LTFT
		writeSTR("LTFT",4);
		if(MUT_init_flag){
			if (time_cam - MUT_Read_time>=0){
				MUT_Read_time = time_cam;
				LTFT = get_LTFTLo();
				if (LTFT < 0){
					LTFT = abs(LTFT);
					Sign=false;
				}
				else Sign=true;
				showParam = LTFT;
			} 
//			showParam=fuel_level_avg_;
//			writeSym(plus, Dot, Round, showParam);
			writeSym(Sign, Dot, Round, showParam);
		}
	}
	break;
	case 9: {// чтение ошибок
		if((time_cam-iso_init_time>10000)&&(ISO_init_flag==0)){
			iso_init_time=time_cam;
			ISO_init_flag=ISO_init();	
		}
	
		if ((MUT_init_flag==false)&&(time_cam-btn_long_press_time>10000)&&(ISO_init_flag==-3)){
			ISO_init_flag=0;
		}
	
		if ((flag==true)&&(time_cam-btn_long_press_time>3000)){
			btn_press_time = time_cam;
			mode_change_permit_flag=false;
			ISO_init_flag=-2;
			if(PIND&(1<<4))	{
				ISO_init_flag=-3;
				mode_change_permit_flag=true;
				DTC_read_flag=false;
				flag=false;
				if ((dtc_number>0)&&(MUT_init_flag==false)){
					Clear_ISO_DTC();
					dtc_number=0;
					ISO_init_flag=1;
				}
				MUT_init_flag=false;//отключение обмена по MUT			
			}
		}
		if (ISO_init_flag==-1)writeSTR("DTC MODE",0);
		if ((ISO_init_flag==-2)or(ISO_init_flag==-3)) writeSTR("WAIT",0);
		if (ISO_init_flag==0) writeSTR("ISO INIT",0);
			
		if (ISO_init_flag==1) {
			if(time_cam-iso_init_time>3000){
/* 				if (time_cam - MUT_Read_time>=100){
					MUT_Read_time = time_cam;
					showParam = get_ISO_RPM(); 
				}
				writeSym(plus, noDot, Round, showParam); */				
				if (!DTC_read_flag){
					dtc_number=get_mode01_pid01();
					DTC_read_flag=true;
					if (dtc_number>0){
						str_dtc=get_ISO_DTC(dtc_number);
					}
					if (dtc_number==0){
						str_dtc="NO DTC";
					}
					if (dtc_number==-1){
						str_dtc="ERR READ";
					}					
				}				
				char charDtc[sizeof(str_dtc)];
				str_dtc.toCharArray(charDtc, sizeof(charDtc));                                
				writeSTR(charDtc,0);        
			}
			else writeSTR("INIT OK",0);	
		}
		if (ISO_init_flag>1) {
			String str=String(ISO_init_flag, HEX);
			char charVar[sizeof(str)];
			str.toCharArray(charVar, sizeof(charVar));		
			writeSTR(charVar,0);	
		}
	}
	break;
/* 	case init_mode:{ //init 	   
		writeSTR("MUT INIT",0);
		mode=x;
	}
	break;	 
	case blink_mode:{ //режим индикации пониженнного напряжения 	   
		if (visible_flag){
			writeSym(true, true, Round, volt_result);
			writeSTR("VBAT",4);        
		} 
		if ((time_cam - blink_time) >= 700){
			visible_flag = !visible_flag;			
			blink_time = time_cam;
			bl--;
		}  
		if (bl==1) mode = mode_save;	  
	}
	break;*/
}

//сохранение режима----
	if ((time_cam-save_time>=30000)&&(save_flag==false)){
		EEPROM.put(0, mode);// 0 байт под номер режима
//		Serial.println("wr");
		save_flag=true;
	}
	if(RPM_>7000) save_data_flag=false;
	if((RPM_==0)&&save_data_flag==false){
//		byte val_1[4];
		DoorBlockStatus=false;//Активируем возможность блокировки дверей после того как двигатель заглушен
		save_data_flag=true;
		EEPROM.put(9, 1);	//флаг наличия сохраненных данных в EEPROM. 
		EEPROM.put(1, FuelConsSummary2);
		EEPROM.put(5, run_summ);
		EEPROM.put(10, FuelConsInTrip);
		EEPROM.put(14, RunInTrip);
		
// старая версия работы с EEPROM:		
/* 		for (int i=0; i<4; i++){ // 1-4 байты под суммарный расход
			val_1[i] = ((FuelConsSummary2 >> i*8) & 0xFF);
			EEPROM.write(1+i, val_1[i]);
		}
		for (int i=0; i<4; i++){//5-8 байты под суммарный пробег
			val_1[i] = ((run_summ >> i*8) & 0xFF);
			EEPROM.write(5+i, val_1[i]);
		}
		for (int i=0; i<4; i++){ // 10-13 под расход за поездку
			val_1[i] = ((FuelConsInTrip >> i*8) & 0xFF);
			EEPROM.write(10+i, val_1[i]);
		}
		for (int i=0; i<4; i++){ // 14-17 под расход за поездку
			val_1[i] = ((RunInTrip >> i*8) & 0xFF);
			EEPROM.write(14+i, val_1[i]);
		} */				
	}// свободно с 18 байта
//----------------------  

	for (int i=0; i<4; i++){
		for(int k=0; k<11; k++){
			if (tx0[i][k]!=tx[i][k]){
				tx0[i][k]=tx[i][k];
				write_flag=true;
			}
		}
	}		
//вывод информации на дисплей
	if (write_flag){
		SPI_WriteArray(&tx0[0][0]); 
		write_flag=false;
		// Serial.println("wr");
	}
//вывод информации по serial
	if(serial_out_flag){
		tx_int(0x29, InjPulseWidth);//длительность впрыска
		tx_int(0x21, RPM_/10);//RPM
		tx_int(0x2F, Speed/10);//Скорость
		tx_int(0x10, coolantTemp/10);//Температура охлаждающей жидкости
	}  
}//end of loop
//Процедура настройки по serial
#ifdef SYSSETUP
void SystemSetup(){
  if (Serial.available()or render_bit){
	  Serial.print(F("\033[10A\r"));//возврат курсора
    for(byte i=0; i<10; i++){
       Serial.print(F("\r"));
       for(byte k=0; k<40; k++){
          Serial.print(" ");
       }
       Serial.print(F("\n"));
    }
    Serial.print(F("\033[10A\r"));
	  MenuPrint(menu_page);
 }
}
void MenuPrint(byte idx){
  byte cmd, input=-1;
  char a;
  a=Serial.read();
  if ((a>=48)and(a<=57)) input=atoi(&a);//Проверка на ввод цифр
  while (Serial.available()){//очищаем буфер
    Serial.read();
  }
  render_bit=false;
	for (byte k=0; k<item_count; k++){
		if (menu_items[k].item_id/10==idx) {
		  ItemPrint(k);
		}
    if (menu_items[k].item_id==input+idx*10){
      cmd=menu_items[k].cmd;
      if (cmd==set_menu) {
        menu_page=menu_items[k].attr;
        render_bit=true;
      }
      if (cmd==set_var) ;
    }
	} 
}
void ItemPrint(byte item_id){
   int len = strlen_P(menu_items[item_id].item_name);
   for (byte k=0; k<len; k++){
      char myChar =  pgm_read_byte_near(menu_items[item_id].item_name+k);
      Serial.print(myChar);
   }
}
#endif

//передача данных по SoftwareSerial
void tx_int(byte id, int data){
	byte i, *tx_byte;
	tx_byte=(byte*)&data;
	SerialOUT.write(id);
	SerialOUT.write(id);
	for(i=0; i<2; i++){
		SerialOUT.write(*(tx_byte++));
	}	
}
void spi_send_byte(uint8_t c){
	//PORTD &= ~(1<<SS_PIN_OUT);
	for (byte bit = 8; bit > 0; bit--){
	if (c & 0x01)
		PORTD |= 1<<SOFT_MOSI_PIN;
	else
		PORTD &= ~(1<<SOFT_MOSI_PIN); 
	delayMicroseconds (1);	
	PORTD |= 1<<SOFT_SCK_PIN;
	delayMicroseconds (1);
	PORTD &= ~(1<<SOFT_SCK_PIN); 
//	delayMicroseconds (1);
    c >>= 1;
}
	//PORTD |= 1<<SS_PIN_OUT;
}
//передача данных на дисплей
void SPI_WriteArray(uint8_t *data)
{
	for (byte i=0; i<4; i++){
		digitalWrite(SS_PIN_OUT, LOW); 
		for(byte k=0; k<11; k++){
			spi_send_byte (*data);
			//bbSPI.transfer(*data);
			*data++; 
		}
		digitalWrite(SS_PIN_OUT, HIGH); 
	}	
}
//Измерение напряжения VCC
uint16_t VCCLevel(){
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	ADCSRA |= _BV(ADSC); // начало преобразований
	while (bit_is_set(ADCSRA, ADSC)); // измерение
	uint8_t low = ADCL; // сначала нужно прочесть ADCL - это запирает ADCH
	uint8_t high = ADCH; // разлочить оба
	long result = (high<<8) | low;
	result = 1524270L / result; // 1125300 = 1.1*1023*1000; Результат Vcc в милливольтах 
	return uint16_t(result);
}
uint16_t FuelLevel (uint16_t Vcc){
	long value = analogRead(2);
	long volt = (((value*1000) / 1023) * Vcc)/1000;
	return uint16_t(volt);
}
//инициаализация ISO
int ISO_init(){
	byte i=0, error=0;
	int a, b[3];
//	Serial_tx_off(); //disable UART so we can "bit-Bang" the slow init.
//	Serial_rx_off();
	OBDSerial.end();
	pinMode(K_OUT, OUTPUT);
	pinMode(K_IN, INPUT);
	digitalWrite (K_OUT, LOW);//start bit 
	delay(200);
	a=0x33; //(00110011)
	for (byte mask = 0x01; mask != 0; mask <<= 1) 
	{ 
		if (a & mask)  
			digitalWrite(K_OUT, HIGH);  
		else 
			digitalWrite(K_OUT, LOW); 
		delay(200); 
	} 
	digitalWrite(K_OUT, HIGH); //stop bit
	delay(260); 
	OBDSerial.begin(10400);
	//delay(60);//60 < W1 < 300
	while (i<3){
		if((b[i]=iso_read_byte())!=-1){
		  i++;
		  delay(10);
		}
		else error++;
		if (error > 70) break;
	}
//	if ((b[0]==0x55)&&(b[1]==0x08)&&(b[2]==0x08)){ 
	if (b[0]!=0x55) return (b[0]*0x10+5);
	if (b[1]!=0x08) return (b[1]*0x10+2);
	if (b[2]!=0x08) return (b[2]*0x10+3);
	delay(25);//25ms < W4 < 50ms 
	OBDSerial.write(0xF7);
	OBDSerial.flush();
	delay(25);//25ms < W4 < 50ms 
//	}	
//	else return 0;
//	if((a=iso_read_byte())!=0xF7){
//		return 6;
//	}
	if((a=iso_read_byte())==0xCC){
		delay(100);//55ms<P3<5s
		return 1;
	}
	else return 7;
}
//ISO checksum
byte iso_checksum(int *data, byte len)
{
  byte crc=0;
  for(byte i=0; i<len; i++)
    crc=crc+data[i];
  return crc;
}

//запрос оборотов ISO
unsigned int get_ISO_RPM(){ 
	int b[6]={0x68,0x6A,0xF1,0x01,0x0C,0xD0}, c[8];
	long v;
	boolean error;
	byte iter=0;
	//b[5]=iso_checksum(b,5);
	do{
		iter++;
		error=false;
		for (int i = 0; i < 6; i++){
			OBDSerial.write(b[i]);
			delay(5);//5<P4<20
		}
		delay(20);//25<P2<50 with key of 0x08
	/* 	for (int i = 0; i < 6; i++){
			if ((c[i]=iso_read_byte())==b[i]){
			}
			else return 0;
		}*/
		for (int i = 0; i < 8; i++){ 
			c[i]=iso_read_byte();
//			Serial.print(String(c[i],HEX));
//			Serial.print(':');
			if (c[i]==-1) {
				error=true;
			}
		}
//		Serial.println();	
	}while((c[7]!=iso_checksum(c,7))&&(iter < REQUEST_ATT));//and(error==true)
	if(!error){
		v = (c[5]*256+c[6])/4;//
		return (int(v*10));
	}
	else return 0;
}
//MODE 01 PID 01 return 4 byte: A7 - MIL status, A6-A0 - DTC_CNT
int get_mode01_pid01(){
	int b[6]={0x68,0x6A,0xF1,0x01,0x01,0x00}, c[10];
	byte read_att=0;
	b[5]=iso_checksum(b,5);
	c[9]=0x00;
	do{
		for (int i = 0; i < 6; i++){
			OBDSerial.write(b[i]);
			delay(5);
		}
		delay(20);//25<P2<50 with key of 0x08
/* 		for (int i = 0; i < 6; i++){
			if ((c[i]=iso_read_byte())==b[i]){
			}
			else return -1;
		}
 */		for (int i = 0; i < 10; i++){
			c[i]=iso_read_byte();
		}
		read_att++;
	} while ((c[9]!=iso_checksum(c,9)) && (read_att<REQUEST_ATT));	
	if (c[9]==iso_checksum(c,9)){
		if (0x80 & c[5]) return (0x7F & c[5]);
		else return 0;
	}
	else return -1;
}
//Чтение ошибок ISO 9141
String get_ISO_DTC(int dtc_numb){
	String dtc;
	int b[5]={0x68,0x6A,0xF1,0x03,0xC6}, c[11]; 
	byte read_att=0;
	b[4]=iso_checksum(b,4);
	c[10]=0x00;
	do{
		for (int i = 0; i < 5; i++){
			OBDSerial.write(b[i]);
			delay(5);
		}
		delay(20);//25<P2<50 with key of 0x08		
/* 		for (int i = 0; i < 5; i++){
			if ((c[i]=iso_read_byte())==b[i]){
			}
			else return "E";
		}
 */		for (int i = 0; i < 11; i++){
			c[i]=iso_read_byte();
		}
		read_att++;
	} while ((c[8]!=iso_checksum(c,8)) && (read_att<REQUEST_ATT));
	if (c[10]==(iso_checksum(c,10))){
		if (dtc_numb>=1){
			if ((0xF0 & c[4])==0) {
				dtc="P0" + String(((c[4]<<8)+c[5]), HEX);
			}		
		}
		if (dtc_numb==2){
			if ((0xF0 & c[6])==0) {
				dtc=dtc+" P0" + String(((c[6]<<8)+c[7]), HEX);
			}		
		}	
		return dtc;
	}
	else return ("CS ERROR "+String(c[10],HEX));
}
//Сброс ошибок ISO 9141
int Clear_ISO_DTC(){
	int b[5]={0x68,0x6A,0xF1,0x04,0x00};
	b[4]=iso_checksum(b,4);
	for (int i = 0; i < 5; i++){
		OBDSerial.write(b[i]);
		delay(5);
	}
/* 	for (int i = 0; i < 5; i++){
		if ((c[i]=iso_read_byte())==b[i]){
			return 1;
		}
		else return -1;
	} */
}
//инициализация MUT
int MUT_init(){
	int i=0, error=0;
	byte b[3];
	OBDSerial.end();//disable UART so we can "bit-Bang" the slow init.
//	OBDSerial_tx_off(); 
//	OBDSerial_rx_off();
	pinMode(K_OUT, OUTPUT);
	pinMode(K_IN, INPUT);
	digitalWrite (K_OUT, HIGH);//старт инита
	delay(300);
	digitalWrite (K_OUT, LOW); //стартовый бит
	delay(200);
	digitalWrite (K_OUT, LOW);
	delay(200*8);
	digitalWrite (K_OUT, HIGH); //стоповый бит
	delay(200);
	OBDSerial.begin(15625);
	delay(120);
	while (i<3){
		if((b[i]=iso_read_byte())!=-1){
		  i++;
		}
		else error++;
	if (error > 5) return 5;
	}
/* 	if ((b[0]==0x55)&&(b[1]==0xEF)&&(b[2]==0x85)){ 
		return 1;
	}	
	else return 0; */
	if (b[0]!=0x55) return 1000+b[0];
	if (b[1]!=0xEF) return 2000+b[1];
	if (b[2]!=0x85) return 3000+b[2];
	return 1;
}
//длительность впрыска форсунок
int get_InjPulseWidth (){
	int b=0,c=0;
	long v;
	byte iter=0;
	do{
		iter++;
		OBDSerial.write(0x29);
		OBDSerial.flush();
//		b = iso_read_byte(); //эхо
		b = iso_read_byte(); 
		OBDSerial.write(0x2A);
		OBDSerial.flush(); 
//		c = iso_read_byte(); //эхо
		c = iso_read_byte();
	}while((iter < REQUEST_ATT)&&((b==-1)||(c==-1)));	
	if ((c!=-1)&&(b!=-1)){
		v = (long(b)*256)+(long(c));
		return int(v/10);
	}	
	else return 0;
}
//Чтение активных DTC, младший байт
int get_ActLoDTC (){
	int b=0;
	OBDSerial.write(0x47);
	if ((b=iso_read_byte())==0x47){
		b = iso_read_byte();
		return b;
	}
	else return -1;	
}
//Чтение активных DTC, старший байт
int get_ActHiDTC (){
	int b=0;
	OBDSerial.write(0x48);
	if ((b=iso_read_byte())==0x48){
		b = iso_read_byte();
		return b;
	}
	else return -1;	
}	
//Очистка ошибок 0xFA or 0xFC
int ClearDTC (){
	int b=0;
	OBDSerial.write(0xFC);
	if ((b=iso_read_byte())==0xFC){
		return 1;
	}
	else return -1;	
}
//запрос температуры двигателя
int get_CoolantTemp(){ 
	int b=0;
	byte iter=0;
	do{
                iter++;
  		OBDSerial.write(0x10);
		OBDSerial.flush();
//		b = iso_read_byte();//эхо
		b = iso_read_byte() - 40;
	}while((iter < REQUEST_ATT)&&(b==-1));
	if(b!=-1){
		return (b*10);
	}
	else return -1;
}
//запрос скорости
int get_Speed(){ 
	int b=0;
	byte iter=0;
	do{
		iter++;
		OBDSerial.write(0x2F);
		OBDSerial.flush();
//		b = iso_read_byte(); //эхо
		b = iso_read_byte();
	}while((iter < REQUEST_ATT)&&(b==-1));
	if(b!=-1){	
		return (b*20);
	}
	else return -100;
}
//запрос нагрузки на двигатель
int get_Load(){ 
	int b=0;
	byte iter=0;
	do{
		iter++;
		OBDSerial.write(0x1C);
		OBDSerial.flush();
//		b = iso_read_byte(); //эхо
		b = iso_read_byte();
	}while((iter<3)&&(b==-1));
	if(b!=-1){
		return b;
	}
	else return -100;
}
//запрос оборотов двигателя в минуту
unsigned int get_RPM(){ 
	int b=0;
	long v;
	byte iter=0;
	do{
		iter++;
		OBDSerial.write(0x21);
		OBDSerial.flush();
//		b = iso_read_byte(); // эхо
		b = iso_read_byte();
	}while((b==-1)&&(iter < REQUEST_ATT));	
	if (b!=-1){
		v = long(b)*3125;
		return (int(v/10));
	}
	else return 0;
}
//запрос долговременной топливоной коррекции LTFTLo 
int get_LTFTLo (){
	byte iter=0;
	int b=0;
	long v;
	do{
		iter++;
		OBDSerial.write(0x0C);
		OBDSerial.flush();
//		b = iso_read_byte(); //эхо
		b = iso_read_byte();
	}while((b==-1)&&(iter<REQUEST_ATT));
	if (b!=-1){
		v = (long(b)*196)-25000;
		return int(v/10);
	}
	else return 0;
}
//Long Term Idle Correction
int get_LTIC (){ 
	int b=0;
	long v;
byte iter=0;
	do{
  iter++;
		OBDSerial.write(0x08);
		OBDSerial.flush();
		b = iso_read_byte();
		b = iso_read_byte()-128;
	}while((b==-1)&&(iter<REQUEST_ATT));
	if (b!=-1){
		return (b*10);
	}
	else return 0;
}
//запрос бортового напряжения
int get_MUT_voltage(){
	byte iter=0;
	int b=0;
	long v;
	do{
		iter++;
		OBDSerial.write(0x14);
		OBDSerial.flush();
//		b = iso_read_byte(); //эхо
		b = iso_read_byte();
	}while((b==-1)&&(iter<REQUEST_ATT));
	if (b!=-1){
		v = long(b)*7333;
		return int(v/1000);
	}
	else return 0;
}
//чтение байта с К линии
int iso_read_byte(){
	int readData;
	boolean success = true;
	byte t=0;
	while(t < READ_ATTEMPTS  && (readData=OBDSerial.read())==-1) {
		delay(1);
		t++;
	} 	
	if (t >= READ_ATTEMPTS) {
		success = false;
		return -1;
	} 
	if (success){
		return readData;
	}
}

void Serial_tx_off() { 
   UCSR0B &= ~(_BV(TXEN0));   
   delay(20);                    //allow time for buffers to flush 
} 
void Serial_rx_off() { 
	UCSR0B &= ~(_BV(RXEN0));   
} 

/*
//вывод знака U
void writeU(){
    tx[0][6]=tx[0][6]|0x01;
    tx[1][5]=tx[1][5]|0x80; tx[1][6]=tx[1][6]|0x04;
    tx[2][5]=tx[2][5]|0x80; tx[2][6]=tx[2][6]|0x04;
}
//вывод слова TEMP 
void writeTEMP(){
	tx[0][5]=tx[0][5]|0x01; tx[0][10]=tx[0][10]|0x08; 
	tx[1][4]=tx[1][4]|0x90; tx[1][5]=tx[1][5]|0xCA; tx[1][6]=tx[1][6]|0x02; tx[1][10]=tx[1][10]|0x10;
	tx[2][4]=tx[2][4]|0x80; tx[2][5]=tx[2][5]|0xE9; tx[2][6]=tx[2][6]|0x05; tx[2][10]=tx[2][10]|0x20;
	tx[3][4]=tx[3][4]|0x60; tx[3][5]=tx[3][5]|0x14; tx[3][6]=tx[3][6]|0x04; tx[3][10]=tx[3][10]|0x40;
}*/
//шкала нагрузки
void LoadBar(int load){
  tx[3][8]|=0x80;
	if (load > 0) tx[3][8]|=0x20;
        if (load > 13) tx[0][8]|=0x10;//<8
	if (load > 27) tx[2][8]|=0x80;//<16
	if (load > 38) tx[1][8]|=0x80;//<24
	if (load > 51) tx[0][8]|=0x80;//<32
	if (load > 64) tx[3][9]|=0x01;
	if (load > 77) tx[3][8]|=0x80;
	if (load > 90) tx[0][9]|=0x01;
	if (load > 102) tx[1][9]|=0x01;
	if (load > 115) tx[2][9]|=0x01;
	if (load > 128) tx[2][9]|=0x02;
	if (load > 141) tx[1][9]|=0x02;
	if (load > 153) tx[0][9]|=0x02;
}
//номер режима
void ModeNumb(byte mode){
	tx[0][3]=tx[0][3]|0x80;
	switch(mode){
		case 0:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x03;
			tx[2][4]=tx[2][4]|0x02;
			tx[3][4]=tx[3][4]|0x03;
		}
		break;
		case 1:{
			tx[1][4]=tx[1][4]|0x02;
			tx[2][4]=tx[2][4]|0x02;
		}
		break;
		case 2:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x01;
			tx[2][4]=tx[2][4]|0x03;
			tx[3][4]=tx[3][4]|0x02;
		}
		break;
		case 3:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x02;
			tx[2][4]=tx[2][4]|0x03;
			tx[3][4]=tx[3][4]|0x02;
		}
		break;
		case 4:{
			tx[1][4]=tx[1][4]|0x02;
			tx[2][4]=tx[2][4]|0x03;
			tx[3][4]=tx[3][4]|0x01;
		}
		break;
		case 5:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x02;
			tx[2][4]=tx[2][4]|0x01;
			tx[3][4]=tx[3][4]|0x03;
		}
		break;
		case 6:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x03;
			tx[2][4]=tx[2][4]|0x01;
			tx[3][4]=tx[3][4]|0x03;
		}
		break;
		case 7:{
			tx[1][4]=tx[1][4]|0x02;
			tx[2][4]=tx[2][4]|0x02;
			tx[3][4]=tx[3][4]|0x02;
		}
		break;
		case 8:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x03;
			tx[2][4]=tx[2][4]|0x03;
			tx[3][4]=tx[3][4]|0x03;
		}
		break;
		case 9:{
			tx[0][4]=tx[0][4]|0x01;
			tx[1][4]=tx[1][4]|0x02;
			tx[2][4]=tx[2][4]|0x03;
			tx[3][4]=tx[3][4]|0x03;
		}
		break;		
	}
}

int16_t TempRead(){ //returns the temperature from one DS18B20 in DEG Celsius   
	if ( OneWire::crc8( addr, 7) != addr[7]) {
//		OBDSerial.println("CRC is not valid!");
		return -1000;
	}
	if ( addr[0] != 0x10 && addr[0] != 0x28) {
//   	OBDSerial.print("Device is not recognized");
		return -1000;
	}
	ds.reset();
	ds.select(addr);
	ds.write(0x44); // start conversion, with parasite power on at the end
// 	delay(750);
	ds.reset();
	ds.select(addr);  
	ds.write(0xBE); // Read Scratchpad
	for (int i = 0; i < 9; i++) { // we need 9 bytes
		data[i] = ds.read();
	}
	int16_t tempRead = ((data[1] << 8) | data[0]); 
	return tempRead;
}
//формирование символов для вывода на экран
void writeSym(boolean SignBit, boolean DotBit, boolean RoundBit,  uint16_t x){
	int16_t rank1, rank2, rank3, rank4, rng;
	rng = x % 10;
	if ((rng < 5)||(!RoundBit)){
		x = x / 10;
	}
	else x = x/10 + 1;     
	rank1 = x / 1000;  
	rank2 = (x / 100) % 10;
	rank3 = (x /10) % 10;
    rank4 = x % 10;	
	if ((!SignBit)&&(rank2!=0)){  //рисуем минус при двузначном значении
		if (rank2 == 1) tx[2][7]|=0x01;
		else tx[1][6]|=0x20;
	}	
	if ((!SignBit)&&(rank2==0)) tx[1][7]|=0x02; //при однозначном
	if(DotBit) tx[3][7]|=0x80;
//рисуем 1 позицию
 switch (rank1) {
	case 1:{
		tx[1][6]|=0x40; tx[2][6]|=0x40;
	}
	break;
 	case 2: {   
		tx[0][6]|=0x10; tx[1][6]|=0x28; tx[2][6]|=0x50; tx[3][6]|=0x40;
	}
    break;
	case 3: {   
		tx[0][6]|=0x10; tx[1][6]|=0x60; tx[2][6]|=0x50; tx[3][6]|=0x40;
	}
    break;
	case 4: {   
		tx[1][6]|=0x60; tx[2][6]|=0x58;
	}
    break;
	case 5: {   
		tx[0][6]|=0x10; tx[1][6]|=0x60; tx[2][6]|=0x18; tx[3][6]|=0x40;
	}
    break;
	case 6: {   
		tx[0][6]|=0x10; tx[1][6]|=0x68; tx[2][6]|=0x18; tx[3][6]|=0x40;
	}
    break;
	case 7: {   
		tx[1][6]|=0x40; tx[2][6]|=0x40; tx[3][6]|=0x40;
	}
    break;
	case 8: {   
		tx[0][6]|=0x10; tx[1][6]|=0x68; tx[2][6]|=0x58; tx[3][6]|=0x40;
	}
    break;
	case 9: {   
		tx[0][6]|=0x10; tx[1][6]|=0x60; tx[2][6]|=0x58; tx[3][6]|=0x40;
	}
    break;
 }  
//рисуем 2 позицию
 switch (rank2) {
  case 0:{
	if (rank1 > 0){// рисуем 0 если 1 разряд не равен 0
		tx[1][6]|=0x80; tx[2][6]|=0x80; tx[0][7]|=0x01; tx[1][7]|=0x04; tx[2][7]|=0x04; tx[3][7]|=0x04;
	}
  }
  break;
  case 1: {   
    tx[1][7]|=0x04; tx[2][7]|=0x04;}
    break;
  case 2: {   
    tx[0][7]|=0x01; tx[1][7]|=0x02; tx[2][7]|=0x05; tx[3][7]|=0x04; tx[1][6]|=0x80;}
    break;
  case 3: {   
    tx[0][7]|=0x01; tx[1][7]|=0x06; tx[2][7]|=0x05; tx[3][7]|=0x04;}
    break;
  case 4: {   
    tx[2][6]|=0x80; tx[1][7]|=0x06; tx[2][7]|=0x05; }
    break;
  case 5: {   
    tx[2][6]|=0x80; tx[0][7]|=0x01; tx[1][7]|=0x06; tx[2][7]|=0x01; tx[3][7]|=0x04;}
    break;
  case 6: {   
    tx[1][6]|=0x80; tx[2][6]|=0x80; tx[0][7]|=0x01; tx[1][7]|=0x06; tx[2][7]|=0x01; tx[3][7]|=0x04;}
    break;
  case 7: {   
    tx[1][7]|=0x04; tx[2][7]|=0x04; tx[3][7]|=0x04;}
    break;
  case 8: {   
    tx[1][6]|=0x80; tx[2][6]|=0x80; tx[0][7]|=0x01; tx[1][7]|=0x06; tx[2][7]|=0x05; tx[3][7]|=0x04;}
    break;
  case 9: {   
    tx[2][6]|=0x80; tx[0][7]|=0x01; tx[1][7]|=0x06; tx[2][7]|=0x05; tx[3][7]|=0x04;}
    break;

  } 
//рисуем 3 позицию
 switch (rank3) {
	case 0: {   
		if ((rank2 > 0)||(rank1>0)||(DotBit)){
			tx[0][7]|=0x10; tx[1][7]|=0x48; tx[2][7]|=0x48; tx[3][7]|=0x40; 
		}	
	}
    break;
	case 1: {   
		tx[1][7]|=0x40; tx[2][7]|=0x40; 
	}
    break;
	case 2: {   
		tx[0][7]|=0x10; tx[1][7]|=0x28; tx[2][7]|=0x50; tx[3][7]|=0x40;
	}
    break;
	case 3: {   
		tx[0][7]|=0x10; tx[1][7]|=0x60; tx[2][7]|=0x50; tx[3][7]|=0x40;
	}
    break;
	case 4: {   
		tx[1][7]|=0x60; tx[2][7]|=0x58;
	}
    break;
	case 5: {   
		tx[0][7]|=0x10; tx[1][7]|=0x60; tx[2][7]|=0x18; tx[3][7]|=0x40;
	}
    break;
	case 6: {   
		tx[0][7]|=0x10; tx[1][7]|=0x68; tx[2][7]|=0x18; tx[3][7]|=0x40;
	}
    break;
	case 7: {   
		tx[1][7]|=0x40; tx[2][7]|=0x40; tx[3][7]|=0x40;
	}
    break;
	case 8: {   
		tx[0][7]|=0x10; tx[1][7]|=0x68; tx[2][7]|=0x58; tx[3][7]|=0x40;
	}
    break;
	case 9: {   
		tx[0][7]|=0x10; tx[1][7]|=0x60; tx[2][7]|=0x58; tx[3][7]|=0x40;
	}
    break;
 } 
//рисуем 4 позицию
 switch (rank4) {
   case 0: {   
    tx[1][7]|=0x80; tx[2][7]|=0x80; tx[0][8]|=0x01; tx[1][8]|=0x04; tx[2][8]|=0x04; tx[3][8]|=0x04;}
    break;
   case 1: {   
    tx[1][8]|=0x04; tx[2][8]|=0x04; }
    break;
  case 2: {   
    tx[1][7]|=0x80; tx[0][8]|=0x01; tx[1][8]|=0x02; tx[2][8]|=0x05; tx[3][8]|=0x04;}
    break;
  case 3: {   
    tx[0][8]|=0x01; tx[1][8]|=0x06; tx[2][8]|=0x05; tx[3][8]|=0x04;}
    break;
  case 4: {   
    tx[2][7]|=0x80; tx[1][8]|=0x06; tx[2][8]|=0x05; }
    break;
  case 5: {   
    tx[2][7]|=0x80; tx[0][8]|=0x01; tx[1][8]|=0x06; tx[2][8]|=0x01; tx[3][8]|=0x04;}
    break;
  case 6: {   
    tx[1][7]|=0x80; tx[2][7]|=0x80; tx[0][8]|=0x01; tx[1][8]|=0x06; tx[2][8]|=0x01; tx[3][8]|=0x04;}
    break;
  case 7: {   
    tx[1][8]|=0x04; tx[2][8]|=0x04; tx[3][8]|=0x04;}
    break;
  case 8: {   
    tx[1][7]|=0x80; tx[2][7]|=0x80; tx[0][8]|=0x01; tx[1][8]|=0x06; tx[2][8]|=0x05; tx[3][8]|=0x04;}
    break;
  case 9: {   
    tx[2][7]|=0x80; tx[0][8]|=0x01; tx[1][8]|=0x06; tx[2][8]|=0x05; tx[3][8]|=0x04;}
    break;
 }
} 
//вывод в верхнюю строку
void writeSym2(boolean DotBit, uint16_t x){
	int16_t rank1, rank2, rank3, rank4, rng;
	rng = x % 10;
        if (rng < 5){
            x = x / 10;
        }
        else x = x/10 + 1;    
	rank1 = x / 1000;  
	rank2 = (x / 100) % 10;
	rank3 = (x /10) % 10;
    rank4 = x % 10;	
	if(DotBit) tx[3][2]|=0x20;
	if (rank1==1) tx[2][3]=0x01;
	switch (rank2) {
		case 0: {   
			if (rank1 > 0){// рисуем 0 если 1 разряд не равен 0
				tx[0][2]|=0xC0; tx[1][2]|=0x40; tx[2][2]|=0xC0; tx[3][2]|=0x40;
			}	
		}
		break;
		case 1: {   
			tx[1][2]|=0x40; tx[2][2]|=0x40;
		}
		break;
		case 2: {   
			tx[0][2]|=0x40; tx[1][2]|=0xC0; tx[2][2]|=0x80; tx[3][2]|=0x40;
		}
		break;
		case 3: {   
			tx[0][2]|=0x40; tx[1][2]|=0xC0; tx[2][2]|=0x40; tx[3][2]|=0x40;
		}
		break;
		case 4: {   
			tx[0][2]|=0x80; tx[1][2]|=0xC0; tx[2][2]|=0x40; 
		}
		break;
		case 5: {   
			tx[0][2]|=0xC0; tx[1][2]|=0x80; tx[2][2]|=0x40; tx[3][2]|=0x40;
		}
		break;
		case 6: {   
			tx[0][2]|=0xC0; tx[1][2]|=0x80; tx[2][2]|=0xC0; tx[3][2]|=0x40;
		}
		break;
		case 7: {   
			tx[0][2]|=0x40; tx[1][2]|=0x40; tx[2][2]|=0x40;
		}
		break;
		case 8: {   
			tx[0][2]|=0xC0; tx[1][2]|=0xC0; tx[2][2]|=0xC0; tx[3][2]|=0x40;
		}
		break;
		case 9: {   
			tx[0][2]|=0xC0; tx[1][2]|=0xC0; tx[2][2]|=0x40; tx[3][2]|=0x40;
		}
		break;		
	}	
	switch (rank3) {
		case 0: {   
			if ((rank2 > 0)||(rank1>0)||(DotBit)){
				tx[0][2]|=0x30; tx[1][2]|=0x10; tx[2][2]|=0x30; tx[3][2]|=0x10;
			}	
		}
		break;
		case 1: {   
			tx[1][2]|=0x10; tx[2][2]|=0x10;
		}
		break;
		case 2: {   
			tx[0][2]|=0x10; tx[1][2]|=0x30; tx[2][2]|=0x20; tx[3][2]|=0x10;
		}
		break;
		case 3: {   
			tx[0][2]|=0x10; tx[1][2]|=0x30; tx[2][2]|=0x10; tx[3][2]|=0x10;
		}
		break;
		case 4: {   
			tx[0][2]|=0x20; tx[1][2]|=0x30; tx[2][2]|=0x10; 
		}
		break;
		case 5: {   
			tx[0][2]|=0x30; tx[1][2]|=0x20; tx[2][2]|=0x10; tx[3][2]|=0x10;
		}
		break;
		case 6: {   
			tx[0][2]|=0x30; tx[1][2]|=0x20; tx[2][2]|=0x30; tx[3][2]|=0x10;
		}
		break;
		case 7: {   
			tx[0][2]|=0x10; tx[1][2]|=0x10; tx[2][2]|=0x10;
		}
		break;
		case 8: {   
			tx[0][2]|=0x30; tx[1][2]|=0x30; tx[2][2]|=0x30; tx[3][2]|=0x10;
		}
		break;
		case 9: {   
			tx[0][2]|=0x30; tx[1][2]|=0x30; tx[2][2]|=0x10; tx[3][2]|=0x10;
		}
		break;
		
	}
	switch (rank4) {
		case 0: {   
			tx[0][2]|=0x0C; tx[1][2]|=0x04; tx[2][2]|=0x0C; tx[3][2]|=0x04;
		}
		break;
		case 1: {   
			tx[1][2]|=0x04; tx[2][2]|=0x04;
		}
		break;
		case 2: {   
			tx[0][2]|=0x04; tx[1][2]|=0x0C; tx[2][2]|=0x08; tx[3][2]|=0x04;
		}
		break;
		case 3: {   
			tx[0][2]|=0x04; tx[1][2]|=0x0C; tx[2][2]|=0x04; tx[3][2]|=0x04;
		}
		break;
		case 4: {   
			tx[0][2]|=0x08; tx[1][2]|=0x0C; tx[2][2]|=0x04; 
		}
		break;
		case 5: {   
			tx[0][2]|=0x0C; tx[1][2]|=0x08; tx[2][2]|=0x04; tx[3][2]|=0x04;
		}
		break;
		case 6: {   
			tx[0][2]|=0x0C; tx[1][2]|=0x08; tx[2][2]|=0x0C; tx[3][2]|=0x04;
		}
		break;
		case 7: {   
			tx[0][2]|=0x04; tx[1][2]|=0x04; tx[2][2]|=0x04;
		}
		break;
		case 8: {   
			tx[0][2]|=0x0C; tx[1][2]|=0x0C; tx[2][2]|=0x0C; tx[3][2]|=0x04;
		}
		break;
		case 9: {   
			tx[0][2]|=0x0C; tx[1][2]|=0x0C; tx[2][2]|=0x04; tx[3][2]|=0x04;
		}
		break;	
	}
}
//запись строки на дисплей
void writeSTR (char str[], byte PozNumb) {
	int i,z0,z1,x0,x1,sum=0;
	static int sum_prev,st,end,k;
/*преобразование int to char
	String str=String(ISO_init_flag, HEX);
	char charVar[sizeof(str)];
	str.toCharArray(charVar, sizeof(charVar));		
	writeSTR(charVar,0);	
*/	
	while (str[sum]!=0){
		sum++;
	}
	if(sum_prev!=sum){
		st=0;
		end=-1;
		k=0;
		sum_prev=sum;
	}	
	if (sum>(8-PozNumb)) {
		if (time_cam-Scroll_time>250){
			if ((end+1)<(8-PozNumb)){
				end++;
			} 
			else {
				if (end==(sum-1)) k++;
				if (st<(sum-1)) st++;
				if (end<(sum-1)) end++;
			}
			Scroll_time=time_cam;
		}	
	}
	else {
		st=0;
		end=sum-1;
	}
	PozNumb=k+(end-st)+PozNumb;
	for (i=st; i<=end; i++){ 
		z0= -(PozNumb/2);//при смене байта при переходе с 1 на 2 позицию
		z1= -(PozNumb/2+PozNumb%2);//при смене байта при переходе с 0 на 1 позицию
		x0= -4*(PozNumb%2-1);//на четной позиции добавлять к сдвигу +4
		x1= 4*(PozNumb%2);//на нечетной позиции добавлять к сдвигу +4
		switch (str[i]) {
			case '0': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[0][8+z1]|=1<<(0+x1);//- низ
				tx[2][8+z1]|=1<<(1+x1);
				tx[0][7+z0]|=1<<(3+x0);
			}
			break;
			case '1': {
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
			}
			break;
			case '2': {
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[0][8+z1]|=1<<(0+x1);//- низ
			}
			break;
			case '3': {
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
				tx[0][8+z1]|=1<<(0+x1);//- низ
			}
			break;
			case '4': {
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
			}
			break;
			case '5': {
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх 
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[0][8+z1]|=1<<(0+x1);//- низ
			}
			break;
			case '6': {
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх 
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[0][8+z1]|=1<<(0+x1);//- низ
			}
			break;
			case '7': {
				tx[3][8+z1]|=1<<(2+x1);//- верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
			}
			break;
			case '8': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[0][8+z1]|=1<<(0+x1);//- низ
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
			}
			break;						
			case '9': {
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[0][8+z1]|=1<<(0+x1);//- низ
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
			}
			break;									
			case 'A': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
				tx[1][8+z1]|=1<<(1+x1);//- ср. пр.
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[3][8+z1]|=1<<(2+x1);//- верх			
			}
			break;						
			case 'B': {
				tx[1][8+z1]|=1<<(0+x1);
				tx[3][8+z1]|=1<<(1+x1);
				tx[0][8+z1]|=1<<(0+x1);				
				tx[1][8+z1]|=1<<(1+x1);
				tx[3][8+z1]|=1<<(2+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(2+x1);
			}
			break;					
			case 'C': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[0][8+z1]|=1<<(0+x1);
				tx[3][8+z1]|=1<<(2+x1);//- верх			
			}
			break;						
			case 'D': {
				tx[1][8+z1]|=1<<(0+x1);
				tx[3][8+z1]|=1<<(1+x1);
				tx[0][8+z1]|=1<<(0+x1);				
				tx[3][8+z1]|=1<<(2+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(2+x1);
			}
			break;					
			
			case 'E': {
				tx[1][7+z0]|=1<<(3+x0);//0x80 
				tx[2][7+z0]|=1<<(3+x0);//0x80
				tx[0][8+z1]|=1<<(0+x1);				
				tx[1][8+z1]|=1<<(1+x1);//0x02
				tx[2][8+z1]|=1<<(0+x1);//0x01
				tx[3][8+z1]|=1<<(2+x1);//0x04
			}
			break;					
			case 'F': {
				tx[1][7+z0]|=1<<(3+x0);//0x80 
				tx[2][7+z0]|=1<<(3+x0);//0x80 
				tx[1][8+z1]|=1<<(1+x1);//0x02
				tx[2][8+z1]|=1<<(0+x1);//0x01
				tx[3][8+z1]|=1<<(2+x1);//0x04
			}
			break;	
			case 'H': {   
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[1][8+z1]|=1<<(2+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(1+x1);//0x02
				tx[2][8+z1]|=1<<(0+x1);//0x01
			}
			break;				
			case 'I': {
				tx[1][8+z1]|=1<<(0+x1);
				tx[3][8+z1]|=1<<(1+x1);
			}
			break;	
			case 'K': {
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх 
				tx[2][8+z1]|=1<<(0+x1);//- ср. лев.
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][8+z1]|=1<<(1+x1);
				tx[0][8+z1]|=1<<(1+x1);
			}
			break;
			
			case 'L': {   
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[0][8+z1]|=1<<(0+x1);//0x01
			}
			break;
			case 'M': {   
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[3][8+z1]|=1<<(0+x1);
				tx[2][8+z1]|=1<<(1+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(2+x1);
			}
			break;
			case 'N': {   
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[3][8+z1]|=1<<(0+x1);
				tx[0][8+z1]|=1<<(1+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(2+x1);
			}
			break;			
			case 'O': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[3][8+z1]|=1<<(2+x1);//- верх	
				tx[0][8+z1]|=1<<(0+x1);//- низ
			}
			break;									
			case 'P': {   
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[3][8+z1]|=1<<(2+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(1+x1);//0x02
				tx[2][8+z1]|=1<<(0+x1);//0x01
			}
			break;	
			case 'R': {   
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[3][8+z1]|=1<<(2+x1);
				tx[2][8+z1]|=1<<(2+x1);
				tx[1][8+z1]|=1<<(1+x1);
				tx[2][8+z1]|=1<<(0+x1);
				tx[0][8+z1]|=1<<(1+x1);
			}
			break;
			case 'S': {
				tx[2][7+z0]|=1<<(3+x0);//0x80
				tx[0][8+z1]|=1<<(0+x1);				
				tx[1][8+z1]|=1<<(1+x1);//0x02
				tx[2][8+z1]|=1<<(0+x1);//0x01
				tx[3][8+z1]|=1<<(2+x1);//0x04
				tx[1][8+z1]|=1<<(2+x1);
			}
			break;								
			case 'T': {
				tx[1][8+z1]|=1<<(0+x1);
				tx[3][8+z1]|=1<<(1+x1);
				tx[3][8+z1]|=1<<(2+x1);
			}
			break;	
			case 'W': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[0][8+z1]|=1<<(1+x1);//\ пр.ниж.
				tx[0][7+z0]|=1<<(3+x0);/// лев.ниж.
			}
			break;												
			case 'U': {
				tx[1][7+z0]|=1<<(3+x0);//| лев. ниж.
				tx[2][7+z0]|=1<<(3+x0);//| лев. верх
				tx[2][8+z1]|=1<<(2+x1);//| пр. верх 
				tx[1][8+z1]|=1<<(2+x1);//| пр. ниж.
				tx[0][8+z1]|=1<<(0+x1);//- низ
			}
			break;									
			case 'V': {
				tx[1][7+z0]|=1<<(3+x0);
				tx[2][7+z0]|=1<<(3+x0);
				tx[0][7+z0]|=1<<(3+x0);
				tx[2][8+z1]|=1<<(1+x1);
			}
			break;
			case 'Y': {
				tx[1][8+z1]|=1<<(0+x1);
				tx[2][8+z1]|=1<<(1+x1);
				tx[3][8+z1]|=1<<(0+x1);
			}
			break;							
		}		
		PozNumb--;
	}
	if ((st==(sum-1))&&(end==(sum-1))) {
		st=0;
		end=-1;
		k=0;
	}	
}      
