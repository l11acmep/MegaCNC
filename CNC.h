// экстренные пины
#define PIN_ESTOP    20
#define PIN_ERRSPINDEL  21

// выходные пины
#define PIN_XSTEP 22
#define PIN_XDIR  23
#define PIN_YSTEP 22
#define PIN_YDIR  23
#define PIN_ZSTEP 22
#define PIN_ZDIR  23
//#define PIN_ASTEP 22  // зарезервировано для оси А
//#define PIN_ADIR  23  // зарезервировано для оси А


// Пины внешнего джойстика
#define PIN_XMINUS 30
#define PIN_XPLUS  31
#define PIN_YMINUS 32
#define PIN_YPLUS  33
#define PIN_ZMINUS 34
#define PIN_ZPLUS  35
#define PIN_START  36
#define PIN_STOP   37

// пины датчиков концевиков хода для определения предельных границ
#define PIN_XHOME  39
#define PIN_YHOME  40
#define PIN_ZHOME  41
// входной пин для определения достижения шпинделем рабочей частоты вращения
#define PIN_SPINDEL_HR 38  


// обозначения клавиш
#define NONE   0 
#define SELECT 1
#define LEFT   2
#define DOWN   3
#define UP     4
#define RIGHT  5

// маски пинов для битовых операций
#define SET_MASK_0  B00000001
#define CLR_MASK_0  B11111110
#define SET_MASK_1  B00000010
#define CLR_MASK_1  B11111101
#define SET_MASK_2  B00000100
#define CLR_MASK_2  B11111011
#define SET_MASK_3  B00001000
#define CLR_MASK_3  B11110111
#define SET_MASK_4  B00010000
#define CLR_MASK_4  B11101111
#define SET_MASK_5  B00100000
#define CLR_MASK_5  B11011111
#define SET_MASK_6  B01000000
#define CLR_MASK_6  B10111111
#define SET_MASK_7  B10000000
#define CLR_MASK_7  B01111111


// сокращения для управления портами напрямую
#define do_step_X       PORTA|=SET_MASK_0
#define set_dirX_right  PORTA|=SET_MASK_1
#define set_dirX_left   PORTA&=CLR_MASK_1
#define do_stepY        PORTA|=SET_MASK_2
#define set_dirY_right  PORTA|=SET_MASK_3
#define set_dirY_left   PORTA&=CLR_MASK_3
#define do_stepZ        PORTA|=SET_MASK_4
#define set_dirZ_right  PORTA|=SET_MASK_5
#define set_dirZ_left   PORTA&=CLR_MASK_5
#define clear_stepsXYZ  PORTA&=B10101010; // для сьроса всех шагов (спад импульсов step) не изменяя направления  
//#define StepA  PORTA |= SET_MASK_6    // зарезервировано для оси А
//#define DirA+  PORTA |= SET_MASK_7    // зарезервировано для оси А
//#define DirA-  PORTA &= CLR_MASK_7    // зарезервировано для оси А
//#define SpindelON     
//#define SpindelOFF

#define move_ON     TIMSK1|=B00000010;   // включение прерывания OCIE1A для движения моторов
#define move_OFF    TIMSK1&=B11111101;   // выключение прерывания OCIE1A для движения моторов


char* menuMessages[] = {"XYZ from EEPROM", 		// загрузить значения координат из EEPROM
						"Set HOME manual",		// установить координаты HOME вручную 
						"Set HOME auto"			// установить координаты HOME автоматически на минимальной скорости 
						};

char* errorMessages[] = {"OK",					// 0 - всё нормально. выполнение программы и сценария без ошибок
                         "SD card FAILED..", 	// 1 - ошибка SD карты
                         "file ERROR", 			  // 2 - ошибка открытия файла
                         "NO INITIALIZATON", 	// 3 - ошибка инициализации
                         "EMERGENCY STOP",		// 4 - нажата кнопка EStop
                         "SPINDEL ERROR",		  // 5 - ошибка шпинделя
                         "X out of LIMIT",		// 6 - выход за границу по X
                         "Y out of LIMIT",		// 7 - выход за границу по Y
                         "Z out of LIMIT",		// 8 - выход за границу по Z
                         };

// смайлик						 
byte smiley[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};

// стрелка
byte arrow[8] = {
  0b00000,
  0b01000,
  0b01100,
  0b01110,
  0b01100,
  0b01000,
  0b00000,
  0b00000
};
