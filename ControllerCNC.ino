/*
  ControllerCNC
  Author: Sergey Kozhevnikov
  Project start: 19.03.2017

  Прерывания:
    D14 - кнопка EStop (экстренный останов) - (int 10)  
    D15 - ошибка шпинделя - (int 9) 

  Входы:
    D30 - кнопка ручное управление X--
    D31 - кнопка ручное управление X++
    D32 - кнопка ручное управление Y--
    D33 - кнопка ручное управление Y++
    D34 - кнопка ручное управление Z--
    D35 - кнопка ручное управление Z++
    D36 - кнопка ручное START
    D37 - кнопка ручное STOP
    D38 - сигнал о выходе шпинделя на рабочий режим  (High Rotation)
    D39 - концевик по оси X
    D40 - концевик по оси Y
    D41 - концевик по оси Z    

  Выходы:
    D50(MISO), D51(MOSI), D52(SCK), D53(SS) - SD карта
    D22 - ось X шаг
    D23 - ось X направление
    D24 - ось Y шаг
    D25 - ось Y направление
    D26 - ось Z шаг
    D27 - ось Z направление
    D28, D29 - резерв для оси A
    
   
  Тайминги: 
    чтение полных 4 кБ с проверкой: 158 мс
    открытие файла: 1 мс
    закрытие файла: 2 мс
    очистка экрана: 2 мс
    вывод полной строки: 6 мс

*/

#include "CNC.h"
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>


// Инициализация пинов LCD дисплея
LiquidCrystal lcd(8, 9, 4, 5, 6, 7 );

// Глобальные переменные
bool bInitialized = false;    // Флаг успешной инициализации
File myFile;                  // Дескриптор для работы с файлом
unsigned long fileSize;       // Размер открытого файла для вычисления процентов выполнения
unsigned long startTime;      // Время начала работы

const uint16_t fileBufferSize = 4096;  // размер буфера для считывания файла с карты
char fileBuffer[fileBufferSize];
uint16_t bufferPointer = 0;
uint16_t buffersCounter = 0;
uint32_t uiLinesCounter = 0;

// переменные для меню
int menuLevel = 0;          // Уровень меню:  0 - главное меню
int menuItem = 0;           // Текущий (выбранный) элемент меню.
bool bPressed = false;      // Нажата кнопка или нет? Используется в key_pressed() при опросе нажатия клавиш.
byte menuCode = 0;          // Используется для идентификации состояния меню, в котором находится программа

// переменные для прерывания
volatile byte errorCode = 0;                    // переменная общей ошибки. Если не 0, то сообщение ошибки отображается на экране
volatile bool bOdd = true;                      
volatile bool bMoveDone = true;                 // флаг прибытия шпинделя в точку заданную текущей строкой сценария. Флаг изменяют ISP_doMove и next_step
           
// параметры двигателей и механики 
#define   stepsPerRevolution    200               // количество шагов на один оборот
#define   driverMicrostepping   2                 // множитель драйвера двигателей
#define   leadScrewPitch        5                 // шаг приводного винта, об/мм
// количество шагов на 1 мм
#define   stepsPerMM            (stepsPerRevolution*driverMicrostepping)/leadScrewPitch                            

// максимальные скорости по осям, мм/сек
#define   maxVelXYZ   44                            // общая максимальная скорость по осям X, Y, Z, мм/сек 
//#define   maxVelX    48                           // максимальная скорость по оси X, мм/сек
//#define   maxVelY    48                           // максимальная скорость по оси Y, мм/сек
//#define   maxVelZ    44                           // максимальная скорость по оси Z, мм/сек
// общая минимальная скорость по осям, мм/сек
// (для поиска нулевых координат)
#define   homeSearchVelXYZ   10                     // минимальная скорость по осям X, Y, Z, мм/сек

// максимальные ускорения по осям, мм/сек^2
//#define   maxAccX   150                           // максимальное ускорение по оси X, мм/сек^2
//#define   maxAccY   300                           // максимальное ускорение по оси Y, мм/сек^2
//#define   maxAccZ   220                           // максимальное ускорение по оси Z, мм/сек^2
// максимальные размеры стола
#define   tableLengthXmm    670                       // максимальный ход по оси X, мм
#define   tableWidthYmm     310                       // максимальный ход по оси Y, мм
#define   tableHeightZmm    150                       // максимальная ход по оси Z, мм
// предельные частоты шагов по осям, Гц
#define   maxFreqXYZ      stepsPerMM*maxVelXYZ    // 3520 Гц   (3520 ш/сек)
#define   minFreqXYZ      stepsPerMM*minVelXYZ    // 160 ш/сек    (160 ш/сек)
//#define   maxFreqX   stepsPerMM*maxVelX
//#define   maxFreqY   stepsPerMM*maxVelY
//#define   maxFreqZ   stepsPerMM*maxVelZ
// параметры таймера прерываний
#define   CPU_clock       16000000
#define   prescaler       8
#define   ISR_freq        1000000                   // (CPU_clock/prescaler)/2; прерывание вызывается в два раза чаще, чем выдаются шаги на моторы
#define   OCR1A_vmax      284                       // формула: ISR_freq/maxVelXYZ
#define   OCR1A_vmin      65535
#define   OCR1A_vsearch   1250                      // формула: ISR_freq/homeSearchVelXYZ
static uint16_t OCR1A_feed  = OCR1A_vsearch;        // установка скорости для перемещения при G1

float nextX = 0, nextY = 0, nextZ = 0;              // следующая точка сценария (координаты-цель текущего движения)

volatile uint16_t curX = 0, curY = 0, curZ = 0;       // текущие координаты шпинделя в шагах от нулевого положения в координатах станка. Изменяются прерыванием таймера.
static uint16_t taskX = 0, taskY = 0, taskZ = 0;      // следующая точка сценария (координаты-цель текущего движения). Пока сюда не придём bMoveDone = false
static uint16_t startX = 0, startY = 0, startZ = 0;   // координаты нулей заготовки 
  
float spindleSpeed = 0.0;                             // скорость шпинделя, об/мин
float feedSpeed = 0.0;                                // скорость подачи, мм/мин
byte curTool = 1;                                     // текущий номер инструмента
static uint16_t ocr  = 32000;                         // значение регистра сравнения для установки частоты  

void setup() {
  Serial.begin(9600);
  // Установка прерываний и настройка пинов
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), ISR_Estop, FALLING);
  pinMode(PIN_ERRSPINDEL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ERRSPINDEL), ISR_Spindel, FALLING);
  // Настройка входных пинов кнопок выносного пульта
  pinMode(PIN_XMINUS, INPUT_PULLUP);
  pinMode(PIN_XPLUS, INPUT_PULLUP);
  pinMode(PIN_YMINUS, INPUT_PULLUP);
  pinMode(PIN_YPLUS, INPUT_PULLUP);
  pinMode(PIN_ZMINUS, INPUT_PULLUP);
  pinMode(PIN_ZPLUS, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  pinMode(PIN_XHOME, INPUT_PULLUP);
  pinMode(PIN_YHOME, INPUT_PULLUP);
  pinMode(PIN_ZHOME, INPUT_PULLUP);
  pinMode(PIN_SPINDEL_HR, INPUT_PULLUP);
  // Настройка выходных пинов
  pinMode(PIN_XSTEP, OUTPUT);
  pinMode(PIN_XDIR, OUTPUT);
  pinMode(PIN_YSTEP, OUTPUT);
  pinMode(PIN_YDIR, OUTPUT);
  pinMode(PIN_ZSTEP, OUTPUT);
  pinMode(PIN_ZDIR, OUTPUT);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Create new characters
  lcd.createChar(0, smiley);
  lcd.createChar(1, arrow);
  // Print startup messages to the LCD.
  lcd.clear();
  lcd.print(" www.RADOMAG.ru ");
  lcd.write(byte(0));         // print smiley 
  lcd.setCursor(0, 1);
  lcd.print("S. Kozhevnikov");
  delay(1000);
  lcd.clear();
  lcd.print("-Controller CNC-");
  lcd.setCursor(0, 1);
  lcd.print("Version 0.9.0417");  // версия мажор.минор.месяцгод(в формате ММГГ) 
  delay(1000);
  // SD initialization
  lcd.clear();
  lcd.print("Initializing SD:");
  lcd.setCursor(0, 1);
  if (!SD.begin()) {
    errorCode = 1;
    return;
  } else {
    lcd.print("SD card PRESENT");
  }
  delay(500);
  myFile = SD.open("/2.tap");   // FILE_READ
  if(!myFile) {
    errorCode = 2;
    return;  
  }
  delay(500);
  
  // заполняем буфер данными из файла
  if (!fill_buffer()) {
    lcd.clear();
    lcd.print("Error file 2.tap");
    return;
  }
   
  lcd.clear();
  lcd.print("-Ready to WORK-");
  startTime = millis();

  // инициализация 16 битного таймера 1
  TCCR1A = 0; 
  OCR1A = OCR1A_vsearch;
  TCCR1B = B00001010;   // сброс при совпадении А и делитель на 8
  TIMSK1 = B00000010;   // включение генерации прерывания по совпадению с OCR1A 

  bInitialized = true;
}

void loop() {
  if(errorCode) show_error();
  if(!bInitialized) { errorCode = 3;   return; }  // бесконечный цикл если не инициализирован



/*
  while(!errorCode || !bMoveDone || next_line()) {
    show_work();
  }
  if(errorCode)
    show_error();
  else
    stop_work();

  show_menu();
*/  
 

 // while(true) {
  /*  if(key_pressed()==UP && d > 10)
      d -= 10;
    if(key_pressed()==DOWN && d < 200)
      d += 10;
    if(key_pressed()==LEFT)
      digitalWrite(PIN_XDIR, HIGH);
    if(key_pressed()==RIGHT)
      digitalWrite(PIN_XDIR, LOW);
    */

    
  //}
  
  
  
  
  /*
  //startTime = millis();
  if(key_pressed() == DOWN) {
    if(next_line()) {
      curX = nextX; 
      curY = nextY;    
      curZ = nextZ;
      show_work(); 
    }
    else {
      stop_work();
      bInitialized = false;
    }
    
  }
*/

/*
  if(!next_line()) {
    stop_work();
  }
  else {
    do_move();  
  }
*/

}

//***********************************************************************
// функция берет из буффера строку, разбирает её (парсит) и записывает в
// переменные nextX, nextY, nextZ значения следующей координаты
// возвращает true если разбор оказался успешным и false если не успешным
// значение false означает конец файла и окончание сценария
// если в строке встречаются управляющие символы G0, G1, M3 и т.д,
// то функция сама выставляет параметры выполнения сценария
// возвращает true если следующая строка успешно обработана из файлового буфера
// внутри функции автоматически обновляется файловый буфер при достижении конца буфера
// при выходе из функции указатель на файловый буффер указывает на первый символ после перевода строки
bool next_line() {
  //Serial.println("next line");
  bool bEndLine = false;
  while(bufferPointer < fileBufferSize || fill_buffer()) {
    // берём символ с текущего указателя на файловый буфер
    char symbol = fileBuffer[bufferPointer];
    //Serial.println(symbol);
    // если символ управляющий, то ставим флаг конца строки и ничего не добавляем во временный буфер
    if(symbol == (char)0x0A || symbol == (char)0x0D ) {     // защита от разрыва буфером файла между символами 0А и 0D
      bEndLine = true;
      //Serial.println("(char)0x0A || (char)0x0D");
      bufferPointer++;
    }
    // если символ не управляющий
    else {
      // и если был конец строки, значит мы убрали все управляющие символы и нужен выход из функции с результатом true
      if(bEndLine) { 
        uiLinesCounter++;
        PORTB ^= B10000000;               // LED мигание на каждую новую строку
        //Serial.println("LINE END");
        return true;
      }
      // если не было конца строки, то проверка символа по алгоритму на соответствие командам 
      else {
        //Serial.println("COMMAND");
        bufferPointer++;  // сразу указываем на позицию после буквы команды
        // располагаем команды по мере убывания частоты появления в тексте. Это снижает количество ложных проверок и увеличивает быстродействие
        // координата оси X 
        if(symbol == 'X' && get_float(nextX)) {           
          // nop  
        }
        // координата оси Y
        else if(symbol == 'Y' && get_float(nextY)) {
          // nop
        }        
        // координата оси Z
        else if(symbol == 'Z' && get_float(nextZ)) {
          // nop
        }        
        else if(symbol == 'G') {
          byte code = get_byte();  //  Serial.print("G "); Serial.println(b);}    // режим перемещения
          if(code == 0) {    // режим G0 перемещение на максимальной скорости
              OCR1A = OCR1A_vmax; // устанавливаем значение регистра-ограничителя счетчика в значение подачи на максимальной скорости
          }
          if(code == 1) {    // режим G1 перемещение на скорости подачи установленной текущей feedSpeed
              TCNT1 = 0;          // сброс счётчика в 0 во избежание ситуации перебегания состояния счетчика уровня TOP
              OCR1A = OCR1A_feed; // устанавливаем значение регистра-ограничителя счетчика в запомненное значение рабочей подачи
          }
        }          
        // скорость подачи 
        else if(symbol == 'F' && get_float(feedSpeed)) {
            uint16_t newOCR1A = 750000/(int)feedSpeed;
            // запоминаем вычисленное значение для будущих установок G1 с проверкой на максимальную и минимальную границы
            if(newOCR1A < OCR1A_vmax)         OCR1A_feed = OCR1A_vmax;   
            else if(newOCR1A > OCR1A_vmin)    OCR1A_feed = OCR1A_vmin;  
            else                              OCR1A_feed = newOCR1A;
            // устанавливаем текущее значение скорости
            OCR1A = OCR1A_feed;
        }   
        // запуск шпинделя или конец программы
        else if(symbol == 'M') {           
          byte b = get_byte();        // Serial.print("M "); Serial.println(b);  
          if(b == 3)
          { 
            //запуск шпинделя
          }                        
          else if(b == 30) 
            return false;         // конец программы. выходим так как если бы кончились данные
        }    
        // смена скорости шпинделя
        else if(symbol == 'S' && get_float(spindleSpeed)) {           
            // устанавливаем новую скорость шпинделя
        }
        // смена инструмента
        else if(symbol == 'T') {
          byte b = get_byte();
        }
        // нераспознанный символ!
        // возможно ошибка парсинга текста. во избежание поломок останавливаемся с ошибкой
        else {
          errorCode = 2;
          lcd.clear();
          lcd.print("-PARSING ERROR!-");
          lcd.setCursor(0, 1);
          lcd.print("Line: ");
          lcd.print(uiLinesCounter);
          while(!key_pressed()) {}          // ждём нажатия кнопки для подтверждения ошибки оператором    
        }

        
      }
      //Serial.println("Not COMMAND");
    }
    

  }
  return false;   // если данные кончились и в буфере и на SD карте, то возвращаем false
}

//*******************************************************************
// функция превращает текстовую строку в  число float
// и записывает полученное число по указателю на переменную float
// указатель файлового буфера должен указывать либо на знак "-" либо на число
// при возврате указатель файлового буффера указывает на первый символ после числа
bool get_float(float &pfNumber) {
  bool bPoint = false;      // была десятичная точка?
  bool bNegative = false;   // отрицательное число?
  float fNumber = 0.0;
  float fDecimal = 0.1;
  while (bufferPointer < fileBufferSize || fill_buffer()) {    
    // берём символ с текущего указателя на файловый буфер
    char symbol = fileBuffer[bufferPointer];  
    // если цифра
    if(isDigit(symbol)) {
      symbol -= 48;
      if(!bPoint) {
        fNumber *= 10.0;
        fNumber += float(symbol);        
        //Serial.println(fNumber);
      } else {
        fNumber += fDecimal * float(symbol);                 
        fDecimal /= 10.0;                     // сдвигаем вправо для следующего десятичного знака 
        //Serial.println(fNumber);
      }
    }
    else {
      if(symbol == '-')  {   // число отрицательное
        bNegative = true;
        bufferPointer++;
        continue;
      }
      else if(symbol == '.')  {   // десятичная точка
        bPoint = true;
        bufferPointer++;
        continue;
      }
      else {                // конец цикла
        if(bNegative) {  fNumber *= -1.0; }
        //Serial.print("fNumber "); Serial.println(fNumber);
        pfNumber = fNumber; 
        return true;
      }
    }
    bufferPointer++;
  }
  return false;   // если данные кончились то возвращаем false
}


//*******************************************************************
// функция превращает текстовую строку в координату (целое число шагов)
// и записывает полученное число по указателю на координату
// указатель файлового буфера должен указывать либо на знак "-" либо на число
// при возврате указатель файлового буффера указывает на первый символ после числа
/*
bool get_coord(uint16_t &coordinate) {
  bool bPoint = false;      // была десятичная точка?
  bool bNegative = false;   // отрицательное число?
  float fNumber = 0.0;
  float fDecimal = 0.1;
  while (bufferPointer < fileBufferSize || fill_buffer()) {    
    // берём символ с текущего указателя на файловый буфер
    char symbol = fileBuffer[bufferPointer];  
    // если цифра
    if(isDigit(symbol)) {
      symbol -= 48;
      if(!bPoint) {
        fNumber *= 10.0;
        fNumber += float(symbol);        
        //Serial.println(fNumber);
      } else {
        fNumber += fDecimal * float(symbol);                 
        fDecimal /= 10.0;                     // сдвигаем вправо для следующего десятичного знака 
        //Serial.println(fNumber);
      }
    }
    else {
      if(symbol == '-')  {   // число отрицательное
        bNegative = true;
        bufferPointer++;
        continue;
      }
      else if(symbol == '.')  {   // десятичная точка
        bPoint = true;
        bufferPointer++;
        continue;
      }
      else {                // конец цикла
        if(bNegative) {  fNumber *= -1.0; }
        //Serial.print("fNumber "); Serial.println(fNumber);
        //*pfNumber = fNumber; 
        return true;
      }
    }
    bufferPointer++;
  }
  return false;   // если данные кончились то возвращаем false
}
*/

//*******************************************************************
// функция превращает текстовую строку в целое число размерности байт
// и возвращает полученное число
// число не может быть больше 255 
// указатель файлового буфера должен указывать на цифру
// при возврате указатель файлового буффера указывает на первый символ после числа
byte get_byte() {
  byte Number = 0;
  while(bufferPointer < fileBufferSize || fill_buffer()) {    
    // берём символ с текущего указателя на файловый буфер
    char symbol = fileBuffer[bufferPointer];  
    if(!isDigit(symbol)) break;               // если дошли до символа, то возвращаем значение
    symbol -= 48;
    Number *= 10;
    Number += (byte)symbol;
    bufferPointer++;
  }
  //Serial.print("byte "); Serial.println(Number);
  return Number;
}

//*********************************************************************
// функция останавливает движение моторов и выключает шпиндель
// показывает на экране что работа выполнена и время выполнения работы
// -= WORK DONE =- 
// Time: 00h 00m 00
// 1234567890123456
void stop_work() {
  // stop spindel
    // stop motors(disable)
    TIMSK1 |= B00000000;  // выключить прерывание таймера       
    lcd.clear();
    lcd.print("-= WORK DONE =- ");
    lcd.setCursor(0, 1);
    unsigned long workTime = (millis() - startTime)/1000;
    unsigned int hours = workTime / 3600;
    unsigned int minutes = (workTime % 3600) / 60;
    unsigned int seconds = (workTime % 3600) % 60;
    lcd.print("Time: ");
    lcd.print(hours); lcd.print("h "); 
    lcd.print(minutes); lcd.print("m ");
    lcd.print(seconds); lcd.print("s");  
    while(!key_pressed()) {}          // ждём нажатия кнопки для подтверждения выполнения оператором
}



//************************************************************************
// функция выводит сообщение об ошибке в зависимости от кода общей ошибки
void show_error() {
    lcd.clear();
    lcd.print("ERROR ");
    lcd.print(errorCode);
    lcd.setCursor(0, 1);
    lcd.print(errorMessages[errorCode]);
    while(!key_pressed()) {}          // ждём нажатия кнопки для подтверждения ошибки оператором
}


//************************************************************************
// функция выводит двухстрочное меню, из которого можно найти начала станка вручную,
// востановить начала станка из EEPROM и другое. 
void show_menu() {

    lcd.clear();
    lcd.print("MENU ");  lcd.print(menuLevel);  lcd.print("/");  lcd.print(menuItem);
  
}

void set_home_position() {
  int key = NONE;
  do {
    key = key_pressed();
    if(key == UP && ocr < 64000)          ocr += 1000;
    if(key == DOWN && ocr > 1000 )        ocr -= 1000;
    if(key == LEFT)                       set_dirX_left;
    if(key == RIGHT)                      set_dirX_right;
  }        
  while(key != SELECT);
  //Serial.println(ocr);
  delay(200);

  
}

//************************************************************************
// функция выводит информацию о выполняющейся работе
// использует bufferPointer как тайиер: 
// если работаем в первой половине буфера, то экран:
// X000.0 Y000.0...
// Z000.0 F1000....
// 1234567890123456
// если работаем во второй половине буфера, то экран:
// Buffers: 0000000
// 0[==========]100
// 1234567890123456
void show_work() {
  lcd.clear();
  if( bufferPointer < 2048 ) {
    lcd.print("X");
    lcd.print(curX);
    lcd.setCursor(7, 0);
    lcd.print("Y");
    lcd.print(curY);    
    lcd.setCursor(0, 1);
    lcd.print("Z");
    lcd.print(curZ);
    lcd.setCursor(7, 1);
    lcd.print("F");
    lcd.print(feedSpeed);         
  } else {
    lcd.print("Buffers: ");
    lcd.print(buffersCounter);
    lcd.setCursor(0, 1);
    lcd.print("0[==========]100");
  }

}

//***********************************************************************
// функция заполняет файловый буфер из открытого файла
// если файл не открыт, то ошибка 
// возвращает true если буфер прочитан с карты
// возвращает false если нет больше данных для буфера в файле (файл полностью прочитан) 
bool fill_buffer() {
   int i = 0;
   if (!myFile.available())
        return false;
   // read from the file until there's nothing else in it:
   while (myFile.available() && i<fileBufferSize) {
     fileBuffer[i] = myFile.read();
     i++;
   }
   // сбрасываем указатель буфера
   bufferPointer = 0;
   buffersCounter++;
   return true;  
}

//**********************************************************************
// функция читает аналоговый выход A0 и возвращает код нажатой кнопки
// >950 - 0 ничего не нажато
// >600 - 1 нажата SELECT
// >550 - 2 нажата LEFT
// >220 - 3 нажата DOWN
// >60 - 4 нажата UP
// иначе - 5 нажата RIGHT
int key_pressed() {
  unsigned int a0 = analogRead(0);
  for( byte i=0; i<7; i++ ) { a0 += analogRead(0); } // читаем 7 + 1 раз значение пина A0
  a0 = a0 >> 3 ;                                  // делим на 8, получаем среднее значение
  if(a0 > 950) { /*bPressed = false;*/ return 0; }  
  //if(bPressed) { return 0; }       // если клавиша уже была нажата, то выходим. защита от ложных срабатываний
  //bPressed = true;
  if(a0 > 600)  return 1;
  if(a0 > 400)  return 2;
  if(a0 > 220)  return 3;
  if(a0 > 60)   return 4;
  return 5;
}

//*******************************************************************
// прерывание от таймера для того чтобы сделать шаг
// вызывается в два раза чаще чем реальная частота переключений
// четный или нечетный тик прерывания по таймеру. в нечётных ставим сигнал шага. в четных снимаем. 
// период обработчика прерывания равен половине периода шага
// частота обработчика прерывания в два раза выше частоты шагов, выдаваемых на моторы  
ISR (TIMER1_COMPA_vect) {
    bOdd = !bOdd;       // инвертируем флаг
    
    if(!bOdd) {
      clear_stepsXYZ;   // спад импульсов шагов по осям
      return;
    }
    
    do_step_X;     // SET_MASK_0;
    
    //digitalWrite(PIN_XSTEP, HIGH);
    //digitalWrite(PIN_YSTEP, HIGH);
    //digitalWrite(PIN_ZSTEP, HIGH);
    
  
}



//*******************************************************************
// прерывание кнопки EStop
void ISR_Estop() {
  // моторы выключаем
  // шпиндель выключаем
  errorCode = 4;

  
}

//*******************************************************************
// прерывание от ошибки шпинделя
void ISR_Spindel() {
  // моторы выключаем
  // шпиндель выключаем
  errorCode = 5;
 
}



