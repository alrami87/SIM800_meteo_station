сегj#define DS_PIN 1
#define RAIN_PIN 2
#define WIND_PIN 3
#define IN_HEAT_PIN 4
#define EXT_HEAT_PIN 5
#define ARDUINO_RESET_PIN 6
#define SIM800_RESET_PIN 10
#define SIM800_TX_PIN 8
#define SIM800_RX_PIN 9

//I2C_PINS A4 A5

#include <SoftwareSerial.h>
SoftwareSerial SIM800(SIM800_TX_PIN, SIM800_RX_PIN);

#include <Adafruit_Si7021.h>
Adafruit_Si7021 Si7021;

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

#include <QMC5883LCompass.h>
QMC5883LCompass compass;

#include <RCWL_1X05.h>
RCWL_1X05 sonar;

#define rain_calibrate_coef 0.1  // кол-во осадков в капле
#define wind_calibrate_coef 1306 // калибровочный коэффициент анемометра
#define read_interval   10000  // милисекунд, измерение раз в 10 секунд //  10 000
#define send_interval  300000 // милисекунд, отправка раз в 5 минут    //  300 000
#define temp_comfort 10   // минимальная температура внутри
#define temp_hyster 1       // температурный гистерезис внутри
#define hum_heat_count 120  // 10 часов (+1 раз в 5 мин) при 100% влажности включаем прогрев Si7021
#define installation_height 200  // высота установки дальномера, см
#define anemometer_angle 0  // поправка на азимут анемометра, градусы

// подключение к интернету
int d = 400, ATsCount = 7;
String ATs[] = {  //массив АТ команд
  "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"",  //Установка настроек подключения
  "AT+SAPBR=3,1,\"APN\",\"internet\"",
  "AT+SAPBR=3,1,\"USER\",\"gdata\"",
  "AT+SAPBR=3,1,\"PWD\",\"gdata\"",
  "AT+SAPBR=1,1",  //Устанавливаем GPRS соединение
  "AT+HTTPINIT",  //Инициализация http сервиса
  "AT+HTTPPARA=\"CID\",1"  //Установка CID параметра для http сессии
};
int ATsDelays[] = {6, 1, 1, 1, 3, 3, 1}; //массив задержек

boolean heating = 1, rain_sending = 0;
int count = 0, wind_azimuth, signal_q_send;
unsigned long tim_read, tim_send, t_rain, t_wind;
unsigned long wind_gercon = 0, gust_gercon, wind_dir_gercon, gust = 0, rain_gercon = 0, dist = 0;
float temp = 0, hum = 0, pres = 0, wind_x = 0, wind_y = 0, wind_speed_moment;
float temp_send, hum_send, pres_send, in_temp_send, rain_count_send, wind_speed_send, wind_gust_send, wind_dir_send, depth_send, surf_temp_send, soil_temp_send, uptime_send, signal_send;


void setup() {
  delay(3000); //дадим время на инициализацию GSM модулю
  Wire.begin();

  pinMode (RAIN_PIN, INPUT_PULLUP);
  pinMode (WIND_PIN, INPUT_PULLUP);
  pinMode (IN_HEAT_PIN, OUTPUT);
  digitalWrite(IN_HEAT_PIN, heating);
  pinMode (SIM800_RESET_PIN, OUTPUT);
  digitalWrite(SIM800_RESET_PIN, HIGH);  // резет модема

  Serial.begin(9600);  //скорость порта
  Serial.println();
  Serial.println("Started!");

  SIM800.begin(9600);
  SIM800_init();
  delay(10);
  bmp.begin();
  delay(10);
  Si7021.begin();
  delay(10);
  Si7021.heater(false);
  delay(10);
  compass.init();
  delay(10);
  sonar.begin();

  tim_read = millis();
  tim_send = millis();

  delay(2000);
  SIM800_init();

  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rain, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), wind, FALLING);

  Si7021.reset();
}

void loop() {
  //Read
  if (millis() - tim_read > read_interval) {
    read_sensors();  // читаем и суммируем показания

    detachInterrupt(digitalPinToInterrupt(WIND_PIN));
    detachInterrupt(digitalPinToInterrupt(RAIN_PIN));

    String sms_text = read_SMS();  // проверяем смс
    if (sms_text == F("reboot"))  {  // перезагрузка модема и ардуины
      digitalWrite(SIM800_POWER_PIN, HIGH);
      delay(d);
      digitalWrite(SIM800_POWER_PIN, LOW);
      delay(d);
      pinMode (ARDUINO_RESET_PIN, OUTPUT);
      digitalWrite(ARDUINO_RESET_PIN, LOW);  //  перезагрузка ардуино
      delay(d);
      digitalWrite(ARDUINO_RESET_PIN, HIGH);
    }
    if (sms_text == F("reboot_modem")) {  // перезагрузка модема
      digitalWrite(SIM800_POWER_PIN, HIGH);
      delay(d);
      digitalWrite(SIM800_POWER_PIN, LOW);
      delay(d);
      SIM800_init();
    }
    if (sms_text == F("send"))  {  // отправка показаний по смс
      send_SMS();
    }

    attachInterrupt(digitalPinToInterrupt(WIND_PIN), wind, FALLING);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rain, FALLING);

    tim_read = millis();
  }

  //Send
  if (millis() - tim_send > send_interval) {
    detachInterrupt(digitalPinToInterrupt(WIND_PIN));
    detachInterrupt(digitalPinToInterrupt(RAIN_PIN));
    calculate_sensors();  // усредняем показания

    if (GPRS_state() != 1)  GPRS_init();  // если интернет не активирован, активируем
    
    send_sensors();  // отправляем показания на сайт

    rain_count_send = 0;
    rain_sending = 0;

    attachInterrupt(digitalPinToInterrupt(WIND_PIN), wind, FALLING);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rain, FALLING);

    tim_read = millis();
    tim_send = millis() + 10;
  }
}
