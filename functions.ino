///////////////
void read_sensors() {
  Serial.println("- r " + String(count + 1));

  // суммирование для усреднения
  temp += Si7021.readTemperature();
  hum += Si7021.readHumidity();
  pres += bmp.readPressure();
  in_temp_send = bmp.readTemperature();

  //sonar.setTemperature(temp_send);
  dist += sonar.read() * sqrt(Si7021.readTemperature() / 293) / 10.0; // определение расстояния до снега в сантиметрах, с учетом скорости звука при данной температуре

  wind_azimuth = 359 - compass.getAzimuth() + anemometer_angle;    // определение направления ветра (плюс поправка на поворот флюгера к северу)
  if (wind_azimuth >= 360)  wind_azimuth = wind_azimuth - 360;     // вычитание одного оборота, если азимут больше 360 градуов
  wind_speed_moment = 1.0 * wind_dir_gercon * wind_calibrate_coef / read_interval;   // скорость ветра во время текущего измерения
  wind_x += wind_speed_moment * sin(wind_azimuth * PI / 180.0);
  wind_y += wind_speed_moment * cos(wind_azimuth * PI / 180.0);

  count++;  // количество слагаемых
  wind_dir_gercon = 0;

  Serial.println("Si7021   " + String(temp / count, 1) + " C,  " + String(hum / count, 1) + " %");
  Serial.println("BMP   " + String(pres * 0.00750062 / count, 2) + " mmHg,  " + String(in_temp_send, 1) + " C");
  Serial.println("DEPTH   " + String(dist / count, 1) + " cm,  compas:"  + String(wind_azimuth) + "*");
  Serial.println("Rain " + String(rain_gercon) + ",    Wind " + String(wind_gercon));

  // определения быстрейшего порыва ветра
  if (gust_gercon > gust)  gust = gust_gercon;
  //Serial.println("   <--  Reading " + String(count) + "   gercon " + String(gust_gercon) + "   gust " + String(gust));
  gust_gercon = 0;

  // Обогрев воронки
  if (in_temp_send < temp_comfort) {
    heating = 0;   // обогрев вкл
    digitalWrite(HEAT_PIN, heating);
    //heater_count++;    // считаем интервалы когда работала грелка
  }
  if (in_temp_send > temp_comfort + temp_hyster) {
    heating = 1;   // обогрев выкл
    digitalWrite(HEAT_PIN, heating);
  }
}
////////////
void calculate_sensors() {
  Serial.println();
  Serial.println("-->  calc");

  // вычисление Si7021 или HTU21d
  temp_send = temp / count;
  hum_send = hum / count;
  // Проверка на ненулевое значение
  if ( isnan(temp_send) )  temp_send = Si7021.readTemperature();
  if ( isnan(hum_send) )  hum_send = Si7021.readHumidity();

  // вычисление BMP280
  pres_send = pres  * 0.00750062 / count;

  // вычисление глубины снега
  depth_send = constrain( installation_height - (1.0 * dist / count), 0, 2);

  // параметры ветра
  wind_speed_send = 1.0 * wind_gercon * wind_calibrate_coef / send_interval;    // скорость ветра
  wind_gust_send  = 1.0 * gust * wind_calibrate_coef / read_interval;           // порывы ветра
  wind_dir_send = atan2(wind_x / count, wind_y / count) * 180.0 / PI;           // вычисление угла вектора направления ветра
  wind_dir_send = wind_dir_send < 0 ? 360 + wind_dir_send : wind_dir_send;      // перевод в градусы

  // качество сигнала
  signal_q_send = GPRS_csq();

  // кол-во освдков
  rain_count_send = 1.0 * rain_gercon * rain_calibrate_coef;

  // обнуление счетчика и переменных накопления
  count = 0;
  temp = 0;
  hum = 0;
  pres = 0;
  dist = 0;
  wind_x = 0;
  wind_y = 0;
  wind_gercon = 0;
  gust_gercon = 0;
  gust = 0;

  Serial.println("SI   " + String(temp_send, 1) + " C,  " + String(hum_send, 1) + " %  ");
  Serial.println("BMP   " + String(pres_send, 2) + " mmHg,  " + String(in_temp_send, 1) + " C");
  Serial.println("SNOW   " + String(depth_send, 1) + " cm,  RAIN " + String(rain_count_send, 1) + " mm");
  Serial.println("WIND   " + String(wind_speed_send) + " m/s,  GUST " + String(wind_gust_send) + " m/s,  " + String(wind_dir_send) + " dgr");
}
//////////////////////////
void SIM800_init() {
  SIM800.println(F("AT"));   // активируем АТ комманды
  delay(d);
  SIM800.println(F("AT+CPMS=\"SM\""));
}
////////////////////////
void GPRS_init() {
  Serial.println(F("GPRS start"));
  for (int i = 0; i < ATsCount; i++) {
    Serial.println(ATs[i]);  //посылаем в монитор порта
    SIM800.println(ATs[i]);  //посылаем в GSM модуль
    delay(d * ATsDelays[i]);
    Serial.println(ReadGSM());  //показываем ответ от GSM модуля
    delay(d);
  }
  Serial.println(F("GPRS OK"));
}
////////////////////////////////
void send_sensors() {
  Serial.println(F("Send start"));
  // Serial.println("setup url");
  // формирование GET запроса
  SIM800.print(F("AT+HTTPPARA=\"URL\",\"http://narodmon.ru/get?"));  // url
  SIM800.print(F("ID=EC:FA:BC:26:D6:1E")); // MAC wemod-D1
  SIM800.print(F("&name=GSM_meteo"));   // имя станции
  SIM800.print(F("&lat=59.440676&lon=30.161269")); // координаты
  SIM800.print(F("&alt=75"));          // высота над уровнем моря
  SIM800.print(F("&TEMP="));       // температура улица
  SIM800.print(String(temp_send, 2));
  SIM800.print(F("&HUM=83.7"));        // влажность улица
  SIM800.print(String(hum_send, 2));
  SIM800.print(F("&BMPP="));    // атм. авление
  SIM800.print(String(in_temp_send, 2));
  SIM800.print(F("&BMPT="));       // температура внутри
  SIM800.print(String(pres_send, 2));
  SIM800.print(F("&RAIN="));        // кол-во осадков
  SIM800.print(String(rain_count_send, 2));
  SIM800.print(F("&WIND="));        // скорость ветра
  SIM800.print(String(wind_speed_send, 2));
  SIM800.print(F("&WS="));          // порывы ветра
  SIM800.print(String(wind_gust_send, 2));
  SIM800.print(F("&DIR="));       // направление ветра
  SIM800.print(String(wind_dir_send, 2));
  SIM800.print(F("&T1="));         // температура поверхности почвы
  SIM800.print(String(surf_temp_send, 2));
  SIM800.print(F("&T2="));         // температура на глубине
  SIM800.print(String(soil_temp_send, 2));
  SIM800.print(F("&UPTIME="));     // время работы
  SIM800.print(String(uptime_send, 2));
  SIM800.print(F("&GSM="));          // уровень сигнала
  SIM800.print(String(signal_send, 2));
  SIM800.print(F("&HEIGH="));      // высота снежного покрова
  SIM800.print(String(depth_send, 2));
  SIM800.println(F("\""));
  delay(d);
  Serial.println(F("GET url"));
  // отправка GET запроса
  SIM800.println(F("AT+HTTPACTION=0"));
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d);
  Serial.println(F("Send done"));
}
/////////////////////////////
String ReadGSM() {  //функция чтения данных от GSM модуля
  String _resp = "";                            // Переменная для хранения результата
  long _timeout = millis() + 1000;             // Переменная для отслеживания таймаута (1 сек)
  while (!SIM800.available() && millis() < _timeout)  {}; // Ждем ответа 10 секунд, если пришел ответ или наступил таймаут, то...
  if (SIM800.available()) {                     // Если есть, что считывать...
    _resp = SIM800.readString();                // ... считываем и запоминаем
  }
  else {                                        // Если пришел таймаут, то...
    Serial.println(F("Timeout..."));               // ... оповещаем об этом и...
  }
  return _resp;
}
////////////////////
int GPRS_state() {    // состояние GPRS подключения
  SIM800.println(F("AT+SAPBR=2,1"));
  String text = ReadGSM();
  text = text.substring(text.indexOf("+SAPBR:") + 8);
  return text.substring(0, text.indexOf(",")).toInt();
}
////////////////////
int GPRS_csq() {    // уровень GSM сигнала
  SIM800.println(F("AT+CSQ"));
  String text = ReadGSM();
  return text.substring(text.indexOf("+CSQ:") + 6, text.indexOf(",")).toInt();
}
/////////////
void wind() {   // обработка концевика дождя
  if (millis() - t_wind > 10) {
    wind_gercon++;
    gust_gercon++;
    wind_dir_gercon++;
    //Serial.println("-  " + String(wind_gercon) + "   " + String(gust_gercon));
    t_wind = millis();
  }
}
////////////////////////////
void rain() {   // обработка концевика ветра
  if (millis() - t_rain > 100) {
    rain_gercon++;
    //Serial.println("..." + String(rain_gercon));
    t_rain = millis();
  }
}
///////////////////////////////
String read_SMS() {
  Serial.println(F("Reading SMS..."));             // чтение смс
  SIM800.println(F("AT+CMGR=1"));  // запрос сообщения
  String text = ReadGSM();
  text = text.substring(text.indexOf("+CMGR:"));   // отбрасываем AT+CMGR=1 и +CMGR:
  text = text.substring(text.indexOf("\r") + 2);   // ищем новую строку
  String code = text.substring(0, text.indexOf(":"));  //  чтение кода до ":"
  text = text.substring(text.indexOf(":") + 1, text.indexOf("\r"));  // чтение текста  после ":" до переноса строки
  SIM800.println(F("AT+CMGDA=\"DEL ALL\""));// удаление всех смс
  ReadGSM();
  if (code == F("5537")) {
    return text;
  } else {
    return F("error");
  }
}
/////////////////////////////
void send_SMS() {
  //Serial.println("Sending SMS...");                // Печать текста
  SIM800.println(F("AT+CMGF=1"));                   // Выбирает текстовый формат SMS
  delay(d);
  SIM800.println(F("AT+CMGS=\"+79215725333\""));  // Отправка СМС на указанный номер +792100000000"
  delay(d);

  SIM800.println("Si7021 " + String(temp_send, 1) + "c, " + String(hum_send, 1) + "%");
  delay(d);
  SIM800.println("BMP " + String(pres_send, 2) + "mm, " + String(in_temp_send, 1) + "c");
  delay(d);
  SIM800.println("SNOW " + String(depth_send, 1) + ", RAIN " + String(rain_count_send, 1));
  delay(d);
  SIM800.println("WIND " + String(wind_speed_send) + ", " + String(wind_gust_send) + ", DIR " + String(wind_dir_send));
  delay(d);

  SIM800.println((char)26);// (требуется в соответствии с таблицей данных)
  delay(d);
  ReadGSM();
  Serial.println(F("Text Sent"));
}
