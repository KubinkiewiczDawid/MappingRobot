#include <SPI.h>
#include <SD.h>

#include <AFMotor.h>
#include "SR04.h" //implementacja bilbioteki do sterowania czujnikami HC-SR04
#include "TimerOne.h"

//Ustawienia karty SD
File mojPlik;
const int8_t WYLACZ_CHIP_SD = 10; //wyłączenie drugiego pinu SD na arduino

int pinCS = 53; // Pin komunikacji z karta SD

//zmienne dla predkosciomierza
const byte silnik_A = 20; //pin przerwania silnik1
const byte silnik_B = 21; //pin przerwania silnik2

volatile int licznik_A = 0; //licznik pulsów enkodera 1
volatile int licznik_B = 0; //licznik pulsów enkodera 2 

volatile int obrot_A; //zmienna prędkości silnika1
volatile int obrot_B; //zmienna prędkości silnika2

float wnekiEnkoderow = 20.00; // liczba wnęk w enkoderach

const float srednicaKola = 65.10; // średnica kół (mm)

//Zmienne dla millis
unsigned long startMillis;
unsigned long obecneMillis;
const unsigned long czestotliwoscSD = 100; //częstotliwość zapisu danych na kartę SD w [ms]

bool czasNaZapis = false;

// struktura do trzymania danych
struct robotLog
{
  uint8_t kierunek; // 0 - prosto 1 - skret
  int predkosc_A; // predkosc silnika1
  int predkosc_B; // predkosc silnika2
};

typedef struct robotLog RobotLog;

// tworzenie bufora (wielkość można zmienić
const unsigned int iloscDanychDoBufora = 10;
RobotLog sdBufor[iloscDanychDoBufora];

// indeks do wyznaczenie obecnej pozycji w buforze
unsigned int indeksPozycjiBufora = 0;

//Ustawienie sensorow
int LEWY_SENSOR_TRIG = A2;
int LEWY_SENSOR_ECHO = A3;
int PRZEDNI_SENSOR_TRIG = A0;
int PRZEDNI_SENSOR_ECHO = A1;

SR04 sr04_lewo = SR04(LEWY_SENSOR_ECHO, LEWY_SENSOR_TRIG);
SR04 sr04_przod = SR04(PRZEDNI_SENSOR_ECHO, PRZEDNI_SENSOR_TRIG);

//Ustawienie silnikow
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(3, MOTOR34_1KHZ);

int BRAK_PRZESZKODY = 0;
int PRZESZKODA = 1;

//Zmienne PID
const int Kp_lewo = 220; // stala uzyta do skretu
const int Ki_lewo = 1; // stala calki uzyta do poprawy bledu
const int Kd_lewo = 1550; // stala rozniczki uzyta do poprawy predkosci poprawy bledu

const int Kp_przod = 4000; // stala uzyta do skretu
const int Ki_przod = 0; // stala calki uzyta do poprawy bledu
const int Kd_przod = 2300; // stala rozniczki uzyta do poprawy predkosci poprawy bledu

const int docelowaObleglosc = 12; // docelowa odleglosc w cm
const int maksymalnaOdlegloscPrzod = 18; // maksymalna odległość do przeszkody z przodu

int Dp = 65; // docelowa prędkość

float blad_lewo = 0; //błąd odległości od ściany
float ostatniBlad_lewo = 0; //ostatni błąd odległości od ściany
float poprawaSledzenia = 0; //poprawa prędkości silników w celu śledzenia ściany
float p_lewo = 0;  //proporcja
float i_lewo = 0;  //calka
float d_lewo = 0;  //rozniczka

float blad_przod = 0; //błąd odległości od przeszkody
float ostatniBlad_przod = 0; //ostatni błąd odległości od przeszkody
float poprawaPrzeszkody = 0;
float p_przod = 0;  //proporcja
float i_przod = 0;  //calka
float d_przod = 0;  //rozniczka

int predkoscLewegoSilnika = 0;
int predkoscPrawegoSilnika = 0;

int sciana = 0;
int przod = 0;

int statusRobota = 0; // 0 - jazda prosto; 1 - skret w lewo; 2 - przeszkoda
bool statusPrzodu = false;

//0 - true; 1 - false
int skretPrawo = 0;
int skretLewo = 0;
int jazdaProsto = 0;

void setup() {
  Serial.begin(9600);
  
  //Ustawienie czytnika kart SD
  pinMode(53, OUTPUT); //ustawienie pinu wyjsciowego do modułu SD
  
  if (!SD.begin(pinCS)) //sprawdzenie czy karta SD jest włożona
  {
    Serial.println("błąd inicializacji karty SD");
  } else
  {
    Serial.println("karta SD gotowa");
  }

  SD.remove("DaneMapy.txt"); //usunięcie pliku z wcześniejszego zapisu
  mojPlik = SD.open("DaneMapy.txt", O_CREAT | O_APPEND | O_WRITE); //stworzenie nowego pliku tekstowego na karcie SD
  mojPlik.close(); //wymagana procedura zamkniecia pliku na karcie SD po kazdym otwarciu

  //włączenie przerwań dla prędkościomierza
  attachInterrupt(digitalPinToInterrupt (silnik_A), ISR_licznik_A, RISING); //zwieksz licznik1 gdy pin predkosciomieza jest na High
  attachInterrupt(digitalPinToInterrupt (silnik_B), ISR_licznik_B, RISING); //zwieksz licznik2 gdy pin predkosciomieza jest na High
}

void loop() {
  obecneMillis = millis();

  czasomierzMillis();
  if(czasNaZapis == true){
    if(predkoscLewegoSilnika != 0 && predkoscPrawegoSilnika != 0){
      licz_predkosc();
      zapiszDane();
    }
  }
  else{
    sekwencja_startowa();
  }
}

void sekwencja_startowa(){
  statusPrzodu = PRZEDNI_STATUS();

  if(statusPrzodu == true){
    if(skretLewo == 0){
      zerowanie_sciana();
    }
    sekwencjaPrzeszkody();

    skretPrawo = 1; // Zmieniam flage skretu na false do nastepnego czasu az natrafi na przeszkode
    jazdaProsto = 0;
  }
  if(statusPrzodu == false){
    sledzSciane();
  }
}

void sledzSciane(){
  //ustawianie flagi skrętu w prawo na false
  skretPrawo = 0;

  //odliczenia PID dla śledzenia ściany
  sciana = sr04_lewo.Distance(); //przypisanie odległości od ściany lewego czujnika do zmiennej "sciana"
  blad_lewo = sciana - docelowaObleglosc; //obliczenie błędu odległości od ściany
  i_lewo = (i_lewo + blad_lewo); //oblicznie części całkującej PID
  d_lewo = (blad_lewo - ostatniBlad_lewo); //obliczenie części różniczkowej PID

 // Serial.println(blad_lewo);

  //obliczenie korekty jaką trzeba nanieść na prędkości kół robota w celu śledzenia ściany
  poprawaSledzenia = Kp_lewo * blad_lewo + Ki_lewo * i_lewo + Kd_lewo * d_lewo;
  poprawaSledzenia = poprawaSledzenia/100;

   //jezeli error_wall jest mniejszy od 10, oznacza to ze robot jedzie obok sciany poprawnie
  if(blad_lewo < 10){
    if(jazdaProsto == 0){
      zerowanie_sciana(); //zerowanie zmiennych przed każdym śledzeniem ściany
    }
    //Ustawienie limitu maksymalnej liczby prędkości
    if(poprawaSledzenia > 30 && poprawaSledzenia > 0){
      poprawaSledzenia = 30;
    }
    //Ustawienie limitu minimalnej liczby prędkości
    if(poprawaSledzenia < -30 && poprawaSledzenia < 0){ 
      poprawaSledzenia = -30;
    }

    //ustalenie prędkości silników
    predkoscPrawegoSilnika = Dp + poprawaSledzenia;
    predkoscLewegoSilnika = Dp - poprawaSledzenia;

    jazdaProsto = 1;
    skretLewo = 0;
    statusRobota = 0; // 0 - jazda prosto
   //Serial.println("jazda prosto");
  }else{
    //jezeli error_wall jest wiekszy od 10, oznacza to ze robot musi skrecic w lewo
    //zeruje wartosci na poczatku skretu w lewo
    if(skretLewo == 0){
      zerowanie_sciana();
      zerowanie_przod();
    }

    //PID do skrętu w lewo
    int predkoscSkretu = 1.5 * blad_lewo + 11 * d_lewo; 

    //Ustawienie limitu maksymalnej liczby prędkości
    if(predkoscSkretu > 30 && predkoscSkretu > 0){
      predkoscSkretu = 30;
    }
    //Ustawienie limitu minimalnej liczby prędkości
    if(predkoscSkretu < -30 && predkoscSkretu < 0){
      predkoscSkretu = -30;
    }
    //ustalenie prędkości silników
    predkoscPrawegoSilnika = Dp + predkoscSkretu;
    predkoscLewegoSilnika = Dp - predkoscSkretu;

    skretLewo = 1;
    jazdaProsto = 0;
    statusRobota = 1; //1 - skret w lewo
    //Serial.println("jazda w lewo");
  }
  //przypisawie obliczonych wcześniej prędkości do silników
  motor1.setSpeed(predkoscPrawegoSilnika);
  motor1.run(FORWARD);
  motor2.setSpeed(predkoscLewegoSilnika);
  motor2.run(FORWARD);

  ostatniBlad_lewo = blad_lewo;
  zerowanie_przod(); //zerowanie wartości przed każdą przeszkodą
}

void sekwencjaPrzeszkody(){

  //odliczenia PID dla ominięcia przeszkody
  przod = sr04_przod.Distance(); //przypisanie odległości od ściany przedniego czujnika do zmiennej "przod"
  blad_przod = (przod - maksymalnaOdlegloscPrzod); //obliczenie błędu odległości od ściany
  i_przod = (i_przod + blad_przod); //oblicznie części całkującej PID
  d_przod = (blad_przod - ostatniBlad_przod); //obliczenie części różniczkowej PID

  //obliczenie korekty jaką trzeba nanieść na prędkości kół robota w celu ominiecia przeszkody
  poprawaPrzeszkody = Kp_przod * blad_przod + Ki_przod * i_przod + Kd_przod * d_przod;
  poprawaPrzeszkody = poprawaPrzeszkody/100;

  //Ustawienie limitu maksymalnej liczby prędkości
  if(poprawaPrzeszkody > 30 && poprawaPrzeszkody > 0){
    poprawaPrzeszkody = 30;
  }
  //Ustawienie limitu minimalnej liczby prędkości
  if(poprawaPrzeszkody < -30 && poprawaPrzeszkody < 0){
    poprawaPrzeszkody = -30;
  }
  
  //ustalenie prędkości silników
  predkoscPrawegoSilnika = Dp + poprawaPrzeszkody;
  predkoscLewegoSilnika = Dp - poprawaPrzeszkody;

  statusRobota = 2; //2 - skręt w prawo (przeszkoda)
  //Serial.println("jazda w prawo");

  //przypisawie obliczonych wcześniej prędkości do silników
  motor1.setSpeed(predkoscPrawegoSilnika);
  motor1.run(FORWARD);
  motor2.setSpeed(predkoscLewegoSilnika);
  motor2.run(FORWARD);

  ostatniBlad_przod = blad_przod;
}

//zerowanie wartości śledzenia ściany
void zerowanie_sciana(void) {
  i_lewo = 0;
  d_lewo = 0;
  ostatniBlad_lewo = 0;
}
//zerowanie wartości przeszkody
void zerowanie_przod(void) {
  i_przod = 0;
  d_przod = 0;
  ostatniBlad_przod = 0;
}

//status przedniego czujnika (sprawdzanie czy z przodu jest przeszkoda)
bool PRZEDNI_STATUS(void) {
  if (sr04_przod.Distance() < maksymalnaOdlegloscPrzod)
    return true; //przeszkoda
  else
    return false; //brak przeszkody
}

//Zapisywanie danych
void zapiszDane(){
  sdBufor[indeksPozycjiBufora].kierunek = statusRobota;
  sdBufor[indeksPozycjiBufora].predkosc_A = obrot_A;
  sdBufor[indeksPozycjiBufora].predkosc_B = obrot_B;

  indeksPozycjiBufora++;
  if (indeksPozycjiBufora == 10) {
    motor1.setSpeed(0);
    motor1.run(FORWARD);
    motor2.setSpeed(0);
    motor2.run(FORWARD);
    mojPlik = SD.open("DaneMapy.txt", O_CREAT | O_APPEND | O_WRITE);
    for (int i = 0; i < 10; i++) {
      if (mojPlik) {
        mojPlik.print(sdBufor[i].kierunek);
        mojPlik.print(",");
        mojPlik.print(sdBufor[i].predkosc_B);
        mojPlik.print(",");
        mojPlik.println(sdBufor[i].predkosc_A);
      }
    }
    mojPlik.close();
    indeksPozycjiBufora = 0;
  }
  czasNaZapis = false;
}

//czasomierz
void czasomierzMillis() {
  if (obecneMillis - startMillis >= czestotliwoscSD) {
    czasNaZapis = true;
    startMillis = obecneMillis;
  }
}

//funkcje dla prędkościomierza
// zliczanie pulsow motor1
void ISR_licznik_A() {
  licznik_A++;
}

//zliczanie pulsow motor2
void ISR_licznik_B() {
  licznik_B++;
}

//obliczanie prędkości obrotowej silników
void licz_predkosc(){
  obrot_A = (licznik_A / wnekiEnkoderow) * 60;
  licznik_A = 0;
  obrot_B = (licznik_B / wnekiEnkoderow) * 60;
  licznik_B = 0;
}
