#include <Wire.h>

#include <util/delay.h>
#include <avr/wdt.h>
#include <stdint.h>

/* ================= VAKIOT ================= */
#define I2C_ADDR_DEFAULT   0x40

/* ================= KOMENNOT ================= */
#define CMD_PA4_TIME    1          //Koska PA4 signaali tuli
#define CMD_ADC_START 2    //Aloita ADC mittaus
#define CMD_ADC_STOP 3     //Lopeta ADC mittaus
#define CMD_ADC_RESULT 4   //ADC keskiarvo
#define CMD_ADC_VALUE 5    //nyt ADC arvo
#define CMD_ADC_LOWER 6    //Lower limit asetus
#define CMD_ADC_UPPER 7    //Upper limit asetus
#define CMD_RESULT    8     //Mittauksen tulos
#define CMD_START     9     //Tapahtuman mittauksen aloitus
#define CMD_STOP      10     //Tapahtuman mittauksen lopetus
#define CMD_I2C_TEST 11     //Testaa I2C toimivuus
#define CMD_RUNTIME 12     //Palauta runtime
#define CMD_SPEED   13      //Adc laskujen keskinopeus
#define CMD_RESPONSE 14    //vastaa +1 arvolla
#define CMD_PRINT 15        //printtaa arvoja
#define CMD_REPORT_TRUE 16        //Ilmoita kun tapahtuma
#define CMD_REPORT_FALSE 17        //Älä ilmoita kun tapahtuma
#define CMD_VREF 18           //Aseta Vref
#define CMD_ID 19             //Palauta device ID
#define CMD_TEMP_START 20           //Aloita lämpötila mittaus
#define CMD_TEMP_STOP 21           //Palauta lämpötila mittaus

/* ================= DATA ================= */
uint8_t I2C_ADDR;        //I2C osoite

volatile uint8_t last_cmd = 0;
uint64_t t_last_message = 0;
uint16_t last_message=0;
volatile bool report = false;

volatile uint16_t lista[70];

volatile uint16_t last_ADC;
volatile uint8_t vaihe;
volatile uint16_t upperLimit;
volatile uint16_t lowerLimit;
volatile uint8_t last_ADC_idx;
volatile uint64_t vikaAika;
volatile uint16_t mid;
float midTarkka;
volatile uint64_t ekaAika;
volatile uint32_t sumXY;
volatile uint16_t sumX;
uint32_t lower;
uint32_t upper;
uint16_t sumY;
uint32_t sumXX;
volatile bool sendedSync;
volatile uint16_t keskiarvoitus = 0x0FFF; //0x1FFFF
volatile bool kellotetaanADC = false;
uint8_t Vref = 0;
int16_t Temp_offset;
uint8_t Temp_kerroin;

union Osissa64{
  uint64_t luku;
  uint8_t lista[8];
};

/* Aikaleimat */
volatile uint32_t tca_overflows = 0;
volatile uint64_t pa4_time = 1;
volatile uint16_t pa4_capture = 0;
volatile uint32_t pa4_ovf_snapshot = 0;
volatile uint64_t palautaSync = 0xFFFFFFFFFFFFFFFF;

uint32_t read_uid() {
  uint32_t tulos = 0;
  //tulos = (tulos << 8) | SIGROW.SERNUM3;
  //tulos = (tulos << 8) | SIGROW.SERNUM4;
  //tulos = (tulos << 8) | SIGROW.SERNUM5;
  tulos = (tulos << 8) | SIGROW.SERNUM6;
  tulos = (tulos << 8) | SIGROW.SERNUM7;
  tulos = (tulos << 8) | SIGROW.SERNUM8;
  tulos = (tulos << 8) | SIGROW.SERNUM9;
  return tulos;
  }
void tcd0_init(){
  TCD0.CTRLA=0;
  TCD0.CTRLB = 0; //One ramp
  TCD0.CMPBCLR = 550;
  //TCD0.CTRLA |= TCD_CLKSEL_SYSCLK_gc | TCD_ENABLE_bm;
  TCD0.CTRLA =  1;

  }

void adc_init(){
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; //Ei pullup ja Digital input buffer disabled

  VREF.CTRLA = Vref;                          //Vref arvo ADC0
  VREF.CTRLB = VREF_ADC0REFEN_bm;             //ADC Vref aina päällä

  ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;           //Single ended pinnin valinta
  ADC0.CTRLB  = 1;                            //Keskiarvoitus 2^n
  ADC0.CTRLC  = 0 << 6;                       //capasitanssi. 0 kun Vref<1V, 1 kun Vref>1V
  ADC0.CTRLC |= 0 << 4;                       //0 Vref, 1 VDD
  ADC0.CTRLC |= ADC_PRESC_DIV8_gc;            //ADC_clk = CLK/DIV
  ADC0.EVCTRL = ADC_STARTEI_bm;

  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.INTCTRL = ADC_RESRDY_bm;               //Result Ready Interrupt Enable
  ADC0.CTRLA  = ADC_ENABLE_bm;                //ADC enabled
  }

ISR(ADC0_RESRDY_vect){
  last_ADC = ADC0.RES;            //Luetaan ADC arvo
  ADC0.INTFLAGS = ADC_RESRDY_bm;              //kuitataan että luettu. Turha??
  if(vaihe==2){                                       //mittaus alkanut alhaalta
    if(last_ADC<upperLimit){                                //vielä ei ole ylittänyt ylärajaa
      uint16_t v = 0;
      if((last_ADC>=mid) && !sendedSync){
        if(report){
          sendReport();
        }
        sendedSync=true;
      }
      if(last_ADC>lowerLimit){
        v = last_ADC - lowerLimit;
      }
      lista[last_ADC_idx]=v;
      last_ADC_idx++;
    }else{                                                  //ylitti ylärajan
      //vikaAika = get_timestamp_cycles_safe();
      vikaAika= (estimate_center_time()*TCD0.CMPBCLR);
      vaihe = 4;
    }
  }else if(vaihe == 3){                                     //mittaus alkanut ylhäältä
    if(last_ADC>lowerLimit){                                //vielä ei alittanut alarajaa
      uint16_t v = 0;
      if((last_ADC<=mid) & !sendedSync){
        if(report){
          sendReport();
        }
        sendedSync=true;
      }
      if(last_ADC>lowerLimit){
        v = last_ADC - lowerLimit;
      }
      lista[last_ADC_idx]=v;
      last_ADC_idx++;
    }else{                                                  //alitti alarajan
      //vikaAika = get_timestamp_cycles_safe();
      vikaAika= (estimate_center_time()*TCD0.CMPBCLR);
      vaihe = 4;
    }
  }else if(vaihe == 1){                                           //ei vielä alkanut mittaus
    if(last_ADC<mid){                                       //Onko alhaalla
      if(last_ADC>lowerLimit){                              //tullut alhaalta
        //odotetaan että uusi mittaus valmis
        wdt_reset();
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
            // Tyhjä silmukka, odotetaan
        }
        last_ADC = ADC0.RES;            //Luetaan ADC arvo
        ekaAika = get_timestamp_cycles_safe();

        uint16_t v = 0;
        if(last_ADC>lowerLimit){
          v = last_ADC - lowerLimit;
        }
        lista[last_ADC_idx]=v;
        last_ADC_idx++;
        vaihe = 2;
        sendedSync=false;
      }
    }else if((last_ADC<upperLimit)){                        //tullut ylhäältä
      //odotetaan että uusi mittaus valmis
      wdt_reset();
      while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
          // Tyhjä silmukka, odotetaan
      }
      last_ADC = ADC0.RES;            //Luetaan ADC arvo
      ekaAika = get_timestamp_cycles_safe();


      uint16_t v = 0;
      if(last_ADC>lowerLimit){
        v = last_ADC - lowerLimit;
      }
      lista[last_ADC_idx]=v;
      last_ADC_idx++;
      vaihe=3;
      sendedSync=false;
    }
    }
  else if(vaihe == 5){                                     //mitataan keskiarvoa
    sumXY += last_ADC;
    if(sumX >= keskiarvoitus){                                          
      sumXY = (uint64_t)((uint64_t)sumXY << 16)/((uint64_t)(keskiarvoitus)+(uint64_t)1);
      vaihe=6; 
    }
    sumX++;
  }
  }

void pa4_event_init() {
  PORTA.DIRCLR = PIN4_bm;
  PORTA.PIN4CTRL = PORT_ISC_INTDISABLE_gc;
  PORTA.INTFLAGS = PIN4_bm;
  }

void evsys_init(void) {
  // Generator: PORTA PIN4 → ASYNC channel 0
  EVSYS.ASYNCCH0 = 0x0E;
  EVSYS.ASYNCUSER0 = 0x3;


  EVSYS.ASYNCCH1 = 0x04; //TCD0_CMPBCLR
  EVSYS.ASYNCUSER1 = 0x4; //multiplexer ADC0, Asyncch1
  }


void tcb0_init() {
  TCB0.CTRLA = 0; // stop
  TCB0.CTRLB = TCB_CNTMODE_CAPT_gc; // Capture mode
  TCB0.EVCTRL = TCB_CAPTEI_bm;      // Enable event input
  TCB0.INTCTRL = TCB_CAPT_bm;       // Capture interrupt
  TCB0.INTFLAGS = TCB_CAPT_bm;
  TCB0.CNT = 0;
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_SYNCUPD_bm | TCB_ENABLE_bm;

  }

ISR(TCB0_INT_vect) {
  // Clear interrupt
  TCB0.INTFLAGS = TCB_CAPT_bm;

  uint16_t cap = TCB0.CCMP;
  uint16_t aika1 = TCB0.CNT;
  uint64_t aika2 = get_timestamp_cycles_safe();
  uint16_t delta = (uint16_t)(aika1 - cap);
  pa4_time = aika2 - (uint64_t)delta;

  
  }

void tca0_init(){
  TCA0.SINGLE.CTRLA = 0; // pysäytä
  TCA0.SINGLE.CTRLD = 0; // SINGLE-mode
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
  TCA0.SINGLE.PER = 0xFFFF;

  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

  
  }




ISR(TCA0_OVF_vect){
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;   //kuittaus
  tca_overflows++;
  }

static inline uint64_t get_timestamp_cycles_safe(void){ //tarkista struct järjestys
  // Union joka yhdistää tca_overflows (32b) ja cnt (16b) 64-bittiseksi
  typedef union {
      struct {
          uint16_t cnt;    // alimmainen 16 bittiä
          uint32_t high;   // seuraavat 32 bittiä
          uint16_t reserved; // ylimääräinen 16 bittiä, jotta union on 64 bittiä
      };
      uint64_t full;
  } timestamp_u;

  timestamp_u ts;

  uint8_t sreg = SREG;
  cli();  // Keskeytykset pois

  ts.high = tca_overflows;           // asetetaan yläosa
  ts.cnt  = TCA0.SINGLE.CNT;         // alapuoli

  // Overflow-tarkistus
  if(TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm){
      ts.cnt = TCA0.SINGLE.CNT;
      ts.high++;
  }

  SREG = sreg;  // palautetaan keskeytykset
  return ts.full;


  /*
    uint64_t high;
    uint16_t cnt;
    uint8_t sreg = SREG;
    cli();
    high = tca_overflows;
    cnt  = TCA0.SINGLE.CNT;
    if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
      cnt = TCA0.SINGLE.CNT;
      high ++;
    }
    SREG = sreg;
    return ((high << 16) | cnt);*/
  }

void write7Bit(uint64_t val, uint8_t bytes) {

  for (int8_t i = bytes - 1; i >= 0; i--) {
    uint8_t chunk = (val >> (i * 7)) & 0x7F;
    Wire.write(chunk);
  }
  
  }

void sendReport(){
  PORTA.OUTCLR = PIN1_bm;   // LOW
  palautaSync = get_timestamp_cycles_safe() + 80*20; //80us eteenpäin
  }
void nollaa(){
  sumY = 0;
  sumXY = 0;
  sumX = 0;
  sumXX = 0;
  last_ADC_idx = 0; 
  }


float estimate_center_time(){ //1,2,4 aste
    uint16_t n = last_ADC_idx;
    float x_mean = 0.5f * (n - 1);

    // Momenttisummat
    float S0 = 0.0f, S1 = 0.0f, S2 = 0.0f, S3 = 0.0f;
    float S4 = 0.0f, S5 = 0.0f, S6 = 0.0f, S7 = 0.0f, S8 = 0.0f;

    // Dataan liittyvät summat
    float T0 = 0.0f, T1 = 0.0f, T2 = 0.0f, T3 = 0.0f, T4 = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        float x  = (float)i - x_mean;
        float x2 = x * x;
        float x3 = x2 * x;
        float x4 = x2 * x2;
        float x5 = x3 * x2;
        float x6 = x3 * x3;
        float x7 = x4 * x3;
        float x8 = x4 * x4;

        float y = lista[i]+lowerLimit;

        S0 += 1.0f;
        S1 += x;
        S2 += x2;
        S3 += x3;
        S4 += x4;
        S5 += x5;
        S6 += x6;
        S7 += x7;
        S8 += x8;

        T0 += y;
        T1 += x  * y;
        T2 += x2 * y;
        T3 += x3 * y;
        T4 += x4 * y;
    }

    // Normaaliyhtälöiden matriisi (4x4) ja oikea puoli
    float A[4][5] = {
        { S0, S1, S2, S4, T0 },
        { S1, S2, S3, S5, T1 },
        { S2, S3, S4, S6, T2 },
        { S4, S5, S6, S8, T4 }
    };

    // Gauss–Jordan-eliminaatio
    for (int i = 0; i < 4; i++)
    {
        float inv = 1.0f / A[i][i];
        for (int j = i; j < 5; j++)
            A[i][j] *= inv;

        for (int k = 0; k < 4; k++)
        {
            if (k == i) continue;
            float f = A[k][i];
            for (int j = i; j < 5; j++)
                A[k][j] -= f * A[i][j];
        }
    }

    float a = A[0][4];
    float b = A[1][4];
    float c = A[2][4];
    float e = A[3][4];

    // Keskipisteen ratkaisu: y(x)=0.5
    float x = (midTarkka - a) / b;   // alkuarvaus

    for (int it = 0; it < 2; it++)
    {
        float f  = a + b*x + c*x*x + e*x*x*x*x - midTarkka;
        float df = b + 2.0f*c*x + 3.0f*e*x*x*x;
        x -= f / df;
    }

    return x + x_mean;
  }

void setAlaYla(){
  lowerLimit=((upper-lower)/10+lower) / 65536;
  upperLimit=((upper-lower)*9/10+lower) / 65536;
  mid = ((lower+upper)/2) / 65536;
  midTarkka = ((lower+upper)/2) / 65536.f;
  }

void receiveEvent(int){
  if (Wire.available()){
    last_cmd = Wire.read();
  }
  t_last_message = 0;
  while (Wire.available()) {
    t_last_message = (t_last_message << 7) | (Wire.read() & 0x7F);
  }
  last_message=t_last_message;
  switch (last_cmd) {
    case CMD_ADC_LOWER:{
      lower = t_last_message;
      setAlaYla();
      break;
      }
    case CMD_ADC_UPPER:{
      upper = t_last_message;
      setAlaYla();
      break;
      }
    case CMD_START:{
      nollaa(); 
      vaihe = 1;
      break;
      }
    case CMD_STOP:{
      nollaa(); 
      vaihe = 0;
      break;
      }
    case CMD_ADC_START:{
      nollaa();
      vaihe = 5;
      break;
      }
    case CMD_ADC_STOP:{
      nollaa(); 
      vaihe = 0;
      break;
      }
    case CMD_REPORT_TRUE:{
      report=true;
      PORTA.OUTSET = PIN1_bm;
      PORTA.DIRSET = PIN1_bm;
      break;
      }
    case CMD_REPORT_FALSE:{
      report=false;
      PORTA.DIRCLR = PIN1_bm;
      PORTA.PIN1CTRL = PORT_PULLUPEN_bm;
      break;
      }
    case CMD_VREF:{
      Vref = last_message << 4 | last_message;
      VREF.CTRLA = Vref;                          //Vref arvo ADC0
      break;
      }
    case CMD_TEMP_START:{
      VREF.CTRLA = 1 << 4 | 1;
      ADC0.MUXPOS = 0x1E;
      _delay_us(1000);
      nollaa();
      vaihe = 5;
      break;
      }
    }
  }

void requestEvent(){
  switch (last_cmd){
    case CMD_I2C_TEST:{
      write7Bit(17,1);
      break;
      }
    case CMD_PA4_TIME:{
      write7Bit(pa4_time,8);
      break;
      }
    case CMD_RUNTIME:{
      uint64_t t = get_timestamp_cycles_safe();
      write7Bit(t,8);
      break;
      }
    case CMD_ADC_RESULT:{
      if(vaihe!=6){
        write7Bit(-1,8);
        break;
      }
      write7Bit(sumXY,8);
      nollaa(); 
      vaihe = 0;
      break;
      }
    case CMD_RESULT:{
      if(vaihe!=4){   //Ei valmis
        write7Bit(-1,8);
        break;
        }
      sumY=0;
      sumXY=0;
      sumX = 0;
      sumXX = 0;
      //t=(uint32_t)(estimate_center_time()*TCD0.CMPBCLR);
      write7Bit(vikaAika,8);
      break;
      }
    case CMD_ADC_VALUE:{
      write7Bit(last_ADC,2);
      break;
      }
    case CMD_PRINT:{
      if(last_message==1){
        write7Bit(last_ADC_idx,8);
      }else if(last_message==2){
        write7Bit(lowerLimit,8);
      }else if(last_message==3){
        write7Bit(upperLimit,8);
      }else if(last_message==4){
        write7Bit(vikaAika,8);
      }else if(last_message==5){
        write7Bit(ekaAika,8);
      }
      else if(last_message>=10){
        write7Bit(lista[last_message-10],8);
      }
      break;
      }
    case CMD_ID:{
      write7Bit(read_uid(), 8);

      break;
      }
    
    case CMD_TEMP_STOP:{      
      sumXY >>= ADC0.CTRLB;
      sumXY /= Temp_kerroin;
      sumXY += Temp_offset;

      write7Bit(sumXY, 8);
      
      VREF.CTRLA = Vref;
      ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;           //Single ended pinnin valinta
      _delay_us(100);
      nollaa();
      vaihe = 0;
      break;
      }
    default:
      break;
  }
  }


void setup(){
  switch (read_uid()){
    case 2235643953:{   //14155411284375601
      I2C_ADDR = 0x41;
      Temp_kerroin=114;
      Temp_offset=-5315;
      break;
      }
    case 3314495277:{    //14155412363226925
      I2C_ADDR = 0x42;
      Temp_kerroin=115;
      Temp_offset=4774;
      break;
      }
    default:{
      I2C_ADDR = I2C_ADDR_DEFAULT;
      break;
      }
    }
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  PORTA.DIRCLR = PIN7_bm;
  PORTA.DIRCLR = PIN1_bm;            // PA1 tuloksi (High-Z)
  PORTA.PIN1CTRL = PORT_PULLUPEN_bm; // sisäinen pull-up päälle

  cli();

  tca0_init();
  tcd0_init();
  pa4_event_init();
  evsys_init();
  adc_init();
  tcb0_init();


  sei();
  }

void loop(){
  if(get_timestamp_cycles_safe()>=palautaSync){
    PORTA.DIRCLR = PIN1_bm;            // takaisin tuloksi
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm; // pull-up uudelleen päälle
    report = false;
    palautaSync = 0xFFFFFFFFFFFFFFFF;
  }
  }
