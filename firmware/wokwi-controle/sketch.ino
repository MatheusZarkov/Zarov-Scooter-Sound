// Teste da logica de controle no ESP32 (Wokwi ou placa de verdade).
// Mostra no monitor serial o punho, a velocidade da roda e o rpm que o som tocaria.
//
// No Wokwi: o potenciometro deslizante faz o papel do SS49E no punho (GPIO34) e o
// potenciometro redondo escolhe a velocidade de uma roda simulada, que gera os pulsos
// do A3144 no GPIO27, ligado ao GPIO35 como se fosse o sensor de verdade.
// Na placa de verdade: SIMULAR_RODA 0 e o A3144 no GPIO35 com pull-up de 10k no 3V3.
//
// Comandos pelo monitor serial:
//   r  grava o punho solto (repouso)     f  grava o punho no fundo
//   m  liga/desliga marchas              c  mostra a calibracao
#include "controle.h"
#include "esp_timer.h"

#define SIMULAR_RODA 1

const int PINO_PUNHO = 34;     // SS49E (ADC1)
const int PINO_RODA = 35;      // A3144, sem pull-up interno: o de 10k e externo
const int PINO_SIM_RODA = 27;  // so no Wokwi: gera os pulsos da roda simulada
const int PINO_SIM_VEL = 32;   // so no Wokwi: potenciometro da velocidade simulada
const float VEL_SIM_MAX_KMH = 45;

controle::Punho punho;
controle::Roda roda;
controle::Motor motor;

volatile uint32_t ultimoPulso = 0, periodoRoda = 0;
portMUX_TYPE trava = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pulsoRoda() {
  uint32_t t = (uint32_t)esp_timer_get_time();
  uint32_t p = t - ultimoPulso;
  if (p < 2000) return;   // ignora repique: nem a 60 km/h os imas passam a menos de 2 ms
  portENTER_CRITICAL_ISR(&trava);
  periodoRoda = (ultimoPulso == 0) ? 0 : p;
  ultimoPulso = t;
  portEXIT_CRITICAL_ISR(&trava);
}

#if SIMULAR_RODA
volatile float freqSimulada = 0;   // pulsos por segundo
void tickRodaSimulada(void*) {
  static float fase = 0;
  float f = freqSimulada;
  if (f < 0.2f) { digitalWrite(PINO_SIM_RODA, HIGH); return; }
  fase += f * 0.0002f;              // chamado a cada 200 us
  if (fase >= 1) fase -= 1;
  digitalWrite(PINO_SIM_RODA, fase < 0.2f ? LOW : HIGH);   // o A3144 puxa para baixo com o ima
}
#endif

float lerPunho() {
  float a[5];
  for (int i = 0; i < 5; i++) a[i] = analogReadMilliVolts(PINO_PUNHO);
  return punho.atualizar(a);
}

void mostrarCalibracao() {
  Serial.printf("calibracao: repouso %.0f mV, fundo %.0f mV, zona morta %.0f%%\n",
                punho.repouso_mV, punho.fundo_mV, punho.zonaMorta * 100);
}

void setup() {
  Serial.begin(115200);
  pinMode(PINO_RODA, INPUT);
  attachInterrupt(digitalPinToInterrupt(PINO_RODA), pulsoRoda, FALLING);
#if SIMULAR_RODA
  pinMode(PINO_SIM_RODA, OUTPUT);
  digitalWrite(PINO_SIM_RODA, HIGH);
  esp_timer_create_args_t args = {};
  args.callback = tickRodaSimulada;
  args.name = "roda_sim";
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, 200);
#endif
  Serial.println("\nTeste de controle do som. Comandos: r repouso, f fundo, m marchas, c calibracao");
  mostrarCalibracao();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') { lerPunho(); punho.repouso_mV = punho.ultimo_mV; mostrarCalibracao(); }
    if (c == 'f') { lerPunho(); punho.fundo_mV = punho.ultimo_mV; mostrarCalibracao(); }
    if (c == 'm') { motor.escolherMarchas(!motor.marchas); Serial.printf("marchas %s\n", motor.marchas ? "ligadas" : "desligadas"); }
    if (c == 'c') mostrarCalibracao();
  }

  // laco de fisica em passo fixo de 100 Hz
  static uint32_t proximo = 0, passos = 0;
  uint32_t agora = micros();
  if ((int32_t)(agora - proximo) < 0) return;
  proximo = ((int32_t)(agora - proximo) > 50000) ? agora + 10000 : proximo + 10000;

#if SIMULAR_RODA
  float kmhSim = analogRead(PINO_SIM_VEL) / 4095.0f * VEL_SIM_MAX_KMH;
  freqSimulada = kmhSim / 3.6f / roda.circunferencia_m * roda.imas;
#endif

  float p = lerPunho();
  portENTER_CRITICAL(&trava);
  uint32_t ultimo = ultimoPulso, periodo = periodoRoda;
  portEXIT_CRITICAL(&trava);
  uint32_t idade = (ultimo == 0) ? 0xFFFFFFFF : (uint32_t)esp_timer_get_time() - ultimo;
  float v = roda.velocidade(periodo, idade);

  motor.passo(0.01f, p, v);
  if (motor.estouro) Serial.println(">>> ESTOURO no escapamento");

  if (++passos % 20 == 0) {   // 5 linhas por segundo
    char marcha[16];
    if (motor.marchas) snprintf(marcha, sizeof marcha, "%da", motor.marcha + 1); else snprintf(marcha, sizeof marcha, "unica");
    Serial.printf("punho %3.0f%% (%4.0f mV) | roda %5.1f km/h | rpm punho %5.0f roda %5.0f | RPM %5.0f (%s) | marcha %s\n",
                  p * 100, punho.ultimo_mV, v * 3.6f, motor.rpmPunho, motor.rpmRoda, motor.rpm, motor.fonte(), marcha);
  }
}
