// Teste da logica de controle no ESP32 com som no buzzer do Wokwi. Arquivo unico:
// a logica de controle.h esta embutida aqui, o projeto tem so este arquivo e o diagram.json.
//
// O buzzer so liga e desliga, entao nao toca as amostras da Hayabusa. Ele toca um trem de
// pulsos estreitos na frequencia das explosoes (4 cilindros, 4 tempos: 2 por volta, rpm/30),
// com variacao pequena de ciclo a ciclo, pulso mais largo com o punho aberto (som mais cheio),
// estalos no estouro e cortes no limitador. Serve para ouvir a logica: o giro seguindo o
// punho, a roda segurando o giro, as trocas de marcha.
//
// No Wokwi: o potenciometro deslizante faz o papel do SS49E no punho (GPIO34) e o
// potenciometro redondo escolhe a velocidade de uma roda simulada, que gera os pulsos
// do A3144 no GPIO27, ligado ao GPIO35 como se fosse o sensor de verdade.
// Na placa de verdade: SIMULAR_RODA = false e o A3144 no GPIO35 com pull-up de 10k no 3V3.
//
// Comandos pelo monitor serial (clique nele antes de digitar):
//   r  grava o punho solto (repouso)     f  grava o punho no fundo
//   m  liga/desliga marchas              c  mostra a calibracao
//   s  liga/desliga o som
#include <math.h>
#include <stdint.h>
#include "esp_timer.h"

// ================================================================ logica de controle
static inline float limitar(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ---------------------------------------------------------------- perfis
// Valores da Hayabusa granular do banco de testes.
struct Perfil {
  float idle, max;        // rpm
  float single;           // relacao unica: rpm por m/s
  float gears[4];         // rpm por m/s em cada marcha
  float up, down;         // rpm de troca para cima e para baixo
  float pop;              // rpm minimo para o estouro
  float dip;              // queda de giro momentanea na troca
  bool embreagem;         // com marcha: motor preso a roda andando
  float arranque;         // rpm em que a embreagem patina saindo
};

// sem marcha: relacao unica, rpm = max(roda, punho)
const Perfil SEM_MARCHA = {1100, 11300, 1115, {1946, 1256, 904, 703}, 9543, 2888, 6300, 0.28f, false, 0};
// com marcha: calibrado pela gravacao com marchas (troca perto de 4950, cai para ~4050)
const Perfil COM_MARCHA = {1400, 11300, 1115, {900, 738, 605, 496}, 4950, 3600, 4000, 0.08f, true, 2500};

// ---------------------------------------------------------------- punho
// SS49E: tensao proporcional ao campo. Mediana de 5 leituras, calibracao entre
// repouso e fundo, tabela de correcao (a curva do sensor nao e linear) e zona morta.
struct Punho {
  // padroes nos limites do ADC do ESP32 (le de ~140 a ~3100 mV); gravar os de verdade com r e f
  float repouso_mV = 150;    // gravar com o punho solto
  float fundo_mV = 3100;     // gravar com o punho no fim do curso
  // posicao real do punho (0 a 1) para leituras normalizadas 0, 0.1, ..., 1.
  // Linear por padrao; preencher com a calibracao do sensor de verdade.
  float tabela[11] = {0, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1};
  float zonaMorta = 0.03f;   // o punho tremula parado; sem isso o estouro dispara sozinho
  float ultimo_mV = 0;

  static float mediana5(float a[5]) {
    for (int i = 1; i < 5; i++)
      for (int j = i; j > 0 && a[j - 1] > a[j]; j--) { float t = a[j]; a[j] = a[j - 1]; a[j - 1] = t; }
    return a[2];
  }

  // leituras: 5 amostras em mV. Devolve o punho de 0 (solto) a 1 (fundo).
  float atualizar(float leituras[5]) {
    ultimo_mV = mediana5(leituras);
    float faixa = fundo_mV - repouso_mV;
    if (fabsf(faixa) < 1) return 0;
    float x = limitar((ultimo_mV - repouso_mV) / faixa, 0, 1);   // funciona com o ima nos dois sentidos
    float p = x * 10;
    int i = (int)p;
    float c = (i >= 10) ? tabela[10] : tabela[i] + (tabela[i + 1] - tabela[i]) * (p - i);
    if (c <= zonaMorta) return 0;
    return limitar((c - zonaMorta) / (1 - zonaMorta), 0, 1);
  }
};

// ---------------------------------------------------------------- roda
// A3144: um pulso por ima. Velocidade pelo periodo entre pulsos; quando os pulsos
// param, a velocidade cai junto com o tempo desde o ultimo pulso, ate zerar no timeout.
struct Roda {
  float circunferencia_m = 0.80f;   // medir na roda de verdade
  int imas = 4;
  float timeout_s = 1.0f;

  // periodo_us: entre os dois ultimos pulsos (0 se ainda nao houve dois); idade_us: desde o ultimo
  float velocidade(uint32_t periodo_us, uint32_t idade_us) const {
    if (periodo_us == 0 || idade_us > timeout_s * 1e6f) return 0;
    float p = (idade_us > periodo_us) ? (float)idade_us : (float)periodo_us;
    return circunferencia_m / imas / (p * 1e-6f);   // m/s
  }
};

// ---------------------------------------------------------------- motor
struct Motor {
  bool marchas = false;
  Perfil P = SEM_MARCHA;
  float rpmPunho = 1100, rpmRoda = 1100, rpm = 1100;
  int marcha = 0;
  float dipTroca = 0, punhoAnterior = 0, travaEstouro = 0;
  bool estouro = false;               // verdadeiro so no passo em que dispara

  void escolherMarchas(bool ligado) {
    marchas = ligado;
    P = ligado ? COM_MARCHA : SEM_MARCHA;
    marcha = 0;
  }

  // dt em segundos (passo fixo de 100 Hz), punho de 0 a 1, v em m/s
  void passo(float dt, float punho, float v) {
    if (marchas) {
      float r = v * P.gears[marcha];
      if (r > P.up && marcha < 3) { marcha++; dipTroca = 1; }
      else if (r < P.down && marcha > 0) { marcha--; }
      rpmRoda = v * P.gears[marcha];
    } else {
      marcha = 0;
      rpmRoda = v * P.single;
    }
    dipTroca *= powf(0.02f, dt);
    rpmRoda = fmaxf(P.idle, rpmRoda * (1 - P.dip * dipTroca));

    // punho filtrado ANTES do max, com subida mais rapida que a descida
    float alvo = P.idle + punho * (P.max - P.idle);
    float k = (alvo > rpmPunho) ? 0.05f : 0.02f;
    rpmPunho += (alvo - rpmPunho) * fminf(1, k * dt * 100);

    if (marchas && P.embreagem) {
      // parado o punho gira livre; andando o motor fica preso a roda e a embreagem
      // patina no giro de saida ate a roda alcancar
      rpm = (v < 0.05f) ? rpmPunho : fmaxf(rpmRoda, fminf(rpmPunho, P.arranque));
    } else {
      rpm = fmaxf(rpmRoda, rpmPunho);   // regra central
    }
    rpm = limitar(rpm, P.idle, P.max);

    float derivada = (punho - punhoAnterior) / dt;
    travaEstouro -= dt;
    estouro = derivada < -3.0f && rpm > P.pop && travaEstouro <= 0;
    if (estouro) travaEstouro = 1.0f;
    punhoAnterior = punho;
  }

  const char* fonte() const { return rpmRoda >= rpmPunho ? "roda" : "punho"; }
};

// ================================================================ hardware

// true no Wokwi; false na placa de verdade, com o A3144 no GPIO35
const bool SIMULAR_RODA = true;

const int PINO_PUNHO = 34;     // SS49E (ADC1)
const int PINO_RODA = 35;      // A3144, sem pull-up interno: o de 10k e externo
const int PINO_SIM_RODA = 27;  // so no Wokwi: gera os pulsos da roda simulada
const int PINO_SIM_VEL = 32;   // so no Wokwi: potenciometro da velocidade simulada
const int PINO_BUZZER = 21;    // so no Wokwi: buzzer
const float VEL_SIM_MAX_KMH = 45;
const float TICK_S = 0.0001f;  // timer de 100 us: roda simulada e som

Punho punho;
Roda roda;
Motor motor;

volatile uint32_t ultimoPulso = 0, periodoRoda = 0;

void IRAM_ATTR pulsoRoda() {
  uint32_t t = (uint32_t)esp_timer_get_time();
  uint32_t p = t - ultimoPulso;
  if (p < 2000) return;   // ignora repique: nem a 60 km/h os imas passam a menos de 2 ms
  periodoRoda = (ultimoPulso == 0) ? 0 : p;
  ultimoPulso = t;
}

// ---------------------------------------------------------------- som
// Escrito pelo laco de 100 Hz, lido pelo timer
volatile bool somLigado = true;
volatile float somFreq = 1100 / 30.0f;   // explosoes por segundo
volatile float somLargura = 0.1f;        // fracao do ciclo com o pino alto
volatile float somVariacao = 0.05f;      // variacao de periodo de um ciclo para o outro
volatile bool somMudo = false;           // corte do limitador
volatile int somEstouro = 0;             // ticks restantes de estalos

static uint32_t semente = 2463534242u;
static inline uint32_t sortear() { semente ^= semente << 13; semente ^= semente >> 17; semente ^= semente << 5; return semente; }
static inline float aleatorio() { return (sortear() >> 8) * (1.0f / 16777216.0f); }

volatile float freqSimulada = 0;   // pulsos da roda por segundo

void tick(void*) {
  if (SIMULAR_RODA) {
    static float fase = 0.5f;
    float f = freqSimulada;
    if (f < 0.2f) { fase = 0.5f; digitalWrite(PINO_SIM_RODA, HIGH); }   // parado: proximo ima so na volta
    else {
      fase += f * TICK_S;
      if (fase >= 1) fase -= 1;
      digitalWrite(PINO_SIM_RODA, fase < 0.2f ? LOW : HIGH);   // o A3144 puxa para baixo com o ima
    }
  }

  if (!somLigado) { digitalWrite(PINO_BUZZER, LOW); return; }
  static float fase = 0, freqCiclo = 0;
  static int estalo = 0;
  if (freqCiclo == 0) freqCiclo = somFreq;
  fase += freqCiclo * TICK_S;
  if (fase >= 1) {   // nova explosao: periodo levemente diferente, como num motor de verdade
    fase -= 1;
    freqCiclo = somFreq * (1 + (aleatorio() - 0.5f) * somVariacao);
  }
  bool nivel = !somMudo && fase < somLargura;

  // estouro: rajadas curtas de ruido em momentos sorteados
  if (somEstouro > 0) {
    somEstouro = somEstouro - 1;
    if (estalo > 0) { estalo--; nivel = sortear() & 1; }
    else if (aleatorio() < 0.0025f) estalo = 30 + sortear() % 90;   // 3 a 12 ms
  } else estalo = 0;

  digitalWrite(PINO_BUZZER, nivel ? HIGH : LOW);
}

void atualizarSom(float p) {
  const Perfil &P = motor.P;
  float giro = limitar((motor.rpm - P.idle) / (P.max - P.idle), 0, 1);
  float f = motor.rpm / 30;

  // limitador: com o punho no fundo e o giro no teto, corta a cada 40 ms
  static int passoCorte = 0;
  bool corte = p > 0.9f && motor.rpm >= P.max - 30;
  passoCorte = corte ? passoCorte + 1 : 0;
  bool mudo = corte && (passoCorte / 4) % 2 == 1;
  if (corte && !mudo) f *= 0.97f;

  somFreq = f;
  somLargura = 0.08f + 0.27f * p;           // punho aberto: pulso mais largo, som mais cheio
  somVariacao = 0.08f - 0.06f * giro;       // lenta mais irregular
  somMudo = mudo;
  if (motor.estouro) somEstouro = 4000;     // 400 ms de estalos
}

// ---------------------------------------------------------------- leitura e serial
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
  pinMode(PINO_BUZZER, OUTPUT);
  digitalWrite(PINO_BUZZER, LOW);
  if (SIMULAR_RODA) {
    pinMode(PINO_SIM_RODA, OUTPUT);
    digitalWrite(PINO_SIM_RODA, HIGH);
  }
  esp_timer_create_args_t args = {};
  args.callback = tick;
  args.name = "tick";
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, 100);
  Serial.println("\nTeste de controle do som. Comandos: r repouso, f fundo, m marchas, c calibracao, s som");
  mostrarCalibracao();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') { lerPunho(); punho.repouso_mV = punho.ultimo_mV; mostrarCalibracao(); }
    if (c == 'f') { lerPunho(); punho.fundo_mV = punho.ultimo_mV; mostrarCalibracao(); }
    if (c == 'm') { motor.escolherMarchas(!motor.marchas); Serial.printf("marchas %s\n", motor.marchas ? "ligadas" : "desligadas"); }
    if (c == 'c') mostrarCalibracao();
    if (c == 's') { somLigado = !somLigado; Serial.printf("som %s\n", somLigado ? "ligado" : "desligado"); }
  }

  // laco de fisica em passo fixo de 100 Hz
  static uint32_t proximo = 0, passos = 0;
  uint32_t agora = micros();
  if ((int32_t)(agora - proximo) < 0) return;
  proximo = ((int32_t)(agora - proximo) > 50000) ? agora + 10000 : proximo + 10000;

  if (SIMULAR_RODA) {
    float kmhSim = analogRead(PINO_SIM_VEL) / 4095.0f * VEL_SIM_MAX_KMH;
    freqSimulada = kmhSim / 3.6f / roda.circunferencia_m * roda.imas;
  }

  float p = lerPunho();
  noInterrupts();
  uint32_t ultimo = ultimoPulso, periodo = periodoRoda;
  interrupts();
  uint32_t idade = (ultimo == 0) ? 0xFFFFFFFF : (uint32_t)esp_timer_get_time() - ultimo;
  float v = roda.velocidade(periodo, idade);

  motor.passo(0.01f, p, v);
  atualizarSom(p);
  if (motor.estouro) Serial.println(">>> ESTOURO no escapamento");

  if (++passos % 20 == 0) {   // 5 linhas por segundo
    char marcha[16];
    if (motor.marchas) snprintf(marcha, sizeof marcha, "%da", motor.marcha + 1); else snprintf(marcha, sizeof marcha, "unica");
    Serial.printf("punho %3.0f%% (%4.0f mV) | roda %5.1f km/h | rpm punho %5.0f roda %5.0f | RPM %5.0f (%s) | marcha %s\n",
                  p * 100, punho.ultimo_mV, v * 3.6f, motor.rpmPunho, motor.rpmRoda, motor.rpm, motor.fonte(), marcha);
  }
}
