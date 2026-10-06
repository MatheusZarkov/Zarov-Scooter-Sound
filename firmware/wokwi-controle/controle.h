// Logica de controle do som: punho, roda e rpm. C++ puro, sem Arduino, para rodar
// igual no ESP32, no Wokwi e no teste do PC (teste_controle.cpp).
// Referencia de comportamento: banco-de-testes-som-scooter.html e CLAUDE.md.
#pragma once
#include <math.h>
#include <stdint.h>

namespace controle {

inline float limitar(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

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
  float repouso_mV = 0;      // gravar com o punho solto
  float fundo_mV = 3300;     // gravar com o punho no fim do curso
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

}  // namespace controle
