// Teste da logica no PC: g++ -O2 -o /tmp/teste teste_controle.cpp && /tmp/teste
#include <stdio.h>
#include "controle.h"

using namespace controle;
static int falhas = 0;
#define CONFERE(cond, ...) do { if (cond) printf("ok    "); else { printf("FALHA "); falhas++; } printf(__VA_ARGS__); printf("\n"); } while (0)

int main() {
  const float dt = 0.01f;

  // punho: mediana descarta pico isolado, zona morta e calibracao invertida
  {
    Punho p; p.repouso_mV = 1650; p.fundo_mV = 2650;
    float a[5] = {1650, 1652, 3300, 1648, 1651};
    CONFERE(p.atualizar(a) == 0, "pico isolado de 3,3 V com o punho solto continua em 0");
    float b[5] = {2150, 2150, 2150, 2150, 2150};
    float meio = p.atualizar(b);
    CONFERE(meio > 0.45f && meio < 0.52f, "meio curso da %.3f", meio);
    float c[5] = {1670, 1670, 1670, 1670, 1670};
    CONFERE(p.atualizar(c) == 0, "2%% de curso cai na zona morta");
    Punho q; q.repouso_mV = 1650; q.fundo_mV = 700;   // ima com o outro polo: tensao desce
    float d[5] = {700, 700, 700, 700, 700};
    CONFERE(q.atualizar(d) == 1, "ima invertido no fundo da 1");
  }

  // roda: velocidade pelo periodo e queda suave quando os pulsos param
  {
    Roda r;   // 0,8 m, 4 imas: 1 pulso a cada 0,2 m
    float v = r.velocidade(20000, 5000);
    CONFERE(fabsf(v - 10) < 0.01f, "pulso a cada 20 ms = %.2f m/s (36 km/h)", v);
    float v2 = r.velocidade(20000, 100000);
    CONFERE(v2 < v && v2 > 1.9f, "sem pulso ha 100 ms a velocidade cai para %.2f m/s", v2);
    CONFERE(r.velocidade(20000, 1100000) == 0, "sem pulso ha 1,1 s zera");
    CONFERE(r.velocidade(0, 0) == 0, "sem pulsos ainda: parado");
  }

  // sem marcha: parado, punho no fundo sobe ate o teto
  {
    Motor m;
    for (int i = 0; i < 300; i++) m.passo(dt, 1, 0);
    CONFERE(m.rpm > 11200, "parado com punho no fundo: %.0f rpm em 3 s", m.rpm);
    CONFERE(m.estouro == false, "sem estouro enquanto segura");
    m.passo(dt, 0, 0);
    CONFERE(m.estouro, "soltou o punho de uma vez no alto: estouro");
    m.passo(dt, 1, 0); m.passo(dt, 0, 0);
    CONFERE(!m.estouro, "trava de 1 s segura o segundo estouro");
    // soltou andando: o som desce junto com a roda, nao cai para a lenta
    Motor n;
    for (int i = 0; i < 100; i++) n.passo(dt, 0, 8.0f);
    CONFERE(fabsf(n.rpm - 8.0f * 1115) < 1 && n.fonte()[0] == 'r', "solto a 8 m/s: %.0f rpm vindo da %s", n.rpm, n.fonte());
  }

  // filtro assimetrico: sobe mais rapido que desce
  {
    Motor m;
    int subir = 0, descer = 0;
    while (m.rpm < 6000) { m.passo(dt, 1, 0); subir++; }
    while (m.rpm > 6000 - 0.5f * (6000 - 1100) && descer < 10000) { m.passo(dt, 0, 0); descer++; }
    CONFERE(descer > subir, "subir ate 6000 levou %d passos; cair metade disso levou %d", subir, descer);
  }

  // com marcha: arrancada, trocas perto de 4950 e reducoes soltando
  {
    Motor m; m.escolherMarchas(true);
    float v = 0; int trocas = 0, ultima = 0; float rpmAntes = 0, rpmDepois = 0;
    for (int i = 0; i < 2000; i++) {
      v += 0.6f * dt;                         // acelera ate 12 m/s em 20 s
      float antes = m.rpm;
      m.passo(dt, 1, v);
      if (m.marcha != ultima) { trocas++; if (trocas == 1) { rpmAntes = antes; rpmDepois = m.rpm; } ultima = m.marcha; }
      if (i == 50) CONFERE(fabsf(m.rpm - 2500) < 1, "saindo: embreagem patina a %.0f rpm", m.rpm);
    }
    CONFERE(trocas == 3 && m.marcha == 3, "acelerando: %d trocas, terminou na %da", trocas, m.marcha + 1);
    CONFERE(rpmAntes > 4900 && rpmDepois < 4300, "primeira troca: %.0f -> %.0f rpm", rpmAntes, rpmDepois);
    int reducoes = 0;
    for (int i = 0; i < 2000; i++) { v = fmaxf(0, v - 0.5f * dt); int a = m.marcha; m.passo(dt, 0, v); if (m.marcha < a) reducoes++; }
    CONFERE(reducoes == 3 && m.marcha == 0, "soltando ate parar: %d reducoes, voltou para a 1a", reducoes);
    for (int i = 0; i < 300; i++) m.passo(dt, 1, 0);
    CONFERE(m.rpm > 11200, "parado com marcha: punho gira livre ate %.0f rpm", m.rpm);
  }

  printf("\n%s (%d falhas)\n", falhas ? "HA FALHAS" : "tudo certo", falhas);
  return falhas ? 1 : 0;
}
