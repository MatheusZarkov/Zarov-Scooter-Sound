# Módulo de som de motor para scooter elétrica

Firmware para ESP32 que gera som de motor a combustão em tempo real, sincronizado
com o acelerador e a velocidade da roda de uma scooter elétrica. Saída pela entrada
P2 de uma caixinha de som, com Bluetooth desativado pelo cabo.

Este arquivo resume decisões tomadas numa conversa de planejamento. Trate como
especificação. Se algo aqui conflitar com a realidade do hardware, pergunte antes
de mudar.

## Arquivos de referência nesta pasta

- `banco-de-testes-som-scooter.html`: simulador no navegador. A lógica de controle
  dele (física, filtro, piso da roda, camadas, estouro) é a referência de
  comportamento para o firmware. Leia o `<script>` antes de começar.
- `esquematico-som-scooter.html`: esquemático da versão alimentada pelo pack.
  ATENÇÃO: está parcialmente desatualizado, ver "Hardware atual" abaixo.

## Hardware atual (versão 1, bancada e primeira instalação)

- ESP32 DevKit V1, 30 pinos, ESP32-WROOM-32.
- Alimentação por power bank no USB do ESP32. Não usa o pack da scooter na v1.
- PCM5102A (módulo roxo GY-PCM5102) na saída P2 da caixinha.
- SS49E (hall linear) no punho, com ímã de neodímio 5x2mm no punho giratório.
- A3144 (hall digital, sensor solto, não o módulo KY-003) na roda, com 4 a 6 ímãs.

### Pinagem

| Função | Pino | Detalhe |
|---|---|---|
| Punho, SS49E | GPIO34 | ADC1. SS49E alimentado em 3V3, saída direto no pino, 100nF no GND |
| Roda, A3144 | GPIO35 | Interrupção por borda. A3144 em VIN (5V). Pull-up externo 10k para 3V3 |
| I2S BCK | GPIO26 | para BCK do PCM5102A |
| I2S WS | GPIO25 | para LCK do PCM5102A |
| I2S DATA | GPIO22 | para DIN do PCM5102A |

PCM5102A: VIN no 5V, SCK no GND (PLL interno), XSMT alto, FMT baixo.

GPIO34 e GPIO35 não têm pull-up interno. Use só ADC1 (GPIO32 a 39), porque ADC2
não funciona com WiFi ativo.

### Diferenças em relação ao esquemático HTML

- Sem buck, fusível, chave e capacitores de entrada: a v1 usa power bank.
- Punho: o esquemático mostra sangria do sinal original com divisor R1/R2. A v1 usa
  SS49E próprio em 3V3, sem divisor.
- Roda: o esquemático mostra sangria do hall do motor com diodo D1. A v1 usa A3144
  próprio com pull-up para 3V3, sem diodo (A3144 é coletor aberto).
- Se um dia voltar a usar o pack: buck XL7015 (entrada até 80V, só 0,6A nominal).
  MAX98357A nunca no 12V, máximo 5,5V.

## Lógica de controle (o núcleo do projeto)

Laço de física em passo fixo, 100 a 120 Hz.

1. Leitura do punho com mediana de 5 amostras e deadband de 2 a 3%. O punho
   tremula parado e sem isso o estouro dispara sozinho.
2. Calibração do punho: gravar tensão em repouso e no fim do curso. A curva do SS49E
   é fortemente não linear (campo cai com o cubo da distância). Usar tabela de
   correção, não mapeamento linear.
3. Velocidade da roda: contar bordas no GPIO35 por interrupção, converter por
   ímãs por volta e circunferência. Timeout para zerar quando para.
4. `rpm_roda = velocidade * relacao`. Com marchas virtuais, relação por marcha e
   queda breve de giro na troca.
5. `rpm_punho` filtrado com coeficientes ASSIMÉTRICOS: sobe mais rápido do que desce
   (referência: subida 0.05, descida 0.02 a 100 Hz).
6. **Regra central: `rpm = max(rpm_roda, rpm_punho_filtrado)`.** Filtrar o punho
   ANTES do max, nunca depois. O piso da roda é instantâneo e sem filtro.
   Isso resolve sozinho: parado o punho manda, andando a roda manda, soltou o punho
   em velocidade o som desce junto com a roda real.
7. Estouro no escapamento: derivada do punho abaixo de -3.0/s com rpm acima de 5000,
   trava de 1 segundo.

## Lógica de áudio

- 4 camadas de loop gravadas em rpm fixo, cada uma tocada a `rpm / rpm_gravado`.
  Crossfade entre as duas camadas vizinhas com ganho em raiz quadrada
  (`sqrt(1-t)` e `sqrt(t)`), nunca linear.
- Dois bancos por camada, "puxando" e "solto", com crossfade pela posição do punho.
  É o timbre de carga e o som de freio motor.
- Camadas de taxa FIXA (vento, pneu, chiado) acompanham só volume, nunca pitch.
- Loops sem clique: crossfade nos pontos de loop ou loop com número inteiro de
  ciclos de explosão.

## Caminhos para o firmware

Opção A, preferida para começar: adaptar o projeto
https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32 , que já resolve síntese por
amostras, inércia, embreagem virtual e transmissão. Trocar a entrada de rádio RC
pela leitura do SS49E e do A3144. Verificar no repositório qual saída de áudio ele
usa: se for o DAC interno do ESP32 nos GPIO25 e 26, há conflito com os pinos I2S
acima, e é preciso decidir entre adaptar para I2S com PCM5102A ou reatribuir pinos.

Opção B: firmware próprio com driver I2S do ESP-IDF ou biblioteca de áudio, portando
a lógica do simulador.

## Primeiras tarefas sugeridas

1. Ler o simulador e o repositório Rc_Engine_Sound_ESP32, e propor qual caminho seguir.
2. Sketch de teste só do I2S: tom senoidal no PCM5102A, para validar a saída.
3. Sketch de leitura dos sensores com saída serial: tensão bruta do punho e
   velocidade da roda, para calibração.
4. Integrar.

## Estilo de trabalho com o usuário

- Português do Brasil. Direto, sem enrolação, sem emojis, sem travessão.
- Explicar o porquê das decisões, o usuário gosta de entender.
- Avisar quando algo puder queimar componente ou for perigoso eletricamente.
