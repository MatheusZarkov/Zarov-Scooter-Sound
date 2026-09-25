# Circuito do módulo de som

Diagrama de ligação para o módulo de som de motor em scooter elétrica, com ESP32.
Duas saídas de áudio alternativas: alto-falante próprio ou entrada P2 da caixinha.

```mermaid
flowchart TD

    subgraph POT["Potência"]
        BAT["Pack da scooter<br/>36V ou 48V"]
        FUS["Fusível 2A<br/>+ chave de ignição"]
        FILT["470uF eletrolítico<br/>+ 100nF cerâmico"]
        B12["Buck step-down<br/>saída 12V 3A"]
        B5["Buck step-down<br/>saída 5V 2A"]
    end

    subgraph ENT["Entradas de sinal"]
        PUN["Punho hall<br/>0,8V a 4,2V"]
        MOT["Sensor hall do motor<br/>onda quadrada 5V"]
        FRE["Manete de freio<br/>contato seco"]
        D1["Divisor 10k / 22k<br/>+ 10k em série"]
        D2["Divisor 10k / 22k"]
    end

    subgraph CER["Cérebro"]
        ESP["ESP32<br/>240MHz, I2S nativo"]
    end

    subgraph AUD["Áudio, escolha um caminho"]
        MAX["MAX98357A<br/>amplificador I2S 3W"]
        SPK["Alto-falante 4 ohm<br/>em caixa fechada"]
        PCM["PCM5102A<br/>DAC I2S"]
        CX["Caixinha pela entrada P2<br/>nível de linha"]
    end

    CTRL["Controladora do motor"]

    BAT --> FUS --> FILT
    FILT --> B12
    FILT --> B5
    B5 -->|"5V no pino VIN"| ESP
    B12 -->|"alimentação do amplificador"| MAX

    CTRL -.->|"sangra o fio de sinal"| PUN
    CTRL -.->|"sangra um dos três fios hall"| MOT

    PUN --> D1 --> ESP
    MOT --> D2 --> ESP
    FRE -->|"pull-up interno"| ESP

    ESP -->|"I2S: BCK, WS, DATA"| MAX
    ESP -->|"I2S: BCK, WS, DATA"| PCM
    MAX --> SPK
    PCM -->|"cabo P2 curto e blindado"| CX

    GND["Terra comum<br/>ponto estrela único"]
    BAT --- GND
    CTRL --- GND
    ESP --- GND
    MAX --- GND
```

## Pinos do ESP32

| Função | Pino | Observação |
|---|---|---|
| Punho, leitura analógica | GPIO34 | ADC1, só entrada, sem pull-up interno |
| Pulso do hall do motor | GPIO35 | ADC1, interrupção por borda |
| Manete de freio | GPIO27 | pull-up interno, ativo em nível baixo |
| I2S bit clock | GPIO26 | |
| I2S word select | GPIO25 | |
| I2S data out | GPIO22 | |
| Potenciômetro de volume | GPIO32 | opcional, ADC1 |

Use apenas pinos do ADC1, que são GPIO32 a GPIO39. O ADC2 fica inutilizável
quando o WiFi está ativo, e vários projetos usam WiFi para a página de configuração.

## O que não pode dar errado

**Divisor de tensão obrigatório.** O ADC do ESP32 lê até 3,3V. Com 10k em série
e 22k para o terra, um sinal de 4,2V chega em 2,89V. Margem suficiente sem
perder resolução.

**Resistor de 10k em série no fio do punho.** Se esse fio encostar no positivo
ou no terra, a controladora interpreta como acelerador no fundo ou como falha
de sensor. O resistor transforma um curto em uma leitura errada inofensiva.

**Não alimente o ESP32 pelo 5V do punho.** Essa linha é a referência de tensão
da controladora e é fraca. Buck separado, direto do pack.

**Terra comum em ponto estrela.** Um único ponto de junção. Terra em malha,
com o motor puxando corrente alta, injeta ruído audível direto no amplificador.

**Filtro na entrada do buck.** A controladora de BLDC chaveia em alguns kHz e
joga isso de volta na linha. Sem o eletrolítico de 470uF, você ouve o motor
elétrico chiando por dentro do som do motor a combustão.

**Opcional, mas recomendado:** optoacoplador no sinal do hall do motor, no lugar
do divisor. Isola o ESP32 da controladora por completo. Custa um 4N25 e dois
resistores.

## Se o motor for sensorless

Sem hall interno, use um A3144 apoiado no garfo e quatro a seis ímãs de
neodímio nos raios ou no disco de freio. Não use um ímã só: com um pulso por
volta a leitura fica lenta e granulada em velocidade de caminhada, exatamente
onde você mais precisa de resolução.
