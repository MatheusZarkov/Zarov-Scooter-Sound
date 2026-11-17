"""Monitor serial do ESP32 para a bancada.

Mostra o punho e o giro do motor como barras que se mexem, numa linha so, e manda
para a placa as teclas digitadas (r, f, m, c). Linhas que nao sao de medida
(estouro, calibracao) aparecem normalmente.
Abre a porta sem mexer em DTR/RTS, para nao reiniciar a placa.
Uso: python monitor_serial.py [COM3] [--bruto]   (Ctrl+C para sair)
"""
import msvcrt
import re
import sys
import time

import serial

args = [a for a in sys.argv[1:] if not a.startswith("--")]
porta = args[0] if args else "COM3"
bruto = "--bruto" in sys.argv

MEDIDA = re.compile(r"punho\s+(\d+)%\s+\(\s*(\d+) mV\).*RPM\s+(\d+)\s+\((\w+)\)")
RPM_MAX = 11300
LARG = 20   # linha inteira com menos de 80 colunas, senao o painel quebra e o \r embaralha


def barra(frac):
    n = round(max(0.0, min(1.0, frac)) * LARG)
    return "#" * n + "." * (LARG - n)


def esperar_porta():
    while True:
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = porta, 115200, 0.05
            s.open()
            return s
        except serial.SerialException:
            print(f"Esperando a placa aparecer em {porta}... (ligue o USB)")
            time.sleep(2)


s = esperar_porta()
print(f"Conectado em {porta}. Gire o potenciometro. Teclas: r repouso, f fundo, m marchas, c calibracao. Ctrl+C sai.\n")
resto = ""
try:
    while True:
        try:
            dados = s.read(512)
        except serial.SerialException:
            print("\n\nA placa sumiu (USB desligado?). Esperando ela voltar...")
            s.close()
            s = esperar_porta()
            print("Voltou.\n")
            continue
        if dados:
            resto += dados.decode("utf-8", "replace")
            *linhas, resto = resto.split("\n")
            for linha in linhas:
                linha = linha.rstrip("\r")
                m = MEDIDA.search(linha)
                if bruto or not m:
                    if linha.strip():
                        print("\n" + linha if not bruto else linha)
                    continue
                punho, mv, rpm, fonte = int(m[1]), int(m[2]), int(m[3]), m[4]
                sys.stdout.write(
                    f"\rpunho [{barra(punho / 100)}] {punho:3d}%  "
                    f"motor [{barra(rpm / RPM_MAX)}] {rpm:5d} rpm {fonte:5s}"
                )
                sys.stdout.flush()
        while msvcrt.kbhit():
            s.write(msvcrt.getch())
except KeyboardInterrupt:
    pass
