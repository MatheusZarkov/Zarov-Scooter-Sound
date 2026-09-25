"""Extrai loops de giro constante de uma gravacao em varredura do Engine Simulator.

Uso: python3 ferramentas/extrair_camadas.py gravacoes/hayabusa_sem_marcha.wav amostras/hayabusa

Passos:
1. Rastreia a frequencia de explosao (ordem 2, 4 cilindros em 4 tempos) ancorada
   no corte de giro, onde o rpm e conhecido e estavel.
2. Para cada camada, pega o trecho da gravacao em que o rpm passa pelo alvo,
   desenrola pelo angulo do virabrequim (giro constante) e fecha num numero
   inteiro de ciclos, com crossfade da continuacao sobre o inicio.

Os tempos em TRECHOS valem para hayabusa_sem_marcha.wav. Outra gravacao precisa
de outros tempos; o espectrograma mostra onde fica cada trecho.
"""
import json
import os
import sys
import wave

import numpy as np
from scipy.signal import stft

CAMADAS = [1100, 2350, 4950, 10400]
DUR = 0.6            # segundos por loop, aproximado
XF_CICLOS = 2        # ciclos de crossfade na emenda
ORDEM = 2            # explosoes por volta: 4 cilindros, 4 tempos
ANCORA = (40.0, 11337 * ORDEM / 60)   # instante no corte de giro e frequencia da ordem 2 ali
TRECHOS = {
    'lenta': (2.5, 7.0),
    'carga_baixa': (7.4, 8.4),   # logo depois de abrir o acelerador
    'carga': (7.0, 38.0),
    'solto': (41.8, 72.3),
}


def ler(caminho):
    w = wave.open(caminho)
    sr, nch = w.getframerate(), w.getnchannels()
    x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, nch)
    return x.astype(float).mean(1) / 32768, sr


def rastrear(x, sr):
    f, t, Z = stft(x, sr, nperseg=16384, noverlap=16384 - int(0.025 * sr))
    S = np.abs(Z)
    df = f[1]

    def pico(j, f0, tol):
        c0 = int(round(f0 / df))
        lo = max(1, min(int(f0 * (1 - tol) / df), c0 - 1))
        hi = max(int(f0 * (1 + tol) / df) + 1, c0 + 2)
        i = int(np.argmax(S[lo:hi, j])) + lo
        a, b, c = np.log(S[i - 1:i + 2, j] + 1e-12)
        return (i + 0.5 * (a - c) / (a - 2 * b + c)) * df

    j0 = int(np.searchsorted(t, ANCORA[0]))
    fr = np.zeros(len(t))
    fr[j0] = pico(j0, ANCORA[1], 0.03)
    for j in range(j0 + 1, len(t)):
        fr[j] = pico(j, fr[j - 1], 0.025)
    for j in range(j0 - 1, -1, -1):
        fr[j] = pico(j, fr[j + 1], 0.025)
    return t, fr * 60 / ORDEM


def main(entrada, saida):
    os.makedirs(saida, exist_ok=True)
    x, sr = ler(entrada)
    t_rpm, rpm = rastrear(x, sr)
    ts = np.arange(len(x)) / sr
    rpm_s = np.interp(ts, t_rpm, rpm)
    ang = np.cumsum(rpm_s / 60 / sr)

    def extrair(R, trecho):
        a, b = TRECHOS[trecho]
        m = (t_rpm >= a) & (t_rpm <= b)
        tc = t_rpm[m][np.argmin(abs(rpm[m] - R))]
        ciclo = 120.0 / R
        n = max(4, int(round(DUR / ciclo)))
        L = int(round(n * ciclo * sr))
        X = int(round(XF_CICLOS * ciclo * sr))
        v0 = ang[int(tc * sr)] - n       # n ciclos = 2n voltas, centrado em tc
        alvo = v0 + np.arange(L + X) * (R / 60 / sr)
        y = np.interp(np.interp(alvo, ang, ts), ts, x)
        out = y[:L].copy()
        r = np.arange(X) / X
        out[:X] = y[:X] * r + y[L:L + X] * (1 - r)
        return out, dict(rpm=R, trecho=trecho, centro_s=round(float(tc), 2), ciclos=n)

    loops, info = {}, []
    for i, R in enumerate(CAMADAS):
        for banco in ('carga', 'solto'):
            trecho = banco
            if i == 0:
                trecho = 'carga_baixa' if banco == 'carga' else 'lenta'
            y, meta = extrair(R, trecho)
            meta.update(banco=banco, arquivo=f'hayabusa_{R}_{banco}.wav')
            loops[meta['arquivo']] = y
            info.append(meta)

    ganho = 0.9 / max(abs(y).max() for y in loops.values())
    for nome, y in loops.items():
        with wave.open(os.path.join(saida, nome), 'wb') as o:
            o.setnchannels(1)
            o.setsampwidth(2)
            o.setframerate(sr)
            o.writeframes((np.clip(y * ganho, -1, 1) * 32767).astype(np.int16).tobytes())
    json.dump(dict(sampleRate=sr, camadas=CAMADAS, ganho=ganho, loops=info),
              open(os.path.join(saida, 'camadas.json'), 'w'), indent=1)
    for d in info:
        print(d)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
