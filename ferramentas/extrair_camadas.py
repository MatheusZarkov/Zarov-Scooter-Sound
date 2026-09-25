"""Extrai loops de giro constante de uma gravacao em varredura do Engine Simulator.

Uso: python3 ferramentas/extrair_camadas.py gravacoes/hayabusa_sem_marcha.wav amostras/hayabusa

Passos:
1. Rastreia a frequencia de explosao (ordem 2, 4 cilindros em 4 tempos) ancorada
   no corte de giro, onde o rpm e conhecido e estavel.
2. Para cada camada, pega o trecho da gravacao em que o rpm passa mais perto do
   alvo, desenrola pelo angulo do virabrequim (giro constante) e fecha num numero
   inteiro de ciclos, com crossfade da continuacao sobre o inicio.

Camadas densas (12% de giro entre vizinhas) porque amostra gravada acelerada
desloca as ressonancias do escapamento: com 4 camadas o som ficava fino no meio
da faixa.

Os tempos em TRECHOS valem para hayabusa_sem_marcha.wav. Outra gravacao precisa
de outros tempos; o espectrograma mostra onde fica cada trecho.
"""
import json
import os
import sys
import wave

import numpy as np
from scipy.signal import resample_poly, stft

N_CAMADAS = 21
RPM_MIN, RPM_MAX = 1100, 11000
CAMADAS = [int(round(RPM_MIN * (RPM_MAX / RPM_MIN) ** (k / (N_CAMADAS - 1)))) for k in range(N_CAMADAS)]
SR_SAIDA = 32000
DUR = 0.35           # segundos por loop, aproximado
MIN_CICLOS = 4
XF_CICLOS = 1        # ciclos de crossfade na emenda
ORDEM = 2            # explosoes por volta: 4 cilindros, 4 tempos
ANCORA = (40.0, 11337 * ORDEM / 60)   # instante no corte de giro e frequencia da ordem 2 ali
TRECHOS = {
    'lenta': (2.5, 7.0),
    'carga': (7.4, 38.0),    # comeca logo depois de abrir o acelerador
    'solto': (41.8, 72.3),
}
FONTES = {'carga': ['carga'], 'solto': ['lenta', 'solto']}


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
    x, sr_in = ler(entrada)
    t_rpm, rpm = rastrear(x, sr_in)
    x = resample_poly(x, SR_SAIDA, sr_in)
    sr = SR_SAIDA
    ts = np.arange(len(x)) / sr
    rpm_s = np.interp(ts, t_rpm, rpm)
    ang = np.cumsum(rpm_s / 60 / sr)

    def extrair(R, banco):
        ciclo = 120.0 / R
        n = max(MIN_CICLOS, int(round(DUR / ciclo)))
        meia = (n + XF_CICLOS) * ciclo / 2 + 0.05
        melhor = None
        for trecho in FONTES[banco]:
            a, b = TRECHOS[trecho]
            m = (t_rpm >= a + meia) & (t_rpm <= b - meia)
            if not m.any():
                continue
            k = np.argmin(abs(rpm[m] - R))
            erro = abs(rpm[m][k] - R)
            if melhor is None or erro < melhor[0]:
                melhor = (erro, t_rpm[m][k], trecho)
        _, tc, trecho = melhor
        L = int(round(n * ciclo * sr))
        X = int(round(XF_CICLOS * ciclo * sr))
        v0 = ang[int(tc * sr)] - n       # n ciclos = 2n voltas, centrado em tc
        alvo = v0 + np.arange(L + X) * (R / 60 / sr)
        src = np.interp(alvo, ang, ts)
        y = np.interp(src, ts, x)
        out = y[:L].copy()
        r = np.arange(X) / X
        out[:X] = y[:X] * r + y[L:L + X] * (1 - r)
        orig = rpm_s[int(src[0] * sr):int(src[-1] * sr) + 1]
        return out, dict(rpm=R, trecho=trecho, centro_s=round(float(tc), 2), ciclos=n,
                         rpm_gravado=[int(orig.min()), int(orig.max())])

    loops, info = {}, []
    for R in CAMADAS:
        for banco in ('carga', 'solto'):
            y, meta = extrair(R, banco)
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
