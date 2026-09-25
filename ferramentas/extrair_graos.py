"""Extrai grãos de um ciclo do motor para a síntese granular.

Uso: python3 ferramentas/extrair_graos.py gravacoes/hayabusa_sem_marcha.wav amostras/hayabusa-graos

Um grão é um ciclo completo (2 voltas do virabrequim) tirado da gravação sem
esticar. Guarda-se um trecho curto de ciclos seguidos a cada 3% de giro, com
carga (subida) e solto (marcha lenta + descida). Na reprodução, a cada ciclo o
motor escolhe um grão gravado perto do giro atual, então quase nada precisa ser
acelerado e não há loop se repetindo.

Os trechos são alinhados em fase entre si (correlação com o vizinho de giro),
para a troca de grão não deslocar as explosões. Um erro de um intervalo entre
explosões (1/4 de ciclo) não se ouve, porque o ritmo continua regular.
"""
import json
import os
import sys
import wave

import numpy as np
from scipy.signal import butter, resample_poly, sosfiltfilt

sys.path.insert(0, os.path.dirname(__file__))
from extrair_camadas import SR_SAIDA, TRECHOS, ler, rastrear  # noqa: E402

PASSO = 0.03         # giro entre trechos guardados
DUR_MIN = 0.12       # segundos por trecho
CICLOS_MIN = 2
CICLOS_LENTA = 3     # marcha lenta: trechos mais longos, o motor passa muito tempo nela
CAUDA = 0.004        # segundos alem do ultimo ciclo, para o crossfade na troca
BANCOS = {'carga': ['carga'], 'solto': ['lenta', 'solto']}
L_REF = 256          # tamanho da forma de onda normalizada usada no alinhamento


def main(entrada, saida, arquivo_camadas):
    os.makedirs(saida, exist_ok=True)
    x, sr_in = ler(entrada)
    t_rpm, rpm = rastrear(x, sr_in)
    x = resample_poly(x, SR_SAIDA, sr_in)
    sr = SR_SAIDA
    ts = np.arange(len(x)) / sr
    rpm_s = np.interp(ts, t_rpm, rpm)
    ang = np.cumsum(rpm_s / 60 / sr)
    xb = sosfiltfilt(butter(4, 2000, 'lp', fs=sr, output='sos'), x)
    ganho = json.load(open(arquivo_camadas))['ganho']   # mesmo volume das camadas

    def trechos_do_banco(banco):
        runs = []
        for nome in BANCOS[banco]:
            a, b = TRECHOS[nome]
            i, fim = int(a * sr), int(b * sr)
            ref = None
            while i < fim:
                R = rpm_s[i]
                if ref is None or abs(R - ref) / ref >= PASSO or nome == 'lenta':
                    ciclo = 120.0 / R
                    n = max(CICLOS_LENTA if nome == 'lenta' else CICLOS_MIN, int(np.ceil(DUR_MIN / ciclo)))
                    alvo = ang[i] + 2 * np.arange(n + 1)
                    fronteiras = np.searchsorted(ang, alvo)
                    if fronteiras[-1] + int(CAUDA * sr) + int(ciclo * sr) >= fim:
                        break
                    runs.append(dict(trecho=nome, b=fronteiras.astype(int), rpm=float(R)))
                    ref = R
                    i = fronteiras[-1]
                    if nome == 'lenta':
                        i += int(0.6 * sr)   # na lenta, um trecho a cada ~0,6 s
                else:
                    i += int(0.005 * sr)
        return sorted(runs, key=lambda r: r['rpm'])

    def forma(b0, n):
        s = xb[b0:b0 + n]
        y = np.interp(np.linspace(0, len(s) - 1, L_REF), np.arange(len(s)), s)
        return (y - y.mean()) / (np.linalg.norm(y - y.mean()) + 1e-12)

    def alinhar(run, ref, env=False):
        n = run['b'][1] - run['b'][0]
        busca = max(1, n // 8)                     # +-1/8 de ciclo: meio intervalo entre explosoes
        melhor, melhor_d = -2, 0
        for d in range(-busca, busca + 1):
            y = forma(run['b'][0] + d, n)
            if env:
                y = np.abs(y); y = (y - y.mean()) / (np.linalg.norm(y - y.mean()) + 1e-12)
            c = float(np.dot(y, ref))
            if c > melhor:
                melhor, melhor_d = c, d
        run['b'] = run['b'] + melhor_d
        return melhor

    def cadeia(runs, i0):
        for sentido in (1, -1):
            k = i0
            while 0 <= k + sentido < len(runs):
                ant, run = runs[k], runs[k + sentido]
                ref = forma(ant['b'][0], ant['b'][1] - ant['b'][0])
                run['corr'] = alinhar(run, ref)
                k += sentido

    bancos = {b: trechos_do_banco(b) for b in BANCOS}
    ancora = {b: int(np.argmin([abs(r['rpm'] - 5000) for r in runs])) for b, runs in bancos.items()}
    cadeia(bancos['carga'], ancora['carga'])
    # o solto se alinha ao carga no mesmo giro pelo envelope (formas diferem entre carga e solto)
    rc = bancos['carga'][ancora['carga']]
    ref = forma(rc['b'][0], rc['b'][1] - rc['b'][0]); ref = np.abs(ref); ref = (ref - ref.mean()) / np.linalg.norm(ref - ref.mean())
    alinhar(bancos['solto'][ancora['solto']], ref, env=True)
    cadeia(bancos['solto'], ancora['solto'])

    meta = dict(sampleRate=sr, ganho=ganho, bancos={})
    cauda = int(CAUDA * sr)
    for banco, runs in bancos.items():
        partes, graos, o = [], [], 0
        for r in runs:
            b = r['b']
            seg = x[b[0]:b[-1] + cauda] * ganho
            for k in range(len(b) - 1):
                L = int(b[k + 1] - b[k])
                graos.append([o + int(b[k] - b[0]), L, round(120.0 * sr / L, 1)])  # rpm pelo proprio comprimento
            partes.append(seg)
            o += len(seg)
        audio = np.clip(np.concatenate(partes), -1, 1)
        nome = f'hayabusa_graos_{banco}.wav'
        with wave.open(os.path.join(saida, nome), 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
            w.writeframes((audio * 32767).astype(np.int16).tobytes())
        graos.sort(key=lambda g: g[2])
        corr = [r.get('corr', 1.0) for r in runs]
        meta['bancos'][banco] = dict(arquivo=nome, graos=graos)
        print(f"{banco}: {len(runs)} trechos, {len(graos)} graos, {len(audio) / sr:.1f} s, "
              f"rpm {graos[0][2]:.0f}-{graos[-1][2]:.0f}, correlacao de alinhamento mediana {np.median(corr):.2f} min {min(corr):.2f}")
    json.dump(meta, open(os.path.join(saida, 'graos.json'), 'w'))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else 'amostras/hayabusa/camadas.json')
