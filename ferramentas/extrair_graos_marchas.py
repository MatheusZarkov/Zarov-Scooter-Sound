"""Grãos da Hayabusa com marchas e do corte de giro, para a síntese granular.

Uso: python3 ferramentas/extrair_graos_marchas.py

Lê gravacoes/hayabusa_marchas.wav e gravacoes/hayabusa_corte.wav e grava em
amostras/hayabusa-graos-marchas/. Os grãos sem marcha (amostras/hayabusa-graos)
continuam como estão; aqui eles só servem de referência de fase e volume, e de
reserva nas faixas de giro que a gravação com marcha não cobre.

O rpm destas gravações é rastreado por trecho: o espaçamento entre harmônicas
dá a meia ordem (rpm/120) sem ambiguidade de oitava, e cada trecho tem faixa de
giro e ponto de partida amarrados ao anterior pela razão da troca de marcha. Só
entram trechos em que o rastreio bate com o espectrograma.
"""
import json
import os
import sys
import wave

import numpy as np
from scipy.signal import butter, resample_poly, sosfiltfilt, stft

sys.path.insert(0, os.path.dirname(__file__))
from extrair_camadas import SR_SAIDA  # noqa: E402

RAIZ = os.path.join(os.path.dirname(__file__), '..')
MARCHAS = os.path.join(RAIZ, 'gravacoes/hayabusa_marchas.wav')
CORTE = os.path.join(RAIZ, 'gravacoes/hayabusa_corte.wav')
SEM = os.path.join(RAIZ, 'amostras/hayabusa-graos')
SAIDA = os.path.join(RAIZ, 'amostras/hayabusa-graos-marchas')

UP, DN = (0.66, 0.86), (1.05, 1.5)
# (inicio, fim, rpm_min, rpm_max, passo por quadro, razao de partida sobre o fim do trecho anterior)
RASTREIO_MARCHAS = [(0.0, 7.75, 1100, 1800, 0.02, None), (7.75, 10.4, 1100, 4000, 0.06, (0.9, 1.15)),
                    (10.4, 14.42, 2500, 6800, 0.03, (0.9, 1.2)), (14.5, 19.62, 3000, 6800, 0.03, UP),
                    (19.7, 29.95, 3000, 6800, 0.03, UP), (30.05, 33.02, 3000, 7200, 0.03, DN),
                    (33.1, 38.45, 1800, 5600, 0.03, (1.15, 1.45))]
RASTREIO_CORTE = [(1.9, 8.28, 10000, 11700, 0.02, None)]
# trechos usados como graos: carga = acelerando em 1a, 2a e 3a; solto = lenta e desacelerando em 3a e 2a
TRECHOS = {'carga': [(10.4, 14.40), (14.52, 19.60), (19.72, 27.0)],
           'solto': [(0.2, 7.7), (27.1, 29.93), (30.08, 33.0), (33.12, 34.7)]}
CORTE_TRECHO = (3.0, 5.5)   # corte continuo, tocado em sequencia para manter o ritmo

PASSO, DUR_MIN, CICLOS_MIN, CICLOS_LENTA, CAUDA, L_REF = 0.03, 0.12, 2, 3, 0.004, 256


def ler(p):
    w = wave.open(p)
    x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, w.getnchannels())
    return x.astype(float).mean(1) / 32768, w.getframerate()


def rastrear(x, sr, trechos):
    f, t, Z = stft(x, sr, nperseg=16384, noverlap=16384 - int(0.025 * sr))
    La = np.log(np.abs(Z) + 1e-9)
    L = La - np.array([np.convolve(c, np.ones(41) / 41, 'same') for c in La.T]).T
    df = f[1]
    rpms = np.geomspace(800, 12500, 900)
    sc = np.zeros((len(rpms), len(t)))
    for i, R in enumerate(rpms):
        fh = R / 120
        k = np.arange(1, int(2500 / fh) + 1)
        on = L[np.clip(np.round(k * fh / df).astype(int), 0, len(f) - 1)].mean(0)
        off = L[np.clip(np.round((k - 0.5) * fh / df).astype(int), 0, len(f) - 1)].mean(0)
        sc[i] = on - off
    lr = np.log(rpms)
    r = np.full(len(t), np.nan)
    ult = None
    for a, b, lo, hi, passo, razao in trechos:
        j0, j1 = np.searchsorted(t, a), np.searchsorted(t, b)
        m = int(np.ceil(np.log(1 + passo) / (lr[1] - lr[0])))
        ok = (rpms >= lo) & (rpms <= hi)
        S = np.where(ok[:, None], sc[:, j0:j1], -1e9)
        D = S[:, 0].copy()
        if razao is not None and ult is not None:
            D = np.where((rpms >= ult * razao[0]) & (rpms <= ult * razao[1]), D, -1e9)
        back = np.zeros(S.shape, int)
        for j in range(1, S.shape[1]):
            best = np.full(len(rpms), -1e18)
            arg = np.zeros(len(rpms), int)
            for dd in range(-m, m + 1):
                src = np.clip(np.arange(len(rpms)) - dd, 0, len(rpms) - 1)
                v = D[src] - 0.02 * abs(dd)
                bb = v > best
                best[bb], arg[bb] = v[bb], src[bb]
            D = best + S[:, j]
            back[:, j] = arg
        p = np.zeros(S.shape[1], int)
        p[-1] = D.argmax()
        for j in range(S.shape[1] - 1, 0, -1):
            p[j - 1] = back[p[j], j]
        r[j0:j1] = rpms[p]
        ult = rpms[p][-1]
    ok = ~np.isnan(r)
    return t[ok], r[ok]


class Fonte:
    """Gravacao a 32 kHz com rpm e angulo do virabrequim por trecho."""

    def __init__(self, caminho, rastreio):
        x, sr_in = ler(caminho)
        self.t_rpm, self.rpm = rastrear(x, sr_in, rastreio)
        self.x = resample_poly(x, SR_SAIDA, sr_in)
        self.sr = SR_SAIDA
        self.xb = sosfiltfilt(butter(4, 2000, 'lp', fs=self.sr, output='sos'), self.x)

    def trecho(self, a, b):
        i0, i1 = int(a * self.sr), int(b * self.sr)
        ts = np.arange(i0, i1) / self.sr
        rpm_s = np.interp(ts, self.t_rpm, self.rpm)
        return i0, rpm_s, np.cumsum(rpm_s / 60 / self.sr)


def forma(xb, b0, n, env=False):
    s = xb[b0:b0 + n]
    y = np.interp(np.linspace(0, len(s) - 1, L_REF), np.arange(len(s)), s)
    if env:
        y = np.abs(y)
    y = y - y.mean()
    return y / (np.linalg.norm(y) + 1e-12)


def alinhar(fonte, run, ref, env=False):
    n = run['b'][1] - run['b'][0]
    busca = max(1, n // 8)
    cs = [np.dot(forma(fonte.xb, run['b'][0] + d, n, env), ref) for d in range(-busca, busca + 1)]
    run['b'] = run['b'] + int(np.argmax(cs)) - busca
    return max(cs)


def trechos_em_runs(fonte, trechos, lenta_ate=1800):
    runs = []
    for a, b in trechos:
        i0, rpm_s, ang = fonte.trecho(a, b)
        i, ref = 0, None
        while i < len(rpm_s):
            R = rpm_s[i]
            lenta = R < lenta_ate
            if ref is None or abs(R - ref) / ref >= PASSO or lenta:
                ciclo = 120.0 / R
                n = max(CICLOS_LENTA if lenta else CICLOS_MIN, int(np.ceil(DUR_MIN / ciclo)))
                fr = np.searchsorted(ang, ang[i] + 2 * np.arange(n + 1))
                if fr[-1] + int((CAUDA + ciclo) * fonte.sr) >= len(rpm_s):
                    break
                runs.append(dict(b=fr.astype(int) + i0, rpm=float(R)))
                ref = R
                i = fr[-1] + (int(0.6 * fonte.sr) if lenta else 0)
            else:
                i += int(0.005 * fonte.sr)
    return sorted(runs, key=lambda r: r['rpm'])


def cadeia(fonte, runs, i0):
    for sentido in (1, -1):
        k = i0
        while 0 <= k + sentido < len(runs):
            ant, run = runs[k], runs[k + sentido]
            alinhar(fonte, run, forma(fonte.xb, ant['b'][0], ant['b'][1] - ant['b'][0]))
            k += sentido


def ler_sem():
    meta = json.load(open(os.path.join(SEM, 'graos.json')))
    bancos = {}
    for banco, d in meta['bancos'].items():
        a, _ = ler(os.path.join(SEM, d['arquivo']))
        bancos[banco] = (a, np.array(d['graos']))
    return bancos


def ref_sem(sem, banco, R):
    """Forma de envelope do grao sem marcha mais proximo de R, e RMS dos graos na faixa."""
    a, G = sem[banco]
    g = G[np.argmin(abs(G[:, 2] - R))]
    y = a[int(g[0]):int(g[0] + g[1])]
    y = np.interp(np.linspace(0, len(y) - 1, L_REF), np.arange(len(y)), y)
    y = np.abs(y) - np.abs(y).mean()
    return y / (np.linalg.norm(y) + 1e-12)


def rms_faixa(audio, G, lo, hi):
    sel = [audio[int(o):int(o + L)] for o, L, r in G if lo <= r <= hi]
    return np.sqrt(np.mean(np.concatenate(sel) ** 2)) if sel else None


def montar(fonte, runs):
    cauda = int(CAUDA * fonte.sr)
    partes, graos, o = [], [], 0
    for r in runs:
        b = r['b']
        partes.append(fonte.x[b[0]:b[-1] + cauda])
        for k in range(len(b) - 1):
            L = int(b[k + 1] - b[k])
            graos.append([o + int(b[k] - b[0]), L, round(120.0 * fonte.sr / L, 1)])
        o += len(partes[-1])
    return np.concatenate(partes), graos


def reserva(G_prim, G_sem):
    """Graos sem marcha nas faixas de giro que a gravacao com marcha nao cobre."""
    rp = np.sort(np.array([g[2] for g in G_prim]))
    buracos = [(0, rp[0] * 0.985)] + [(a * 1.015, b * 0.985) for a, b in zip(rp[:-1], rp[1:]) if b / a > 1.03] + [(rp[-1] * 1.015, 1e9)]
    return [[int(g[0]), int(g[1]), float(g[2])] for g in G_sem if any(lo <= g[2] <= hi for lo, hi in buracos)]


def gravar(nome, audio, sr):
    with wave.open(os.path.join(SAIDA, nome), 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes((np.clip(audio, -1, 1) * 32767).astype(np.int16).tobytes())


def main():
    os.makedirs(SAIDA, exist_ok=True)
    sem = ler_sem()
    mar = Fonte(MARCHAS, RASTREIO_MARCHAS)
    cor = Fonte(CORTE, RASTREIO_CORTE)
    meta = dict(sampleRate=SR_SAIDA, bancos={})

    faixa_ganho = {'carga': (4000, 5000), 'solto': (3600, 4800)}
    for banco in ('carga', 'solto'):
        runs = trechos_em_runs(mar, TRECHOS[banco])
        i0 = int(np.argmin([abs(r['rpm'] - 4500) for r in runs]))
        alinhar(mar, runs[i0], ref_sem(sem, banco, runs[i0]['rpm']), env=True)
        cadeia(mar, runs, i0)
        audio, G = montar(mar, runs)
        a_sem, G_sem = sem[banco]
        lo, hi = faixa_ganho[banco]
        ganho = rms_faixa(a_sem, G_sem, lo, hi) / rms_faixa(audio, G, lo, hi)
        audio = audio * ganho
        extra = reserva(G, G_sem)
        todos = sorted([g + [0] for g in G] + [g + [1] for g in extra], key=lambda g: g[2])
        nome = f'hayabusa_marchas_{banco}.wav'
        gravar(nome, audio, SR_SAIDA)
        meta['bancos'][banco] = dict(arquivo=nome, graos=todos, ganho=round(float(ganho), 3))
        rp = [g[2] for g in G]
        print(f"{banco}: {len(runs)} trechos, {len(G)} graos proprios ({min(rp):.0f}-{max(rp):.0f} rpm), "
              f"{len(extra)} da reserva sem marcha, {len(audio) / SR_SAIDA:.1f} s, ganho {ganho:.3f}")

    # corte: um trecho so, em sequencia
    i0, rpm_s, ang = cor.trecho(*CORTE_TRECHO)
    R = float(np.median(rpm_s))
    fr = np.searchsorted(ang, ang[0] + 2 * np.arange(int((ang[-1] - ang[0]) / 2) - 1))
    run = dict(b=fr.astype(int) + i0, rpm=R)
    alinhar(cor, run, ref_sem(sem, 'carga', R), env=True)
    audio, G = montar(cor, [run])
    a_sem, G_sem = sem['carga']
    ganho = rms_faixa(a_sem, G_sem, 10300, 11200) / np.sqrt(np.mean(audio ** 2))
    audio = audio * ganho
    gravar('hayabusa_corte.wav', audio, SR_SAIDA)
    meta['bancos']['corte'] = dict(arquivo='hayabusa_corte.wav', graos=[g + [0] for g in G], ganho=round(float(ganho), 3))
    print(f"corte: {len(G)} graos em sequencia, rpm mediano {R:.0f}, {len(audio) / SR_SAIDA:.1f} s, ganho {ganho:.3f}")
    json.dump(meta, open(os.path.join(SAIDA, 'graos.json'), 'w'))


if __name__ == '__main__':
    main()
