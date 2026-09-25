// Gera o pacote do motor fisico que vai embutido no banco de testes.
//   git clone --depth 1 https://github.com/HowdyMoto/engine-sim ferramentas/motor-fisico/engine-sim
//   cd ferramentas/motor-fisico && bun build entrada.ts --target=browser --format=iife --minify --outfile=motor_fisico.js
// Depois troque o conteudo do <script id="motor-fisico-src"> do HTML pelo motor_fisico.js.
import { SomCinematico } from './somCinematico';
(globalThis as any).SomCinematico = SomCinematico;
