# Labirinto do Minotauro

## Descrição
O projeto simula o **Labirinto do Minotauro**, onde um **prisioneiro** tenta escapar de um labirinto representado como um **grafo ponderado G = (V, E)**.  
O **Minotauro** se move dentro do mesmo labirinto e tenta capturar o prisioneiro.

---

## Estrutura
O projeto possui quatro versões do código:

| Arquivo | Descrição |
|----------|------------|
| `labirinto com grafico e com labirinto implementado diretamente no codigo.py` | Versão com interface gráfica, labirinto definido no próprio código. |
| `labirinto sem grafico e com labirinto implementado diretamente no codigo.py` | Versão em modo texto, labirinto definido no próprio código. |
| `labirinto com grafico e necessita do labirinto.txt.py` | Versão com interface gráfica, lê o labirinto de um arquivo `labirinto.txt`. |
| `labirinto sem grafico e necessita do labirinto.txt.py` | Versão em modo texto, lê o labirinto de um arquivo `labirinto.txt`. |

---

## Formato do arquivo `labirinto.txt`
```
|  Linha  |  Exemplo  |                         Significado                             |
| ------- | --------- | --------------------------------------------------------------- |
| 1       | `int`     | Número total de vértices do labirinto                           |
| 2       | `int = n` | Número total de arestas                                         |
| 3 até n | `u v w`   | Lista de arestas (caminhos) com pesos                           |
| n+1     | `int`     | Vértice de entrada (onde o prisioneiro começa)                  |
| n+2     | `int`     | Vértice de saída (onde deve escapar)                            |
| n+3     | `int`     | Posição inicial do Minotauro                                    |
| n+4     | `int`     | Parâmetro de percepção do Minotauro — alcance olfativo/auditivo |
| n+5     | `int`     | Tempo máximo / comida inicial do prisioneiro                    |
```

---

## Como executar
### Versão sem gráfico:
```bash
python "labirinto sem grafico e necessita do labirinto.txt.py"
```

### Versão com gráfico:
```bash
python "labirinto com grafico e necessita do labirinto.txt.py"
```

Certifique-se de ter o arquivo `labirinto.txt` no mesmo diretório.

---

## Exemplo de `labirinto.txt`
```
20
30
1 2 4
1 3 6
2 4 7
3 5 5
4 5 3
5 6 4
6 7 6
7 8 5
8 9 3
9 10 5
10 11 4
11 12 6
12 13 7
13 14 4
14 15 5
15 16 6
16 17 4
17 18 5
18 19 6
19 20 5
2 6 8
3 7 9
4 8 6
5 9 7
6 10 8
7 11 9
8 12 7
9 13 8
10 14 6
11 15 7
1
20
10
25
120
```

---

## Funcionamento básico
- O **prisioneiro** tenta chegar do vértice de entrada até o de saída antes da comida acabar.  
- O **Minotauro** persegue o prisioneiro quando está dentro do alcance `p(G)`.  
- O jogo termina quando:
  - o prisioneiro escapa,  
  - a comida acaba, ou  
  - o Minotauro o alcança.
