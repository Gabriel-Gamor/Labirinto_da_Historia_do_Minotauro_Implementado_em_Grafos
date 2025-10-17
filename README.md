

Como funciona o documento labirinto.txt:
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
