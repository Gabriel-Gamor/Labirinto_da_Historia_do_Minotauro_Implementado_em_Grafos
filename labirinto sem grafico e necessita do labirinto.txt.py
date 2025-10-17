# labirinto sem grafico e necessita do labirinto.txt

import heapq  # import do módulo de fila de prioridade (heap) usado no Dijkstra
import random  # import para gerar movimentos aleatórios quando necessário
import sys  # import para acessar argumentos da linha de comando e encerrar o programa
from typing import Dict, List, Tuple, Optional, Set  # anotações de tipo para legibilidade

# -------------------------
# Estruturas e utilitários
# -------------------------

class Grafo:
    # Classe que representa um grafo não direcionado com pesos nas arestas.
    def __init__(self):
        # dicionário de adjacência: chave = vértice (int), valor = lista de (vizinho, peso)
        self.adj: Dict[int, List[Tuple[int, float]]] = {}
        # conjunto de vértices presentes no grafo (útil para garantir que vértices isolados existam)
        self.vertices: Set[int] = set()

    def adicionar_aresta(self, u: int, v: int, peso: float):
        # Garante que existam listas de adjacência para u e v (cria vazias se não existirem)
        self.adj.setdefault(u, [])
        self.adj.setdefault(v, [])
        # adiciona a aresta em ambos os sentidos (grafo não direcionado)
        self.adj[u].append((v, float(peso)))
        self.adj[v].append((u, float(peso)))
        # registra os vértices no conjunto de vértices
        self.vertices.add(u)
        self.vertices.add(v)

    def obter_vizinhos(self, u: int) -> List[Tuple[int, float]]:
        # Retorna a lista de pares (vértice vizinho, peso) para o vértice u
        return self.adj.get(u, [])  # se u não existir, retorna lista vazia

def dijkstra(grafo: Grafo, inicio: int, nos_proibidos: Optional[Set[int]] = None) -> Tuple[Dict[int, float], Dict[int, Optional[int]]]:
    # Implementação do algoritmo de Dijkstra para caminhos mínimos em grafos com pesos não-negativos.
    # Retorna dois dicionários:
    #   dist[v] = distância mínima de 'inicio' até v
    #   pred[v] = predecessor de v no caminho mínimo (None se não houver predecessor)
    if nos_proibidos is None:
        nos_proibidos = set()  # se não informado, considera conjunto vazio de nós proibidos

    # inicializa todas as distâncias com infinito e predecessores com None
    dist = {v: float('inf') for v in grafo.vertices}
    pred = {v: None for v in grafo.vertices}

    # se o nó inicial estiver entre os proibidos, não há caminho válido: retorna arrays vazios/iniciais
    if inicio in nos_proibidos:
        return dist, pred

    # distância do início para si mesmo é zero
    dist[inicio] = 0.0
    # fila de prioridade (distância, vértice)
    pq = [(0.0, inicio)]

    # loop principal do Dijkstra
    while pq:
        d, u = heapq.heappop(pq)  # extrai o par com menor distância conhecida
        if d > dist[u]:
            # se esta entrada estiver desatualizada (existem entradas anteriores), ignora
            continue
        # explora vizinhos do vértice u
        for v, w in grafo.obter_vizinhos(u):
            # ignora vizinhos que são nós proibidos
            if v in nos_proibidos:
                continue
            nd = d + w  # distância candidata até v passando por u
            if nd < dist[v]:
                # se encontramos caminho menor, atualiza distância e predecessor
                dist[v] = nd
                pred[v] = u
                heapq.heappush(pq, (nd, v))  # insere nova distância na fila de prioridade
    return dist, pred

def reconstruir_caminho(pred: Dict[int, Optional[int]], inicio: int, fim: int) -> List[int]:
    # Reconstrói o caminho do vértice 'inicio' até 'fim' usando o dicionário de predecessores 'pred'.
    caminho = []
    atual = fim
    # sobe pelos predecessores até o início (ou até None)
    while atual is not None:
        caminho.append(atual)
        if atual == inicio:
            # se chegamos ao início, paramos
            break
        atual = pred.get(atual)
    caminho.reverse()  # como percorremos do fim ao início, invertemos para ordem correta
    # garante que o primeiro elemento seja o início (caso contrário, não há caminho válido)
    if caminho and caminho[0] == inicio:
        return caminho
    return []  # caminho inválido ou inexistente

# -------------------------
# Leitura de arquivo
# -------------------------

def ler_labirinto_do_arquivo(caminho: str) -> Tuple[Grafo, int, int, int, int, float, float]:
    """
    Lê um arquivo labirinto.txt e retorna:
    - grafo: objeto Grafo com as arestas lidas
    - n: número de vértices declarado no arquivo
    - m: número de arestas declarado no arquivo
    - entrada: índice do vértice de entrada
    - saida: índice do vértice de saída
    - pos_minotauro: posição inicial do Minotauro
    - p_g: parâmetro de percepção (distância em que o Minotauro detecta o prisioneiro)
    - tau: tempo máximo / quantidade de comida inicial do prisioneiro
    """
    linhas = []  # lista que armazenará as linhas significativas do arquivo (sem comentários nem vazias)
    with open(caminho, "r", encoding="utf-8") as f:
        for raw in f:
            s = raw.strip()  # remove espaços em branco nas extremidades
            if not s or s.startswith("#"):
                # pula linhas vazias e linhas que começarem com '#' (comentários)
                continue
            linhas.append(s)  # armazena linha válida

    # validação: precisa ter pelo menos 2 linhas (n e m)
    if len(linhas) < 2:
        raise ValueError("Arquivo inválido: conteúdo insuficiente.")

    idx = 0
    n = int(linhas[idx]); idx += 1  # lê número de vértices (primeira linha útil)
    m = int(linhas[idx]); idx += 1  # lê número de arestas (segunda linha útil)

    # validação simples: confirma que existem linhas suficientes para m arestas + 4 parâmetros finais
    if len(linhas) < 2 + m + 4:
        raise ValueError("Arquivo inválido: número de linhas menor que o esperado pelas arestas e parâmetros.")

    grafo = Grafo()  # cria instância do grafo

    # lê as m linhas seguintes, cada uma com: u v w
    for i in range(m):
        parts = linhas[idx].split()
        if len(parts) < 3:
            # se a linha não tiver pelo menos 3 campos, considera mal formada
            raise ValueError(f"Linha de aresta mal formada: '{linhas[idx]}'")
        u, v, w = int(parts[0]), int(parts[1]), float(parts[2])
        grafo.adicionar_aresta(u, v, w)  # adiciona aresta ao grafo
        idx += 1

    # após as arestas, lê os 4 parâmetros: entrada, saida, pos_minotauro, p_g, tau
    entrada = int(linhas[idx]); idx += 1
    saida = int(linhas[idx]); idx += 1
    pos_minotauro = int(linhas[idx]); idx += 1
    p_g = float(linhas[idx]); idx += 1
    tau = float(linhas[idx]); idx += 1

    # Certificar que todos os vértices de 1..n existem no grafo (mesmo sem arestas)
    for v in range(1, n+1):
        if v not in grafo.vertices:
            # se o vértice estiver ausente, adiciona ao conjunto e cria lista de adjacência vazia
            grafo.vertices.add(v)
            grafo.adj.setdefault(v, [])

    # Validações básicas de consistência
    if entrada == saida:
        raise ValueError("Entrada e saída devem ser diferentes.")
    if pos_minotauro in (entrada, saida):
        raise ValueError("Posição inicial do Minotauro deve ser diferente da entrada e da saída.")
    return grafo, n, m, entrada, saida, pos_minotauro, p_g, tau

# -------------------------
# Simulação
# -------------------------

class Simulacao:
    # Classe que encapsula todo o estado da simulação e a lógica de evolução por turnos.
    def __init__(self, grafo: Grafo, n: int, entrada: int, saida: int, pos_minotauro: int, p_g: float, tau: float):
        # armazena parâmetros e referências
        self.grafo = grafo
        self.n = n
        self.entrada = entrada
        self.saida = saida
        self.pos_minotauro = pos_minotauro  # posição atual do Minotauro
        self.pos_prisioneiro = entrada  # prisioneiro começa na entrada
        self.p_g = p_g  # distância de percepção do Minotauro
        self.comida_restante = float(tau)  # tau é tratada como "comida" / recurso do prisioneiro
        self.turno_atual = 0  # contador de turnos
        self.minotauro_vivo = True  # sinaliza se o Minotauro está vivo após combates
        self.fim_de_jogo = False  # sinaliza fim da simulação
        self.status_final = "Em andamento..."  # mensagem final (atualizada ao final)

        # Estratégia de exploração do prisioneiro: DFS com "novelo de lã" (stack + backtrack)
        self.dfs_stack = [self.pos_prisioneiro]  # pilha representando o novelo (começa na entrada)
        self.visitados_prisioneiro = {self.pos_prisioneiro}  # conjunto de vértices já visitados pelo prisioneiro
        self.parentes_dfs = {self.pos_prisioneiro: None}  # mapa de parentes para reconstrução se necessário
        self.caminho_prisioneiro = [self.pos_prisioneiro]  # log do caminho percorrido pelo prisioneiro (para relatório)

        # Informação do Minotauro sobre detectar e perseguir
        self.detectado = False  # se o Minotauro já detectou o prisioneiro
        self.momento_deteccao: Optional[int] = None  # turno em que a detecção ocorreu
        self.momento_alcance: Optional[int] = None  # turno em que o Minotauro alcançou o prisioneiro (se ocorreu)
        self.caminho_perseguicao_minotauro: List[int] = []  # registro do trajeto do Minotauro durante perseguição

        # Regra especificada: Minotauro não usa entrada nem saída para se mover
        self.nos_proibidos_minotauro = {self.entrada, self.saida}

    def executar_passo(self):
        # Executa um turno (passo) da simulação: primeiro o prisioneiro, depois o Minotauro, e verifica condições de término.
        if self.fim_de_jogo:
            # se o jogo já terminou, não faz nada
            return
        self.turno_atual += 1  # incrementa contador de turno

        # Movimento do prisioneiro: 1 vértice por turno; exploração tipo DFS com novelo
        pos_anterior_p = self.pos_prisioneiro  # salva posição anterior do prisioneiro para verificar mudança

        if self.dfs_stack:
            # se houver vértices na pilha, tentamos avançar (DFS)
            atual = self.dfs_stack[-1]  # topo da pilha = vértice atual
            encontrou = False  # flag que indica se encontrou vizinho não visitado
            vizinhos = list(self.grafo.obter_vizinhos(atual))  # lista de (vizinho, peso)
            random.shuffle(vizinhos)  # embaralha a ordem para diversificar a exploração em execuções diferentes
            for vizinho, peso in vizinhos:
                if vizinho not in self.visitados_prisioneiro:
                    # se o vizinho não foi visitado, prisioneiro caminha até ele
                    self.visitados_prisioneiro.add(vizinho)
                    self.parentes_dfs[vizinho] = atual
                    self.dfs_stack.append(vizinho)  # empilha novo vértice (novelo estica)
                    self.pos_prisioneiro = vizinho  # atualiza posição do prisioneiro
                    self.comida_restante -= peso  # gasta comida proporcional ao peso da aresta
                    encontrou = True
                    break  # anda apenas para um vizinho neste turno
            if not encontrou:
                # se não encontrou vizinhos não visitados -> faz backtrack (volta pelo novelo)
                if len(self.dfs_stack) > 1:
                    vertice_backtrack = self.dfs_stack.pop()  # remove o topo (estamos saindo dele)
                    novo_atual = self.dfs_stack[-1]  # novo topo após pop = vértice para o qual voltamos
                    # calcula peso entre vértice que saímos e o novo atual (para reduzir comida)
                    peso_b = next((p for v, p in self.grafo.obter_vizinhos(vertice_backtrack) if v == novo_atual), 0.0)
                    self.pos_prisioneiro = novo_atual  # atualiza posição do prisioneiro para o vértice anterior
                    self.comida_restante -= peso_b  # gasta comida para o movimento de volta
                else:
                    # se não há para onde voltar e não há novos vizinhos -> preso em beco sem saída
                    self.fim_de_jogo = True
                    self.status_final = "MORTO: Preso em beco sem saída."
        else:
            # se a pilha estiver vazia (não deveria normalmente), finaliza por falta de movimentos
            self.fim_de_jogo = True
            self.status_final = "MORTO: Sem movimentos possíveis."

        # se prisioneiro mudou de posição durante este turno, registra no caminho
        if pos_anterior_p != self.pos_prisioneiro:
            self.caminho_prisioneiro.append(self.pos_prisioneiro)

        # Verificar término imediato (prisioneiro encontrou saída)
        if self.verificar_condicoes_termino():
            # se terminou (escape), não executar movimentos do Minotauro
            return

        # Movimento do Minotauro (apenas se estiver vivo)
        if self.minotauro_vivo:
            pos_anterior_m = self.pos_minotauro  # armazena posição anterior do Minotauro (não usado explicitamente depois, só por clareza)
            # Calcula distâncias do minotauro para todos os nós ignorando entrada/saída (nós proibidos)
            dist, pred = dijkstra(self.grafo, self.pos_minotauro, nos_proibidos=self.nos_proibidos_minotauro)

            # obtém distância do minotauro até a posição atual do prisioneiro
            dist_para_prisioneiro = dist.get(self.pos_prisioneiro, float('inf'))
            if dist_para_prisioneiro <= self.p_g:
                # caso a distância esteja dentro do alcance de percepção, o Minotauro detectou o prisioneiro
                if not self.detectado:
                    # registra a primeira detecção (momento e caminho inicial)
                    self.detectado = True
                    self.momento_deteccao = self.turno_atual
                    self.caminho_perseguicao_minotauro.append(self.pos_minotauro)
                # perseguir: mover até 2 vértices ao longo do caminho mínimo em direção ao prisioneiro
                caminho = reconstruir_caminho(pred, self.pos_minotauro, self.pos_prisioneiro)
                # se o caminho estiver vazio ou não começar na posição atual, significa que não há caminho válido
                if not caminho or caminho[0] != self.pos_minotauro:
                    # caminho não encontrado (talvez prisioneiro em entrada/saída que são proibidos)
                    # então o minotauro escolhe um vizinho válido aleatório (ou fica parado se não houver)
                    vizinhos_validos = [v for v, _ in self.grafo.obter_vizinhos(self.pos_minotauro) if v not in self.nos_proibidos_minotauro]
                    if vizinhos_validos:
                        self.pos_minotauro = random.choice(vizinhos_validos)
                else:
                    # move até 2 passos pelo caminho mínimo (cada passo é mover para o próximo vértice do caminho)
                    passos = min(2, len(caminho)-1)  # máximo 2 passos, não ultrapassa o fim do caminho
                    for s in range(1, passos+1):
                        destino = caminho[s]  # próximo vértice no caminho
                        self.pos_minotauro = destino  # move o minotauro
                        # registra cada movimento da perseguição para posterior relatório
                        self.caminho_perseguicao_minotauro.append(self.pos_minotauro)
                        # se alcançou o prisioneiro, interrompe a movimentação
                        if self.pos_minotauro == self.pos_prisioneiro:
                            break
            else:
                # se não detectou, Minotauro faz movimento aleatório de 1 vértice (sem entrar na entrada/saída)
                vizinhos_validos = [ (v,p) for v,p in self.grafo.obter_vizinhos(self.pos_minotauro) if v not in self.nos_proibidos_minotauro ]
                if vizinhos_validos:
                    # escolhe aleatoriamente um par (v,p) e move para v
                    self.pos_minotauro = random.choice(vizinhos_validos)[0]

        # Depois dos movimentos, verificar fim (captura, comida, saída)
        self.verificar_condicoes_termino()

    def verificar_condicoes_termino(self) -> bool:
        # Verifica diversas condições de término e atualiza o estado. Retorna True se jogo terminou.
        if self.fim_de_jogo:
            # se já estava terminado, retorna imediatamente
            return True
        # prisioneiro chegou na saída -> salvo
        if self.pos_prisioneiro == self.saida:
            self.status_final = "SALVO: O prisioneiro escapou!"
            self.fim_de_jogo = True
            return True
        # comida acabou -> prisioneiro morre de fome
        if self.comida_restante <= 0:
            self.status_final = "MORTO: Acabou a comida."
            self.fim_de_jogo = True
            return True
        # se minotauro e prisioneiro estão no mesmo vértice e minotauro está vivo -> combate
        if self.pos_prisioneiro == self.pos_minotauro and self.minotauro_vivo:
            # chance de 1% do prisioneiro vencer (evento raro conforme enunciado)
            chance = random.random()  # valor uniformemente em [0,1)
            if chance <= 0.01:
                # prisioneiro vence o combate; minotauro morre; jogo continua (prisioneiro precisa escapar ainda)
                self.minotauro_vivo = False
                self.status_final = "SOBREVIVEU (temporario): Minotauro derrotado — precisa escapar antes da comida acabar."
                # Não encerra o jogo aqui porque prisioneiro ainda pode morrer por falta de comida ou escapar
                print(f"\n*** EVENTO RARO: No turno {self.turno_atual}, o prisioneiro derrotou o Minotauro! ***\n")
            else:
                # prisioneiro é capturado e devorado -> morte imediata e fim do jogo
                self.status_final = "MORTO: Capturado e devorado pelo Minotauro."
                self.momento_alcance = self.turno_atual  # registra o turno do alcance
                self.fim_de_jogo = True
                return True
        return self.fim_de_jogo  # se nenhuma condição de término ocorreu, retorna False

    def relatorio_final(self):
        # Gera um relatório final impresso no terminal com os resultados e estatísticas da simulação.
        print("\n--- RELATÓRIO FINAL ---")
        print(f"Resultado: {self.status_final}")
        print(f"Turnos jogados: {self.turno_atual}")
        # converte comida restante para inteiro ao exibir (para legibilidade)
        print(f"Comida restante (inteiro): {int(self.comida_restante)}")
        print("\nCaminho percorrido pelo prisioneiro:")
        # exibe o caminho do prisioneiro como sequência de vértices separados por '->'
        print(" -> ".join(map(str, self.caminho_prisioneiro)))
        if self.detectado:
            # se ocorreu detecção, imprime informações adicionais sobre a perseguição
            print(f"\nPrisioneiro detectado pelo Minotauro no turno: {self.momento_deteccao}")
            if self.momento_alcance:
                # se o Minotauro alcançou o prisioneiro, imprime o turno correspondente
                print(f"Momento em que o Minotauro alcançou o prisioneiro (se ocorreu): {self.momento_alcance}")
            if self.caminho_perseguicao_minotauro:
                print("Caminho do Minotauro durante a perseguição:")
                print(" -> ".join(map(str, self.caminho_perseguicao_minotauro)))
        else:
            # se nunca detectou, informa isso
            print("\nO Minotauro nunca detectou o prisioneiro.")

# -------------------------
# Função principal
# -------------------------

def main():
    # Função de entrada do script, considera "labirinto.txt"
    arquivo = "labirinto.txt"

    try:
        # tenta ler o labirinto do arquivo e obter parâmetros necessários
        grafo, n, m, entrada, saida, pos_minotauro, p_g, tau = ler_labirinto_do_arquivo(arquivo)
    except Exception as e:
        # em caso de erro na leitura/validação do arquivo, informa o erro e encerra com código 1
        print("Erro ao ler arquivo:", e)
        sys.exit(1)

    # imprime configuração carregada para conferência pelo usuário
    print("--- CONFIGURAÇÃO DO LABIRINTO ---")
    print(f"Vértices (n): {n}, Arestas (m): {m}")
    print(f"Entrada: {entrada}, Saída: {saida}")
    print(f"Posição inicial do Minotauro: {pos_minotauro}")
    print(f"Parâmetro de percepção p(G): {p_g}")
    print(f"Tempo máximo / comida (tau): {tau}")
    print("\n--- INICIANDO SIMULAÇÃO ---\n")

    # cria instância da simulação com os parâmetros lidos
    sim = Simulacao(grafo, n, entrada, saida, pos_minotauro, p_g, tau)

    # loop principal de execução: enquanto não for fim de jogo, executa passos
    while not sim.fim_de_jogo:
        sim.executar_passo()
        # Exibe resumo do turno (não exibe quando o jogo finaliza nesse passo - será mostrado no relatório final)
        if not sim.fim_de_jogo:
            # monta a linha de status do turno atual
            line = f"Turno {sim.turno_atual}: Prisioneiro={sim.pos_prisioneiro}, "
            if sim.minotauro_vivo:
                line += f"Minotauro={sim.pos_minotauro}, "
            else:
                line += "Minotauro=MORTO, "
            line += f"Comida={int(sim.comida_restante)}"
            # se prisioneiro foi detectado e minotauro segue vivo, marca com tag [DETECTADO]
            if sim.detectado and sim.minotauro_vivo:
                line += " [DETECTADO]"
            print(line)  # imprime resumo do turno

    # ao final do loop, imprime o relatório final detalhado
    sim.relatorio_final()

# ponto de entrada do script quando executado diretamente
if __name__ == "__main__":
    main()
