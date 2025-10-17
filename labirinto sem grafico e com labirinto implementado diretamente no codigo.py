#labirinto sem grafico e com labirinto implementado diretamente no codigo

import heapq
import random

# --------------------------------------------------------------------------
# --- ÁREA DE CONFIGURAÇÃO DO LABIRINTO ---
# --- Altere os valores aqui para gerar labirintos diferentes ---
# --------------------------------------------------------------------------
def obter_configuracao_labirinto():
    """
    Esta função centraliza todas as configurações para a GERAÇÃO do labirinto.
    A lista de "arestas" será criada aleatoriamente com base nos parâmetros abaixo.
    """
    config = {
        # ------------ MODIFIQUE AQUI ------------
        "num_vertices": 25,       # Total de pontos/salas no labirinto.
        "arestas_extras": 10,     # Conexões adicionais para criar mais caminhos e ciclos.
        "comida_inicial": 300,    # Quantidade de comida que o prisioneiro começa.
        # ----------------------------------------

        "vertice_entrada": 1,
        "vertice_saida": 25, # Será ajustado para ser igual a num_vertices.

        "parametro_percepcao": 30, # Distância para o minotauro detectar o prisioneiro.
    }

    # Ajusta a saída para ser sempre o último vértice.
    if config["num_vertices"] > 0:
        config["vertice_saida"] = config["num_vertices"]

    # Gera as arestas (caminhos) aleatoriamente.
    config["arestas"] = gerar_arestas_aleatorias(config["num_vertices"], config["arestas_extras"])
    return config

def gerar_arestas_aleatorias(num_vertices, arestas_extras, peso_max=20):
    """
    Gera uma lista de arestas aleatórias, garantindo que o grafo seja sempre conectado.
    O 'peso' da aresta representa o custo de comida para atravessá-la.
    """
    if num_vertices < 2:
        return []

    arestas = []
    bordas_existentes = set()

    # Passo 1: Garante a conectividade criando um caminho que une todos os vértices.
    visitados = {1}
    nao_visitados = set(range(2, num_vertices + 1))
    while nao_visitados:
        u = random.choice(list(visitados))
        v = random.choice(list(nao_visitados))
        peso = random.randint(5, peso_max)
        arestas.append((u, v, peso))
        bordas_existentes.add(tuple(sorted((u, v))))
        nao_visitados.remove(v)
        visitados.add(v)

    # Passo 2: Adiciona as arestas extras para criar mais complexidade.
    num_arestas_desejado = (num_vertices - 1) + arestas_extras
    max_tentativas = num_vertices * num_vertices
    tentativas = 0
    while len(arestas) < num_arestas_desejado and tentativas < max_tentativas:
        u = random.randint(1, num_vertices)
        v = random.randint(1, num_vertices)
        if u != v and tuple(sorted((u, v))) not in bordas_existentes:
            peso = random.randint(5, peso_max)
            arestas.append((u, v, peso))
            bordas_existentes.add(tuple(sorted((u, v))))
        tentativas += 1

    return arestas

# --------------------------------------------------------------------------
# --- LÓGICA DA SIMULAÇÃO ---
# --------------------------------------------------------------------------
class Grafo:
    """Representa o labirinto como um grafo."""
    def __init__(self):
        self.adj = {}
        self.vertices = set()

    def adicionar_aresta(self, u, v, peso):
        self.adj.setdefault(u, [])
        self.adj.setdefault(v, [])
        self.adj[u].append((v, peso))
        self.adj[v].append((u, peso))
        self.vertices.add(u)
        self.vertices.add(v)

    def obter_vizinhos(self, u):
        return self.adj.get(u, [])

def dijkstra(grafo, inicio, nos_proibidos=None):
    """
    Algoritmo de Dijkstra para encontrar o caminho mais curto.
    Pode receber um conjunto de 'nos_proibidos' para ignorar no cálculo do caminho.
    """
    if nos_proibidos is None:
        nos_proibidos = set()

    if inicio in nos_proibidos:
        return {v: float('inf') for v in grafo.vertices}, {v: None for v in grafo.vertices}

    distancias = {v: float('inf') for v in grafo.vertices}
    predecessores = {v: None for v in grafo.vertices}
    distancias[inicio] = 0
    pq = [(0, inicio)]
    
    while pq:
        dist, u = heapq.heappop(pq)
        if dist > distancias[u]:
            continue
        for v, peso in grafo.obter_vizinhos(u):
            if v in nos_proibidos:
                continue
            
            if distancias[u] + peso < distancias[v]:
                distancias[v] = distancias[u] + peso
                predecessores[v] = u
                heapq.heappush(pq, (distancias[v], v))
    return distancias, predecessores

def reconstruir_caminho(predecessores, inicio, fim):
    """Reconstrói o caminho a partir dos resultados de Dijkstra."""
    caminho = []
    atual = fim
    while atual is not None:
        caminho.append(atual)
        if atual == inicio: break
        atual = predecessores.get(atual)
    return caminho[::-1] if caminho and caminho[-1] == inicio else []

class Simulacao:
    """Controla toda a dinâmica e o estado da simulação."""
    def __init__(self, config):
        self.config = config
        self.grafo = Grafo()
        for u, v, peso in self.config["arestas"]:
            self.grafo.adicionar_aresta(u, v, peso)

        self.entrada = self.config["vertice_entrada"]
        self.saida = self.config["vertice_saida"]
        self.p_g = self.config["parametro_percepcao"]
        self.comida_inicial = self.config["comida_inicial"]

        pos_proibidas = {self.entrada, self.saida}
        pos_validas = [v for v in self.grafo.vertices if v not in pos_proibidas]
        self.pos_minotauro = random.choice(pos_validas) if pos_validas else list(self.grafo.vertices)[0]
        self.pos_prisioneiro = self.entrada

        self.turno_atual = 0
        self.comida_restante = self.comida_inicial
        self.minotauro_vivo = True
        self.fim_de_jogo = False
        self.status_final = "Em andamento..."
        
        self.dfs_stack = [self.pos_prisioneiro]
        self.visitados_prisioneiro = {self.pos_prisioneiro}
        self.parentes_dfs = {self.pos_prisioneiro: None}

        self.caminho_prisioneiro = [self.pos_prisioneiro]
        self.caminho_perseguicao_minotauro = []
        self.detectado = False
        self.momento_deteccao = None

    def executar_passo(self):
        """Executa um turno completo da simulação."""
        if self.fim_de_jogo: return
        self.turno_atual += 1

        # --- Movimento do Prisioneiro ---
        pos_anterior_p = self.pos_prisioneiro
        if self.dfs_stack:
            atual = self.dfs_stack[-1]
            encontrou_caminho = False
            vizinhos = self.grafo.obter_vizinhos(atual)
            random.shuffle(vizinhos)
            for vizinho, peso in vizinhos:
                if vizinho not in self.visitados_prisioneiro:
                    self.visitados_prisioneiro.add(vizinho)
                    self.parentes_dfs[vizinho] = atual
                    self.dfs_stack.append(vizinho)
                    self.pos_prisioneiro = vizinho
                    self.comida_restante -= peso
                    encontrou_caminho = True
                    break
            if not encontrou_caminho:
                if len(self.dfs_stack) > 1:
                    vertice_backtrack = self.dfs_stack.pop()
                    pai = self.parentes_dfs[vertice_backtrack]
                    peso_b = next((p for v, p in self.grafo.obter_vizinhos(vertice_backtrack) if v == pai), 1)
                    self.pos_prisioneiro = pai
                    self.comida_restante -= peso_b
                else:
                    self.fim_de_jogo = True
                    self.status_final = "MORTO: Preso em um beco sem saída."
        
        if pos_anterior_p != self.pos_prisioneiro:
            self.caminho_prisioneiro.append(self.pos_prisioneiro)
        
        if self.verificar_condicoes_termino(): return

        # --- Movimento do Minotauro ---
        # SÓ SE MOVE SE ESTIVER VIVO
        if self.minotauro_vivo:
            pos_anterior_m = self.pos_minotauro
            nos_proibidos_minotauro = {self.entrada, self.saida}
            
            dist, pred = dijkstra(self.grafo, self.pos_minotauro, nos_proibidos=nos_proibidos_minotauro)
            
            if dist.get(self.pos_prisioneiro, float('inf')) <= self.p_g:
                if not self.detectado:
                    self.detectado = True
                    self.momento_deteccao = self.turno_atual
                    self.caminho_perseguicao_minotauro.append(pos_anterior_m)

                caminho = reconstruir_caminho(pred, self.pos_minotauro, self.pos_prisioneiro)
                if len(caminho) > 2: self.pos_minotauro = caminho[2]
                elif len(caminho) > 1: self.pos_minotauro = caminho[1]
            else:
                vizinhos_m = self.grafo.obter_vizinhos(self.pos_minotauro)
                vizinhos_validos = [v for v, p in vizinhos_m if v not in nos_proibidos_minotauro]
                if vizinhos_validos:
                    self.pos_minotauro = random.choice(vizinhos_validos)
            
            if self.detectado and pos_anterior_m != self.pos_minotauro:
                 self.caminho_perseguicao_minotauro.append(self.pos_minotauro)

        self.verificar_condicoes_termino()

    def verificar_condicoes_termino(self):
        """Verifica se alguma das condições de fim de jogo foi atingida."""
        if self.fim_de_jogo: return True

        if self.pos_prisioneiro == self.saida:
            self.status_final = "SALVO: O prisioneiro escapou!"
            self.fim_de_jogo = True
        elif self.comida_restante <= 0:
            self.status_final = "MORTO: Acabou a comida."
            self.fim_de_jogo = True
        elif self.pos_prisioneiro == self.pos_minotauro and self.minotauro_vivo:
            if random.random() <= 0.99: # 1% de chance de vencer
                self.status_final = "SOBREVIVEU: Minotauro derrotado em combate!"
                self.minotauro_vivo = False # AMEAÇA ELIMINADA!
                # Imprime a mensagem do evento no momento em que ele ocorre.
                print(f"\n*** EVENTO RARO: No turno {self.turno_atual}, o prisioneiro derrotou o Minotauro! ***\n")
            else:
                self.status_final = "MORTO: Capturado e devorado pelo Minotauro."
                self.fim_de_jogo = True
        return self.fim_de_jogo

def executar_simulacao_completa():
    """Função principal que configura, executa e relata a simulação."""
    config = obter_configuracao_labirinto()
    simulacao = Simulacao(config)

    print("--- CONFIGURAÇÃO DO LABIRINTO ---")
    print(f"Vértices: {config['num_vertices']}, Arestas Extras: {config['arestas_extras']}")
    print(f"Entrada: {simulacao.entrada}, Saída: {simulacao.saida}")
    print(f"Comida Inicial do Prisioneiro: {simulacao.comida_inicial}")
    print(f"Distância de Percepção do Minotauro: {simulacao.p_g}")
    print(f"Posição Inicial do Minotauro: {simulacao.pos_minotauro}")
    print("\n--- INICIANDO SIMULAÇÃO ---\n")

    while not simulacao.fim_de_jogo:
        simulacao.executar_passo()
        
        # Constrói a string de status do turno dinamicamente
        status_turno = f"Turno {simulacao.turno_atual}: "
        status_turno += f"Prisioneiro em {simulacao.pos_prisioneiro}, "
        
        # MODIFICAÇÃO: Mostra o status do Minotauro (vivo ou morto)
        if simulacao.minotauro_vivo:
            status_turno += f"Minotauro em {simulacao.pos_minotauro}, "
        else:
            status_turno += "Minotauro: MORTO, "
            
        status_turno += f"Comida: {round(simulacao.comida_restante)}"
        if simulacao.detectado and simulacao.minotauro_vivo:
            status_turno += " [DETECTADO!]"
        
        # Só imprime o status se o jogo não tiver acabado nesse exato passo
        if not simulacao.fim_de_jogo:
            print(status_turno)

    print("\n--- FIM DA SIMULAÇÃO ---")
    print(f"\nResultado Final: {simulacao.status_final}")
    print(f"Comida Restante: {round(simulacao.comida_restante)}")
    print(f"Total de Turnos: {simulacao.turno_atual}")

    print("\nCaminho Percorrido pelo Prisioneiro:")
    print(" -> ".join(map(str, simulacao.caminho_prisioneiro)))

    if simulacao.momento_deteccao:
        print(f"\nO prisioneiro foi detectado no turno: {simulacao.momento_deteccao}")
        if simulacao.caminho_perseguicao_minotauro:
            print("Caminho do Minotauro durante a perseguição (até sua morte ou fim do jogo):")
            print(" -> ".join(map(str, simulacao.caminho_perseguicao_minotauro)))
    else:
        print("\nO Minotauro nunca detectou o prisioneiro.")

if __name__ == "__main__":
    executar_simulacao_completa()

