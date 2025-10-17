#labirinto com grafico e com labirinto implementado diretamente no codigo

import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import random
import math

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
        if dist > distancias[u]: continue
        for v, peso in grafo.obter_vizinhos(u):
            if v in nos_proibidos: continue
            if distancias[u] + peso < distancias[v]:
                distancias[v] = distancias[u] + peso
                predecessores[v] = u
                heapq.heappush(pq, (distancias[v], v))
    return distancias, predecessores

def reconstruir_caminho(predecessores, inicio, fim):
    caminho = []
    atual = fim
    while atual is not None:
        caminho.append(atual)
        if atual == inicio: break
        atual = predecessores.get(atual)
    return caminho[::-1] if caminho and caminho[-1] == inicio else []

class Simulacao:
    def __init__(self, config):
        self.config = config
        self.resetar()

    def resetar(self):
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
        self.comida_restante = self.comida_inicial
        self.minotauro_vivo = True
        self.fim_de_jogo = False
        self.status_final = "Em andamento..."
        self.dfs_stack = [self.pos_prisioneiro]
        self.visitados_prisioneiro = {self.pos_prisioneiro}
        self.parentes_dfs = {self.pos_prisioneiro: None}

    def executar_passo(self):
        if self.fim_de_jogo: return None, None
        
        pos_anterior_p = self.pos_prisioneiro
        if self.dfs_stack:
            atual = self.dfs_stack[-1]
            encontrou_caminho = False
            for vizinho, peso in sorted(self.grafo.obter_vizinhos(atual), key=lambda x: random.random()):
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
                    self.status_final = "MORTO: Preso em um beco sem saída."
                    self.fim_de_jogo = True
        
        movimento_p = (pos_anterior_p, self.pos_prisioneiro)
        if self.verificar_condicoes_termino(): return movimento_p, None

        pos_anterior_m = self.pos_minotauro
        if self.minotauro_vivo:
            nos_proibidos_minotauro = {self.entrada, self.saida}
            dist, pred = dijkstra(self.grafo, self.pos_minotauro, nos_proibidos=nos_proibidos_minotauro)
            if dist.get(self.pos_prisioneiro, float('inf')) <= self.p_g:
                caminho = reconstruir_caminho(pred, self.pos_minotauro, self.pos_prisioneiro)
                if len(caminho) > 2: self.pos_minotauro = caminho[2]
                elif len(caminho) > 1: self.pos_minotauro = caminho[1]
            else:
                vizinhos_m = self.grafo.obter_vizinhos(self.pos_minotauro)
                vizinhos_validos = [v for v, p in vizinhos_m if v not in nos_proibidos_minotauro]
                if vizinhos_validos: self.pos_minotauro = random.choice(vizinhos_validos)
        
        movimento_m = (pos_anterior_m, self.pos_minotauro)
        self.verificar_condicoes_termino()
        return movimento_p, movimento_m

    def verificar_condicoes_termino(self):
        if self.fim_de_jogo: return True
        if self.pos_prisioneiro == self.saida:
            self.status_final = "SALVO: O prisioneiro escapou!"
            self.fim_de_jogo = True
        elif self.comida_restante <= 0:
            self.status_final = "MORTO: Acabou a comida."
            self.fim_de_jogo = True
        elif self.pos_prisioneiro == self.pos_minotauro and self.minotauro_vivo:
            if random.random() <= 0.01: # 1% de chance de vencer
                self.status_final = "SOBREVIVEU: Minotauro derrotado!"
                self.minotauro_vivo = False # AMEAÇA ELIMINADA!
            else:
                self.status_final = "MORTO: Capturado pelo Minotauro."
                self.fim_de_jogo = True
        return self.fim_de_jogo

# --------------------------------------------------------------------------
# --- INTERFACE GRÁFICA (GUI) ---
# --------------------------------------------------------------------------
class LabirintoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Simulador do Labirinto do Minotauro")
        self.root.geometry("1000x800")

        self.simulacao_rodando = False
        self.node_radius = 15
        # --- CORREÇÃO 1: INICIALIZAR A VARIÁVEL DE CONTROLE DO 'after' ---
        self._after_id = None 

        control_frame = ttk.Frame(root, padding="10")
        control_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas = tk.Canvas(root, bg="#f0f0f0")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.start_button = ttk.Button(control_frame, text="Iniciar Simulação", command=self.iniciar_simulacao)
        self.start_button.pack(side=tk.LEFT, padx=5)
        self.reset_button = ttk.Button(control_frame, text="Gerar Novo Labirinto", command=self.gerar_novo_labirinto)
        self.reset_button.pack(side=tk.LEFT, padx=5)

        ttk.Label(control_frame, text="Velocidade:").pack(side=tk.LEFT, padx=(10, 0))
        self.speed_scale = ttk.Scale(control_frame, from_=50, to=1000, orient=tk.HORIZONTAL)
        self.speed_scale.set(500)
        self.speed_scale.pack(side=tk.LEFT, padx=5)

        self.info_var = tk.StringVar(value="Status: Pressione 'Iniciar'")
        ttk.Label(control_frame, textvariable=self.info_var, font=("Helvetica", 10)).pack(side=tk.RIGHT, padx=10)

        self.gerar_novo_labirinto()
        self.root.bind("<Configure>", lambda e: self.redesenhar_tudo())

    def gerar_novo_labirinto(self):
        self.simulacao_rodando = False
        # --- CORREÇÃO 2: VERIFICAR SE O 'after' EXISTE ANTES DE CANCELAR ---
        if self._after_id:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        
        self.simulacao = Simulacao(obter_configuracao_labirinto())
        self.preparar_visualizacao()
        self.start_button.config(state=tk.NORMAL)
        self.reset_button.config(state=tk.NORMAL)

    def preparar_visualizacao(self):
        self.gerar_layout_grafo()
        self.desenhar_estado_inicial()

    def gerar_layout_grafo(self):
        self.node_coords = {}
        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 700
        center_x, center_y = width / 2, height / 2
        radius = min(center_x, center_y) * 0.85
        nodes = sorted(list(self.simulacao.grafo.vertices))
        if not nodes: return
        angle_step = 2 * math.pi / len(nodes)
        for i, node in enumerate(nodes):
            angle = i * angle_step - math.pi / 2
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self.node_coords[node] = (x, y)

    def desenhar_estado_inicial(self):
        self.canvas.delete("all")
        if not self.node_coords: return
        for u, neighbors in self.simulacao.grafo.adj.items():
            for v, peso in neighbors:
                if u < v:
                    x1, y1 = self.node_coords[u]
                    x2, y2 = self.node_coords[v]
                    self.canvas.create_line(x1, y1, x2, y2, fill="gray", width=1)
                    self.canvas.create_text((x1+x2)/2, (y1+y2)/2, text=str(peso), fill="darkgreen")
        for node, (x, y) in self.node_coords.items():
            fill_color = "white"
            if node == self.simulacao.entrada: fill_color = "palegreen"
            if node == self.simulacao.saida: fill_color = "plum"
            self.canvas.create_oval(x-self.node_radius, y-self.node_radius, x+self.node_radius, y+self.node_radius, fill=fill_color, outline="black", width=2)
            self.canvas.create_text(x, y, text=str(node), font=("Helvetica", 10, "bold"))
        px, py = self.node_coords[self.simulacao.pos_prisioneiro]
        self.prisioneiro_obj = self.canvas.create_oval(px-8, py-8, px+8, py+8, fill="blue", outline="white", width=2)
        mx, my = self.node_coords[self.simulacao.pos_minotauro]
        self.minotauro_obj = self.canvas.create_oval(mx-10, my-10, mx+10, my+10, fill="red", outline="white", width=2)
        self.atualizar_info()

    def redesenhar_tudo(self):
        self.gerar_layout_grafo()
        self.desenhar_estado_inicial()

    def iniciar_simulacao(self):
        if not self.simulacao_rodando:
            self.simulacao_rodando = True
            self.start_button.config(state=tk.DISABLED)
            self.reset_button.config(state=tk.DISABLED)
            self.executar_proximo_passo()

    def executar_proximo_passo(self):
        if not self.simulacao_rodando: return
        
        era_vivo = self.simulacao.minotauro_vivo
        mov_p, mov_m = self.simulacao.executar_passo()
        
        if era_vivo and not self.simulacao.minotauro_vivo:
            messagebox.showinfo("Evento Raro!", "O prisioneiro derrotou o Minotauro em combate!")
            self.canvas.delete(self.minotauro_obj)

        self.atualizar_visualizacao(mov_p, mov_m)
        self.atualizar_info()

        if self.simulacao.fim_de_jogo:
            self.simulacao_rodando = False
            self.reset_button.config(state=tk.NORMAL)
        else:
            delay = int(1050 - self.speed_scale.get())
            self._after_id = self.root.after(delay, self.executar_proximo_passo)

    def atualizar_visualizacao(self, mov_p, mov_m):
        if mov_p and mov_p[1] in self.node_coords:
            x2, y2 = self.node_coords[mov_p[1]]
            self.canvas.coords(self.prisioneiro_obj, x2-8, y2-8, x2+8, y2+8)
        if self.simulacao.minotauro_vivo and mov_m and mov_m[1] in self.node_coords:
            x2, y2 = self.node_coords[mov_m[1]]
            self.canvas.coords(self.minotauro_obj, x2-10, y2-10, x2+10, y2+10)
        
        if hasattr(self, 'prisioneiro_obj'): self.canvas.tag_raise(self.prisioneiro_obj)
        if hasattr(self, 'minotauro_obj') and self.simulacao.minotauro_vivo:
            self.canvas.tag_raise(self.minotauro_obj)

    def atualizar_info(self):
        comida = round(self.simulacao.comida_restante)
        status = self.simulacao.status_final
        
        minotauro_status = f"Minotauro: {self.simulacao.pos_minotauro}" if self.simulacao.minotauro_vivo else "Minotauro: MORTO"
        
        self.info_var.set(f"Status: {status} | Comida: {comida} | Prisioneiro: {self.simulacao.pos_prisioneiro} | {minotauro_status}")

if __name__ == "__main__":
    root = tk.Tk()
    app = LabirintoGUI(root)
    root.mainloop()

