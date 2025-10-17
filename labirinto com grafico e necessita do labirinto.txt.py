#labirinto com grafico e necessita do labirinto.txt

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import heapq
import random
import math
import sys
from typing import Dict, List, Tuple, Optional, Set

# -------------------------
# Leitura do arquivo
# -------------------------
def ler_labirinto_arquivo(caminho: str):
    linhas = []
    with open(caminho, "r", encoding="utf-8") as f:
        for raw in f:
            s = raw.strip()
            if not s: 
                continue
            if s.startswith("#"):
                continue
            linhas.append(s)

    if len(linhas) < 2:
        raise ValueError("Arquivo inválido: conteúdo insuficiente.")

    idx = 0
    try:
        n = int(linhas[idx]); idx += 1
        m = int(linhas[idx]); idx += 1
    except Exception as e:
        raise ValueError("Erro ao ler n ou m: " + str(e))

    if len(linhas) < 2 + m + 4:
        raise ValueError("Arquivo inválido: número de linhas menor que o esperado pelas arestas e parâmetros.")

    arestas = []
    for i in range(m):
        parts = linhas[idx].split()
        if len(parts) < 3:
            raise ValueError(f"Linha de aresta mal formada: '{linhas[idx]}'")
        u, v, w = int(parts[0]), int(parts[1]), float(parts[2])
        arestas.append((u, v, float(w)))
        idx += 1

    try:
        entrada = int(linhas[idx]); idx += 1
        saida = int(linhas[idx]); idx += 1
        pos_minotauro = int(linhas[idx]); idx += 1
        p_g = float(linhas[idx]); idx += 1
        tau = float(linhas[idx]); idx += 1
    except Exception as e:
        raise ValueError("Erro ao ler parâmetros finais: " + str(e))

    # validações simples
    if entrada == saida:
        raise ValueError("Entrada e saída devem ser diferentes.")
    if pos_minotauro in (entrada, saida):
        raise ValueError("Posição inicial do Minotauro deve ser diferente da entrada e da saída.")
    if not (1 <= entrada <= n and 1 <= saida <= n and 1 <= pos_minotauro <= n):
        raise ValueError("Entrada/saída/pos_minotauro devem estar no intervalo 1..n.")

    config = {
        "num_vertices": n,
        "m": m,
        "arestas": arestas,
        "vertice_entrada": entrada,
        "vertice_saida": saida,
        "pos_minotauro": pos_minotauro,
        "parametro_percepcao": p_g,
        "comida_inicial": tau
    }
    return config

# -------------------------
# Estruturas do grafo e Dijkstra
# -------------------------
class Grafo:
    def __init__(self):
        self.adj: Dict[int, List[Tuple[int, float]]] = {}
        self.vertices: Set[int] = set()

    def adicionar_aresta(self, u: int, v: int, peso: float):
        self.adj.setdefault(u, [])
        self.adj.setdefault(v, [])
        self.adj[u].append((v, float(peso)))
        self.adj[v].append((u, float(peso)))
        self.vertices.add(u)
        self.vertices.add(v)

    def obter_vizinhos(self, u: int) -> List[Tuple[int, float]]:
        return self.adj.get(u, [])

def dijkstra(grafo: Grafo, inicio: int, nos_proibidos: Optional[Set[int]] = None):
    if nos_proibidos is None:
        nos_proibidos = set()
    if inicio in nos_proibidos:
        return {v: float('inf') for v in grafo.vertices}, {v: None for v in grafo.vertices}
    dist = {v: float('inf') for v in grafo.vertices}
    pred = {v: None for v in grafo.vertices}
    dist[inicio] = 0.0
    pq = [(0.0, inicio)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        for v, w in grafo.obter_vizinhos(u):
            if v in nos_proibidos: 
                continue
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                pred[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, pred

def reconstruir_caminho(pred: Dict[int, Optional[int]], inicio: int, fim: int) -> List[int]:
    caminho = []
    atual = fim
    while atual is not None:
        caminho.append(atual)
        if atual == inicio:
            break
        atual = pred.get(atual)
    caminho.reverse()
    if caminho and caminho[0] == inicio:
        return caminho
    return []

# -------------------------
# Simulação (adaptada para usar config lido do arquivo)
# -------------------------
class Simulacao:
    def __init__(self, config: dict):
        self.config = config
        self.resetar()

    def resetar(self):
        # monta grafo
        self.grafo = Grafo()
        for u, v, peso in self.config["arestas"]:
            self.grafo.adicionar_aresta(u, v, peso)
        # garante que todos os nós 1..n existam (mesmo sem arestas)
        for v in range(1, self.config["num_vertices"] + 1):
            if v not in self.grafo.vertices:
                self.grafo.vertices.add(v)
                self.grafo.adj.setdefault(v, [])

        self.entrada = int(self.config["vertice_entrada"])
        self.saida = int(self.config["vertice_saida"])
        self.pos_minotauro = int(self.config.get("pos_minotauro", random.choice([v for v in self.grafo.vertices if v not in {self.entrada, self.saida}])))
        self.pos_prisioneiro = self.entrada
        self.p_g = float(self.config["parametro_percepcao"])
        self.comida_inicial = float(self.config["comida_inicial"])
        self.comida_restante = float(self.comida_inicial)
        self.minotauro_vivo = True
        self.fim_de_jogo = False
        self.status_final = "Em andamento..."
        # DFS-like exploration pelo prisioneiro (novelo de lã)
        self.dfs_stack = [self.pos_prisioneiro]
        self.visitados_prisioneiro = {self.pos_prisioneiro}
        self.parentes_dfs = {self.pos_prisioneiro: None}
        self.caminho_prisioneiro = [self.pos_prisioneiro]
        # perseguição
        self.detectado = False
        self.momento_deteccao: Optional[int] = None
        self.momento_alcance: Optional[int] = None
        self.caminho_perseguicao_minotauro: List[int] = []
        # Nos proibidos do minotauro (não usa entrada/saída)
        self.nos_proibidos_minotauro = {self.entrada, self.saida}

    def executar_passo(self):
        if self.fim_de_jogo:
            return None, None

        # Turno do prisioneiro (1 vértice)
        pos_anterior_p = self.pos_prisioneiro
        if self.dfs_stack:
            atual = self.dfs_stack[-1]
            encontrou = False
            vizinhos = list(self.grafo.obter_vizinhos(atual))
            random.shuffle(vizinhos)
            for vizinho, peso in vizinhos:
                if vizinho not in self.visitados_prisioneiro:
                    self.visitados_prisioneiro.add(vizinho)
                    self.parentes_dfs[vizinho] = atual
                    self.dfs_stack.append(vizinho)
                    self.pos_prisioneiro = vizinho
                    self.comida_restante -= peso
                    encontrou = True
                    break
            if not encontrou:
                # backtrack
                if len(self.dfs_stack) > 1:
                    vertice_backtrack = self.dfs_stack.pop()
                    novo_atual = self.dfs_stack[-1]
                    peso_back = next((p for v,p in self.grafo.obter_vizinhos(vertice_backtrack) if v == novo_atual), 0.0)
                    self.pos_prisioneiro = novo_atual
                    self.comida_restante -= peso_back
                else:
                    # Sem para onde ir
                    self.status_final = "MORTO: Preso em beco sem saída."
                    self.fim_de_jogo = True
        else:
            self.fim_de_jogo = True
            self.status_final = "MORTO: Sem movimentos."

        if pos_anterior_p != self.pos_prisioneiro:
            self.caminho_prisioneiro.append(self.pos_prisioneiro)

        # verifica se prisioneiro saiu
        if self.verificar_condicoes_termino():
            return (pos_anterior_p, self.pos_prisioneiro), None

        # Turno do Minotauro (pode mover 1 ou 2 vértices se detectou)
        pos_anterior_m = self.pos_minotauro
        if self.minotauro_vivo:
            dist, pred = dijkstra(self.grafo, self.pos_minotauro, nos_proibidos=self.nos_proibidos_minotauro)
            dist_para_p = dist.get(self.pos_prisioneiro, float('inf'))
            if dist_para_p <= self.p_g:
                # detectou: marca momento se ainda não marcado
                if not self.detectado:
                    self.detectado = True
                    # registramos o primeiro nó do minotauro na trilha de perseguição
                    self.caminho_perseguicao_minotauro.append(self.pos_minotauro)
                    self.momento_deteccao = getattr(self, "turno_atual", None)
                caminho = reconstruir_caminho(pred, self.pos_minotauro, self.pos_prisioneiro)
                if caminho and caminho[0] == self.pos_minotauro:
                    passos = min(2, max(0, len(caminho)-1))
                    for s in range(1, passos+1):
                        destino = caminho[s]
                        self.pos_minotauro = destino
                        self.caminho_perseguicao_minotauro.append(self.pos_minotauro)
                        if self.pos_minotauro == self.pos_prisioneiro:
                            break
                else:
                    # sem caminho: andar aleatoriamente para vizinho válido
                    viz_validos = [v for v,p in self.grafo.obter_vizinhos(self.pos_minotauro) if v not in self.nos_proibidos_minotauro]
                    if viz_validos:
                        self.pos_minotauro = random.choice(viz_validos)
            else:
                # nao detectou -> move 1 vértice aleatório (não usa entrada/saída)
                viz_validos = [ (v,p) for v,p in self.grafo.obter_vizinhos(self.pos_minotauro) if v not in self.nos_proibidos_minotauro ]
                if viz_validos:
                    self.pos_minotauro = random.choice(viz_validos)[0]

        # Verifica condições finais depois dos movimentos
        self.verificar_condicoes_termino()
        return (pos_anterior_p, self.pos_prisioneiro), (pos_anterior_m, self.pos_minotauro)

    def verificar_condicoes_termino(self):
        # set turno atual (opcional para registrar momento de detecção)
        self.turno_atual = getattr(self, "turno_atual", 0) + 1
        setattr(self, "turno_atual", self.turno_atual)

        if self.fim_de_jogo:
            return True
        if self.pos_prisioneiro == self.saida:
            self.status_final = "SALVO: O prisioneiro escapou!"
            self.fim_de_jogo = True
            return True
        if self.comida_restante <= 0:
            self.status_final = "MORTO: Acabou a comida."
            self.fim_de_jogo = True
            return True
        if self.pos_prisioneiro == self.pos_minotauro and self.minotauro_vivo:
            if random.random() <= 0.01:
                # prisioneiro derrota minotauro (evento raro)
                self.minotauro_vivo = False
                self.status_final = "SOBREVIVEU (temporário): Minotauro derrotado — precisa escapar antes da comida acabar."
                # não finaliza imediatamente (ainda precisa escapar)
                print(f"\n*** EVENTO RARO: No turno {self.turno_atual}, o prisioneiro derrotou o Minotauro! ***\n")
            else:
                self.status_final = "MORTO: Capturado e devorado pelo Minotauro."
                self.momento_alcance = self.turno_atual
                self.fim_de_jogo = True
                return True
        return self.fim_de_jogo

    def relatorio_final(self):
        report_lines = []
        report_lines.append("\n--- RELATÓRIO FINAL ---")
        report_lines.append(f"Resultado: {self.status_final}")
        report_lines.append(f"Turnos (aprox): {getattr(self, 'turno_atual', 0)}")
        report_lines.append(f"Comida restante (inteiro): {int(self.comida_restante)}")
        report_lines.append("\nCaminho do prisioneiro:")
        report_lines.append(" -> ".join(map(str, self.caminho_prisioneiro)))
        if self.detectado:
            report_lines.append(f"\nPrisioneiro detectado no turno: {self.momento_deteccao}")
            if self.momento_alcance:
                report_lines.append(f"Minotauro alcançou no turno: {self.momento_alcance}")
            if self.caminho_perseguicao_minotauro:
                report_lines.append("Caminho do Minotauro durante perseguição:")
                report_lines.append(" -> ".join(map(str, self.caminho_perseguicao_minotauro)))
        else:
            report_lines.append("\nO Minotauro nunca detectou o prisioneiro.")
        return "\n".join(report_lines)

# -------------------------
# GUI (Tkinter)
# -------------------------
class LabirintoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Simulador do Labirinto do Minotauro (arquivo)")
        self.root.geometry("1000x800")

        self.simulacao_rodando = False
        self.node_radius = 15
        self._after_id = None
        self.current_file = "labirinto.txt"  # padrão

        # frames
        control_frame = ttk.Frame(root, padding="6")
        control_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas = tk.Canvas(root, bg="#f7f7f7")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # controles
        self.open_button = ttk.Button(control_frame, text="Abrir arquivo...", command=self.abrir_arquivo)
        self.open_button.pack(side=tk.LEFT, padx=4)
        self.start_button = ttk.Button(control_frame, text="Iniciar Simulação", command=self.iniciar_simulacao)
        self.start_button.pack(side=tk.LEFT, padx=4)
        self.reset_button = ttk.Button(control_frame, text="Recarregar Labirinto", command=self.gerar_novo_labirinto)
        self.reset_button.pack(side=tk.LEFT, padx=4)

        ttk.Label(control_frame, text="Velocidade:").pack(side=tk.LEFT, padx=(8,0))
        self.speed_scale = ttk.Scale(control_frame, from_=50, to=1000, orient=tk.HORIZONTAL)
        self.speed_scale.set(500)
        self.speed_scale.pack(side=tk.LEFT, padx=6)

        self.info_var = tk.StringVar(value="Status: carregando arquivo...")
        ttk.Label(control_frame, textvariable=self.info_var).pack(side=tk.RIGHT, padx=8)

        # tenta carregar o arquivo padrão (labirinto.txt)
        try:
            config = ler_labirinto_arquivo(self.current_file)
            self.config = config
        except Exception as e:
            # se falhar, gera um aviso e cria um labirinto vazio temporário
            messagebox.showwarning("Aviso", f"Falha ao carregar '{self.current_file}': {e}\nUse 'Abrir arquivo...' para escolher um arquivo válido.")
            # cria configuração mínima vazia para evitar erros na GUI
            self.config = {
                "num_vertices": 0, "m": 0, "arestas": [], "vertice_entrada": 1,
                "vertice_saida": 1, "pos_minotauro": 1, "parametro_percepcao": 0, "comida_inicial": 0
            }

        # prepara simulação a partir da config carregada
        self.simulacao = Simulacao(self.config)
        self.preparar_visualizacao()
        self.root.bind("<Configure>", lambda e: self.redesenhar_tudo())

    def abrir_arquivo(self):
        filename = filedialog.askopenfilename(title="Abrir labirinto", filetypes=[("Text files","*.txt"),("All files","*.*")])
        if not filename:
            return
        try:
            config = ler_labirinto_arquivo(filename)
        except Exception as e:
            messagebox.showerror("Erro", f"Não foi possível ler o arquivo: {e}")
            return
        self.current_file = filename
        self.config = config
        self.gerar_novo_labirinto()
        self.info_var.set(f"Arquivo: {filename}")

    def gerar_novo_labirinto(self):
        # cancela loop atual se existir
        self.simulacao_rodando = False
        if self._after_id:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        # recria simulação a partir da configuração atual (proveniente do arquivo)
        self.simulacao = Simulacao(self.config)
        self.preparar_visualizacao()
        self.start_button.config(state=tk.NORMAL)
        self.reset_button.config(state=tk.NORMAL)

    def preparar_visualizacao(self):
        self.gerar_layout_grafo()
        self.desenhar_estado_inicial()

    def gerar_layout_grafo(self):
        self.node_coords = {}
        width = self.canvas.winfo_width() or 900
        height = self.canvas.winfo_height() or 700
        center_x, center_y = width / 2, height / 2
        radius = min(center_x, center_y) * 0.78
        nodes = sorted(list(self.simulacao.grafo.vertices))
        if not nodes:
            return
        angle_step = 2 * math.pi / len(nodes)
        for i, node in enumerate(nodes):
            angle = i * angle_step - math.pi/2
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self.node_coords[node] = (x, y)

    def desenhar_estado_inicial(self):
        self.canvas.delete("all")
        if not getattr(self, "node_coords", None):
            return
        # arestas (apenas uma vez por par)
        drawn = set()
        for u, neighbors in self.simulacao.grafo.adj.items():
            for v, peso in neighbors:
                if (v,u) in drawn or (u,v) in drawn:
                    continue
                drawn.add((u,v))
                if u in self.node_coords and v in self.node_coords:
                    x1,y1 = self.node_coords[u]
                    x2,y2 = self.node_coords[v]
                    self.canvas.create_line(x1,y1,x2,y2, fill="gray", width=1)
                    # peso no meio
                    self.canvas.create_text((x1+x2)/2, (y1+y2)/2, text=str(int(peso)), fill="darkgreen", font=("Helvetica",8))
        # nós
        for node, (x,y) in self.node_coords.items():
            fill_color = "white"
            if node == self.simulacao.entrada: fill_color = "palegreen"
            if node == self.simulacao.saida: fill_color = "plum"
            self.canvas.create_oval(x-self.node_radius, y-self.node_radius, x+self.node_radius, y+self.node_radius,
                                    fill=fill_color, outline="black", width=2)
            self.canvas.create_text(x, y, text=str(node), font=("Helvetica", 10, "bold"))
        # prisioneiro e minotauro
        if self.simulacao.pos_prisioneiro in self.node_coords:
            px,py = self.node_coords[self.simulacao.pos_prisioneiro]
            self.prisioneiro_obj = self.canvas.create_oval(px-8, py-8, px+8, py+8, fill="blue", outline="white", width=2)
        else:
            self.prisioneiro_obj = None
        if self.simulacao.pos_minotauro in self.node_coords and self.simulacao.minotauro_vivo:
            mx,my = self.node_coords[self.simulacao.pos_minotauro]
            self.minotauro_obj = self.canvas.create_oval(mx-10, my-10, mx+10, my+10, fill="red", outline="white", width=2)
        else:
            self.minotauro_obj = None
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
        if not self.simulacao_rodando:
            return
        era_vivo = self.simulacao.minotauro_vivo
        mov_p, mov_m = self.simulacao.executar_passo()

        # se minotauro morreu no passo -> notificar
        if era_vivo and not self.simulacao.minotauro_vivo:
            messagebox.showinfo("Evento Raro", "O prisioneiro derrotou o Minotauro em combate!")
            # remove visual do minotauro
            if hasattr(self, "minotauro_obj") and self.minotauro_obj:
                try:
                    self.canvas.delete(self.minotauro_obj)
                except Exception:
                    pass

        self.atualizar_visualizacao(mov_p, mov_m)
        self.atualizar_info()

        if self.simulacao.fim_de_jogo:
            self.simulacao_rodando = False
            self.reset_button.config(state=tk.NORMAL)
            # mostra relatório final em popup
            report = self.simulacao.relatorio_final()
            messagebox.showinfo("Relatório Final", report)
        else:
            delay = int(1050 - self.speed_scale.get())
            self._after_id = self.root.after(delay, self.executar_proximo_passo)

    def atualizar_visualizacao(self, mov_p, mov_m):
        # atualizar posição do prisioneiro
        if mov_p and mov_p[1] in self.node_coords and hasattr(self, "prisioneiro_obj") and self.prisioneiro_obj:
            x,y = self.node_coords[mov_p[1]]
            self.canvas.coords(self.prisioneiro_obj, x-8, y-8, x+8, y+8)
        # atualizar minotauro
        if self.simulacao.minotauro_vivo and mov_m and mov_m[1] in self.node_coords and hasattr(self, "minotauro_obj") and self.minotauro_obj:
            x,y = self.node_coords[mov_m[1]]
            self.canvas.coords(self.minotauro_obj, x-10, y-10, x+10, y+10)

        # traz os agentes à frente
        if hasattr(self, "prisioneiro_obj") and self.prisioneiro_obj:
            self.canvas.tag_raise(self.prisioneiro_obj)
        if hasattr(self, "minotauro_obj") and self.minotauro_obj:
            self.canvas.tag_raise(self.minotauro_obj)

    def atualizar_info(self):
        comida = int(round(self.simulacao.comida_restante))
        status = self.simulacao.status_final
        min_status = f"Minotauro: {self.simulacao.pos_minotauro}" if self.simulacao.minotauro_vivo else "Minotauro: MORTO"
        self.info_var.set(f"Status: {status} | Comida: {comida} | Prisioneiro: {self.simulacao.pos_prisioneiro} | {min_status}")

# -------------------------
# Execução
# -------------------------
def main():
    root = tk.Tk()
    app = LabirintoGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()


