#include <stdio.h>
#include <stdlib.h>

typedef struct Grafo{
    int eh_ponderado;
    int nro_vertices;
    int grau_max; //tamanho das listas
    int** arestas; //Matriz de arestas
    float** pesos; //quantidade de elementos em cada lista
    int* grau;
}Grafo;

//criar grafo
Grafo * cria_grafo(int nro_vertices, int grau_max, int eh_ponderado){
    Grafo * GR=(Grafo*) malloc(sizeof(Grafo));
    if(GR != NULL){
        GR->nro_vertices = nro_vertices;
        GR->grau_max = grau_max;
        GR->eh_ponderado = (eh_ponderado != 0) ? 1:0;
        GR->grau = (int *) calloc(nro_vertices, sizeof(int));
        GR->arestas = (int **) malloc(nro_vertices * sizeof(int));
        
        for(int i=0; i<nro_vertices; i++){
            GR->arestas[i] = (int *) malloc(grau_max * sizeof(int));
            if(GR->eh_ponderado){
                GR->pesos = (float **) malloc(nro_vertices * sizeof(float*));
                for(int j=0; j<nro_vertices; i++){
                    GR->pesos[j] = (float*) malloc(grau_max * sizeof(float));
                }
            }
        }
    }
    // entre as linhas 21-24 ocorre a Criação da matriz arestas
    // entre as linhas 25-28 ocorre a Criação da matriz pesos
    return GR;
}
//liberando um grafo 
void libera_Grafo(Grafo* GR){
    if(GR != NULL){
        for(int i=0; i<GR->nro_vertices; i++){
            free(GR->arestas[i]);
            free(GR->arestas);

            if(GR->eh_ponderado){
                for(int j=0; j<GR->nro_vertices; j++){
                    free(GR->pesos[i]);
                    free(GR->arestas);
                }
                free(GR->grau);
                free(GR);
            }
        }
    }
    //entre as linhas 40-42 ocorre a liberação da matriz arestas
    //entre as linhas 44-48 ocorre a liberação da matriz pesos
}
//inserção de Aresta no grafo
int insereAresta(Grafo* GR, int orig, int dest, int eh_digrafo, float peso){
    if(GR == NULL) return 0;
    if(orig < 0 || orig >= GR->nro_vertices) return 0;
    if(dest < 0 || dest >= GR->nro_vertices) return 0;

    GR->arestas[orig] [GR->grau[orig]] = dest;
    if(GR->eh_ponderado)
        GR->pesos[orig] [GR->grau[orig]] = peso;
    GR->grau[orig]++;
    if(eh_digrafo == 0)
        insereAresta(GR,dest,orig,1,peso);
    return 1;
    // entre as linhas 60-61 ocorre a verificação de se a vertice existe
    // entre as linhas 63-66 ocorre a inserção dos valores no final da linha
    // entre as linhas 67-68 ocorre a inserção de outra aresta se NAO for digrafo
}
//função auxiliar: realiza o calculo
void buscaProfundidade(Grafo *GR, int ini, int* visitado, int cont){
    visitado[ini] = cont;
    for(int i=0;i<GR->grau[ini];i++){
        if(!visitado[GR->arestas[ini][i]]){
            buscaProfundidade(GR, GR->arestas[ini][i], visitado, cont+1);
        }
    }
    //essa função marca o vertice como visitado e visita os vizinhos ainda não visitados
}
//funçao principal: faz interação com o usuário
void buscaProfundidade_Grafo(Grafo* GR, int ini, int* visitado){
    int cont = 1;
    for(int i=0; i<GR->nro_vertices;i++){
        visitado[i]=0;
    }
    buscaProfundidade(GR, ini, visitado, cont);
    // aqui a função marca vertice como nao visitados
}
void buscaLargura_Grafo(Grafo* GR, int ini, int* visitado){
    int vert, NV, cont = 1, *fila, IF = 0, FF = 0;
    for(int i = 0;i<GR->nro_vertices;i++)visitado[i] = 0; //marca vertices como não visitados

    NV = GR->nro_vertices;
    fila = (int*) malloc(NV * sizeof(int));
    FF++;
    fila[FF] = ini; visitado[ini] = cont;
    // da linha 97-100 cria a fila.visita e insere ini na fila
    while(IF != FF){
        IF = (IF + 1) % NV;
        vert = fila[IF];
        cont++;
        //da linha 103-104 pega o pprimeiro da fila
        for(int i=0;i<GR->grau[vert];i++){
            if(!visitado[GR->arestas[vert][i]]){
                FF = (FF + 1) % NV;
                fila[FF] = GR->arestas[vert][i];
                visitado[GR->arestas[vert][i]] = cont;
            }
        }        
        //nesse for ele visita so vizinhos ainda não visitados e coloca na fila
        free(fila);
    }
}
int procuraMenorDistancia(float *dist, int *visitado, int NV){
    int menor = -1, primeiro = 1;
    for(int i=0; i < NV; i++){
        if(dist[i] >= 0 && visitado[i] == 0){
            if(primeiro){
                menor = i;
                primeiro = 0;
                }else{
                    if(dist[menor] > dist[i]) menor = i;
                }
            }
        //nessse if ocorre a procura de vertice com menor distaancia e que nao tenha sido visitado
    }
    return menor;
}
void menorCaminho_Grafo(Grafo* GR, int ini, int *ant, float *dist){
    int cont, NV, ind, * visitado, u;
    cont = NV = GR->nro_vertices;
    visitado = (int*) malloc(NV * sizeof(int));
    for(int i=0; i<NV; i++){
        ant[i] = -1;
        dist[i] = -1;
        visitado[i] = 0;
    }
    //cria o vetor auxiliar e inicializa distancias e  anteriores
    dist[ini] = 0;
    while(cont > 0){
        u = procuraMenorDistancia(dist, visitado, NV);
        if(u == -1)break;
        visitado[u] = 1;
        cont--;
        // procura vertice com menor distancia e marca como visitado
        for(int i=0; i<GR->grau[u];i++){
            ind = GR->arestas[u][i];
            if(dist[ind] < 0){
                dist[ind] = dist[u] + 1;
                //aqui ele ver a distancia mas pode ser tbm o peso;
                //dist[ind] = dist[u] + GR->pesos[u][i];
                ant[ind] = u;
            }else{
                if(dist[ind] > dist[u] + 1){
                    //if(dist[ind] > dist[u]+1){ou peso da are}
                    dist[ind] = dist[u] + 1;
                    // ou peso da aresta
                    //dist[ind] = dist[u] + GR->pesos[u][i];
                    ant[ind] = u;
                }
            }
            //nesse for ocorre a atualização da distancia dos vizinhos
        }
    }
    free(visitado);
}

int main(){
    Grafo *GR = cria_grafo(5, 5, 0);
    int eh_diagrafo = 1;
    insereAresta(GR, 0, 1, eh_diagrafo, 0);
    insereAresta(GR, 1, 3, eh_diagrafo, 0);
    insereAresta(GR, 1, 2, eh_diagrafo, 0);
    insereAresta(GR, 2, 4, eh_diagrafo, 0);
    insereAresta(GR, 3, 0, eh_diagrafo, 0);
    insereAresta(GR, 3, 4, eh_diagrafo, 0);
    insereAresta(GR, 4, 1, eh_diagrafo, 0);
    int vis[5], ant[5];
    float dist[5];
    buscaProfundidade_Grafo(GR, 0, vis);
    buscaLargura_Grafo(GR, 0, vis);
    menorCaminho_Grafo(GR, 0, ant, dist);
    libera_Grafo(GR);
    return 0;
}
