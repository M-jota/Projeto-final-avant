# (AVANT) Desafio final Trainee 2025/2 - Hang The Hook 🪝

[ O repositório contém os pacotes com nós realizados pelo Grupo 3 da Eletrônica do Trainee 2025/2. ]

Enunciado:
Elaborar algoritmos ROS para o drone:
* Detectar e seguir a linha azul presente no ambiente simulado até chegar à mangueira vermelha, 
* Identificar o centro da mangueira e alinhar o centro do drone com o centro detectado, 
* Quando o alinhamento for alcançado, o sistema deve publicar uma mensagem em um tópico ROS indicando que a centralização foi concluída com sucesso.
Além disso, fazer:
* Um segundo nó ROS que executa o comando para liberar o relé, acionando a soltura do gancho.

## Aplicações Utilizadas:
* ROS2 (Humble)
* Docker
* Python3 + OpenCV
* Gazebo/Rviz

## Execução:
É necessário criar um container com a imagem abaixo e inserir os pacotes desse repositório (de acordo com os comandos):
```
docker pull joao0607/desafiofinal2025
```

```
docker run -it \
--name desafiofinal2025 \
--privileged \
-e DISPLAY=$DISPLAY \
-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
--net=host \
joao0607/desafiofinal2025
```

```
git clone https://github.com/M-jota/Projeto-final-avant
```
|->> Insira o que foi clonado dentro do container

Para as próximas vezes:
```
docker start desafiofinal2025 
```

```
docker exec -it desafiofinal2025  bash
```
Para executar a simulação:
```
cd ~/ardu_ws/Startup
./start.sh
```
#### OBS.: Com esta versão do repositório, ainda não foi possível executar a simulação

## Esquema demonstrativo da lógica dos tópicos e da relação publisher-subscriber
``` 
           + ------------------+
          |   node_visao      |
          +-------------------+
          |                   |
          |  Publishes:        |
          |  /posicao_mangueira
          |  /deteccao_mangueira
          |  /centralizada_mangueira
          |  /erro_linha_azul
          |  /deteccao_linha_azul
          +---------+---------+
                    |
      +-------------+----------------+
      |                              |
      v                              v
+-------------------+          +-------------------+
|  node_navegacao   |          |   node_rele       |
+-------------------+          +-------------------+
| Subscribes:       |          | Subscribes:       |
| /posicao_mangueira|          | /centralizada_    |
| /deteccao_mangueira|         | mangueira         |
| /centralizada_mangueira|     +-------------------+
| /erro_linha_azul  |
| /deteccao_linha_azul|
+-------------------+
| Publishes:         |
| /cmd_vel /movimento |
+-------------------+
```
## Grupo 3:
[Letícia](https://github.com/Letsts)
  
[Maycon](https://github.com/M-jota)
