<!-- indicando os procedimentos de instalação e requisitos (versão OS e ROS, pacotes instalados, etc.), informações sobre motivação, utilidade, instalação, execução e exemplo de resultados. (iii) o relatório deve introduzir os conceitos básicos da disciplina que foram utilizados no projeto. As questões propostas podem ser usadas como guia para melhorar a qualidade do texto. -->

# Obstacle Counter 
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-v3.7-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-v22.04.4-red)](https://ubuntu.com/download)
[![Gazebo](https://img.shields.io/badge/Gazebo-vX-orange)](https://gazebosim.org/docs)
## Sobre
Este projeto explora conceitos de percepção na utilização de um sistema robótico para realização de uma tarefa: identificar e contar obstáculos presentes em um ambiente.

## Motivação
A principal motivação deste trabalho é aplicar os conhecimentos teóricos adquiridos nas aulas da disciplina Robôs Autônomos ministadas pelo Prof. Dr. Ricardo Carminati de Mello, na Universidade Federal do Espírito Santo (UFES) para resolução de problemas na área da robótica. Além disso, o projeto aqui desenvolvido poderá ser implementado em um robô real para aprimorar seu processo de percepção, extraindo mais informações do ambiente, como a quantidade de obstáculos presentes. Essa informação poderia ser utilizada para a verificação da quantidade de obstáculos presentes no ambiente, assim como a possibilidade de estimar suas posições no referencial global.

## Métodos e resultados



Para realizar o experimento foi gerado diferentes modelos de mundos no Gazebo. O robô utilizado foi o "burger", mas poderia ser qualquer outro modelo turtlebot3. O LIDAR (Light detection and ranging) presente no turtlebot3 foi utilizado como principal sensor para realizar a extração de características do ambiente. A navegação e localização do robô é feita por meio do pacote [NAV2](https://navigation.ros.org/) disponível para ROS2, sua utilização é moestrada no gif abaixo. Foi utilizado [ROS2 Humble](https://docs.ros.org/en/humble/index.html) para realização deste projeto. 


<p align="center">
<img src= "https://navigation.ros.org/_images/navigation_with_recovery_behaviours.gif" width=750>
</p>

### Percepção
Foi utilizado o sensor a laser para extrair informações do mundo. O conjunto de pontos fornecidos pelo laser é dado em cordenadas polares, esse conjunto foi transformado em uma núvem de pontos em coordenadas cartesianas no referencial do robô. Uma transformação é necessária para transpor a núvem de pontos para um referêncial fixo, que seria o do mapa. Para realizar essa transformação deve-se conhecer a pose do robô, esta foi obtida pela sua odometria. 

### DBSCAN

DBSCAN, abreviação do termo "Density Based Spatial Clustering of Applicationwith Noise" (Clusterização Espacial Baseada em Densidade de Aplicações com Ruído) é um método de clusterização não paramétrico baseado em densidade que é significativamente efetivo para identificar clusters deformato arbitrário e de diferentes tamanhos, identificar e separar os ruídos dos dados e detectar clusters “naturais” e seus arranjos dentro do espaço de dados, sem qualquer
informação preliminar sobre os grupos.

Este método foi aplicado à nuvem de pontos obtida na etapa anterior a fim de encontrar agrupamentos de pontos. Os conjuntos de pontos agrupados são chamados de clusters, e estes são considerados obstáculos. Por meio de cada cluster, é possível extrair seu centróide, que equivale à posição daquele obstáculo no mundo. 

O critério de parada ocorre quando encontra-se o cluster principal, que representa as paredes do ambiente "turtlebot3_world", que um cluster formado maior conjunto de pontos. Caso este seja detectado o programa se encerra, gerando uma imagem dos clusters, assim como um arquivo .txt contendo os obstáculos contabilizados e suas respectivas posições. A imagem mostrando os clusters detectados no ambiente principal é mostrada abaixo.

<p align="center">
<img src= "https://github.com/vinihernech/obstacle-counter/blob/main/obstacle_counter/obstacles.png" width=750>
</p>


 
## Instruções de execução

O projeto pode ser executado localmente, mas recomenda-se a utilização de um container Docker. Caso não tenha o Docker instalado, basta realizar sua instação neste [link](https://docs.docker.com/desktop/install/linux-install/).
Com o Docker baixado e instalado na máquina Linux, deve-se abrir 3 terminais e executar os comandos abaixo nos respectivos terminais.

### 1 terminal:

```xhost + local:docker```

```export DISPLAY=:1```

```sudo docker run --name obstacle_counter -it --net=host --device /dev/dri/ -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority:ro vinihernech/obsctacle_detector:v3```

```cd src/obstacle_counter```

```. ros2_startup.sh```

```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py```


### 2 terminal: 

```xhost + local:docker```

```export DISPLAY=:1```

```sudo docker exec -it obstacle_counter bash```

```cd src/obstacle_counter```

```. ros2_startup.sh```

```ros2 launch nav2_bringup bringup_launch.py map:=/opt/ros2_ws/src/obstacle_counter/maps/map.yaml```


### 3 terminal:

```xhost + local:docker```

```export DISPLAY=:1```

```sudo docker exec -it obstacle_counter bash```

```cd src/obstacle_counter```

```. ros2_startup.sh```

```ros2 launch obstacle_counter obstacle_counter.launch.py```

## Mais informações 
* [ROS2 Humble docs](https://docs.ros.org/en/humble/index.html), Página de documentação do ROS2: Humble;
* [DBSCAN](https://www.maxwell.vrac.puc-rio.br/24787/24787_6.PDF), Descrição do método DBSCAN;
* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/), Manual eletrônico do ROBOTIS para TurtleBot3;
* [Gazebo](https://gazebosim.org/docs), Documentação simulador Gazebo;
