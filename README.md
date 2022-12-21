<!-- indicando os procedimentos de instalação e requisitos (versão OS e ROS, pacotes instalados, etc.), informações sobre motivação, utilidade, instalação, execução e exemplo de resultados. (iii) o relatório deve introduzir os conceitos básicos da disciplina que foram utilizados no projeto. As questões propostas podem ser usadas como guia para melhorar a qualidade do texto. -->

# Obstacle Counter 
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-v3.7-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-v22.04.4-red)](https://ubuntu.com/download)
[![Gazebo](https://img.shields.io/badge/Gazebo-vX-orange)](https://gazebosim.org/docs)

O projeto em questão explora conceitos de percepção na utilização de um sistema robótico para realização de uma tarefa: identificar e contar objetos presentes em um mundo (simulado no Gazebo).

## Motivação
A principal motivação do trabalho é aplicar em problemas práticos os conhecimentos teóricos das aulas da disciplina Robôs Autônomos, ministadas pelo Prof. Dr. Ricardo Carminati de Mello, na Universidade Federal do Espírito Santo (UFES). Além disso, é interessante avaliar os métodos de resolução, visto que podem ser implementadas em vários casos, como: CITAR EXEMPLOS DE UTILIDADE XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX.

## Métodos e resultados

É utilizado o Gazebo para projetar um mundo no qual é simulado um robô com um sensor a laser - para identificação de objetos -, que se movimenta por pontos aleatórios até que XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX, usando o Nav2. 

Com os dados da posição dos objetos retornados pelo sensor já convertidos para o referencial do mundo, é realizado o algoritmo de implementação do DBSCAN, o qual clusteriza os objetos de acordo com a distância euclidiana entre eles. Posteriomente, salva em um arquivo a quantidade de clusters (que são os objetos) identificados, indicando a sua posição.

Como cosiderações foi escolhido que objetos próximos a parede não seriam contabilizados, visto que pode se considerar como extensão dela e objetos muito próximos acabam também sendo contabilizados como um só, já que a área útil entre eles é dificilmente aproveitada.

Esse método, rendeu os seguintes resultados:



## Instruções de execução

Com o Docker baixado e instalado na máquina Linux, abrir 3 terminais e executar os códigos abaixo no respectivo terminal.

###1 terminal:

```xhost + local:docker```

```export DISPLAY=:1```

```sudo docker run --name obstacle_counter -it --net=host --device /dev/dri/ -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority:ro vinihernech/obsctacle_detector:v1```

```cd src/obstacle_counter```

```. ros2_startup.sh```

```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py```


###2 terminal: 

```xhost + local:docker```

```export DISPLAY=:1```

```sudo docker exec -it obstacle_counter bash```

```cd src/obstacle_counter```

```. ros2_startup.sh```

```ros2 launch nav2_bringup bringup_launch.py map:=/opt/ros2_ws/src/obstacle_counter/maps/map.yaml```


###3 terminal:

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