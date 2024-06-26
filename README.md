# robotica_furg
Robô



## Requisitos

- ROS 2 Humble

## Instalação

1. Clone o repositório:

    ```bash
    git clone https://github.com/seu-usuario/nome-do-repositorio.git
    ```

2. Navegue para o diretório do projeto:

    ```bash
    cd furg_ros
    ```

3. Compile o código:

    ```bash
    colcon build
    ```

## Uso

### Executando o nó principal

1. Inicie o ROS 2:

    ```bash
    source /opt/ros/humble/setup.bash
    ```

2. Execute:

    ```bash
     ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
    ```
3. Execute:

    ```bash
    ros2 run control_stage controlador.py
    ```

## Contato

Meu email [seu-email](mailto:tonydanilofroes16@gmail.com)
