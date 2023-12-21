| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# _LEIA-ME_

(Este arquivo de texto tem o objetivo de descrever a organização do repositório e apresentar o funcionamento do trabalho final).

1 - As atividades desenvolvidas em laboratório ao longo do semestre se encontram organizadas no arquivo "main" da pasta "final final final/tcp".

2 - O trabalho final, que constitui a última atividade avaliativa da disciplina, se encontra no arquivo "ledc_fade_example_main", da pasta main externa.

## Como utilizar o Trabalho

Para que o código seja compilado e executado com sucesso, necessita-se a disponibilidade dos seguintes componentes:

1 - ESP 32.

2 - Display LCD i2c.

3 - Botão normalmente aberto.

4 - Led RGB.

No caso, todos estes dispositivos estão presentes no módulo utilizado, elaborado pelo professor da disciplina.

O objetivo do programa desenvolvido consiste em controlar as cores de um LED RGB a partir de comandos do usuário em um botão, bem como na demonstração dos efeitos de _fade up_ e _fade down_, implementados via hardware.

## Organização do Código

(O código é baseado no exemplo ledc_fade e nos exemplos do GPIO e I2C aplicados em aula);

- São definidos os canais e modos de operação do PWM para controle do LED. Além disso, são definidos os pinos de saída do PWM, dos GPIOs do botão, e das linhas de comunicação I2C do display LCD (SCL e SDA).

- São definidas variáveis para guardar resultados de operações intermediárias, bem como o valor instantâneo do duty cicle de cada canal do LED RGB (um para cada cor).

- É feita a declaração das funções do fade.

- Criam-se as estruturas para passagem de parâmetros via fila (queue).

- É criada a flag de interrupção do GPIO.

- Declaram-se as filas.

- A task do GPIO é definida. Nela, recebe-se o número da porta correspondente ao botão acionado, que é enviado da interrupção. Ainda na task, incrementa-se uma variável a cada seleção do botão, que resultará na escolha das cores na _"main"_.

- Na _"main"_, as configurações de GPIO para os botões são inicializadas, a task do GPIO é instalada, a comunicação do display é inicializada e as definições de fade são configuradas.

- No loop da _"main"_, faz-se, primeiramente, o _fade up_ dos três canais ao nível máximo; em seguida, recebe-se da task do botão, via fila, o valor da cor selecionada. Faz-se tanto o _fade up_ quanto o _fade down_ somente para a cor selecionada. No LCD, identifica-se o LED e o percentual do duty cicle correspondente.