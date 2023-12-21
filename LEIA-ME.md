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

4 - LED RGB.

No caso, todos estes dispositivos estão presentes no módulo utilizado, elaborado pelo professor da disciplina.

O objetivo do programa desenvolvido consiste em controlar as cores de um LED RGB a partir de comandos do usuário em um botão, bem como na demonstração dos efeitos de _fade up_ e _fade down_, implementados via hardware.

## Organização do Código

(O código é baseado no exemplo ledc_fade e nos exemplos do GPIO e I2C aplicados em aula);

- São definidos os canais e modos de operação do PWM para controle do LED. Além disso, são definidos os pinos de saída do PWM, dos GPIOs do botão, e das linhas de comunicação I2C do display LCD (SCL e SDA).

- São definidos os parâmetos que podem ser modificados de acordo com a necessidade do projeto, como o número de canais, o duty cicle máximo e o tempo de "fade".

- São definidas variáveis para guardar resultados de operações intermediárias, bem como o valor instantâneo do duty cicle de cada canal do LED RGB (um para cada cor).

- É feita a declaração das funções do "fade".

- Criam-se as estruturas para passagem de parâmetros via fila (queue).

- É criada a flag de interrupção do GPIO.

- Declaram-se as filas.

- A task do GPIO é definida. Nela, recebe-se o número da porta correspondente ao botão acionado, que é enviado da interrupção. Ainda na task, incrementa-se uma variável a cada seleção do botão, que resultará na escolha das cores na _"main"_.

- Na _"main"_, as configurações de GPIO para os botões são inicializadas, a task do GPIO é instalada, a comunicação do display é inicializada e as definições de fade são configuradas.

## Explicação do código

- No loop da _"main"_, são realizados três processos distintos, nomeados como "Fade IN" (incremento do brilho), "Modo Colors" (diferentes cores predefinidas) e "Fade OUT" (decremento do brilho).

### Modo Fade IN
- O primeiro processo consiste no incremento do brilho (_fade in_ ou _fade up_), a partir do zero e, via hardware, de cada uma das cores que compõem o LED RGB. Neste modo, todos os LEDs se encontram apagados, sendo que o primeiro LED a ser incrementado é o vemelho, levando 5000 milissegundos (5 segundos) para realizar o fade IN completo, partindo de 0 a 4000 de duty cycle.
- Após o LED vermelho atingir seu brilho máximo, ele o mantém enquanto o segundo LED, verde, realiza seu incremento. Durante este processo, o LED azul se mantém apagado. Sendo assim, com o incremento do LED verde enquanto o LED vermelho está com o brilho máximo, é possível observar a combinação R-G resultando nas cores laranja, durante cerca da metade do ciclo do LED verde (em 2,5 segundos), e finalizando na cor amarela quando atinge seu brilho máximo (em 5 segundos).
- Por fim, o LED azul é incrementado enquanto os LEDs vermelho e verde mantém seu brilho máximo, de modo que, com a combinação R-G-B, a cor amarela é convertida gradualmente em com branca, que indica que todos os LEDs vermelho, verde e azul se encontram em seu brilho máximo.
- Concomitantmente a este processo, todas as porcentagens de brilho são indicadas no terminal durante o incremento do seu respectivo LED, no formato: Duty cicle instantâneo do LED [num]: [porcentagem do duty cycle]%
- Já no display LCD, cada uma das porcentagens das cores que compõem o RGB, que realizam o incemento, são mostradas concomitantemente, sendo uma por linha, no formato: LED [num]: [porcentagem do duty cycle]%

### Modo COLORS
- O segundo processo consiste em demonstar combinações de cores que podem ser feitas com o LED RGB, com a finalidade de explorar a possibilidade de realizar processos de _fade in/up_ e _fade out/down_ de modo praticamente instantâneo.
- Sendo assim, definimos uma sequência de cores em conformidade com as cores do arco-íris, acrescida da cor branca para finalizar o processo.
- Realizamos a definição de cada um dos valores R-G-B que compõem as cores a partir de uma lógica de switch-case, definindo os valores dos duty cycles em conformidade com o valor que foi predefinido (4000) durante os defines.
- Deste modo, o LED RGB transiciona de uma cor para a outra realizando o incremente/decremento em um intervalo predefinido de 100 milissegundos, o que não é perceptível a olho nu, e pode ser facilmente modificado.
- Foi adicionado um delay

### Modo Fade OUT
- faz-se, primeiramente, o _fade up_ dos três canais ao nível máximo; em seguida, recebe-se da task do botão, via fila, o valor da cor selecionada. Faz-se tanto o _fade up_ quanto o _fade down_ somente para a cor selecionada. No LCD, identifica-se o LED e o percentual do duty cicle correspondente.
