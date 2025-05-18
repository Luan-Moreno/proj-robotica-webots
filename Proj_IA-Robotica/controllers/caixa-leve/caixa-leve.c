#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>
#include <stdlib.h>
#include <time.h>

#define TIME_STEP 64
#define QtdCaixas 20
#define qtdSensores 8

int main()
{
    double leituraSensores[qtdSensores];
    
    double velEsquerda = 1;
    double velDireita = 1;

    bool caixaMovida = false;
    WbNodeRef caixas[QtdCaixas];
    char identificadorCaixa[20] = {0};
  
    int tempoImovel = 0;
    int direcaoAtual = 0;
    int tentativas = 0;
    
    int faseDestravamento = 0;
    int ladoTentativa = 0;

    double posXAnterior = 0.0;
    double posZAnterior = 0.0;

    wb_robot_init();

    WbDeviceTag motorEsquerda = wb_robot_get_device("left wheel motor");
    WbDeviceTag motorDireita = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(motorEsquerda, INFINITY);
    wb_motor_set_position(motorDireita, INFINITY);
    wb_motor_set_velocity(motorEsquerda, 0);
    wb_motor_set_velocity(motorDireita, 0);

    WbDeviceTag sensoresProximidade[qtdSensores];
    char nomeSensor[4];
    for (int i = 0; i < qtdSensores; i++)
    {
        sprintf(nomeSensor, "ps%d", i);
        sensoresProximidade[i] = wb_robot_get_device(nomeSensor);
        wb_distance_sensor_enable(sensoresProximidade[i], TIME_STEP);
    }

    for (int i = 0; i < QtdCaixas; i++)
    {
        sprintf(identificadorCaixa, "CAIXA%02d", i + 1);
        caixas[i] = wb_supervisor_node_get_from_def(identificadorCaixa);

        if (caixas[i] != NULL)
            printf("Localização da %s salva\n", identificadorCaixa);
        else
            printf("Erro ao acessar %s\n", identificadorCaixa);
    }

    const double *posicaoInicial = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
    posXAnterior = posicaoInicial[0];
    posZAnterior = posicaoInicial[2];

    while (wb_robot_step(TIME_STEP) != -1)
    {
        double posicoesAntes[QtdCaixas][3];

        for (int i = 0; i < qtdSensores; i++)
            leituraSensores[i] = wb_distance_sensor_get_value(sensoresProximidade[i]) - 60;

        for (int i = 0; i < QtdCaixas; i++)
        {
            const double *pos = wb_supervisor_node_get_position(caixas[i]);
            posicoesAntes[i][0] = pos[0];
            posicoesAntes[i][1] = pos[1];
            posicoesAntes[i][2] = pos[2];
        }

        const double *posicaoAtual = wb_supervisor_node_get_position(wb_supervisor_node_get_self());

        if (!caixaMovida &&
            fabs(posicaoAtual[0] - posXAnterior) < 0.001 &&
            fabs(posicaoAtual[2] - posZAnterior) < 0.001)
            tempoImovel += TIME_STEP;
        else
        {
            tempoImovel = 0;
            tentativas = 0;
        }

        posXAnterior = posicaoAtual[0];
        posZAnterior = posicaoAtual[2];

        if (tempoImovel >= 1100 && !caixaMovida)
        {
            tentativas++;
            printf("Robô travado! Ajustando... (tentativa %d de 3).\n", tentativas);

            if (tentativas >= 3)
            {
                direcaoAtual = (direcaoAtual + 1) % 4;
                tentativas = 0;
            }

            switch (direcaoAtual)
            {
                case 0:
                    velDireita = -0.4;
                    velEsquerda = 1;
                    break;
                case 1:
                    velDireita = 1;
                    velEsquerda = -0.4;
                    break;
                case 2:
                    velDireita = 0.4;
                    velEsquerda = 1;
                    break;
                case 3:
                    velDireita = 1;
                    velEsquerda = 0.4;
                    break;
            }

            tempoImovel = 0;
        }
        else if (caixaMovida)
        {
            velDireita = 1;
            velEsquerda = -1;
        }
        else if ((leituraSensores[7] > 1000 || leituraSensores[0] > 1000 || leituraSensores[1] > 1000) &&
                 (leituraSensores[2] > 1000 || leituraSensores[3] > 1000))
        {
            velDireita = 1;
            velEsquerda = -1;
        }
        else if ((leituraSensores[7] > 1000 || leituraSensores[0] > 1000 || leituraSensores[6] > 1000) &&
                 (leituraSensores[4] > 1000 || leituraSensores[5] > 1000))
        {
            velDireita = -1;
            velEsquerda = 1;
        }
        else if (leituraSensores[7] > 1000 || leituraSensores[0] > 1000)
        {
            velDireita = -1;
            velEsquerda = 1;
        }
        else
        {
            velDireita = 1;
            velEsquerda = 1;
        }

        if (velDireita < 0 && velEsquerda < 0)
        {
            if (direcaoAtual % 2 == 0)
            {
                velDireita = 0.15;
                velEsquerda = 1;
            }
            else
            {
                velDireita = 1;
                velEsquerda = 0.15;
            }
        }

        wb_motor_set_velocity(motorEsquerda, 6.28 * velEsquerda);
        wb_motor_set_velocity(motorDireita, 6.28 * velDireita);

        wb_robot_step(TIME_STEP);

        for (int i = 0; i < QtdCaixas; i++)
        {
            const double *posicaoAtualCaixa = wb_supervisor_node_get_position(caixas[i]);

            if (fabs(posicoesAntes[i][0] - posicaoAtualCaixa[0]) > 0.001 ||
                fabs(posicoesAntes[i][1] - posicaoAtualCaixa[1]) > 0.001 ||
                fabs(posicoesAntes[i][2] - posicaoAtualCaixa[2]) > 0.001)
            {
                if (!caixaMovida)
                    printf("Robô encontrou a caixa leve!!\n");

                caixaMovida = true;
                break;
            }
        }
    }

    wb_robot_cleanup();

    return 0;
}
