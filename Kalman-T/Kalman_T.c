#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#define Q 1e-6
#define R 1e-1


unsigned int myRandom(int start, int end)
{
	return start+(rand()%(end - start + 1));
}

float GetPrediction(float kalman)
{
    /*
    X(k|k-1)=A X(k-1|k-1)+B U(k) ……….. (1)
    假定温度恒定不变 --> X(k|k-1)=X(k-1|k-1)
    */
    return kalman;
}
float GetPredictionCovariance(float covariance )
{
    /*
    P(k|k-1)=A P(k-1|k-1) A’+Q ……… (2)
    --> P(k|k-1)=P(k-1|k-1) +Q
    */
    return (float)(covariance + Q)*1.0F;
}

float GetKalman(float prediction, float Kg, float measure)
{
    /*
    X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ……… (3)
    --> X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1))
    */
    return prediction + Kg*(measure - prediction);
}

float GetKg(float PredictionCovariance)
{
    /*
    Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R) ……… (4)
    --> Kg(k)= P(k|k-1) / (P(k|k-1) + R)
    */
    return (float)(PredictionCovariance/(PredictionCovariance + R))*1.0F;
}
float GetCovariance(float Kg, float PredictionCovariance)
{
    /*
    P(k|k)=（I-Kg(k) H）P(k|k-1) ……… (5)
    --> P(k|k)=（1-Kg(k)）P(k|k-1)
    */
    return (1-Kg)*PredictionCovariance;
}
int main(void)
{
    int i = 0;
    int N = 200;
    float Kg = 0.0F;
    float kalman = 25.0F;
    float measure = 0.0F;
    float covariance = 5.0F;
    float prediction = 0.0F;
    float PredictionCovariance = 0.0F;

    for(i = 0; i < N; i++){
        prediction = GetPrediction(kalman);
        PredictionCovariance = GetPredictionCovariance(covariance);

        srand((unsigned)time(NULL)+i);
        measure = myRandom(18, 32);
        kalman = GetKalman(prediction, Kg, measure);
        Kg = GetKg(PredictionCovariance);
        covariance = GetCovariance(Kg, PredictionCovariance);
        printf("<N=%d>\t measure:%.2f\t Kg:%.2f\t covariance:%.2f\t kalman:%.2f\r\n",
            i, measure, Kg, covariance, kalman);
    }
    return 0;
}
