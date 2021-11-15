#include <hFramework.h>
#include <DistanceSensor.h>

using namespace hModules;

int PWR = 500;
int ROT = 2200;
int WALL_DISTANCE = 30;
int FRONT_DISTANCE = 10;

bool right_is_clear, front_is_clear;


void turn_right(int power, int rotation)    //-----Funkcje ruchu pojazdu-----
{
    hMot1.setPower(-power);
    hMot2.setPower(-power);
    sys.delay(2500);
    hMot1.setPower(-power);
    hMot2.setPower(power);
    sys.delay(rotation);
    hMot1.setPower(-power);
    hMot2.setPower(-power);
    sys.delay(2000);
}


void turn_left(int power, int rotation) 
{
    hMot1.setPower(power);
    hMot2.setPower(-power);
    sys.delay(rotation);
}


void forward(int power)
{
    hMot1.setPower(-power);
    hMot2.setPower(-power);
}

void stop()
{
    hMot1.setPower(0);
    hMot2.setPower(0);
}
                        //----------

void hMain()
{
    sys.delay(10000);

    DistanceSensor sens2(hSens2);   //-----Konfiguracja ultradzwiękowych czujników odległości-----
    DistanceSensor sens3(hSens3);
    DistanceSensor sens4(hSens4);
    DistanceSensor sens5(hSens5);   //----------


	while (true)
	{
        int dist2 = sens2.getDistance();    //-----Zczytywanie danych z czujników-----
        int dist3 = sens3.getDistance();
        int dist4 = sens4.getDistance();
        int dist5 = sens5.getDistance();    //----------
		Serial.printf("Dist2:  %d\n Dist3:  %d\nDist4:  %d\nDist5:  %d\n\n\n", dist2, dist3, dist4 ,dist5);


        if (dist5 < WALL_DISTANCE and dist5 > 0)    //-----Prawa strona-----
            right_is_clear = false;
        else if (dist5 < 0)
            stop();
        else
            right_is_clear = true;  //----------


        if (dist3 < FRONT_DISTANCE and dist3 > 0)   //-----Przód-----
            front_is_clear = false;
        else if (dist3 < 0)
            stop();
        else
            front_is_clear = true;  //----------
        

        
        // ----- Logika Programu -----
        if (right_is_clear)
        {
            turn_right(PWR, ROT);
        }
        else if (front_is_clear and not right_is_clear)
            forward(PWR);
        else
            turn_left(PWR, ROT);

	}
}


