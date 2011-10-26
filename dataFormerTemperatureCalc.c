#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dataFormerTemperatureCalc.h"

//Konstanten zur Berechnung
#define ALPHA 3.9083E-03
#define BETA -5.775E-07
#define GAMMA -4.183E-12
#define R0 100
#define EPSILON 1E-06

//Koeffizienten C1 bzw. C2 selbst entwickelt auf Grundlage des Buchs Simulation Sensorschaltungen
#define R25 1010
#define C1 7.8849E-03
#define C2 19.25E-06
#define TN 25

//Linearisierungswiderstand
#define RLIN 5560//2668

void berechnungWidKTY(double *double_widerstand);
/* Umrechnung ADC- Wert in Widerstandswert
 * Eingabe: ADC-Wert
 * Ausgabe: Widerstandswert
 */

void tempBerechnungPT100KTY(double *double_temperatur);
/* Berechnung der Temperatur an hand des Widerstandwertes
 * Eingabe: Widerstandswert
 * Ausgabe: Temperaturwert
 */

double tempBerechnungPT100Neg(double t, double rt);
/* Gleichung zur Berechnung des PT100 bei negativen Temperaturen
 * Eingabe: Temperatur, Widerstand
 * Rueckgabe: Admuxvariable
 * (Rueckgabefunktion wegen Pointer fuer regulaFalsi)
 */

void tempBerechnungPT100Pos(double *temperatur);
/* Temperaturberechnung fuer PT100
 * Eingabe: Widerstandswert
 * Ausgabe: Temperaturwert
 */

int regulaFalsi(double (*function_pointer) (double t, double rt), double xm,double xn, double rt, double *result);
/* Nullstellensuchalgorithmus -> Sekantenverfahren
 * Eingabe: Funktionszeiger mit Rueckgabewert des Ergebnisses, Start- und Endwert des Intervalls
 * Ausgabe: Ergebnis der Nullstellensuche
 */

void hornerSchema (int n, float *x0, float *polyarray);
/* Hornerschema zur Funktionswertberechnung eines Polynoms n-ten Grades
 * Schneller als Berechnung durch direkte Multiplikation der Fktn-Werte
 * Eingabe: n: Grad des Poynoms
 *         x0: Berechnungsstelle
 *        *polyarray: Koeffizienten, 0.-Stelle entspricht Koeff des hoechsten Polynoms
 * Ausgabe: x0: Loesung des Polynoms
 */

void berechnungWidKTY(double *double_widerstand){
    *double_widerstand=RLIN/(4096*20/ *double_widerstand-1);           //Berechnung Widerstand aus ADC Wert, nach Spannungsteilerregel
}

void tempBerechnungPT100KTY(double *double_temperatur){
    //fuer tempBerechnungPT100KTY-122 von 150°C - -55°C nach dem Buch Simulation Sensorschaltungen
    *double_temperatur=(-(C1-2*C2*TN)+sqrt((C1-2*C2*TN)*(C1-2*C2*TN)+4*C2*(*double_temperatur/R25-1+C1*TN-C2*TN*TN)))/(2*C2);
}

void tempBerechnungPT100Pos(double *temperatur){
    //Unterscheidung, ob Widerstand respektive Temperatur >100 Ohm bzw 0°C
    //nach Schrue, S.235 und Vieweg ET Handbuch, S.255 und EN 60751
    double r;
    r=*temperatur;
    if(r >= 100){ //wenn mehr als 0°C
		*temperatur = (-ALPHA+sqrt(ALPHA*ALPHA-4*BETA*(1-r/R0)))/(2*BETA);
	}
	else{ //wenn weniger als 0°C
		regulaFalsi(tempBerechnungPT100Neg, 250,-250,r,temperatur);
		regulaFalsi(tempBerechnungPT100Neg, *temperatur-30,*temperatur+30,r,temperatur);
		regulaFalsi(tempBerechnungPT100Neg, *temperatur+10,*temperatur-10,r,temperatur);
	}
}

double tempBerechnungPT100Neg(double t, double rt){
    double rueckgabe;
    rueckgabe = R0*GAMMA*pow(t,4)-100*R0*GAMMA*pow(t,3)+R0*BETA*pow(t,2)+R0*ALPHA*t+R0-rt;
    return rueckgabe;
}

int regulaFalsi(double (*function_pointer) (double t, double rt), double xm, double xn, double rt, double *result){
    double xnplus1;  											//Hilfsvariable
    if (function_pointer(xm,rt)*function_pointer(xn,rt) < 0){   //Sicherstellen, dass Fktn-werte unterschiedliche Vorzeichen haben (-> Konvergenzpruefung)
        xnplus1 = xm; 											//Zum korrekten Einstieg in die While-Bedingung(Vorzaehlen)
        while (fabs(xn-xnplus1) > EPSILON){   					//solange, bis Abweichung des vorigen zum neuen Berechnungswert sehr klein ist
            xnplus1 = xn-(xn-xm)*function_pointer(xn,rt)/(function_pointer(xn,rt)-function_pointer(xm,rt)); //nach Bronstein, 5. Aufl. 2001, S. 909
            if (function_pointer(xm,rt)*function_pointer(xn,rt) > 0){ //wenn f(xm) und f(xn) gleiche Vorzeichen haben
                xm=xn;
            }
            xn=xnplus1; //xn wird berechneter Wert
        }
        *result=xn;
    }
    else{//sonst Abbruch, da Konvergenz nicht sichergestellt
        printf("regulaFalsi: Konvergenz nicht sicher gestellt.\n");
        return 1;
    }
    return 0;
}

/*
void hornerSchema (int n, float *x0, float *polyarray){
    int i;
    float ergebnis;
    ergebnis = *polyarray;
    for(i=0; i==n-2; i++){
    ergebnis= *(polyarray+(i+1)) + ergebnis * x0;
    }
    *x0=ergebnis;
}*/
