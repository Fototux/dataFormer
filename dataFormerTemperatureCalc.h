#ifndef DATAFORMERTEMPERATURECALC_INCLUDED
#define DATAFORMERTEMPPERATURECALC_INCLUDED

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

#endif // DATAFORMERTEMPRECHNUNG_INCLUDED
