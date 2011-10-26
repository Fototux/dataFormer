/* Name: main.c
 * Project: USB-Analogdatenwandler
 * Author: Daniel Lehmann
 * Mail: mail@dlehmann.net
 * Creation Date: 2011-04-12
 * Tabsize: 4
 * License: GPL v2
 * Version: v0.4.0
 * Version Date: 10.06.2011
 */
/*USB_DataFormer:
 *Dieses Projekt ermoeglicht das Erfassen von analogen Spannungssignalen
 *uber die USB-Schnittstelle. Als Mikrokontroller wird die ATTiny-Serie
 *von Atmel eingesetzt. Auf diese ist die Ansteuerung ausgelegt.
 *Werden andere Mikroprozessoren genutzt, ist die Software entsprechend anzupassen.
 *Der Nutzer kann ueber Stiftleisten die entpsrechende Konfiguration vor-
 *nehmen. Die Software bietet die Moeglichkeit die Betriebsmodi des AD-
 *Wandler im laufenden Betrieb zu wechseln. So lassen sich die Spg-Referenz
 *(1,1 V, 2,56 V, VCC), die entsprechenden Eingaenge (PBx, PBy, PBz)
 *und der Modus (einkanal oder differentiell) waehlen.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */
#include <math.h>
#include <time.h>   //Zeitfunktionen (aktuelle Zeit/ Laufzeit ausgeben)
#include "dataFormerTools.h"
#include "dataFormerTemperatureCalc.h"
#include "dataFormerADOperations.h"

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */

/*Funktionsaufrufe in der Kommandozeile*/
#define PSCMD_ADSTART  	0 //AD-Wandlung starten
#define PSCMD_ADREAD   	1 //AD-Werte auslesen
#define PSCMD_ADSTATUS  2 //Statusregister ADCSRA und ADMUX auslesen
#define PSCMD_ADWRITEADCSRA 3 //
#define PSCMD_ADWRITEADMUX 4
#define PSCMD_ADCHANNEL 5
#define PSCMD_ADAREF 6
#define PSCMD_ADGETDATA 7
#define PSCMD_ADCONVSTART 8


#define KORREKTUR 0.95
#define STROM 1.08E-03

#define OFFSET 4//-0.25//-3.5
//Wenn Debugging gewollt, auf 1 oder groesser setzen, sonst auf 0
#define DEBUG 1

#define ADKORR0 16.524606
#define ADKORR1 0.9533445
#define ADKORR2 -0.0000025
#define ADKORR3 0.0000014
#define ADKORR4 -7.149E-09
#define ADKORR5 1.585E-11
#define ADKORR6 -1.65E-14
#define ADKORR7 6.581E-18

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

//Schleifenfunktion zur Ermittlung der AD-Werte
void adschleife(int schleifenanzahl, double *adwert, usb_dev_handle *handle);
/*Eingabe: Anzahl der Schleifendurchlaeufe
  Ausgabe: gemittelter AD-Wert*/

//Einstellen der AD-Referenz (ADMUX-Register)
void setreference(int reference, usb_dev_handle *handle);
/*Eingabe: 2 fuer 1,1 V; 6 fuer 2,5 V*/

//Einstellen der Verstaerkung (ADMUX-Register)
void setadchannel(int channel, usb_dev_handle *handle);
/*Eingabe: 7 fuer 20x und 6 fuer 1x*/

//Ermitteln der eingestellten Verstaerkung und Referenz
int admuxstatus(float *status, usb_dev_handle *handle);
/*Ausgabe: Faktor zur Berechnung Aref/(1024*[20])
  Rueckgabe: Admuxvariable*/

//Einstellen des ADC auf passende Referenz und Verstaerkung
int adreferenzoptimum(float *admuxist, usb_dev_handle *handle);
/*Ausgabe: Faktor zur Berechnung Aref/(1024*[20])*/

int main(int argc, char **argv)
{
    usb_dev_handle      *handle = NULL;
    unsigned char       buffer[8];
    int                 nBytes;
	//int i=0;				//Hilfsvariable
	float	adcfaktor=0;
//	float	offset=0;
    double double_adcw=0;	//Variable fuer Auslesen ADC und Tempberechnung
    time_t          time_zeitangabe;            //Zeitangabe


    argumentCheckMin(argc, argv, 2);
    usb_init();
    if(usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, "www.fototux.com", USBDEV_SHARED_PRODUCT, "DataFormer") != 0){
        fprintf(stderr, "Could not find USB device \"DataFormer\" with vid=0x%x pid=0x%x\n", USBDEV_SHARED_VENDOR, USBDEV_SHARED_PRODUCT);
        exit(1);
    }
    if(strcmp(argv[1], "adstart") == 0){
        //nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, PSCMD_ADSTART, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        //usbByteCheck(nBytes);
		adschleife(256,&double_adcw, handle);
		double_adcw=double_adcw*1.1/1024/20; //4.936/1025/20;
		printf("%4.6f adwert\n",double_adcw);							//Ausgabe
    }
    else if(strcmp(argv[1], "adread") == 0){
		int zx=0;
		//zx=adreferenzoptimum(&adcfaktor, handle);							//Einstellen der optimalen Auflösung des ADC
		printf("%2.8f adcfaktor\n %d referenz\n",adcfaktor,zx);							//Ausgabe
		adschleife(32,&double_adcw, handle);    //Auslesen des ADC mit Mittelwertbildung
        double_adcw += OFFSET;                  //Offsetkorrektur
        double_adcw =double_adcw /4.0;          //Anpassen des Oversample-Werts auf 10-Bit
        double_adcw=ADKORR7*pow(double_adcw,7)+ADKORR6*pow(double_adcw,6)+ADKORR5*pow(double_adcw,5)+ADKORR4*pow(double_adcw,4)+ADKORR3*pow(double_adcw,3)-+ADKORR2*pow(double_adcw,2)+ADKORR1*double_adcw+ADKORR0; //Korrektur INL des ADC
        double_adcw=(double_adcw) * 3.300/ 1024.0 /20.0 *KORREKTUR; //Umrechnung in Spannung; AD-Wert *Referenz/Auflösung/Vertärkung*Korrektur
        #if DEBUG
	        printf("Berechnete Spannung: %3.2f ",double_adcw*1000);
        #endif
        double_adcw=(double_adcw) / STROM;  //Umrechnung in Widerstandswert
        #if DEBUG
	        printf("Berechneter Widerstand: %3.2f ",double_adcw);
	    #endif
        tempBerechnungPT100Pos(&double_adcw);   //Berechnung der Temperatur anhand des Wiederstands
	    printf("Temperatur: %3.2f\n",double_adcw);
	    printf("%s", ctime(&time_zeitangabe));
		/*if (zx<2){	//Festlegen der Offsetkorrektur fuer differentielle Messung
			if (zx==0){
				offset=0.3;
			}
			else{	//zx==1
				offset=0;
			}
		}*/
    }
    else if(strcmp(argv[1], "adstatus") == 0){
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADSTATUS, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
		int statusAdcsraArray[8];				//Binaerarray fuer ADCSRA Register
		int statusAdmuxArray[8];				//Binaerarray fuer ADMUX Register
		int k;									//Hilfsvariable
        intToBin(buffer[0], statusAdcsraArray);	//Wandlung der Integer in Binaer
        intToBin(buffer[1], statusAdmuxArray);	//Wandlung der Integer in Binaer
		printf("ADCSRA Status:\n");				//Ausgabe ADSCRA
		for (k=7; k >= 0; k--){					//Spaltenweises Ausgeben des Arrays
			printf("%d ", statusAdcsraArray[k]);
		}
		printf("\n");
		printf("ADMUX Status:\n");				//Ausgabe ADMUX
		for (k=7; k >= 0; k--){					//Spaltenweises Ausgeben des Arrays
			printf("%d ", statusAdmuxArray[k]);
		}
		printf("\n");
    }
    else if(strcmp(argv[1], "adwriteadcsra") == 0){
        argumentCheckMin(argc, argv, 3);
        int adcsra = atoi(argv[2]);
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADWRITEADCSRA, adcsra, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
        int statusAdcsra = buffer[0];
        printf("ADCSRA: %d\n", statusAdcsra);
    }
    else if(strcmp(argv[1], "adwriteadmux") == 0){
        argumentCheckMin(argc, argv, 3);
        int admux = atoi(argv[2]);
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADWRITEADMUX, admux, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
        int statusAdmux = buffer[0];
        printf("ADMUX: %d\n", statusAdmux);
	}
    else if(strcmp(argv[1], "adchannelselect") == 0){
        argumentCheckMin(argc, argv, 3);
        int adcs = atoi(argv[2]);
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADCHANNEL, adcs, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
        adcs = buffer[0];
        printf("ADMUX: %d\n", adcs);
	}
    else if(strcmp(argv[1], "adaref") == 0){
        argumentCheckMin(argc, argv, 3);
        int adaref = atoi(argv[2]);
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADAREF, adaref, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
        adaref = buffer[0];
        printf("ADMUX: %d\n", adaref);
	}
    else if(strcmp(argv[1], "adgetdata") == 0){
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADGETDATA, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
        float U;
        U = buffer[0] | (buffer[1] << 8);
        printf("%f\n", U);
    }
    else if(strcmp(argv[1], "adconvstart") == 0){
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADCONVSTART, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        usbByteCheck(nBytes);
    }
	else{
            nBytes = 0;
            usage(argv[0]);
            exit(1);
	}
    usb_close(handle);
    return 0;
}

void adschleife(int schleifenanzahl, double *adwert, usb_dev_handle *handle){
	int i=0;										//Zaehlvariable
	int	nBytes;
    unsigned char	buffer[8];

	for(i=schleifenanzahl; i>0; i--){				//256 Werte auslesen und mitteln
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADREAD, 0, 0, (char *)buffer, sizeof(buffer), 5000);
		usbByteCheck(nBytes);						//Ueberpruefung, ob Uebertragung korrekt
		*adwert += buffer[0] | (buffer[1] << 8);	//Werte aufaddieren, Buffer wird als High- und Lowbyte gesendet und hier zusammengefügt
	}
	*adwert=*adwert/schleifenanzahl;				//Teilen durch Anzahl der Durchlaeufe
}

void setreference(int reference, usb_dev_handle *handle){
	int	nBytes;
    unsigned char	buffer[8];
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADAREF, reference, 0, (char *)buffer, sizeof(buffer), 5000);
	usbByteCheck(nBytes);
}

void setadchannel(int channel, usb_dev_handle *handle){
	int	nBytes;
    unsigned char	buffer[8];
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADCHANNEL, channel, 0, (char *)buffer, sizeof(buffer), 5000);
	usbByteCheck(nBytes);
}

int admuxstatus(float *status, usb_dev_handle *handle){						//Rueckgabe, welche Referenz genutzt wird und welche Verstaerkung eingestellt ist
	int	statusAdmuxArray[8];						//zum Speichern der Binaerwerte
	int k; 											//Hilfsvariable
	int	nBytes;
    unsigned char	buffer[8];
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADSTATUS, 0, 0, (char *)buffer, sizeof(buffer), 5000);
    usbByteCheck(nBytes);							//ADMUX auslesen und Rueckgabe pruefen
    intToBin(buffer[1], statusAdmuxArray);	//Integerwert wandeln in Binaer
	//Adref2 ist bei 2,56 V (1) und 1,1 V (0)
	//Mux0 ist bei 20x (1) und bei 1x (0), im folgenden invertiert (~); 20x (0) und 1x (1)
	//Zusammenfuegen zu Zahl zwischen 0 und 3 (Refs2 ist "Highbyte")
	//0 enstpricht maximaler Empfindlichkeit, 3 der Minimalen
	// Amux	Adref	k	Bedeutung
	//	0		0	0	20x, 1,1 V
	//	0		1	1	20x, 2,56 V
	//	1		0	2	1x, 1,1 V
	//	1		0	3	1x, 2,56 V
	k= (statusAdmuxArray[4]) | ((statusAdmuxArray[0] ^ 1) << 1);
	printf("%4d k\n",statusAdmuxArray[4]);							//Ausgabe
	printf("%4d k\n",statusAdmuxArray[0]);							//Ausgabe
	printf("%4d k\n",((statusAdmuxArray[0]) ^ 1));							//Ausgabe
	printf("%4d k\n",k);							//Ausgabe
	if (k==0){
		*status=1.1*1.1/(1024.0*20);
	}
	else if (k==1){
		*status=0.985*2.56/(1024.0*20);
	}
	else if (k==2){
		*status=1.1/1024.0;
	}
	else{
		*status=2.56/1024.0;
	}
	return k;
}

int adreferenzoptimum(float *admuxist, usb_dev_handle *handle){
	float admuxstat=0;
	double wert=0;
	int	admuxfaktor=0;
	int optimum=0;
	//welche AdReferenz
	admuxfaktor=admuxstatus(&admuxstat, handle);
	//viermaliges Auslesen um Wertebereich zu ermitteln
	adschleife(4,&wert, handle);
	//Abfragen, welche Referenz im Verhaeltnis zum AD-Wert und entsprechende Einstellung der Parameter
	//
	while (optimum!=1){
		if ((admuxfaktor==0 &&(wert <= 900)) | (admuxfaktor==1 && (wert <= 400)) | (admuxfaktor==2 && (wert <= 45)) | (admuxfaktor==3 && (wert <= 18))){
			//und Aufloesung 2,56 V / 20x, dann halbieren der Referenz
			setreference(2, handle);	// 1.1 V
			setadchannel(7, handle);	// 20x
			*admuxist=1.1*1.1/(1024.0*20);	//=0
			admuxfaktor=0;
			optimum=1;
		}
		else if ((admuxfaktor==1 && ((400 < wert) && (wert <= 920))) | (admuxfaktor==2 && ((45 < wert) && (wert <= 105) )) | (admuxfaktor==3 && ((18 < wert) && (wert <= 45)))){
			//Aufloesung und Referenz aendern von 1,1 V / 1x
			setreference(6, handle);	//auf 2,56 V
			setadchannel(7, handle);	//auf 20x
			*admuxist=0.985*2.56/(1024.0*20);	//=1
			admuxfaktor=1;
			optimum=1;
		}
		else if ((admuxfaktor==2 && ((105 < wert) && (wert <= 920))) | (admuxfaktor==3 && ((45 < wert) && (wert <= 400)))){
			//Aufloesung und Referenz aendern von 2,56 V / 1x auf
			setreference(2, handle);	//auf 1,1 V
			setadchannel(6, handle);	//auf 1x
			*admuxist=1.1/1024.0;		//=2
			admuxfaktor=2;
			optimum=1;
		}
		else if ((admuxfaktor==0 && (wert > 900 )) | (admuxfaktor==1 && (wert > 920)) | (admuxfaktor==2 && (wert > 920 ))){
			setreference(6, handle);//auf 2,56 V
			setadchannel(6, handle);//auf 1x
			admuxfaktor=3;
			optimum=0;
			adschleife(4,&wert, handle);
		}
		else{ //sonst
			setreference(6, handle);//auf 2,56 V
			setadchannel(6, handle);//auf 1x
			*admuxist=2.56/1024.0;	//=3
			admuxfaktor=3;
			optimum=1;
		}
	}
	return admuxfaktor;
}
