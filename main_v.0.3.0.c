/* Name: main.c
 * Project: USB-Analogdatenwandler
 * Author: Daniel Lehmann
 * Mail: mail@dlehmann.net
 * Creation Date: 2011-04-12
 * Tabsize: 4
 * License: GPL v2
 * Version: v0.3.0
 * Version Date: 20.05.2011
 */
/*USB_DataFormer:
 *Dieses Projekt ermoeglicht das Erfassen von analogen Spannungssignalen
 *uber die USB-Schnittstelle. Als Mikrokontroller wird die ATTiny-Serie
 *von Atmel eingesetzt. Auf diese ist die Ansteuerung ausgelegt.
 *Werden andere Mikroprozessoren genutzt, ist die Software entsprechend anzupassen.
 *Der Nutzer kann ueber Stiftleisten die entpsrechende Konfiguration vor-
 *nehmen. Die Software bietet die Moeglichkeit die Betriebsmodi des AD-
 *Wandler im laufenden Betrieb zu wechseln. So lassen sich die Spg-Referenz
 *(1,1 V, 2,56 V, extern), die entsprechenden Eingaenge (PBx, PBy, PBz)
 *und der Modus (einkanal oder differentiell) waehlen.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */
#include <math.h>

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */

/*Funktionsaufrufe in der Kommandozeile*/
#define PSCMD_ADSTART  	0 //AD-Wandlung starten
#define PSCMD_ADREAD   	1 //AD-Werte auslesen
#define PSCMD_ADSTATUS  2 //Statusregister ADCSRA und ADMUX auslesen
#define PSCMD_ADWRITEADCSRA 3
#define PSCMD_ADWRITEADMUX 4

#define alpha 3.908E-03
#define beta -5.775E-07
#define gamma -4.183E-12
#define R0 100
#define epsilon 1E-06
#define korrekturfaktor 22.527

static void usage(char *name);
/*Ausgabe der zu verwendenden Konsolenaufrufe
 *Uebernommen aus Referenzprojekt PowerSwitch*/

static int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen);
/*Abfrage USB-Datenstring
 *Uebernommen aus Referenzprojekt PowerSwitch*/

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

static int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName);
/*Oeffnen des USB-Geraets
 *Uebernommen aus Referenzprojekt PowerSwitch*/

double ptpolyfourth(double t, double rt);
 /*
 */

int regulafalsi(double xm,double xn, double rt, double *result);
/*
*/

int tempberechnung (double *temperatur);
/*Berechnung der Temperatur eines PT100
 *Eingabe: Widerstandswert
 *Ausgabe: Temperaturwert*/

void hornerschema (int n, float *x0, float *polyarray);
/*Hornerschema zur Funktionswertberechnung eines Polynoms n-ten Grades
 *Schneller als Berechnung durch direkte Multiplikation der Fktn-Werte
 *Eingabe: n: Grad des Poynoms
          x0: Berechnungsstelle
          *polyarray: Koeffizienten, 0.-Stelle entspricht Koeff des hoechsten Polynoms
 *Ausgabe: x0: Loesung des Polynoms*/

int main(int argc, char **argv)
{
    usb_dev_handle      *handle = NULL;
    unsigned char       buffer[8];
    int                 nBytes;

    if(argc < 2){
        usage(argv[0]);
        exit(1);
    }
    usb_init();
    if(usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, "www.fototux.com", USBDEV_SHARED_PRODUCT, "DataFormer") != 0){
        fprintf(stderr, "Could not find USB device \"DataFormer\" with vid=0x%x pid=0x%x\n", USBDEV_SHARED_VENDOR, USBDEV_SHARED_PRODUCT);
        exit(1);
    }
    if(strcmp(argv[1], "adstart") == 0){
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADSTART, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        if(nBytes < 2){
            if(nBytes < 0)
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            exit(1);
        }
        printf("test succeeded\n");
    }
    else if(strcmp(argv[1], "adread") == 0){
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADREAD, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        if(nBytes < 2){
            if(nBytes < 0)
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            fprintf(stderr, "only %d bytes status received\n", nBytes);
            exit(1);
        }
        double U;
        U = buffer[0] | (buffer[1] << 8);
		//U = U * 2.4271845/korrekturfaktor*1.03;
        //tempberechnung(&U);
		printf("%5.3f\n",U);

    }
    else if(strcmp(argv[1], "adstatus") == 0){
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ADSTATUS, 0, 0, (char *)buffer, sizeof(buffer), 5000);
        if(nBytes < 2){
            if(nBytes < 0)
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            fprintf(stderr, "only %d bytes status received\n", nBytes);
            exit(1);
        }
		int statusAdcsraArray[8];
		int statusAdmuxArray[8];
		int j,k;
        int statusAdcsra = buffer[0];
        int statusAdmux = buffer[1];
        printf("ADCSRA: %d; ADMUX: %d\n", statusAdcsra, statusAdmux);
		for (j=7; j >=0; j--){
			if ((buffer[0] & (1 << j)) == 0)
				{statusAdcsraArray[j] = 0;}
			else
				{statusAdcsraArray[j] = 1;}
			if ((buffer[1] & (1 << j)) == 0)
				{statusAdmuxArray[j] = 0;}
			else
				{statusAdmuxArray[j] = 1;}
		}
		printf("ADCSRA Status:\n");
		for (k=7; k >= 0; k--){
			printf("%d ", statusAdcsraArray[k]);
		}
		printf("\n");
			printf("ADMUX Status:\n");
		for (k=7; k >= 0; k--){
			printf("%d ", statusAdmuxArray[k]);
		}
		printf("\n");
    }
    else if(strcmp(argv[1], "adwriteadcsra") == 0){
        if(argc < 1){
            usage(argv[0]);
            exit(1);
        }
        int adcsra = atoi(argv[2]);
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, PSCMD_ADWRITEADCSRA, adcsra, 0, (char *)buffer, sizeof(buffer), 5000);
        if(nBytes < 2){
            if(nBytes < 0)
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            fprintf(stderr, "only %d bytes status received\n", nBytes);
            exit(1);
        }
        int statusAdcsra = buffer[0];
        printf("ADCSRA: %d\n", statusAdcsra);
    }
    else if(strcmp(argv[1], "adwriteadmux") == 0){
        if(argc < 2){
            usage(argv[0]);
            exit(1);
        }
        int admux = atoi(argv[2]);
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, PSCMD_ADWRITEADMUX, admux, 0, (char *)buffer, sizeof(buffer), 5000);
        if(nBytes < 2){
            if(nBytes < 0)
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            fprintf(stderr, "only %d bytes status received\n", nBytes);
            exit(1);
        }
        int statusAdmux = buffer[0];
        printf("ADMUX: %d\n", statusAdmux);
	}
    else if(strcmp(argv[1], "adgetdata") == 0){
        if(argc < 1){
            usage(argv[0]);
            exit(1);
        }
        char buffer2[16];
        nBytes = usb_interrupt_read(handle,USB_ENDPOINT_IN | 1,(char *)buffer2,sizeof(buffer2),1000);
        if(nBytes < 2){
            if(nBytes < 0)
                fprintf(stderr, "USB error: %s\n", usb_strerror());
            fprintf(stderr, "only %d bytes status received\n", nBytes);
            exit(1);
        }
    }
	else{
            nBytes = 0;
            usage(argv[0]);
            exit(1);
	}
    usb_close(handle);
    return 0;
}

static void usage(char *name)
{
    fprintf(stderr, "usage:\n");
    fprintf(stderr, "  %s adstart\n", name);
    fprintf(stderr, "  %s adread\n", name);
    fprintf(stderr, "  %s adstatus\n", name);
    fprintf(stderr, "  %s adgetdata\n", name);
    fprintf(stderr, "  %s adwriteadcsra <ADMUX>\n", name);
    fprintf(stderr, "  %s adwriteadcsra <ADMUX>\n", name);
	fprintf(stderr, "<ADCSRA> <ADMUX> are read as integer\n");
    //fprintf(stderr, "  %s off <port> [<duration>]\n", name);
    //fprintf(stderr, "Ports are single digits in the range 0...7\n");
    //fprintf(stderr, "The pulse duration for switching temporarily is given in seconds.\n");
}

static int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
{
    char    buffer[256];
    int     rval, i;

    if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0)
        return rval;
    if(buffer[1] != USB_DT_STRING)
        return 0;
    if((unsigned char)buffer[0] < rval)
        rval = (unsigned char)buffer[0];
    rval /= 2;
    /* lossy conversion to ISO Latin1 */
    for(i=1;i<rval;i++){
        if(i > buflen)  /* destination buffer overflow */
            break;
        buf[i-1] = buffer[2 * i];
        if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
            buf[i-1] = '?';
    }
    buf[i-1] = 0;
    return i-1;
}

static int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName)
{
    struct usb_bus      *bus;
    struct usb_device   *dev;
    usb_dev_handle      *handle = NULL;
    int                 errorCode = USB_ERROR_NOTFOUND;
    static int          didUsbInit = 0;

    if(!didUsbInit){
        didUsbInit = 1;
        usb_init();
    }
    usb_find_busses();
    usb_find_devices();
    for(bus=usb_get_busses(); bus; bus=bus->next){
        for(dev=bus->devices; dev; dev=dev->next){
            if(dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product){
                char    string[256];
                int     len;
                handle = usb_open(dev); /* we need to open the device in order to query strings */
                if(!handle){
                    errorCode = USB_ERROR_ACCESS;
                    fprintf(stderr, "Warning: cannot open USB device: %s\n", usb_strerror());
                    continue;
                }
                if(vendorName == NULL && productName == NULL){  /* name does not matter */
                    break;
                }
                /* now check whether the names match: */
                len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, 0x0409, string, sizeof(string));
                if(len < 0){
                    errorCode = USB_ERROR_IO;
                    fprintf(stderr, "Warning: cannot query manufacturer for device: %s\n", usb_strerror());
                }
                else{
                    errorCode = USB_ERROR_NOTFOUND;
                    /* fprintf(stderr, "seen device from vendor ->%s<-\n", string); */
                    if(strcmp(string, vendorName) == 0){
                        len = usbGetStringAscii(handle, dev->descriptor.iProduct, 0x0409, string, sizeof(string));
                        if(len < 0){
                            errorCode = USB_ERROR_IO;
                            fprintf(stderr, "Warning: cannot query product for device: %s\n", usb_strerror());
                        }
                        else{
                            errorCode = USB_ERROR_NOTFOUND;
                            /* fprintf(stderr, "seen product ->%s<-\n", string); */
                            if(strcmp(string, productName) == 0)
                                break;
                        }
                    }
                }
                usb_close(handle);
                handle = NULL;
            }
        }
        if(handle)
            break;
    }
    if(handle != NULL){
        errorCode = 0;
        *device = handle;
    }
    return errorCode;
}

int tempberechnung (double *temperatur)
{
    //Unterscheidung, ob Widerstand respektive Temperatur >100 Ohm bzw 0°C
    //nach Schrue, S.235 und Vieweg ET Handbuch, S.255 und EN 60751
    double r;
    r=*temperatur;
    if(r >= 110){
		*temperatur = (-alpha*R0+sqrt(R0*alpha*R0*alpha-4*beta*(R0-r)/R0))/(R0*beta);
	}
	else{
		regulafalsi(250,-250,r,temperatur);
		regulafalsi(*temperatur-30,*temperatur+30,r,temperatur);
		regulafalsi(*temperatur+10,*temperatur-10,r,temperatur);
	}
	return 0;
}

double ptpolyfourth(double t, double rt){
    double rueckgabe;
    rueckgabe = R0*gamma*pow(t,4)-100*R0*gamma*pow(t,3)+R0*beta*pow(t,2)+R0*alpha*t+R0-rt;
    return rueckgabe;
}

//Aufruf Sekantenverfahren fuer PT100 Temp.berechnung
int regulafalsi(double xm,double xn, double rt, double *result){
    double xnplus1;  //Hilfsvariable
    if (ptpolyfourth(xm,rt)*ptpolyfourth(xn,rt) < 0){   //Sicherstellen, dass Fktn-werte unterschiedliche Vorzeichen haben (-> Konvergenzpruefung)
        xnplus1 = xm; //Zum korrekten Einstieg in die While-Bedingung
        while (fabs(xn-xnplus1) > epsilon){    //solange, bis Abweichung des vorigen zum neuen Berechnungswert sehr klein ist
            xnplus1 = xn-(xn-xm)*ptpolyfourth(xn,rt)/(ptpolyfourth(xn,rt)-ptpolyfourth(xm,rt)); //nach Bronstein, 5. Aufl. 2001, S. 909
            if (ptpolyfourth(xm,rt)*ptpolyfourth(xn,rt) > 0){ //wenn f(xm) und f(xn) gleiche Vorzeichen haben
                xm=xn;
            }
            xn=xnplus1;
        }
        *result=xn;
    }
    else{//sonst Abbruch, da Konvergenz nicht sichergestellt
        printf("RegulaFalsi: Konvergenz nicht sicher gestellt.\n");
        return 1;
    }
    return 0;
}

/*
void hornerschema (int n, float *x0, float *polyarray){
    int i;
    float ergebnis;
    ergebnis = *polyarray;
    for(i=0; i==n-2; i++){
    ergebnis= *(polyarray+(i+1)) + ergebnis * x0;
    }
    *x0=ergebnis;
}*/
