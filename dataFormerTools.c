#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */
#include <math.h>
#include "dataFormerTools.h"

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

void usbByteCheck(int numofBytes);

void usage(char *name);
/*Ausgabe der zu verwendenden Konsolenaufrufe
 *Uebernommen aus Referenzprojekt PowerSwitch*/

void argumentCheckMin(int argumentcounter, char **argumentvalue, int minimumarg);

void intToBin(int intIn, int *binArray);

int usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen);

int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName);

void usbByteCheck(int numofBytes)
{
    if(numofBytes < 2){
        if(numofBytes < 0)
            fprintf(stderr, "USB error: %s\n", usb_strerror());
        exit(1);
    }
}

void argumentCheckMin(int argumentcounter, char **argumentvalue, int minimumarg)
{
    if(argumentcounter < minimumarg){
        usage(argumentvalue[0]);
        exit(1);
    }
}

void usage(char *name)
{
    fprintf(stderr, "usage:\n");
    fprintf(stderr, "  %s adstart\n", name);
    fprintf(stderr, "  %s adread\n", name);
    fprintf(stderr, "  %s adstatus\n", name);
    fprintf(stderr, "  %s adwriteadcsra <ADMUX>\n", name);
    fprintf(stderr, "  %s adwriteadcsra <ADMUX>\n", name);
	fprintf(stderr, "<ADCSRA> <ADMUX> are read as integer\n");
	fprintf(stderr, "  %s adchannelselesct <data>\n", name);
	fprintf(stderr, "  %s adaref <data>\n", name);
	fprintf(stderr, "  %s adgetdata\n", name);
	fprintf(stderr, "  %s adconvstart\n", name);
    //fprintf(stderr, "  %s off <port> [<duration>]\n", name);
    //fprintf(stderr, "Ports are single digits in the range 0...7\n");
    //fprintf(stderr, "The pulse duration for switching temporarily is given in seconds.\n");
}

void intToBin(int intIn, int *binArray)
{
    int j;
    for (j=0; j < 8; j++){
        if ((intIn & (1 << j)) == 0){
            binArray[j] = 0;
        }
        else{
            binArray[j] = 1;
        }
    }
}

int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
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

int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName)
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
