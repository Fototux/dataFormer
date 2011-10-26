#ifndef DATAFORMERTOOLS_INCLUDED
#define DATAFORMERTOOLS_INCLUDED

#include <usb.h>    //noetig, da sonst Kompilerfehler wegen unbekannter Sturktur usb_dev_handle

void usbByteCheck(int numofBytes);

void usage(char *name);

void argumentCheckMin(int argumentcounter, char **argumentvalue, int minimumarg);

void intToBin(int intIn, int *binArray);

int usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen);

int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName);

#endif // DATAFORMERTOOLS_INCLUDED
