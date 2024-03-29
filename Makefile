# Name: Makefile
# Project: dataFormer
# Author: Christian Starkjohann
# Creation Date: 2005-01-16
# Tabsize: 4
# Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
# This Revision: $Id: Makefile 276 2007-03-20 10:53:04Z cs $


# Concigure the following definitions according to your system. The dataFormer
# tool has been successfully compiled on Mac OS X, Linux and Windows.

# Use the following 3 lines on Unix (uncomment the framework on Mac OS X):
USBFLAGS = `libusb-config --cflags`
USBLIBS = `libusb-config --libs` #-framework CoreFoundation
EXE_SUFFIX =

# Use the following 3 lines on Windows and comment out the 3 above. You may
# have to change the include paths to where you installed libusb-win32
#USBFLAGS = -I/usr/local/include
#USBLIBS = -L/usr/local/lib -lusb
#EXE_SUFFIX = .exe	


CC		= gcc
CFLAGS	= $(USBFLAGS) -O -Wall -lm
LIBS	= $(USBLIBS)

PROGRAM = dataFormer$(EXE_SUFFIX)


all: $(PROGRAM)

.c.o:
	$(CC) $(CFLAGS) dataFormerTools.h dataFormerTools.c -c $<

$(PROGRAM): main.o
	$(CC) -o $(PROGRAM) main.c dataFormerTools.o dataFormerTools.h -lm $(LIBS)

strip: $(PROGRAM)
	strip $(PROGRAM)

clean:
	rm -f *.o $(PROGRAM)
