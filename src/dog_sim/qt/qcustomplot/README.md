# QCustomPlot

## Official description

From the [QCustomPlot homepage](http://www.qcustomplot.com/):   

QCustomPlot is a Qt C++ widget for plotting and data visualization. It has no further dependencies and is well documented. This plotting library focuses on making good looking, publication quality 2D plots, graphs and charts, as well as offering high performance for realtime visualization applications. Have a look at the Setting Up and the Basic Plotting tutorials to get started.

QCustomPlot can export to various formats such as vectorized PDF files and rasterized images like PNG, JPG and BMP. QCustomPlot is the solution for displaying of realtime data inside the application as well as producing high quality plots for other media.

## Unofficial catkin package

This catkin package contains the QCustomPlot header and source files, which are downloaded from [here](http://www.qcustomplot.com/index.php/download).

## Update procedure

1. Download and extract the .tar.gz file from [here](http://www.qcustomplot.com/index.php/download).
2. Overwrite the appropriate files in this package. Note that an include and a src folder have been introduced to store qcustomplot.h resp. qcustomplot.cpp.
3. In the qcustomplot.cpp file, change
    ´´´
    #include "qcustomplot.h"
    ´´´
   to
    ´´´
    #include "qcustomplot/qcustomplot.h"
    ´´´
4. Update the version number of the catkin package according to the version number of QCustomPlot.
