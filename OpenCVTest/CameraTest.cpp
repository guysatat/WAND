// CameraTest.cpp : Defines entry point for console application.
#include "stdafx.h"
#include "FlyCapture2.h"
#include <cv.h>
#include <highgui.h>
#include <unistd.h>
#include "colormap.hpp"

#include "arduino-serial-lib.h"

#include <stdio.h>    // Standard input/output definitions 
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 
#include <sys/ioctl.h>
      
    // Disable C4996 warnings
#pragma warning(disable: 4996)
#define SOFTWARE_TRIGGER_CAMERA
      
    using namespace FlyCapture2;
      
    const int col_size = 640;
    const int row_size = 480;
    const int data_size = row_size * col_size;

    double min = 0;
    double max = 0;

    int GDPixThresh = 20;
    int TIME_BUFF_SIZE = 2;
    int GD_BUFF_SIZE = 4;
    int spaceSize = 4;
    double jetThresh = 500.0;

    bool GDBuffRdy = false;
    bool timeBuffRdy = false;
      
    // Forward declarations
    bool CheckSoftwareTriggerPresence( Camera* pCam );
    bool PollForTriggerReady( Camera* pCam );
    bool FireSoftwareTrigger( Camera* pCam );
    void PrintError( Error error );
    void ReleaseImage( IplImage* pimg,
                   IplImage* pimg_bw,
                   CvMemStorage* pstorage );
      
    void GrabImages( Camera* pcam,
                     IplImage* pimg, 
                     int fd, 
                     cv::Mat* GDarray,
                     int index,
                     cv::Mat* muBuffer,
                     cv::Mat* varBuffer);

    // Main Control Loop
    int main(int , char **)
    {
        Camera cam;
        CameraInfo camInfo;
        Property prop;
        Error error;
        BusManager busMgr;
        PGRGuid guid;
        Format7PacketInfo fmt7PacketInfo;
        Format7ImageSettings fmt7ImageSettings;
        CvMemStorage* storage = NULL;
        IplImage* img = NULL;
        IplImage* img_bw = NULL;
        TriggerMode triggerMode;
        char serialport[256];

        cv::Mat GDarray[GD_BUFF_SIZE];
        cv::Mat muBuffer[TIME_BUFF_SIZE];
        cv::Mat varBuffer[TIME_BUFF_SIZE];

        strcpy(serialport,"/dev/ttyACM1");

        int fd = -1;

        fd = serialport_init(serialport, 9600);
      
        // Create OpenCV structs for grayscale image
        img = cvCreateImage( cvSize( col_size, row_size ),
                 IPL_DEPTH_8U,
                 1 );
        img_bw = cvCloneImage( img );
      
        storage = cvCreateMemStorage( 0 );
      
        // Get Flea2 camera
        error = busMgr.GetCameraFromIndex( 0, &guid );
      
        if ( error != PGRERROR_OK )
        {
        PrintError( error );
            return -1;
        }
      
        // Connect to the camera
        error = cam.Connect( &guid );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }

        // Get camera information
        error = cam.GetCameraInfo(&camInfo);
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
      
    #ifndef SOFTWARE_TRIGGER_CAMERA
        // Check for external trigger support
        TriggerModeInfo triggerModeInfo;
        error = cam.GetTriggerModeInfo( &triggerModeInfo ;
      
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
        return -1;
        }
      
        if ( triggerModeInfo.present != true )
        {
            printf( "Camera doesn't support external trigger!\n" );
        return -1;
        }
      
    #endif
        // Get current trigger settings
        error = cam.GetTriggerMode( &triggerMode );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
      
        // Set camera to trigger mode 0
        triggerMode.onOff = true;
        triggerMode.mode = 0;
        triggerMode.parameter = 0;
    #ifdef SOFTWARE_TRIGGER_CAMERA
      
        // A source of 7 means software trigger
        triggerMode.source = 7;
    #else
      
        // Triggering the camera externally using source 0.
        triggerMode.source = 0;
    #endif

    // Set camera triggering mode
    error = cam.SetTriggerMode( &triggerMode );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    // Set property settings (shutter)
    prop.type = SHUTTER;
    prop.onOff = true;
    prop.autoManualMode = false;
    prop.absControl = true;
    // 20ms
    prop.absValue = 20;
    error = cam.SetProperty(&prop);

    if ( error != PGRERROR_OK)
    {
     PrintError( error );
     return -1;
    }
      
    // Power on the camera
    const unsigned int k_cameraPower = 0x610;
    const unsigned int k_powerVal = 0x80000000;
    printf("Powering Camera\n");
    error = cam.WriteRegister( k_cameraPower, k_powerVal );
    
      
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    printf("Powered Camera\n");
    fflush(stdout);
      
    // Poll to ensure camera is ready
    bool retVal = PollForTriggerReady( &cam );
    if( !retVal )
    {
        PrintError( error );
        return -1;
    }
      
    // Set camera configuration: region of 24 x 480 pixels
    // greyscale image mode
    fmt7ImageSettings.width = col_size;
    fmt7ImageSettings.height = row_size;
    fmt7ImageSettings.mode = MODE_0;
    fmt7ImageSettings.offsetX = row_size/2;
    fmt7ImageSettings.offsetY = col_size/2;
    fmt7ImageSettings.pixelFormat = PIXEL_FORMAT_MONO8;
      
    // Validate Format 7 settings
    bool valid;
    error = cam.ValidateFormat7Settings( &fmt7ImageSettings,
                           &valid,
         &fmt7PacketInfo );
    unsigned int num_bytes =
        fmt7PacketInfo.recommendedBytesPerPacket;
      
    // Set Format 7 (partial image mode) settings
    printf("setting format7\n");
    error = cam.SetFormat7Configuration( &fmt7ImageSettings,
                                         num_bytes );
    if ( error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    printf("format7 set\n");
    fflush(stdout);
      
    // Start capturing images
    error = cam.StartCapture();
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

    #ifdef SOFTWARE_TRIGGER_CAMERA
      
    if ( !CheckSoftwareTriggerPresence( &cam ) )
    {
        printf( "SOFT_ASYNC_TRIGGER not implemented on "
                "this camera! Stopping application\n" );
        return -1;
    }
      
    #else
    printf( "Trigger the camera by sending a trigger pulse"
            " to GPIO%d.\n",
            triggerMode.source );
    #endif
      
    error = cam.StartCapture();

    printf("started capture\n");
    fflush(stdout);
      
    // Warm up - necessary to get decent images.
    // See Flea2 Technical Ref.: camera will typically not
    // send first 2 images acquired after power-up
    // It may therefore take several (n) images to get
    // satisfactory image, where n is undefined
    for ( int i = 0; i < 30; i++ )
    {
        // Check that the trigger is ready
        PollForTriggerReady( &cam );
      
        // Fire software trigger
        FireSoftwareTrigger( &cam );
      
        Image im;
      
        // Retrieve image before starting main loop
        Error error = cam.RetrieveBuffer( &im );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
    }
      
    #ifdef SOFTWARE_TRIGGER_CAMERA
      
    if ( !CheckSoftwareTriggerPresence( &cam ) )
    {
        printf( "SOFT_ASYNC_TRIGGER not implemented on this"
                " camera! Stopping application\n");
        return -1;
    }
    #else
      
    printf( "Trigger camera by sending trigger pulse to"
            " GPIO%d.\n",
            triggerMode.source );
    #endif
      
    
    // Grab images acc. to number of hw/sw trigger events
    for ( int i = 0; i < 50000; i++ )
    {
        GrabImages( &cam, img, fd, GDarray, i, muBuffer, varBuffer);
        printf("On frame: %d\n",i);
    }
      
    // Turn trigger mode off.
    triggerMode.onOff = false;
    error = cam.SetTriggerMode( &triggerMode );
    if ( error != PGRERROR_OK )
    {
        PrintError( error );
        return -1;
    }

        printf( "\nFinished grabbing images\n" );
      
        // Stop capturing images error = cam.StopCapture();
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
      
        // Disconnect the camera
        error = cam.Disconnect();
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
      
        ReleaseImage( img, img_bw, storage);
        serialport_close(fd);
        return 0;
    }
      
    // Print error trace
    void PrintError( Error error )
    {
        error.PrintErrorTrace();
    }
      
    // Check for the presence of software trigger
    bool CheckSoftwareTriggerPresence( Camera* pCam )
    {
        const unsigned int k_triggerInq = 0x530;
        Error error;
        unsigned int regVal = 0;
        error = pCam->ReadRegister( k_triggerInq, &regVal );
        if ( error != PGRERROR_OK )
        {
            // TODO
        }
      
        if( ( regVal & 0x10000 ) != 0x10000 )
        {
            return false;
        }
      
        return true;
    }

    // Start polling for trigger ready
    bool PollForTriggerReady( Camera* pCam )
    {
        const unsigned int k_softwareTrigger = 0x62C;
        Error error;
        unsigned int regVal = 0;
      
        do
        {
            error = pCam->ReadRegister( k_softwareTrigger,
                                           &regVal );
      
            if ( error != PGRERROR_OK )
            {
                // TODO
            }
        } while ( (regVal >> 31) != 0 );
      
        return true;
    }
      
    // Launch the software trigger event
    bool FireSoftwareTrigger( Camera* pCam )
    {
        const unsigned int k_softwareTrigger = 0x62C;
        const unsigned int k_fireVal = 0x80000000;
        Error error;
      
        error = pCam->WriteRegister( k_softwareTrigger,
                                        k_fireVal );
      
        if ( error != PGRERROR_OK )
        {
            // TODO
        }
      
        return true;
    }
      
    // Tidy up memory allocated for images etc
    void ReleaseImage( IplImage* pimg,
                       IplImage* pimg_bw,
                       CvMemStorage* pstorage )
    {
        cvReleaseImage( &pimg );
        cvReleaseImage( &pimg_bw );
        cvClearMemStorage( pstorage );
    }

    // Grab camera grayscale image and convert into
    // an OpenCV image
    void GrabImages( Camera* pcam,
                     IplImage* pimg, int fd, 
                     cv::Mat* GDarray, int index,
                     cv::Mat* muBuffer,
                     cv::Mat* varBuffer)
    {
        Image image;

      
        #ifdef SOFTWARE_TRIGGER_CAMERA
      
        // Check that the trigger is ready
        PollForTriggerReady( pcam );
      
        // Fire software trigger
        bool retVal = FireSoftwareTrigger( pcam );
      
        if ( !retVal )
        {
            // TODO.
        }
      
        #endif
      
        // Retrieve image before starting main loop
        Error error = pcam->RetrieveBuffer( &image );

        //if( fd == -1 ) error("serial port not opened");
        serialport_writebyte(fd,(char) 1);
        
        if ( error != PGRERROR_OK )
        {
            PrintError (error);
        }
        

        // Copy FlyCapture2 image into OpenCV struct
        memcpy( pimg->imageData,
        image.GetData(),
        data_size );
      
        // Save the bitmap to file
        //cvSaveImage( "orig.bmp", pimg );
        cv::Mat imgMat(pimg);
        //cv::Mat img;
        

        int rows = imgMat.rows;
        int cols = imgMat.cols;

        // Speckle contrast computations
     

    cv::Mat GDim(rows,cols,CV_64F);

     if (1) { // Use GD processing
     if (index < GD_BUFF_SIZE) {
        imgMat.convertTo(GDarray[index], CV_64F);
     }
     else   { 
        GDBuffRdy = true;
        imgMat.convertTo(GDarray[index % GD_BUFF_SIZE], CV_64F);
     } 
     }
     // Iterate on images
     // pixPoint = imgMat.data;

     if (GDBuffRdy) {
 
     for (int i=0; i < rows; i++)   {
        for (int j=0; j < cols; j++)    {

            min = GDarray[0].at<double>(i,j);
            max = min;

            for (int k=1;k<GD_BUFF_SIZE;k++)  {               
                double pixVal = GDarray[k].at<double>(i,j);
                /*if (pixVal < GDPixThresh)   {
                    pixVal = 0;
                }*/
                if (pixVal < min) {
                    min = pixVal;
                }
                if (pixVal > max)   {
                    max = pixVal;
                }
            }

            GDim.at<double>(i,j) = max - min;
        }
     }  

    }  else    {
        imgMat.convertTo(GDim, CV_64F);
    } 

    
    if (1)  { //use time buffers
    
    if (index < TIME_BUFF_SIZE)  {
        cv::blur(GDim, muBuffer[index], cv::Size(spaceSize, spaceSize));
        cv::blur(GDim.mul(GDim), varBuffer[index], cv::Size(spaceSize, spaceSize));
    }
    else    {
        timeBuffRdy = true;
        cv::blur(GDim, muBuffer[index % TIME_BUFF_SIZE], cv::Size(spaceSize, spaceSize));
        cv::blur(GDim.mul(GDim), varBuffer[index % TIME_BUFF_SIZE], cv::Size(spaceSize, spaceSize));
    }
    }

    cv::Mat mu(rows,cols,CV_64F);
    cv::Mat mu2(rows,cols,CV_64F);

    if (timeBuffRdy)   {
    mu2 = varBuffer[0].clone();
    mu = muBuffer[0].clone();

    for (int i=1;i<TIME_BUFF_SIZE;i++){           
            cv::add(muBuffer[i],mu,mu);
            cv::add(varBuffer[i],mu2,mu2);
    }
    } else {
        cv::blur(GDim,mu, cv::Size(spaceSize, spaceSize));
        cv::blur(GDim.mul(GDim), mu2, cv::Size(spaceSize, spaceSize));
    }
          
    cv::Mat muSq(rows,cols,CV_64F);
    muSq = mu.mul(mu);

    cv::Mat var(rows,cols,CV_64F);
    cv::subtract(mu2.mul(TIME_BUFF_SIZE),muSq,var);


    // TODO: Check for small values of var, then set to large number.

     cv::Mat sig(rows,cols,CV_64F);
     cv::divide(muSq,var,sig);

     // Scale pixel value of output
    double min;
    double max;
    cv::minMaxIdx(sig, &min, &max);
    //fprintf(stdout,"Max val: %f",max);
    cv::Mat adjSig(rows,cols,CV_64F);
    cv::Mat jetSig(rows,cols,CV_64F);
    cv::convertScaleAbs(sig, adjSig, 255 / jetThresh);
    cv::applyColorMap(adjSig,jetSig, 2);
      

        cv::namedWindow("skinperfusiondemo",CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
        cvSetWindowProperty("skinperfusiondemo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        cv::imshow("skinperfusiondemo",jetSig);

        //delete &sig;
        //delete &k;
        //delete &adjSig;
        //sleep();
        cv::waitKey(5);
    } 

int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {
        perror("serialport_init: Unable to open port ");
        return -1;
    }
    
    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

//
int serialport_close( int fd )
{
    return close( fd );
}

//
int serialport_writebyte( int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}